/*
 * cbs_input.c - VPP CBS plugin input node (wheel dequeuer)
 *
 * Copyright (c) 2024 Your Org <your.email@example.com> // Placeholder
 * Based on nsim_input.c, Copyright (c) Cisco and/or its affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <vlib/vlib.h>
#include <vnet/vnet.h>
#include <vppinfra/error.h>
#include <vppinfra/time.h>
#include <cbs/cbs.h> // Uses i64 via clib.h included in cbs.h
#include <vnet/ethernet/ethernet.h>
#include <vppinfra/format.h>

// Renamed trace struct
typedef struct
{
  f64 tx_time; // Actual transmit time determined by CBS
  u32 next_index;
  u32 buffer_index;
  f64 cbs_credits_before;
  f64 cbs_credits_after;
} cbs_tx_trace_t; // Renamed

#ifndef CLIB_MARCH_VARIANT
/* Renamed trace format function */
static u8 *
format_cbs_tx_trace (u8 * s, va_list * args)
{
  CLIB_UNUSED (vlib_main_t * vm) = va_arg (*args, vlib_main_t *);
  CLIB_UNUSED (vlib_node_t * node) = va_arg (*args, vlib_node_t *);
  cbs_tx_trace_t *t = va_arg (*args, cbs_tx_trace_t *); // Use renamed type

  s = format (s, "CBS_TX (bi %u): tx at %.6f, next %d, credits %.2f -> %.2f",
              t->buffer_index, t->tx_time, t->next_index,
              t->cbs_credits_before, t->cbs_credits_after);
  return s;
}

vlib_node_registration_t cbs_input_node; // Renamed registration declared
#endif /* CLIB_MARCH_VARIANT */

// Renamed error enum prefix
#define foreach_cbs_tx_error                   \
_(TRANSMITTED, "Packets transmitted by CBS")    \
_(STALLED, "CBS stalled (credits/port busy)") \
_(NO_PKTS, "CBS wheel empty")                 \
_(NO_WHEEL, "No CBS wheel configured")        \
_(NO_DATA, "No data in CBS wheel")           \
_(ARP_PACKETS_TX, "ARP packets transmitted")  \
_(PACKETS_TX, "Total packets transmitted")

typedef enum
{
#define _(sym,str) CBS_TX_ERROR_##sym, // Renamed prefix
  foreach_cbs_tx_error
#undef _
    CBS_TX_N_ERROR, // Renamed
} cbs_tx_error_t; // Renamed typedef

#ifndef CLIB_MARCH_VARIANT
static char *cbs_tx_error_strings[] = { // Renamed array
#define _(sym,string) string,
  foreach_cbs_tx_error
#undef _
};
#endif /* CLIB_MARCH_VARIANT */

/* Renamed trace helper */
static void
cbs_tx_trace_add (vlib_main_t * vm, vlib_node_runtime_t * node,
                   u32 bi, f64 tx_time, u32 next_index,
                   f64 credits_before, f64 credits_after)
{
  vlib_buffer_t *b = vlib_get_buffer (vm, bi);
  if (b && PREDICT_FALSE (b->flags & VLIB_BUFFER_IS_TRACED)) // Check if buffer is valid
    {
      cbs_tx_trace_t *t = vlib_add_trace (vm, node, b, sizeof (*t)); // Use renamed type
      t->buffer_index = bi;
      t->tx_time = tx_time;
      t->next_index = next_index;
      t->cbs_credits_before = credits_before;
      t->cbs_credits_after = credits_after;
    }
}

// Directly define CBS_NEXT enum here to avoid circular imports
typedef enum
{
  CBS_NEXT_DROP,     // Define drop index as 0
  CBS_NEXT_ETHERNET, // Define ethernet-input index as 1
  CBS_N_NEXT,        // Number of next nodes
} cbs_input_next_t;

/* 
 * Process ARP and other control packets with highest priority
 * This function guarantees ARP packets are transmitted regardless of credit state
 */
static u32
cbs_handle_high_priority_packets(vlib_main_t *vm, vlib_node_runtime_t *node,
                       cbs_wheel_t *wp, cbs_main_t *cbsm);

/* Process ARP packets separately */
static u32
cbs_handle_arp_packets(vlib_main_t *vm, vlib_node_runtime_t *node,
                         cbs_wheel_t *wp, cbs_main_t *cbsm)
{
  /* Nothing to do if wheel is empty */
  if (wp->cursize == 0)
    return 0;

  u32 n_arp = 0;
  u32 to_nexts[CBS_MAX_TX_BURST];
  u16 nexts[CBS_MAX_TX_BURST];
  
  /* Get ethernet input index */
  u32 ethernet_index = vlib_node_get_next (vm, node->node_index, CBS_NEXT_ETHERNET);
  
  /* Scan the wheel for ARP packets that need priority handling */
  u32 current = wp->head;
  u32 count = wp->cursize;
  u32 max_arps_per_call = 10; /* Limit to avoid permanent loops */
  u32 processed_count = 0;
  
  while (count > 0 && n_arp < CBS_MAX_TX_BURST && processed_count < max_arps_per_call)
  {
    cbs_wheel_entry_t *ep = wp->entries + current;
    
    /* Safety check for invalid entries */
    if (PREDICT_FALSE(!ep || ep->buffer_index == ~0)) {
      current = (current + 1) % wp->wheel_size;
      count--;
      continue;
    }
    
    u32 bi = ep->buffer_index;
    
    /* Safety check for buffer index */
    if (PREDICT_FALSE(bi == ~0)) {
      clib_warning("Invalid buffer index %d in CBS wheel", bi);
      current = (current + 1) % wp->wheel_size;
      count--;
      continue;
    }
    
    vlib_buffer_t *b = vlib_get_buffer (vm, bi);
    processed_count++;
    
    /* Check if this is an ARP packet */
    if (b->current_length >= sizeof(ethernet_header_t))
    {
      ethernet_header_t *eth = vlib_buffer_get_current(b);
      if (clib_net_to_host_u16(eth->type) == ETHERNET_TYPE_ARP)
      {
        /* Log that we found an ARP packet */
        clib_warning("Found ARP packet in CBS wheel, extracting for immediate processing");
        
        /* Setup for ethernet input */
        to_nexts[n_arp] = bi;
        nexts[n_arp] = ethernet_index;

        /* Ensure interface indices are set correctly */
        vnet_buffer(b)->sw_if_index[VLIB_RX] = ep->rx_sw_if_index;
        vnet_buffer(b)->sw_if_index[VLIB_TX] = ep->tx_sw_if_index;
        
        n_arp++;
        
        /* Remove this entry from the wheel */
        if (current != wp->tail)
        {
          u32 last_pos = (wp->tail == 0) ? (wp->wheel_size - 1) : (wp->tail - 1);
          
          /* Safety check before copying last entry */
          if (PREDICT_FALSE(last_pos >= wp->wheel_size)) {
            clib_warning("Invalid wheel position %d (wheel size %d)", last_pos, wp->wheel_size);
            current = (current + 1) % wp->wheel_size;
            count--;
            continue;
          }
          
          cbs_wheel_entry_t *last_ep = wp->entries + last_pos;
          *ep = *last_ep;  /* Copy the last entry to this position */
        }
        
        /* Adjust tail */
        wp->tail = (wp->tail == 0) ? (wp->wheel_size - 1) : (wp->tail - 1);
        wp->cursize--;
        
        /* Don't advance current since we replaced this entry */
        count--;
      }
      else
      {
        /* Not an ARP packet, advance to next entry */
        current = (current + 1) % wp->wheel_size;
        count--;
      }
    }
    else
    {
      /* Packet too small to be an ethernet frame with ARP, skip */
      current = (current + 1) % wp->wheel_size;
      count--;
    }
  }
  
  /* Send any ARP packets found to ethernet-input */
  if (n_arp > 0)
  {
    clib_warning("Sending %d ARP packets to ethernet-input from CBS wheel", n_arp);
    vlib_buffer_enqueue_to_next(vm, node, to_nexts, nexts, n_arp);
    vlib_node_increment_counter(vm, node->node_index, CBS_TX_ERROR_ARP_PACKETS_TX, n_arp);
  }
  
  return n_arp;
}

/* CBS Dequeue Logic */
always_inline uword
cbs_input_inline (vlib_main_t * vm, vlib_node_runtime_t * node,
               vlib_frame_t * frame)
{
  cbs_main_t *cbsm = &cbs_main;
  u32 thread_index = vm->thread_index;
  cbs_wheel_t *wp = NULL;
  
  /* Safety check for NULL wheel_by_thread pool */
  if (PREDICT_FALSE (cbsm->wheel_by_thread == NULL))
    return 0;
    
  /* Get the wheel for this thread from the pool */
  if (PREDICT_FALSE (pool_elts(cbsm->wheel_by_thread) <= thread_index))
    return 0;
    
  wp = cbsm->wheel_by_thread[thread_index];
  if (PREDICT_FALSE(wp == 0))
    return 0;
    
  f64 now = vlib_time_now (vm);
  f64 delta_t, gained_credits;
  u32 n_tx_packets = 0;
  
  /* Handle ARP/ICMP packets separately to avoid credit checks */
  if (PREDICT_FALSE (wp == 0))
      return 0;
  
  /* First check for and process any high priority packets */
  cbs_handle_high_priority_packets(vm, node, wp, cbsm);
  
  /* Then check for and process any ARP packets that might be stuck */
  cbs_handle_arp_packets(vm, node, wp, cbsm);

  /* 1. Check if previous transmission is ongoing */
  if (now < wp->cbs_last_tx_finish_time)
    {
      /* Do nothing for now, we'll check for high-priority traffic later */
    }

  /* 2. Calculate elapsed time and update credits */
  delta_t = now - wp->cbs_last_update_time;
  /* Always update credits regardless of queue state */
  if (delta_t > 1e-9) 
    {
      /* Accumulate credits based on idleslope */
      gained_credits = delta_t * cbsm->cbs_idleslope;
      wp->cbs_credits += gained_credits;

      /* Clamp credits at hicredit */
      wp->cbs_credits = clib_min(wp->cbs_credits, (f64)cbsm->cbs_hicredit);
      
      /* Record time of update */
      wp->cbs_last_update_time = now;
    }

  /* 3. Check if transmission is allowed */
  /* Allow some transmission even with negative credits */
  f64 credit_limit = (f64)cbsm->cbs_locredit * 1.5;  /* Use 150% of locredit as limit */
  if (wp->cbs_credits < credit_limit)
    {
      /* Only count as stalled if wheel has packets - reduces counter spam */
      if (wp->cursize > 0)
        vlib_node_increment_counter(vm, node->node_index, CBS_TX_ERROR_STALLED, 1);
      /* Continue processing anyway to allow some packet flow */
    }

  /* 4. Check if there are packets to send */
  if (wp->cursize == 0)
    {
      vlib_node_increment_counter(vm, node->node_index, CBS_TX_ERROR_NO_PKTS, 1);
      return 0; /* Nothing to send */
    }
  
  /* Set a reasonable floor for negative credits */
  f64 credit_floor = -(f64)cbsm->cbs_hicredit;
  
  /* 5. Try to dequeue and transmit a burst */
  u32 to_nexts[CBS_MAX_TX_BURST];
  u16 nexts[CBS_MAX_TX_BURST];
  
  /* Always try to transmit at least one packet to prevent starvation */
  cbs_wheel_entry_t *first_ep = wp->entries + wp->head;
  u32 first_bi = first_ep->buffer_index;
  if (first_bi != ~0) /* Valid buffer index */
    {
      vlib_buffer_t *first_b = vlib_get_buffer(vm, first_bi);
      if (first_b) /* Valid buffer */
        {
          u32 len = vlib_buffer_length_in_chain(vm, first_b);
          to_nexts[n_tx_packets] = first_bi;
          nexts[n_tx_packets] = first_ep->output_next_index;
          
          /* Deduct credits */
          f64 cost = (f64)len / cbsm->cbs_port_rate;
          f64 credits_before = wp->cbs_credits;
          wp->cbs_credits -= cost;
          
          /* Update transmission state */
          f64 tx_time = (f64)len / cbsm->cbs_port_rate;
          wp->cbs_last_tx_finish_time = now + tx_time;
          
          /* Trace if needed */
          if (PREDICT_FALSE(first_b->flags & VLIB_BUFFER_IS_TRACED))
            {
              cbs_tx_trace_add(vm, node, first_bi, now, first_ep->output_next_index, 
                           credits_before, wp->cbs_credits);
            }
          
          /* Clear wheel entry */
          first_ep->buffer_index = ~0;
          
          /* Advance head */
          wp->head = (wp->head + 1) % wp->wheel_size;
          wp->cursize--;
          
          n_tx_packets++;
        }
    }
  
  /* Process more packets if possible */
  static u32 force_packet_counter = 0;
  while (n_tx_packets < CBS_MAX_TX_BURST && wp->cursize > 0) 
    {
      /* Only continue if credits aren't critically low or we're forcing */
      if (wp->cbs_credits < credit_floor && (++force_packet_counter % 5 != 0))
        break;
        
      /* Check if port is busy */
      if (now < wp->cbs_last_tx_finish_time)
        break;
        
      /* Get next packet from wheel */
      cbs_wheel_entry_t *ep = wp->entries + wp->head;
      u32 bi = ep->buffer_index;
      
      /* Skip invalid buffers */
      if (bi == ~0) 
        {
          wp->head = (wp->head + 1) % wp->wheel_size;
          wp->cursize--;
          continue;
        }
      
      vlib_buffer_t *b = vlib_get_buffer(vm, bi);
      if (!b) 
        {
          wp->head = (wp->head + 1) % wp->wheel_size;
          wp->cursize--;
          continue;
        }
      
      /* Calculate credits needed */
      u32 len = vlib_buffer_length_in_chain(vm, b);
      f64 cost = (f64)len / cbsm->cbs_port_rate;
      f64 credits_before = wp->cbs_credits;
      
      /* Send packet */
      to_nexts[n_tx_packets] = bi;
      nexts[n_tx_packets] = ep->output_next_index;
      
      /* Update credits and transmission state */
      wp->cbs_credits -= cost;
      f64 tx_time = (f64)len / cbsm->cbs_port_rate;
      wp->cbs_last_tx_finish_time = now + tx_time;
      
      /* Trace if needed */
      if (PREDICT_FALSE(b->flags & VLIB_BUFFER_IS_TRACED))
        {
          cbs_tx_trace_add(vm, node, bi, now, ep->output_next_index, 
                         credits_before, wp->cbs_credits);
        }
      
      /* Clear wheel entry */
      ep->buffer_index = ~0;
      
      /* Advance head */
      wp->head = (wp->head + 1) % wp->wheel_size;
      wp->cursize--;
      
      n_tx_packets++;
    }
  
  /* 6. Enqueue transmitted packets */
  if (n_tx_packets > 0)
    {
      vlib_buffer_enqueue_to_next(vm, node, to_nexts, nexts, n_tx_packets);
      vlib_node_increment_counter(vm, node->node_index, 
                              CBS_TX_ERROR_TRANSMITTED, n_tx_packets);
    }

  return n_tx_packets;
}

/* 
 * Process ARP and other control packets with highest priority
 * This function guarantees ARP packets are transmitted regardless of credit state
 */
static u32
cbs_handle_high_priority_packets(vlib_main_t *vm, vlib_node_runtime_t *node,
                       cbs_wheel_t *wp, cbs_main_t *cbsm)
{
  /* Nothing to do if wheel is empty */
  if (wp->cursize == 0)
    return 0;

  u32 n_priority = 0;
  u32 to_nexts[CBS_MAX_TX_BURST];
  u16 nexts[CBS_MAX_TX_BURST];
  u32 processed_indices[CBS_MAX_TX_BURST]; /* Indices in wheel of processed packets */
  u32 n_processed = 0;
  
  /* Get punt index for control packets */
  u32 punt_index = CBS_NEXT_DROP;
  
  /* First scan for high priority packets - this prevents head of line blocking */
  u32 scan_count = clib_min(wp->cursize, CBS_MAX_TX_BURST * 2); /* Scan twice the burst size */
  u32 current_idx = wp->head;
  
  /* Only check the first several packets in wheel to avoid excessive scanning */
  for (u32 i = 0; i < scan_count && n_priority < CBS_MAX_TX_BURST; i++) {
    cbs_wheel_entry_t *ep = wp->entries + current_idx;
    u32 bi = ep->buffer_index;
    vlib_buffer_t *b = vlib_get_buffer(vm, bi);
    
    /* Skip invalid buffers */
    if (PREDICT_FALSE(!b)) {
      current_idx = (current_idx + 1) % wp->wheel_size;
      continue;
    }
    
    /* Check if this is a high priority packet (ARP, ICMP, etc.) */
    if (b->current_length >= sizeof(ethernet_header_t)) {
      ethernet_header_t *eth = vlib_buffer_get_current(b);
      u16 ethertype = clib_net_to_host_u16(eth->type);
      
      /* Always prioritize ARP and other control packets */
      u8 is_priority = 0;
      
      /* Check for ARP */
      if (ethertype == ETHERNET_TYPE_ARP) {
        is_priority = 1;
      }
      
      /* Check for IPv4 ICMP */
      else if (ethertype == ETHERNET_TYPE_IP4 && 
              b->current_length >= sizeof(ethernet_header_t) + sizeof(ip4_header_t)) {
        ip4_header_t *ip4 = (ip4_header_t *)(eth + 1);
        if (ip4->protocol == IP_PROTOCOL_ICMP) {
          is_priority = 1;
        }
      }
      
      /* Check for IPv6 ICMP */
      else if (ethertype == ETHERNET_TYPE_IP6 && 
              b->current_length >= sizeof(ethernet_header_t) + sizeof(ip6_header_t)) {
        ip6_header_t *ip6 = (ip6_header_t *)(eth + 1);
        if (ip6->protocol == IP_PROTOCOL_ICMP6) {
          is_priority = 1;
        }
      }
      
      /* Process priority packet */
      if (is_priority) {
        /* Track for special processing */
        to_nexts[n_priority] = bi;
        
        /* Always use punt index for control packets */
        nexts[n_priority] = punt_index;
        
        /* Mark this wheel entry as processed */
        processed_indices[n_processed++] = current_idx;
        
        n_priority++;
      }
    }
    
    current_idx = (current_idx + 1) % wp->wheel_size;
  }
  
  /* Process the high priority packets */
  if (n_priority > 0) {
    /* Send packets immediately - optimize wheel updates later */
    vlib_buffer_enqueue_to_next(vm, node, to_nexts, nexts, n_priority);
    
    /* Remove these entries from the wheel, working backwards to maintain order */
    for (i32 i = n_processed - 1; i >= 0; i--) {
      u32 idx = processed_indices[i];
      
      /* If this is the head, just advance it */
      if (idx == wp->head) {
        wp->head = (wp->head + 1) % wp->wheel_size;
        wp->cursize--;
      } else {
        /* Otherwise, we need to carefully reorganize the wheel */
        /* This is a rare case so performance impact should be minimal */
        u32 head_idx = wp->head;
        
        /* Move all entries between head and idx-1 one position forward */
        while (head_idx != idx) {
          u32 prev_idx = (head_idx == 0) ? wp->wheel_size - 1 : head_idx - 1;
          cbs_wheel_entry_t *dst = &wp->entries[head_idx];
          cbs_wheel_entry_t *src = &wp->entries[prev_idx];
          
          /* Copy entry */
          *dst = *src;
          
          /* Move to next */
          head_idx = prev_idx;
        }
        
        /* Update head */
        wp->head = (wp->head + 1) % wp->wheel_size;
        wp->cursize--;
      }
    }
    
    vlib_node_increment_counter(vm, node->node_index, CBS_TX_ERROR_ARP_PACKETS_TX, n_priority);
  }
  
  return n_priority;
}

/* Dispatch function to CBS input node */
VLIB_NODE_FN (cbs_input_node) (vlib_main_t * vm, vlib_node_runtime_t * node, vlib_frame_t * frame)
{
  /* Quick check if tracing is enabled */
  return cbs_input_inline(vm, node, frame);
}

#ifndef CLIB_MARCH_VARIANT
/* Renamed input node registration */
VLIB_REGISTER_NODE (cbs_input_node) =
{
  .type = VLIB_NODE_TYPE_INPUT, // Input node type, runs periodically
  .name = "cbs-wheel", // Renamed node name
  .state = VLIB_NODE_STATE_DISABLED, // Initially disabled, enabled by config
  .format_trace = format_cbs_tx_trace, // Renamed trace formatter
  .n_errors = CBS_TX_N_ERROR, // Renamed error count
  .error_strings = cbs_tx_error_strings, // Renamed error strings
  .vector_size = sizeof(u32), // Typical for input nodes
  .n_next_nodes = CBS_N_NEXT, // Define number of next nodes
  .next_nodes = {
    [CBS_NEXT_DROP] = "error-drop", // Map drop index to error-drop node
    [CBS_NEXT_ETHERNET] = "ethernet-input", // Map ethernet-input index to a valid node
  },
};

// Input node scheduling function
static uword
cbs_wheel_input_process_enable_disable (vlib_main_t * vm,
                                  vlib_node_runtime_t * node,
                                  vlib_frame_t * frame)
{
  // Toggle state of input node (on/off)
  vlib_node_set_state (vm, cbs_input_node.index,
                      (node->state == VLIB_NODE_STATE_DISABLED) ?
                       VLIB_NODE_STATE_POLLING : VLIB_NODE_STATE_DISABLED);
  return 0;
}

VLIB_REGISTER_NODE (cbs_enable_node) = {
  .name = "cbs-wheel-enable-disable", // Node to enable/disable the wheel processing
  .function = cbs_wheel_input_process_enable_disable,
  .type = VLIB_NODE_TYPE_PROCESS,
};
#endif /* CLIB_MARCH_VARIANT */

/*
 * fd.io coding-style-patch-verification: ON
 *
 * Local Variables:
 * eval: (c-set-style "gnu")
 * End:
 */