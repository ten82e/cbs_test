/*
 * node.c - VPP CBS plugin node functions (enqueue logic) with ARP bypass
 *
 * Copyright (c) 2024 Your Org <your.email@example.com> // Placeholder
 * Based on nsim node.c, Copyright (c) Cisco and/or its affiliates.
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
#include <vnet/feature/feature.h>
#include <vnet/ethernet/ethernet.h>
#include <vppinfra/error.h>
#include <vppinfra/vec.h>
#include <vppinfra/macros.h>
#include <vppinfra/byte_order.h>
#include <cbs/cbs.h>

typedef struct
{
  u32 tx_sw_if_index;
  int is_drop;
  u32 buffer_index;
  u16 ethertype;
  u8 action_taken; // Add action for detailed trace
} cbs_trace_t;

#define foreach_cbs_error                              \
_(BUFFERED, "Packets buffered to CBS wheel")            \
_(DROPPED, "Packets dropped (various reasons)")         \
_(ARP_BYPASSED, "ARP packets bypassed")                \
_(NO_WHEEL, "No CBS wheel configured")                 \
_(INVALID_NEXT_INDEX, "Invalid next index")            \
_(WHEEL_FULL, "CBS wheel full, packet dropped")        \
_(CBS_TX_ERROR_STALLED, "cbs-wheel credits stalled")    \
_(CBS_TX_ERROR_NO_PKTS, "cbs-wheel empty")             \
_(CBS_TX_ERROR_TRANSMITTED, "cbs packets transmitted") \
_(CBS_TX_ERROR_ARP_PACKETS_TX, "cbs priority packets transmitted")

typedef enum
{
#define _(sym,str) CBS_ERROR_##sym,
  foreach_cbs_error
#undef _
    CBS_N_ERROR,
} cbs_error_t;

typedef enum
{
  CBS_NEXT_DROP,
  CBS_NEXT_PUNT,
  CBS_N_NEXT,
} cbs_next_t;

#ifndef CLIB_MARCH_VARIANT
/* 
 * Forward declarations 
 */
static void
cbs_trace_buffer (vlib_main_t * vm, vlib_node_runtime_t * node,
               vlib_buffer_t * b, cbs_wheel_t *wp, int is_drop, u16 ethertype, u8 action_taken);

always_inline void
cbs_buffer_fwd_lookup (cbs_main_t * cbsm, vlib_buffer_t * b,
            u16 * next, u8 is_cross_connect);

static void __attribute__ ((unused))
cbs_dispatch_buffer (vlib_main_t * vm, vlib_node_runtime_t * node,
              cbs_main_t * cbsm, cbs_wheel_t * wp, vlib_buffer_t * b,
              u32 bi, cbs_node_ctx_t * ctx, u8 is_cross_connect,
              u8 is_trace);

static u8 *
format_cbs_trace (u8 * s, va_list * args)
{
  vlib_main_t * vm = va_arg (*args, vlib_main_t *);
  CLIB_UNUSED (vlib_node_t * node) = va_arg (*args, vlib_node_t *);
  cbs_trace_t *t = va_arg (*args, cbs_trace_t *);
  vlib_buffer_t *b = vlib_get_buffer(vm, t->buffer_index);
  char * action_str = "Unknown";

  if (t->is_drop)
      action_str = "Wheel_Full_Drop";
  else if (t->ethertype == ETHERNET_TYPE_ARP)
      action_str = "ARP_Bypass";
  else
       action_str = "Buffered";


  s = format (s, "CBS (bi %u): %s (rx_sw %d, tx_sw %d, type 0x%04x)",
              t->buffer_index, action_str,
              (b ? vnet_buffer(b)->sw_if_index[VLIB_RX] : ~0),
              t->tx_sw_if_index, t->ethertype);
  return s;
}

static char * cbs_error_strings[] = {
#define _(sym,string) string,
  foreach_cbs_error
#undef _
};

vlib_node_registration_t cbs_cross_connect_node;
vlib_node_registration_t cbs_output_feature_node;

/* 
 * Inline processing function - Main packet processing 
 */
always_inline uword
cbs_inline (vlib_main_t * vm, vlib_node_runtime_t * node, vlib_frame_t * frame,
         int is_cross_connect, int is_trace);

/*
 * Node function definitions */
VLIB_NODE_FN (cbs_cross_connect_node) (vlib_main_t * vm, vlib_node_runtime_t * node, vlib_frame_t * frame)
{
    return cbs_inline (vm, node, frame, 1 /*is_cross_connect*/, (node->flags & VLIB_NODE_FLAG_TRACE));
}

VLIB_NODE_FN (cbs_output_feature_node) (vlib_main_t * vm, vlib_node_runtime_t * node, vlib_frame_t * frame)
{
    return cbs_inline (vm, node, frame, 0 /*is_cross_connect*/, (node->flags & VLIB_NODE_FLAG_TRACE));
}


#ifndef CLIB_MARCH_VARIANT
/* Node Registration */
VLIB_REGISTER_NODE (cbs_cross_connect_node) =
{
  .name = "cbs-cross-connect",
  .vector_size = sizeof (u32),
  .format_trace = format_cbs_trace,
  .type = VLIB_NODE_TYPE_INTERNAL,
  .n_errors = ARRAY_LEN(cbs_error_strings),
  .error_strings = cbs_error_strings,
  .n_next_nodes = CBS_N_NEXT,
  .next_nodes = {
    [CBS_NEXT_DROP] = "error-drop",
    [CBS_NEXT_PUNT] = "punt-dispatch",
  },
};

VLIB_REGISTER_NODE (cbs_output_feature_node) =
{
  .name = "cbs-output-feature",
  .vector_size = sizeof (u32),
  .format_trace = format_cbs_trace,
  .type = VLIB_NODE_TYPE_INTERNAL,
  .n_errors = ARRAY_LEN(cbs_error_strings),
  .error_strings = cbs_error_strings,
  .n_next_nodes = CBS_N_NEXT,
  .next_nodes = {
    [CBS_NEXT_DROP] = "error-drop",
    [CBS_NEXT_PUNT] = "punt-dispatch",
  },
};

/* Helper struct for tracing */
CLIB_PACKED (struct cbs_output_trace {
  u32 next_index;
  u32 sw_if_index;
});

/* Register as feature in the feature arc */
VNET_FEATURE_INIT (cbs, static) = {
  .arc_name = "interface-output",
  .node_name = "cbs-output-feature",
  .runs_before = VNET_FEATURES ("interface-output-arc-end"),
};
#endif /* CLIB_MARCH_VARIANT */

/* Action constants */
#define CBS_ACTION_BUFFER 4 /* For buffering packets in wheel */

/*
 * fd.io coding-style-patch-verification: ON
 *
 * Local Variables:
 * eval: (c-set-style "gnu")
 * End:
 */

/* 
 * Inline processing function - Main packet processing 
 */
always_inline uword
cbs_inline (vlib_main_t * vm, vlib_node_runtime_t * node, vlib_frame_t * frame,
	     int is_cross_connect, int is_trace)
{
    cbs_main_t *cbsm = &cbs_main;
    u32 n_left_from, *from;
    vlib_buffer_t *bufs[VLIB_FRAME_SIZE], **b;
    u32 drops[VLIB_FRAME_SIZE];
    u32 reorders[VLIB_FRAME_SIZE]; /* Used for bypassed ARP and reordered packets */
    u16 reorders_nexts[VLIB_FRAME_SIZE]; /* Next indices for bypassed ARP and reordered packets */
    cbs_node_ctx_t ctx;

    /* Early check if per-thread wheels are configured */
    if (PREDICT_FALSE(!cbsm->wheel_by_thread || vm->thread_index >= vec_len(cbsm->wheel_by_thread))) {
        vlib_node_increment_counter (vm, node->node_index, CBS_ERROR_NO_WHEEL, frame->n_vectors);
        /* If no wheel configured, just pass packets through to next node */
        from = vlib_frame_vector_args (frame);
        n_left_from = frame->n_vectors;
        vlib_get_buffers (vm, from, bufs, n_left_from);
        
        /* Process packets one by one without using CBS wheel */
        for (int i = 0; i < n_left_from; i++) {
            u16 next_index;
            vnet_feature_next_u16(&next_index, bufs[i]);
            
            if (PREDICT_FALSE(next_index >= node->n_next_nodes)) {
                next_index = CBS_NEXT_DROP;
                vlib_node_increment_counter (vm, node->node_index, CBS_ERROR_INVALID_NEXT_INDEX, 1);
            }
            
            reorders[i] = from[i];
            reorders_nexts[i] = next_index;
        }
        
        /* Enqueue all packets to their next nodes */
        if (n_left_from > 0) {
            vlib_buffer_enqueue_to_next (vm, node, reorders, reorders_nexts, n_left_from);
        }
        
        return frame->n_vectors;
    }

    /* Get wheel pointer for this thread */
    cbs_wheel_t *wp = cbsm->wheel_by_thread[vm->thread_index];
    
    /* Safety check for thread's wheel */
    if (PREDICT_FALSE(wp == NULL)) {
        vlib_node_increment_counter (vm, node->node_index, CBS_ERROR_NO_WHEEL, frame->n_vectors);
        /* If wheel is NULL, just pass packets through to next node */
        from = vlib_frame_vector_args (frame);
        n_left_from = frame->n_vectors;
        vlib_get_buffers (vm, from, bufs, n_left_from);
        
        /* Process packets one by one without using CBS wheel */
        for (int i = 0; i < n_left_from; i++) {
            u16 next_index;
            vnet_feature_next_u16(&next_index, bufs[i]);
            
            if (PREDICT_FALSE(next_index >= node->n_next_nodes)) {
                next_index = CBS_NEXT_DROP;
                vlib_node_increment_counter (vm, node->node_index, CBS_ERROR_INVALID_NEXT_INDEX, 1);
            }
            
            reorders[i] = from[i];
            reorders_nexts[i] = next_index;
        }
        
        /* Enqueue all packets to their next nodes */
        if (n_left_from > 0) {
            vlib_buffer_enqueue_to_next (vm, node, reorders, reorders_nexts, n_left_from);
        }
        
        return frame->n_vectors;
    }

    /* Setup for processing packets */
    from = vlib_frame_vector_args (frame);
    n_left_from = frame->n_vectors;
    vlib_get_buffers (vm, from, bufs, n_left_from);
    b = bufs;
    
    /* Initialize context */
    ctx.n_buffered = 0; 
    ctx.drop = drops; ctx.reord = reorders; ctx.reord_nexts = reorders_nexts;
    ctx.fwd = NULL; ctx.fwd_nexts = NULL;

    /* Process packets using the optimized dispatch function */
    while (n_left_from >= 4) {
        /* Prefetch the next iteration */
        if (n_left_from >= 8) {
            vlib_prefetch_buffer_header (b[4], LOAD);
            vlib_prefetch_buffer_header (b[5], LOAD);
            vlib_prefetch_buffer_header (b[6], LOAD);
            vlib_prefetch_buffer_header (b[7], LOAD);
        }
        
        /* Process current packets */
        cbs_dispatch_buffer (vm, node, cbsm, wp, b[0], from[0], &ctx, is_cross_connect, is_trace);
        cbs_dispatch_buffer (vm, node, cbsm, wp, b[1], from[1], &ctx, is_cross_connect, is_trace);
        cbs_dispatch_buffer (vm, node, cbsm, wp, b[2], from[2], &ctx, is_cross_connect, is_trace);
        cbs_dispatch_buffer (vm, node, cbsm, wp, b[3], from[3], &ctx, is_cross_connect, is_trace);
        
        /* Move to next packets */
        b += 4; from += 4; n_left_from -= 4;
    }
    
    /* Process remaining packets */
    while (n_left_from > 0) {
        cbs_dispatch_buffer (vm, node, cbsm, wp, b[0], from[0], &ctx, is_cross_connect, is_trace);
        b += 1; from += 1; n_left_from -= 1;
    }

    /* Handle dropped packets */
    u32 n_dropped = ctx.drop - drops;
    if (n_dropped > 0) {
        vlib_buffer_free (vm, drops, n_dropped);
        vlib_node_increment_counter (vm, node->node_index, CBS_ERROR_DROPPED, n_dropped);
    }

   /* Handle bypassed/ARP packets */
   u32 n_bypassed = ctx.reord - reorders;
   if (n_bypassed > 0) {
       /* Use VPP helper function for batch enqueueing */
       vlib_buffer_enqueue_to_next (vm, node, reorders, reorders_nexts, n_bypassed);
       
       /* Update counters */
       vlib_node_increment_counter (vm, node->node_index, CBS_ERROR_ARP_BYPASSED, n_bypassed);
    }

  /* Update buffered counter */
  if (ctx.n_buffered > 0) {
     vlib_node_increment_counter (vm, node->node_index, CBS_ERROR_BUFFERED, ctx.n_buffered);
  }
  
  return frame->n_vectors;
}
#endif /* CLIB_MARCH_VARIANT */

/* 
 * Trace packet processing for debugging 
 */
static void
cbs_trace_buffer (vlib_main_t * vm, vlib_node_runtime_t * node,
               vlib_buffer_t * b, cbs_wheel_t *wp, int is_drop, u16 ethertype, u8 action_taken)
{
  if (PREDICT_FALSE(b && (b->flags & VLIB_BUFFER_IS_TRACED)))
    {
      cbs_trace_t *t = vlib_add_trace (vm, node, b, sizeof (*t));
      t->buffer_index = vlib_get_buffer_index(vm, b);
      t->is_drop = is_drop;
      t->tx_sw_if_index = vnet_buffer (b)->sw_if_index[VLIB_TX];
      t->ethertype = ethertype;
      t->action_taken = action_taken;
    }
}

/* 
 * Determine packet next index based on forwarding configuration 
 */
always_inline void
cbs_buffer_fwd_lookup (cbs_main_t * cbsm, vlib_buffer_t * b,
            u16 * next, u8 is_cross_connect)
{
  if (is_cross_connect) {
      vnet_buffer (b)->sw_if_index[VLIB_TX] =
    (vnet_buffer (b)->sw_if_index[VLIB_RX] == cbsm->sw_if_index0) ? 
     cbsm->sw_if_index1 : cbsm->sw_if_index0;
      
      /* Check interface indices are valid */
      if (PREDICT_FALSE(vnet_buffer (b)->sw_if_index[VLIB_TX] == ~0)) {
        *next = CBS_NEXT_DROP;
        return;
      }
      
      /* Set appropriate next index based on TX interface */
      *next = (vnet_buffer (b)->sw_if_index[VLIB_TX] == cbsm->sw_if_index0) ? 
              cbsm->output_next_index0 : cbsm->output_next_index1;
  } else { /* Output feature */
      u32 sw_if_index = vnet_buffer (b)->sw_if_index[VLIB_TX];
      
      /* Check interface index is valid */
      if (PREDICT_FALSE(sw_if_index == ~0)) {
        *next = CBS_NEXT_DROP;
        return;
      }
      
      /* Check interface has a valid next index configured */
      if (sw_if_index < vec_len(cbsm->output_next_index_by_sw_if_index) &&
          cbsm->output_next_index_by_sw_if_index[sw_if_index] != ~0)
          *next = cbsm->output_next_index_by_sw_if_index[sw_if_index];
      else
          *next = CBS_NEXT_DROP;
  }
  
  /* Safety check for next index */
  if (PREDICT_FALSE(*next >= CBS_N_NEXT))
    *next = CBS_NEXT_DROP;
}

/* 
 * Final dispatch function: ARP Bypass + Buffering 
 */
static void __attribute__ ((unused))
cbs_dispatch_buffer (vlib_main_t * vm, vlib_node_runtime_t * node,
              cbs_main_t * cbsm, cbs_wheel_t * wp, vlib_buffer_t * b,
              u32 bi, cbs_node_ctx_t * ctx, u8 is_cross_connect,
              u8 is_trace)
{
    ethernet_header_t * eth;
    i16 l2_hdr_offset = vnet_buffer(b)->l2_hdr_offset;
    eth = (ethernet_header_t *)((u8*)vlib_buffer_get_current(b) + l2_hdr_offset);
    u16 ethertype = clib_net_to_host_u16(eth->type);

    /* --- 1. Bypass ARP First --- */
    if (ethertype == ETHERNET_TYPE_ARP)
    {
        u16 next_index;
        vnet_feature_next_u16(&next_index, b);
        
        ctx->reord[0] = bi;
        ctx->reord_nexts[0] = next_index;
        ctx->reord += 1;
        ctx->reord_nexts += 1;
        
        /* Increment ARP bypass counter */
        vlib_node_increment_counter (vm, node->node_index, CBS_ERROR_ARP_BYPASSED, 1);
        if (PREDICT_FALSE(is_trace)) {
            cbs_trace_buffer(vm, node, b, wp, 0, ethertype, 0 /* No specific action */);
        }
    }
    /* --- 2. Buffer to CBS Wheel --- */
    else
    {
        u16 next_idx_after_wheel = 0;

        /* Determine next index based on forwarding type */
        cbs_buffer_fwd_lookup(cbsm, b, &next_idx_after_wheel, is_cross_connect);
        
        /* First check if we have space in the wheel */
        if (PREDICT_FALSE(wp->cursize >= wp->wheel_size)) {
            /* Drop the packet if wheel is full */
            *ctx->drop++ = bi;
            cbs_trace_buffer(vm, node, b, wp, 1 /* is_drop */, ethertype, CBS_ERROR_WHEEL_FULL);
            vlib_node_increment_counter(vm, node->node_index, CBS_ERROR_WHEEL_FULL, 1);
            return;
        }
        
        /* We have space in the wheel, so add the packet */
        cbs_wheel_entry_t *e = &wp->entries[wp->tail];
        
        /* Initialize the wheel entry */
        e->buffer_index = bi;
        e->rx_sw_if_index = vnet_buffer(b)->sw_if_index[VLIB_RX];
        e->tx_sw_if_index = vnet_buffer(b)->sw_if_index[VLIB_TX];
        e->output_next_index = next_idx_after_wheel;
        e->ethertype = ethertype;
        
        /* Update wheel pointers and counters */
        wp->tail = (wp->tail + 1) % wp->wheel_size;
        wp->cursize++;
        ctx->n_buffered++;
        
        /* Trace the buffering action */
        cbs_trace_buffer(vm, node, b, wp, 0 /* no drop */, ethertype, CBS_ACTION_BUFFER);
    }
}
