/*
 * cbs_input.c - VPP CBS plugin input node (dequeue logic based on CBS)
 * Implemented as INPUT node + Polling, similar to nsim.
 *
 * Copyright (c) 2024 Your Org <your.email@example.com> // Placeholder
 * Based on nsim_input.c, Copyright (c) Cisco and/or its affiliates.
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#include <vlib/vlib.h>
#include <vnet/vnet.h>
#include <vppinfra/error.h>
#include <vppinfra/time.h>
#include <vppinfra/format.h> // For format functions
#include <cbs/cbs.h>
#include <vnet/ethernet/ethernet.h>
// #include <vlib/unix/process.h> // Removed

/** \brief Trace structure for CBS dequeue node */
typedef struct
{
  u32 buffer_index;       /**< Dequeued buffer index */
  u32 next_index;         /**< Next node index buffer is sent to */
  f64 tx_time;            /**< Actual transmit time determined by CBS */
  f64 cbs_credits_before; /**< Credits before this packet transmission */
  f64 cbs_credits_after;  /**< Credits after this packet transmission */
  u32 packet_len;         /**< Length of the transmitted packet */
} cbs_tx_trace_t;

#ifndef CLIB_MARCH_VARIANT
/* Forward declarations */
static u8 * format_cbs_tx_trace (u8 * s, va_list * args);
vlib_node_registration_t cbs_input_node; // Declare registration
#endif // CLIB_MARCH_VARIANT

/** \brief Dequeue node error counters */
#define foreach_cbs_tx_error                    \
_(TRANSMITTED, "Packets transmitted by CBS")    \
_(STALLED_CREDITS, "CBS stalled (insufficient credits)") \
_(STALLED_PORT_BUSY, "CBS stalled (port busy)") \
_(NO_PKTS, "CBS wheel empty")                   \
_(NO_WHEEL, "No CBS wheel configured for thread")\
_(INVALID_BUFFER, "Invalid buffer found in wheel")

typedef enum
{
#define _(sym,str) CBS_TX_ERROR_##sym,
  foreach_cbs_tx_error
#undef _
    CBS_TX_N_ERROR,
} cbs_tx_error_t;

#ifndef CLIB_MARCH_VARIANT
static char *cbs_tx_error_strings[] = {
#define _(sym,string) string,
  foreach_cbs_tx_error
#undef _
};
#endif // CLIB_MARCH_VARIANT

/**
 * @brief Add trace record for CBS dequeue node.
 */
static void
cbs_input_add_trace (vlib_main_t * vm, vlib_node_runtime_t * node,
                   u32 bi, f64 tx_time, u32 next_index,
                   f64 credits_before, f64 credits_after, u32 len)
{
  vlib_buffer_t *b = vlib_get_buffer (vm, bi);
  // Check trace flag on the node and buffer
  if (PREDICT_FALSE(node->flags & VLIB_NODE_FLAG_TRACE) && PREDICT_FALSE(b && (b->flags & VLIB_BUFFER_IS_TRACED)))
    {
      cbs_tx_trace_t *t = vlib_add_trace (vm, node, b, sizeof (*t));
      t->buffer_index = bi;
      t->tx_time = tx_time;
      t->next_index = next_index;
      t->cbs_credits_before = credits_before;
      t->cbs_credits_after = credits_after;
      t->packet_len = len;
    }
}

/**
 * @brief Inline function containing the core CBS dequeue logic.
 * Implements standard CBS algorithm.
 * Returns number of packets transmitted (uword).
 */
always_inline uword // Return uword (packets processed)
cbs_input_inline (vlib_main_t * vm, vlib_node_runtime_t * node,
                  vlib_frame_t * frame) // Frame is not used by INPUT node usually
{
  cbs_main_t *cbsm = &cbs_main;
  u32 thread_index = vm->thread_index;
  cbs_wheel_t *wp = NULL;
  f64 now;
  u32 n_tx_packets = 0;
  // Frame-local arrays for enqueueing
  u32 to_nexts[CBS_MAX_TX_BURST];
  u16 nexts[CBS_MAX_TX_BURST]; // Stores the *actual* next node index
  f64 min_suspend = 1e-6; // Used for stall calculation reporting only

  /* 0. Check if configured */
  if (PREDICT_FALSE(!cbsm->is_configured))
      return 0; // Not configured, do nothing

  /* 1. Get Wheel for this thread */
  if (PREDICT_FALSE (thread_index >= vec_len(cbsm->wheel_by_thread) || !(wp = cbsm->wheel_by_thread[thread_index])))
    {
      vlib_node_increment_counter (vm, node->node_index, CBS_TX_ERROR_NO_WHEEL, 1);
      return 0; // No wheel
    }

  /* 2. Check if wheel is empty */
  if (PREDICT_TRUE (wp->cursize == 0))
    {
      return 0; /* Nothing to send */
    }

  // *** Use clib_warning for logging ***
  clib_warning("CBS-Wheel Input (Thread %u): Polling. cursize=%u credits=%.2f last_tx_finish=%.6f",
             thread_index, wp->cursize, wp->cbs_credits, wp->cbs_last_tx_finish_time);

  now = vlib_time_now (vm);

  /* 3. Update Credits based on idleslope */
  f64 delta_t = now - wp->cbs_last_update_time;
  if (delta_t > 1e-9)
    {
      f64 gained_credits = delta_t * cbsm->cbs_idleslope;
      wp->cbs_credits += gained_credits;
      wp->cbs_credits = clib_min(wp->cbs_credits, cbsm->cbs_hicredit);
      wp->cbs_last_update_time = now;
      // clib_warning("CBS-Wheel Input (Thread %u): Credits updated (+%.2f) to %.2f", thread_index, gained_credits, wp->cbs_credits);
    }

  /* 4. Process dequeue candidates in a burst */
  while (n_tx_packets < CBS_MAX_TX_BURST && wp->cursize > 0)
    {
      f64 tx_allowed_time = wp->cbs_last_tx_finish_time;
      cbs_wheel_entry_t *ep = wp->entries + wp->head;
      u32 bi = ep->buffer_index;

      if (PREDICT_FALSE(bi == ~0)) { wp->head = (wp->head + 1) % wp->wheel_size; wp->cursize--; continue; }
      vlib_buffer_t *b = vlib_get_buffer(vm, bi);
      if (PREDICT_FALSE(!b)) {
        clib_warning("CBS-Wheel Input (Thread %u): Invalid buffer index %u found in wheel", thread_index, bi);
        vlib_node_increment_counter (vm, node->node_index, CBS_TX_ERROR_INVALID_BUFFER, 1);
        wp->head = (wp->head + 1) % wp->wheel_size; wp->cursize--; continue;
      }
      u32 len = vlib_buffer_length_in_chain(vm, b);

      /* C. Check Credits */
      if (wp->cbs_credits < 0)
        {
          f64 wait_time = 0.0;
          if (cbsm->cbs_idleslope > 1e-9)
              wait_time = (-wp->cbs_credits / cbsm->cbs_idleslope) + min_suspend;

          clib_warning("CBS-Wheel Input (Thread %u): Stalled credits (%.2f) for bi %u (len %u), wait %.6f s", thread_index, wp->cbs_credits, bi, len, wait_time);
          vlib_node_increment_counter (vm, node->node_index, CBS_TX_ERROR_STALLED_CREDITS, 1);
          break;
        }

      /* D. Check if port is free */
       if (now < tx_allowed_time)
        {
          clib_warning("CBS-Wheel Input (Thread %u): Stalled port busy (now %.6f < finish %.6f) for bi %u, wait %.6f s", thread_index, now, tx_allowed_time, bi, tx_allowed_time - now + min_suspend);
          vlib_node_increment_counter (vm, node->node_index, CBS_TX_ERROR_STALLED_PORT_BUSY, 1);
          break;
        }

      /* E. Transmit Packet */
      f64 credits_before = wp->cbs_credits;
      u32 next_index_for_buffer = ep->output_next_index;
      to_nexts[n_tx_packets] = bi;
      nexts[n_tx_packets] = (u16) next_index_for_buffer;

      /* F. Update CBS State */
      f64 tx_duration = (f64)len / cbsm->cbs_port_rate;
      f64 credit_change = tx_duration * cbsm->cbs_sendslope;
      wp->cbs_credits += credit_change;
      wp->cbs_credits = clib_max(wp->cbs_credits, cbsm->cbs_locredit);
      wp->cbs_last_tx_finish_time = now + tx_duration;
      wp->cbs_last_update_time = now;

      clib_warning("CBS-Wheel Input (Thread %u): Enqueuing bi %u (len %u) to next_idx %u (credit %.2f -> %.2f, finish %.6f)",
                   thread_index, bi, len, next_index_for_buffer, credits_before, wp->cbs_credits, wp->cbs_last_tx_finish_time);

      /* G. Add Trace */
      cbs_input_add_trace(vm, node, bi, now, next_index_for_buffer, credits_before, wp->cbs_credits, len);

      /* H. Advance Wheel Head */
      ep->buffer_index = ~0;
      wp->head = (wp->head + 1) % wp->wheel_size;
      wp->cursize--;
      n_tx_packets++;
    }

  /* 5. Enqueue transmitted packets */
  if (n_tx_packets > 0)
    {
      // clib_warning("CBS-Wheel Input (Thread %u): Calling enqueue_to_next for %u packets", thread_index, n_tx_packets);
      vlib_buffer_enqueue_to_next(vm, node, to_nexts, nexts, n_tx_packets);
      vlib_node_increment_counter(vm, node->node_index, CBS_TX_ERROR_TRANSMITTED, n_tx_packets);
    }

  return n_tx_packets; // Return number of packets processed
}

/* Node Function defined using VLIB_NODE_FN macro */
VLIB_NODE_FN (cbs_input_node) (vlib_main_t * vm, vlib_node_runtime_t * node, vlib_frame_t * frame)
{
    // Call the inline function which contains the actual logic
    return cbs_input_inline (vm, node, frame);
}


/* Node Registration (Back to INPUT type, DISABLED state) */
#ifndef CLIB_MARCH_VARIANT
VLIB_REGISTER_NODE (cbs_input_node) =
{
  .type = VLIB_NODE_TYPE_INPUT,      // Changed back to INPUT
  .name = "cbs-wheel",
  .state = VLIB_NODE_STATE_DISABLED, // Start DISABLED, enable polling via config
  .format_trace = format_cbs_tx_trace,
  .n_errors = CBS_TX_N_ERROR,
  .error_strings = cbs_tx_error_strings,
  .vector_size = sizeof(u32),         // Add vector_size like nsim
  /* No .n_next_nodes or .next_nodes needed */
};

/* Trace formatting */
static u8 *
format_cbs_tx_trace (u8 * s, va_list * args)
{
  CLIB_UNUSED (vlib_main_t * vm) = va_arg (*args, vlib_main_t *);
  CLIB_UNUSED (vlib_node_t * node) = va_arg (*args, vlib_node_t *);
  cbs_tx_trace_t *t = va_arg (*args, cbs_tx_trace_t *);

  s = format (s, "CBS_DEQ (bi %u len %u): tx @ %.6f, next %u, credit %.2f -> %.2f",
              t->buffer_index, t->packet_len, t->tx_time, t->next_index,
              t->cbs_credits_before, t->cbs_credits_after);
  return s;
}
#endif // CLIB_MARCH_VARIANT

/*
 * fd.io coding-style-patch-verification: ON
 *
 * Local Variables:
 * eval: (c-set-style "gnu")
 * End:
 */