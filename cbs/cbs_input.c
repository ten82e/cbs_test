/*
 * cbs_input.c - VPP CBS plugin input node (dequeue logic based on CBS)
 * Implemented as INPUT node + Polling.
 *
 * Copyright (c) 2024 Your Org <your.email@example.com> // Placeholder
 * Based on nsim_input.c, Copyright (c) Cisco and/or its affiliates.
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#include <vlib/vlib.h>
#include <vnet/vnet.h>
#include <vppinfra/error.h> // Needed for clib_warning
#include <vppinfra/time.h>
#include <vppinfra/format.h>
#include <vppinfra/macros.h>
#include <cbs/cbs.h>
#include <vnet/ethernet/ethernet.h>
// #include <vlib/log.h> // Keep if other parts still use vlib_log, remove if not

/* --- Error Code Definitions --- */
#define foreach_cbs_tx_error                    \
_(TRANSMITTED, "Packets transmitted by CBS")    \
_(STALLED_CREDITS, "CBS stalled (insufficient credits)") \
_(STALLED_PORT_BUSY, "CBS stalled (port busy)") \
_(NO_PKTS_IN_WHEEL, "CBS wheel empty when polled")       \
_(NO_WHEEL_FOR_THREAD, "No CBS wheel configured for thread")\
_(INVALID_BUFFER, "Invalid buffer index found in wheel")

typedef enum
{
#define _(sym,str) CBS_TX_ERROR_##sym,
  foreach_cbs_tx_error
#undef _
    CBS_TX_N_ERROR,
} cbs_tx_error_t;


/** \brief Trace structure for CBS dequeue node */
typedef struct
{
  u32 buffer_index;
  u32 next_index;
  f64 tx_time;
  f64 cbs_credits_before;
  f64 cbs_credits_after;
  u32 packet_len;
} cbs_tx_trace_t;


/* --- Static Function Declarations/Definitions --- */

// Forward declaration for trace function
static void
cbs_input_add_trace (vlib_main_t * vm, vlib_node_runtime_t * node,
                   u32 bi, f64 tx_time, u32 next_index,
                   f64 credits_before, f64 credits_after, u32 len);


/* --- Input Node Function (Inline) --- */
static_always_inline uword
cbs_input_inline (vlib_main_t * vm, vlib_node_runtime_t * node,
                  vlib_frame_t * frame)
{
   cbs_main_t *cbsm = &cbs_main;
   u32 thread_index = vm->thread_index;
   cbs_wheel_t *wp = NULL;
   f64 now;
   u32 n_tx_packets = 0;
   u32 to_next_bufs[CBS_MAX_TX_BURST];
   u16 to_next_nodes[CBS_MAX_TX_BURST];

   // --- Initial checks ---
   if (PREDICT_FALSE(!cbsm->is_configured)) return 0;

   if (PREDICT_FALSE (thread_index >= vec_len(cbsm->wheel_by_thread) || !(wp = cbsm->wheel_by_thread[thread_index]))) {
       vlib_node_increment_counter (vm, node->node_index, CBS_TX_ERROR_NO_WHEEL_FOR_THREAD, 1);
       // clib_warning("T%u: No wheel found!", thread_index); // Optional debug
       return 0;
   }

   if (PREDICT_TRUE (wp->cursize == 0)) {
       // Increment counter only if needed for debugging empty polls
       // vlib_node_increment_counter(vm, node->node_index, CBS_TX_ERROR_NO_PKTS_IN_WHEEL, 1);
       return 0;
   }

   // --- Update Credits ---
   now = vlib_time_now (vm); // Get current time once for this poll cycle

   f64 delta_t = now - wp->cbs_last_update_time;
   if (PREDICT_TRUE(delta_t > 1e-9)) { // Avoid division by zero or negative time
       f64 gained_credits = delta_t * cbsm->cbs_idleslope;
       wp->cbs_credits += gained_credits;
       wp->cbs_credits = clib_min(wp->cbs_credits, cbsm->cbs_hicredit); // Cap at hicredit
       wp->cbs_last_update_time = now;
   }

   // --- Transmission Loop (Modified Logic) ---
   f64 current_tx_allowed_time = wp->cbs_last_tx_finish_time; // Initialize current allowed time for this burst

   while (n_tx_packets < CBS_MAX_TX_BURST && wp->cursize > 0) {

       // *** Port Busy Check (Using current_tx_allowed_time) ***
       if (now < current_tx_allowed_time) {
           // Log only if this is the *first* check in the loop that fails
           if (n_tx_packets == 0) {
                // clib_warning("CBS_DBG T%u: STALLED (port busy loop: now %.9f < allowed %.9f)", thread_index, now, current_tx_allowed_time); // Optional debug
                vlib_node_increment_counter (vm, node->node_index, CBS_TX_ERROR_STALLED_PORT_BUSY, 1);
           }
           break; // Stop sending for this poll cycle
       }

       cbs_wheel_entry_t *ep = wp->entries + wp->head;
       u32 bi = ep->buffer_index;

       // --- Buffer Validity Check ---
       if (PREDICT_FALSE(bi == ~0)) { // Skip already dequeued/invalid entries
           wp->head = (wp->head + 1) % wp->wheel_size;
           wp->cursize--;
           continue;
       }
       vlib_buffer_t *b = vlib_get_buffer(vm, bi);
       if (PREDICT_FALSE(!b)) {
           clib_warning("T%u: Invalid buffer index %u found in wheel", thread_index, bi); // Keep this warning
           vlib_node_increment_counter (vm, node->node_index, CBS_TX_ERROR_INVALID_BUFFER, 1);
           ep->buffer_index = ~0; // Mark as invalid in wheel
           wp->head = (wp->head + 1) % wp->wheel_size;
           wp->cursize--;
           continue;
       }
       u32 len = vlib_buffer_length_in_chain(vm, b);
       f64 credits_before = wp->cbs_credits; // For trace
       u32 next_node_index_for_buffer = ep->output_next_index;

       // --- Credit Check ---
       // Sendslope check removed as per simplified CBS definition (sendslope < 0 check)
       // if (wp->cbs_credits < 0 && cbsm->cbs_sendslope < 0) { // Original check
       if (wp->cbs_credits < cbsm->cbs_locredit && cbsm->cbs_sendslope <= 0) { // More standard check: below locredit and not gaining credits faster than sending
            // Log only if this is the *first* check in the loop that fails
            if (n_tx_packets == 0) {
                // clib_warning("CBS_DBG T%u: STALLED (credits %.4f < locredit %.4f && sendslope %.4f <= 0)",
                //             thread_index, wp->cbs_credits, cbsm->cbs_locredit, cbsm->cbs_sendslope); // Optional debug
                vlib_node_increment_counter (vm, node->node_index, CBS_TX_ERROR_STALLED_CREDITS, 1);
            }
            break; // Stop sending due to insufficient credits
        }

       // --- Prepare for Enqueue ---
       to_next_bufs[n_tx_packets] = bi;
       to_next_nodes[n_tx_packets] = (u16) next_node_index_for_buffer;

       // --- Calculate Transmission Duration & Update Credits ---
       f64 tx_duration = (f64)len / cbsm->cbs_port_rate;
       f64 credit_change = tx_duration * cbsm->cbs_sendslope; // Sendslope = idle - port
       wp->cbs_credits += credit_change;
       // Note: Credit is allowed to go below locredit during transmission

       // ★★★ Update the next allowed transmission time for *this burst* ★★★
       // Use the later of 'now' or the previous 'allowed' time as the start point
       current_tx_allowed_time = clib_max(now, current_tx_allowed_time) + tx_duration;

       // --- Add Trace & Update Wheel State ---
       cbs_input_add_trace(vm, node, bi, now, next_node_index_for_buffer, credits_before, wp->cbs_credits, len);
       ep->buffer_index = ~0; // Mark buffer as dequeued in the wheel entry
       wp->head = (wp->head + 1) % wp->wheel_size;
       wp->cursize--;
       n_tx_packets++;

     } // end while loop

   // --- Final Enqueue & State Update ---
   if (n_tx_packets > 0) {
       // ★★★ Record the final calculated allowed time for the next poll cycle ★★★
       wp->cbs_last_tx_finish_time = current_tx_allowed_time;

       vlib_buffer_enqueue_to_next(vm, node, to_next_bufs, to_next_nodes, n_tx_packets);
       vlib_node_increment_counter(vm, node->node_index, CBS_TX_ERROR_TRANSMITTED, n_tx_packets);
     }
   // else {
   //    // Optional: Log or count cases where the loop exited without sending (e.g., only stalls occurred)
   // }

   return n_tx_packets;
}


/* --- Trace Add Function --- */
// (Definition moved up for clarity, no functional change)
static void
cbs_input_add_trace (vlib_main_t * vm, vlib_node_runtime_t * node,
                   u32 bi, f64 tx_time, u32 next_index,
                   f64 credits_before, f64 credits_after, u32 len)
{
   vlib_buffer_t *b = vlib_get_buffer (vm, bi);
   // Check if tracing is enabled for the node AND the buffer
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


/* --- Node Function Wrapper --- */
VLIB_NODE_FN (cbs_input_node) (vlib_main_t * vm, vlib_node_runtime_t * node, vlib_frame_t * frame)
{
    // Directly call the inline function
    return cbs_input_inline (vm, node, frame);
}

/* --- Non-Variant Specific Code (Error Strings, Trace Formatting, Registration) --- */
#ifndef CLIB_MARCH_VARIANT

static char *cbs_tx_error_strings[] = {
#define _(sym,string) string,
  foreach_cbs_tx_error
#undef _
};

/* --- Trace Formatting --- */
static u8 *
format_cbs_tx_trace (u8 * s, va_list * args)
{
  CLIB_UNUSED (vlib_main_t * vm) = va_arg (*args, vlib_main_t *);
  CLIB_UNUSED (vlib_node_t * node) = va_arg (*args, vlib_node_t *);
  cbs_tx_trace_t *t = va_arg (*args, cbs_tx_trace_t *);

  // Format the trace output
  s = format (s, "CBS_DEQ (bi %u len %u): tx @ %.9f, next %u, credit %.4f -> %.4f", // Increased precision for time/credit
              t->buffer_index, t->packet_len, t->tx_time, t->next_index,
              t->cbs_credits_before, t->cbs_credits_after);
  return s;
}

/* --- Node Registration --- */
vlib_node_registration_t cbs_input_node; // Make sure it's defined before use

VLIB_REGISTER_NODE (cbs_input_node) =
{
  .type = VLIB_NODE_TYPE_INPUT, // Input node type, polled by scheduler
  .name = "cbs-wheel",
  .state = VLIB_NODE_STATE_DISABLED, // Initially disabled, enabled by configuration
  .format_trace = format_cbs_tx_trace,
  .n_errors = CBS_TX_N_ERROR,
  .error_strings = cbs_tx_error_strings,
  .vector_size = sizeof(u32), // Standard vector size
  // No specific next nodes defined here, decided dynamically by wheel entries
};

#endif // CLIB_MARCH_VARIANT

/*
 * fd.io coding-style-patch-verification: ON
 * Local Variables:
 * eval: (c-set-style "gnu")
 * End:
 */