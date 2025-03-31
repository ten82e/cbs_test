/*
 * node.c - VPP CBS plugin enqueue node functions (buffering logic only)
 * Pure CBS shaper node. No Loss/Reorder.
 *
 * Copyright (c) 2024 Your Org <your.email@example.com> // Placeholder
 * Based on nsim node.c, Copyright (c) Cisco and/or its affiliates.
 * Licensed under the Apache License, Version 2.0 (the "License");
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


/* --- Error Code Definitions --- */
#define foreach_cbs_error                              \
_(BUFFERED, "Packets buffered to CBS wheel")            \
_(DROPPED_WHEEL_FULL, "Packets dropped (wheel full)")    \
_(DROPPED_LOOKUP_FAIL, "Packets dropped (fwd lookup failed)") \
_(NO_WHEEL, "No CBS wheel configured for thread (forwarded)") \
_(NOT_CONFIGURED, "CBS not configured (forwarded)")

typedef enum
{
#define _(sym,str) CBS_ERROR_##sym,
  foreach_cbs_error
#undef _
    CBS_N_ERROR,
} cbs_error_t;

/* --- Next Node Definitions --- */
// Only drop path is defined for enqueue nodes
typedef enum
{
  CBS_NEXT_DROP,
  CBS_N_NEXT // Should be 1
} cbs_next_t;

/* --- Trace Structure --- */
typedef struct
{
  u32 buffer_index;
  u32 rx_sw_if_index;
  u32 tx_sw_if_index; // TX sw_if_index *before* potential modification in lookup
  cbs_trace_action_t trace_action;
  u32 calculated_next_index; // Next index *after* the wheel node
} cbs_trace_t;


/* --- Static Function Declarations/Definitions --- */

/** @brief Add trace record for CBS enqueue node. */
static void
cbs_add_trace (vlib_main_t * vm, vlib_node_runtime_t * node,
               vlib_buffer_t * b, cbs_trace_action_t trace_action,
               u32 calculated_next_index)
{
   if (PREDICT_FALSE((node->flags & VLIB_NODE_FLAG_TRACE) && (b->flags & VLIB_BUFFER_IS_TRACED)))
     {
       cbs_trace_t *t = vlib_add_trace (vm, node, b, sizeof (*t));
       t->buffer_index = vlib_get_buffer_index(vm, b);
       t->rx_sw_if_index = vnet_buffer(b)->sw_if_index[VLIB_RX];
       t->tx_sw_if_index = vnet_buffer(b)->sw_if_index[VLIB_TX]; // Record original TX
       t->trace_action = trace_action;
       t->calculated_next_index = calculated_next_index;
     }
}

/** @brief Determine the next node index *after* the CBS wheel. */
always_inline void
cbs_buffer_fwd_lookup (cbs_main_t * cbsm, vlib_buffer_t * b,
                       u32 * next, u8 is_cross_connect)
{
  vlib_main_t * vm = vlib_get_main(); // Get vlib_main for logging node names
  u32 tx_sw_if_index = vnet_buffer (b)->sw_if_index[VLIB_TX]; // Original TX before potential change
  u32 rx_sw_if_index = vnet_buffer (b)->sw_if_index[VLIB_RX];
  u32 bi = vlib_get_buffer_index(vm, b);
  (void)bi; // Suppress unused warning if VLIB_DEBUG=0

  if (is_cross_connect) {
      u32 peer_sw_if_index = (rx_sw_if_index == cbsm->sw_if_index0) ? cbsm->sw_if_index1 : cbsm->sw_if_index0;

      // Validate before assigning to buffer's TX index
      if (PREDICT_FALSE(peer_sw_if_index == (u32)~0 || rx_sw_if_index == (u32)~0 ||
                        (rx_sw_if_index != cbsm->sw_if_index0 && rx_sw_if_index != cbsm->sw_if_index1))) {
          #if VLIB_DEBUG > 0
          clib_warning("CBS FWD Lookup FAIL (XConn Invalid If): bi %u, invalid rx %u / peer %u (cfg: %u, %u)",
                       bi, rx_sw_if_index, peer_sw_if_index, cbsm->sw_if_index0, cbsm->sw_if_index1);
          #endif
          *next = CBS_NEXT_DROP;
          return;
      }
      // Modify buffer's TX sw_if_index for the peer
      vnet_buffer (b)->sw_if_index[VLIB_TX] = peer_sw_if_index;

      // Lookup the next node based on the *peer* interface
      *next = (peer_sw_if_index == cbsm->sw_if_index0) ? cbsm->output_next_index0 : cbsm->output_next_index1;

      if (PREDICT_FALSE(*next == CBS_NEXT_DROP || *next == (u32)~0)) { // Cast ~0
         #if VLIB_DEBUG > 0
          vlib_node_t *next_node = vlib_get_node_by_next_node_and_edge(vm, cbs_input_node.index, *next);
          clib_warning("CBS FWD Lookup FAIL (XConn Invalid Next): bi %u, rx %u, peer %u -> next_index %u ('%U') (cfg next0: %u, next1: %u)",
                       bi, rx_sw_if_index, peer_sw_if_index, *next,
                       format_vlib_node_name, vm, (next_node ? next_node->index : (u32)~0),
                       cbsm->output_next_index0, cbsm->output_next_index1);
         #endif
         *next = CBS_NEXT_DROP; // Ensure drop on failure
      }


  } else { /* Output feature mode */
      // --- ADD DEBUG LOG START ---
      #if VLIB_DEBUG > 0
      u32 vec_len_check = vec_len(cbsm->output_next_index_by_sw_if_index);
      // Read value for logging only if index is valid
      u32 stored_next_check = (tx_sw_if_index != (u32)~0 && tx_sw_if_index < vec_len_check) ? cbsm->output_next_index_by_sw_if_index[tx_sw_if_index] : (u32)-2;
      clib_warning("CBS FWD Lookup DBG: Enter Output Mode: bi %u, tx_sw_if %u, vec_len %u, read stored_next %u",
                    bi, tx_sw_if_index, vec_len_check, stored_next_check);
      #endif
      // --- ADD DEBUG LOG END ---

      // Use the original tx_sw_if_index for lookup
      if (PREDICT_FALSE(tx_sw_if_index == (u32)~0)) { // Cast ~0
          #if VLIB_DEBUG > 0
          clib_warning("CBS FWD Lookup FAIL (Output Invalid TX If): bi %u, invalid tx_sw_if_index %u", bi, tx_sw_if_index);
          #endif
          *next = CBS_NEXT_DROP;
          return;
      }

      u32 vec_len_safe = vec_len(cbsm->output_next_index_by_sw_if_index);
      u32 potential_next = (u32)~0; // Initialize potential_next

      if (tx_sw_if_index < vec_len_safe) {
           potential_next = cbsm->output_next_index_by_sw_if_index[tx_sw_if_index];
      }

      // Check if the retrieved value is valid (~0 means not set or invalid)
      if (PREDICT_TRUE(potential_next != (u32)~0)) {
          *next = potential_next;
          #if VLIB_DEBUG > 1 // Make this log level higher if too verbose
          vlib_node_t *next_node = vlib_get_node_by_next_node_and_edge(vm, cbs_input_node.index, *next);
          clib_warning("CBS FWD Lookup OK (Output): bi %u, tx_sw_if %u -> next_index %u ('%U')",
                       bi, tx_sw_if_index, *next,
                       format_vlib_node_name, vm, (next_node ? next_node->index : (u32)~0));
          #endif
      } else {
          #if VLIB_DEBUG > 0 // Modified debug log
          clib_warning("CBS FWD Lookup FAIL (Output Not Found/Invalid): bi %u, tx_sw_if %u not found or invalid next in vec (len %u, next_val %u)",
                       bi, tx_sw_if_index, vec_len_safe, potential_next); // Log potential_next which would be ~0
          #endif
          *next = CBS_NEXT_DROP;
      }
  }

  // Final safety check (should be redundant if logic above is correct, but keep for safety)
  if (PREDICT_FALSE(*next == (u32)~0)) {
       #if VLIB_DEBUG > 0
       clib_warning("CBS FWD Lookup FAIL (Final Check): bi %u for tx_sw_if %u resulted in next ~0", bi, tx_sw_if_index);
       #endif
      *next = CBS_NEXT_DROP;
  }
}


/** @brief Processes a single buffer: buffer to wheel or drop. */
// ============================================================================
// MODIFIED cbs_dispatch_buffer starts here
// ============================================================================
always_inline void
cbs_dispatch_buffer (vlib_main_t * vm, vlib_node_runtime_t * node,
                     cbs_main_t * cbsm, cbs_wheel_t * wp, vlib_buffer_t * b,
                     u32 bi, cbs_node_ctx_t * ctx, u8 is_cross_connect)
{
    // Check wheel full condition first
    if (PREDICT_FALSE(wp->cursize >= wp->wheel_size)) {
        ctx->drop[0] = bi;
        ctx->drop++;
        // ホイール満杯によるドロップはトレース（nsimも同様の可能性）
        cbs_add_trace(vm, node, b, CBS_TRACE_ACTION_DROP_WHEEL_FULL, CBS_NEXT_DROP);
        return;
    }

    // --- MODIFICATION START ---
    // 先にホイールにエントリを追加する
    cbs_wheel_entry_t *e = &wp->entries[wp->tail];
    wp->tail = (wp->tail + 1) % wp->wheel_size;
    wp->cursize++;
    ctx->n_buffered++;

    // ルックアップを実行し、結果を直接エントリに保存する
    // ルックアップが失敗した場合 (CBS_NEXT_DROP or ~0) でも、
    // その値が保存され、後のデキュー時にドロップされることになる
    cbs_buffer_fwd_lookup(cbsm, b, &e->output_next_index, is_cross_connect); // Lookup and store

    e->buffer_index = bi;
    e->rx_sw_if_index = vnet_buffer(b)->sw_if_index[VLIB_RX]; // Store original RX
    e->tx_sw_if_index = vnet_buffer(b)->sw_if_index[VLIB_TX]; // Store potentially modified TX
    // e->output_next_index は cbs_buffer_fwd_lookup で設定済み

    // バッファリング成功のトレースを追加 (ルックアップ結果も含む)
    cbs_add_trace(vm, node, b, CBS_TRACE_ACTION_BUFFER, e->output_next_index);
    // Removed the specific trace/drop for lookup failure at enqueue time.
    // --- MODIFICATION END ---
}
// ============================================================================
// MODIFIED cbs_dispatch_buffer ends here
// ============================================================================


/* --- Main Node Function --- */
static_always_inline uword
cbs_inline_fn (vlib_main_t * vm, vlib_node_runtime_t * node, vlib_frame_t * frame,
	       int is_cross_connect)
{
    cbs_main_t *cbsm = &cbs_main;
    u32 thread_index = vm->thread_index;
    cbs_wheel_t *wp = NULL;
    u32 n_left_from, *from;
    vlib_buffer_t *bufs[VLIB_FRAME_SIZE], **b;
    u32 drops[VLIB_FRAME_SIZE]; // Array to store indices of buffers to be dropped
    cbs_node_ctx_t ctx;       // Context for this frame processing

    from = vlib_frame_vector_args (frame);
    n_left_from = frame->n_vectors;
    vlib_get_buffers (vm, from, bufs, n_left_from);
    b = bufs;

    // --- Fallback Logic: If CBS not configured or no wheel for this thread ---
    if (PREDICT_FALSE(!cbsm->is_configured || thread_index >= vec_len(cbsm->wheel_by_thread) || !(wp = cbsm->wheel_by_thread[thread_index]))) {
        // ... (フォールバック処理は変更なし) ...
         u16 *next_indices = vlib_node_get_runtime_data (vm, node->node_index);
         if (PREDICT_FALSE(!next_indices)) {
             vlib_buffer_free (vm, from, frame->n_vectors);
             // Note: Using CBS_ERROR_DROPPED_LOOKUP_FAIL here might be slightly inaccurate now,
             // as it could be NO_WHEEL or NOT_CONFIGURED, but it indicates a failure to forward.
             vlib_node_increment_counter(vm, node->node_index, CBS_ERROR_DROPPED_LOOKUP_FAIL, frame->n_vectors);
             return frame->n_vectors;
         }
        vlib_buffer_enqueue_to_next(vm, node, from, next_indices, frame->n_vectors);
        vlib_node_increment_counter (vm, node->node_index,
                                     cbsm->is_configured ? CBS_ERROR_NO_WHEEL : CBS_ERROR_NOT_CONFIGURED,
                                     frame->n_vectors);
        return frame->n_vectors;
    }

    // --- Normal Processing Logic: Buffer packets to the CBS wheel ---
    ctx.drop = drops;          // Point to the start of the drop array
    ctx.n_buffered = 0;      // Reset counters for this frame
    // ctx.n_lookup_drop = 0; // <<<--- Removed counter initialization, no longer needed here

    // --- Process Packets in Batches ---
    // ... (ループ処理は変更なし、内部で修正された cbs_dispatch_buffer が呼ばれる) ...
    while (n_left_from >= 8) {
        vlib_prefetch_buffer_header(b[4], STORE); vlib_prefetch_buffer_header(b[5], STORE);
        vlib_prefetch_buffer_header(b[6], STORE); vlib_prefetch_buffer_header(b[7], STORE);

        cbs_dispatch_buffer (vm, node, cbsm, wp, b[0], from[0], &ctx, is_cross_connect);
        cbs_dispatch_buffer (vm, node, cbsm, wp, b[1], from[1], &ctx, is_cross_connect);
        cbs_dispatch_buffer (vm, node, cbsm, wp, b[2], from[2], &ctx, is_cross_connect);
        cbs_dispatch_buffer (vm, node, cbsm, wp, b[3], from[3], &ctx, is_cross_connect);
        cbs_dispatch_buffer (vm, node, cbsm, wp, b[4], from[4], &ctx, is_cross_connect);
        cbs_dispatch_buffer (vm, node, cbsm, wp, b[5], from[5], &ctx, is_cross_connect);
        cbs_dispatch_buffer (vm, node, cbsm, wp, b[6], from[6], &ctx, is_cross_connect);
        cbs_dispatch_buffer (vm, node, cbsm, wp, b[7], from[7], &ctx, is_cross_connect);

        b += 8; from += 8; n_left_from -= 8;
    }
     while (n_left_from >= 4) {
        vlib_prefetch_buffer_header(b[0], STORE); vlib_prefetch_buffer_header(b[1], STORE);
        vlib_prefetch_buffer_header(b[2], STORE); vlib_prefetch_buffer_header(b[3], STORE);

        cbs_dispatch_buffer (vm, node, cbsm, wp, b[0], from[0], &ctx, is_cross_connect);
        cbs_dispatch_buffer (vm, node, cbsm, wp, b[1], from[1], &ctx, is_cross_connect);
        cbs_dispatch_buffer (vm, node, cbsm, wp, b[2], from[2], &ctx, is_cross_connect);
        cbs_dispatch_buffer (vm, node, cbsm, wp, b[3], from[3], &ctx, is_cross_connect);

        b += 4; from += 4; n_left_from -= 4;
    }
    // Process remaining packets individually
    while (n_left_from > 0) {
        cbs_dispatch_buffer (vm, node, cbsm, wp, b[0], from[0], &ctx, is_cross_connect);
        b += 1; from += 1; n_left_from -= 1;
    }

    // --- Handle Dropped Packets ---
    u32 n_dropped_total = ctx.drop - drops; // Calculate total drops in this frame
    if (PREDICT_FALSE(n_dropped_total > 0)) {
        vlib_buffer_free (vm, drops, n_dropped_total); // Free the dropped buffers

        // --- MODIFICATION START ---
        // Increment drop counters
        // u32 n_wheel_full_drops = n_dropped_total - ctx.n_lookup_drop; // Original
        // Now all drops counted here are due to wheel full, as lookup failures are not tracked separately here.
        u32 n_wheel_full_drops = n_dropped_total;
        if (n_wheel_full_drops > 0) {
             vlib_node_increment_counter (vm, node->node_index, CBS_ERROR_DROPPED_WHEEL_FULL, n_wheel_full_drops);
        }
        // Removed the specific counter increment for DROPPED_LOOKUP_FAIL
        // if (ctx.n_lookup_drop > 0) {
        //      vlib_node_increment_counter (vm, node->node_index, CBS_ERROR_DROPPED_LOOKUP_FAIL, ctx.n_lookup_drop);
        // }
         #if VLIB_DEBUG > 0
         if (n_dropped_total > 0) {
             // Adjusted log message
             clib_warning("CBS Enqueue node %s: Dropped %u packets (WheelFull: %u)",
                          node->name, n_dropped_total, n_wheel_full_drops);
         }
         #endif
         // --- MODIFICATION END ---
    }

   // Increment buffered packet counter
   if (ctx.n_buffered > 0) {
      vlib_node_increment_counter (vm, node->node_index, CBS_ERROR_BUFFERED, ctx.n_buffered);
   }

   return frame->n_vectors; // Return total number of packets processed in the input frame
}


/* --- Node Function Definitions --- */
// Wrapper function for cross-connect mode
VLIB_NODE_FN (cbs_cross_connect_node) (vlib_main_t * vm, vlib_node_runtime_t * node, vlib_frame_t * frame)
{
    return cbs_inline_fn (vm, node, frame, 1 /* is_cross_connect = true */);
}

// Wrapper function for output feature mode
VLIB_NODE_FN (cbs_output_feature_node) (vlib_main_t * vm, vlib_node_runtime_t * node, vlib_frame_t * frame)
{
    return cbs_inline_fn (vm, node, frame, 0 /* is_cross_connect = false */);
}


/* --- Node Registration and Trace Formatting (Base Implementation Only) --- */
#ifndef CLIB_MARCH_VARIANT

// --- Error Strings ---
static char * cbs_error_strings[] = {
#define _(sym,string) string,
  foreach_cbs_error
#undef _
};

// --- Trace Formatting ---
/**
 * @brief Format trace output for enqueue nodes.
 */
static u8 *
format_cbs_trace (u8 * s, va_list * args)
{
  CLIB_UNUSED (vlib_main_t * vm) = va_arg (*args, vlib_main_t *);
  CLIB_UNUSED (vlib_node_t * node) = va_arg (*args, vlib_node_t *);
  cbs_trace_t *t = va_arg (*args, cbs_trace_t *);
  char * action_str = "UNKNOWN";

  switch(t->trace_action) {
      case CBS_TRACE_ACTION_BUFFER: action_str = "BUFFER"; break;
      case CBS_TRACE_ACTION_DROP_WHEEL_FULL: action_str = "DROP_WHEEL_FULL"; break;
      // CBS_TRACE_ACTION_DROP_LOOKUP_FAIL is no longer generated here, but keep case for completeness? Or remove? Let's remove for now.
      // case CBS_TRACE_ACTION_DROP_LOOKUP_FAIL: action_str = "DROP_LOOKUP_FAIL"; break;
      default: break;
  }

  // Include the calculated next index in the trace
  s = format (s, "CBS_ENQ (bi %u): %s rx_sw %u tx_sw %u next_idx %u",
              t->buffer_index, action_str, t->rx_sw_if_index, t->tx_sw_if_index,
              t->calculated_next_index);
  return s;
}

// --- Node Registrations ---
// Define node registration variables (used by VLIB_REGISTER_NODE)
vlib_node_registration_t cbs_cross_connect_node;
vlib_node_registration_t cbs_output_feature_node;

VLIB_REGISTER_NODE (cbs_cross_connect_node) =
{
  .name = "cbs-cross-connect",
  .vector_size = sizeof (u32),
  .format_trace = format_cbs_trace,
  .type = VLIB_NODE_TYPE_INTERNAL, // Internal node, processes packets between other nodes
  .n_errors = ARRAY_LEN(cbs_error_strings),
  .error_strings = cbs_error_strings,
  .n_next_nodes = CBS_N_NEXT, // Only has a drop node as next
  .next_nodes = {
    [CBS_NEXT_DROP] = "error-drop", // Default drop node
  },
  // Add .process_function if needed for runtime node state changes
};

VLIB_REGISTER_NODE (cbs_output_feature_node) =
{
  .name = "cbs-output-feature",
  .vector_size = sizeof (u32),
  .format_trace = format_cbs_trace,
  .type = VLIB_NODE_TYPE_INTERNAL, // Internal node
  .n_errors = ARRAY_LEN(cbs_error_strings),
  .error_strings = cbs_error_strings,
  .n_next_nodes = CBS_N_NEXT, // Only has a drop node as next
  .next_nodes = {
    [CBS_NEXT_DROP] = "error-drop", // Default drop node
  },
};

#endif // CLIB_MARCH_VARIANT

/*
 * fd.io coding-style-patch-verification: ON
 * Local Variables:
 * eval: (c-set-style "gnu")
 * End:
 */