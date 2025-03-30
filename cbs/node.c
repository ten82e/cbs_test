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
// #include <vppinfra/random.h> // Removed
#include <vppinfra/byte_order.h>
#include <cbs/cbs.h>

/** \brief Trace structure for CBS enqueue node */
typedef struct
{
  u32 buffer_index;
  u32 rx_sw_if_index;
  u32 tx_sw_if_index;
  // u16 ethertype; // Removed
  cbs_trace_action_t trace_action;
} cbs_trace_t;

/** \brief Enqueue node error counters (Simplified) */
#define foreach_cbs_error                              \
_(BUFFERED, "Packets buffered to CBS wheel")            \
_(DROPPED_WHEEL_FULL, "Packets dropped (wheel full)")    \
_(NO_WHEEL, "No CBS wheel configured for thread")        \
_(INTERNAL_ERROR, "Internal processing errors (lookup failed)")

typedef enum
{
#define _(sym,str) CBS_ERROR_##sym,
  foreach_cbs_error
#undef _
    CBS_N_ERROR,
} cbs_error_t;

/** \brief Next node definitions for enqueue nodes (Only DROP needed) */
typedef enum
{
  CBS_NEXT_DROP,
  CBS_N_NEXT
} cbs_next_t;


#ifndef CLIB_MARCH_VARIANT // Start of base implementation block

/* Error strings */
static char * cbs_error_strings[] = {
#define _(sym,string) string,
  foreach_cbs_error
#undef _
};

/* Node registrations (forward declarations for VLIB_NODE_FN) */
vlib_node_registration_t cbs_cross_connect_node;
vlib_node_registration_t cbs_output_feature_node;

/* Trace formatting function definition (BEFORE VLIB_REGISTER_NODE) */
static u8 *
format_cbs_trace (u8 * s, va_list * args)
{
  CLIB_UNUSED (vlib_main_t * vm) = va_arg (*args, vlib_main_t *);
  CLIB_UNUSED (vlib_node_t * node) = va_arg (*args, vlib_node_t *);
  cbs_trace_t *t = va_arg (*args, cbs_trace_t *);
  char * action_str = "UNKNOWN";

  // Updated trace actions (Simplified)
  switch(t->trace_action) {
      case CBS_TRACE_ACTION_BUFFER: action_str = "BUFFER"; break;
      // case CBS_TRACE_ACTION_DROP_SIM: action_str = "DROP_SIM"; break; // Removed
      // case CBS_TRACE_ACTION_REORDER_SIM: action_str = "REORDER_SIM"; break; // Removed
      case CBS_TRACE_ACTION_DROP_WHEEL_FULL: action_str = "DROP_WHEEL_FULL"; break;
      default: break;
  }

  s = format (s, "CBS_ENQ (bi %u): %s rx_sw %d tx_sw %d", // Removed ethertype
              t->buffer_index, action_str, t->rx_sw_if_index, t->tx_sw_if_index);
  return s;
}


/* Static inline functions (common logic) */

/**
 * @brief Add trace record for CBS enqueue node.
 */
static void
cbs_add_trace (vlib_main_t * vm, vlib_node_runtime_t * node,
               vlib_buffer_t * b, cbs_trace_action_t trace_action)
{
  if (PREDICT_FALSE((node->flags & VLIB_NODE_FLAG_TRACE) && (b->flags & VLIB_BUFFER_IS_TRACED)))
    {
      cbs_trace_t *t = vlib_add_trace (vm, node, b, sizeof (*t));
      // ethernet_header_t * eth = vlib_buffer_get_current(b); // No longer needed

      t->buffer_index = vlib_get_buffer_index(vm, b);
      t->rx_sw_if_index = vnet_buffer(b)->sw_if_index[VLIB_RX];
      t->tx_sw_if_index = vnet_buffer(b)->sw_if_index[VLIB_TX];
      // t->ethertype removed
      t->trace_action = trace_action;
    }
}


/**
 * @brief Determine the next node index *after* the CBS wheel.
 */
always_inline void
cbs_buffer_fwd_lookup (cbs_main_t * cbsm, vlib_buffer_t * b,
                       u32 * next, u8 is_cross_connect)
{
  u32 tx_sw_if_index = vnet_buffer (b)->sw_if_index[VLIB_TX];
  u32 rx_sw_if_index = vnet_buffer (b)->sw_if_index[VLIB_RX];
  // u32 bi = vlib_get_buffer_index(vlib_get_main(), b); // Removed unused variable

  // clib_warning("Lookup START: bi %u, rx_sw %u, tx_sw %u, is_cross %d", bi, rx_sw_if_index, tx_sw_if_index, is_cross_connect);

  if (is_cross_connect) {
      tx_sw_if_index = (rx_sw_if_index == cbsm->sw_if_index0) ? cbsm->sw_if_index1 : cbsm->sw_if_index0;
      vnet_buffer (b)->sw_if_index[VLIB_TX] = tx_sw_if_index;

      if (PREDICT_FALSE(tx_sw_if_index == ~0 || rx_sw_if_index == ~0)) {
        // clib_warning("Lookup DROP (XConn): Invalid sw_if_index (rx %u, tx %u) for bi %u", rx_sw_if_index, tx_sw_if_index, bi);
        *next = CBS_NEXT_DROP;
        return;
      }
      *next = (tx_sw_if_index == cbsm->sw_if_index0) ? cbsm->output_next_index0 : cbsm->output_next_index1;
      // clib_warning("Lookup OK (XConn): bi %u, next %u (based on tx_sw %u)", bi, *next, tx_sw_if_index);

  } else { /* Output feature mode */
      if (PREDICT_FALSE(tx_sw_if_index == ~0)) {
        // clib_warning("Lookup DROP (Output): Invalid TX sw_if_index (%u) for bi %u", tx_sw_if_index, bi);
        *next = CBS_NEXT_DROP;
        return;
      }
      u32 vec_len_safe = vec_len(cbsm->output_next_index_by_sw_if_index);
      if (tx_sw_if_index < vec_len_safe &&
          cbsm->output_next_index_by_sw_if_index[tx_sw_if_index] != ~0) {
          *next = cbsm->output_next_index_by_sw_if_index[tx_sw_if_index];
          // clib_warning("Lookup OK (Output): bi %u, next %u from vector index %u", bi, *next, tx_sw_if_index);
      } else {
          // clib_warning("Lookup DROP (Output): No valid next index for bi %u, sw_if_index %u (vec_len %u, value %d)", bi, tx_sw_if_index, vec_len_safe, (tx_sw_if_index < vec_len_safe ? (i32)cbsm->output_next_index_by_sw_if_index[tx_sw_if_index] : -2));
          *next = CBS_NEXT_DROP;
      }
  }
}


/**
 * @brief Processes a single buffer: buffer or drop (wheel full).
 */
always_inline void
cbs_dispatch_buffer (vlib_main_t * vm, vlib_node_runtime_t * node,
                     cbs_main_t * cbsm, cbs_wheel_t * wp, vlib_buffer_t * b,
                     u32 bi, cbs_node_ctx_t * ctx, u8 is_cross_connect)
{
    // Loss and Reorder checks removed

    /* Buffer to CBS Wheel (if space available) */
    if (PREDICT_FALSE(wp->cursize >= wp->wheel_size)) {
        ctx->drop[0] = bi;
        ctx->drop++;
        vlib_node_increment_counter (vm, node->node_index, CBS_ERROR_DROPPED_WHEEL_FULL, 1);
        cbs_add_trace(vm, node, b, CBS_TRACE_ACTION_DROP_WHEEL_FULL);
        return;
    }

    u32 next_idx_after_wheel;
    cbs_buffer_fwd_lookup(cbsm, b, &next_idx_after_wheel, is_cross_connect);
    // clib_warning("Dispatch Buffer: bi %u, lookup result %u", bi, next_idx_after_wheel);

    if (PREDICT_FALSE(next_idx_after_wheel == CBS_NEXT_DROP)) {
         ctx->drop[0] = bi;
         ctx->drop++;
         vlib_node_increment_counter (vm, node->node_index, CBS_ERROR_INTERNAL_ERROR, 1);
         // Use DROP_WHEEL_FULL trace action for simplicity as it's a drop within dispatch
         cbs_add_trace(vm, node, b, CBS_TRACE_ACTION_DROP_WHEEL_FULL);
         return;
    }

    cbs_wheel_entry_t *e = &wp->entries[wp->tail];
    e->buffer_index = bi;
    e->rx_sw_if_index = vnet_buffer(b)->sw_if_index[VLIB_RX];
    e->tx_sw_if_index = vnet_buffer(b)->sw_if_index[VLIB_TX];
    e->output_next_index = next_idx_after_wheel;
    // e->ethertype removed

    wp->tail = (wp->tail + 1) % wp->wheel_size;
    wp->cursize++;
    ctx->n_buffered++;
    cbs_add_trace(vm, node, b, CBS_TRACE_ACTION_BUFFER);
}


/**
 * @brief Main function (non-static) for CBS enqueue nodes.
 */
uword
cbs_inline_fn (vlib_main_t * vm, vlib_node_runtime_t * node, vlib_frame_t * frame,
	       int is_cross_connect)
{
    cbs_main_t *cbsm = &cbs_main;
    u32 thread_index = vm->thread_index;
    cbs_wheel_t *wp = NULL;
    u32 n_left_from, *from;
    vlib_buffer_t *bufs[VLIB_FRAME_SIZE], **b;
    u32 drops[VLIB_FRAME_SIZE];
    // u8 action_flags[VLIB_FRAME_SIZE]; // Removed action flags
    cbs_node_ctx_t ctx;
    u32 n_dropped_fallback = 0;

    from = vlib_frame_vector_args (frame);
    n_left_from = frame->n_vectors;
    vlib_get_buffers (vm, from, bufs, n_left_from);
    b = bufs;

    if (PREDICT_FALSE(!cbsm->is_configured || thread_index >= vec_len(cbsm->wheel_by_thread) || !(wp = cbsm->wheel_by_thread[thread_index]))) {
        vlib_node_increment_counter (vm, node->node_index, CBS_ERROR_NO_WHEEL, frame->n_vectors);
        // Fallback: Forward directly
        u32 n_fwd = 0;
        u32 current_n_left = n_left_from;
        u32 *current_from = from;
        vlib_buffer_t **current_b = b;
        u32 tmp_fwd[VLIB_FRAME_SIZE];
        u16 tmp_fwd_nexts[VLIB_FRAME_SIZE];

        while(current_n_left > 0)
        {
            u16 next_index;
            vnet_feature_next_u16(&next_index, current_b[0]);
            if (PREDICT_FALSE(next_index >= node->n_next_nodes)) {
                 drops[n_dropped_fallback++] = current_from[0];
            } else {
                 tmp_fwd[n_fwd] = current_from[0];
                 tmp_fwd_nexts[n_fwd] = next_index;
                 n_fwd++;
            }
            current_b += 1; current_from += 1; current_n_left -= 1;
        }
        if (n_dropped_fallback > 0) {
            vlib_buffer_free(vm, drops, n_dropped_fallback);
            vlib_node_increment_counter(vm, node->node_index, CBS_ERROR_INTERNAL_ERROR, n_dropped_fallback);
        }
        if (n_fwd > 0) {
            vlib_buffer_enqueue_to_next(vm, node, tmp_fwd, tmp_fwd_nexts, n_fwd);
        }
        return frame->n_vectors;
    }

    ctx.drop = drops;
    // Removed action_flags, reord, reord_nexts, n_loss, n_reordered init
    ctx.n_buffered = 0;

    // Removed action setting logic

    while (n_left_from >= 4) {
        if (n_left_from >= 8) {
           vlib_prefetch_buffer_header (b[4], LOAD);
           vlib_prefetch_buffer_header (b[5], LOAD);
           vlib_prefetch_buffer_header (b[6], LOAD);
           vlib_prefetch_buffer_header (b[7], LOAD);
        }

        // Removed action_flags pointer advancement from dispatch call
        cbs_dispatch_buffer (vm, node, cbsm, wp, b[0], from[0], &ctx, is_cross_connect);
        cbs_dispatch_buffer (vm, node, cbsm, wp, b[1], from[1], &ctx, is_cross_connect);
        cbs_dispatch_buffer (vm, node, cbsm, wp, b[2], from[2], &ctx, is_cross_connect);
        cbs_dispatch_buffer (vm, node, cbsm, wp, b[3], from[3], &ctx, is_cross_connect);

        b += 4; from += 4; n_left_from -= 4;
    }

    while (n_left_from > 0) {
        cbs_dispatch_buffer (vm, node, cbsm, wp, b[0], from[0], &ctx, is_cross_connect);
        b += 1; from += 1; n_left_from -= 1;
    }

    u32 n_dropped_total = ctx.drop - drops;
    if (n_dropped_total > 0) {
        vlib_buffer_free (vm, drops, n_dropped_total);
        // No loss simulation counter anymore
    }

    // Removed reorder/bypass enqueue block

   if (ctx.n_buffered > 0) {
      vlib_node_increment_counter (vm, node->node_index, CBS_ERROR_BUFFERED, ctx.n_buffered);
   }

   return frame->n_vectors;
}

/* Node Functions */
VLIB_NODE_FN (cbs_cross_connect_node) (vlib_main_t * vm, vlib_node_runtime_t * node, vlib_frame_t * frame)
{
    return cbs_inline_fn (vm, node, frame, 1);
}

VLIB_NODE_FN (cbs_output_feature_node) (vlib_main_t * vm, vlib_node_runtime_t * node, vlib_frame_t * frame)
{
    return cbs_inline_fn (vm, node, frame, 0);
}


/* Node Registration */
VLIB_REGISTER_NODE (cbs_cross_connect_node) =
{
  .name = "cbs-cross-connect",
  .vector_size = sizeof (u32),
  .format_trace = format_cbs_trace,
  .type = VLIB_NODE_TYPE_INTERNAL,
  .n_errors = ARRAY_LEN(cbs_error_strings), // Updated error count
  .error_strings = cbs_error_strings,
  .n_next_nodes = CBS_N_NEXT,
  .next_nodes = {
    [CBS_NEXT_DROP] = "error-drop",
  },
};

VLIB_REGISTER_NODE (cbs_output_feature_node) =
{
  .name = "cbs-output-feature",
  .vector_size = sizeof (u32),
  .format_trace = format_cbs_trace,
  .type = VLIB_NODE_TYPE_INTERNAL,
  .n_errors = ARRAY_LEN(cbs_error_strings), // Updated error count
  .error_strings = cbs_error_strings,
  .n_next_nodes = CBS_N_NEXT,
  .next_nodes = {
    [CBS_NEXT_DROP] = "error-drop",
  },
};

#endif // CLIB_MARCH_VARIANT

/* ... (fd.io tags etc.) ... */