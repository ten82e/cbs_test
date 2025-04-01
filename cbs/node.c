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
#include <vppinfra/error.h> // Needed for clib_warning
#include <vppinfra/vec.h>
#include <vppinfra/macros.h>
#include <vppinfra/byte_order.h>
#include <cbs/cbs.h>
// #include <vlib/log.h> // Keep if other parts still use vlib_log, remove if not

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
  u32 tx_sw_if_index;
  cbs_trace_action_t trace_action;
  u32 calculated_next_index;
} cbs_trace_t;


/* --- Static Function Declarations/Definitions --- */

static void
cbs_add_trace (vlib_main_t * vm, vlib_node_runtime_t * node,
               vlib_buffer_t * b, cbs_trace_action_t trace_action,
               u32 calculated_next_index);

/** @brief Determine the next node index *after* the CBS wheel. */
always_inline void
cbs_buffer_fwd_lookup (cbs_main_t * cbsm, vlib_buffer_t * b,
                       u32 * next, u8 is_cross_connect)
{
  vlib_main_t * vm = vlib_get_main();
  u32 tx_sw_if_index = vnet_buffer (b)->sw_if_index[VLIB_TX];
  u32 rx_sw_if_index = vnet_buffer (b)->sw_if_index[VLIB_RX];
  u32 bi = vlib_get_buffer_index(vm, b);
  (void)bi;

  if (is_cross_connect) {
      u32 peer_sw_if_index = (rx_sw_if_index == cbsm->sw_if_index0) ? cbsm->sw_if_index1 : cbsm->sw_if_index0;

      if (PREDICT_FALSE(peer_sw_if_index == (u32)~0 || rx_sw_if_index == (u32)~0 ||
                        (rx_sw_if_index != cbsm->sw_if_index0 && rx_sw_if_index != cbsm->sw_if_index1))) {
          clib_warning("FWD Lookup FAIL (XConn Invalid If): bi %u, invalid rx %u / peer %u (cfg: %u, %u)",
                       bi, rx_sw_if_index, peer_sw_if_index, cbsm->sw_if_index0, cbsm->sw_if_index1);
          *next = CBS_NEXT_DROP;
          return;
      }
      vnet_buffer (b)->sw_if_index[VLIB_TX] = peer_sw_if_index;

      *next = (peer_sw_if_index == cbsm->sw_if_index0) ? cbsm->output_next_index0 : cbsm->output_next_index1;

      if (PREDICT_FALSE(*next == CBS_NEXT_DROP || *next == (u32)~0)) {
         clib_warning("FWD Lookup FAIL (XConn Invalid Next): bi %u, rx %u, peer %u -> next_index %u (cfg next0: %u, next1: %u)",
                      bi, rx_sw_if_index, peer_sw_if_index, *next,
                      cbsm->output_next_index0, cbsm->output_next_index1);
         *next = CBS_NEXT_DROP;
      }

  } else { /* Output feature mode */
      // --- MODIFIED: Remove unused variable ---
      // u32 vec_len_check = vec_len(cbsm->output_next_index_by_sw_if_index);
      // --- END MODIFIED ---
      // clib_warning("FWD Lookup DBG: Enter Output Mode: bi %u, tx_sw_if %u, vec_len %u", // Removed stored_next_check
      //               bi, tx_sw_if_index, vec_len_check);

      if (PREDICT_FALSE(tx_sw_if_index == (u32)~0)) {
          clib_warning("FWD Lookup FAIL (Output Invalid TX If): bi %u, invalid tx_sw_if_index %u", bi, tx_sw_if_index);
          *next = CBS_NEXT_DROP;
          return;
      }

      u32 vec_len_safe = vec_len(cbsm->output_next_index_by_sw_if_index);
      u32 potential_next = (u32)~0;

      if (tx_sw_if_index < vec_len_safe) {
           potential_next = cbsm->output_next_index_by_sw_if_index[tx_sw_if_index];
      }

      if (PREDICT_TRUE(potential_next != (u32)~0)) {
          *next = potential_next;
          // clib_warning("FWD Lookup OK (Output): bi %u, tx_sw_if %u -> next_index %u",
          //              bi, tx_sw_if_index, *next);
      } else {
          clib_warning("FWD Lookup FAIL (Output Not Found/Invalid): bi %u, tx_sw_if %u not found or invalid next in vec (len %u, next_val %u)",
                       bi, tx_sw_if_index, vec_len_safe, potential_next);
          *next = CBS_NEXT_DROP;
      }
  }

  if (PREDICT_FALSE(*next == (u32)~0)) {
       clib_warning("FWD Lookup FAIL (Final Check): bi %u for tx_sw_if %u resulted in next ~0", bi, tx_sw_if_index);
      *next = CBS_NEXT_DROP;
  }
}

/** @brief Processes a single buffer: buffer to wheel or drop. */
always_inline void
cbs_dispatch_buffer (vlib_main_t * vm, vlib_node_runtime_t * node,
                     cbs_main_t * cbsm, cbs_wheel_t * wp, vlib_buffer_t * b,
                     u32 bi, cbs_node_ctx_t * ctx, u8 is_cross_connect)
{
    if (PREDICT_FALSE(wp->cursize >= wp->wheel_size)) {
        ctx->drop[0] = bi;
        ctx->drop++;
        cbs_add_trace(vm, node, b, CBS_TRACE_ACTION_DROP_WHEEL_FULL, CBS_NEXT_DROP);
        return;
    }

    cbs_wheel_entry_t *e = &wp->entries[wp->tail];
    wp->tail = (wp->tail + 1) % wp->wheel_size;
    wp->cursize++;
    ctx->n_buffered++;

    cbs_buffer_fwd_lookup(cbsm, b, &e->output_next_index, is_cross_connect);

    e->buffer_index = bi;
    e->rx_sw_if_index = vnet_buffer(b)->sw_if_index[VLIB_RX];
    e->tx_sw_if_index = vnet_buffer(b)->sw_if_index[VLIB_TX];

    cbs_add_trace(vm, node, b, CBS_TRACE_ACTION_BUFFER, e->output_next_index);
}

/* --- Definition of cbs_add_trace --- */
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
       t->tx_sw_if_index = vnet_buffer(b)->sw_if_index[VLIB_TX];
       t->trace_action = trace_action;
       t->calculated_next_index = calculated_next_index;
     }
}


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
    u32 drops[VLIB_FRAME_SIZE];
    cbs_node_ctx_t ctx;

    from = vlib_frame_vector_args (frame);
    n_left_from = frame->n_vectors;
    vlib_get_buffers (vm, from, bufs, n_left_from);
    b = bufs;

    if (PREDICT_FALSE(!cbsm->is_configured || thread_index >= vec_len(cbsm->wheel_by_thread) || !(wp = cbsm->wheel_by_thread[thread_index]))) {
         u16 *next_indices = vlib_node_get_runtime_data (vm, node->node_index);
         if (PREDICT_FALSE(!next_indices)) {
             vlib_buffer_free (vm, from, frame->n_vectors);
             vlib_node_increment_counter(vm, node->node_index, CBS_ERROR_DROPPED_LOOKUP_FAIL, frame->n_vectors);
             // clib_warning("Enqueue Fallback: Dropped %u packets (no next node data)", frame->n_vectors);
             return frame->n_vectors;
         }
        vlib_buffer_enqueue_to_next(vm, node, from, next_indices, frame->n_vectors);
        vlib_node_increment_counter (vm, node->node_index,
                                     cbsm->is_configured ? CBS_ERROR_NO_WHEEL : CBS_ERROR_NOT_CONFIGURED,
                                     frame->n_vectors);
        // clib_warning("Enqueue Fallback: Forwarded %u packets directly (%s)", frame->n_vectors,
        //                 cbsm->is_configured ? "no wheel for thread" : "not configured");
        return frame->n_vectors;
    }

    ctx.drop = drops;
    ctx.n_buffered = 0;

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
    while (n_left_from > 0) {
        cbs_dispatch_buffer (vm, node, cbsm, wp, b[0], from[0], &ctx, is_cross_connect);
        b += 1; from += 1; n_left_from -= 1;
    }

    u32 n_dropped_total = ctx.drop - drops;
    if (PREDICT_FALSE(n_dropped_total > 0)) {
        vlib_buffer_free (vm, drops, n_dropped_total);

        u32 n_wheel_full_drops = n_dropped_total;
        if (n_wheel_full_drops > 0) {
             vlib_node_increment_counter (vm, node->node_index, CBS_ERROR_DROPPED_WHEEL_FULL, n_wheel_full_drops);
        }
         if (n_dropped_total > 0) {
             clib_warning("Enqueue node %s: Dropped %u packets (WheelFull: %u)",
                          vlib_get_node(vm, node->node_index)->name, n_dropped_total, n_wheel_full_drops);
         }
    }

   if (ctx.n_buffered > 0) {
      vlib_node_increment_counter (vm, node->node_index, CBS_ERROR_BUFFERED, ctx.n_buffered);
   }

   return frame->n_vectors;
}


/* ... (Node Functions, Registration, Trace Formatting unchanged) ... */

VLIB_NODE_FN (cbs_cross_connect_node) (vlib_main_t * vm, vlib_node_runtime_t * node, vlib_frame_t * frame)
{
    return cbs_inline_fn (vm, node, frame, 1 /* is_cross_connect = true */);
}

VLIB_NODE_FN (cbs_output_feature_node) (vlib_main_t * vm, vlib_node_runtime_t * node, vlib_frame_t * frame)
{
    return cbs_inline_fn (vm, node, frame, 0 /* is_cross_connect = false */);
}

#ifndef CLIB_MARCH_VARIANT

static char * cbs_error_strings[] = {
#define _(sym,string) string,
  foreach_cbs_error
#undef _
};

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
      default: break;
  }

  s = format (s, "CBS_ENQ (bi %u): %s rx_sw %u tx_sw %u next_idx %u",
              t->buffer_index, action_str, t->rx_sw_if_index, t->tx_sw_if_index,
              t->calculated_next_index);
  return s;
}

vlib_node_registration_t cbs_cross_connect_node;
vlib_node_registration_t cbs_output_feature_node;

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
  },
};

#endif // CLIB_MARCH_VARIANT

/* ... */