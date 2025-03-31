/*
 * cbs.c - VPP CBS Plugin main implementation
 * Pure CBS shaper. No Loss/Reorder simulation.
 * Copyright (c) 2024 Your Org <your.email@example.com> // Placeholder
 * Based on nsim.c, Copyright (c) Cisco and/or its affiliates.
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#include <vnet/vnet.h>
#include <vnet/plugin/plugin.h>
#include <vnet/feature/feature.h>
#include <cbs/cbs.h>

#include <vlibapi/api.h>
#include <vlibmemory/api.h>
#include <vpp/app/version.h>
#include <vppinfra/format.h>
#include <vppinfra/byte_order.h>
#include <vppinfra/mem.h>
#include <vnet/api_errno.h> // For VNET_API_ERROR_* constants
#include <vppinfra/cpu.h>
#include <vlib/node_funcs.h>
#include <vppinfra/error.h> // Included for potential clib_error usage (though removed now)

#include <cbs/cbs.api_enum.h>
#include <cbs/cbs.api_types.h>

// Prototype for function in auto-generated cbs.api.c
static u16 setup_message_id_table (void);

#define REPLY_MSG_ID_BASE cbs_main.msg_id_base
#include <vlibapi/api_helper_macros.h>

cbs_main_t cbs_main;

// --- Forward declarations for static functions ---
static clib_error_t * cbs_init (vlib_main_t * vm);
static int cbs_configure_internal (cbs_main_t * cbsm, f64 port_rate_bps, f64 idleslope_kbps,
                                   f64 hicredit_bytes, f64 locredit_bytes,
                                   f64 bandwidth_bps_hint, u32 packet_size);
static cbs_wheel_t* cbs_wheel_alloc (cbs_main_t *cbsm, u32 thread_index);
static void cbs_wheel_free(cbs_main_t *cbsm, cbs_wheel_t *wp);

// CLI and API handlers (declarations needed if used before definition within #ifndef block)
#ifndef CLIB_MARCH_VARIANT
static clib_error_t * set_cbs_command_fn (vlib_main_t * vm, unformat_input_t * input, vlib_cli_command_t * cmd);
static clib_error_t * show_cbs_command_fn (vlib_main_t * vm, unformat_input_t * input, vlib_cli_command_t * cmd);
static clib_error_t * cbs_cross_connect_enable_disable_command_fn (vlib_main_t * vm, unformat_input_t * input, vlib_cli_command_t * cmd);
static clib_error_t * cbs_output_feature_enable_disable_command_fn (vlib_main_t * vm, unformat_input_t * input, vlib_cli_command_t * cmd);
static uword unformat_cbs_rate (unformat_input_t * input, va_list * args);
static uword unformat_cbs_slope (unformat_input_t * input, va_list * args);
static u8 * format_cbs_rate (u8 *s, va_list *args);
static u8 * format_cbs_slope (u8 *s, va_list *args);
static u8 * format_cbs_config (u8 * s, va_list * args);
static void vl_api_cbs_cross_connect_enable_disable_t_handler (vl_api_cbs_cross_connect_enable_disable_t * mp);
static void vl_api_cbs_output_feature_enable_disable_t_handler (vl_api_cbs_output_feature_enable_disable_t * mp);
static void vl_api_cbs_configure_t_handler (vl_api_cbs_configure_t * mp);
#endif // CLIB_MARCH_VARIANT


// --- Enable/Disable Functions ---
int
cbs_cross_connect_enable_disable (cbs_main_t * cbsm, u32 sw_if_index0,
				   u32 sw_if_index1, int enable_disable)
{
  vnet_sw_interface_t *sw0, *sw1;
  vnet_hw_interface_t *hw0, *hw1;
  vlib_main_t * vm = cbsm->vlib_main;
  vnet_main_t *vnm = cbsm->vnet_main;
  int rv = 0;

  if (PREDICT_FALSE(cbsm->is_configured == 0 && enable_disable))
    return VNET_API_ERROR_FEATURE_DISABLED;

  if (!vnet_sw_if_index_is_api_valid(sw_if_index0)) return VNET_API_ERROR_INVALID_SW_IF_INDEX;
  if (!vnet_sw_if_index_is_api_valid(sw_if_index1)) return VNET_API_ERROR_INVALID_SW_IF_INDEX_2;

  sw0 = vnet_get_sw_interface (vnm, sw_if_index0);
  sw1 = vnet_get_sw_interface (vnm, sw_if_index1);

  if (sw0->type != VNET_SW_INTERFACE_TYPE_HARDWARE) return VNET_API_ERROR_INVALID_INTERFACE;
  if (sw1->type != VNET_SW_INTERFACE_TYPE_HARDWARE) return VNET_API_ERROR_INVALID_INTERFACE;

  hw0 = vnet_get_hw_interface (vnm, sw_if_index0);
  hw1 = vnet_get_hw_interface (vnm, sw_if_index1);
  // Safety check for hw pointers
  if (PREDICT_FALSE(!hw0 || !hw1)) {
      // clib_warning is okay here, or return error
      clib_warning("CBS XCONN Enable: Failed to get hw interface struct for sw_if %u or %u", sw_if_index0, sw_if_index1);
      return VNET_API_ERROR_INVALID_INTERFACE;
  }


  if (enable_disable) {
      cbsm->output_next_index0 = vlib_node_add_next (vm, cbs_input_node.index, hw0->output_node_index);
      cbsm->output_next_index1 = vlib_node_add_next (vm, cbs_input_node.index, hw1->output_node_index);

      #if VLIB_DEBUG > 0 // Original debug log
      clib_warning("CBS Xconn Enable: Added next for sw_if %u -> %u (input node %u, hw node %u)",
                   sw_if_index0, cbsm->output_next_index0, cbs_input_node.index, hw0->output_node_index);
      clib_warning("CBS Xconn Enable: Added next for sw_if %u -> %u (input node %u, hw node %u)",
                   sw_if_index1, cbsm->output_next_index1, cbs_input_node.index, hw1->output_node_index);
      #endif

      // Original check for failure (add_next returns ~0 on failure)
      if (PREDICT_FALSE(cbsm->output_next_index0 == ~0 || cbsm->output_next_index1 == ~0)) {
          // Using clib_error here is acceptable as it indicates a setup failure
          clib_error("CBS Xconn Enable: FAILED to add next node arcs!");
          // Potentially clean up partially added arcs and return error
          // For simplicity now, just log the error.
          // return VNET_API_ERROR_UNSPECIFIED; // Example error
      }
  } else {
      cbsm->output_next_index0 = ~0;
      cbsm->output_next_index1 = ~0;
  }

  cbsm->sw_if_index0 = enable_disable ? sw_if_index0 : ~0;
  cbsm->sw_if_index1 = enable_disable ? sw_if_index1 : ~0;

  rv = vnet_feature_enable_disable ("device-input", "cbs-cross-connect",
			                           sw_if_index0, enable_disable, 0, 0);
  if (rv != 0) return rv;

  rv = vnet_feature_enable_disable ("device-input", "cbs-cross-connect",
			                           sw_if_index1, enable_disable, 0, 0);
  if (rv != 0) {
     vnet_feature_enable_disable("device-input", "cbs-cross-connect", sw_if_index0, 0, 0, 0);
     return rv;
   }

  return rv;
}

int
cbs_output_feature_enable_disable (cbs_main_t * cbsm, u32 sw_if_index,
				    int enable_disable)
{
  vnet_sw_interface_t *sw;
  vnet_hw_interface_t *hw;
  vlib_main_t * vm = cbsm->vlib_main;
  vnet_main_t *vnm = cbsm->vnet_main;
  int rv = 0;

  if (PREDICT_FALSE(cbsm->is_configured == 0 && enable_disable))
    return VNET_API_ERROR_FEATURE_DISABLED;

  if (!vnet_sw_if_index_is_api_valid(sw_if_index)) return VNET_API_ERROR_INVALID_SW_IF_INDEX;

  sw = vnet_get_sw_interface (vnm, sw_if_index);
  if (sw->type != VNET_SW_INTERFACE_TYPE_HARDWARE) return VNET_API_ERROR_INVALID_INTERFACE;

  hw = vnet_get_hw_interface (vnm, sw_if_index);
  // Safety check for hw pointer
  if (PREDICT_FALSE(!hw)) {
      // clib_warning is okay here, or return error
      clib_warning("CBS Output Enable: Failed to get hw interface for sw_if %u", sw_if_index);
      return VNET_API_ERROR_INVALID_INTERFACE;
  }

  if (enable_disable) {
      // Original code before adding explicit logging
      vec_validate_init_empty (cbsm->output_next_index_by_sw_if_index, sw_if_index, ~0);
      u32 added_next = vlib_node_add_next (vm, cbs_input_node.index, hw->output_node_index);
      cbsm->output_next_index_by_sw_if_index[sw_if_index] = added_next;

      #if VLIB_DEBUG > 0 // Original debug log
      clib_warning("CBS Output Enable: Added next for sw_if %u -> %u (input node %u, hw node %u)",
                   sw_if_index, added_next, cbs_input_node.index, hw->output_node_index);
      #endif

      // Original check for failure (add_next returns ~0 on failure)
      if (PREDICT_FALSE(added_next == ~0)) {
          // Using clib_error here is acceptable as it indicates a setup failure
          clib_error("CBS Output Enable: FAILED to add next node arc for sw_if %u!", sw_if_index);
          // return VNET_API_ERROR_UNSPECIFIED; // Example error
      }
  } else {
      if (sw_if_index < vec_len(cbsm->output_next_index_by_sw_if_index)) {
          cbsm->output_next_index_by_sw_if_index[sw_if_index] = ~0;
      }
  }

  rv = vnet_feature_enable_disable ("interface-output", "cbs-output-feature",
			                           sw_if_index, enable_disable, 0, 0);

  return rv;
}

// --- Wheel Allocation/Deallocation ---
/**
 * @brief Allocate and initialize a CBS wheel for a specific thread.
 * Uses the main thread's time for initial timestamp values.
 */
static cbs_wheel_t *
cbs_wheel_alloc (cbs_main_t *cbsm, u32 thread_index)
{
  cbs_wheel_t *wp;
  uword alloc_size = sizeof (cbs_wheel_t) +
                     cbsm->wheel_slots_per_wrk * sizeof (cbs_wheel_entry_t);

  wp = (cbs_wheel_t *) clib_mem_alloc_aligned (alloc_size, CLIB_CACHE_LINE_BYTES);
  if (PREDICT_FALSE(!wp)) return 0;
  clib_memset (wp, 0, alloc_size);

  wp->wheel_size = cbsm->wheel_slots_per_wrk;
  wp->cursize = 0;
  wp->head = 0;
  wp->tail = 0;
  wp->entries = (cbs_wheel_entry_t *) (wp + 1);

  wp->cbs_credits = 0.0;

  f64 main_thread_now = vlib_time_now(cbsm->vlib_main);
  wp->cbs_last_update_time = main_thread_now;
  wp->cbs_last_tx_finish_time = main_thread_now;

  return wp;
}

/** @brief Free memory allocated for a CBS wheel. */
static void cbs_wheel_free(cbs_main_t *cbsm, cbs_wheel_t *wp)
{
    if (wp) {
        clib_mem_free(wp);
    }
}

// --- Configuration Function ---
/** @brief Internal function to apply CBS configuration. */
static int
cbs_configure_internal (cbs_main_t * cbsm, f64 port_rate_bps,
			     f64 idleslope_kbps, f64 hicredit_bytes,
			     f64 locredit_bytes, f64 bandwidth_bps_hint,
			     u32 packet_size)
{
  u64 wheel_slots_per_wrk;
  int i;
  vlib_main_t *vm = cbsm->vlib_main;
  int n_threads = vlib_get_n_threads();
  f64 port_rate_bytes_sec, idleslope_bytes_sec, sendslope_bytes_sec;
  f64 effective_bandwidth_for_sizing;

  #if VLIB_DEBUG > 0
  clib_warning("CBS Configure Internal: port_rate=%.2f Gbps, idleslope=%.2f Kbps, hi=%.0f, lo=%.0f, hint=%.2f Mbps, pkt_size=%u",
               port_rate_bps / CBS_GBPS_TO_BPS, idleslope_kbps, hicredit_bytes, locredit_bytes, bandwidth_bps_hint / CBS_MBPS_TO_BPS, packet_size);
  #endif

  // --- Validate Parameters ---
  if (PREDICT_FALSE(port_rate_bps <= 0.0)) return VNET_API_ERROR_INVALID_VALUE;
  if (PREDICT_FALSE(idleslope_kbps <= 0.0)) return VNET_API_ERROR_INVALID_VALUE_2;
  if (PREDICT_FALSE(hicredit_bytes < locredit_bytes)) return VNET_API_ERROR_INVALID_VALUE_3;

  if (packet_size == 0) packet_size = CBS_DEFAULT_PACKET_SIZE;
  if (PREDICT_FALSE(packet_size < 64 || packet_size > 9000)) return VNET_API_ERROR_INVALID_VALUE_4;

  port_rate_bytes_sec = port_rate_bps / CBS_BITS_PER_BYTE;
  idleslope_bytes_sec = (idleslope_kbps * CBS_KBPS_TO_BPS) / CBS_BITS_PER_BYTE;
  sendslope_bytes_sec = idleslope_bytes_sec - port_rate_bytes_sec;

  // --- Cleanup existing configuration ---
  if (cbsm->is_configured) {
      clib_warning("CBS Configure: Re-configuring. Cleaning up previous state.");
      vlib_worker_thread_barrier_sync (vm);
      for (i = 0; i < vec_len (cbsm->wheel_by_thread); i++) {
          vlib_main_t *wrk_vm = vlib_get_main_by_index(i);
          if (wrk_vm && cbs_input_node.index != ~0) {
              vlib_node_set_state (wrk_vm, cbs_input_node.index, VLIB_NODE_STATE_DISABLED);
              #if VLIB_DEBUG > 0
              clib_warning("CBS Configure: Disabled polling for cbs-wheel on thread %d", i);
              #endif
          }
          if (cbsm->wheel_by_thread[i]) {
            cbs_wheel_free(cbsm, cbsm->wheel_by_thread[i]);
            cbsm->wheel_by_thread[i] = 0;
          }
      }
      vec_reset_length(cbsm->wheel_by_thread);
      vlib_worker_thread_barrier_release (vm);
      clib_warning("CBS Configure: Previous wheels freed and polling disabled.");
      cbsm->is_configured = 0;
  }

  // --- Store new configuration ---
  cbsm->cbs_port_rate = port_rate_bytes_sec;
  cbsm->cbs_idleslope = idleslope_bytes_sec;
  cbsm->cbs_sendslope = sendslope_bytes_sec;
  cbsm->cbs_hicredit = hicredit_bytes;
  cbsm->cbs_locredit = locredit_bytes;
  cbsm->packet_size = packet_size;

  effective_bandwidth_for_sizing = (bandwidth_bps_hint > 0) ? bandwidth_bps_hint : port_rate_bps;
  cbsm->configured_bandwidth = effective_bandwidth_for_sizing / CBS_BITS_PER_BYTE;

  // --- Calculate Wheel Size ---
  f64 buffer_time_target = 0.010;
  u64 total_buffer_bytes = (cbsm->cbs_port_rate * buffer_time_target);
  total_buffer_bytes = clib_max(total_buffer_bytes, (u64)cbsm->packet_size * 1024);

  u32 num_workers = vlib_num_workers();
  u64 per_worker_buffer_bytes = (num_workers > 0) ? (total_buffer_bytes / num_workers) : total_buffer_bytes;
  per_worker_buffer_bytes = clib_max(per_worker_buffer_bytes, (u64)cbsm->packet_size * 256);

  wheel_slots_per_wrk = per_worker_buffer_bytes / cbsm->packet_size;
  wheel_slots_per_wrk = clib_max(wheel_slots_per_wrk, (u64)CBS_MIN_WHEEL_SLOTS);
  wheel_slots_per_wrk++;
  cbsm->wheel_slots_per_wrk = wheel_slots_per_wrk;

  #if VLIB_DEBUG > 0
  clib_warning("CBS Configure: Calculated wheel size = %lu slots/worker (target %.3f s buffer)", cbsm->wheel_slots_per_wrk, buffer_time_target);
  #endif

  // --- Allocate Wheels ---
  vec_validate (cbsm->wheel_by_thread, n_threads - 1);
  #if VLIB_DEBUG > 0
  clib_warning("CBS Configure: Allocating wheels for %d threads (0 to %d)", n_threads, n_threads - 1);
  #endif
  for (i = 0; i < n_threads; i++) {
      cbsm->wheel_by_thread[i] = cbs_wheel_alloc (cbsm, i);
      if (PREDICT_FALSE(!cbsm->wheel_by_thread[i])) {
         clib_error("CBS Configure: ERROR - Wheel allocation failed for thread %d", i);
         for (int j = 0; j < i; j++) {
             if (cbsm->wheel_by_thread[j]) {
                cbs_wheel_free(cbsm, cbsm->wheel_by_thread[j]);
                cbsm->wheel_by_thread[j] = 0;
             }
         }
         vec_reset_length(cbsm->wheel_by_thread);
         return VNET_API_ERROR_UNSPECIFIED;
      }
  }
  #if VLIB_DEBUG > 0
  clib_warning("CBS Configure: Wheels allocated successfully.");
  #endif

  // --- Finalize and Enable Polling ---
  cbsm->is_configured = 1;
  #if VLIB_DEBUG > 0
  clib_warning("CBS Configure: Configuration complete.");
  #endif

  vlib_worker_thread_barrier_sync (vm);
  for (i = 0; i < n_threads; i++) {
      vlib_main_t *wrk_vm = vlib_get_main_by_index(i);
      if (wrk_vm) {
          if (PREDICT_TRUE(cbs_input_node.index != ~0)) {
             vlib_node_set_state (wrk_vm, cbs_input_node.index, VLIB_NODE_STATE_POLLING);
             #if VLIB_DEBUG > 0
             clib_warning("CBS Configure: Enabling polling for cbs-wheel on thread %u", i);
             #endif
          } else {
             clib_error("CBS Configure: ERROR - cbs_input_node index invalid, cannot enable polling on thread %u", i);
          }
        }
    }
  vlib_worker_thread_barrier_release (vm);

  return 0; // Success
}


/* --- Base Implementation Block (API/CLI Handlers, Init, etc.) --- */
#ifndef CLIB_MARCH_VARIANT

/* --- API Message Handlers (Defined before cbs.api.c include) --- */
static void vl_api_cbs_cross_connect_enable_disable_t_handler
  (vl_api_cbs_cross_connect_enable_disable_t * mp)
{
  vl_api_cbs_cross_connect_enable_disable_reply_t *rmp;
  cbs_main_t *cbsm = &cbs_main;
  int rv;
  u32 sw_if_index0 = clib_net_to_host_u32(mp->sw_if_index0);
  u32 sw_if_index1 = clib_net_to_host_u32(mp->sw_if_index1);

  if (!vnet_sw_if_index_is_api_valid (sw_if_index0)) { rv = VNET_API_ERROR_INVALID_SW_IF_INDEX; goto reply; }
  if (!vnet_sw_if_index_is_api_valid (sw_if_index1)) { rv = VNET_API_ERROR_INVALID_SW_IF_INDEX_2; goto reply; }

  rv = cbs_cross_connect_enable_disable (cbsm, sw_if_index0, sw_if_index1, (int) (mp->enable_disable));

reply:
  REPLY_MACRO (VL_API_CBS_CROSS_CONNECT_ENABLE_DISABLE_REPLY);
}

static void vl_api_cbs_output_feature_enable_disable_t_handler
  (vl_api_cbs_output_feature_enable_disable_t * mp)
{
  vl_api_cbs_output_feature_enable_disable_reply_t *rmp;
  cbs_main_t *cbsm = &cbs_main;
  int rv;

  VALIDATE_SW_IF_INDEX(mp);

  rv = cbs_output_feature_enable_disable (cbsm, clib_net_to_host_u32(mp->sw_if_index), (int) (mp->enable_disable));

BAD_SW_IF_INDEX_LABEL;
  REPLY_MACRO (VL_API_CBS_OUTPUT_FEATURE_ENABLE_DISABLE_REPLY);
}

static void
vl_api_cbs_configure_t_handler (vl_api_cbs_configure_t * mp)
{
  vl_api_cbs_configure_reply_t *rmp;
  cbs_main_t *cbsm = &cbs_main;
  f64 port_rate_bps, idleslope_kbps, bandwidth_bps_hint;
  f64 hicredit_bytes, locredit_bytes;
  u32 packet_size;
  int rv;

  port_rate_bps = (f64) clib_net_to_host_u64 (mp->port_rate_bps);
  idleslope_kbps = (f64) clib_net_to_host_u64 (mp->idleslope_kbps);
  hicredit_bytes = (f64) ((i32) clib_net_to_host_u32(mp->hicredit_bytes));
  locredit_bytes = (f64) ((i32) clib_net_to_host_u32(mp->locredit_bytes));
  packet_size = clib_net_to_host_u32 (mp->average_packet_size);
  bandwidth_bps_hint = (f64) clib_net_to_host_u64 (mp->bandwidth_in_bits_per_second);

  rv = cbs_configure_internal (cbsm, port_rate_bps, idleslope_kbps,
				    hicredit_bytes, locredit_bytes,
				    bandwidth_bps_hint, packet_size);

  REPLY_MACRO (VL_API_CBS_CONFIGURE_REPLY);
}


/* --- Plugin Initialization --- */
static clib_error_t *
cbs_init (vlib_main_t * vm)
{
  cbs_main_t *cbsm = &cbs_main;
  clib_error_t * error = 0;

  cbsm->vlib_main = vm;
  cbsm->vnet_main = vnet_get_main ();

  cbsm->sw_if_index0 = ~0;
  cbsm->sw_if_index1 = ~0;
  cbsm->output_next_index0 = ~0;
  cbsm->output_next_index1 = ~0;
  cbsm->is_configured = 0;
  cbsm->output_next_index_by_sw_if_index = 0;
  cbsm->wheel_by_thread = 0;

  cbsm->msg_id_base = setup_message_id_table ();

  cbsm->arc_index = vnet_get_feature_arc_index ("interface-output");
  if (cbsm->arc_index == (u16)~0) {
        error = clib_error_return (0, "Failed to get feature arc index for 'interface-output'");
        goto done;
  }

done:
  return error;
}

VLIB_INIT_FUNCTION (cbs_init);

/* --- Feature registrations --- */
VNET_FEATURE_INIT (cbs_cross_connect_feat, static) =
{
  .arc_name = "device-input",
  .node_name = "cbs-cross-connect",
  .runs_before = VNET_FEATURES ("ethernet-input"),
};

VNET_FEATURE_INIT (cbs_output_feature_feat, static) = {
  .arc_name = "interface-output",
  .node_name = "cbs-output-feature",
  .runs_before = VNET_FEATURES ("interface-output-arc-end"),
};

/* --- Plugin Registration --- */
VLIB_PLUGIN_REGISTER () =
{
  .version = VPP_BUILD_VER,
  .description = "Credit Based Shaper (CBS) Plugin",
};

/* --- CLI Formatters/Unformatters --- */
static uword
unformat_cbs_rate (unformat_input_t * input, va_list * args)
{
  f64 *result = va_arg (*args, f64 *); f64 tmp;
  if (unformat (input, "%f gbps", &tmp) || unformat (input, "%f gbit", &tmp)) *result = tmp * CBS_GBPS_TO_BPS;
  else if (unformat (input, "%f mbps", &tmp) || unformat (input, "%f mbit", &tmp)) *result = tmp * CBS_MBPS_TO_BPS;
  else if (unformat (input, "%f kbps", &tmp) || unformat (input, "%f kbit", &tmp)) *result = tmp * CBS_KBPS_TO_BPS;
  else if (unformat (input, "%f bps", &tmp) || unformat (input, "%f bit", &tmp)) *result = tmp;
  else return 0; return 1;
}
static uword
unformat_cbs_slope (unformat_input_t * input, va_list * args)
{
  f64 *result = va_arg (*args, f64 *); f64 tmp;
  if (unformat (input, "%f kbps", &tmp) || unformat (input, "%f kbit", &tmp)) *result = tmp;
  else return 0; return 1;
}

static u8 *
format_cbs_rate (u8 *s, va_list *args)
{
  f64 rate_bytes_sec = va_arg (*args, f64);
  f64 rate_bps = rate_bytes_sec * CBS_BITS_PER_BYTE;
  if (rate_bps >= CBS_GBPS_TO_BPS * 0.99) s = format (s, "%.2f Gbps", rate_bps / CBS_GBPS_TO_BPS);
  else if (rate_bps >= CBS_MBPS_TO_BPS * 0.99) s = format (s, "%.2f Mbps", rate_bps / CBS_MBPS_TO_BPS);
  else if (rate_bps >= CBS_KBPS_TO_BPS * 0.99) s = format (s, "%.2f Kbps", rate_bps / CBS_KBPS_TO_BPS);
  else s = format (s, "%.2f bps", rate_bps);
  return s;
}
static u8 *
format_cbs_slope (u8 *s, va_list *args)
{
  f64 slope_bytes_sec = va_arg (*args, f64);
  f64 slope_kbps = (slope_bytes_sec * CBS_BITS_PER_BYTE) / CBS_KBPS_TO_BPS;
  s = format (s, "%.2f Kbps", slope_kbps);
  return s;
}

static u8 *
format_cbs_config (u8 * s, va_list * args)
{
   cbs_main_t *cbsm = &cbs_main;
   int verbose __attribute__((unused)) = va_arg (*args, int);

   s = format (s, "CBS Configuration:\n");
   s = format (s, "  Port Rate:       %U\n", format_cbs_rate, cbsm->cbs_port_rate);
   s = format (s, "  Idle Slope:      %U\n", format_cbs_slope, cbsm->cbs_idleslope);
   // Use format_cbs_rate for sendslope, which is bytes/sec converted to rate
   s = format (s, "  Send Slope:      %U/sec (calculated)\n", format_cbs_rate, cbsm->cbs_sendslope);
   s = format (s, "  HiCredit:        %.0f bytes\n", cbsm->cbs_hicredit);
   s = format (s, "  LoCredit:        %.0f bytes\n", cbsm->cbs_locredit);

   s = format (s, "Internal Sizing:\n");
   s = format (s, "  Avg Packet Size: %u bytes\n", cbsm->packet_size);
   s = format (s, "  Bandwidth Hint:  %U (for wheel sizing)\n", format_cbs_rate, cbsm->configured_bandwidth);
   s = format (s, "  Wheel Size:      %u slots/worker\n", cbsm->wheel_slots_per_wrk);

   s = format (s, "\nEnabled Interfaces:\n");
    if (cbsm->sw_if_index0 != ~0) {
         s = format (s, "  Cross-connect: %U <--> %U\n", format_vnet_sw_if_index_name, cbsm->vnet_main, cbsm->sw_if_index0, format_vnet_sw_if_index_name, cbsm->vnet_main, cbsm->sw_if_index1);
    } else {
        int output_feature_enabled = 0;
        u32 i;
        for (i = 0; i < vec_len (cbsm->output_next_index_by_sw_if_index); i++) {
          if (cbsm->output_next_index_by_sw_if_index[i] != ~0) {
            if (!output_feature_enabled) {
              s = format (s, "  Output Feature on:\n");
              output_feature_enabled = 1;
            }
            s = format (s, "    %U\n", format_vnet_sw_if_index_name, cbsm->vnet_main, i);
          }
        }
        if (!output_feature_enabled) {
            s = format(s, "  None\n");
        }
    }

   return s;
}

/* --- CLI Command Handlers --- */
static clib_error_t *
cbs_cross_connect_enable_disable_command_fn (vlib_main_t * vm,
					      unformat_input_t * input,
					      vlib_cli_command_t * cmd)
{
   cbs_main_t *cbsm = &cbs_main;
   unformat_input_t _line_input, *line_input = &_line_input;
   u32 sw_if_index0 = ~0;
   u32 sw_if_index1 = ~0;
   int enable_disable = 1;
   u32 tmp;
   int rv;
   clib_error_t * error = 0;

   if (!unformat_user (input, unformat_line_input, line_input)) return 0;

   while (unformat_check_input (line_input) != UNFORMAT_END_OF_INPUT) {
       if (unformat (line_input, "disable")) enable_disable = 0;
       else if (unformat (line_input, "%U", unformat_vnet_sw_interface, cbsm->vnet_main, &tmp)) {
           if (sw_if_index0 == ~0) sw_if_index0 = tmp;
           else if (sw_if_index1 == ~0) sw_if_index1 = tmp;
           else { error = clib_error_return (0, "Please specify only two interfaces"); goto done;}
         }
       else if (unformat (line_input, "sw_if_index %u", &tmp)) {
           if (sw_if_index0 == ~0) sw_if_index0 = tmp;
           else if (sw_if_index1 == ~0) sw_if_index1 = tmp;
           else { error = clib_error_return (0, "Please specify only two interfaces"); goto done;}
        }
       else { error = clib_error_return (0, "unknown input '%U'", format_unformat_error, line_input); goto done; }
     }

   if (sw_if_index0 == ~0 || sw_if_index1 == ~0) { error = clib_error_return (0, "Please specify two interfaces"); goto done; }

   rv = cbs_cross_connect_enable_disable (cbsm, sw_if_index0, sw_if_index1, enable_disable);

   switch (rv) {
     case 0: break;
     case VNET_API_ERROR_FEATURE_DISABLED: error = clib_error_return (0, "CBS not configured, please 'set cbs ...' first"); break;
     case VNET_API_ERROR_INVALID_SW_IF_INDEX:
     case VNET_API_ERROR_INVALID_SW_IF_INDEX_2: error = clib_error_return(0, "Invalid software interface index"); break;
     case VNET_API_ERROR_INVALID_INTERFACE: error = clib_error_return (0, "Invalid interface (must be hardware)"); break;
     default:
         error = clib_error_return (0, "cbs_cross_connect_enable_disable failed: %d", rv);
         break;
   }

 done:
   unformat_free (line_input);
   return error;
}

static clib_error_t *
cbs_output_feature_enable_disable_command_fn (vlib_main_t * vm,
					       unformat_input_t * input,
					       vlib_cli_command_t * cmd)
{
    cbs_main_t *cbsm = &cbs_main;
    unformat_input_t _line_input, *line_input = &_line_input;
    u32 sw_if_index = ~0;
    int enable_disable = 1;
    int rv;
    clib_error_t * error = 0;

    if (!unformat_user (input, unformat_line_input, line_input)) return 0;

    while (unformat_check_input (line_input) != UNFORMAT_END_OF_INPUT) {
        if (unformat (line_input, "disable")) enable_disable = 0;
        else if (unformat (line_input, "%U", unformat_vnet_sw_interface, cbsm->vnet_main, &sw_if_index)) ;
        else if (unformat (line_input, "sw_if_index %u", &sw_if_index));
        else { error = clib_error_return (0, "unknown input `%U'", format_unformat_error, line_input); goto done; }
      }

    if (sw_if_index == ~0) { error = clib_error_return (0, "Please specify one interface"); goto done;}

    rv = cbs_output_feature_enable_disable (cbsm, sw_if_index, enable_disable);

    switch (rv) {
      case 0: break;
      case VNET_API_ERROR_FEATURE_DISABLED: error = clib_error_return (0, "CBS not configured, please 'set cbs ...' first"); break;
      case VNET_API_ERROR_INVALID_SW_IF_INDEX: error = clib_error_return(0, "Invalid software interface index"); break;
      case VNET_API_ERROR_INVALID_INTERFACE: error = clib_error_return(0, "Invalid interface (must be hardware)"); break;
      default:
          error = clib_error_return (0, "cbs_output_feature_enable_disable failed: %d", rv);
          break;
      }

  done:
    unformat_free (line_input);
    return error;
}

static clib_error_t *
set_cbs_command_fn (vlib_main_t * vm, unformat_input_t * input, vlib_cli_command_t * cmd)
{
    cbs_main_t *cbsm = &cbs_main;
    f64 port_rate_bps = 0.0, idleslope_kbps = 0.0, bandwidth_bps_hint = 0.0;
    f64 hicredit_bytes = 0.0, locredit_bytes = 0.0;
    u32 packet_size = 0;
    int rv;
    clib_error_t * error = 0;
    int port_rate_set = 0, idleslope_set = 0, hicredit_set = 0, locredit_set = 0;

    while (unformat_check_input (input) != UNFORMAT_END_OF_INPUT) {
        if (unformat (input, "port_rate %U", unformat_cbs_rate, &port_rate_bps)) port_rate_set = 1;
        else if (unformat (input, "idleslope %U", unformat_cbs_slope, &idleslope_kbps)) idleslope_set = 1;
        else if (unformat (input, "hicredit %f", &hicredit_bytes)) hicredit_set = 1;
        else if (unformat (input, "locredit %f", &locredit_bytes)) locredit_set = 1;
        else if (unformat (input, "bandwidth %U", unformat_cbs_rate, &bandwidth_bps_hint));
        else if (unformat (input, "packet-size %u", &packet_size));
        else { error = clib_error_return (0, "unknown input '%U'", format_unformat_error, input); goto done; }
      }

    if (!port_rate_set || !idleslope_set || !hicredit_set || !locredit_set) {
        error = clib_error_return (0, "Mandatory parameters missing. Required: port_rate, idleslope, hicredit, locredit");
        goto done;
    }

    rv = cbs_configure_internal (cbsm, port_rate_bps, idleslope_kbps, hicredit_bytes, locredit_bytes, bandwidth_bps_hint, packet_size);

    switch (rv) {
      case 0:
          vlib_cli_output (vm, "%U", format_cbs_config, 0);
          break;
      case VNET_API_ERROR_INVALID_VALUE: error = clib_error_return (0, "Invalid port_rate (must be > 0)"); break;
      case VNET_API_ERROR_INVALID_VALUE_2: error = clib_error_return (0, "Invalid idleslope (must be > 0)"); break;
      case VNET_API_ERROR_INVALID_VALUE_3: error = clib_error_return (0, "Invalid credits (hicredit must be >= locredit)"); break;
      case VNET_API_ERROR_INVALID_VALUE_4: error = clib_error_return (0, "Invalid packet size (must be 64-9000, or 0 for default)"); break;
      case VNET_API_ERROR_UNSPECIFIED: error = clib_error_return(0, "Configuration failed (e.g., memory allocation error)"); break;
      default:
          error = clib_error_return (0, "cbs_configure_internal failed: %d", rv);
          break;
    }

  done:
    return error;
}

static clib_error_t *
show_cbs_command_fn (vlib_main_t * vm, unformat_input_t * input, vlib_cli_command_t * cmd)
{
    cbs_main_t *cbsm = &cbs_main;
    int verbose = 0;
    clib_error_t * error = 0;

    if (unformat (input, "verbose")) verbose = 1;
    else if (unformat_check_input(input) != UNFORMAT_END_OF_INPUT) {
       error = clib_error_return (0, "unknown input '%U'", format_unformat_error, input);
       goto done;
    }

    if (PREDICT_FALSE(cbsm->is_configured == 0)) {
      error = clib_error_return (0, "CBS not configured. Use 'set cbs ...' first.");
      goto done;
    }

    vlib_cli_output (vm, "%U", format_cbs_config, verbose);

  done:
    return error;
}


/* --- CLI Command Registrations --- */
VLIB_CLI_COMMAND (set_cbs_command, static) =
{
  .path = "set cbs",
  .short_help = "set cbs port_rate <rate> idleslope <kbps> hicredit <bytes> locredit <bytes> [bandwidth <rate>] [packet-size <n>]",
  .function = set_cbs_command_fn,
};

VLIB_CLI_COMMAND (show_cbs_command, static) =
{
  .path = "show cbs",
  .short_help = "Display CBS configuration and state [verbose]",
  .function = show_cbs_command_fn,
};

VLIB_CLI_COMMAND (cbs_enable_disable_command, static) =
{
  .path = "cbs cross-connect enable-disable",
  .short_help = "cbs cross-connect enable-disable <intfc1> <intfc2> [disable]",
  .function = cbs_cross_connect_enable_disable_command_fn,
};

VLIB_CLI_COMMAND (cbs_output_feature_enable_disable_command, static) =
{
  .path = "cbs output-feature enable-disable",
  .short_help = "cbs output-feature enable-disable <interface> [disable]",
  .function = cbs_output_feature_enable_disable_command_fn,
};

#endif // CLIB_MARCH_VARIANT

/* Include the auto-generated API C file *AFTER* handler and CLI definitions */
#include <cbs/cbs.api.c>

/*
 * fd.io coding-style-patch-verification: ON
 * Local Variables:
 * eval: (c-set-style "gnu")
 * End:
 */