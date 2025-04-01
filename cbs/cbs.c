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
#include <vppinfra/error.h>
#include <vlib/log.h>       // For logging

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
  vlib_log_class_t log_class = cbsm->log_class; // Get log class
  int rv = 0;
  u32 added_next0 = (u32)~0, added_next1 = (u32)~0; // Track added indices

  if (PREDICT_FALSE(cbsm->is_configured == 0 && enable_disable))
    return VNET_API_ERROR_FEATURE_DISABLED;

  if (!vnet_sw_if_index_is_api_valid(sw_if_index0)) return VNET_API_ERROR_INVALID_SW_IF_INDEX;
  if (!vnet_sw_if_index_is_api_valid(sw_if_index1)) return VNET_API_ERROR_INVALID_SW_IF_INDEX_2;

  sw0 = vnet_get_sw_interface (vnm, sw_if_index0);
  sw1 = vnet_get_sw_interface (vnm, sw_if_index1);

  // Only support hardware interfaces for cross-connect for simplicity now
  if (sw0->type != VNET_SW_INTERFACE_TYPE_HARDWARE) return VNET_API_ERROR_INVALID_INTERFACE;
  if (sw1->type != VNET_SW_INTERFACE_TYPE_HARDWARE) return VNET_API_ERROR_INVALID_INTERFACE;


  hw0 = vnet_get_hw_interface (vnm, sw_if_index0);
  hw1 = vnet_get_hw_interface (vnm, sw_if_index1);
  // Safety check for hw pointers (Keep this basic check)
  if (PREDICT_FALSE(!hw0 || !hw1)) {
      vlib_log_err(log_class, "XCONN Enable: Failed to get hw interface struct for sw_if %u or %u", sw_if_index0, sw_if_index1);
      return VNET_API_ERROR_INVALID_INTERFACE;
  }


  if (enable_disable) {
      u32 target_node_index0 = hw0->output_node_index;
      u32 target_node_index1 = hw1->output_node_index;
      added_next0 = vlib_node_add_next (vm, cbs_input_node.index, target_node_index0);
      added_next1 = vlib_node_add_next (vm, cbs_input_node.index, target_node_index1);
      cbsm->output_next_index0 = added_next0; // Store result
      cbsm->output_next_index1 = added_next1; // Store result

      vlib_log_debug(log_class, "Xconn Enable: Trying to add next for sw_if %u: '%U' (%u) -> '%U' (%u), result_next_index %u",
                     sw_if_index0,
                     format_vlib_node_name, vm, cbs_input_node.index, cbs_input_node.index,
                     format_vlib_node_name, vm, target_node_index0, target_node_index0,
                     added_next0);
      vlib_log_debug(log_class, "Xconn Enable: Trying to add next for sw_if %u: '%U' (%u) -> '%U' (%u), result_next_index %u",
                     sw_if_index1,
                     format_vlib_node_name, vm, cbs_input_node.index, cbs_input_node.index,
                     format_vlib_node_name, vm, target_node_index1, target_node_index1,
                     added_next1);

      // --- REMOVED vlib_node_add_next failure check (to match nsim) ---
  } else {
      cbsm->output_next_index0 = ~0;
      cbsm->output_next_index1 = ~0;
      vlib_log_debug(log_class, "Xconn Disable: Cleared next indices");
  }

  cbsm->sw_if_index0 = enable_disable ? sw_if_index0 : ~0;
  cbsm->sw_if_index1 = enable_disable ? sw_if_index1 : ~0;

  rv = vnet_feature_enable_disable ("device-input", "cbs-cross-connect",
			                           sw_if_index0, enable_disable, 0, 0);
  // --- REMOVED feature enable failure check/rollback (to match nsim) ---
  // If the first one failed, return immediately
  if (rv != 0) return rv;


  rv = vnet_feature_enable_disable ("device-input", "cbs-cross-connect",
			                           sw_if_index1, enable_disable, 0, 0);
  // --- REMOVED feature enable failure check/rollback (to match nsim) ---

  return rv; // Return the result of the second call directly
}

int
cbs_output_feature_enable_disable (cbs_main_t * cbsm, u32 sw_if_index,
				    int enable_disable)
{
  vnet_sw_interface_t *sw;
  vnet_hw_interface_t *hw;
  vlib_main_t * vm = cbsm->vlib_main;
  vnet_main_t *vnm = cbsm->vnet_main;
  vlib_log_class_t log_class = cbsm->log_class; // Get log class
  int rv = 0;
  u32 added_next = (u32)~0; // Initialize added_next

  if (PREDICT_FALSE(cbsm->is_configured == 0 && enable_disable))
    return VNET_API_ERROR_FEATURE_DISABLED;

  if (!vnet_sw_if_index_is_api_valid(sw_if_index)) return VNET_API_ERROR_INVALID_SW_IF_INDEX;

  sw = vnet_get_sw_interface (vnm, sw_if_index);
  // --- MODIFIED: Check for HARDWARE type only, like nsim ---
  if (sw->type != VNET_SW_INTERFACE_TYPE_HARDWARE)
      return VNET_API_ERROR_INVALID_INTERFACE;
  // --- END MODIFICATION ---


  // --- MODIFIED: Use vnet_get_hw_interface like nsim ---
  hw = vnet_get_hw_interface (vnm, sw_if_index);
  // --- END MODIFICATION ---
  // Safety check for hw pointer
  if (PREDICT_FALSE(!hw)) {
      vlib_log_err(log_class, "Output Enable: Failed to get hardware interface for sw_if %u", sw_if_index);
      return VNET_API_ERROR_INVALID_INTERFACE;
  }

  if (enable_disable) {
      vec_validate_init_empty (cbsm->output_next_index_by_sw_if_index, sw_if_index, ~0);
      u32 target_node_index = hw->output_node_index;
      added_next = vlib_node_add_next (vm, cbs_input_node.index, target_node_index);
      cbsm->output_next_index_by_sw_if_index[sw_if_index] = added_next; // Store result

      vlib_log_debug(log_class, "Output Enable DBG: Stored next_index %u for sw_if %u in output_next_index_by_sw_if_index",
                   added_next, sw_if_index);
      vlib_log_debug(log_class, "Output Enable: Trying to add next for sw_if %u: '%U' (%u) -> '%U' (%u), result_next_index %u",
                   sw_if_index,
                   format_vlib_node_name, vm, cbs_input_node.index, cbs_input_node.index,
                   format_vlib_node_name, vm, target_node_index, target_node_index,
                   added_next);

       // --- REMOVED vlib_node_add_next failure check (to match nsim) ---

  } else {
      // --- MODIFIED: Remove explicit clear of next_index on disable to match nsim ---
      vlib_log_debug(log_class, "Output Disable: No explicit clear for next index (matching nsim)");
  }

  // Enable/Disable the feature on the interface-output arc
  rv = vnet_feature_enable_disable ("interface-output", "cbs-output-feature",
			                           sw_if_index, enable_disable, 0, 0);

  // --- REMOVED feature enable failure check/rollback (to match nsim) ---

  return rv; // Return result directly
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

  wp->cbs_credits = 0.0; // Initialize credits

  // --- REVERTED ---
  // Always use the main thread's time context when called from configure
  // This is because this function is called from the main thread during configuration.
  f64 now = vlib_time_now(cbsm->vlib_main);
  // --- REVERTED END ---

  wp->cbs_last_update_time = now;
  wp->cbs_last_tx_finish_time = now;

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
  vlib_log_class_t log_class = cbsm->log_class; // Get log class
  int n_threads = vlib_get_n_threads();
  f64 port_rate_bytes_sec, idleslope_bytes_sec, sendslope_bytes_sec;
  f64 effective_bandwidth_for_sizing;

  vlib_log_notice(log_class, "Configure Internal: port_rate=%.2f Gbps, idleslope=%.2f Kbps, hi=%.0f, lo=%.0f, hint=%.2f Mbps, pkt_size=%u",
                  port_rate_bps / CBS_GBPS_TO_BPS, idleslope_kbps, hicredit_bytes, locredit_bytes, bandwidth_bps_hint / CBS_MBPS_TO_BPS, packet_size);

  // --- Validate Parameters ---
  if (PREDICT_FALSE(port_rate_bps <= 0.0)) return VNET_API_ERROR_INVALID_VALUE;
  if (PREDICT_FALSE(idleslope_kbps < 0.0)) return VNET_API_ERROR_INVALID_VALUE_2; // Allow 0 idleslope? Standard says > 0.
  if (PREDICT_FALSE(hicredit_bytes < locredit_bytes)) return VNET_API_ERROR_INVALID_VALUE_3;

  if (packet_size == 0) packet_size = CBS_DEFAULT_PACKET_SIZE;
  if (PREDICT_FALSE(packet_size < 64 || packet_size > 9000)) return VNET_API_ERROR_INVALID_VALUE_4;

  port_rate_bytes_sec = port_rate_bps / CBS_BITS_PER_BYTE;
  idleslope_bytes_sec = (idleslope_kbps * CBS_KBPS_TO_BPS) / CBS_BITS_PER_BYTE;
  sendslope_bytes_sec = idleslope_bytes_sec - port_rate_bytes_sec;

  // --- Cleanup existing configuration ---
  if (cbsm->is_configured) {
      vlib_log_notice(log_class, "Configure: Re-configuring. Cleaning up previous state.");
      vlib_worker_thread_barrier_sync (vm);
      for (i = 0; i < vec_len (cbsm->wheel_by_thread); i++) {
          vlib_main_t *wrk_vm = vlib_get_main_by_index(i);
          if (wrk_vm && cbs_input_node.index != (u32)~0) { // Check node index validity
              vlib_node_set_state (wrk_vm, cbs_input_node.index, VLIB_NODE_STATE_DISABLED);
              vlib_log_debug(log_class, "Configure: Disabled polling for cbs-wheel on thread %d", i);
          } else if (wrk_vm && cbs_input_node.index == (u32)~0) {
              // Use vlib_log directly instead of vlib_log_warning macro
              vlib_log(VLIB_LOG_LEVEL_WARNING, log_class, "Configure: cbs_input_node index invalid, cannot disable polling on thread %d", i);
          }
          if (cbsm->wheel_by_thread[i]) {
            cbs_wheel_free(cbsm, cbsm->wheel_by_thread[i]);
            cbsm->wheel_by_thread[i] = 0;
          }
      }
      vec_reset_length(cbsm->wheel_by_thread);
      vlib_worker_thread_barrier_release (vm);
      vlib_log_notice(log_class, "Configure: Previous wheels freed and polling disabled.");
      cbsm->is_configured = 0; // Mark as not configured until fully set up again
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
  // Using a fixed buffer time target might be simpler than complex bandwidth calculations
  f64 buffer_time_target = 0.010; // Target 10ms buffering
  u64 total_buffer_bytes = (cbsm->cbs_port_rate * buffer_time_target);
  // Ensure a minimum size based on packets
  total_buffer_bytes = clib_max(total_buffer_bytes, (u64)cbsm->packet_size * 1024); // At least 1024 packets worth

  u32 num_workers = vlib_num_workers();
  u64 per_worker_buffer_bytes = (num_workers > 0) ? (total_buffer_bytes / num_workers) : total_buffer_bytes;
  // Ensure minimum size per worker
  per_worker_buffer_bytes = clib_max(per_worker_buffer_bytes, (u64)cbsm->packet_size * 256); // At least 256 packets worth

  wheel_slots_per_wrk = per_worker_buffer_bytes / cbsm->packet_size;
  wheel_slots_per_wrk = clib_max(wheel_slots_per_wrk, (u64)CBS_MIN_WHEEL_SLOTS); // Ensure absolute minimum slots
  wheel_slots_per_wrk++; // Add one for safety/rounding
  cbsm->wheel_slots_per_wrk = wheel_slots_per_wrk;

  vlib_log_notice(log_class, "Configure: Calculated wheel size = %lu slots/worker (target %.3f s buffer)", cbsm->wheel_slots_per_wrk, buffer_time_target);

  // --- Allocate Wheels ---
  vec_validate (cbsm->wheel_by_thread, n_threads - 1);
  vlib_log_debug(log_class, "Configure: Allocating wheels for %d threads (0 to %d)", n_threads, n_threads - 1);
  for (i = 0; i < n_threads; i++) {
      cbsm->wheel_by_thread[i] = cbs_wheel_alloc (cbsm, i);
      if (PREDICT_FALSE(!cbsm->wheel_by_thread[i])) {
         vlib_log_err(log_class, "Configure: ERROR - Wheel allocation failed for thread %d", i);
         // Cleanup previously allocated wheels
         for (int j = 0; j < i; j++) {
             if (cbsm->wheel_by_thread[j]) {
                cbs_wheel_free(cbsm, cbsm->wheel_by_thread[j]);
                cbsm->wheel_by_thread[j] = 0;
             }
         }
         vec_reset_length(cbsm->wheel_by_thread);
         return VNET_API_ERROR_UNSPECIFIED; // Use standard unspecified error
      }
  }
  vlib_log_debug(log_class, "Configure: Wheels allocated successfully.");

  // --- Finalize and Enable Polling ---
  cbsm->is_configured = 1; // Mark as configured *after* successful setup
  vlib_log_notice(log_class, "Configure: Configuration complete. Enabling polling.");

  vlib_worker_thread_barrier_sync (vm);
  for (i = 0; i < n_threads; i++) {
      vlib_main_t *wrk_vm = vlib_get_main_by_index(i);
      if (wrk_vm) {
          if (PREDICT_TRUE(cbs_input_node.index != (u32)~0)) { // Check node index validity
             vlib_node_set_state (wrk_vm, cbs_input_node.index, VLIB_NODE_STATE_POLLING);
             vlib_log_debug(log_class, "Configure: Enabling polling for cbs-wheel on thread %u", i);
          } else {
             vlib_log_err(log_class, "Configure: ERROR - cbs_input_node index invalid, cannot enable polling on thread %u", i);
             // This indicates a potential VPP startup or plugin registration issue
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

  // Basic validation first
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
  u32 sw_if_index = clib_net_to_host_u32(mp->sw_if_index); // Convert once

  // Use VALIDATE_SW_IF_INDEX macro for standard interface validation
  VALIDATE_SW_IF_INDEX(mp); // Sets rv and jumps to BAD_SW_IF_INDEX_LABEL on error

  // If validation passed, call the internal function
  rv = cbs_output_feature_enable_disable (cbsm, sw_if_index, (int) (mp->enable_disable));

// Macro jumps here on validation failure
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
  // Convert signed i32 carefully from network u32
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
  cbsm->log_class = vlib_log_register_class("cbs", 0); // Register log class

  vlib_log_debug(cbsm->log_class, "CBS plugin initializing");

  // Initialize main struct fields to safe defaults
  cbsm->sw_if_index0 = ~0;
  cbsm->sw_if_index1 = ~0;
  cbsm->output_next_index0 = ~0;
  cbsm->output_next_index1 = ~0;
  cbsm->is_configured = 0;
  cbsm->output_next_index_by_sw_if_index = 0; // Initialize vector pointer to NULL
  cbsm->wheel_by_thread = 0;                  // Initialize vector pointer to NULL
  cbsm->msg_id_base = 0;                      // Initialize msg_id_base
  cbsm->arc_index = (u16)~0;                  // Initialize arc_index


  cbsm->msg_id_base = setup_message_id_table ();
  if (cbsm->msg_id_base == (u16)~0) { // Check for failure from setup
      error = clib_error_return (0, "Failed to setup API message ID table");
      goto done;
  }


  // Get arc index for interface-output
  cbsm->arc_index = vnet_get_feature_arc_index ("interface-output");
  if (cbsm->arc_index == (u16)~0) {
        error = clib_error_return (0, "Failed to get feature arc index for 'interface-output'");
        goto done;
  }

  vlib_log_debug(cbsm->log_class, "CBS plugin initialization complete");

done:
  return error;
}

VLIB_INIT_FUNCTION (cbs_init);

/* --- Feature registrations --- */
VNET_FEATURE_INIT (cbs_cross_connect_feat, static) =
{
  .arc_name = "device-input",
  .node_name = "cbs-cross-connect",
  .runs_before = VNET_FEATURES ("ethernet-input"), // Run before L3 processing
};

VNET_FEATURE_INIT (cbs_output_feature_feat, static) = {
  .arc_name = "interface-output",
  .node_name = "cbs-output-feature",
  // Run late in the output arc, but before the final interface transmit node
  .runs_before = VNET_FEATURES ("interface-output-arc-end"),
};

/* --- Plugin Registration --- */
VLIB_PLUGIN_REGISTER () =
{
  .version = VPP_BUILD_VER,
  .description = "Credit Based Shaper (CBS) Plugin",
  // Add other fields like .default_disabled=1 if desired
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
  // Only accept kbps for idleslope for clarity as per standard
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
   int verbose __attribute__((unused)) = va_arg (*args, int); // Keep verbose argument for potential future use

   s = format (s, "CBS Configuration:\n");
   if (!cbsm->is_configured) {
        s = format(s, "  Not configured.\n");
        return s;
   }
   s = format (s, "  Port Rate:       %U\n", format_cbs_rate, cbsm->cbs_port_rate);
   s = format (s, "  Idle Slope:      %U\n", format_cbs_slope, cbsm->cbs_idleslope);
   // Use format_cbs_rate for sendslope, as it's also a rate in bytes/sec
   s = format (s, "  Send Slope:      %U/sec (calculated)\n", format_cbs_rate, cbsm->cbs_sendslope);
   s = format (s, "  HiCredit:        %.0f bytes\n", cbsm->cbs_hicredit);
   s = format (s, "  LoCredit:        %.0f bytes\n", cbsm->cbs_locredit);

   s = format (s, "Internal Sizing:\n");
   s = format (s, "  Avg Packet Size: %u bytes\n", cbsm->packet_size);
   s = format (s, "  Bandwidth Hint:  %U (for wheel sizing)\n", format_cbs_rate, cbsm->configured_bandwidth);
   s = format (s, "  Wheel Size:      %u slots/worker\n", cbsm->wheel_slots_per_wrk);

   s = format (s, "\nEnabled Interfaces:\n");
    if (cbsm->sw_if_index0 != (u32)~0) { // Check explicitly against ~0
         s = format (s, "  Cross-connect: %U <--> %U\n",
                     format_vnet_sw_if_index_name, cbsm->vnet_main, cbsm->sw_if_index0,
                     format_vnet_sw_if_index_name, cbsm->vnet_main, cbsm->sw_if_index1);
    } else {
        int output_feature_enabled = 0;
        u32 i;
        // Use pool_foreach for potentially sparse vectors if they become pools later
        // For vec, simple loop is fine. Check length before iterating.
        if (cbsm->output_next_index_by_sw_if_index) {
            for (i = 0; i < vec_len (cbsm->output_next_index_by_sw_if_index); i++) {
              // Check if the index i is valid AND if the next index is set
              if (cbsm->output_next_index_by_sw_if_index[i] != (u32)~0) {
                // Double check if the sw_if_index is still valid in VPP
                 if (!pool_is_free_index(cbsm->vnet_main->interface_main.sw_interfaces, i)) {
                     if (!output_feature_enabled) {
                       s = format (s, "  Output Feature on:\n");
                       output_feature_enabled = 1;
                     }
                     s = format (s, "    %U\n", format_vnet_sw_if_index_name, cbsm->vnet_main, i);
                 }
              }
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
   vlib_log_class_t log_class = cbsm->log_class; // Get log class
   unformat_input_t _line_input, *line_input = &_line_input;
   u32 sw_if_index0 = ~0;
   u32 sw_if_index1 = ~0;
   int enable_disable = 1;
   u32 tmp;
   int rv;
   clib_error_t * error = 0;

   /* Get a line of input. */
   if (!unformat_user (input, unformat_line_input, line_input))
     return 0;


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

   vlib_log_notice(log_class, "%s cbs cross-connect: %U <--> %U",
                   enable_disable ? "enable" : "disable",
                   format_vnet_sw_if_index_name, cbsm->vnet_main, sw_if_index0,
                   format_vnet_sw_if_index_name, cbsm->vnet_main, sw_if_index1);

   rv = cbs_cross_connect_enable_disable (cbsm, sw_if_index0, sw_if_index1, enable_disable);

   switch (rv) {
     case 0: break; // Success
     case VNET_API_ERROR_FEATURE_DISABLED: error = clib_error_return (0, "CBS not configured, please 'set cbs ...' first"); break;
     case VNET_API_ERROR_INVALID_SW_IF_INDEX:
     case VNET_API_ERROR_INVALID_SW_IF_INDEX_2: error = clib_error_return(0, "Invalid software interface index"); break;
     case VNET_API_ERROR_INVALID_INTERFACE: error = clib_error_return (0, "Invalid interface type (must be hardware)"); break;
     case VNET_API_ERROR_UNSPECIFIED: // Handle the generic error code
          error = clib_error_return (0, "CBS cross-connect setup failed (unspecified internal error)");
          break;
     default:
         error = clib_error_return (0, "cbs_cross_connect_enable_disable failed: rv %d", rv);
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
    vlib_log_class_t log_class = cbsm->log_class; // Get log class
    unformat_input_t _line_input, *line_input = &_line_input;
    u32 sw_if_index = ~0;
    int enable_disable = 1;
    int rv;
    clib_error_t * error = 0;

    /* Get a line of input. */
    if (!unformat_user (input, unformat_line_input, line_input))
        return 0;

    while (unformat_check_input (line_input) != UNFORMAT_END_OF_INPUT) {
        if (unformat (line_input, "disable")) enable_disable = 0;
        else if (unformat (line_input, "%U", unformat_vnet_sw_interface, cbsm->vnet_main, &sw_if_index)) ;
        else if (unformat (line_input, "sw_if_index %u", &sw_if_index));
        else { error = clib_error_return (0, "unknown input `%U'", format_unformat_error, line_input); goto done; }
      }

    if (sw_if_index == ~0) { error = clib_error_return (0, "Please specify one interface"); goto done;}

    vlib_log_notice(log_class, "%s cbs output-feature: %U",
                    enable_disable ? "enable" : "disable",
                    format_vnet_sw_if_index_name, cbsm->vnet_main, sw_if_index);

    rv = cbs_output_feature_enable_disable (cbsm, sw_if_index, enable_disable);

    switch (rv) {
      case 0: break; // Success
      case VNET_API_ERROR_FEATURE_DISABLED: error = clib_error_return (0, "CBS not configured, please 'set cbs ...' first"); break;
      case VNET_API_ERROR_INVALID_SW_IF_INDEX: error = clib_error_return(0, "Invalid software interface index"); break;
      case VNET_API_ERROR_INVALID_INTERFACE: error = clib_error_return(0, "Invalid interface type (must be hardware)"); break; // Adjusted error message due to code change
      case VNET_API_ERROR_UNSPECIFIED: // Handle the generic error code
          error = clib_error_return (0, "CBS output feature setup failed (unspecified internal error)");
          break;
      default:
          error = clib_error_return (0, "cbs_output_feature_enable_disable failed: rv %d", rv);
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
    vlib_log_class_t log_class = cbsm->log_class; // Get log class
    f64 port_rate_bps = 0.0, idleslope_kbps = 0.0, bandwidth_bps_hint = 0.0;
    f64 hicredit_bytes = 0.0, locredit_bytes = 0.0;
    u32 packet_size = 0; // Use 0 to signify default
    int rv;
    clib_error_t * error = 0;
    // Track mandatory parameters
    int port_rate_set = 0, idleslope_set = 0, hicredit_set = 0, locredit_set = 0;

    while (unformat_check_input (input) != UNFORMAT_END_OF_INPUT) {
        if (unformat (input, "port_rate %U", unformat_cbs_rate, &port_rate_bps)) port_rate_set = 1;
        else if (unformat (input, "idleslope %U", unformat_cbs_slope, &idleslope_kbps)) idleslope_set = 1;
        else if (unformat (input, "hicredit %f", &hicredit_bytes)) hicredit_set = 1;
        else if (unformat (input, "locredit %f", &locredit_bytes)) locredit_set = 1;
        else if (unformat (input, "bandwidth %U", unformat_cbs_rate, &bandwidth_bps_hint)); // Optional
        else if (unformat (input, "packet-size %u", &packet_size)); // Optional
        else { error = clib_error_return (0, "unknown input '%U'", format_unformat_error, input); goto done; }
      }

    // Check if all mandatory parameters were provided
    if (!port_rate_set || !idleslope_set || !hicredit_set || !locredit_set) {
        error = clib_error_return (0, "Mandatory parameters missing. Required: port_rate, idleslope, hicredit, locredit");
        goto done;
    }

    vlib_log_notice(log_class, "Set CBS config: port_rate %.2fG, idle %.2fK, hi %.0f, lo %.0f, bw_hint %.2fM, pkt_size %u",
                    port_rate_bps / CBS_GBPS_TO_BPS, idleslope_kbps, hicredit_bytes, locredit_bytes,
                    bandwidth_bps_hint / CBS_MBPS_TO_BPS, packet_size);

    rv = cbs_configure_internal (cbsm, port_rate_bps, idleslope_kbps, hicredit_bytes, locredit_bytes, bandwidth_bps_hint, packet_size);

    switch (rv) {
      case 0: // Success
          vlib_cli_output (vm, "%U", format_cbs_config, 0 /* verbose=0 */);
          break;
      case VNET_API_ERROR_INVALID_VALUE: error = clib_error_return (0, "Invalid port_rate (must be > 0)"); break;
      case VNET_API_ERROR_INVALID_VALUE_2: error = clib_error_return (0, "Invalid idleslope (must be >= 0)"); break;
      case VNET_API_ERROR_INVALID_VALUE_3: error = clib_error_return (0, "Invalid credits (hicredit must be >= locredit)"); break;
      case VNET_API_ERROR_INVALID_VALUE_4: error = clib_error_return (0, "Invalid packet size (must be 64-9000, or 0 for default)"); break;
      case VNET_API_ERROR_UNSPECIFIED: error = clib_error_return(0, "Configuration failed (unspecified internal error)"); break;
      default:
          error = clib_error_return (0, "cbs_configure_internal failed: rv %d", rv);
          break;
    }

  done:
    return error;
}

static clib_error_t *
show_cbs_command_fn (vlib_main_t * vm, unformat_input_t * input, vlib_cli_command_t * cmd)
{
    int verbose = 0;
    clib_error_t * error = 0;

    // Allow "verbose" argument
    if (unformat (input, "verbose")) verbose = 1;
    // Check for any other unknown arguments
    else if (unformat_check_input(input) != UNFORMAT_END_OF_INPUT) {
       error = clib_error_return (0, "unknown input '%U'", format_unformat_error, input);
       goto done;
    }

    // No need to check is_configured here, format_cbs_config handles it.
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