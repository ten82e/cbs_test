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
// #include <vppinfra/random.h> // Removed
#include <vnet/api_errno.h>
#include <vppinfra/cpu.h>
#include <vlib/node_funcs.h>
// #include <vlib/process.h> // <-- この行を削除

#include <cbs/cbs.api_enum.h>
#include <cbs/cbs.api_types.h>

// Prototype for static function defined in auto-generated cbs.api.c
static u16 setup_message_id_table (void);

#define REPLY_MSG_ID_BASE cbs_main.msg_id_base
#include <vlibapi/api_helper_macros.h>

cbs_main_t cbs_main;

/* Action functions */
// ... (cbs_cross_connect_enable_disable, cbs_output_feature_enable_disable - 変更なし) ...
int
cbs_cross_connect_enable_disable (cbs_main_t * cbsm, u32 sw_if_index0,
				   u32 sw_if_index1, int enable_disable)
{
  vnet_sw_interface_t *sw;
  vnet_hw_interface_t *hw;
  vlib_main_t * vm = cbsm->vlib_main;
  int rv = 0;
  // int i, num_workers = vlib_num_workers(); // Removed

  if (cbsm->is_configured == 0 && enable_disable)
    return VNET_API_ERROR_FEATURE_DISABLED;

  if (!pool_is_free_index (cbsm->vnet_main->interface_main.sw_interfaces, sw_if_index0))
    sw = vnet_get_sw_interface (cbsm->vnet_main, sw_if_index0);
  else
    return VNET_API_ERROR_INVALID_SW_IF_INDEX;

  if (!pool_is_free_index (cbsm->vnet_main->interface_main.sw_interfaces, sw_if_index1))
     sw = vnet_get_sw_interface (cbsm->vnet_main, sw_if_index1);
   else
     return VNET_API_ERROR_INVALID_SW_IF_INDEX_2;

  if (sw->type != VNET_SW_INTERFACE_TYPE_HARDWARE)
    return VNET_API_ERROR_INVALID_INTERFACE;
   sw = vnet_get_sw_interface (cbsm->vnet_main, sw_if_index0);
   if (sw->type != VNET_SW_INTERFACE_TYPE_HARDWARE)
     return VNET_API_ERROR_INVALID_INTERFACE;


  if (enable_disable && cbsm->is_configured) {
      hw = vnet_get_hw_interface (cbsm->vnet_main, sw_if_index0);
      cbsm->output_next_index0 = vlib_node_add_next (vm, cbs_input_node.index, hw->output_node_index);

      hw = vnet_get_hw_interface (cbsm->vnet_main, sw_if_index1);
      cbsm->output_next_index1 = vlib_node_add_next (vm, cbs_input_node.index, hw->output_node_index);
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

   // Removed process signal

  return rv;
}

int
cbs_output_feature_enable_disable (cbs_main_t * cbsm, u32 sw_if_index,
				    int enable_disable)
{
  vnet_sw_interface_t *sw;
  vnet_hw_interface_t *hw;
  vlib_main_t * vm = cbsm->vlib_main;
  int rv = 0;
  // int i, num_workers = vlib_num_workers(); // Removed

  if (cbsm->is_configured == 0 && enable_disable)
    return VNET_API_ERROR_FEATURE_DISABLED;

  if (pool_is_free_index (cbsm->vnet_main->interface_main.sw_interfaces, sw_if_index))
    return VNET_API_ERROR_INVALID_SW_IF_INDEX;

  sw = vnet_get_sw_interface (cbsm->vnet_main, sw_if_index);
  if (sw->type != VNET_SW_INTERFACE_TYPE_HARDWARE)
    return VNET_API_ERROR_INVALID_INTERFACE;

  if (enable_disable && cbsm->is_configured) {
      hw = vnet_get_hw_interface (cbsm->vnet_main, sw_if_index);
      vec_validate_init_empty (cbsm->output_next_index_by_sw_if_index, sw_if_index, ~0);
      cbsm->output_next_index_by_sw_if_index[sw_if_index] = vlib_node_add_next (
        vm, cbs_input_node.index, hw->output_node_index);
  }

  rv = vnet_feature_enable_disable ("interface-output", "cbs-output-feature",
			       sw_if_index, enable_disable, 0, 0);

  if (!enable_disable) {
      if (sw_if_index < vec_len(cbsm->output_next_index_by_sw_if_index)) {
          cbsm->output_next_index_by_sw_if_index[sw_if_index] = ~0;
      }
  }

  // Removed process signal

  return rv;
}

static cbs_wheel_t *
cbs_wheel_alloc (cbs_main_t *cbsm, u32 thread_index)
{
  cbs_wheel_t *wp;
  uword alloc_size = sizeof (cbs_wheel_t) +
                     cbsm->wheel_slots_per_wrk * sizeof (cbs_wheel_entry_t);
  vlib_main_t *current_vm = cbsm->vlib_main;

  wp = (cbs_wheel_t *) clib_mem_alloc_aligned (alloc_size, CLIB_CACHE_LINE_BYTES);
  if (!wp) return 0;
  clib_memset (wp, 0, alloc_size);

  wp->wheel_size = cbsm->wheel_slots_per_wrk;
  wp->entries = (void *) (wp + 1);

  wp->cbs_credits = 0.0;
  wp->cbs_last_update_time = vlib_time_now(current_vm);
  wp->cbs_last_tx_finish_time = vlib_time_now(current_vm);

  return wp;
}

static void cbs_wheel_free(cbs_main_t *cbsm, cbs_wheel_t *wp)
{
    if (wp) clib_mem_free(wp);
}

// Updated function signature (removed drop_fraction)
static int
cbs_configure_internal (cbs_main_t * cbsm, f64 port_rate_bps,
			     f64 idleslope_kbps, f64 hicredit_bytes,
			     f64 locredit_bytes, f64 bandwidth_bps_hint,
			     u32 packet_size)
{
  u64 wheel_slots_per_wrk;
  int i, num_workers;
  vlib_main_t *vm = cbsm->vlib_main;
  f64 port_rate_bytes_sec, idleslope_bytes_sec, sendslope_bytes_sec;
  f64 effective_bandwidth_for_sizing;

  clib_warning("CBS Configure Internal called. port_rate=%.2f Gbps, idleslope=%.2f Kbps, hi=%.0f, lo=%.0f",
               port_rate_bps / CBS_GBPS_TO_BPS, idleslope_kbps, hicredit_bytes, locredit_bytes);

  if (port_rate_bps <= 0.0) return VNET_API_ERROR_INVALID_VALUE;
  if (idleslope_kbps <= 0.0) return VNET_API_ERROR_INVALID_VALUE_2;
  if (hicredit_bytes < locredit_bytes) return VNET_API_ERROR_INVALID_VALUE_3;
  if (packet_size == 0) packet_size = CBS_DEFAULT_PACKET_SIZE;
  if (packet_size < 64 || packet_size > 9000) return VNET_API_ERROR_INVALID_VALUE_4;
  // Removed fraction validations

  port_rate_bytes_sec = port_rate_bps / CBS_BITS_PER_BYTE;
  idleslope_bytes_sec = (idleslope_kbps * CBS_KBPS_TO_BPS) / CBS_BITS_PER_BYTE;
  sendslope_bytes_sec = idleslope_bytes_sec - port_rate_bytes_sec;

  num_workers = vlib_num_workers ();

  if (cbsm->is_configured)
    {
      clib_warning("CBS Configure: Cleaning up previous configuration.");
      vlib_worker_thread_barrier_sync (vm);
      for (i = 0; i < vec_len (cbsm->wheel_by_thread); i++)
	    {
	      if (cbsm->wheel_by_thread[i]) {
	          cbs_wheel_free(cbsm, cbsm->wheel_by_thread[i]);
	          cbsm->wheel_by_thread[i] = 0;
          }
	    }
      vec_reset_length(cbsm->wheel_by_thread);
      vlib_worker_thread_barrier_release (vm);
      clib_warning("CBS Configure: Previous wheels freed.");
    }

  cbsm->cbs_port_rate = port_rate_bytes_sec;
  cbsm->cbs_idleslope = idleslope_bytes_sec;
  cbsm->cbs_sendslope = sendslope_bytes_sec;
  cbsm->cbs_hicredit = hicredit_bytes;
  cbsm->cbs_locredit = locredit_bytes;
  // cbsm->drop_fraction = 0.0; // Removed
  cbsm->packet_size = packet_size;

  effective_bandwidth_for_sizing = (bandwidth_bps_hint > 0) ? bandwidth_bps_hint : port_rate_bps;
  cbsm->configured_bandwidth = effective_bandwidth_for_sizing / CBS_BITS_PER_BYTE;

  f64 buffer_time_target = 0.010;
  u64 total_buffer_bytes = (cbsm->cbs_port_rate * buffer_time_target);
  total_buffer_bytes = clib_max(total_buffer_bytes, (u64)packet_size * 1024);

  u64 per_worker_buffer_bytes = (num_workers > 0) ? (total_buffer_bytes / num_workers) : total_buffer_bytes;
  per_worker_buffer_bytes = clib_max(per_worker_buffer_bytes, (u64)packet_size * 256);

  wheel_slots_per_wrk = per_worker_buffer_bytes / packet_size;
  wheel_slots_per_wrk = clib_max(wheel_slots_per_wrk, 1024);
  wheel_slots_per_wrk++;
  cbsm->wheel_slots_per_wrk = wheel_slots_per_wrk;
  clib_warning("CBS Configure: Calculated wheel size = %lu slots/worker", wheel_slots_per_wrk);

  vec_validate (cbsm->wheel_by_thread, num_workers);
  clib_warning("CBS Configure: Allocating wheels for %d threads (0 to %d)", num_workers + 1, num_workers);
  for (i = 0; i < num_workers + 1; i++)
    {
      cbsm->wheel_by_thread[i] = cbs_wheel_alloc (cbsm, i);
      if (!cbsm->wheel_by_thread[i]) {
         clib_warning("CBS Configure: ERROR - Wheel allocation failed for thread %d", i);
         for (int j = 0; j < i; j++) {
             if (cbsm->wheel_by_thread[j]) {
                cbs_wheel_free(cbsm, cbsm->wheel_by_thread[j]);
                cbsm->wheel_by_thread[j] = 0;
             }
         }
         vec_reset_length(cbsm->wheel_by_thread);
         cbsm->is_configured = 0;
         return VNET_API_ERROR_UNSPECIFIED;
      }
    }
  clib_warning("CBS Configure: Wheels allocated successfully.");

  cbsm->is_configured = 1;
  clib_warning("CBS Configure: Configuration complete.");

  // Removed process signal loop

  return 0;
}

#ifndef CLIB_MARCH_VARIANT // Start Base Implementation Block

/* API Handlers */

static void vl_api_cbs_cross_connect_enable_disable_t_handler
  (vl_api_cbs_cross_connect_enable_disable_t * mp)
{
  vl_api_cbs_cross_connect_enable_disable_reply_t *rmp;
  cbs_main_t *cbsm = &cbs_main;
  int rv;
  u32 sw_if_index0 = ntohl(mp->sw_if_index0);
  u32 sw_if_index1 = ntohl(mp->sw_if_index1);

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

  rv = cbs_output_feature_enable_disable (cbsm, ntohl(mp->sw_if_index), (int) (mp->enable_disable));

  BAD_SW_IF_INDEX_LABEL;
  REPLY_MACRO (VL_API_CBS_OUTPUT_FEATURE_ENABLE_DISABLE_REPLY);
}

static void
vl_api_cbs_configure_t_handler (vl_api_cbs_configure_t * mp)
{
  vl_api_cbs_configure_reply_t *rmp;
  cbs_main_t *cbsm = &cbs_main;
  f64 port_rate_bps, idleslope_kbps, bandwidth_bps_hint = 0.0;
  f64 hicredit_bytes, locredit_bytes;
  u32 packet_size;
  int rv;

  port_rate_bps = (f64) clib_net_to_host_u64 (mp->port_rate_bps);
  idleslope_kbps = (f64) clib_net_to_host_u64 (mp->idleslope_kbps);
  hicredit_bytes = (f64) ((i32) ntohl(mp->hicredit_bytes));
  locredit_bytes = (f64) ((i32) ntohl(mp->locredit_bytes));
  packet_size = ntohl (mp->average_packet_size);
  bandwidth_bps_hint = (f64) clib_net_to_host_u64 (mp->bandwidth_in_bits_per_second);
  // Removed drop/reorder params

  // Call configure internal without drop/reorder fractions
  rv = cbs_configure_internal (cbsm, port_rate_bps, idleslope_kbps,
				    hicredit_bytes, locredit_bytes,
				    bandwidth_bps_hint, packet_size);

  REPLY_MACRO (VL_API_CBS_CONFIGURE_REPLY);
}


/* Plugin Initialization */
static clib_error_t *
cbs_init (vlib_main_t * vm)
{
  cbs_main_t *cbsm = &cbs_main;

  cbsm->vlib_main = vm;
  cbsm->vnet_main = vnet_get_main ();
  // cbsm->seed = (u32) clib_cpu_time_now(); // Removed
  cbsm->sw_if_index0 = ~0;
  cbsm->sw_if_index1 = ~0;
  cbsm->is_configured = 0;
  // cbsm->poll_main_thread = 0; // Removed

  cbsm->msg_id_base = setup_message_id_table ();

  cbsm->arc_index = vnet_get_feature_arc_index ("interface-output");

  cbsm->output_next_index_by_sw_if_index = 0;
  cbsm->wheel_by_thread = 0;

  return 0;
}

VLIB_INIT_FUNCTION (cbs_init);

/* Feature registrations */
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

/* Plugin Registration */
VLIB_PLUGIN_REGISTER () =
{
  .version = VPP_BUILD_VER,
  .description = "Credit Based Shaper (CBS) Plugin",
};

/* CLI unformatters */
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

/* CLI formatters */
static u8 *
format_cbs_rate (u8 *s, va_list *args)
{
  f64 rate_bytes_sec = va_arg (*args, f64); f64 rate_bps = rate_bytes_sec * CBS_BITS_PER_BYTE;
  if (rate_bps >= CBS_GBPS_TO_BPS) s = format (s, "%.2f Gbps", rate_bps / CBS_GBPS_TO_BPS);
  else if (rate_bps >= CBS_MBPS_TO_BPS) s = format (s, "%.2f Mbps", rate_bps / CBS_MBPS_TO_BPS);
  else if (rate_bps >= CBS_KBPS_TO_BPS) s = format (s, "%.2f Kbps", rate_bps / CBS_KBPS_TO_BPS);
  else s = format (s, "%.2f bps", rate_bps); return s;
}
static u8 *
format_cbs_slope (u8 *s, va_list *args)
{
  f64 slope_bytes_sec = va_arg (*args, f64); f64 slope_kbps = (slope_bytes_sec * CBS_BITS_PER_BYTE) / CBS_KBPS_TO_BPS;
  s = format (s, "%.2f Kbps", slope_kbps); return s;
}

static u8 *
format_cbs_config (u8 * s, va_list * args)
{
  cbs_main_t *cbsm = &cbs_main;
  // int verbose = va_arg (*args, int); // Removed verbose flag

  s = format (s, "CBS Configuration:\n");
  s = format (s, "  Port Rate:       %U\n", format_cbs_rate, cbsm->cbs_port_rate);
  s = format (s, "  Idle Slope:      %U\n", format_cbs_slope, cbsm->cbs_idleslope);
  s = format (s, "  Send Slope:      %U/sec\n", format_cbs_rate, cbsm->cbs_sendslope);
  s = format (s, "  HiCredit:        %.0f bytes\n", cbsm->cbs_hicredit);
  s = format (s, "  LoCredit:        %.0f bytes\n", cbsm->cbs_locredit);

  // Removed optional features display
  // s = format (s, "Optional Features:\n");
  // ...

  s = format (s, "Internal Sizing:\n");
  s = format (s, "  Avg Packet Size: %u bytes\n", cbsm->packet_size);
  s = format (s, "  Bandwidth Hint:  %U (for wheel sizing)\n", format_cbs_rate, cbsm->configured_bandwidth);
  s = format (s, "  Wheel Size:      %u slots/worker\n", cbsm->wheel_slots_per_wrk);

  // Removed verbose block

   s = format (s, "\nEnabled Interfaces:\n");
   if (cbsm->sw_if_index0 != ~0) {
        s = format (s, "  Cross-connect: %U <--> %U\n", format_vnet_sw_if_index_name, cbsm->vnet_main, cbsm->sw_if_index0, format_vnet_sw_if_index_name, cbsm->vnet_main, cbsm->sw_if_index1);
   } else if (vec_len(cbsm->output_next_index_by_sw_if_index) > 0) {
       s = format (s, "  Output Feature on:\n"); u32 i;
       for (i = 0; i < vec_len (cbsm->output_next_index_by_sw_if_index); i++) {
         if (i < vec_len(cbsm->output_next_index_by_sw_if_index) &&
             cbsm->output_next_index_by_sw_if_index[i] != ~0)
           s = format (s, "    %U\n", format_vnet_sw_if_index_name, cbsm->vnet_main, i);
       }
   } else { s = format(s, "  None\n"); }

  return s;
}

/* CLI Command Handlers */
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

  if (!unformat_user (input, unformat_line_input, line_input))
    return 0;

  while (unformat_check_input (line_input) != UNFORMAT_END_OF_INPUT)
    {
      if (unformat (line_input, "disable")) enable_disable = 0;
      else if (unformat (line_input, "%U", unformat_vnet_sw_interface, cbsm->vnet_main, &tmp))
	    {
          if (sw_if_index0 == ~0) sw_if_index0 = tmp;
          else if (sw_if_index1 == ~0) sw_if_index1 = tmp;
          else return clib_error_return (0, "Please specify only two interfaces");
        }
      else if (unformat (line_input, "sw_if_index %u", &tmp))
       {
          if (sw_if_index0 == ~0) sw_if_index0 = tmp;
          else if (sw_if_index1 == ~0) sw_if_index1 = tmp;
          else return clib_error_return (0, "Please specify only two interfaces");
       }
      else return clib_error_return (0, "unknown input '%U'", format_unformat_error, line_input);
    }
  unformat_free (line_input);

  if (sw_if_index0 == ~0 || sw_if_index1 == ~0) return clib_error_return (0, "Please specify two interfaces");

  rv = cbs_cross_connect_enable_disable (cbsm, sw_if_index0, sw_if_index1, enable_disable);

  switch (rv) {
    case 0: break;
    case VNET_API_ERROR_FEATURE_DISABLED: return clib_error_return (0, "CBS not configured, please 'set cbs ...' first");
    case VNET_API_ERROR_INVALID_SW_IF_INDEX:
    case VNET_API_ERROR_INVALID_SW_IF_INDEX_2:
    case VNET_API_ERROR_INVALID_INTERFACE: return clib_error_return (0, "Invalid interface (must be hardware)");
    default: return clib_error_return (0, "cbs_cross_connect_enable_disable returned %d", rv);
  }
  return 0;
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

  if (!unformat_user (input, unformat_line_input, line_input)) return 0;

  while (unformat_check_input (line_input) != UNFORMAT_END_OF_INPUT) {
      if (unformat (line_input, "disable")) enable_disable = 0;
      else if (unformat (line_input, "%U", unformat_vnet_sw_interface, cbsm->vnet_main, &sw_if_index)) ;
      else if (unformat (line_input, "sw_if_index %u", &sw_if_index));
      else {
	    clib_error_t *error = clib_error_return (0, "unknown input `%U'", format_unformat_error, line_input);
	    unformat_free (line_input); return error;
	}
    }
  unformat_free (line_input);

  if (sw_if_index == ~0) return clib_error_return (0, "Please specify one interface");

  rv = cbs_output_feature_enable_disable (cbsm, sw_if_index, enable_disable);

  switch (rv) {
    case 0: break;
    case VNET_API_ERROR_FEATURE_DISABLED: return clib_error_return (0, "CBS not configured, please 'set cbs ...' first");
    case VNET_API_ERROR_INVALID_SW_IF_INDEX:
    case VNET_API_ERROR_INVALID_INTERFACE: return clib_error_return(0, "Invalid interface (must be hardware)");
    default: return clib_error_return (0, "cbs_output_feature_enable_disable returned %d", rv);
    }
  return 0;
}

static clib_error_t *
set_cbs_command_fn (vlib_main_t * vm, unformat_input_t * input, vlib_cli_command_t * cmd)
{
  cbs_main_t *cbsm = &cbs_main;
  f64 port_rate_bps = 0.0, idleslope_kbps = 0.0, bandwidth_bps_hint = 0.0;
  f64 hicredit_bytes = 0.0, locredit_bytes = 0.0;
  u32 packet_size = 0;
  // Removed drop/reorder variables
  int rv;
  int port_rate_set = 0, idleslope_set = 0, hicredit_set = 0, locredit_set = 0;

  while (unformat_check_input (input) != UNFORMAT_END_OF_INPUT) {
      if (unformat (input, "port_rate %U", unformat_cbs_rate, &port_rate_bps)) port_rate_set = 1;
      else if (unformat (input, "idleslope %U", unformat_cbs_slope, &idleslope_kbps)) idleslope_set = 1;
      else if (unformat (input, "hicredit %f", &hicredit_bytes)) hicredit_set = 1;
      else if (unformat (input, "locredit %f", &locredit_bytes)) locredit_set = 1;
      else if (unformat (input, "bandwidth %U", unformat_cbs_rate, &bandwidth_bps_hint));
      else if (unformat (input, "packet-size %u", &packet_size));
      // Removed drop/reorder parsing
      else return clib_error_return (0, "unknown input '%U'", format_unformat_error, input);
    }

  if (!port_rate_set || !idleslope_set || !hicredit_set || !locredit_set) return clib_error_return (0, "Mandatory parameters missing: port_rate, idleslope, hicredit, locredit");

  // Removed fraction calculation

  // Call configure internal without drop/reorder fractions
  rv = cbs_configure_internal (cbsm, port_rate_bps, idleslope_kbps, hicredit_bytes, locredit_bytes, bandwidth_bps_hint, packet_size);

  switch (rv) {
    case 0: vlib_cli_output (vm, "%U", format_cbs_config, 0); break;
    case VNET_API_ERROR_INVALID_VALUE: return clib_error_return (0, "Invalid port_rate (> 0)");
    case VNET_API_ERROR_INVALID_VALUE_2: return clib_error_return (0, "Invalid idleslope (> 0)");
    case VNET_API_ERROR_INVALID_VALUE_3: return clib_error_return (0, "Invalid credits (hicredit >= locredit)");
    case VNET_API_ERROR_INVALID_VALUE_4: return clib_error_return (0, "Invalid packet size (64-9000)");
    // Removed invalid fraction error case
    case VNET_API_ERROR_UNSPECIFIED: return clib_error_return(0, "Configuration failed (e.g., memory allocation error)");
    default: return clib_error_return (0, "cbs_configure_internal failed: %d", rv);
  }
  return 0;
}

VLIB_CLI_COMMAND (set_cbs_command, static) =
{
  .path = "set cbs",
  .short_help = "set cbs port_rate <rate> idleslope <kbps> hicredit <bytes> locredit <bytes> [bandwidth <rate>] [packet-size <n>]", // Updated help
  .function = set_cbs_command_fn,
};


static clib_error_t *
show_cbs_command_fn (vlib_main_t * vm, unformat_input_t * input, vlib_cli_command_t * cmd)
{
  cbs_main_t *cbsm = &cbs_main;
  int verbose = 0; // Keep verbose for potential future additions
  if (unformat (input, "verbose")) verbose = 1;
  else if (unformat_check_input(input) != UNFORMAT_END_OF_INPUT)
     return clib_error_return (0, "unknown input '%U'", format_unformat_error, input);

  if (cbsm->is_configured == 0) return clib_error_return (0, "CBS not configured. Use 'set cbs ...'");

  vlib_cli_output (vm, "%U", format_cbs_config, verbose);
  return 0;
}

VLIB_CLI_COMMAND (show_cbs_command, static) =
{
  .path = "show cbs",
  .short_help = "Display CBS configuration and state",
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

/* Include the auto-generated API C file *AFTER* handler definitions */
#include <cbs/cbs.api.c>

/* ... (fd.io tags etc.) ... */