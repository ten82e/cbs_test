/*
 * cbs.c - VPP CBS Plugin main implementation
 * ... (Copyright etc.) ...
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
#include <vppinfra/random.h>
#include <vnet/api_errno.h>

#include <cbs/cbs.api_enum.h>
#include <cbs/cbs.api_types.h>

// Prototype needed before cbs_init
static u16 setup_message_id_table (void);

#define REPLY_MSG_ID_BASE cbs_main.msg_id_base
#include <vlibapi/api_helper_macros.h>

cbs_main_t cbs_main;

/* Action functions */
int
cbs_cross_connect_enable_disable (cbs_main_t * cbsm, u32 sw_if_index0,
				   u32 sw_if_index1, int enable_disable)
{
  vnet_sw_interface_t *sw;
  vnet_hw_interface_t *hw;
  int rv = 0;

  if (cbsm->is_configured == 0)
    return VNET_API_ERROR_FEATURE_DISABLED;

  if (pool_is_free_index (cbsm->vnet_main->interface_main.sw_interfaces, sw_if_index0))
    return VNET_API_ERROR_INVALID_SW_IF_INDEX;
  if (pool_is_free_index (cbsm->vnet_main->interface_main.sw_interfaces, sw_if_index1))
    return VNET_API_ERROR_INVALID_SW_IF_INDEX;

  sw = vnet_get_sw_interface (cbsm->vnet_main, sw_if_index0);
  if (sw->type != VNET_SW_INTERFACE_TYPE_HARDWARE)
    return VNET_API_ERROR_INVALID_SW_IF_INDEX;
  sw = vnet_get_sw_interface (cbsm->vnet_main, sw_if_index1);
  if (sw->type != VNET_SW_INTERFACE_TYPE_HARDWARE)
    return VNET_API_ERROR_INVALID_SW_IF_INDEX;

  hw = vnet_get_hw_interface (cbsm->vnet_main, sw_if_index0);
  cbsm->output_next_index0 =
    vlib_node_add_next (cbsm->vlib_main, cbs_input_node.index, hw->output_node_index);

  hw = vnet_get_hw_interface (cbsm->vnet_main, sw_if_index1);
  cbsm->output_next_index1 =
    vlib_node_add_next (cbsm->vlib_main, cbs_input_node.index, hw->output_node_index);

  cbsm->sw_if_index0 = sw_if_index0;
  cbsm->sw_if_index1 = sw_if_index1;

  vnet_feature_enable_disable ("device-input", "cbs-cross-connect",
			       sw_if_index0, enable_disable, 0, 0);
  vnet_feature_enable_disable ("device-input", "cbs-cross-connect",
			       sw_if_index1, enable_disable, 0, 0);

  return rv;
}

int
cbs_output_feature_enable_disable (cbs_main_t * cbsm, u32 sw_if_index,
				    int enable_disable)
{
  vnet_sw_interface_t *sw;
  vnet_hw_interface_t *hw;
  int rv = 0;

  if (cbsm->is_configured == 0)
    return VNET_API_ERROR_FEATURE_DISABLED;

  if (pool_is_free_index (cbsm->vnet_main->interface_main.sw_interfaces, sw_if_index))
    return VNET_API_ERROR_INVALID_SW_IF_INDEX;

  sw = vnet_get_sw_interface (cbsm->vnet_main, sw_if_index);
  if (sw->type != VNET_SW_INTERFACE_TYPE_HARDWARE)
    return VNET_API_ERROR_INVALID_SW_IF_INDEX;

  hw = vnet_get_hw_interface (cbsm->vnet_main, sw_if_index);
  vec_validate_init_empty (cbsm->output_next_index_by_sw_if_index, sw_if_index, ~0);
  cbsm->output_next_index_by_sw_if_index[sw_if_index] = vlib_node_add_next (
    cbsm->vlib_main, cbs_input_node.index, hw->output_node_index);

  vnet_feature_enable_disable ("interface-output", "cbs-output-feature",
			       sw_if_index, enable_disable, 0, 0);
  return rv;
}

static cbs_wheel_t *
cbs_wheel_alloc (cbs_main_t *cbsm)
{
  cbs_wheel_t *wp;
  uword alloc_size = sizeof (cbs_wheel_t) +
                     cbsm->wheel_slots_per_wrk * sizeof (cbs_wheel_entry_t);

  wp = (cbs_wheel_t *) clib_mem_alloc_aligned (alloc_size, CLIB_CACHE_LINE_BYTES);
  if (!wp) return 0;
  clib_memset (wp, 0, alloc_size);

  wp->wheel_size = cbsm->wheel_slots_per_wrk;
  wp->entries = (void *) (wp + 1);

  wp->cbs_credits = 0.0;
  // Initialize time stamps using the correct vlib_main instance
  wp->cbs_last_update_time = vlib_time_now(cbsm->vlib_main);
  wp->cbs_last_tx_finish_time = vlib_time_now(cbsm->vlib_main);

  return wp;
}

static void cbs_wheel_free(cbs_main_t *cbsm, cbs_wheel_t *wp)
{
    if (wp) clib_mem_free(wp);
}

static int
cbs_configure_internal (cbs_main_t * cbsm, f64 port_rate_bps,
			     f64 idleslope_kbps, i64 hicredit_bytes,
			     i64 locredit_bytes, f64 bandwidth_bps,
			     u32 packet_size, f64 drop_fraction,
			     f64 reorder_fraction)
{
  u64 wheel_slots_per_wrk;
  int i, num_workers;
  vlib_main_t *vm = cbsm->vlib_main;
  f64 port_rate_bytes_sec, idleslope_bytes_sec, sendslope_bytes_sec;
  f64 effective_bandwidth_for_sizing;

  /* Input validation */
  if (port_rate_bps <= 0.0) return VNET_API_ERROR_INVALID_VALUE;
  if (idleslope_kbps <= 0.0) return VNET_API_ERROR_INVALID_VALUE_2;
  if (hicredit_bytes < locredit_bytes) return VNET_API_ERROR_INVALID_VALUE_3;
  if (packet_size == 0) packet_size = 1500;
  if (packet_size < 64 || packet_size > 9000) return VNET_API_ERROR_INVALID_VALUE_4;

  /* Convert rates */
  port_rate_bytes_sec = port_rate_bps / CBS_BITS_PER_BYTE;
  idleslope_bytes_sec = (idleslope_kbps * CBS_KBPS_TO_BPS) / CBS_BITS_PER_BYTE;
  sendslope_bytes_sec = idleslope_bytes_sec - port_rate_bytes_sec;

  num_workers = vlib_num_workers ();

  /* Free old wheels */
  if (cbsm->is_configured)
    {
      // Deactivate input node polling first before freeing wheels
      for (i = 0; i < vec_len (cbsm->wheel_by_thread); i++)
      {
          vlib_main_t *this_vm = vlib_get_main_by_index (i);
          if (this_vm)
             vlib_node_set_state (this_vm, cbs_input_node.index, VLIB_NODE_STATE_DISABLED);
      }
      vlib_worker_thread_barrier_sync (vm); // Ensure state change is seen

      for (i = 0; i < vec_len (cbsm->wheel_by_thread); i++)
	{
	  if (cbsm->wheel_by_thread[i]) {
	      cbs_wheel_free(cbsm, cbsm->wheel_by_thread[i]);
	      cbsm->wheel_by_thread[i] = 0;
          }
	}
      vec_reset_length(cbsm->wheel_by_thread);
      vlib_worker_thread_barrier_release (vm);
    }

  /* Store CBS parameters */
  cbsm->cbs_port_rate = port_rate_bytes_sec;
  cbsm->cbs_idleslope = idleslope_bytes_sec;
  cbsm->cbs_sendslope = sendslope_bytes_sec;
  cbsm->cbs_hicredit = hicredit_bytes;
  cbsm->cbs_locredit = locredit_bytes;

  /* Store optional parameters */
  cbsm->drop_fraction = drop_fraction;
  cbsm->reorder_fraction = reorder_fraction;
  cbsm->packet_size = packet_size;

  /* Calculate wheel size */
  effective_bandwidth_for_sizing = (bandwidth_bps > 0) ? bandwidth_bps : port_rate_bps;
  cbsm->bandwidth = effective_bandwidth_for_sizing;
  f64 buffer_time_heuristic = 0.1;
  u64 total_buffer_bytes = (effective_bandwidth_for_sizing / CBS_BITS_PER_BYTE) * buffer_time_heuristic;
  // Ensure non-zero buffer bytes calculation
  if (total_buffer_bytes == 0) total_buffer_bytes = packet_size * 1024; // Default fallback size
  u64 per_worker_buffer_bytes = (num_workers > 0) ? (total_buffer_bytes / num_workers) : total_buffer_bytes;
  if (per_worker_buffer_bytes == 0) per_worker_buffer_bytes = packet_size * 1024; // Ensure non-zero per worker

  wheel_slots_per_wrk = per_worker_buffer_bytes / packet_size;
  wheel_slots_per_wrk = clib_max(wheel_slots_per_wrk, 1024); // Ensure minimum size
  wheel_slots_per_wrk++; // Add one for safety/rounding
  cbsm->wheel_slots_per_wrk = wheel_slots_per_wrk;

  /* Allocate wheels for ALL threads (main + workers) */
  vec_validate (cbsm->wheel_by_thread, num_workers); // 0..num_workers indices
  for (i = 0; i < num_workers + 1; i++) // Always loop 0 to num_workers
    {
      cbsm->wheel_by_thread[i] = cbs_wheel_alloc (cbsm);
      if (!cbsm->wheel_by_thread[i]) {
         // Allocation failed - cleanup previously allocated wheels
         for (int j = 0; j < i; j++) {
             if (cbsm->wheel_by_thread[j]) cbs_wheel_free(cbsm, cbsm->wheel_by_thread[j]);
             cbsm->wheel_by_thread[j] = 0;
         }
         vec_set_len(cbsm->wheel_by_thread, 0); // Reset vector length
         cbsm->is_configured = 0; // Mark as not configured
         return VNET_API_ERROR_UNSPECIFIED; // Corrected error code
      }
    }

  vlib_worker_thread_barrier_sync (vm);

  /* Enable input node polling state *based on poll_main_thread flag* */
  int start_idx = (!cbsm->poll_main_thread && num_workers) ? 1 : 0; // Determine start index based on flag
  for (i = start_idx; i < num_workers + 1; i++)
    {
      vlib_main_t *this_vm = vlib_get_main_by_index (i);
      if (this_vm)
        vlib_node_set_state (this_vm, cbs_input_node.index, VLIB_NODE_STATE_POLLING);
    }

  vlib_worker_thread_barrier_release (vm);

  cbsm->is_configured = 1;
  return 0;
}

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

  if (!unformat_user (input, unformat_line_input, line_input)) return 0;

  while (unformat_check_input (line_input) != UNFORMAT_END_OF_INPUT)
    {
      if (unformat (line_input, "disable")) enable_disable = 0;
      else if (unformat (line_input, "%U", unformat_vnet_sw_interface, cbsm->vnet_main, &tmp))
	{ if (sw_if_index0 == ~0) sw_if_index0 = tmp; else sw_if_index1 = tmp; }
      else return clib_error_return (0, "unknown input '%U'", format_unformat_error, line_input);
    }
  unformat_free (line_input);

  if (sw_if_index0 == ~0 || sw_if_index1 == ~0) return clib_error_return (0, "Please specify two interfaces");

  rv = cbs_cross_connect_enable_disable (cbsm, sw_if_index0, sw_if_index1, enable_disable);

  switch (rv) {
    case 0: break;
    case VNET_API_ERROR_FEATURE_DISABLED: return clib_error_return (0, "Not configured, please 'set cbs ...' first");
    case VNET_API_ERROR_INVALID_SW_IF_INDEX: return clib_error_return (0, "Invalid interface");
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
    case VNET_API_ERROR_FEATURE_DISABLED: return clib_error_return (0, "Not configured, please 'set cbs ...' first");
    case VNET_API_ERROR_INVALID_SW_IF_INDEX: return clib_error_return(0, "Invalid interface");
    default: return clib_error_return (0, "cbs_output_feature_enable_disable returned %d", rv);
    }
  return 0;
}


/* API Handlers (Definitions must come BEFORE the include of cbs.api.c) */

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
  u32 sw_if_index = ntohl(mp->sw_if_index);

  VALIDATE_SW_IF_INDEX(mp);

  rv = cbs_output_feature_enable_disable (cbsm, sw_if_index, (int) (mp->enable_disable));

  BAD_SW_IF_INDEX_LABEL;
  REPLY_MACRO (VL_API_CBS_OUTPUT_FEATURE_ENABLE_DISABLE_REPLY);
}

static void
vl_api_cbs_configure_t_handler (vl_api_cbs_configure_t * mp)
{
  vl_api_cbs_configure_reply_t *rmp;
  cbs_main_t *cbsm = &cbs_main;
  f64 port_rate_bps, idleslope_kbps, bandwidth_bps = 0.0;
  i64 hicredit_bytes, locredit_bytes;
  u32 packet_size, packets_per_drop, packets_per_reorder;
  f64 drop_fraction = 0.0, reorder_fraction = 0.0;
  int rv;

  port_rate_bps = (f64) clib_net_to_host_u64 (mp->port_rate_bps);
  idleslope_kbps = (f64) clib_net_to_host_u64 (mp->idleslope_kbps);
  hicredit_bytes = (i64) ntohl(mp->hicredit_bytes);
  locredit_bytes = (i64) ntohl(mp->locredit_bytes);
  packet_size = ntohl (mp->average_packet_size);
  bandwidth_bps = (f64) clib_net_to_host_u64 (mp->bandwidth_in_bits_per_second);
  packets_per_drop = ntohl (mp->packets_per_drop);
  packets_per_reorder = ntohl (mp->packets_per_reorder);

  if (packets_per_drop > 0) drop_fraction = 1.0 / (f64) (packets_per_drop);
  if (packets_per_reorder > 0) reorder_fraction = 1.0 / (f64) packets_per_reorder;

  rv = cbs_configure_internal (cbsm, port_rate_bps, idleslope_kbps,
				    hicredit_bytes, locredit_bytes,
				    bandwidth_bps, packet_size,
				    drop_fraction, reorder_fraction);

  REPLY_MACRO (VL_API_CBS_CONFIGURE_REPLY);
}


/* Plugin Initialization */
static clib_error_t *
cbs_init (vlib_main_t * vm)
{
  cbs_main_t *cbsm = &cbs_main;

  cbsm->vlib_main = vm;
  cbsm->vnet_main = vnet_get_main ();
  clib_time_init (&cbsm->clib_time);
  cbsm->seed = (u32) clib_time_now (&cbsm->clib_time);
  cbsm->sw_if_index0 = ~0;
  cbsm->sw_if_index1 = ~0;

  // Call setup_message_id_table here
  cbsm->msg_id_base = setup_message_id_table ();
  cbsm->arc_index = vnet_get_feature_arc_index ("interface-output");
  
  // Verify that wheel-based nodes are consistent with drops
  if (vec_len (cbsm->wheel_by_thread) == 0)
    {
      // Allocate wheel per thread
      vec_validate (cbsm->wheel_by_thread, vlib_get_thread_main ()->n_vlib_mains - 1);
    }
  
  // Add required next nodes to cbs-wheel input node to fix the invalid next_index errors
  u32 input_node_index = vlib_get_node_by_name(vm, (u8*)"cbs-wheel")->index;
  
  // Add ethernet-input as a valid next node (matching CBS_NEXT_PUNT=1)
  vlib_node_add_next (vm, input_node_index, vlib_get_node_by_name(vm, (u8*)"ethernet-input")->index);
  
  // Add known next nodes for standard lookup paths
  u32 ip4_lookup_node_index = vlib_get_node_by_name(vm, (u8*)"ip4-lookup")->index;
  vlib_node_add_next (vm, input_node_index, ip4_lookup_node_index);
  
  // Add interface output nodes when hardware interfaces are available
  if (0) // Only do this if needed and interfaces exist, for now disabled to avoid crash
  {
    vnet_hw_interface_t *hi0, *hi1;
    hi0 = vnet_get_hw_interface (cbsm->vnet_main, 0);
    if (hi0)
      cbsm->output_next_index0 = vlib_node_add_next (vm, input_node_index, hi0->output_node_index);
    
    hi1 = vnet_get_hw_interface (cbsm->vnet_main, 1);
    if (hi1)
      cbsm->output_next_index1 = vlib_node_add_next (vm, input_node_index, hi1->output_node_index);
  }

  return 0;
}

VLIB_INIT_FUNCTION (cbs_init);

/* Feature registrations */
VNET_FEATURE_INIT (cbs_cross_connect, static) =
{
  .arc_name = "device-input",
  .node_name = "cbs-cross-connect",
  .runs_before = VNET_FEATURES ("ethernet-input"),
};

VNET_FEATURE_INIT (cbs_output_feature, static) = {
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
  s = format (s, "%.2f Kbps/sec", slope_kbps); return s;
}

static u8 *
format_cbs_config (u8 * s, va_list * args)
{
  cbs_main_t *cbsm = &cbs_main;
  int verbose = va_arg (*args, int);

  s = format (s, "CBS Configuration:\n");
  s = format (s, "  Port Rate:       %U\n", format_cbs_rate, cbsm->cbs_port_rate);
  s = format (s, "  Idle Slope:      %U\n", format_cbs_slope, cbsm->cbs_idleslope);
  s = format (s, "  Send Slope:      %U\n", format_cbs_slope, cbsm->cbs_sendslope);
  s = format (s, "  HiCredit:        %lld bytes\n", cbsm->cbs_hicredit); // Use %lld
  s = format (s, "  LoCredit:        %lld bytes\n", cbsm->cbs_locredit); // Use %lld

  s = format (s, "Optional Features:\n");
  if (cbsm->drop_fraction > 0) s = format (s, "  Drop Fraction:   %.5f (1 per %.0f pkts)\n", cbsm->drop_fraction, 1.0/cbsm->drop_fraction);
  else s = format (s, "  Drop Fraction:   0 (disabled)\n");
  if (cbsm->reorder_fraction > 0) s = format (s, "  Reorder Fraction:%.5f (1 per %.0f pkts)\n", cbsm->reorder_fraction, 1.0/cbsm->reorder_fraction);
  else s = format (s, "  Reorder Fraction:0 (disabled)\n");

  s = format (s, "Internal Sizing:\n");
  s = format (s, "  Avg Packet Size: %u bytes\n", cbsm->packet_size);
  s = format (s, "  Bandwidth Used:  %U (for wheel sizing)\n", format_cbs_rate, cbsm->bandwidth / CBS_BITS_PER_BYTE);
  s = format (s, "  Wheel Size:      %u slots/worker\n", cbsm->wheel_slots_per_wrk);

  if (verbose) { s = format (s, "  Poll Main Thread:%u\n", cbsm->poll_main_thread); }

   s = format (s, "\nEnabled Interfaces:\n");
   if (cbsm->sw_if_index0 != ~0) {
        s = format (s, "  Cross-connect: %U <--> %U\n", format_vnet_sw_if_index_name, cbsm->vnet_main, cbsm->sw_if_index0, format_vnet_sw_if_index_name, cbsm->vnet_main, cbsm->sw_if_index1);
   } else if (vec_len(cbsm->output_next_index_by_sw_if_index) > 0) {
       s = format (s, "  Output Feature on:\n"); u32 i;
       for (i = 0; i < vec_len (cbsm->output_next_index_by_sw_if_index); i++) {
         if (i < vec_len(cbsm->output_next_index_by_sw_if_index) && cbsm->output_next_index_by_sw_if_index[i] != ~0)
           s = format (s, "    %U\n", format_vnet_sw_if_index_name, cbsm->vnet_main, i);
       }
   } else { s = format(s, "  None\n"); }

  return s;
}

static clib_error_t *
set_cbs_command_fn (vlib_main_t * vm, unformat_input_t * input, vlib_cli_command_t * cmd)
{
  cbs_main_t *cbsm = &cbs_main;
  f64 port_rate_bps = 0.0, idleslope_kbps = 0.0, bandwidth_bps = 0.0;
  i64 hicredit_bytes = 0, locredit_bytes = 0; // Use i64
  u32 packet_size = 0;
  u32 packets_per_drop = 0, packets_per_reorder = 0;
  f64 drop_fraction = 0.0, reorder_fraction = 0.0;
  int rv;
  int port_rate_set = 0, idleslope_set = 0, hicredit_set = 0, locredit_set = 0;

  while (unformat_check_input (input) != UNFORMAT_END_OF_INPUT) {
      if (unformat (input, "port_rate %U", unformat_cbs_rate, &port_rate_bps)) port_rate_set = 1;
      else if (unformat (input, "idleslope %U", unformat_cbs_slope, &idleslope_kbps)) idleslope_set = 1;
      else if (unformat (input, "hicredit %lld", &hicredit_bytes)) hicredit_set = 1; // Use %lld
      else if (unformat (input, "locredit %lld", &locredit_bytes)) locredit_set = 1; // Use %lld
      else if (unformat (input, "bandwidth %U", unformat_cbs_rate, &bandwidth_bps));
      else if (unformat (input, "packet-size %u", &packet_size));
      else if (unformat (input, "packets-per-drop %u", &packets_per_drop));
      else if (unformat (input, "packets-per-reorder %u", &packets_per_reorder));
      else if (unformat (input, "drop-fraction %f", &drop_fraction)) { if (drop_fraction < 0.0 || drop_fraction > 1.0) return clib_error_return(0, "drop fraction must be 0.0-1.0"); }
      else if (unformat (input, "reorder-fraction %f", &reorder_fraction)) { if (reorder_fraction < 0.0 || reorder_fraction > 1.0) return clib_error_return(0, "reorder fraction must be 0.0-1.0"); }
      else return clib_error_return (0, "unknown input '%U'", format_unformat_error, input);
    }

  if (!port_rate_set || !idleslope_set || !hicredit_set || !locredit_set) return clib_error_return (0, "Mandatory parameters missing: port_rate, idleslope, hicredit, locredit");

  if (packets_per_drop > 0 && drop_fraction == 0.0) drop_fraction = 1.0 / (f64) (packets_per_drop);
  if (packets_per_reorder > 0 && reorder_fraction == 0.0) reorder_fraction = 1.0 / (f64) packets_per_reorder;

  rv = cbs_configure_internal (cbsm, port_rate_bps, idleslope_kbps, hicredit_bytes, locredit_bytes, bandwidth_bps, packet_size, drop_fraction, reorder_fraction);

  switch (rv) {
    case 0: vlib_cli_output (vm, "%U", format_cbs_config, 0); break;
    case VNET_API_ERROR_INVALID_VALUE: return clib_error_return (0, "Invalid port_rate (> 0)");
    case VNET_API_ERROR_INVALID_VALUE_2: return clib_error_return (0, "Invalid idleslope (> 0)");
    case VNET_API_ERROR_INVALID_VALUE_3: return clib_error_return (0, "Invalid credits (hicredit >= locredit)");
    case VNET_API_ERROR_INVALID_VALUE_4: return clib_error_return (0, "Invalid packet size (64-9000)");
    // Use VNET_API_ERROR_UNSPECIFIED for allocation failure
    case VNET_API_ERROR_UNSPECIFIED: return clib_error_return(0, "Configuration failed (e.g., memory allocation)");
    default: return clib_error_return (0, "cbs_configure_internal failed: %d", rv);
  }
  return 0;
}

/*? %%clicmd:fn set_cbs_command_fn%% */
/*?
 * Configure the Credit Based Shaper (CBS).
 * Sets core CBS parameters (port_rate, idleslope, hicredit, locredit).
 * Optional parameters control loss/reorder and internal buffer sizing.
 *
 * Mandatory: port_rate <rate>, idleslope <kbps>, hicredit <bytes>, locredit <bytes>
 * Optional: [bandwidth <rate>] [packet-size <bytes>] [packets-per-drop <n> | drop-fraction <f>] [packets-per-reorder <n> | reorder-fraction <f>]
 *
 * @cliexpar
 * @clistart
 * set cbs port_rate 1 gbps idleslope 10000 kbps hicredit 1500 locredit -1500
 * @cliend
 * @cliexcmd{set cbs port_rate <rate> idleslope <kbps> hicredit <bytes> locredit <bytes> [options]}
?*/
VLIB_CLI_COMMAND (set_cbs_command, static) =
{
  .path = "set cbs",
  .short_help = "set cbs port_rate <rate> idleslope <kbps> hicredit <bytes> locredit <bytes> [options]",
  .function = set_cbs_command_fn,
};


static clib_error_t *
show_cbs_command_fn (vlib_main_t * vm, unformat_input_t * input, vlib_cli_command_t * cmd)
{
  cbs_main_t *cbsm = &cbs_main;
  int verbose = 0;
  if (unformat (input, "verbose")) verbose = 1;
  else if (unformat_check_input(input) != UNFORMAT_END_OF_INPUT) return unformat_parse_error (input);

  if (cbsm->is_configured == 0) return clib_error_return (0, "CBS not configured. Use 'set cbs ...'");

  vlib_cli_output (vm, "%U", format_cbs_config, verbose);
  return 0;
}

/*? %%clicmd:fn show_cbs_command_fn%% */
/*?
 * Display state info for the CBS plugin.
 * Shows configured CBS parameters, optional features, internal sizing,
 * and enabled interfaces. Use 'verbose' for more details.
 * @cliexpar
 * @clistart
 * show cbs
 * show cbs verbose
 * @cliend
 * @cliexcmd{show cbs [verbose]}
?*/
VLIB_CLI_COMMAND (show_cbs_command, static) =
{
  .path = "show cbs",
  .short_help = "Display CBS configuration",
  .function = show_cbs_command_fn,
};

/*? %%clicmd:fn cbs_cross_connect_enable_disable_command_fn%% */
/*?
 * Enable or disable CBS cross-connect between two interfaces.
 * Requires prior configuration using 'set cbs ...'.
 * @cliexpar
 * @clistart
 * cbs cross-connect enable-disable GigabitEthernet0/0/0 GigabitEthernet0/0/1
 * cbs cross-connect enable-disable GigabitEthernet0/0/0 GigabitEthernet0/0/1 disable
 * @cliend
 * @cliexcmd{cbs cross-connect enable-disable <interface1> <interface2> [disable]}
?*/
VLIB_CLI_COMMAND (cbs_enable_disable_command, static) =
{
  .path = "cbs cross-connect enable-disable",
  .short_help = "cbs cross-connect enable-disable <intfc1> <intfc2> [disable]",
  .function = cbs_cross_connect_enable_disable_command_fn,
};

/*? %%clicmd:fn cbs_output_feature_enable_disable_command_fn%% */
/*?
 * Enable or disable the CBS output feature on an interface.
 * Requires prior configuration using 'set cbs ...'.
 * @cliexpar
 * @clistart
 * cbs output-feature enable-disable GigabitEthernet0/0/0
 * cbs output-feature enable-disable GigabitEthernet0/0/0 disable
 * @cliend
 * @cliexcmd{cbs output-feature enable-disable <interface> [disable]}
?*/
VLIB_CLI_COMMAND (cbs_output_feature_enable_disable_command, static) =
{
  .path = "cbs output-feature enable-disable",
  .short_help = "cbs output-feature enable-disable <interface> [disable]",
  .function = cbs_output_feature_enable_disable_command_fn,
};

/* Include the auto-generated API C file *AFTER* handler definitions */
#include <cbs/cbs.api.c>

/*
 * fd.io coding-style-patch-verification: ON
 *
 * Local Variables:
 * eval: (c-set-style "gnu")
 * End:
 */