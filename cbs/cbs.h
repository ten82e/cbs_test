/*
 * cbs.h - VPP CBS Plugin header file
 * Pure CBS shaper based on nsim structure. No loss/reorder simulation.
 *
 * Copyright (c) 2024 Your Org <your.email@example.com> // Placeholder
 * Based on nsim.h, Copyright (c) Cisco and/or its affiliates.
 * Licensed under the Apache License, Version 2.0 (the "License");
 */
#ifndef __included_cbs_h__
#define __included_cbs_h__

#include <vnet/vnet.h>
#include <vnet/ip/ip.h>
#include <vnet/ethernet/ethernet.h>
#include <vnet/feature/feature.h>

#include <vppinfra/hash.h>
#include <vppinfra/error.h>
#include <vppinfra/time.h>
// #include <vppinfra/random.h> // Removed as seed/fractions removed
#include <vppinfra/clib.h>

#define CBS_MAX_TX_BURST 32
#define CBS_DEFAULT_PACKET_SIZE 1500

/** \brief CBS Wheel Entry */
typedef struct
{
  u32 buffer_index;
  u32 rx_sw_if_index;
  u32 tx_sw_if_index;
  u32 output_next_index;
  // u16 ethertype; // Optional: Keep only if needed for debugging wheel contents
  u16 pad[2]; // Adjusted padding
} cbs_wheel_entry_t;

/** \brief CBS Wheel Structure (per thread) */
typedef struct
{
  u32 wheel_size;
  u32 cursize;
  u32 head;
  u32 tail;
  f64 cbs_credits;
  f64 cbs_last_update_time;
  f64 cbs_last_tx_finish_time;
  cbs_wheel_entry_t *entries;
    CLIB_CACHE_LINE_ALIGN_MARK (pad);
} cbs_wheel_t;


// Actions removed

// Define enum for node.c trace actions explicitly
// Removed DROP_SIM and REORDER_SIM
typedef enum {
    CBS_TRACE_ACTION_BUFFER,
    CBS_TRACE_ACTION_DROP_WHEEL_FULL,
    // Add other trace points if needed
} cbs_trace_action_t;


/** \brief Context structure for the packet processing node (node.c) */
typedef struct cbs_node_ctx
{
  u32 *drop;          /**< Pointer to array for dropped buffer indices (wheel full) */
  u32 n_buffered;     /**< Number of packets buffered to the wheel in this frame */
  // Removed action_flags, n_loss, reord, reord_nexts, n_reordered
} cbs_node_ctx_t;


/** \brief Main CBS Plugin State */
typedef struct
{
  u16 msg_id_base;
  u16 arc_index;
  u32 sw_if_index0, sw_if_index1;
  u32 output_next_index0, output_next_index1;
  u32 *output_next_index_by_sw_if_index;
  // u32 seed; // Removed
  cbs_wheel_t **wheel_by_thread;
  u32 packet_size;
  // f64 drop_fraction;  // Removed
  // f64 reorder_fraction; // Removed
  u32 wheel_slots_per_wrk;
  // u32 poll_main_thread; // Removed
  f64 cbs_idleslope;
  f64 cbs_sendslope;
  f64 cbs_hicredit;
  f64 cbs_locredit;
  f64 cbs_port_rate;
  f64 configured_bandwidth;
  int is_configured;
  vlib_main_t *vlib_main;
  vnet_main_t *vnet_main;
} cbs_main_t;

extern cbs_main_t cbs_main;

// Node registrations
extern vlib_node_registration_t cbs_cross_connect_node;
extern vlib_node_registration_t cbs_output_feature_node;
extern vlib_node_registration_t cbs_input_node; // The dequeue node

// Define bytes/sec conversion constants
#define CBS_BITS_PER_BYTE 8.0
#define CBS_KBPS_TO_BPS 1000.0
#define CBS_MBPS_TO_BPS 1000000.0
#define CBS_GBPS_TO_BPS 1000000000.0

#endif /* __included_cbs_h__ */

/* ... (fd.io tags etc.) ... */