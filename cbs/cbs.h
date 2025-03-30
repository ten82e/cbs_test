/*
 * cbs.h - VPP CBS Plugin header file
 *
 * Copyright (c) 2024 Your Org <your.email@example.com> // Placeholder
 * Based on nsim.h, Copyright (c) Cisco and/or its affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef __included_cbs_h__ // Renamed include guard
#define __included_cbs_h__

#include <vnet/vnet.h>
#include <vnet/ip/ip.h>
#include <vnet/ethernet/ethernet.h>
#include <vnet/feature/feature.h>

#include <vppinfra/hash.h>
#include <vppinfra/error.h>
#include <vppinfra/time.h>
#include <vppinfra/clib.h> // Include for i64 type

#define CBS_MAX_TX_BURST 32	/**< max packets in a tx burst from CBS wheel */

/** \brief CBS Wheel Entry (renamed) */
typedef struct
{
  /** \brief Buffer index */
  u32 buffer_index;
  
  /** \brief RX interface index */
  u32 rx_sw_if_index;
  
  /** \brief TX interface index */
  u32 tx_sw_if_index;
  
  /** \brief Output next index */
  u32 output_next_index;

  /** \brief Ethernet type - needed for processing decision */
  u16 ethertype;
} cbs_wheel_entry_t; /** < renamed wheel entry */

typedef struct
{
  u32 wheel_size;
  u32 cursize; // Number of packets currently in the wheel
  u32 head;    // Index to dequeue next
  u32 tail;    // Index to enqueue next

  /* CBS State Variables */
  f64 cbs_credits;            /* Current credit balance in bytes (can be negative) */
  f64 cbs_last_update_time;   /* Last time credits were updated (vlib time) */
  f64 cbs_last_tx_finish_time;/* Estimated finish time of the last transmitted packet (vlib time) */

  cbs_wheel_entry_t *entries; // Uses renamed struct
    CLIB_CACHE_LINE_ALIGN_MARK (pad);
} cbs_wheel_t; // Renamed struct

// Context structure for the packet processing node (node.c)
typedef struct cbs_node_ctx // Renamed struct
{
  vnet_feature_config_main_t *fcm;
  u32 *drop;
  u32 *reord;
  u16 *reord_nexts;
  u32 *fwd; // For packets bypassing the wheel (e.g., reordered, or maybe future features)
  u16 *fwd_nexts;
  u8 *action; // For loss/reorder simulation
  u32 n_buffered; // Packets added to the wheel
  u32 n_loss;
  u32 n_reordered;
} cbs_node_ctx_t; // Renamed typedef

#define foreach_cbs_action			\
  _(DROP, "Packet loss")			\
  _(REORDER, "Packet reorder")		\
  _(BUFFER, "Packet buffered")

enum cbs_action_bit // Renamed enum
{
#define _(sym, str) CBS_ACTION_##sym##_BIT, // Renamed prefix
  foreach_cbs_action
#undef _
};

typedef enum cbs_action // Renamed enum
{
#define _(sym, str) CBS_ACTION_##sym = 1 << CBS_ACTION_##sym##_BIT, // Renamed prefix
  foreach_cbs_action
#undef _
} cbs_action_e; // Renamed typedef

typedef struct
{
  /* API message ID base */
  u16 msg_id_base;

  /* Feature arc index (for output feature mode) */
  u16 arc_index;

  /* Cross-connect state */
  u32 sw_if_index0, sw_if_index1;
  u32 output_next_index0, output_next_index1;

  /* Output feature state */
  u32 *output_next_index_by_sw_if_index;

  /* Random seed for loss/reorder */
  u32 seed;

  /* Per-thread scheduler wheels */
  cbs_wheel_t **wheel_by_thread; // Uses renamed struct

  /* Config parameters */
  f64 bandwidth;          /* Configured bandwidth for wheel sizing (bits/sec) */
  u32 packet_size;        /* Average packet size for wheel sizing */
  f64 drop_fraction;      /* 0.0 to 1.0 */
  f64 reorder_fraction;   /* 0.0 to 1.0 */
  u32 wheel_slots_per_wrk;/* Calculated wheel size */
  u32 poll_main_thread;   /* Option to poll main thread wheel */

  /* CBS Config parameters */
  f64 cbs_idleslope;    /* Credit accumulation rate (bytes/sec) */
  f64 cbs_sendslope;    /* Credit depletion rate (bytes/sec, usually negative) */
  i64 cbs_hicredit;     /* Max credits (bytes) - Changed s64 to i64 */
  i64 cbs_locredit;     /* Min credits (bytes, usually negative) - Changed s64 to i64 */
  f64 cbs_port_rate;    /* Port transmission rate (bytes/sec) */

  u64 mmap_size; // Size of memory map for each wheel

  /* State */
  int is_configured; // Flag indicating if CBS parameters are set

  /* convenience */
  vlib_main_t *vlib_main;
  vnet_main_t *vnet_main;
  clib_time_t clib_time; // For time functions

} cbs_main_t; // Renamed struct and typedef

extern cbs_main_t cbs_main; // Renamed global variable

// Renamed node registrations
extern vlib_node_registration_t cbs_cross_connect_node;
extern vlib_node_registration_t cbs_output_feature_node;
extern vlib_node_registration_t cbs_input_node;

// Define bytes/sec conversion constants
#define CBS_BITS_PER_BYTE 8.0
#define CBS_KBPS_TO_BPS 1000.0
#define CBS_MBPS_TO_BPS 1000000.0
#define CBS_GBPS_TO_BPS 1000000000.0

#endif /* __included_cbs_h__ */

/*
 * fd.io coding-style-patch-verification: ON
 *
 * Local Variables:
 * eval: (c-set-style "gnu")
 * End:
 */