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
#include <vppinfra/clib.h> // For cache line alignment macro
#include <vlib/log.h>      // Include for vlib_log_class_t

// Constants
#define CBS_MAX_TX_BURST 8         /**< Max packets to dequeue in one go from wheel (Reduced from 32) */
#define CBS_DEFAULT_PACKET_SIZE 1500 /**< Default average packet size if not specified */
#define CBS_BITS_PER_BYTE 8.0
#define CBS_KBPS_TO_BPS 1000.0
#define CBS_MBPS_TO_BPS 1000000.0
#define CBS_GBPS_TO_BPS 1000000000.0
#define CBS_MIN_WHEEL_SLOTS 2048    /**< Minimum guaranteed slots in the wheel */

/** \brief CBS Wheel Entry (stores packet info in the queue) */
typedef struct
{
  u32 buffer_index;       /**< Index of the buffered packet */
  u32 rx_sw_if_index;     /**< Original RX software interface index */
  u32 tx_sw_if_index;     /**< Target TX software interface index (after potential cross-connect change) */
  u32 output_next_index;  /**< Next node index *after* the cbs-wheel node */
  u16 pad[2];             /**< Padding for alignment */
} cbs_wheel_entry_t;

/** \brief CBS Wheel Structure (per thread) */
typedef struct
{
  u32 wheel_size;         /**< Total number of slots in this wheel */
  u32 cursize;            /**< Current number of packets in the wheel */
  u32 head;               /**< Index to dequeue from */
  u32 tail;               /**< Index to enqueue to */
  f64 cbs_credits;        /**< Current credit balance for this thread/queue */
  f64 cbs_last_update_time; /**< Time when credits were last updated */
  f64 cbs_last_tx_finish_time; /**< Time when the last packet transmission from this wheel finished */
  // f64 cbs_last_poll_time; // Optional: For reducing log spam when wheel is empty
  cbs_wheel_entry_t *entries; /**< Pointer to the array of wheel entries */
    CLIB_CACHE_LINE_ALIGN_MARK (pad); /**< Ensure structure ends on a cache line boundary */
} cbs_wheel_t;


/** \brief Trace actions for enqueue node (node.c) */
typedef enum {
    CBS_TRACE_ACTION_BUFFER,            /**< Packet buffered into the wheel */
    CBS_TRACE_ACTION_DROP_WHEEL_FULL,   /**< Packet dropped because the wheel was full */
    CBS_TRACE_ACTION_DROP_LOOKUP_FAIL,  /**< Packet dropped due to lookup failure */
} cbs_trace_action_t;


/** \brief Context structure for the enqueue nodes (node.c) */
typedef struct cbs_node_ctx
{
  u32 *drop;          /**< Pointer to array for dropped buffer indices */
  u32 n_buffered;     /**< Number of packets buffered to the wheel in this frame */
  // u32 n_lookup_drop;  /**< Number of packets dropped due to lookup failure (removed) */
} cbs_node_ctx_t;


/** \brief Main CBS Plugin State */
typedef struct
{
  /* Plugin infrastructure */
  u16 msg_id_base;    /**< API message ID base */
  vlib_main_t *vlib_main; /**< VLIB main pointer */
  vnet_main_t *vnet_main; /**< VNET main pointer */
  vlib_log_class_t log_class; /**< Log class for this plugin */

  /* Feature arcs */
  u16 arc_index;      /**< Index for the "interface-output" feature arc */

  /* Configuration State */
  int is_configured;    /**< Flag indicating if CBS parameters are set */

  /* CBS Parameters (converted to bytes/sec where applicable) */
  f64 cbs_port_rate;    /**< Port rate in bytes/sec */
  f64 cbs_idleslope;    /**< Idle slope in bytes/sec */
  f64 cbs_sendslope;    /**< Send slope in bytes/sec (idleslope - port_rate) */
  f64 cbs_hicredit;     /**< High credit limit in bytes */
  f64 cbs_locredit;     /**< Low credit limit in bytes */

  /* Wheel Sizing Parameters */
  u32 packet_size;      /**< Average packet size hint (bytes) */
  f64 configured_bandwidth; /**< Bandwidth hint used for wheel sizing (bytes/sec) */
  u32 wheel_slots_per_wrk; /**< Number of slots per worker thread wheel */

  /* Per-thread data */
  cbs_wheel_t **wheel_by_thread; /**< Vector of pointers to per-thread wheels */

  /* Cross Connect specific state */
  u32 sw_if_index0;     /**< First sw_if_index for cross-connect mode (~0 if not used) */
  u32 sw_if_index1;     /**< Second sw_if_index for cross-connect mode (~0 if not used) */
  u32 output_next_index0; /**< Next node index after wheel for sw_if_index0 output */
  u32 output_next_index1; /**< Next node index after wheel for sw_if_index1 output */

  /* Output Feature specific state */
  u32 *output_next_index_by_sw_if_index; /**< Vector mapping sw_if_index to next node index after wheel */

} cbs_main_t;

extern cbs_main_t cbs_main;

// Node registrations (defined in respective .c files)
extern vlib_node_registration_t cbs_cross_connect_node;
extern vlib_node_registration_t cbs_output_feature_node;
extern vlib_node_registration_t cbs_input_node; // The dequeue node ("cbs-wheel")

#endif /* __included_cbs_h__ */