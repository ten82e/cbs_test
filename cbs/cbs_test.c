/*
 * cbs_test.c - VPP API test C file for CBS plugin
 *
 * Copyright (c) 2024 Your Org <your.email@example.com> // Placeholder
 * Based on nsim_test.c, Copyright (c) Cisco and/or its affiliates.
 * Licensed under the Apache License, Version 2.0 (the "License");
 */
#include <vat/vat.h>
#include <vlibapi/api.h>
#include <vlibmemory/api.h>
#include <vppinfra/error.h>
#include <vppinfra/byte_order.h>
#include <vppinfra/format.h>
#include <vppinfra/clib.h>

#include <cbs/cbs.h>

// Declare unformat_sw_if_index
uword unformat_sw_if_index (unformat_input_t * input, va_list * args);

/* Declare message IDs */
#include <cbs/cbs.api_enum.h>
#include <cbs/cbs.api_types.h>

typedef struct
{
  u16 msg_id_base;
  vat_main_t *vat_main;
} cbs_test_main_t;

cbs_test_main_t cbs_test_main;

#define __plugin_msg_base cbs_test_main.msg_id_base
#include <vlibapi/vat_helper_macros.h>

/* VAT test function for cbs_cross_connect_enable_disable */
static int
api_cbs_cross_connect_enable_disable (vat_main_t * vam)
{
  unformat_input_t *i = vam->input;
  int enable_disable = 1;
  u32 sw_if_index0 = ~0, sw_if_index1 = ~0;
  u32 tmp_if_index = ~0;
  vl_api_cbs_cross_connect_enable_disable_t *mp;
  int ret;

  while (unformat_check_input (i) != UNFORMAT_END_OF_INPUT) {
      if (unformat (i, "disable")) enable_disable = 0;
      else if (unformat (i, "%U", unformat_sw_if_index, vam, &tmp_if_index)) {
          if (sw_if_index0 == ~0) sw_if_index0 = tmp_if_index;
          else if (sw_if_index1 == ~0) sw_if_index1 = tmp_if_index;
          else { errmsg ("Specify only two interfaces\n"); return -99; }
          tmp_if_index = ~0;
      } else if (unformat (i, "sw_if_index %u", &tmp_if_index)) {
          if (sw_if_index0 == ~0) sw_if_index0 = tmp_if_index;
          else if (sw_if_index1 == ~0) sw_if_index1 = tmp_if_index;
          else { errmsg ("Specify only two interfaces\n"); return -99; }
          tmp_if_index = ~0;
      }
      else break;
    }

  if (sw_if_index0 == ~0 || sw_if_index1 == ~0) { errmsg ("missing interface (need two)\n"); return -99; }

  M (CBS_CROSS_CONNECT_ENABLE_DISABLE, mp);
  mp->sw_if_index0 = htonl (sw_if_index0);
  mp->sw_if_index1 = htonl (sw_if_index1);
  mp->enable_disable = enable_disable;

  S (mp); W (ret); return ret;
}

/* VAT test function for cbs_output_feature_enable_disable */
static int
api_cbs_output_feature_enable_disable (vat_main_t * vam)
{
  unformat_input_t *i = vam->input;
  int enable_disable = 1;
  u32 sw_if_index = ~0;
  vl_api_cbs_output_feature_enable_disable_t *mp;
  int ret;

  while (unformat_check_input (i) != UNFORMAT_END_OF_INPUT) {
      if (unformat (i, "disable")) enable_disable = 0;
      else if (unformat (i, "%U", unformat_sw_if_index, vam, &sw_if_index)) ;
      else if (unformat (i, "sw_if_index %u", &sw_if_index)) ;
      else break;
    }

  if (sw_if_index == ~0) { errmsg ("missing interface\n"); return -99; }

  M (CBS_OUTPUT_FEATURE_ENABLE_DISABLE, mp);
  mp->sw_if_index = htonl (sw_if_index);
  mp->enable_disable = enable_disable;

  S (mp); W (ret); return ret;
}

/* VAT unformatters matching CLI unformatters */
static uword unformat_vat_cbs_rate (unformat_input_t * input, va_list * args) {
  f64 *r = va_arg (*args, f64 *); f64 t;
  if (unformat(input,"%f gbps", &t)||unformat(input,"%f gbit",&t)) *r = t*CBS_GBPS_TO_BPS;
  else if (unformat(input,"%f mbps",&t)||unformat(input,"%f mbit",&t)) *r = t*CBS_MBPS_TO_BPS;
  else if (unformat(input,"%f kbps",&t)||unformat(input,"%f kbit",&t)) *r = t*CBS_KBPS_TO_BPS;
  else if (unformat(input,"%f bps",&t)||unformat(input,"%f bit",&t)) *r = t; else return 0; return 1;
}
static uword unformat_vat_cbs_slope (unformat_input_t * input, va_list * args) {
  f64 *r = va_arg(*args, f64*); f64 t;
  if (unformat(input,"%f kbps",&t)||unformat(input,"%f kbit",&t)) *r = t; else return 0; return 1;
}

/* VAT test function for cbs_configure */
static int
api_cbs_configure (vat_main_t * vam)
{
  unformat_input_t *i = vam->input;
  vl_api_cbs_configure_t *mp;
  f64 port_rate_bps = 0.0, idleslope_kbps = 0.0, bandwidth_bps = 0.0;
  f64 hicredit_f = 0.0, locredit_f = 0.0;
  i32 hicredit_bytes = 0, locredit_bytes = 0;
  u32 packet_size = 0;
  // u32 packets_per_drop = 0; // Removed
  int ret;
  int port_rate_set = 0, idleslope_set = 0, hicredit_set = 0, locredit_set = 0;

  /* Parse args */
  while (unformat_check_input (i) != UNFORMAT_END_OF_INPUT) {
      if (unformat (i, "port_rate %U", unformat_vat_cbs_rate, &port_rate_bps)) port_rate_set = 1;
      else if (unformat (i, "idleslope %U", unformat_vat_cbs_slope, &idleslope_kbps)) idleslope_set = 1;
      else if (unformat (i, "hicredit %f", &hicredit_f)) hicredit_set = 1;
      else if (unformat (i, "locredit %f", &locredit_f)) locredit_set = 1;
      else if (unformat (i, "bandwidth %U", unformat_vat_cbs_rate, &bandwidth_bps));
      else if (unformat (i, "packet-size %u", &packet_size));
      // else if (unformat (i, "packets-per-drop %u", &packets_per_drop)); // Removed
      else { errmsg ("unknown input '%U'", format_unformat_error, i); return -99; }
    }

  if (!port_rate_set || !idleslope_set || !hicredit_set || !locredit_set) {
       errmsg ("Mandatory params missing: port_rate, idleslope, hicredit, locredit\n");
       return -99;
  }

  hicredit_bytes = (i32)hicredit_f;
  locredit_bytes = (i32)locredit_f;

  M (CBS_CONFIGURE, mp);
  mp->port_rate_bps = clib_host_to_net_u64 ((u64)port_rate_bps);
  mp->idleslope_kbps = clib_host_to_net_u64 ((u64)idleslope_kbps);
  mp->hicredit_bytes = htonl (hicredit_bytes);
  mp->locredit_bytes = htonl (locredit_bytes);
  mp->average_packet_size = htonl (packet_size);
  mp->bandwidth_in_bits_per_second = clib_host_to_net_u64 ((u64)bandwidth_bps);
  // mp->packets_per_drop = htonl (packets_per_drop); // Removed

  S (mp); W (ret); return ret;
}


/* Include the auto-generated VAT test C file */
#include <cbs/cbs.api_test.c>

/* ... (fd.io tags etc.) ... */