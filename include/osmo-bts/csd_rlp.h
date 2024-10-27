/*
 * Declarations for functions in csd_rlp.c: alignment of downlink RLP frames
 * and RLP GSMTAP mechanism for CSD NT modes.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <osmocom/core/bits.h>
#include <osmocom/gsm/l1sap.h>
#include <osmo-bts/lchan.h>

void gsmtap_csd_rlp_process(struct gsm_lchan *lchan, bool is_uplink,
			    const struct ph_tch_param *tch_ind,
			    const ubit_t *data, unsigned int data_len);
