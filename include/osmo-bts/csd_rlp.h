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

extern const uint8_t csd_tchf48_nt_e2_map[26];

/* Per TS 48.020 section 15.1, the cadence of E2+E3 bits in a properly
 * aligned sequence of pseudo-V.110 frames forming a single RLP frame
 * is 00-01-10-11.  The following constant captures this bit sequence
 * in hex, for comparison against align_bits output from
 * csd_v110_rtp_decode() or against rlpdl_align_bits accumulator
 * in CSD NT lchan state.
 */
#define	NTCSD_ALIGNED_EBITS	0x1B

void ntcsd_dl_reset(struct gsm_lchan *lchan);
void ntcsd_dl_input_48(struct gsm_lchan *lchan, const ubit_t *data_bits,
			uint8_t align_bits);
void ntcsd_dl_input_96(struct gsm_lchan *lchan, const ubit_t *data_bits,
			uint8_t align_bits);
bool ntcsd_dl_output(struct gsm_lchan *lchan, ubit_t *rlp_frame_out);

void gsmtap_csd_rlp_process(struct gsm_lchan *lchan, bool is_uplink,
			    const struct ph_tch_param *tch_ind,
			    const ubit_t *data, unsigned int data_len);
void gsmtap_csd_rlp_dl(struct gsm_lchan *lchan, uint32_t fn,
			const ubit_t *data, unsigned int data_len);
