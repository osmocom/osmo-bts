/* This module has been split from l1sap.c; original header comments preserved:
 *
 * (C) 2011 by Harald Welte <laforge@gnumonks.org>
 * (C) 2013 by Andreas Eversberg <jolly@eversberg.eu>
 *
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>

#include <osmocom/core/bits.h>
#include <osmocom/core/msgb.h>
#include <osmocom/gsm/l1sap.h>
#include <osmocom/gsm/gsm_utils.h>
#include <osmocom/gsm/rsl.h>
#include <osmocom/gsm/rlp.h>
#include <osmocom/gsm/rtp_extensions.h>
#include <osmocom/core/gsmtap.h>
#include <osmocom/core/gsmtap_util.h>
#include <osmocom/core/utils.h>

#include <osmo-bts/logging.h>
#include <osmo-bts/gsm_data.h>
#include <osmo-bts/lchan.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/csd_rlp.h>

/* In the case of TCH/F4.8 NT, each 240-bit RLP frame is split between
 * two channel-coding blocks of 120 bits each.  We need to know which
 * frame numbers correspond to which half: in the UL-to-RTP path we have
 * to set bit E2 based on the TDMA frame number at which we received the
 * block in question, and in the DL direction we have to transmit the
 * right half at the right time.
 *
 * See GSM 05.03 section 3.4.1 and the mapping tables of GSM 05.02;
 * having "e2_map" in the array name shall serve as a mnemonic as to
 * the sense of this array: 0 means 1st half and 1 means 2nd half,
 * exactly as the value of bit E2 per TS 48.020 section 15.1.
 */
const uint8_t csd_tchf48_nt_e2_map[26] = {
	[4]  = 1,	/* B1 position */
	[13] = 1,	/* B3 position */
	[21] = 1,	/* B5 position */
};

/* This function resets (clears) the state of the DL alignment buffer.
 * It needs to be called when we encounter a gap (packet loss, invalid
 * packets, etc) in our RTP input stream. */
void ntcsd_dl_reset(struct gsm_lchan *lchan)
{
	lchan->tch.csd.rlpdl_fill_level = 0;
}

/* This function is to be called with the decoded content of a single
 * incoming RTP packet (data and alignment bits) for TCH/[FH]4.8 NT. */
void ntcsd_dl_input_48(struct gsm_lchan *lchan, const ubit_t *data_bits,
			uint8_t align_bits)
{
	memmove(lchan->tch.csd.rlpdl_data_bits,
		lchan->tch.csd.rlpdl_data_bits + 60 * 2, 60 * 5);
	memcpy(lchan->tch.csd.rlpdl_data_bits + 60 * 5, data_bits, 60 * 2);
	lchan->tch.csd.rlpdl_align_bits <<= 4;
	lchan->tch.csd.rlpdl_align_bits |= (align_bits & 0xF);
	lchan->tch.csd.rlpdl_fill_level += 2;
	if (lchan->tch.csd.rlpdl_fill_level > 7)
		lchan->tch.csd.rlpdl_fill_level = 7;
}

/* This function is to be called with the decoded content of a single
 * incoming RTP packet (data and alignment bits) for TCH/F9.6 NT. */
void ntcsd_dl_input_96(struct gsm_lchan *lchan, const ubit_t *data_bits,
			uint8_t align_bits)
{
	memmove(lchan->tch.csd.rlpdl_data_bits,
		lchan->tch.csd.rlpdl_data_bits + 60 * 4, 60 * 3);
	memcpy(lchan->tch.csd.rlpdl_data_bits + 60 * 3, data_bits, 60 * 4);
	lchan->tch.csd.rlpdl_align_bits <<= 8;
	lchan->tch.csd.rlpdl_align_bits |= (align_bits & 0xFF);
	lchan->tch.csd.rlpdl_fill_level += 4;
	if (lchan->tch.csd.rlpdl_fill_level > 7)
		lchan->tch.csd.rlpdl_fill_level = 7;
}

/* This function is to be called to obtain a complete RLP frame for
 * downlink transmission.  It will provide either a properly aligned
 * frame (return value true) or a filler (return value false). */
bool ntcsd_dl_output(struct gsm_lchan *lchan, ubit_t *rlp_frame_out)
{
	if (lchan->tch.csd.rlpdl_fill_level < 4)
		goto no_frame_out;
	if (((lchan->tch.csd.rlpdl_align_bits >> 0) & 0xFF) == NTCSD_ALIGNED_EBITS) {
		memcpy(rlp_frame_out, lchan->tch.csd.rlpdl_data_bits + 60 * 3,
			60 * 4);
		return true;
	}
	if (lchan->tch.csd.rlpdl_fill_level < 5)
		goto no_frame_out;
	if (((lchan->tch.csd.rlpdl_align_bits >> 2) & 0xFF) == NTCSD_ALIGNED_EBITS) {
		memcpy(rlp_frame_out, lchan->tch.csd.rlpdl_data_bits + 60 * 2,
			60 * 4);
		return true;
	}
	if (lchan->tch.csd.rlpdl_fill_level < 6)
		goto no_frame_out;
	if (((lchan->tch.csd.rlpdl_align_bits >> 4) & 0xFF) == NTCSD_ALIGNED_EBITS) {
		memcpy(rlp_frame_out, lchan->tch.csd.rlpdl_data_bits + 60 * 1,
			60 * 4);
		return true;
	}
	if (lchan->tch.csd.rlpdl_fill_level < 7)
		goto no_frame_out;
	if (((lchan->tch.csd.rlpdl_align_bits >> 6) & 0xFF) == NTCSD_ALIGNED_EBITS) {
		memcpy(rlp_frame_out, lchan->tch.csd.rlpdl_data_bits, 60 * 4);
		return true;
	}
no_frame_out:
	/* TS 44.021 section 12.1 says that a missing/unavailable 240-bit
	 * RLP frame is to be filled with 0 bits, unlike ones-fill
	 * used everywhere else in the world of V.110 and CSD. */
	memset(rlp_frame_out, 0, 60 * 4);
	return false;
}

/* process one MAC block of unpacked bits of a non-transparent CSD channel */
void gsmtap_csd_rlp_process(struct gsm_lchan *lchan, bool is_uplink,
			    const struct ph_tch_param *tch_ind,
			    const ubit_t *data, unsigned int data_len)
{
	struct gsm_bts_trx *trx = lchan->ts->trx;
	struct gsmtap_inst *inst = trx->bts->gsmtap.inst;
	pbit_t *rlp_buf;
	uint16_t arfcn;
	int byte_len;

	if (!inst || !trx->bts->gsmtap.rlp)
		return;

	if (lchan->csd_mode != LCHAN_CSD_M_NT)
		return;

	if (is_uplink)
		rlp_buf = lchan->tch.csd.rlp_buf_ul;
	else
		rlp_buf = lchan->tch.csd.rlp_buf_dl;

	/* TCH/F 9.6: 4x60bit block => 240bit RLP frame
	 * TCH/F 4.8: 2x 2x60bit blocks starting at B0/B2/B4 => 240bit RLP frame
	 * TCH/H 4.8: 4x60bit block => 240bit RLP frame
	 * TCH/F 2.4: 2x36bit blocks => transparent only
	 * TCH/H 2.4: 4x36bit blocks => transparent only
	 * TCH/F 14.4: 2x 290 bit block (starting with M1=0) => 576-bit RLP frame
	 */

	if (lchan->type == GSM_LCHAN_TCH_F &&
	    lchan->tch_mode == GSM48_CMODE_DATA_6k0 && is_uplink) {
		/* In this mode we have 120-bit MAC blocks; two of them need
		 * to be concatenated to render a 240-bit RLP frame. The first
		 * block is present in B0/B2/B4, and we have to use FN to
		 * detect this position.
		 * This code path is only for UL: in the case of DL,
		 * alignment logic elsewhere in the code will present us
		 * with a fully assembled RLP frame. */
		OSMO_ASSERT(data_len == 120);
		if (csd_tchf48_nt_e2_map[tch_ind->fn % 26] == 0) {
			osmo_ubit2pbit_ext(rlp_buf, 0, data, 0, data_len, 1);
			return;
		}
		osmo_ubit2pbit_ext(rlp_buf, 120, data, 0, data_len, 1);
		byte_len = 240/8;
	} else if (lchan->type == GSM_LCHAN_TCH_F && lchan->tch_mode == GSM48_CMODE_DATA_14k5) {
		/* in this mode we have 290bit MAC blocks containing M1, M2 and 288 data bits;
		 * two of them need to be concatenated to render a
		 * 576-bit RLP frame. The start of a RLP frame is
		 * denoted by a block with M1-bit set to 0. */
		OSMO_ASSERT(data_len == 290);
		ubit_t m1 = data[0];
		if (m1 == 0) {
			osmo_ubit2pbit_ext(rlp_buf, 0, data, 2, data_len, 1);
			return;
		}
		osmo_ubit2pbit_ext(rlp_buf, 288, data, 2, data_len, 1);
		byte_len = 576/8;
	} else {
		byte_len = osmo_ubit2pbit_ext(rlp_buf, 0, data, 0, data_len, 1);
	}

	if (trx->bts->gsmtap.rlp_skip_null) {
		struct osmo_rlp_frame_decoded rlpf;
		int rc = osmo_rlp_decode(&rlpf, 0, rlp_buf, byte_len);
		if (rc == 0 && rlpf.ftype == OSMO_RLP_FT_U && rlpf.u_ftype == OSMO_RLP_U_FT_NULL)
			return;
	}

	arfcn = trx->arfcn;
	if (is_uplink)
		arfcn |= GSMTAP_ARFCN_F_UPLINK;

	gsmtap_send_ex(inst, GSMTAP_TYPE_GSM_RLP, arfcn, lchan->ts->nr,
		       lchan->type == GSM_LCHAN_TCH_H ? GSMTAP_CHANNEL_VOICE_H : GSMTAP_CHANNEL_VOICE_F,
		       lchan->nr, tch_ind->fn, tch_ind->rssi, 0, rlp_buf, byte_len);

}

/* wrapper for downlink path */
void gsmtap_csd_rlp_dl(struct gsm_lchan *lchan, uint32_t fn,
			const ubit_t *data, unsigned int data_len)
{
	/* 'fake' tch_ind containing all-zero so gsmtap code can be shared
	 * between UL and DL */
	const struct ph_tch_param fake_tch_ind = { .fn = fn };
	gsmtap_csd_rlp_process(lchan, false, &fake_tch_ind, data, data_len);
}
