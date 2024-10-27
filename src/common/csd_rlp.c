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

	if (lchan->type == GSM_LCHAN_TCH_F && lchan->tch_mode == GSM48_CMODE_DATA_6k0) {
		/* in this mode we have 120bit MAC blocks; two of them need to be concatenated
		 * to render a 240-bit RLP frame. The fist block is present in B0/B2/B4.
		 * The E7 bit is used to indicate the Frame MF0a */
		OSMO_ASSERT(data_len == 120);
		ubit_t e7 = data[4*7+3];
		if (e7 == 0) {
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
