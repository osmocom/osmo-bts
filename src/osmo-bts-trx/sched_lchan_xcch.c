/*
 * (C) 2013 by Andreas Eversberg <jolly@eversberg.eu>
 * (C) 2015-2017 by Harald Welte <laforge@gnumonks.org>
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
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdint.h>
#include <errno.h>

#include <osmocom/core/bits.h>
#include <osmocom/core/utils.h>
#include <osmocom/coding/gsm0503_coding.h>

#include <osmo-bts/bts.h>
#include <osmo-bts/l1sap.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/scheduler.h>
#include <osmo-bts/scheduler_backend.h>

#include <sched_utils.h>

/*! \brief a single (SDCCH/SACCH) burst was received by the PHY, process it */
int rx_data_fn(struct l1sched_trx *l1t, enum trx_chan_type chan,
	       uint8_t bid, const struct trx_ul_burst_ind *bi)
{
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, bi->tn);
	struct l1sched_chan_state *chan_state = &l1ts->chan_state[chan];
	sbit_t *burst, **bursts_p = &chan_state->ul_bursts;
	uint32_t *first_fn = &chan_state->ul_first_fn;
	uint8_t *mask = &chan_state->ul_mask;
	float *rssi_sum = &chan_state->rssi_sum;
	uint8_t *rssi_num = &chan_state->rssi_num;
	int32_t *toa256_sum = &chan_state->toa256_sum;
	uint8_t *toa_num = &chan_state->toa_num;
	int32_t *ci_cb_sum = &chan_state->ci_cb_sum;
	uint8_t *ci_cb_num = &chan_state->ci_cb_num;
	uint8_t l2[GSM_MACBLOCK_LEN], l2_len;
	int n_errors = 0;
	int n_bits_total = 0;
	int16_t lqual_cb;
	uint16_t ber10k;
	int rc;

	/* If handover RACH detection is turned on, treat this burst as an Access Burst.
	 * Handle NOPE.ind as usually to ensure proper Uplink measurement reporting. */
	if (chan_state->ho_rach_detect == 1 && ~bi->flags & TRX_BI_F_NOPE_IND)
		return rx_rach_fn(l1t, chan, bid, bi);

	LOGL1S(DL1P, LOGL_DEBUG, l1t, bi->tn, chan, bi->fn,
	       "Received Data, bid=%u\n", bid);

	/* allocate burst memory, if not already */
	if (!*bursts_p) {
		*bursts_p = talloc_zero_size(tall_bts_ctx, 464);
		if (!*bursts_p)
			return -ENOMEM;
	}

	/* clear burst & store frame number of first burst */
	if (bid == 0) {
		memset(*bursts_p, 0, 464);
		*mask = 0x0;
		*first_fn = bi->fn;
		*rssi_sum = 0;
		*rssi_num = 0;
		*toa256_sum = 0;
		*toa_num = 0;
		*ci_cb_sum = 0;
		*ci_cb_num = 0;
	}

	/* update mask + RSSI */
	*mask |= (1 << bid);
	*rssi_sum += bi->rssi;
	(*rssi_num)++;
	*toa256_sum += bi->toa256;
	(*toa_num)++;

	/* C/I: Carrier-to-Interference ratio (in centiBels) */
	if (bi->flags & TRX_BI_F_CI_CB) {
		*ci_cb_sum += bi->ci_cb;
		(*ci_cb_num)++;
	}

	/* Copy burst to buffer of 4 bursts. If the burst indication contains
	 * no data, ensure that the buffer does not stay uninitialized */
	burst = *bursts_p + bid * 116;
	if (bi->burst_len > 0) {
		memcpy(burst, bi->burst + 3, 58);
		memcpy(burst + 58, bi->burst + 87, 58);
	} else
		memset(burst, 0, 58 * 2);

	/* wait until complete set of bursts */
	if (bid != 3)
		return 0;

	/* check for complete set of bursts */
	if ((*mask & 0xf) != 0xf) {
		LOGL1S(DL1P, LOGL_NOTICE, l1t, bi->tn, chan, bi->fn,
			"Received incomplete data (%u/%u)\n",
			bi->fn % l1ts->mf_period, l1ts->mf_period);

		/* we require first burst to have correct FN */
		if (!(*mask & 0x1)) {
			*mask = 0x0;
			return 0;
		}
	}
	*mask = 0x0;

	/* decode */
	rc = gsm0503_xcch_decode(l2, *bursts_p, &n_errors, &n_bits_total);
	if (rc) {
		LOGL1S(DL1P, LOGL_NOTICE, l1t, bi->tn, chan, bi->fn,
			"Received bad data (%u/%u)\n",
			bi->fn % l1ts->mf_period, l1ts->mf_period);
		l2_len = 0;
	} else
		l2_len = GSM_MACBLOCK_LEN;

	lqual_cb = *ci_cb_num ? (*ci_cb_sum / *ci_cb_num) : 0;
	ber10k = compute_ber10k(n_bits_total, n_errors);
	return _sched_compose_ph_data_ind(l1t, bi->tn, *first_fn,
					  chan, l2, l2_len,
					  *rssi_sum / *rssi_num,
					  *toa256_sum / *toa_num,
					  lqual_cb, ber10k,
					  PRES_INFO_UNKNOWN);
}

/* obtain a to-be-transmitted xCCH (e.g SACCH or SDCCH) burst */
int tx_data_fn(struct l1sched_trx *l1t, enum trx_chan_type chan,
	       uint8_t bid, struct trx_dl_burst_req *br)
{
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, br->tn);
	struct gsm_bts_trx_ts *ts = &l1t->trx->ts[br->tn];
	struct msgb *msg = NULL; /* make GCC happy */
	ubit_t *burst, **bursts_p = &l1ts->chan_state[chan].dl_bursts;

	/* send burst, if we already got a frame */
	if (bid > 0) {
		if (!*bursts_p)
			return 0;
		goto send_burst;
	}

	/* get mac block from queue */
	msg = _sched_dequeue_prim(l1t, br->tn, br->fn, chan);
	if (msg)
		goto got_msg;

	LOGL1S(DL1P, LOGL_INFO, l1t, br->tn, chan, br->fn, "No prim for transmit.\n");

no_msg:
	/* free burst memory */
	if (*bursts_p) {
		talloc_free(*bursts_p);
		*bursts_p = NULL;
	}
	return -ENODEV;

got_msg:
	/* check validity of message */
	if (msgb_l2len(msg) != GSM_MACBLOCK_LEN) {
		LOGL1S(DL1P, LOGL_FATAL, l1t, br->tn, chan, br->fn, "Prim not 23 bytes, please FIX! "
			"(len=%d)\n", msgb_l2len(msg));
		/* free message */
		msgb_free(msg);
		goto no_msg;
	}

	/* BURST BYPASS */

	/* handle loss detection of SACCH */
	if (L1SAP_IS_LINK_SACCH(trx_chan_desc[chan].link_id)) {
		/* count and send BFI */
		if (++(l1ts->chan_state[chan].lost_frames) > 1) {
			/* TODO: Should we pass old TOA here? Otherwise we risk
			 * unnecessary decreasing TA */

			/* Note: RSSI is set to 0 to indicate to the higher
			 * layers that this is a faked ph_data_ind */
			_sched_compose_ph_data_ind(l1t, br->tn, 0, chan, NULL, 0,
						   0, 0, 0, 10000,
						   PRES_INFO_INVALID);
		}
	}

	/* allocate burst memory, if not already */
	if (!*bursts_p) {
		*bursts_p = talloc_zero_size(tall_bts_ctx, 464);
		if (!*bursts_p)
			return -ENOMEM;
	}

	/* encode bursts */
	gsm0503_xcch_encode(*bursts_p, msg->l2h);

	/* free message */
	msgb_free(msg);

send_burst:
	/* compose burst */
	burst = *bursts_p + bid * 116;
	memcpy(br->burst + 3, burst, 58);
	memcpy(br->burst + 61, _sched_tsc[gsm_ts_tsc(ts)], 26);
	memcpy(br->burst + 87, burst + 58, 58);

	br->burst_len = GSM_BURST_LEN;

	LOGL1S(DL1P, LOGL_DEBUG, l1t, br->tn, chan, br->fn, "Transmitting burst=%u.\n", bid);

	return 0;
}
