/*
 * (C) 2013 by Andreas Eversberg <jolly@eversberg.eu>
 * (C) 2015-2017 by Harald Welte <laforge@gnumonks.org>
 * Contributions by sysmocom - s.f.m.c. GmbH
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

/* Add two arrays of sbits */
static void add_sbits(sbit_t * current, const sbit_t * previous)
{
	unsigned int i;
	for (i = 0; i < 464; i++) {
		*current = (*current) / 2 + (*previous) / 2;
		current++;
		previous++;
	}
}

/*! \brief a single (SDCCH/SACCH) burst was received by the PHY, process it */
int rx_data_fn(struct l1sched_ts *l1ts, const struct trx_ul_burst_ind *bi)
{
	struct l1sched_chan_state *chan_state = &l1ts->chan_state[bi->chan];
	sbit_t *burst, **bursts_p = &chan_state->ul_bursts;
	uint32_t *first_fn = &chan_state->ul_first_fn;
	uint8_t *mask = &chan_state->ul_mask;
	uint8_t l2[GSM_MACBLOCK_LEN], l2_len;
	struct l1sched_meas_set meas_avg;
	int n_errors = 0;
	int n_bits_total = 0;
	uint16_t ber10k;
	int rc;
	struct gsm_lchan *lchan = chan_state->lchan;
	bool rep_sacch = L1SAP_IS_LINK_SACCH(trx_chan_desc[bi->chan].link_id) && lchan->rep_acch.ul_sacch_active;

	/* If handover RACH detection is turned on, treat this burst as an Access Burst.
	 * Handle NOPE.ind as usually to ensure proper Uplink measurement reporting. */
	if (chan_state->ho_rach_detect == 1 && ~bi->flags & TRX_BI_F_NOPE_IND)
		return rx_rach_fn(l1ts, bi);

	LOGL1SB(DL1P, LOGL_DEBUG, l1ts, bi, "Received Data, bid=%u\n", bi->bid);

	/* clear burst & store frame number of first burst */
	if (bi->bid == 0) {
		memset(*bursts_p, 0, 464);
		*mask = 0x0;
		*first_fn = bi->fn;
	}

	/* update mask */
	*mask |= (1 << bi->bid);

	/* store measurements */
	trx_sched_meas_push(chan_state, bi);

	/* Copy burst to buffer of 4 bursts. If the burst indication contains
	 * no data, ensure that the buffer does not stay uninitialized */
	burst = *bursts_p + bi->bid * 116;
	if (bi->burst_len > 0) {
		memcpy(burst, bi->burst + 3, 58);
		memcpy(burst + 58, bi->burst + 87, 58);
	} else
		memset(burst, 0, 58 * 2);

	/* wait until complete set of bursts */
	if (bi->bid != 3)
		return 0;

	/* average measurements of the last 4 bursts */
	trx_sched_meas_avg(chan_state, &meas_avg, SCHED_MEAS_AVG_M_S4N4);

	/* check for complete set of bursts */
	if ((*mask & 0xf) != 0xf) {
		LOGL1SB(DL1P, LOGL_NOTICE, l1ts, bi, "Received incomplete data (%u/%u)\n",
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
		LOGL1SB(DL1P, LOGL_NOTICE, l1ts, bi, "Received bad data (%u/%u)\n",
			bi->fn % l1ts->mf_period, l1ts->mf_period);
		l2_len = 0;

		/* When SACCH Repetition is active, we may try to decode the
		 * current SACCH block by including the information from the
		 * information from the previous SACCH block. See also:
		 * 3GPP TS 44.006, section 11.2 */
		if (rep_sacch) {
			add_sbits(*bursts_p, chan_state->ul_bursts_prev);
			rc = gsm0503_xcch_decode(l2, *bursts_p, &n_errors, &n_bits_total);
			if (rc) {
				LOGL1SB(DL1P, LOGL_NOTICE, l1ts, bi,
				       "Combining current SACCH block with previous SACCH block also yields bad data (%u/%u)\n",
				       bi->fn % l1ts->mf_period, l1ts->mf_period);
			} else {
				LOGL1SB(DL1P, LOGL_DEBUG, l1ts, bi,
				       "Combining current SACCH block with previous SACCH block yields good data (%u/%u)\n",
				       bi->fn % l1ts->mf_period, l1ts->mf_period);
				l2_len = GSM_MACBLOCK_LEN;
			}
		}
	} else
		l2_len = GSM_MACBLOCK_LEN;

	ber10k = compute_ber10k(n_bits_total, n_errors);

	/* Keep a copy to ease decoding in the next repetition pass */
	if (rep_sacch)
		memcpy(chan_state->ul_bursts_prev, *bursts_p, 464);

	return _sched_compose_ph_data_ind(l1ts, *first_fn,
					  bi->chan, l2, l2_len,
					  meas_avg.rssi, meas_avg.toa256,
					  meas_avg.ci_cb, ber10k,
					  PRES_INFO_UNKNOWN);
}

/* obtain a to-be-transmitted xCCH (e.g SACCH or SDCCH) burst */
int tx_data_fn(struct l1sched_ts *l1ts, struct trx_dl_burst_req *br)
{
	struct msgb *msg = NULL; /* make GCC happy */
	ubit_t *burst, **bursts_p = &l1ts->chan_state[br->chan].dl_bursts;

	/* send burst, if we already got a frame */
	if (br->bid > 0)
		goto send_burst;

	/* get mac block from queue */
	msg = _sched_dequeue_prim(l1ts, br);
	if (msg == NULL) {
		LOGL1SB(DL1P, LOGL_INFO, l1ts, br, "No prim for transmit.\n");
		return -ENODEV;
	}

	/* check validity of message */
	if (msgb_l2len(msg) != GSM_MACBLOCK_LEN) {
		LOGL1SB(DL1P, LOGL_FATAL, l1ts, br, "Prim has odd len=%u != %u\n",
			msgb_l2len(msg), GSM_MACBLOCK_LEN);
		/* free message */
		msgb_free(msg);
		return -EINVAL;
	}

	/* BURST BYPASS */

	/* encode bursts */
	gsm0503_xcch_encode(*bursts_p, msg->l2h);

	/* free message */
	msgb_free(msg);

send_burst:
	/* compose burst */
	burst = *bursts_p + br->bid * 116;
	memcpy(br->burst + 3, burst, 58);
	memcpy(br->burst + 61, TRX_GMSK_NB_TSC(br), 26);
	memcpy(br->burst + 87, burst + 58, 58);

	br->burst_len = GSM_BURST_LEN;

	LOGL1SB(DL1P, LOGL_DEBUG, l1ts, br, "Transmitting burst=%u.\n", br->bid);

	return 0;
}
