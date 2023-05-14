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

/* Maximum size of a EGPRS message in bytes */
#define EGPRS_0503_MAX_BYTES	155

/*! \brief a single PDTCH burst was received by the PHY, process it */
int rx_pdtch_fn(struct l1sched_ts *l1ts, const struct trx_ul_burst_ind *bi)
{
	struct l1sched_chan_state *chan_state = &l1ts->chan_state[bi->chan];
	sbit_t *burst, *bursts_p = chan_state->ul_bursts;
	uint32_t first_fn;
	uint8_t *mask = &chan_state->ul_mask;
	struct l1sched_meas_set meas_avg;
	uint8_t l2[EGPRS_0503_MAX_BYTES];
	int n_errors = 0;
	int n_bursts_bits = 0;
	int n_bits_total = 0;
	uint16_t ber10k;
	int rc;
	enum osmo_ph_pres_info_type presence_info;

	LOGL1SB(DL1P, LOGL_DEBUG, l1ts, bi, "Received PDTCH bid=%u\n", bi->bid);

	/* An MS may be polled to send an ACK in form of four Access Bursts */
	if (bi->flags & TRX_BI_F_ACCESS_BURST)
		return rx_rach_fn(l1ts, bi);

	/* clear burst */
	if (bi->bid == 0) {
		memset(bursts_p, 0, GSM0503_EGPRS_BURSTS_NBITS);
		*mask = 0x0;
	}

	/* update mask */
	*mask |= (1 << bi->bid);

	/* store measurements */
	trx_sched_meas_push(chan_state, bi);

	/* copy burst to buffer of 4 bursts */
	switch (bi->burst_len) {
	case EGPRS_BURST_LEN:
		burst = bursts_p + bi->bid * 348;
		memcpy(burst, bi->burst + 9, 174);
		memcpy(burst + 174, bi->burst + 261, 174);
		n_bursts_bits = GSM0503_EGPRS_BURSTS_NBITS;
		break;
	case GSM_BURST_LEN:
		burst = bursts_p + bi->bid * 116;
		memcpy(burst, bi->burst + 3, 58);
		memcpy(burst + 58, bi->burst + 87, 58);
		n_bursts_bits = GSM0503_GPRS_BURSTS_NBITS;
		break;
	case 0:
		/* NOPE.ind, assume GPRS? */
		burst = bursts_p + bi->bid * 116;
		memset(burst, 0, 116);
		n_bursts_bits = GSM0503_GPRS_BURSTS_NBITS;
	}

	/* wait until complete set of bursts */
	if (bi->bid != 3)
		return 0;

	/* average measurements of the last 4 bursts */
	trx_sched_meas_avg(chan_state, &meas_avg, SCHED_MEAS_AVG_M_S4N4);

	/* check for complete set of bursts */
	if ((*mask & 0xf) != 0xf) {
		LOGL1SB(DL1P, LOGL_DEBUG, l1ts, bi, "Received incomplete frame (%u/%u)\n",
			bi->fn % l1ts->mf_period, l1ts->mf_period);
	}
	*mask = 0x0;

	/*
	 * Attempt to decode EGPRS bursts first. For 8-PSK EGPRS this is all we
	 * do. Attempt GPRS decoding on EGPRS failure. If the burst is GPRS,
	 * then we incur decoding overhead of 31 bits on the Type 3 EGPRS
	 * header, which is tolerable.
	 */
	rc = gsm0503_pdtch_egprs_decode(l2, bursts_p, n_bursts_bits,
				NULL, &n_errors, &n_bits_total);

	if ((bi->burst_len == GSM_BURST_LEN) && (rc < 0)) {
		rc = gsm0503_pdtch_decode(l2, bursts_p, NULL,
				  &n_errors, &n_bits_total);
	}

	if (rc > 0) {
		presence_info = PRES_INFO_BOTH;
	} else {
		LOGL1SB(DL1P, LOGL_DEBUG, l1ts, bi, "Received bad PDTCH (%u/%u)\n",
			bi->fn % l1ts->mf_period, l1ts->mf_period);
		rc = 0;
		presence_info = PRES_INFO_INVALID;
	}

	ber10k = compute_ber10k(n_bits_total, n_errors);

	first_fn = GSM_TDMA_FN_SUB(bi->fn, 3);
	return _sched_compose_ph_data_ind(l1ts,
					  first_fn, bi->chan, l2, rc,
					  meas_avg.rssi, meas_avg.toa256,
					  meas_avg.ci_cb, ber10k,
					  presence_info);
}

/* obtain a to-be-transmitted PDTCH (packet data) burst */
int tx_pdtch_fn(struct l1sched_ts *l1ts, struct trx_dl_burst_req *br)
{
	struct msgb *msg = NULL; /* make GCC happy */
	ubit_t *burst, *bursts_p = l1ts->chan_state[br->chan].dl_bursts;
	enum trx_mod_type *mod = &l1ts->chan_state[br->chan].dl_mod_type;
	int rc = 0;

	/* send burst, if we already got a frame */
	if (br->bid > 0)
		goto send_burst;

	/* get mac block from queue */
	msg = _sched_dequeue_prim(l1ts, br);
	if (!msg) {
		/* It's totally fine to get no block here, since PCU may submit
		 * empty blocks when there's no MS listening. The scheduler will
		 * take care of filling C0 with dummy bursts to keep expected
		 * power transmit levels
		 */
		return -ENODEV;
	}

	/* BURST BYPASS */

	/* encode bursts */
	rc = gsm0503_pdtch_egprs_encode(bursts_p, msg->l2h, msg->tail - msg->l2h);
	if (rc < 0)
		rc = gsm0503_pdtch_encode(bursts_p, msg->l2h, msg->tail - msg->l2h);

	/* check validity of message */
	if (rc < 0) {
		LOGL1SB(DL1P, LOGL_FATAL, l1ts, br, "Prim invalid length, please FIX! "
			"(len=%ld)\n", (long)(msg->tail - msg->l2h));
		/* free message */
		msgb_free(msg);
		return -EINVAL;
	} else if (rc == GSM0503_EGPRS_BURSTS_NBITS) {
		*mod = TRX_MOD_T_8PSK;
	} else {
		*mod = TRX_MOD_T_GMSK;
	}

	/* free message */
	msgb_free(msg);

send_burst:
	/* compose burst */
	if (*mod == TRX_MOD_T_8PSK) {
		burst = bursts_p + br->bid * 348;
		memset(br->burst, 1, 9);
		memcpy(br->burst + 9, burst, 174);
		memcpy(br->burst + 183, TRX_8PSK_NB_TSC(br), 78);
		memcpy(br->burst + 261, burst + 174, 174);
		memset(br->burst + 435, 1, 9);

		br->burst_len = EGPRS_BURST_LEN;
	} else {
		burst = bursts_p + br->bid * 116;
		memcpy(br->burst + 3, burst, 58);
		memcpy(br->burst + 61, TRX_GMSK_NB_TSC(br), 26);
		memcpy(br->burst + 87, burst + 58, 58);

		br->burst_len = GSM_BURST_LEN;
	}

	LOGL1SB(DL1P, LOGL_DEBUG, l1ts, br, "Transmitting burst=%u.\n", br->bid);

	return 0;
}
