/*
 * (C) 2013 by Andreas Eversberg <jolly@eversberg.eu>
 * (C) 2015-2017 by Harald Welte <laforge@gnumonks.org>
 * (C) 2019 by Vadim Yanitskiy <axilirator@gmail.com>
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
#include <limits.h>
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

/* 3GPP TS 05.02, section 5.2.7 */
#define RACH_EXT_TAIL_LEN	8
#define RACH_SYNCH_SEQ_LEN	41

enum rach_synch_seq_t {
	RACH_SYNCH_SEQ_UNKNOWN = -1,
	RACH_SYNCH_SEQ_TS0, /* GSM, GMSK (default) */
	RACH_SYNCH_SEQ_TS1, /* EGPRS, 8-PSK */
	RACH_SYNCH_SEQ_TS2, /* EGPRS, GMSK */
	RACH_SYNCH_SEQ_NUM
};

static struct value_string rach_synch_seq_names[] = {
	{ RACH_SYNCH_SEQ_UNKNOWN,	"UNKNOWN" },
	{ RACH_SYNCH_SEQ_TS0,		"TS0: GSM, GMSK" },
	{ RACH_SYNCH_SEQ_TS1,		"TS1: EGPRS, 8-PSK" },
	{ RACH_SYNCH_SEQ_TS2,		"TS2: EGPRS, GMSK" },
	{ 0, NULL },
};

static enum rach_synch_seq_t rach_get_synch_seq(sbit_t *bits, int *best_score)
{
	sbit_t *synch_seq_burst = bits + RACH_EXT_TAIL_LEN;
	enum rach_synch_seq_t seq = RACH_SYNCH_SEQ_TS0;
	int score[RACH_SYNCH_SEQ_NUM] = { 0 };
	int max_score = INT_MIN;
	int i, j;

	/* 3GPP TS 05.02, section 5.2.7 "Access burst (AB)", synch. sequence bits */
	static const char synch_seq_ref[RACH_SYNCH_SEQ_NUM][RACH_SYNCH_SEQ_LEN] = {
		[RACH_SYNCH_SEQ_TS0] = "01001011011111111001100110101010001111000",
		[RACH_SYNCH_SEQ_TS1] = "01010100111110001000011000101111001001101",
		[RACH_SYNCH_SEQ_TS2] = "11101111001001110101011000001101101110111",
	};

	/* Get a multiplier for j-th bit of i-th synch. sequence */
#define RACH_SYNCH_SEQ_MULT \
	(synch_seq_ref[i][j] == '1' ? -1 : 1)

	/* For each synch. sequence, count the bit match score. Since we deal with
	 * soft-bits (-127...127), we sum the absolute values of matching ones,
	 * and subtract the absolute values of different ones, so the resulting
	 * score is more accurate than it could be with hard-bits. */
	for (i = 0; i < RACH_SYNCH_SEQ_NUM; i++) {
		for (j = 0; j < RACH_SYNCH_SEQ_LEN; j++)
			score[i] += RACH_SYNCH_SEQ_MULT * synch_seq_burst[j];

		/* Keep the maximum value updated */
		if (score[i] > max_score) {
			max_score = score[i];
			seq = i;
		}
	}

	/* Calculate an approximate level of our confidence */
	if (best_score != NULL)
		*best_score = max_score;

	/* At least 1/3 of a synch. sequence shall match */
	if (max_score < (127 * RACH_SYNCH_SEQ_LEN / 3))
		return RACH_SYNCH_SEQ_UNKNOWN;

	return seq;
}

int rx_rach_fn(struct l1sched_ts *l1ts, const struct trx_ul_burst_ind *bi)
{
	struct gsm_bts_trx *trx = l1ts->ts->trx;
	struct osmo_phsap_prim l1sap;
	int n_errors = 0;
	int n_bits_total = 0;
	uint16_t ra11;
	uint8_t ra;
	int rc;

	/* Ignore NOPE indications, they're of no use here */
	if (bi->flags & TRX_BI_F_NOPE_IND)
		return 0;

	/* TSC (Training Sequence Code) is an optional parameter of the UL burst
	 * indication. We need this information in order to decide whether an
	 * Access Burst is 11-bit encoded or not (see OS#1854). If this information
	 * is absent, we try to correlate the received synch. sequence with the
	 * known ones (3GPP TS 05.02, section 5.2.7), and fall-back to the default
	 * TS0 if it fails. */
	enum rach_synch_seq_t synch_seq = RACH_SYNCH_SEQ_TS0;
	int best_score = 127 * RACH_SYNCH_SEQ_LEN;

	/* If logical channel is not either of RACH, PDTCH or PTCCH, this is a
	 * handover Access Burst, which is always encoded as 8-bit and shall
	 * contain the generic training sequence (TS0). */
	if (bi->chan == TRXC_RACH || bi->chan == TRXC_PDTCH || bi->chan == TRXC_PTCCH) {
		if (bi->flags & TRX_BI_F_TS_INFO)
			synch_seq = (enum rach_synch_seq_t) bi->tsc;
		else
			synch_seq = rach_get_synch_seq((sbit_t *) bi->burst, &best_score);
	}

	LOGL1SB(DL1P, LOGL_DEBUG, l1ts, bi,
	       "Received%s RACH (%s): rssi=%d toa256=%d",
	       TRX_CHAN_IS_DEDIC(bi->chan) ? " handover" : "",
	       get_value_string(rach_synch_seq_names, synch_seq),
	       bi->rssi, bi->toa256);
	if (bi->flags & TRX_BI_F_CI_CB)
		LOGPC(DL1P, LOGL_DEBUG, " C/I=%d cB", bi->ci_cb);
	else
		LOGPC(DL1P, LOGL_DEBUG, " match=%.1f%%",
		      best_score * 100.0 / (127 * RACH_SYNCH_SEQ_LEN));
	LOGPC(DL1P, LOGL_DEBUG, "\n");

	/* Compose a new L1SAP primitive */
	memset(&l1sap, 0x00, sizeof(l1sap));
	osmo_prim_init(&l1sap.oph, SAP_GSM_PH, PRIM_PH_RACH, PRIM_OP_INDICATION, NULL);
	l1sap.u.rach_ind.chan_nr = trx_chan_desc[bi->chan].chan_nr | bi->tn;
	l1sap.u.rach_ind.acc_delay = (bi->toa256 >= 0) ? bi->toa256 / 256 : 0;
	l1sap.u.rach_ind.acc_delay_256bits = bi->toa256;
	l1sap.u.rach_ind.rssi = bi->rssi;
	l1sap.u.rach_ind.fn = bi->fn;

	/* Link quality is defined by C/I (Carrier-to-Interference ratio),
	 * which has optional presence. If it's absent, report the
	 * minimum acceptable value to pass L1SAP checks. */
	if (bi->flags & TRX_BI_F_CI_CB)
		l1sap.u.rach_ind.lqual_cb = bi->ci_cb;
	else
		l1sap.u.rach_ind.lqual_cb = trx->bts->min_qual_rach;

	/* Decode RACH depending on its synch. sequence */
	switch (synch_seq) {
	case RACH_SYNCH_SEQ_TS1:
	case RACH_SYNCH_SEQ_TS2:
		rc = gsm0503_rach_ext_decode_ber(&ra11, bi->burst + RACH_EXT_TAIL_LEN + RACH_SYNCH_SEQ_LEN,
						 trx->bts->bsic, &n_errors, &n_bits_total);
		if (rc) {
			LOGL1SB(DL1P, LOGL_DEBUG, l1ts, bi, "Received bad Access Burst\n");
			return 0;
		}

		if (synch_seq == RACH_SYNCH_SEQ_TS1)
			l1sap.u.rach_ind.burst_type = GSM_L1_BURST_TYPE_ACCESS_1;
		else
			l1sap.u.rach_ind.burst_type = GSM_L1_BURST_TYPE_ACCESS_2;

		l1sap.u.rach_ind.is_11bit = 1;
		l1sap.u.rach_ind.ra = ra11;
		break;

	case RACH_SYNCH_SEQ_TS0:
	default:
		/* Fall-back to the default TS0 if needed */
		if (synch_seq != RACH_SYNCH_SEQ_TS0) {
			LOGL1SB(DL1P, LOGL_DEBUG, l1ts, bi, "Falling-back to the default TS0\n");
			synch_seq = RACH_SYNCH_SEQ_TS0;
		}

		rc = gsm0503_rach_decode_ber(&ra, bi->burst + RACH_EXT_TAIL_LEN + RACH_SYNCH_SEQ_LEN,
					     trx->bts->bsic, &n_errors, &n_bits_total);
		if (rc) {
			LOGL1SB(DL1P, LOGL_DEBUG, l1ts, bi, "Received bad Access Burst\n");
			return 0;
		}

		l1sap.u.rach_ind.burst_type = GSM_L1_BURST_TYPE_ACCESS_0;
		l1sap.u.rach_ind.is_11bit = 0;
		l1sap.u.rach_ind.ra = ra;
		break;
	}

	l1sap.u.rach_ind.ber10k = compute_ber10k(n_bits_total, n_errors);

	/* forward primitive */
	l1sap_up(trx, &l1sap);

	return 0;
}
