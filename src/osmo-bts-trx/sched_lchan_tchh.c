/*
 * (C) 2013 by Andreas Eversberg <jolly@eversberg.eu>
 * (C) 2015-2017 by Harald Welte <laforge@gnumonks.org>
 * (C) 2020-2023 by sysmocom - s.f.m.c. GmbH <info@sysmocom.de>
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
#include <osmocom/core/msgb.h>
#include <osmocom/core/utils.h>

#include <osmocom/gsm/gsm0502.h>

#include <osmocom/codec/codec.h>

#include <osmocom/coding/gsm0503_coding.h>
#include <osmocom/coding/gsm0503_amr_dtx.h>

#include <osmocom/netif/amr.h>

#include <osmo-bts/bts.h>
#include <osmo-bts/l1sap.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/scheduler.h>
#include <osmo-bts/scheduler_backend.h>
#include <osmo-bts/msg_utils.h>

#include <sched_utils.h>
#include <amr_loop.h>

/* 3GPP TS 45.009, table 3.2.1.3-{2,4}: AMR on Uplink TCH/H.
 *
 * +---+---+---+---+---+---+
 * | a | b | c | d | e | f |  Burst 'a' received first
 * +---+---+---+---+---+---+
 *  ^^^^^^^^^^^^^^^^^^^^^^^   FACCH frame  (bursts 'a' .. 'f')
 *  ^^^^^^^^^^^^^^^           Speech frame (bursts 'a' .. 'd')
 *
 * TDMA frame number of burst 'f' is always used as the table index. */
static const uint8_t sched_tchh_ul_amr_cmi_map[26] = {
	[10] = 1, /* TCH/H(0): a=0  / d=6  / f=10 */
	[19] = 1, /* TCH/H(0): a=8  / d=15 / f=19 */
	[2]  = 1, /* TCH/H(0): a=17 / d=23 / f=2 */

	[11] = 1, /* TCH/H(1): a=1  / d=7  / f=11 */
	[20] = 1, /* TCH/H(1): a=9  / d=16 / f=20 */
	[3]  = 1, /* TCH/H(1): a=18 / d=24 / f=3 */
};

/* TDMA frame number of burst 'a' should be used as the table index.
 * This mapping is valid for both FACCH/H(0) and FACCH/H(1). */
const uint8_t sched_tchh_dl_amr_cmi_map[26] = {
	[4]  = 1, /* TCH/H(0): a=4 */
	[13] = 1, /* TCH/H(0): a=13 */
	[21] = 1, /* TCH/H(0): a=21 */

	[5]  = 1, /* TCH/H(1): a=5 */
	[14] = 1, /* TCH/H(1): a=14 */
	[22] = 1, /* TCH/H(1): a=22 */
};

/* 3GPP TS 45.002, table 1 in clause 7: Mapping tables.
 * TDMA frame number of burst 'f' is always used as the table index. */
static const uint8_t sched_tchh_ul_facch_map[26] = {
	[10] = 1, /* FACCH/H(0): B0(0,2,4,6,8,10) */
	[11] = 1, /* FACCH/H(1): B0(1,3,5,7,9,11) */
	[19] = 1, /* FACCH/H(0): B1(8,10,13,15,17,19) */
	[20] = 1, /* FACCH/H(1): B1(9,11,14,16,18,20) */
	[2]  = 1, /* FACCH/H(0): B2(17,19,21,23,0,2) */
	[3]  = 1, /* FACCH/H(1): B2(18,20,22,24,1,3) */
};

/* TDMA frame number of burst 'a' is used as the table index. */
extern const uint8_t sched_tchh_dl_facch_map[26];

/* 3GPP TS 45.002, table 2 in clause 7: Mapping tables for TCH/H2.4 and TCH/H4.8.
 *
 * +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 * | a | b | c | d | e | f | g | h | i | j | k | l | m | n | o | p | q | r | s | t | u | v |
 * +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 *
 * TCH/H(0): B0(0,2,4,6,8,10,13,15,17,19,21,23,0,2,4,6,8,10,13,15,17,19)
 * TCH/H(1): B0(1,3,5,7,9,11,14,16,18,20,22,24,1,3,5,7,9,11,14,16,18,20)
 * TCH/H(0): B1(8,10,13,15,17,19,21,23,0,2,4,6,8,10,13,15,17,19,21,23,0,2)
 * TCH/H(1): B1(9,11,14,16,18,20,22,24,1,3,5,7,9,11,14,16,18,20,22,24,1,3)
 * TCH/H(0): B2(17,19,21,23,0,2,4,6,8,10,13,15,17,19,21,23,0,2,4,6,8,10)
 * TCH/H(1): B2(18,20,22,24,1,3,5,7,9,11,14,16,18,20,22,24,1,3,5,7,9,11)
 *
 * TDMA frame number of burst 'v' % 26 is the table index.
 * This mapping is valid for both TCH/H(0) and TCH/H(1). */
static const uint8_t sched_tchh_ul_csd_map[26] = {
	[19] = 1, /* TCH/H(0): B0(0  ... 19) */
	[20] = 1, /* TCH/H(1): B0(1  ... 20) */
	[2]  = 1, /* TCH/H(0): B1(8  ... 2) */
	[3]  = 1, /* TCH/H(1): B1(9  ... 3) */
	[10] = 1, /* TCH/H(0): B2(17 ... 10) */
	[11] = 1, /* TCH/H(1): B2(18 ... 11) */
};

/* TDMA frame number of burst 'a' % 26 is the table index.
 * This mapping is valid for both TCH/H(0) and TCH/H(1). */
static const uint8_t sched_tchh_dl_csd_map[26] = {
	[0]  = 1, /* TCH/H(0): B0(0  ... 19) */
	[1]  = 1, /* TCH/H(1): B0(1  ... 20) */
	[8]  = 1, /* TCH/H(0): B1(8  ... 2) */
	[9]  = 1, /* TCH/H(1): B1(9  ... 3) */
	[17] = 1, /* TCH/H(0): B2(17 ... 10) */
	[18] = 1, /* TCH/H(1): B2(18 ... 11) */
};

static int decode_hr_facch(struct l1sched_ts *l1ts,
			   const struct trx_ul_burst_ind *bi)
{
	struct l1sched_chan_state *chan_state = &l1ts->chan_state[bi->chan];
	const sbit_t *bursts_p = chan_state->ul_bursts;
	struct l1sched_meas_set meas_avg;
	uint8_t data[GSM_MACBLOCK_LEN];
	int n_errors, n_bits_total;
	int rc;

	rc = gsm0503_tch_hr_facch_decode(&data[0], BUFTAIL8(bursts_p),
					 &n_errors, &n_bits_total);
	if (rc != GSM_MACBLOCK_LEN)
		return rc;

	/* average measurements of the last 6 bursts, obtain TDMA Fn of the first burst */
	trx_sched_meas_avg(chan_state, &meas_avg, SCHED_MEAS_AVG_M_S6N6);

	_sched_compose_ph_data_ind(l1ts, meas_avg.fn, bi->chan,
				   &data[0], GSM_MACBLOCK_LEN,
				   compute_ber10k(n_bits_total, n_errors),
				   meas_avg.rssi,
				   meas_avg.toa256,
				   meas_avg.ci_cb,
				   PRES_INFO_UNKNOWN);
	return GSM_MACBLOCK_LEN;
}

/* Process a single Uplink TCH/H burst received by the PHY.
 * This function is visualized in file 'doc/trx_sched_tch.txt'. */
int rx_tchh_fn(struct l1sched_ts *l1ts, const struct trx_ul_burst_ind *bi)
{
	struct l1sched_chan_state *chan_state = &l1ts->chan_state[bi->chan];
	struct gsm_lchan *lchan = chan_state->lchan;
	sbit_t *burst, *bursts_p = chan_state->ul_bursts;
	uint32_t *mask = &chan_state->ul_mask;
	uint8_t rsl_cmode = chan_state->rsl_cmode;
	uint8_t tch_mode = chan_state->tch_mode;
	uint8_t tch_data[240]; /* large enough to hold 240 unpacked bits for CSD */
	int rc = 0; /* initialize to make gcc happy */
	int amr = 0;
	int n_errors = 0;
	int n_bits_total = 0;
	bool bfi_flag = false;
	enum sched_meas_avg_mode meas_avg_mode = SCHED_MEAS_AVG_M_S6N4;
	struct l1sched_meas_set meas_avg;
	unsigned int fn_begin;
	uint16_t ber10k = 0;
	uint8_t is_sub = 0;
	uint8_t ft;
	bool fn_is_cmi;

	/* If handover RACH detection is turned on, treat this burst as an Access Burst.
	 * Handle NOPE.ind as usually to ensure proper Uplink measurement reporting. */
	if (chan_state->ho_rach_detect == 1 && ~bi->flags & TRX_BI_F_NOPE_IND)
		return rx_rach_fn(l1ts, bi);

	LOGL1SB(DL1P, LOGL_DEBUG, l1ts, bi, "Received TCH/H, bid=%u\n", bi->bid);

	/* shift the buffer by 2 bursts leftwards */
	if (bi->bid == 0) {
		memmove(BUFPOS(bursts_p, 0), BUFPOS(bursts_p, 2), 20 * BPLEN);
		memset(BUFPOS(bursts_p, 20), 0, 2 * BPLEN);
		*mask = *mask << 2;
	}

	/* update mask */
	*mask |= (1 << bi->bid);

	/* store measurements */
	trx_sched_meas_push(chan_state, bi);

	/* copy burst to end of buffer of 24 bursts */
	burst = BUFPOS(bursts_p, 20 + bi->bid);
	if (bi->burst_len > 0) {
		memcpy(burst, bi->burst + 3, 58);
		memcpy(burst + 58, bi->burst + 87, 58);
	}

	/* wait until complete set of bursts */
	if (bi->bid != 1)
		return 0;

	/* fill up the burst buffer so that we have 6 bursts in there */
	if (OSMO_UNLIKELY((*mask & 0x3f) != 0x3f)) {
		LOGL1SB(DL1P, LOGL_DEBUG, l1ts, bi,
			"UL burst buffer is not filled up: mask=0x%02x != 0x3f\n",
			*mask);
		return 0; /* TODO: send BFI */
	}

	/* skip decoding of the last 4 bursts of FACCH/H */
	if (chan_state->ul_ongoing_facch) {
		chan_state->ul_ongoing_facch = 0;
		/* we have already sent the first BFI when a FACCH/H frame
		 * was decoded (see below), now send the second one. */
		trx_sched_meas_avg(chan_state, &meas_avg, meas_avg_mode);
		/* meas_avg.fn now contains TDMA frame number of the first burst */
		fn_begin = meas_avg.fn;
		goto bfi;
	}

	/* TCH/H: speech and signalling frames are interleaved over 4 and 6 bursts,
	 * respectively, while CSD frames are interleaved over 22 bursts.  Unless
	 * we're in CSD mode, decode only the last 6 bursts to avoid introducing
	 * additional delays. */
	switch (tch_mode) {
	case GSM48_CMODE_SIGN:
		meas_avg_mode = SCHED_MEAS_AVG_M_S6N6;
		/* fall-through */
	case GSM48_CMODE_SPEECH_V1: /* HR or signalling */
		rc = gsm0503_tch_hr_decode2(tch_data, BUFTAIL8(bursts_p),
					    !sched_tchh_ul_facch_map[bi->fn % 26],
					    &n_errors, &n_bits_total);
		if (rc == GSM_HR_BYTES) { /* only for valid *speech* frames */
			bool is_sid = osmo_hr_check_sid(tch_data, GSM_HR_BYTES);
			lchan_set_marker(is_sid, lchan); /* DTXu */
		}
		break;
	case GSM48_CMODE_SPEECH_AMR: /* AMR */
		/* the first FN 0,8,17 or 1,9,18 defines that CMI is included
		 * in frame, the first FN 4,13,21 or 5,14,22 defines that CMR
		 * is included in frame.
		 */

		/* See comment in function rx_tchf_fn() */
		switch (chan_state->amr_last_dtx) {
		case AHS_ONSET:
		case AHS_SID_FIRST_INH:
		case AHS_SID_UPDATE_INH:
			lchan_set_marker(false, lchan);
			break;
		}

		fn_is_cmi = sched_tchh_ul_amr_cmi_map[bi->fn % 26];

		/* See comment in function rx_tchf_fn() */
		amr = sizeof(struct amr_hdr);
		rc = gsm0503_tch_ahs_decode_dtx(tch_data + amr, BUFTAIL8(bursts_p),
						!sched_tchh_ul_facch_map[bi->fn % 26],
						!fn_is_cmi, chan_state->codec,
						chan_state->codecs, &chan_state->ul_ft,
						&chan_state->ul_cmr, &n_errors, &n_bits_total,
						&chan_state->amr_last_dtx);

		/* Tag all frames that are not regular AMR voice frames
		   as SUB-Frames */
		if (chan_state->amr_last_dtx != AMR_OTHER) {
			LOGL1SB(DL1P, LOGL_DEBUG, l1ts, bi,
				"Received AMR DTX frame (rc=%d, BER %d/%d): %s\n",
				rc, n_errors, n_bits_total,
				gsm0503_amr_dtx_frame_name(chan_state->amr_last_dtx));
			is_sub = 1;
		}

		/* See comment in function rx_tchf_fn() */
		switch (chan_state->amr_last_dtx) {
		case AHS_SID_FIRST_P1:
		case AHS_SID_FIRST_P2:
		case AHS_SID_UPDATE:
		case AHS_SID_UPDATE_CN:
			lchan_set_marker(true, lchan);
			lchan->rtp_tx_marker = false;
			break;
		}

		switch (chan_state->amr_last_dtx) {
		case AHS_SID_FIRST_P1:
		case AHS_SID_FIRST_P2:
		case AHS_SID_UPDATE:
		case AHS_SID_UPDATE_CN:
		case AHS_SID_FIRST_INH:
		case AHS_SID_UPDATE_INH:
			meas_avg_mode = SCHED_MEAS_AVG_M_S6N2;
			break;
		case AHS_ONSET:
			meas_avg_mode = SCHED_MEAS_AVG_M_S4N2;
			break;
		}

		/* only good speech frames get rtp header */
		if (rc != GSM_MACBLOCK_LEN && rc >= 4) {
			if (chan_state->amr_last_dtx == AMR_OTHER) {
				ft = chan_state->codec[chan_state->ul_ft];
			} else {
				/* SID frames will always get Frame Type Index 8 (AMR_SID) */
				ft = AMR_SID;
			}
			rc = osmo_amr_rtp_enc(tch_data,
				chan_state->codec[chan_state->ul_cmr],
			        ft, AMR_GOOD);
		}

		break;
	/* CSD (TCH/H4.8): 6.0 kbit/s radio interface rate */
	case GSM48_CMODE_DATA_6k0:
		if (!sched_tchh_ul_csd_map[bi->fn % 26])
			return 0; /* CSD: skip decoding attempt, need 2 more bursts */
		/* FACCH/F does not steal TCH/H4.8 frames, but only disturbs some bits */
		decode_hr_facch(l1ts, bi);
		rc = gsm0503_tch_hr48_decode(&tch_data[0], BUFPOS(bursts_p, 0),
					     &n_errors, &n_bits_total);
		meas_avg_mode = SCHED_MEAS_AVG_M_S22N22;
		break;
	/* CSD (TCH/H2.4): 3.6 kbit/s radio interface rate */
	case GSM48_CMODE_DATA_3k6:
		if (!sched_tchh_ul_csd_map[bi->fn % 26])
			return 0; /* CSD: skip decoding attempt, need 2 more bursts */
		/* FACCH/F does not steal TCH/H2.4 frames, but only disturbs some bits */
		decode_hr_facch(l1ts, bi);
		rc = gsm0503_tch_hr24_decode(&tch_data[0], BUFPOS(bursts_p, 0),
					     &n_errors, &n_bits_total);
		meas_avg_mode = SCHED_MEAS_AVG_M_S22N22;
		break;
	default:
		LOGL1SB(DL1P, LOGL_ERROR, l1ts, bi,
			"TCH mode %u invalid, please fix!\n",
			tch_mode);
		return -EINVAL;
	}

	ber10k = compute_ber10k(n_bits_total, n_errors);

	/* average measurements of the last N (depends on mode) bursts */
	trx_sched_meas_avg(chan_state, &meas_avg, meas_avg_mode);
	/* meas_avg.fn now contains TDMA frame number of the first burst */
	fn_begin = meas_avg.fn;

	if (tch_mode == GSM48_CMODE_SPEECH_AMR)
		trx_loop_amr_input(chan_state, &meas_avg);

	/* Check if the frame is bad */
	if (rc < 0) {
		LOGL1SB(DL1P, LOGL_NOTICE, l1ts, bi, "Received bad data (%u/%u)\n",
			bi->fn % l1ts->mf_period, l1ts->mf_period);
		bfi_flag = true;
	} else if (rc < 4) {
		LOGL1SB(DL1P, LOGL_NOTICE, l1ts, bi,
			"Received bad data (%u/%u) with invalid codec mode %d\n",
			bi->fn % l1ts->mf_period, l1ts->mf_period, rc);
		bfi_flag = true;
	}

	if (bfi_flag)
		rc = 0;		/* this is how we signal BFI to l1sap */

	/* FACCH */
	if (rc == GSM_MACBLOCK_LEN) {
		chan_state->ul_ongoing_facch = 1;
		/* In order to provide an even stream of measurement reports in *speech*
		 * mode, here we intentionally invalidate RSSI for FACCH, so that this
		 * report gets dropped in process_l1sap_meas_data().  The averaged results
		 * will be sent with the first (see below) and second (see above) BFIs. */
		_sched_compose_ph_data_ind(l1ts, fn_begin, bi->chan,
					   &tch_data[amr], GSM_MACBLOCK_LEN,
					   ber10k,
					   tch_mode == GSM48_CMODE_SIGN ? meas_avg.rssi : 0,
					   meas_avg.toa256,
					   meas_avg.ci_cb,
					   PRES_INFO_UNKNOWN);
		ber10k = 0;
bfi:
		/* A FACCH/H frame replaces two speech frames, so we need to send two BFIs.
		 * One is sent here, another will be sent two bursts later (see above). */
		rc = 0;
	}

	if (rsl_cmode == RSL_CMOD_SPD_SIGN)
		return 0;

	/* TCH or BFI */
	return _sched_compose_tch_ind(l1ts, fn_begin, bi->chan,
				      &tch_data[0], rc,
				      ber10k,
				      meas_avg.rssi,
				      meas_avg.toa256,
				      meas_avg.ci_cb,
				      is_sub);
}

/* common section for generation of TCH bursts (TCH/H and TCH/F).
 * FIXME: this function is over-complicated, refactor / get rid of it. */
extern void tch_dl_dequeue(struct l1sched_ts *l1ts, const struct trx_dl_burst_req *br,
			   struct msgb **msg_tch, struct msgb **msg_facch);

/* obtain a to-be-transmitted TCH/H (Half Traffic Channel) burst */
int tx_tchh_fn(struct l1sched_ts *l1ts, struct trx_dl_burst_req *br)
{
	struct l1sched_chan_state *chan_state = &l1ts->chan_state[br->chan];
	uint8_t tch_mode = chan_state->tch_mode;
	ubit_t *burst, *bursts_p = chan_state->dl_bursts;
	uint8_t *mask = &chan_state->dl_mask;
	struct msgb *msg_facch = NULL;
	struct msgb *msg_tch = NULL;
	struct msgb *msg = NULL;

	/* send burst, if we already got a frame */
	if (br->bid > 0) {
		if ((*mask & 0x01) != 0x01)
			return -ENOMSG;
		goto send_burst;
	}

	*mask = *mask << 2;

	/* BURST BYPASS */

	/* shift buffer by 2 bursts for interleaving */
	memmove(BUFPOS(bursts_p, 0), BUFPOS(bursts_p, 2), 20 * BPLEN);
	memset(BUFPOS(bursts_p, 20), 0, 2 * BPLEN);

	/* for half-rate CSD we dequeue every 4th burst */
	if (chan_state->rsl_cmode == RSL_CMOD_SPD_DATA) {
		if (!sched_tchh_dl_csd_map[br->fn % 26])
			goto send_burst;
	}

	/* dequeue a TCH and/or a FACCH message to be transmitted */
	tch_dl_dequeue(l1ts, br, &msg_tch, &msg_facch);

	/* if we're sending 2 middle bursts of FACCH/H */
	if (chan_state->dl_ongoing_facch) {
		/* FACCH/H shall not be scheduled at wrong FNs */
		OSMO_ASSERT(msg_facch == NULL);
		msgb_free(msg_tch); /* drop 2nd speech frame */
		chan_state->dl_ongoing_facch = 0;
		goto send_burst;
	}

	/* no message at all, send a dummy L2 frame on FACCH */
	if (msg_tch == NULL && msg_facch == NULL) {
		static const uint8_t dummy[GSM_MACBLOCK_LEN] = {
			0x03, 0x03, 0x01, /* TODO: use randomized padding */
			0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b,
			0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b,
		};
		int rc;

		LOGL1SB(DL1P, LOGL_INFO, l1ts, br, "No TCH or FACCH prim for transmit.\n");
		/* If the channel mode is TCH/HS, transmit a dummy speech block
		 * with inverted CRC3, designed to induce a BFI condition in
		 * the MS receiver.  In all other channel modes, transmit
		 * dummy FACCH like we always did before.
		 *
		 * FIXME: someone who knows AMR needs to look at this problem
		 * and decide what is the correct BTS Tx behavior for frame
		 * gaps in TCH/AHS.  See OS#6049.
		 */
		if (tch_mode == GSM48_CMODE_SPEECH_V1) {
			rc = gsm0503_tch_hr_encode(bursts_p, NULL, 0);
			if (rc == 0)
				goto send_burst;
		}

		/* FACCH/H can only be scheduled at specific TDMA offset */
		if (!sched_tchh_dl_facch_map[br->fn % 26]) {
			/* FACCH/H is not allowed, send half-filled bursts with even numbered
			 * bits contaning 232 encoded bits of the previous L2 frame, and 232
			 * odd numbered bits all set to 0. */
			goto send_burst;
		}

		gsm0503_tch_hr_encode(BUFPOS(bursts_p, 0), dummy, sizeof(dummy));
		if (chan_state->rsl_cmode != RSL_CMOD_SPD_DATA)
			chan_state->dl_ongoing_facch = 1;
		chan_state->dl_facch_bursts = 6;
		goto send_burst;
	}

	/* Unlike SACCH, FACCH has no dedicated slots on the multiframe layout.
	 * It's multiplexed together with TCH (speech or data) frames basically
	 * by replacing (stealing) their bits, either completely or partly. */
	msg = (msg_facch != NULL) ? msg_facch : msg_tch;
	if (msg == msg_facch) {
		if (chan_state->rsl_cmode != RSL_CMOD_SPD_DATA)
			chan_state->dl_ongoing_facch = 1;
		chan_state->dl_facch_bursts = 6;
	}

	/* populate the buffer with bursts */
	switch (tch_mode) {
	case GSM48_CMODE_SIGN:
	case GSM48_CMODE_SPEECH_V1:
		gsm0503_tch_hr_encode(BUFPOS(bursts_p, 0), msg->l2h, msgb_l2len(msg));
		break;
	case GSM48_CMODE_SPEECH_AMR:
		/* the first FN 4,13,21 or 5,14,22 defines that CMI is included
		 * in frame, the first FN 0,8,17 or 1,9,18 defines that CMR is
		 * included in frame. */
		gsm0503_tch_ahs_encode(BUFPOS(bursts_p, 0),
				       msgb_l2(msg), msgb_l2len(msg),
				       !sched_tchh_dl_amr_cmi_map[br->fn % 26],
				       chan_state->codec,
				       chan_state->codecs,
				       chan_state->dl_ft,
				       chan_state->dl_cmr);
		break;
	/* CSD (TCH/H4.8): 6.0 kbit/s radio interface rate */
	case GSM48_CMODE_DATA_6k0:
		gsm0503_tch_hr48_encode(BUFPOS(bursts_p, 0), msgb_l2(msg_tch));
		if (msg_facch != NULL)
			gsm0503_tch_hr_facch_encode(BUFPOS(bursts_p, 0), msgb_l2(msg_facch));
		break;
	/* CSD (TCH/H2.4): 3.6 kbit/s radio interface rate */
	case GSM48_CMODE_DATA_3k6:
		gsm0503_tch_hr24_encode(BUFPOS(bursts_p, 0), msgb_l2(msg_tch));
		if (msg_facch != NULL)
			gsm0503_tch_hr_facch_encode(BUFPOS(bursts_p, 0), msgb_l2(msg_facch));
		break;
	default:
		OSMO_ASSERT(0);
	}

	/* free messages */
	msgb_free(msg_tch);
	msgb_free(msg_facch);

send_burst:
	/* compose burst */
	burst = BUFPOS(bursts_p, br->bid);
	memcpy(br->burst + 3, burst, 58);
	memcpy(br->burst + 61, TRX_GMSK_NB_TSC(br), 26);
	memcpy(br->burst + 87, burst + 58, 58);

	br->burst_len = GSM_BURST_LEN;

	if (chan_state->dl_facch_bursts > 0) {
		chan_state->dl_facch_bursts--;
		br->flags |= TRX_BR_F_FACCH;
	}

	*mask |= (1 << br->bid);

	LOGL1SB(DL1P, LOGL_DEBUG, l1ts, br, "Transmitting burst=%u.\n", br->bid);

	return 0;
}
