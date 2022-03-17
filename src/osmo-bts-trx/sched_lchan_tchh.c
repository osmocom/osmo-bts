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
#include <osmocom/core/msgb.h>
#include <osmocom/core/utils.h>

#include <osmocom/gsm/gsm0502.h>

#include <osmocom/codec/codec.h>
#include <osmocom/codec/ecu.h>

#include <osmocom/coding/gsm0503_coding.h>
#include <osmocom/coding/gsm0503_amr_dtx.h>

#include <osmo-bts/bts.h>
#include <osmo-bts/l1sap.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/scheduler.h>
#include <osmo-bts/scheduler_backend.h>
#include <osmo-bts/msg_utils.h>

#include <sched_utils.h>
#include <loops.h>

/*! \brief a single TCH/H burst was received by the PHY, process it */
int rx_tchh_fn(struct l1sched_ts *l1ts, const struct trx_ul_burst_ind *bi)
{
	struct l1sched_chan_state *chan_state = &l1ts->chan_state[bi->chan];
	struct gsm_lchan *lchan = chan_state->lchan;
	sbit_t *burst, **bursts_p = &chan_state->ul_bursts;
	uint8_t *mask = &chan_state->ul_mask;
	uint8_t rsl_cmode = chan_state->rsl_cmode;
	uint8_t tch_mode = chan_state->tch_mode;
	uint8_t tch_data[128]; /* just to be safe */
	int rc, amr = 0;
	int n_errors = 0;
	int n_bits_total = 0;
	bool bfi_flag = false;
	/* Note on FN-10: If we are at FN 10, we decoded an even aligned
	 * TCH/FACCH frame, because our burst buffer carries 6 bursts.
	 * Even FN ending at: 10,11,19,20,2,3
	 */
	int fn_is_odd = (((bi->fn + 26 - 10) % 26) >> 2) & 1;
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

	/* allocate burst memory, if not already */
	if (!*bursts_p) {
		*bursts_p = talloc_zero_size(l1ts, 696);
		if (!*bursts_p)
			return -ENOMEM;
	}

	/* shift the buffer by 2 bursts leftwards */
	if (bi->bid == 0) {
		memcpy(*bursts_p, *bursts_p + 232, 232);
		memcpy(*bursts_p + 232, *bursts_p + 464, 232);
		memset(*bursts_p + 464, 0, 232);
		*mask = *mask << 2;
	}

	/* update mask */
	*mask |= (1 << bi->bid);

	/* store measurements */
	trx_sched_meas_push(chan_state, bi);

	/* copy burst to end of buffer of 6 bursts */
	burst = *bursts_p + bi->bid * 116 + 464;
	if (bi->burst_len > 0) {
		memcpy(burst, bi->burst + 3, 58);
		memcpy(burst + 58, bi->burst + 87, 58);
	} else
		memset(burst, 0, 116);

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

	/* decode
	 * also shift buffer by 4 bursts for interleaving */
	switch ((rsl_cmode != RSL_CMOD_SPD_SPEECH) ? GSM48_CMODE_SPEECH_V1
								: tch_mode) {
	case GSM48_CMODE_SPEECH_V1: /* HR or signalling */
		/* Note on FN-10: If we are at FN 10, we decoded an even aligned
		 * TCH/FACCH frame, because our burst buffer carries 6 bursts.
		 * Even FN ending at: 10,11,19,20,2,3
		 */
		rc = gsm0503_tch_hr_decode(tch_data, *bursts_p,
			fn_is_odd, &n_errors, &n_bits_total);
		if (rc == (GSM_HR_BYTES + 1)) { /* only for valid *speech* frames */
			/* gsm0503_tch_hr_decode() prepends a ToC octet (see RFC5993), skip it */
			bool is_sid = osmo_hr_check_sid(&tch_data[1], GSM_HR_BYTES);
			if (is_sid) /* Mark SID frames as such: F = 0, FT = 010 */
				tch_data[0] = (0x02 << 4);
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
		amr = 2;
		rc = gsm0503_tch_ahs_decode_dtx(tch_data + amr, *bursts_p,
			fn_is_odd, !fn_is_cmi, chan_state->codec,
			chan_state->codecs, &chan_state->ul_ft,
			&chan_state->ul_cmr, &n_errors, &n_bits_total, &chan_state->amr_last_dtx);

		/* Tag all frames that are not regular AMR voice frames
		   as SUB-Frames */
		if (chan_state->amr_last_dtx != AMR_OTHER) {
			LOGL1SB(DL1P, LOGL_DEBUG, l1ts, bi, "Received AMR SID frame: %s\n",
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

		if (rc)
			trx_loop_amr_input(chan_state, n_errors, n_bits_total);

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

	if (rc != GSM_MACBLOCK_LEN && lchan->ecu_state)
		osmo_ecu_frame_in(lchan->ecu_state, bfi_flag, tch_data, rc);

	if (bfi_flag)
		goto bfi;

	/* FACCH */
	if (rc == GSM_MACBLOCK_LEN) {
		chan_state->ul_ongoing_facch = 1;
		/* In order to provide an even stream of measurement reports, here we
		 * intentionally invalidate RSSI, so that this report gets dropped in
		 * process_l1sap_meas_data().  The averaged results will still be sent
		 * with the first BFI (see below). */
		_sched_compose_ph_data_ind(l1ts, fn_begin, bi->chan,
			tch_data + amr, GSM_MACBLOCK_LEN,
			0, /* intentionally invalidate RSSI */
			meas_avg.toa256, meas_avg.ci_cb, ber10k,
			PRES_INFO_UNKNOWN);
		ber10k = 0;
bfi:
		/* A FACCH/H frame replaces two speech frames, so we need to send two BFIs.
		 * One is sent here, another will be sent two bursts later (see above). */
		if (rsl_cmode == RSL_CMOD_SPD_SPEECH) {
			/* indicate bad frame */
			if (lchan->tch.dtx.ul_sid) {
				/* DTXu: pause in progress. Push empty payload to upper layers */
				rc = 0;
				goto compose_l1sap;
			}

			/* If there is an ECU active on this channel, use its output */
			if (lchan->ecu_state) {
				rc = osmo_ecu_frame_out(lchan->ecu_state, tch_data);
				if (rc >= 0) /* Otherwise we send a BFI */
					goto compose_l1sap;
			}

			switch (tch_mode) {
			case GSM48_CMODE_SPEECH_V1: /* HR */
				tch_data[0] = 0x70; /* F = 0, FT = 111 */
				memset(tch_data + 1, 0, 14);
				rc = 15;
				break;
			case GSM48_CMODE_SPEECH_AMR: /* AMR */
				rc = osmo_amr_rtp_enc(tch_data,
					chan_state->codec[chan_state->ul_cmr],
					chan_state->codec[chan_state->ul_ft],
					AMR_BAD);
				if (rc < 2) {
					LOGL1SB(DL1P, LOGL_ERROR, l1ts, bi,
					       "Failed to encode AMR_BAD frame (rc=%d), "
					       "not sending BFI\n", rc);
					return -EINVAL;
				}
				memset(tch_data + 2, 0, rc - 2);
				break;
			default:
				LOGL1SB(DL1P, LOGL_ERROR, l1ts, bi,
					"TCH mode %u invalid, please fix!\n", tch_mode);
				return -EINVAL;
			}
		}
	}

	if (rsl_cmode != RSL_CMOD_SPD_SPEECH)
		return 0;

compose_l1sap:
	/* TCH or BFI */
	return _sched_compose_tch_ind(l1ts, fn_begin, bi->chan, tch_data, rc,
				      meas_avg.toa256, ber10k, meas_avg.rssi,
				      meas_avg.ci_cb, is_sub);
}

/* common section for generation of TCH bursts (TCH/H and TCH/F).
 * FIXME: this function is over-complicated, refactor / get rid of it. */
extern void tx_tch_common(struct l1sched_ts *l1ts,
			  const struct trx_dl_burst_req *br,
			  struct msgb **_msg_tch, struct msgb **_msg_facch);

/* obtain a to-be-transmitted TCH/H (Half Traffic Channel) burst */
int tx_tchh_fn(struct l1sched_ts *l1ts, struct trx_dl_burst_req *br)
{
	struct msgb *msg_tch = NULL, *msg_facch = NULL;
	struct l1sched_chan_state *chan_state = &l1ts->chan_state[br->chan];
	uint8_t tch_mode = chan_state->tch_mode;
	ubit_t *burst, **bursts_p = &chan_state->dl_bursts;

	/* send burst, if we already got a frame */
	if (br->bid > 0) {
		if (!*bursts_p)
			return -ENODEV;
		goto send_burst;
	}

	/* get TCH and/or FACCH */
	tx_tch_common(l1ts, br, &msg_tch, &msg_facch);

	/* check for FACCH alignment */
	if (msg_facch && ((((br->fn + 4) % 26) >> 2) & 1)) {
		LOGL1SB(DL1P, LOGL_ERROR, l1ts, br,
			"Cannot transmit FACCH starting on even frames, please fix RTS!\n");
		msgb_free(msg_facch);
		msg_facch = NULL;
	}

	/* BURST BYPASS */

	/* allocate burst memory, if not already,
	 * otherwise shift buffer by 2 bursts for interleaving */
	if (!*bursts_p) {
		*bursts_p = talloc_zero_size(l1ts, 696);
		if (!*bursts_p)
			return -ENOMEM;
	} else {
		memcpy(*bursts_p, *bursts_p + 232, 232);
		if (chan_state->dl_ongoing_facch) {
			memcpy(*bursts_p + 232, *bursts_p + 464, 232);
			memset(*bursts_p + 464, 0, 232);
		} else {
			memset(*bursts_p + 232, 0, 232);
		}
	}

	/* no message at all, send a dummy L2 frame on FACCH */
	if (!msg_tch && !msg_facch && !chan_state->dl_ongoing_facch) {
		static const uint8_t dummy[GSM_MACBLOCK_LEN] = {
			0x03, 0x03, 0x01, /* TODO: use randomized padding */
			0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b,
			0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b,
		};

		LOGL1SB(DL1P, LOGL_INFO, l1ts, br, "No TCH or FACCH prim for transmit.\n");
		gsm0503_tch_hr_encode(*bursts_p, dummy, sizeof(dummy));
		goto send_burst;
	}

	/* encode bursts (prioritize FACCH) */
	if (msg_facch) {
		gsm0503_tch_hr_encode(*bursts_p, msg_facch->l2h, msgb_l2len(msg_facch));
		chan_state->dl_ongoing_facch = 1; /* first of two TCH frames */
		chan_state->dl_facch_bursts = 6;
	} else if (chan_state->dl_ongoing_facch) /* second of two TCH frames */
		chan_state->dl_ongoing_facch = 0; /* we are done with FACCH */
	else if (tch_mode == GSM48_CMODE_SPEECH_AMR)
		/* the first FN 4,13,21 or 5,14,22 defines that CMI is included
		 * in frame, the first FN 0,8,17 or 1,9,18 defines that CMR is
		 * included in frame. */
		gsm0503_tch_ahs_encode(*bursts_p, msg_tch->l2h + 2,
			msgb_l2len(msg_tch) - 2, !dl_amr_fn_is_cmi(br->fn),
			chan_state->codec, chan_state->codecs,
			chan_state->dl_ft,
			chan_state->dl_cmr);
	else
		gsm0503_tch_hr_encode(*bursts_p, msg_tch->l2h, msgb_l2len(msg_tch));

	/* free message */
	if (msg_tch)
		msgb_free(msg_tch);
	if (msg_facch)
		msgb_free(msg_facch);

send_burst:
	/* compose burst */
	burst = *bursts_p + br->bid * 116;
	memcpy(br->burst + 3, burst, 58);
	memcpy(br->burst + 61, TRX_GMSK_NB_TSC(br), 26);
	memcpy(br->burst + 87, burst + 58, 58);

	br->burst_len = GSM_BURST_LEN;

	if (chan_state->dl_facch_bursts > 0) {
		chan_state->dl_facch_bursts--;
		br->flags |= TRX_BR_F_FACCH;
	}

	LOGL1SB(DL1P, LOGL_DEBUG, l1ts, br, "Transmitting burst=%u.\n", br->bid);

	return 0;
}
