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

#include <osmocom/netif/amr.h>

#include <osmo-bts/bts.h>
#include <osmo-bts/l1sap.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/scheduler.h>
#include <osmo-bts/scheduler_backend.h>
#include <osmo-bts/msg_utils.h>

#include <sched_utils.h>
#include <amr_loop.h>

/* 3GPP TS 45.009, table 3.2.1.3-{1,3}: AMR on Uplink TCH/F.
 *
 * +---+---+---+---+---+---+---+---+
 * | a | b | c | d | e | f | g | h |  Burst 'a' received first
 * +---+---+---+---+---+---+---+---+
 *  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^   Speech/FACCH frame  (bursts 'a' .. 'h')
 *
 * TDMA frame number of burst 'h' is always used as the table index. */
static const uint8_t sched_tchf_ul_amr_cmi_map[26] = {
	[7]  = 1, /* TCH/F: a=0  / h=7 */
	[16] = 1, /* TCH/F: a=8  / h=16 */
	[24] = 1, /* TCH/F: a=17 / h=24 */
};

/* TDMA frame number of burst 'a' should be used as the table index. */
static const uint8_t sched_tchf_dl_amr_cmi_map[26] = {
	[4]  = 1, /* TCH/F: a=4 */
	[13] = 1, /* TCH/F: a=13 */
	[21] = 1, /* TCH/F: a=21 */
};

extern const uint8_t sched_tchh_dl_amr_cmi_map[26];

/*! \brief a single TCH/F burst was received by the PHY, process it */
int rx_tchf_fn(struct l1sched_ts *l1ts, const struct trx_ul_burst_ind *bi)
{
	struct l1sched_chan_state *chan_state = &l1ts->chan_state[bi->chan];
	struct gsm_lchan *lchan = chan_state->lchan;
	sbit_t *burst, *bursts_p = chan_state->ul_bursts;
	uint8_t *mask = &chan_state->ul_mask;
	uint8_t rsl_cmode = chan_state->rsl_cmode;
	uint8_t tch_mode = chan_state->tch_mode;
	uint8_t tch_data[128]; /* just to be safe */
	enum sched_meas_avg_mode meas_avg_mode = SCHED_MEAS_AVG_M_S8N8;
	struct l1sched_meas_set meas_avg;
	int rc, amr = 0;
	int n_errors = 0;
	int n_bits_total = 0;
	bool bfi_flag = false;
	unsigned int fn_begin;
	uint16_t ber10k;
	uint8_t is_sub = 0;
	uint8_t ft;
	bool amr_is_cmr;

	/* If handover RACH detection is turned on, treat this burst as an Access Burst.
	 * Handle NOPE.ind as usually to ensure proper Uplink measurement reporting. */
	if (chan_state->ho_rach_detect == 1 && ~bi->flags & TRX_BI_F_NOPE_IND)
		return rx_rach_fn(l1ts, bi);

	LOGL1SB(DL1P, LOGL_DEBUG, l1ts, bi, "Received TCH/F, bid=%u\n", bi->bid);

	/* shift the buffer by 4 bursts leftwards */
	if (bi->bid == 0) {
		memcpy(bursts_p, bursts_p + 464, 464);
		memset(bursts_p + 464, 0, 464);
		*mask = *mask << 4;
	}

	/* update mask */
	*mask |= (1 << bi->bid);

	/* store measurements */
	trx_sched_meas_push(chan_state, bi);

	/* copy burst to end of buffer of 8 bursts */
	burst = bursts_p + bi->bid * 116 + 464;
	if (bi->burst_len > 0) {
		memcpy(burst, bi->burst + 3, 58);
		memcpy(burst + 58, bi->burst + 87, 58);
	} else
		memset(burst, 0, 116);

	/* wait until complete set of bursts */
	if (bi->bid != 3)
		return 0;

	/* fill up the burst buffer so that we have 8 bursts in there */
	if (OSMO_UNLIKELY((*mask & 0xff) != 0xff)) {
		LOGL1SB(DL1P, LOGL_DEBUG, l1ts, bi,
			"UL burst buffer is not filled up: mask=0x%02x != 0xff\n",
			*mask);
		return 0; /* TODO: send BFI */
	}

	/* decode
	 * also shift buffer by 4 bursts for interleaving */
	switch (tch_mode) {
	case GSM48_CMODE_SIGN:
	case GSM48_CMODE_SPEECH_V1: /* FR */
		rc = gsm0503_tch_fr_decode(tch_data, bursts_p, 1, 0, &n_errors, &n_bits_total);
		if (rc == GSM_FR_BYTES) /* only for valid *speech* frames */
			lchan_set_marker(osmo_fr_is_any_sid(tch_data), lchan); /* DTXu */
		break;
	case GSM48_CMODE_SPEECH_EFR: /* EFR */
		rc = gsm0503_tch_fr_decode(tch_data, bursts_p, 1, 1, &n_errors, &n_bits_total);
		if (rc == GSM_EFR_BYTES) /* only for valid *speech* frames */
			lchan_set_marker(osmo_efr_is_any_sid(tch_data), lchan); /* DTXu */
		break;
	case GSM48_CMODE_SPEECH_AMR: /* AMR */
		/* the first FN 0,8,17 defines that CMI is included in frame,
		 * the first FN 4,13,21 defines that CMR is included in frame.
		 * NOTE: A frame ends 7 FN after start.
		 */
		amr_is_cmr = !sched_tchf_ul_amr_cmi_map[bi->fn % 26];

		/* The AFS_ONSET frame itself does not result into an RTP frame
		 * since it only contains a recognition pattern that marks the
		 * end of the DTX interval. To mark the end of the DTX interval
		 * in the RTP stream as well, the voice frame after the
		 * AFS_ONSET frame is used. */
		if (chan_state->amr_last_dtx == AFS_ONSET)
			lchan_set_marker(false, lchan);

		/* Store AMR payload in tch-data with an offset of 2 bytes, so
		 * that we can easily prepend/fill the RTP AMR header (struct
		 * amr_hdr) with osmo_amr_rtp_enc() later on. The amr variable
		 * is used far below to account for the decoded offset in case
		 * we receive an FACCH frame instead of a voice frame (we
		 * do not know this before we actually decode the frame) */
		amr = sizeof(struct amr_hdr);
		rc = gsm0503_tch_afs_decode_dtx(tch_data + amr, bursts_p,
			amr_is_cmr, chan_state->codec, chan_state->codecs, &chan_state->ul_ft,
			&chan_state->ul_cmr, &n_errors, &n_bits_total, &chan_state->amr_last_dtx);

		/* Tag all frames that are not regular AMR voice frames as
		 * SUB-Frames */
		if (chan_state->amr_last_dtx != AMR_OTHER) {
			LOGL1SB(DL1P, LOGL_DEBUG, l1ts, bi,
				"Received AMR DTX frame (rc=%d, BER %d/%d): %s\n",
				rc, n_errors, n_bits_total,
				gsm0503_amr_dtx_frame_name(chan_state->amr_last_dtx));
			is_sub = 1;
		}

		/* The occurrence of the following frames indicates that we
		 * are either at the beginning or in the middle of a talk
		 * spurt. We update the SID status accordingly, but we do
		 * not want the marker to be set, since this must only
		 * happen when the talk spurt is over (see above) */
		switch (chan_state->amr_last_dtx) {
		case AFS_SID_FIRST:
		case AFS_SID_UPDATE:
		case AFS_SID_UPDATE_CN:
			lchan_set_marker(true, lchan);
			lchan->rtp_tx_marker = false;
			break;
		}

		switch (chan_state->amr_last_dtx) {
		case AFS_SID_FIRST:
		case AFS_SID_UPDATE_CN:
			meas_avg_mode = SCHED_MEAS_AVG_M_S8N4;
			break;
		case AFS_SID_UPDATE:
		case AFS_ONSET:
			meas_avg_mode = SCHED_MEAS_AVG_M_S4N4;
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
	default:
		LOGL1SB(DL1P, LOGL_ERROR, l1ts, bi,
			"TCH mode %u invalid, please fix!\n",
			tch_mode);
		return -EINVAL;
	}

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

	if (rc != GSM_MACBLOCK_LEN && lchan->ecu_state)
		osmo_ecu_frame_in(lchan->ecu_state, bfi_flag, tch_data, rc);

	ber10k = compute_ber10k(n_bits_total, n_errors);
	if (bfi_flag)
		goto bfi;

	/* FACCH */
	if (rc == GSM_MACBLOCK_LEN) {
		_sched_compose_ph_data_ind(l1ts, fn_begin, bi->chan,
			tch_data + amr, GSM_MACBLOCK_LEN,
			meas_avg.rssi, meas_avg.toa256,
			meas_avg.ci_cb, ber10k,
			PRES_INFO_UNKNOWN);

		/* If we are in SPEECH mode we will generate a fake (BFI) TCH
		 * indication as well. This indication is needed by the higher
		 * layers, however we already have reported the measurement
		 * result for the current block together with the FACCH.
		 * To avoid reporting the same measurement result again with
		 * the fake (BFI) TCH indication we set meas_avg.rssi to zero.
		 * Doing so tells l1sap.c to ignore the measurement result. */
		meas_avg.rssi = 0;

bfi:
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

			/* In order to signal BFI in our UL RTP output, we need
			 * to push an empty payload to l1sap.  The upper layer
			 * will choose the correct RTP representation of this
			 * BFI based on model-independent vty config. */
			rc = 0;
		}
	}

	if (rsl_cmode != RSL_CMOD_SPD_SPEECH)
		return 0;

	/* TCH or BFI */
compose_l1sap:
	return _sched_compose_tch_ind(l1ts, fn_begin, bi->chan, tch_data, rc,
				      meas_avg.toa256, ber10k, meas_avg.rssi,
				      meas_avg.ci_cb, is_sub);
}

/* common section for generation of TCH bursts (TCH/H and TCH/F).
 * FIXME: this function is over-complicated, refactor / get rid of it. */
struct msgb *tch_dl_dequeue(struct l1sched_ts *l1ts, struct trx_dl_burst_req *br)
{
	struct msgb *msg1, *msg2, *msg_tch = NULL, *msg_facch = NULL;
	struct l1sched_chan_state *chan_state = &l1ts->chan_state[br->chan];
	uint8_t rsl_cmode = chan_state->rsl_cmode;
	uint8_t tch_mode = chan_state->tch_mode;
	struct osmo_phsap_prim *l1sap;

	/* get frame and unlink from queue */
	msg1 = _sched_dequeue_prim(l1ts, br);
	msg2 = _sched_dequeue_prim(l1ts, br);
	if (msg1) {
		l1sap = msgb_l1sap_prim(msg1);
		if (l1sap->oph.primitive == PRIM_TCH) {
			msg_tch = msg1;
			if (msg2) {
				l1sap = msgb_l1sap_prim(msg2);
				if (l1sap->oph.primitive == PRIM_TCH) {
					LOGL1SB(DL1P, LOGL_FATAL, l1ts, br, "TCH twice, please FIX!\n");
					msgb_free(msg2);
				} else
					msg_facch = msg2;
			}
		} else {
			msg_facch = msg1;
			if (msg2) {
				l1sap = msgb_l1sap_prim(msg2);
				if (l1sap->oph.primitive != PRIM_TCH) {
					LOGL1SB(DL1P, LOGL_FATAL, l1ts, br, "FACCH twice, please FIX!\n");
					msgb_free(msg2);
				} else
					msg_tch = msg2;
			}
		}
	}

	/* check validity of message */
	if (msg_facch && msgb_l2len(msg_facch) != GSM_MACBLOCK_LEN) {
		LOGL1SB(DL1P, LOGL_FATAL, l1ts, br, "Prim has odd len=%u != %u\n",
			msgb_l2len(msg_facch), GSM_MACBLOCK_LEN);
		/* free message */
		msgb_free(msg_facch);
		msg_facch = NULL;
	}

	/* prioritize FACCH over speech */
	if (msg_facch) {
		/* Unlike SACCH, FACCH has no dedicated slots on the multiframe layout.
		 * It's multiplexed together with TCH (speech or data) frames basically
		 * by replacing (stealing) them.  This is common for both TCH/F and
		 * TCH/H, with the only difference that FACCH/H steals two TCH frames
		 * (not just one) due to a longer interleaving period. */
		msgb_free(msg_tch);
		return msg_facch;
	}

	/* check validity of message, get AMR ft and cmr */
	if (msg_tch) {
		int len;
		uint8_t cmr_codec;
		int ft, i;
		enum osmo_amr_type ft_codec;
		enum osmo_amr_quality bfi;
		int8_t sti, cmi;
		bool amr_is_cmr;

		if (rsl_cmode != RSL_CMOD_SPD_SPEECH) {
			LOGL1SB(DL1P, LOGL_NOTICE, l1ts, br, "Dropping speech frame, "
				"because we are not in speech mode\n");
			goto free_bad_msg;
		}

		switch (tch_mode) {
		case GSM48_CMODE_SPEECH_V1: /* FR / HR */
			if (br->chan != TRXC_TCHF) /* HR */
				len = GSM_HR_BYTES;
			else
				len = GSM_FR_BYTES;
			break;
		case GSM48_CMODE_SPEECH_EFR: /* EFR */
			if (br->chan != TRXC_TCHF)
				goto inval_mode2;
			len = GSM_EFR_BYTES;
			break;
		case GSM48_CMODE_SPEECH_AMR: /* AMR */
			len = osmo_amr_rtp_dec(msg_tch->l2h, msgb_l2len(msg_tch),
					       &cmr_codec, &cmi, &ft_codec,
					       &bfi, &sti);
			if (len < 0) {
				LOGL1SB(DL1P, LOGL_ERROR, l1ts, br, "Cannot send invalid AMR payload\n");
				goto free_bad_msg;
			}
			ft = -1;
			for (i = 0; i < chan_state->codecs; i++) {
				if (chan_state->codec[i] == ft_codec)
					ft = i;
			}
			if (ft < 0) {
				LOGL1SB(DL1P, LOGL_ERROR, l1ts, br,
					"Codec (FT = %d) of RTP frame not in list\n", ft_codec);
				goto free_bad_msg;
			}
			if (br->chan == TRXC_TCHF)
				amr_is_cmr = !sched_tchf_dl_amr_cmi_map[br->fn % 26];
			else /* TRXC_TCHH_0 or TRXC_TCHH_1 */
				amr_is_cmr = !sched_tchh_dl_amr_cmi_map[br->fn % 26];
			if (amr_is_cmr && chan_state->dl_ft != ft) {
				LOGL1SB(DL1P, LOGL_NOTICE, l1ts, br, "Codec (FT = %d) "
					" of RTP cannot be changed now, but in next frame\n", ft_codec);
				goto free_bad_msg;
			}
			chan_state->dl_ft = ft;
			if (bfi == AMR_BAD) {
				LOGL1SB(DL1P, LOGL_NOTICE, l1ts, br, "Transmitting 'bad AMR frame'\n");
				goto free_bad_msg;
			}
			break;
		default:
inval_mode2:
			LOGL1SB(DL1P, LOGL_ERROR, l1ts, br, "TCH mode invalid, please fix!\n");
			goto free_bad_msg;
		}
		if (msgb_l2len(msg_tch) != len) {
			LOGL1SB(DL1P, LOGL_ERROR, l1ts, br, "Cannot send payload with "
				"invalid length! (expecting %d, received %d)\n",
				len, msgb_l2len(msg_tch));
free_bad_msg:
			/* free message */
			msgb_free(msg_tch);
			msg_tch = NULL;
		}
	}

	return msg_tch;
}

/* obtain a to-be-transmitted TCH/F (Full Traffic Channel) burst */
int tx_tchf_fn(struct l1sched_ts *l1ts, struct trx_dl_burst_req *br)
{
	struct l1sched_chan_state *chan_state = &l1ts->chan_state[br->chan];
	uint8_t tch_mode = chan_state->tch_mode;
	ubit_t *burst, *bursts_p = chan_state->dl_bursts;
	struct msgb *msg;

	/* send burst, if we already got a frame */
	if (br->bid > 0)
		goto send_burst;

	/* BURST BYPASS */

	 /* shift buffer by 4 bursts for interleaving */
	memcpy(bursts_p, bursts_p + 464, 464);
	memset(bursts_p + 464, 0, 464);

	/* dequeue a message to be transmitted */
	msg = tch_dl_dequeue(l1ts, br);

	/* no message at all, send a dummy L2 frame on FACCH */
	if (msg == NULL) {
		static const uint8_t dummy[GSM_MACBLOCK_LEN] = {
			0x03, 0x03, 0x01, /* TODO: use randomized padding */
			0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b,
			0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b,
		};

		LOGL1SB(DL1P, LOGL_DEBUG, l1ts, br, "No TCH or FACCH prim for transmit.\n");
		gsm0503_tch_fr_encode(bursts_p, dummy, sizeof(dummy), 1);
		chan_state->dl_facch_bursts = 8;
		goto send_burst;
	}

	/* populate the buffer with bursts */
	if (msgb_l2len(msg) == GSM_MACBLOCK_LEN) {
		gsm0503_tch_fr_encode(bursts_p, msg->l2h, msgb_l2len(msg), 1);
		chan_state->dl_facch_bursts = 8;
	} else if (tch_mode == GSM48_CMODE_SPEECH_AMR) {
		/* the first FN 4,13,21 defines that CMI is included in frame,
		 * the first FN 0,8,17 defines that CMR is included in frame.
		 */
		gsm0503_tch_afs_encode(bursts_p, msg->l2h + sizeof(struct amr_hdr),
			msgb_l2len(msg) - sizeof(struct amr_hdr),
			!sched_tchf_dl_amr_cmi_map[br->fn % 26],
			chan_state->codec, chan_state->codecs,
			chan_state->dl_ft,
			chan_state->dl_cmr);
	} else {
		gsm0503_tch_fr_encode(bursts_p, msg->l2h, msgb_l2len(msg), 1);
	}

	/* free message */
	msgb_free(msg);

send_burst:
	/* compose burst */
	burst = bursts_p + br->bid * 116;
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
