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

static int decode_fr_facch(struct l1sched_ts *l1ts,
			   const struct trx_ul_burst_ind *bi)
{
	struct l1sched_chan_state *chan_state = &l1ts->chan_state[bi->chan];
	const sbit_t *bursts_p = chan_state->ul_bursts;
	struct l1sched_meas_set meas_avg;
	uint8_t data[GSM_MACBLOCK_LEN];
	int n_errors, n_bits_total;
	int rc;

	rc = gsm0503_tch_fr_facch_decode(&data[0], BUFTAIL8(bursts_p),
					 &n_errors, &n_bits_total);
	if (rc != GSM_MACBLOCK_LEN)
		return rc;

	/* average measurements of the last 8 bursts, obtain TDMA Fn of the first burst */
	trx_sched_meas_avg(chan_state, &meas_avg, SCHED_MEAS_AVG_M_S8N8);

	_sched_compose_ph_data_ind(l1ts, meas_avg.fn, bi->chan,
				   &data[0], GSM_MACBLOCK_LEN,
				   compute_ber10k(n_bits_total, n_errors),
				   meas_avg.rssi,
				   meas_avg.toa256,
				   meas_avg.ci_cb,
				   PRES_INFO_UNKNOWN);
	return GSM_MACBLOCK_LEN;
}

/* Process a single Uplink TCH/F burst received by the PHY.
 * This function is visualized in file 'doc/trx_sched_tch.txt'. */
int rx_tchf_fn(struct l1sched_ts *l1ts, const struct trx_ul_burst_ind *bi)
{
	struct l1sched_chan_state *chan_state = &l1ts->chan_state[bi->chan];
	struct gsm_lchan *lchan = chan_state->lchan;
	sbit_t *burst, *bursts_p = chan_state->ul_bursts;
	uint32_t *mask = &chan_state->ul_mask;
	uint8_t rsl_cmode = chan_state->rsl_cmode;
	uint8_t tch_mode = chan_state->tch_mode;
	uint8_t tch_data[290]; /* large enough to hold 290 unpacked bits for CSD */
	enum sched_meas_avg_mode meas_avg_mode = SCHED_MEAS_AVG_M_S8N8;
	struct l1sched_meas_set meas_avg;
	int rc, amr = 0;
	int n_errors = 0;
	int n_bits_total = 0;
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
		memmove(BUFPOS(bursts_p, 0), BUFPOS(bursts_p, 4), 20 * BPLEN);
		memset(BUFPOS(bursts_p, 20), 0, 4 * BPLEN);
		*mask = *mask << 4;
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
	if (bi->bid != 3)
		return 0;

	/* fill up the burst buffer so that we have 8 bursts in there */
	if (OSMO_UNLIKELY((*mask & 0xff) != 0xff)) {
		LOGL1SB(DL1P, LOGL_DEBUG, l1ts, bi,
			"UL burst buffer is not filled up: mask=0x%02x != 0xff\n",
			*mask);
		return 0; /* TODO: send BFI */
	}

	/* TCH/F: speech and signalling frames are interleaved over 8 bursts, while
	 * CSD frames are interleaved over 22 bursts.  Unless we're in CSD mode,
	 * decode only the last 8 bursts to avoid introducing additional delays. */
	switch (tch_mode) {
	case GSM48_CMODE_SIGN:
	case GSM48_CMODE_SPEECH_V1: /* FR */
		rc = gsm0503_tch_fr_decode(tch_data, BUFTAIL8(bursts_p),
					   1, 0, &n_errors, &n_bits_total);
		if (rc == GSM_FR_BYTES) /* only for valid *speech* frames */
			lchan_set_marker(osmo_fr_is_any_sid(tch_data), lchan); /* DTXu */
		break;
	case GSM48_CMODE_SPEECH_EFR: /* EFR */
		rc = gsm0503_tch_fr_decode(tch_data, BUFTAIL8(bursts_p),
					   1, 1, &n_errors, &n_bits_total);
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
		rc = gsm0503_tch_afs_decode_dtx(tch_data + amr, BUFTAIL8(bursts_p),
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
	/* CSD (TCH/F9.6): 12.0 kbit/s radio interface rate */
	case GSM48_CMODE_DATA_12k0:
		/* FACCH/F does not steal TCH/F9.6 frames, but only disturbs some bits */
		decode_fr_facch(l1ts, bi);
		rc = gsm0503_tch_fr96_decode(&tch_data[0], BUFPOS(bursts_p, 0),
					     &n_errors, &n_bits_total);
		meas_avg_mode = SCHED_MEAS_AVG_M_S24N22;
		break;
	/* CSD (TCH/F4.8): 6.0 kbit/s radio interface rate */
	case GSM48_CMODE_DATA_6k0:
		/* FACCH/F does not steal TCH/F4.8 frames, but only disturbs some bits */
		decode_fr_facch(l1ts, bi);
		rc = gsm0503_tch_fr48_decode(&tch_data[0], BUFPOS(bursts_p, 0),
					     &n_errors, &n_bits_total);
		meas_avg_mode = SCHED_MEAS_AVG_M_S24N22;
		break;
	/* CSD (TCH/F2.4): 3.6 kbit/s radio interface rate */
	case GSM48_CMODE_DATA_3k6:
		/* TCH/F2.4 employs the same interleaving as TCH/FS (8 bursts),
		 * so FACCH/F *does* steal TCH/F2.4 frames completely. */
		if (decode_fr_facch(l1ts, bi) == GSM_MACBLOCK_LEN)
			return 0; /* TODO: emit BFI */
		rc = gsm0503_tch_fr24_decode(&tch_data[0], BUFTAIL8(bursts_p),
					     &n_errors, &n_bits_total);
		meas_avg_mode = SCHED_MEAS_AVG_M_S8N8;
		break;
	/* CSD (TCH/F14.4): 14.5 kbit/s radio interface rate */
	case GSM48_CMODE_DATA_14k5:
		/* FACCH/F does not steal TCH/F14.4 frames, but only disturbs some bits */
		decode_fr_facch(l1ts, bi);
		rc = gsm0503_tch_fr144_decode(&tch_data[0], BUFPOS(bursts_p, 0),
					      &n_errors, &n_bits_total);
		meas_avg_mode = SCHED_MEAS_AVG_M_S24N22;
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
	if (rc < 4) {
		LOGL1SB(DL1P, LOGL_NOTICE, l1ts, bi,
			BAD_DATA_MSG_FMT "\n", BAD_DATA_MSG_ARGS);
		rc = 0;		/* this is how we signal BFI to l1sap */
	} else if (rc == GSM_MACBLOCK_LEN) { /* FACCH/F */
		_sched_compose_ph_data_ind(l1ts, fn_begin, bi->chan,
					   &tch_data[amr], GSM_MACBLOCK_LEN,
					   ber10k,
					   meas_avg.rssi,
					   meas_avg.toa256,
					   meas_avg.ci_cb,
					   PRES_INFO_UNKNOWN);

		/* If we are in SPEECH mode we will generate a fake (BFI) TCH
		 * indication as well. This indication is needed by the higher
		 * layers, however we already have reported the measurement
		 * result for the current block together with the FACCH.
		 * To avoid reporting the same measurement result again with
		 * the fake (BFI) TCH indication we set meas_avg.rssi to zero.
		 * Doing so tells l1sap.c to ignore the measurement result. */
		meas_avg.rssi = 0;
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
void tch_dl_dequeue(struct l1sched_ts *l1ts, struct trx_dl_burst_req *br,
		    struct msgb **msg_tch, struct msgb **msg_facch)
{
	struct msgb *msg1, *msg2;
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
			*msg_tch = msg1;
			if (msg2) {
				l1sap = msgb_l1sap_prim(msg2);
				if (l1sap->oph.primitive == PRIM_TCH) {
					LOGL1SB(DL1P, LOGL_FATAL, l1ts, br, "TCH twice, please FIX!\n");
					msgb_free(msg2);
				} else
					*msg_facch = msg2;
			}
		} else {
			*msg_facch = msg1;
			if (msg2) {
				l1sap = msgb_l1sap_prim(msg2);
				if (l1sap->oph.primitive != PRIM_TCH) {
					LOGL1SB(DL1P, LOGL_FATAL, l1ts, br, "FACCH twice, please FIX!\n");
					msgb_free(msg2);
				} else
					*msg_tch = msg2;
			}
		}
	}

	/* check validity of message */
	if (*msg_facch != NULL && msgb_l2len(*msg_facch) != GSM_MACBLOCK_LEN) {
		LOGL1SB(DL1P, LOGL_FATAL, l1ts, br, "Prim has odd len=%u != %u\n",
			msgb_l2len(*msg_facch), GSM_MACBLOCK_LEN);
		/* free message */
		msgb_free(*msg_facch);
		*msg_facch = NULL;
	}

	/* check validity of message, get AMR ft and cmr */
	if (*msg_tch != NULL) {
		int len;
		uint8_t cmr_codec;
		int ft, i;
		enum osmo_amr_type ft_codec;
		enum osmo_amr_quality bfi;
		int8_t sti, cmi;
		bool amr_is_cmr;

		if (OSMO_UNLIKELY(rsl_cmode == RSL_CMOD_SPD_SIGN)) {
			LOGL1SB(DL1P, LOGL_NOTICE, l1ts, br, "Dropping a TCH frame, "
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
			len = osmo_amr_rtp_dec(msgb_l2((*msg_tch)), msgb_l2len(*msg_tch),
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
			/* pull the AMR header, it's not being sent over Um */
			(*msg_tch)->l2h += sizeof(struct amr_hdr);
			len -= sizeof(struct amr_hdr);
			break;
		case GSM48_CMODE_DATA_14k5: /* TCH/F14.4 */
			if (OSMO_UNLIKELY(br->chan != TRXC_TCHF))
				goto inval_mode2;
			len = 290;
			break;
		case GSM48_CMODE_DATA_12k0: /* TCH/F9.6 */
			if (OSMO_UNLIKELY(br->chan != TRXC_TCHF))
				goto inval_mode2;
			len = 4 * 60;
			break;
		case GSM48_CMODE_DATA_6k0: /* TCH/[FH]4.8 */
			if (br->chan == TRXC_TCHF)
				len = 2 * 60;
			else
				len = 4 * 60;
			break;
		case GSM48_CMODE_DATA_3k6: /* TCH/[FH]2.4 */
			if (br->chan == TRXC_TCHF)
				len = 2 * 36;
			else
				len = 4 * 36;
			break;
		default:
inval_mode2:
			LOGL1SB(DL1P, LOGL_ERROR, l1ts, br, "TCH mode invalid, please fix!\n");
			goto free_bad_msg;
		}
		if (msgb_l2len(*msg_tch) != len) {
			LOGL1SB(DL1P, LOGL_ERROR, l1ts, br, "Cannot send payload with "
				"invalid length! (expecting %d, received %d)\n",
				len, msgb_l2len(*msg_tch));
free_bad_msg:
			/* free message */
			msgb_free(*msg_tch);
			*msg_tch = NULL;
		}
	}
}

struct msgb *tch_dummy_msgb(size_t size, uint8_t pad)
{
	struct msgb *msg;

	msg = msgb_alloc(size, __func__);
	OSMO_ASSERT(msg != NULL);

	msg->l2h = msgb_put(msg, size);
	memset(msg->l2h, pad, size);

	return msg;
}

/* obtain a to-be-transmitted TCH/F (Full Traffic Channel) burst */
int tx_tchf_fn(struct l1sched_ts *l1ts, struct trx_dl_burst_req *br)
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

	*mask = *mask << 4;

	/* BURST BYPASS */

	 /* shift buffer by 4 bursts for interleaving */
	memmove(BUFPOS(bursts_p, 0), BUFPOS(bursts_p, 4), 20 * BPLEN);
	memset(BUFPOS(bursts_p, 20), 0, 4 * BPLEN);

	/* dequeue a TCH and/or a FACCH message to be transmitted */
	tch_dl_dequeue(l1ts, br, &msg_tch, &msg_facch);
	if (msg_tch == NULL && msg_facch == NULL) {
		int rc;

		LOGL1SB(DL1P, LOGL_DEBUG, l1ts, br, "No TCH or FACCH prim for transmit.\n");
		/* If the channel mode is TCH/FS or TCH/EFS, transmit a dummy
		 * speech block with inverted CRC3, designed to induce a BFI
		 * condition in the MS receiver.  In all other channel modes,
		 * transmit dummy FACCH like we always did before.
		 *
		 * FIXME: someone who knows AMR needs to look at this problem
		 * and decide what is the correct BTS Tx behavior for frame
		 * gaps in TCH/AFS.  See OS#6049.
		 */
		switch (tch_mode) {
		case GSM48_CMODE_SPEECH_V1:
		case GSM48_CMODE_SPEECH_EFR:
			rc = gsm0503_tch_fr_encode(BUFPOS(bursts_p, 0), NULL, 0, 1);
			if (rc == 0)
				goto send_burst;
			/* fall-through */
		case GSM48_CMODE_SIGN:
		default:
			/* TODO: use randomized padding */
			msg_facch = tch_dummy_msgb(GSM_MACBLOCK_LEN, GSM_MACBLOCK_PADDING);
			/* dummy LAPDm func=UI frame */
			msg_facch->l2h[0] = 0x03;
			msg_facch->l2h[1] = 0x03;
			msg_facch->l2h[2] = 0x01;
			break;
		}
	}

	/* Unlike SACCH, FACCH has no dedicated slots on the multiframe layout.
	 * It's multiplexed together with TCH (speech or data) frames basically
	 * by replacing (stealing) their bits, either completely or partly. */
	msg = (msg_facch != NULL) ? msg_facch : msg_tch;
	if (msg == msg_facch)
		chan_state->dl_facch_bursts = 8;

	/* populate the buffer with bursts */
	switch (tch_mode) {
	case GSM48_CMODE_SIGN:
	case GSM48_CMODE_SPEECH_V1:
	case GSM48_CMODE_SPEECH_EFR:
		gsm0503_tch_fr_encode(BUFPOS(bursts_p, 0), msg->l2h, msgb_l2len(msg), 1);
		break;
	case GSM48_CMODE_SPEECH_AMR:
		/* the first FN 4,13,21 defines that CMI is included in frame,
		 * the first FN 0,8,17 defines that CMR is included in frame.
		 */
		gsm0503_tch_afs_encode(BUFPOS(bursts_p, 0),
				       msgb_l2(msg), msgb_l2len(msg),
				       !sched_tchf_dl_amr_cmi_map[br->fn % 26],
				       chan_state->codec,
				       chan_state->codecs,
				       chan_state->dl_ft,
				       chan_state->dl_cmr);
		break;
	/* CSD (TCH/F9.6): 12.0 kbit/s radio interface rate */
	case GSM48_CMODE_DATA_12k0:
		if (msg_tch != NULL)
			gsm0503_tch_fr96_encode(BUFPOS(bursts_p, 0), msgb_l2(msg_tch));
		if (msg_facch != NULL)
			gsm0503_tch_fr_facch_encode(BUFPOS(bursts_p, 0), msgb_l2(msg_facch));
		break;
	/* CSD (TCH/F4.8): 6.0 kbit/s radio interface rate */
	case GSM48_CMODE_DATA_6k0:
		if (msg_tch != NULL)
			gsm0503_tch_fr48_encode(BUFPOS(bursts_p, 0), msgb_l2(msg_tch));
		if (msg_facch != NULL)
			gsm0503_tch_fr_facch_encode(BUFPOS(bursts_p, 0), msgb_l2(msg_facch));
		break;
	/* CSD (TCH/F2.4): 3.6 kbit/s radio interface rate */
	case GSM48_CMODE_DATA_3k6:
		/* FACCH/F does steal a TCH/F2.4 frame completely */
		if (msg == msg_facch)
			gsm0503_tch_fr_facch_encode(BUFPOS(bursts_p, 0), msgb_l2(msg_facch));
		else
			gsm0503_tch_fr24_encode(BUFPOS(bursts_p, 0), msgb_l2(msg_tch));
		break;
	/* CSD (TCH/F14.4): 14.5 kbit/s radio interface rate */
	case GSM48_CMODE_DATA_14k5:
		if (msg_tch != NULL)
			gsm0503_tch_fr144_encode(BUFPOS(bursts_p, 0), msgb_l2(msg_tch));
		if (msg_facch != NULL)
			gsm0503_tch_fr_facch_encode(BUFPOS(bursts_p, 0), msgb_l2(msg_facch));
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
