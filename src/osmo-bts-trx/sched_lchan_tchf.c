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

/*! \brief a single TCH/F burst was received by the PHY, process it */
int rx_tchf_fn(struct l1sched_ts *l1ts, const struct trx_ul_burst_ind *bi)
{
	struct l1sched_chan_state *chan_state = &l1ts->chan_state[bi->chan];
	struct gsm_lchan *lchan = chan_state->lchan;
	sbit_t *burst, **bursts_p = &chan_state->ul_bursts;
	uint8_t *mask = &chan_state->ul_mask;
	uint8_t rsl_cmode = chan_state->rsl_cmode;
	uint8_t tch_mode = chan_state->tch_mode;
	uint8_t tch_data[128]; /* just to be safe */
	enum sched_meas_avg_mode meas_avg_mode = SCHED_MEAS_AVG_M_OCTO;
	struct l1sched_meas_set meas_avg;
	int rc, amr = 0;
	int n_errors = 0;
	int n_bits_total = 0;
	bool bfi_flag = false;
	unsigned int fn_begin;
	uint16_t ber10k;
	uint8_t is_sub = 0;
	uint8_t ft;

	/* If handover RACH detection is turned on, treat this burst as an Access Burst.
	 * Handle NOPE.ind as usually to ensure proper Uplink measurement reporting. */
	if (chan_state->ho_rach_detect == 1 && ~bi->flags & TRX_BI_F_NOPE_IND)
		return rx_rach_fn(l1ts, bi);

	LOGL1SB(DL1P, LOGL_DEBUG, l1ts, bi, "Received TCH/F, bid=%u\n", bi->bid);

	/* allocate burst memory, if not already */
	if (!*bursts_p) {
		*bursts_p = talloc_zero_size(tall_bts_ctx, 928);
		if (!*bursts_p)
			return -ENOMEM;
	}

	/* clear burst */
	if (bi->bid == 0) {
		memset(*bursts_p + 464, 0, 464);
		*mask = 0x0;
	}

	/* update mask */
	*mask |= (1 << bi->bid);

	/* store measurements */
	trx_sched_meas_push(chan_state, bi);

	/* copy burst to end of buffer of 8 bursts */
	burst = *bursts_p + bi->bid * 116 + 464;
	if (bi->burst_len > 0) {
		memcpy(burst, bi->burst + 3, 58);
		memcpy(burst + 58, bi->burst + 87, 58);
	} else
		memset(burst, 0, 116);

	/* wait until complete set of bursts */
	if (bi->bid != 3)
		return 0;

	/* check for complete set of bursts */
	if ((*mask & 0xf) != 0xf) {
		LOGL1SB(DL1P, LOGL_NOTICE, l1ts, bi,
			"Received incomplete frame (%u/%u)\n",
			bi->fn % l1ts->mf_period, l1ts->mf_period);
	}
	*mask = 0x0;

	/* decode
	 * also shift buffer by 4 bursts for interleaving */
	switch ((rsl_cmode != RSL_CMOD_SPD_SPEECH) ? GSM48_CMODE_SPEECH_V1
								: tch_mode) {
	case GSM48_CMODE_SPEECH_V1: /* FR */
		rc = gsm0503_tch_fr_decode(tch_data, *bursts_p, 1, 0, &n_errors, &n_bits_total);
		if (rc >= 0)
			lchan_set_marker(osmo_fr_check_sid(tch_data, rc), lchan); /* DTXu */
		break;
	case GSM48_CMODE_SPEECH_EFR: /* EFR */
		rc = gsm0503_tch_fr_decode(tch_data, *bursts_p, 1, 1, &n_errors, &n_bits_total);
		break;
	case GSM48_CMODE_SPEECH_AMR: /* AMR */
		/* the first FN 0,8,17 defines that CMI is included in frame,
		 * the first FN 4,13,21 defines that CMR is included in frame.
		 * NOTE: A frame ends 7 FN after start.
		 */

		/* The AFS_ONSET frame itself does not result into an RTP frame
		 * since it only contains a recognition pattern that marks the
		 * end of the DTX interval. To mark the end of the DTX interval
		 * in the RTP stream as well, the voice frame after the
		 * AFS_ONSET frame is used. */
		if (chan_state->amr_last_dtx == AFS_ONSET)
			lchan_set_marker(false, lchan);

		/* we store tch_data + 2 header bytes, the amr variable set to
		 * 2 will allow us to skip the first 2 bytes in case we did
		 * receive an FACCH frame instead of a voice frame (we do not
		 * know this before we actually decode the frame) */
		amr = 2;
		rc = gsm0503_tch_afs_decode_dtx(tch_data + amr, *bursts_p,
			(((bi->fn + 26 - 7) % 26) >> 2) & 1, chan_state->codec,
			chan_state->codecs, &chan_state->ul_ft,
			&chan_state->ul_cmr, &n_errors, &n_bits_total, &chan_state->amr_last_dtx);

		/* Tag all frames that are not regular AMR voice frames as
		 * SUB-Frames */
		if (chan_state->amr_last_dtx != AMR_OTHER) {
			LOGL1SB(DL1P, LOGL_DEBUG, l1ts, bi, "Received AMR SID frame: %s\n",
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
			meas_avg_mode = SCHED_MEAS_AVG_M8_FIRST_QUAD;
			break;
		case AFS_SID_UPDATE:
		case AFS_ONSET:
			meas_avg_mode = SCHED_MEAS_AVG_M_QUAD;
			break;
		}

		if (rc)
			trx_loop_amr_input(chan_state, n_errors, n_bits_total);
		/* only good speech frames get rtp header */
		if (rc != GSM_MACBLOCK_LEN && rc >= 4) {
			if (chan_state->amr_last_dtx == AMR_OTHER) {
				ft = chan_state->codec[chan_state->ul_cmr];
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
	memcpy(*bursts_p, *bursts_p + 464, 464);

	/* average measurements of the last N (depends on mode) bursts */
	trx_sched_meas_avg(chan_state, &meas_avg, meas_avg_mode);

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
		fn_begin = gsm0502_fn_remap(bi->fn, FN_REMAP_FACCH_F);
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

			switch (tch_mode) {
			case GSM48_CMODE_SPEECH_V1: /* FR */
				memset(tch_data, 0, GSM_FR_BYTES);
				tch_data[0] = 0xd0;
				rc = GSM_FR_BYTES;
				break;
			case GSM48_CMODE_SPEECH_EFR: /* EFR */
				memset(tch_data, 0, GSM_EFR_BYTES);
				tch_data[0] = 0xc0;
				rc = GSM_EFR_BYTES;
				break;
			case GSM48_CMODE_SPEECH_AMR: /* AMR */
				rc = osmo_amr_rtp_enc(tch_data,
					chan_state->codec[chan_state->dl_cmr],
					chan_state->codec[chan_state->dl_ft],
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

	/* TCH or BFI */
compose_l1sap:
	fn_begin = gsm0502_fn_remap(bi->fn, FN_REMAP_TCH_F);
	return _sched_compose_tch_ind(l1ts, fn_begin, bi->chan, tch_data, rc,
				      /* FIXME: what should we use for BFI here? */
				      bfi_flag ? bi->toa256 : meas_avg.toa256, ber10k,
				      bfi_flag ? bi->rssi : meas_avg.rssi, is_sub);
}

/* common section for generation of TCH bursts (TCH/H and TCH/F).
 * FIXME: this function is over-complicated, refactor / get rid of it. */
void tx_tch_common(struct l1sched_ts *l1ts, struct trx_dl_burst_req *br,
		   struct msgb **_msg_tch, struct msgb **_msg_facch)
{
	struct msgb *msg1, *msg2, *msg_tch = NULL, *msg_facch = NULL;
	struct l1sched_chan_state *chan_state = &l1ts->chan_state[br->chan];
	uint8_t rsl_cmode = chan_state->rsl_cmode;
	uint8_t tch_mode = chan_state->tch_mode;
	struct osmo_phsap_prim *l1sap;

	/* handle loss detection of received TCH frames */
	if (rsl_cmode == RSL_CMOD_SPD_SPEECH
	    && ++(chan_state->lost_frames) > 5) {
		uint8_t tch_data[GSM_FR_BYTES];
		int len;

		LOGL1SB(DL1P, LOGL_NOTICE, l1ts, br, "Missing TCH bursts detected, sending BFI\n");

		/* indicate bad frame */
		switch (tch_mode) {
		case GSM48_CMODE_SPEECH_V1: /* FR / HR */
			if (br->chan != TRXC_TCHF) { /* HR */
				tch_data[0] = 0x70; /* F = 0, FT = 111 */
				memset(tch_data + 1, 0, 14);
				len = 15;
				break;
			}
			memset(tch_data, 0, GSM_FR_BYTES);
			len = GSM_FR_BYTES;
			break;
		case GSM48_CMODE_SPEECH_EFR: /* EFR */
			if (br->chan != TRXC_TCHF)
				goto inval_mode1;
			memset(tch_data, 0, GSM_EFR_BYTES);
			len = GSM_EFR_BYTES;
			break;
		case GSM48_CMODE_SPEECH_AMR: /* AMR */
			len = osmo_amr_rtp_enc(tch_data,
				chan_state->codec[chan_state->dl_cmr],
				chan_state->codec[chan_state->dl_ft], AMR_BAD);
			if (len < 2) {
				LOGL1SB(DL1P, LOGL_ERROR, l1ts, br,
					"Failed to encode AMR_BAD frame (rc=%d), "
					"not sending BFI\n", len);
				return;
			}
			memset(tch_data + 2, 0, len - 2);
			break;
		default:
inval_mode1:
			LOGL1SB(DL1P, LOGL_ERROR, l1ts, br, "TCH mode invalid, please fix!\n");
			len = 0;
		}

		if (len) {
			/* Note: RSSI/ToA256 is set to 0 to indicate to the higher
			 * layers that this is a faked tch_ind */
			_sched_compose_tch_ind(l1ts, br->fn, br->chan,
					       tch_data, len, 0, 10000, 0, 0);
		}
	}

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
	} else if (msg2) {
		l1sap = msgb_l1sap_prim(msg2);
		if (l1sap->oph.primitive == PRIM_TCH)
			msg_tch = msg2;
		else
			msg_facch = msg2;
	}

	/* check validity of message */
	if (msg_facch && msgb_l2len(msg_facch) != GSM_MACBLOCK_LEN) {
		LOGL1SB(DL1P, LOGL_FATAL, l1ts, br, "Prim has odd len=%u != %u\n",
			msgb_l2len(msg_facch), GSM_MACBLOCK_LEN);
		/* free message */
		msgb_free(msg_facch);
		msg_facch = NULL;
	}

	/* check validity of message, get AMR ft and cmr */
	if (!msg_facch && msg_tch) {
		int len;
		uint8_t cmr_codec;
		int cmr, ft, i;
		enum osmo_amr_type ft_codec;
		enum osmo_amr_quality bfi;
		int8_t sti, cmi;

		if (rsl_cmode != RSL_CMOD_SPD_SPEECH) {
			LOGL1SB(DL1P, LOGL_NOTICE, l1ts, br, "Dropping speech frame, "
				"because we are not in speech mode\n");
			goto free_bad_msg;
		}

		switch (tch_mode) {
		case GSM48_CMODE_SPEECH_V1: /* FR / HR */
			if (br->chan != TRXC_TCHF) /* HR */
				len = 15;
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
			cmr = -1;
			ft = -1;
			for (i = 0; i < chan_state->codecs; i++) {
				if (chan_state->codec[i] == cmr_codec)
					cmr = i;
				if (chan_state->codec[i] == ft_codec)
					ft = i;
			}
			if (cmr >= 0) { /* new request */
				chan_state->dl_cmr = cmr;
				/* disable AMR loop */
				trx_loop_amr_set(chan_state, 0);
			} else {
				/* enable AMR loop */
				trx_loop_amr_set(chan_state, 1);
			}
			if (ft < 0) {
				LOGL1SB(DL1P, LOGL_ERROR, l1ts, br,
					"Codec (FT = %d) of RTP frame not in list\n", ft_codec);
				goto free_bad_msg;
			}
			if (fn_is_codec_mode_request(br->fn) && chan_state->dl_ft != ft) {
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
		if (len < 0) {
			LOGL1SB(DL1P, LOGL_ERROR, l1ts, br, "Cannot send invalid AMR payload\n");
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
			goto send_frame;
		}
	}

send_frame:
	*_msg_tch = msg_tch;
	*_msg_facch = msg_facch;
}

/* obtain a to-be-transmitted TCH/F (Full Traffic Channel) burst */
int tx_tchf_fn(struct l1sched_ts *l1ts, struct trx_dl_burst_req *br)
{
	struct msgb *msg_tch = NULL, *msg_facch = NULL;
	struct l1sched_chan_state *chan_state = &l1ts->chan_state[br->chan];
	uint8_t tch_mode = chan_state->tch_mode;
	ubit_t *burst, **bursts_p = &chan_state->dl_bursts;

	/* send burst, if we already got a frame */
	if (br->bid > 0) {
		if (!*bursts_p)
			return 0;
		goto send_burst;
	}

	tx_tch_common(l1ts, br, &msg_tch, &msg_facch);

	/* BURST BYPASS */

	/* allocate burst memory, if not already,
	 * otherwise shift buffer by 4 bursts for interleaving */
	if (!*bursts_p) {
		*bursts_p = talloc_zero_size(tall_bts_ctx, 928);
		if (!*bursts_p)
			return -ENOMEM;
	} else {
		memcpy(*bursts_p, *bursts_p + 464, 464);
		memset(*bursts_p + 464, 0, 464);
	}

	/* no message at all */
	if (!msg_tch && !msg_facch) {
		LOGL1SB(DL1P, LOGL_INFO, l1ts, br, "No TCH or FACCH prim for transmit.\n");
		goto send_burst;
	}

	/* encode bursts (prioritize FACCH) */
	if (msg_facch)
		gsm0503_tch_fr_encode(*bursts_p, msg_facch->l2h, msgb_l2len(msg_facch),
			1);
	else if (tch_mode == GSM48_CMODE_SPEECH_AMR)
		/* the first FN 4,13,21 defines that CMI is included in frame,
		 * the first FN 0,8,17 defines that CMR is included in frame.
		 */
		gsm0503_tch_afs_encode(*bursts_p, msg_tch->l2h + 2,
			msgb_l2len(msg_tch) - 2, fn_is_codec_mode_request(br->fn),
			chan_state->codec, chan_state->codecs,
			chan_state->dl_ft,
			chan_state->dl_cmr);
	else
		gsm0503_tch_fr_encode(*bursts_p, msg_tch->l2h, msgb_l2len(msg_tch), 1);

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

	LOGL1SB(DL1P, LOGL_DEBUG, l1ts, br, "Transmitting burst=%u.\n", br->bid);

	return 0;
}
