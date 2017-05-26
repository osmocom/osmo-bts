/* Scheduler worker functions for OsmoBTS-TRX */

/* (C) 2013 by Andreas Eversberg <jolly@eversberg.eu>
 * (C) 2015 by Alexander Chemeris <Alexander.Chemeris@fairwaves.co>
 * (C) 2015 by Harald Welte <laforge@gnumonks.org>
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
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <stdint.h>
#include <ctype.h>

#include <osmocom/core/msgb.h>
#include <osmocom/core/talloc.h>
#include <osmocom/codec/codec.h>
#include <osmocom/core/bits.h>
#include <osmocom/gsm/a5.h>
#include <osmocom/coding/gsm0503_coding.h>

#include <osmo-bts/gsm_data.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/rsl.h>
#include <osmo-bts/l1sap.h>
#include <osmo-bts/msg_utils.h>
#include <osmo-bts/scheduler.h>
#include <osmo-bts/scheduler_backend.h>

#include "l1_if.h"
#include "trx_if.h"
#include "loops.h"

extern void *tall_bts_ctx;

/* clock states */
static uint32_t transceiver_lost;
uint32_t transceiver_last_fn;
static struct timeval transceiver_clock_tv;
static struct osmo_timer_list transceiver_clock_timer;

/* Enable this to multiply TOA of RACH by 10.
 * This is useful to check tenth of timing advances with RSSI test tool.
 * Note that regular phones will not work when using this test! */
//#define TA_TEST

/* Maximum size of a EGPRS message in bytes */
#define EGPRS_0503_MAX_BYTES		155


/* Compute the bit error rate in 1/10000 units */
static inline uint16_t compute_ber10k(int n_bits_total, int n_errors)
{
	if (n_bits_total == 0)
		return 10000;
	else
		return 10000 * n_errors / n_bits_total;
}

/*
 * TX on downlink
 */

/* an IDLE burst returns nothing. on C0 it is replaced by dummy burst */
ubit_t *tx_idle_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, uint16_t *nbits)
{
	LOGP(DL1C, LOGL_DEBUG, "Transmitting %s fn=%u ts=%u trx=%u\n",
		trx_chan_desc[chan].name, fn, tn, l1t->trx->nr);

	if (nbits)
		*nbits = GSM_BURST_LEN;

	return NULL;
}

/* obtain a to-be-transmitted FCCH (frequency correction channel) burst */
ubit_t *tx_fcch_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, uint16_t *nbits)
{
	LOGP(DL1C, LOGL_DEBUG, "Transmitting %s fn=%u ts=%u trx=%u\n",
		trx_chan_desc[chan].name, fn, tn, l1t->trx->nr);

	if (nbits)
		*nbits = GSM_BURST_LEN;

	/* BURST BYPASS */

	return (ubit_t *) _sched_fcch_burst;
}

/* obtain a to-be-transmitted SCH (synchronization channel) burst */
ubit_t *tx_sch_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, uint16_t *nbits)
{
	static ubit_t bits[GSM_BURST_LEN], burst[78];
	uint8_t sb_info[4];
	struct	gsm_time t;
	uint8_t t3p, bsic;

	LOGP(DL1C, LOGL_DEBUG, "Transmitting %s fn=%u ts=%u trx=%u\n",
		trx_chan_desc[chan].name, fn, tn, l1t->trx->nr);

	/* BURST BYPASS */

	/* create SB info from GSM time and BSIC */
	gsm_fn2gsmtime(&t, fn);
	t3p = t.t3 / 10;
	bsic = l1t->trx->bts->bsic;
	sb_info[0] =
		((bsic &  0x3f) << 2) |
		((t.t1 & 0x600) >> 9);
	sb_info[1] = 
		((t.t1 & 0x1fe) >> 1);
	sb_info[2] = 
		((t.t1 & 0x001) << 7) |
		((t.t2 &  0x1f) << 2) |
		((t3p  &   0x6) >> 1);
	sb_info[3] =
		 (t3p  &   0x1);

	/* encode bursts */
	gsm0503_sch_encode(burst, sb_info);

	/* compose burst */
	memset(bits, 0, 3);
	memcpy(bits + 3, burst, 39);
	memcpy(bits + 42, _sched_sch_train, 64);
	memcpy(bits + 106, burst + 39, 39);
	memset(bits + 145, 0, 3);

	if (nbits)
		*nbits = GSM_BURST_LEN;

	return bits;
}

/* obtain a to-be-transmitted data (SACCH/SDCCH) burst */
ubit_t *tx_data_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, uint16_t *nbits)
{
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);
	struct gsm_bts_trx_ts *ts = &l1t->trx->ts[tn];
	uint8_t link_id = trx_chan_desc[chan].link_id;
	uint8_t chan_nr = trx_chan_desc[chan].chan_nr | tn;
	struct msgb *msg = NULL; /* make GCC happy */
	ubit_t *burst, **bursts_p = &l1ts->chan_state[chan].dl_bursts;
	static ubit_t bits[GSM_BURST_LEN];

	/* send burst, if we already got a frame */
	if (bid > 0) {
		if (!*bursts_p)
			return NULL;
		goto send_burst;
	}

	/* send clock information to loops process */
	if (L1SAP_IS_LINK_SACCH(link_id))
		trx_loop_sacch_clock(l1t, chan_nr, &l1ts->chan_state[chan]);

	/* get mac block from queue */
	msg = _sched_dequeue_prim(l1t, tn, fn, chan);
	if (msg)
		goto got_msg;

	LOGP(DL1C, LOGL_INFO, "%s has not been served !! No prim for "
		"trx=%u ts=%u at fn=%u to transmit.\n", 
		trx_chan_desc[chan].name, l1t->trx->nr, tn, fn);

no_msg:
	/* free burst memory */
	if (*bursts_p) {
		talloc_free(*bursts_p);
		*bursts_p = NULL;
	}
	return NULL;

got_msg:
	/* check validity of message */
	if (msgb_l2len(msg) != GSM_MACBLOCK_LEN) {
		LOGP(DL1C, LOGL_FATAL, "Prim not 23 bytes, please FIX! "
			"(len=%d)\n", msgb_l2len(msg));
		/* free message */
		msgb_free(msg);
		goto no_msg;
	}

	/* BURST BYPASS */

	/* handle loss detection of SACCH */
	if (L1SAP_IS_LINK_SACCH(trx_chan_desc[chan].link_id)) {
		/* count and send BFI */
		if (++(l1ts->chan_state[chan].lost) > 1) {
			/* TODO: Should we pass old TOA here? Otherwise we risk
			 * unnecessary decreasing TA */

			/* Send uplink measurement information to L2 */
			l1if_process_meas_res(l1t->trx, tn, fn, trx_chan_desc[chan].chan_nr | tn,
				456, 456, -110, 0);
			/* FIXME: use actual values for BER etc */
			_sched_compose_ph_data_ind(l1t, tn, 0, chan, NULL, 0,
						   -110, 0, 0, 10000,
						   PRES_INFO_INVALID);
		}
	}

	/* allocate burst memory, if not already */
	if (!*bursts_p) {
		*bursts_p = talloc_zero_size(tall_bts_ctx, 464);
		if (!*bursts_p)
			return NULL;
	}

	/* encode bursts */
	gsm0503_xcch_encode(*bursts_p, msg->l2h);

	/* free message */
	msgb_free(msg);

send_burst:
	/* compose burst */
	burst = *bursts_p + bid * 116;
	memset(bits, 0, 3);
	memcpy(bits + 3, burst, 58);
	memcpy(bits + 61, _sched_tsc[gsm_ts_tsc(ts)], 26);
	memcpy(bits + 87, burst + 58, 58);
	memset(bits + 145, 0, 3);

	if (nbits)
		*nbits = GSM_BURST_LEN;

	LOGP(DL1C, LOGL_DEBUG, "Transmitting %s fn=%u ts=%u trx=%u burst=%u\n",
		trx_chan_desc[chan].name, fn, tn, l1t->trx->nr, bid);

	return bits;
}

/* obtain a to-be-transmitted PDTCH (packet data) burst */
ubit_t *tx_pdtch_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, uint16_t *nbits)
{
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);
	struct gsm_bts_trx_ts *ts = &l1t->trx->ts[tn];
	struct msgb *msg = NULL; /* make GCC happy */
	ubit_t *burst, **bursts_p = &l1ts->chan_state[chan].dl_bursts;
	enum trx_burst_type *burst_type = &l1ts->chan_state[chan].dl_burst_type;
	static ubit_t bits[EGPRS_BURST_LEN];
	int rc = 0;

	/* send burst, if we already got a frame */
	if (bid > 0) {
		if (!*bursts_p)
			return NULL;
		goto send_burst;
	}

	/* get mac block from queue */
	msg = _sched_dequeue_prim(l1t, tn, fn, chan);
	if (msg)
		goto got_msg;

	LOGP(DL1C, LOGL_INFO, "%s has not been served !! No prim for "
		"trx=%u ts=%u at fn=%u to transmit.\n", 
		trx_chan_desc[chan].name, l1t->trx->nr, tn, fn);

no_msg:
	/* free burst memory */
	if (*bursts_p) {
		talloc_free(*bursts_p);
		*bursts_p = NULL;
	}
	return NULL;

got_msg:
	/* BURST BYPASS */

	/* allocate burst memory, if not already */
	if (!*bursts_p) {
		*bursts_p = talloc_zero_size(tall_bts_ctx,
					     GSM0503_EGPRS_BURSTS_NBITS);
		if (!*bursts_p)
			return NULL;
	}

	/* encode bursts */
	rc = gsm0503_pdtch_egprs_encode(*bursts_p, msg->l2h, msg->tail - msg->l2h);
	if (rc < 0)
		rc = gsm0503_pdtch_encode(*bursts_p, msg->l2h, msg->tail - msg->l2h);

	/* check validity of message */
	if (rc < 0) {
		LOGP(DL1C, LOGL_FATAL, "Prim invalid length, please FIX! "
			"(len=%ld)\n", msg->tail - msg->l2h);
		/* free message */
		msgb_free(msg);
		goto no_msg;
	} else if (rc == GSM0503_EGPRS_BURSTS_NBITS) {
		*burst_type = TRX_BURST_8PSK;
	} else {
		*burst_type = TRX_BURST_GMSK;
	}

	/* free message */
	msgb_free(msg);

send_burst:
	/* compose burst */
	if (*burst_type == TRX_BURST_8PSK) {
		burst = *bursts_p + bid * 348;
		memset(bits, 1, 9);
		memcpy(bits + 9, burst, 174);
		memcpy(bits + 183, _sched_egprs_tsc[gsm_ts_tsc(ts)], 78);
		memcpy(bits + 261, burst + 174, 174);
		memset(bits + 435, 1, 9);

		if (nbits)
			*nbits = EGPRS_BURST_LEN;
	} else {
		burst = *bursts_p + bid * 116;
		memset(bits, 0, 3);
		memcpy(bits + 3, burst, 58);
		memcpy(bits + 61, _sched_tsc[gsm_ts_tsc(ts)], 26);
		memcpy(bits + 87, burst + 58, 58);
		memset(bits + 145, 0, 3);

		if (nbits)
			*nbits = GSM_BURST_LEN;
	}

	LOGP(DL1C, LOGL_DEBUG, "Transmitting %s fn=%u ts=%u trx=%u burst=%u\n",
		trx_chan_desc[chan].name, fn, tn, l1t->trx->nr, bid);

	return bits;
}

/* determine if the FN is transmitting a CMR (1) or not (0) */
static inline int fn_is_codec_mode_request(uint32_t fn)
{
	return (((fn + 4) % 26) >> 2) & 1;
}

/* common section for generation of TCH bursts (TCH/H and TCH/F) */
static void tx_tch_common(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, struct msgb **_msg_tch,
	struct msgb **_msg_facch)
{
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);
	struct msgb *msg1, *msg2, *msg_tch = NULL, *msg_facch = NULL;
	struct l1sched_chan_state *chan_state = &l1ts->chan_state[chan];
	uint8_t rsl_cmode = chan_state->rsl_cmode;
	uint8_t tch_mode = chan_state->tch_mode;
	struct osmo_phsap_prim *l1sap;

	/* handle loss detection of received TCH frames */
	if (rsl_cmode == RSL_CMOD_SPD_SPEECH
	    && ++(chan_state->lost) > 5) {
		uint8_t tch_data[GSM_FR_BYTES];
		int len;

		LOGP(DL1C, LOGL_NOTICE, "Missing TCH bursts detected, sending "
			"BFI for %s\n", trx_chan_desc[chan].name);

		/* indicate bad frame */
		switch (tch_mode) {
		case GSM48_CMODE_SPEECH_V1: /* FR / HR */
			if (chan != TRXC_TCHF) { /* HR */
				tch_data[0] = 0x70; /* F = 0, FT = 111 */
				memset(tch_data + 1, 0, 14);
				len = 15;
				break;
			}
			memset(tch_data, 0, GSM_FR_BYTES);
			len = GSM_FR_BYTES;
			break;
		case GSM48_CMODE_SPEECH_EFR: /* EFR */
			if (chan != TRXC_TCHF)
				goto inval_mode1;
			memset(tch_data, 0, GSM_EFR_BYTES);
			len = GSM_EFR_BYTES;
			break;
		case GSM48_CMODE_SPEECH_AMR: /* AMR */
			len = osmo_amr_rtp_enc(tch_data,
				chan_state->codec[chan_state->dl_cmr],
				chan_state->codec[chan_state->dl_ft], AMR_BAD);
			if (len < 2)
				break;
			memset(tch_data + 2, 0, len - 2);
			_sched_compose_tch_ind(l1t, tn, fn, chan, tch_data, len);
			break;
		default:
inval_mode1:
			LOGP(DL1C, LOGL_ERROR, "TCH mode invalid, please "
				"fix!\n");
			len = 0;
		}
		if (len)
			_sched_compose_tch_ind(l1t, tn, fn, chan, tch_data, len);
	}

	/* get frame and unlink from queue */
	msg1 = _sched_dequeue_prim(l1t, tn, fn, chan);
	msg2 = _sched_dequeue_prim(l1t, tn, fn, chan);
	if (msg1) {
		l1sap = msgb_l1sap_prim(msg1);
		if (l1sap->oph.primitive == PRIM_TCH) {
			msg_tch = msg1;
			if (msg2) {
				l1sap = msgb_l1sap_prim(msg2);
				if (l1sap->oph.primitive == PRIM_TCH) {
					LOGP(DL1C, LOGL_FATAL, "TCH twice, "
						"please FIX! ");
					msgb_free(msg2);
				} else
					msg_facch = msg2;
			}
		} else {
			msg_facch = msg1;
			if (msg2) {
				l1sap = msgb_l1sap_prim(msg2);
				if (l1sap->oph.primitive != PRIM_TCH) {
					LOGP(DL1C, LOGL_FATAL, "FACCH twice, "
						"please FIX! ");
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
		LOGP(DL1C, LOGL_FATAL, "Prim not 23 bytes, please FIX! "
			"(len=%d)\n", msgb_l2len(msg_facch));
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
			LOGP(DL1C, LOGL_NOTICE, "%s Dropping speech frame, "
				"because we are not in speech mode trx=%u "
				"ts=%u at fn=%u.\n", trx_chan_desc[chan].name,
				l1t->trx->nr, tn, fn);
			goto free_bad_msg;
		}

		switch (tch_mode) {
		case GSM48_CMODE_SPEECH_V1: /* FR / HR */
			if (chan != TRXC_TCHF) { /* HR */
				len = 15;
				if (msgb_l2len(msg_tch) >= 1
				 && (msg_tch->l2h[0] & 0xf0) != 0x00) {
					LOGP(DL1C, LOGL_NOTICE, "%s "
						"Transmitting 'bad "
						"HR frame' trx=%u ts=%u at "
						"fn=%u.\n",
						trx_chan_desc[chan].name,
						l1t->trx->nr, tn, fn);
					goto free_bad_msg;
				}
				break;
			}
			len = GSM_FR_BYTES;
			if (msgb_l2len(msg_tch) >= 1
			 && (msg_tch->l2h[0] >> 4) != 0xd) {
				LOGP(DL1C, LOGL_NOTICE, "%s Transmitting 'bad "
					"FR frame' trx=%u ts=%u at fn=%u.\n",
					trx_chan_desc[chan].name,
					l1t->trx->nr, tn, fn);
				goto free_bad_msg;
			}
			break;
		case GSM48_CMODE_SPEECH_EFR: /* EFR */
			if (chan != TRXC_TCHF)
				goto inval_mode2;
			len = GSM_EFR_BYTES;
			if (msgb_l2len(msg_tch) >= 1
			 && (msg_tch->l2h[0] >> 4) != 0xc) {
				LOGP(DL1C, LOGL_NOTICE, "%s Transmitting 'bad "
					"EFR frame' trx=%u ts=%u at fn=%u.\n",
					trx_chan_desc[chan].name,
					l1t->trx->nr, tn, fn);
				goto free_bad_msg;
			}
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
				LOGP(DL1C, LOGL_ERROR, "%s Codec (FT = %d) "
					" of RTP frame not in list. "
					"trx=%u ts=%u\n",
					trx_chan_desc[chan].name, ft_codec,
					l1t->trx->nr, tn);
				goto free_bad_msg;
			}
			if (fn_is_codec_mode_request(fn) && chan_state->dl_ft != ft) {
				LOGP(DL1C, LOGL_NOTICE, "%s Codec (FT = %d) "
					" of RTP cannot be changed now, but in "
					"next frame. trx=%u ts=%u\n",
					trx_chan_desc[chan].name, ft_codec,
					l1t->trx->nr, tn);
				goto free_bad_msg;
			}
			chan_state->dl_ft = ft;
			if (bfi == AMR_BAD) {
				LOGP(DL1C, LOGL_NOTICE, "%s Transmitting 'bad "
					"AMR frame' trx=%u ts=%u at fn=%u.\n",
					trx_chan_desc[chan].name,
					l1t->trx->nr, tn, fn);
				goto free_bad_msg;
			}
			break;
		default:
inval_mode2:
			LOGP(DL1C, LOGL_ERROR, "TCH mode invalid, please "
				"fix!\n");
			goto free_bad_msg;
		}
		if (len < 0) {
			LOGP(DL1C, LOGL_ERROR, "Cannot send invalid AMR "
				"payload\n");
			goto free_bad_msg;
		}
		if (msgb_l2len(msg_tch) != len) {
			LOGP(DL1C, LOGL_ERROR, "Cannot send payload with "
				"invalid length! (expecing %d, received %d)\n",
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
ubit_t *tx_tchf_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, uint16_t *nbits)
{
	struct msgb *msg_tch = NULL, *msg_facch = NULL;
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);
	struct gsm_bts_trx_ts *ts = &l1t->trx->ts[tn];
	struct l1sched_chan_state *chan_state = &l1ts->chan_state[chan];
	uint8_t tch_mode = chan_state->tch_mode;
	ubit_t *burst, **bursts_p = &chan_state->dl_bursts;
	static ubit_t bits[GSM_BURST_LEN];

	/* send burst, if we already got a frame */
	if (bid > 0) {
		if (!*bursts_p)
			return NULL;
		goto send_burst;
	}

	tx_tch_common(l1t, tn, fn, chan, bid, &msg_tch, &msg_facch);

	/* BURST BYPASS */

	/* allocate burst memory, if not already,
	 * otherwise shift buffer by 4 bursts for interleaving */
	if (!*bursts_p) {
		*bursts_p = talloc_zero_size(tall_bts_ctx, 928);
		if (!*bursts_p)
			return NULL;
	} else {
		memcpy(*bursts_p, *bursts_p + 464, 464);
		memset(*bursts_p + 464, 0, 464);
	}

	/* no message at all */
	if (!msg_tch && !msg_facch) {
		LOGP(DL1C, LOGL_INFO, "%s has not been served !! No prim for "
			"trx=%u ts=%u at fn=%u to transmit.\n", 
			trx_chan_desc[chan].name, l1t->trx->nr, tn, fn);
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
			msgb_l2len(msg_tch) - 2, fn_is_codec_mode_request(fn),
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
	burst = *bursts_p + bid * 116;
	memset(bits, 0, 3);
	memcpy(bits + 3, burst, 58);
	memcpy(bits + 61, _sched_tsc[gsm_ts_tsc(ts)], 26);
	memcpy(bits + 87, burst + 58, 58);
	memset(bits + 145, 0, 3);

	if (nbits)
		*nbits = GSM_BURST_LEN;

	LOGP(DL1C, LOGL_DEBUG, "Transmitting %s fn=%u ts=%u trx=%u burst=%u\n",
		trx_chan_desc[chan].name, fn, tn, l1t->trx->nr, bid);

	return bits;
}

/* obtain a to-be-transmitted TCH/H (Half Traffic Channel) burst */
ubit_t *tx_tchh_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, uint16_t *nbits)
{
	struct msgb *msg_tch = NULL, *msg_facch = NULL;
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);
	struct gsm_bts_trx_ts *ts = &l1t->trx->ts[tn];
	struct l1sched_chan_state *chan_state = &l1ts->chan_state[chan];
	uint8_t tch_mode = chan_state->tch_mode;
	ubit_t *burst, **bursts_p = &chan_state->dl_bursts;
	static ubit_t bits[GSM_BURST_LEN];

	/* send burst, if we already got a frame */
	if (bid > 0) {
		if (!*bursts_p)
			return NULL;
		goto send_burst;
	}

	/* get TCH and/or FACCH */
	tx_tch_common(l1t, tn, fn, chan, bid, &msg_tch, &msg_facch);

	/* check for FACCH alignment */
	if (msg_facch && ((((fn + 4) % 26) >> 2) & 1)) {
		LOGP(DL1C, LOGL_ERROR, "%s Cannot transmit FACCH starting on "
			"even frames, please fix RTS!\n",
			trx_chan_desc[chan].name);
		msgb_free(msg_facch);
		msg_facch = NULL;
	}

	/* BURST BYPASS */

	/* allocate burst memory, if not already,
	 * otherwise shift buffer by 2 bursts for interleaving */
	if (!*bursts_p) {
		*bursts_p = talloc_zero_size(tall_bts_ctx, 696);
		if (!*bursts_p)
			return NULL;
	} else {
		memcpy(*bursts_p, *bursts_p + 232, 232);
		if (chan_state->dl_ongoing_facch) {
			memcpy(*bursts_p + 232, *bursts_p + 464, 232);
			memset(*bursts_p + 464, 0, 232);
		} else {
			memset(*bursts_p + 232, 0, 232);
		}
	}

	/* no message at all */
	if (!msg_tch && !msg_facch && !chan_state->dl_ongoing_facch) {
		LOGP(DL1C, LOGL_INFO, "%s has not been served !! No prim for "
			"trx=%u ts=%u at fn=%u to transmit.\n", 
			trx_chan_desc[chan].name, l1t->trx->nr, tn, fn);
		goto send_burst;
	}

	/* encode bursts (prioritize FACCH) */
	if (msg_facch) {
		gsm0503_tch_hr_encode(*bursts_p, msg_facch->l2h, msgb_l2len(msg_facch));
		chan_state->dl_ongoing_facch = 1; /* first of two TCH frames */
	} else if (chan_state->dl_ongoing_facch) /* second of two TCH frames */
		chan_state->dl_ongoing_facch = 0; /* we are done with FACCH */
	else if (tch_mode == GSM48_CMODE_SPEECH_AMR)
		/* the first FN 4,13,21 or 5,14,22 defines that CMI is included
		 * in frame, the first FN 0,8,17 or 1,9,18 defines that CMR is
		 * included in frame. */
		gsm0503_tch_ahs_encode(*bursts_p, msg_tch->l2h + 2,
			msgb_l2len(msg_tch) - 2, fn_is_codec_mode_request(fn),
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
	burst = *bursts_p + bid * 116;
	memset(bits, 0, 3);
	memcpy(bits + 3, burst, 58);
	memcpy(bits + 61, _sched_tsc[gsm_ts_tsc(ts)], 26);
	memcpy(bits + 87, burst + 58, 58);
	memset(bits + 145, 0, 3);

	if (nbits)
		*nbits = GSM_BURST_LEN;

	LOGP(DL1C, LOGL_DEBUG, "Transmitting %s fn=%u ts=%u trx=%u burst=%u\n",
		trx_chan_desc[chan].name, fn, tn, l1t->trx->nr, bid);

	return bits;
}


/*
 * RX on uplink (indication to upper layer)
 */

int rx_rach_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, sbit_t *bits, uint16_t nbits,
	int8_t rssi, float toa)
{
	uint8_t chan_nr;
	struct osmo_phsap_prim l1sap;
	uint8_t ra;
	int rc;

	chan_nr = trx_chan_desc[chan].chan_nr | tn;

	LOGP(DL1C, LOGL_DEBUG, "Received Access Burst on %s fn=%u toa=%.2f\n",
		trx_chan_desc[chan].name, fn, toa);

	/* decode */
	rc = gsm0503_rach_decode(&ra, bits + 8 + 41, l1t->trx->bts->bsic);
	if (rc) {
		LOGP(DL1C, LOGL_DEBUG, "Received bad AB frame at fn=%u "
			"(%u/51)\n", fn, fn % 51);
		return 0;
	}

	/* compose primitive */
	/* generate prim */
	memset(&l1sap, 0, sizeof(l1sap));
	osmo_prim_init(&l1sap.oph, SAP_GSM_PH, PRIM_PH_RACH, PRIM_OP_INDICATION,
		NULL);
	l1sap.u.rach_ind.chan_nr = chan_nr;
	l1sap.u.rach_ind.ra = ra;
#ifdef TA_TEST
#warning TIMING ADVANCE TEST-HACK IS ENABLED!!!
	toa *= 10;
#endif
	l1sap.u.rach_ind.acc_delay = (toa >= 0) ? toa : 0;
	l1sap.u.rach_ind.fn = fn;

	/* 11bit RACH is not supported for osmo-trx */
	l1sap.u.rach_ind.is_11bit = 0;
	l1sap.u.rach_ind.burst_type = GSM_L1_BURST_TYPE_ACCESS_0;

	/* forward primitive */
	l1sap_up(l1t->trx, &l1sap);

	return 0;
}

/*! \brief a single (SDCCH/SACCH) burst was received by the PHY, process it */
int rx_data_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, sbit_t *bits, uint16_t nbits,
	int8_t rssi, float toa)
{
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);
	struct l1sched_chan_state *chan_state = &l1ts->chan_state[chan];
	sbit_t *burst, **bursts_p = &chan_state->ul_bursts;
	uint32_t *first_fn = &chan_state->ul_first_fn;
	uint8_t *mask = &chan_state->ul_mask;
	float *rssi_sum = &chan_state->rssi_sum;
	uint8_t *rssi_num = &chan_state->rssi_num;
	float *toa_sum = &chan_state->toa_sum;
	uint8_t *toa_num = &chan_state->toa_num;
	uint8_t l2[GSM_MACBLOCK_LEN], l2_len;
	int n_errors, n_bits_total;
	uint16_t ber10k;
	int rc;

	/* handle RACH, if handover RACH detection is turned on */
	if (chan_state->ho_rach_detect == 1)
		return rx_rach_fn(l1t, tn, fn, chan, bid, bits, GSM_BURST_LEN, rssi, toa);

	LOGP(DL1C, LOGL_DEBUG, "Data received %s fn=%u ts=%u trx=%u bid=%u\n",
		trx_chan_desc[chan].name, fn, tn, l1t->trx->nr, bid);

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
		*first_fn = fn;
		*rssi_sum = 0;
		*rssi_num = 0;
		*toa_sum = 0;
		*toa_num = 0;
	}

	/* update mask + RSSI */
	*mask |= (1 << bid);
	*rssi_sum += rssi;
	(*rssi_num)++;
	*toa_sum += toa;
	(*toa_num)++;

	/* copy burst to buffer of 4 bursts */
	burst = *bursts_p + bid * 116;
	memcpy(burst, bits + 3, 58);
	memcpy(burst + 58, bits + 87, 58);

	/* send burst information to loops process */
	if (L1SAP_IS_LINK_SACCH(trx_chan_desc[chan].link_id)) {
		trx_loop_sacch_input(l1t, trx_chan_desc[chan].chan_nr | tn,
			chan_state, rssi, toa);
	}

	/* wait until complete set of bursts */
	if (bid != 3)
		return 0;

	/* check for complete set of bursts */
	if ((*mask & 0xf) != 0xf) {
		LOGP(DL1C, LOGL_NOTICE, "Received incomplete data frame at "
			"fn=%u (%u/%u) for %s\n", *first_fn,
			(*first_fn) % l1ts->mf_period, l1ts->mf_period,
			trx_chan_desc[chan].name);

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
		LOGP(DL1C, LOGL_DEBUG, "Received bad data frame at fn=%u "
			"(%u/%u) for %s\n", *first_fn,
			(*first_fn) % l1ts->mf_period, l1ts->mf_period,
			trx_chan_desc[chan].name);
		l2_len = 0;
	} else
		l2_len = GSM_MACBLOCK_LEN;

	/* Send uplink measurement information to L2 */
	l1if_process_meas_res(l1t->trx, tn, fn, trx_chan_desc[chan].chan_nr | tn,
		n_errors, n_bits_total, *rssi_sum / *rssi_num, *toa_sum / *toa_num);
	ber10k = compute_ber10k(n_bits_total, n_errors);
	return _sched_compose_ph_data_ind(l1t, tn, *first_fn, chan, l2, l2_len,
					  *rssi_sum / *rssi_num,
					  4 * (*toa_sum) / *toa_num, 0, ber10k,
					  PRES_INFO_UNKNOWN);
}

/*! \brief a single PDTCH burst was received by the PHY, process it */
int rx_pdtch_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, sbit_t *bits, uint16_t nbits,
	int8_t rssi, float toa)
{
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);
	struct l1sched_chan_state *chan_state = &l1ts->chan_state[chan];
	sbit_t *burst, **bursts_p = &chan_state->ul_bursts;
	uint8_t *mask = &chan_state->ul_mask;
	float *rssi_sum = &chan_state->rssi_sum;
	uint8_t *rssi_num = &chan_state->rssi_num;
	float *toa_sum = &chan_state->toa_sum;
	uint8_t *toa_num = &chan_state->toa_num;
	uint8_t l2[EGPRS_0503_MAX_BYTES];
	int n_errors, n_bursts_bits, n_bits_total;
	uint16_t ber10k;
	int rc;

	LOGP(DL1C, LOGL_DEBUG, "PDTCH received %s fn=%u ts=%u trx=%u bid=%u\n", 
		trx_chan_desc[chan].name, fn, tn, l1t->trx->nr, bid);

	/* allocate burst memory, if not already */
	if (!*bursts_p) {
		*bursts_p = talloc_zero_size(tall_bts_ctx,
					     GSM0503_EGPRS_BURSTS_NBITS);
		if (!*bursts_p)
			return -ENOMEM;
	}

	/* clear burst */
	if (bid == 0) {
		memset(*bursts_p, 0, GSM0503_EGPRS_BURSTS_NBITS);
		*mask = 0x0;
		*rssi_sum = 0;
		*rssi_num = 0;
		*toa_sum = 0;
		*toa_num = 0;
	}

	/* update mask + rssi */
	*mask |= (1 << bid);
	*rssi_sum += rssi;
	(*rssi_num)++;
	*toa_sum += toa;
	(*toa_num)++;

	/* copy burst to buffer of 4 bursts */
	if (nbits == EGPRS_BURST_LEN) {
		burst = *bursts_p + bid * 348;
		memcpy(burst, bits + 9, 174);
		memcpy(burst + 174, bits + 261, 174);
		n_bursts_bits = GSM0503_EGPRS_BURSTS_NBITS;
	} else {
		burst = *bursts_p + bid * 116;
		memcpy(burst, bits + 3, 58);
		memcpy(burst + 58, bits + 87, 58);
		n_bursts_bits = GSM0503_GPRS_BURSTS_NBITS;
	}

	/* wait until complete set of bursts */
	if (bid != 3)
		return 0;

	/* check for complete set of bursts */
	if ((*mask & 0xf) != 0xf) {
		LOGP(DL1C, LOGL_DEBUG, "Received incomplete PDTCH block "
			"ending at fn=%u (%u/%u) for %s\n", fn,
			fn % l1ts->mf_period, l1ts->mf_period,
			trx_chan_desc[chan].name);
	}
	*mask = 0x0;

	/*
	 * Attempt to decode EGPRS bursts first. For 8-PSK EGPRS this is all we
	 * do. Attempt GPRS decoding on EGPRS failure. If the burst is GPRS,
	 * then we incur decoding overhead of 31 bits on the Type 3 EGPRS
	 * header, which is tolerable.
	 */
	rc = gsm0503_pdtch_egprs_decode(l2, *bursts_p, n_bursts_bits,
				NULL, &n_errors, &n_bits_total);

	if ((nbits == GSM_BURST_LEN) && (rc < 0)) {
		rc = gsm0503_pdtch_decode(l2, *bursts_p, NULL,
				  &n_errors, &n_bits_total);
	}


	/* Send uplink measurement information to L2 */
	l1if_process_meas_res(l1t->trx, tn, fn, trx_chan_desc[chan].chan_nr | tn,
		n_errors, n_bits_total, *rssi_sum / *rssi_num, *toa_sum / *toa_num);

	if (rc <= 0) {
		LOGP(DL1C, LOGL_DEBUG, "Received bad PDTCH block ending at "
			"fn=%u (%u/%u) for %s\n", fn, fn % l1ts->mf_period,
			l1ts->mf_period, trx_chan_desc[chan].name);
		return 0;
	}
	ber10k = compute_ber10k(n_bits_total, n_errors);
	return _sched_compose_ph_data_ind(l1t, tn, (fn + GSM_HYPERFRAME - 3) % GSM_HYPERFRAME, chan,
		l2, rc, *rssi_sum / *rssi_num, 4 * (*toa_sum) / *toa_num, 0,
					  ber10k, PRES_INFO_BOTH);
}

/*! \brief a single TCH/F burst was received by the PHY, process it */
int rx_tchf_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, sbit_t *bits, uint16_t nbits,
	int8_t rssi, float toa)
{
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);
	struct l1sched_chan_state *chan_state = &l1ts->chan_state[chan];
	sbit_t *burst, **bursts_p = &chan_state->ul_bursts;
	uint8_t *mask = &chan_state->ul_mask;
	uint8_t rsl_cmode = chan_state->rsl_cmode;
	uint8_t tch_mode = chan_state->tch_mode;
	uint8_t tch_data[128]; /* just to be safe */
	int rc, amr = 0;
	int n_errors, n_bits_total;
	struct gsm_lchan *lchan =
		get_lchan_by_chan_nr(l1t->trx, trx_chan_desc[chan].chan_nr | tn);

	/* handle rach, if handover rach detection is turned on */
	if (chan_state->ho_rach_detect == 1)
		return rx_rach_fn(l1t, tn, fn, chan, bid, bits, GSM_BURST_LEN, rssi, toa);

	LOGP(DL1C, LOGL_DEBUG, "TCH/F received %s fn=%u ts=%u trx=%u bid=%u\n", 
		trx_chan_desc[chan].name, fn, tn, l1t->trx->nr, bid);

	/* allocate burst memory, if not already */
	if (!*bursts_p) {
		*bursts_p = talloc_zero_size(tall_bts_ctx, 928);
		if (!*bursts_p)
			return -ENOMEM;
	}

	/* clear burst */
	if (bid == 0) {
		memset(*bursts_p + 464, 0, 464);
		*mask = 0x0;
	}

	/* update mask */
	*mask |= (1 << bid);

	/* copy burst to end of buffer of 8 bursts */
	burst = *bursts_p + bid * 116 + 464;
	memcpy(burst, bits + 3, 58);
	memcpy(burst + 58, bits + 87, 58);

	/* wait until complete set of bursts */
	if (bid != 3)
		return 0;

	/* check for complete set of bursts */
	if ((*mask & 0xf) != 0xf) {
		LOGP(DL1C, LOGL_NOTICE, "Received incomplete TCH frame ending "
			"at fn=%u (%u/%u) for %s\n", fn,
			fn % l1ts->mf_period, l1ts->mf_period,
			trx_chan_desc[chan].name);
	}
	*mask = 0x0;

	/* decode
	 * also shift buffer by 4 bursts for interleaving */
	switch ((rsl_cmode != RSL_CMOD_SPD_SPEECH) ? GSM48_CMODE_SPEECH_V1
								: tch_mode) {
	case GSM48_CMODE_SPEECH_V1: /* FR */
		rc = gsm0503_tch_fr_decode(tch_data, *bursts_p, 1, 0, &n_errors, &n_bits_total);
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
		rc = gsm0503_tch_afs_decode(tch_data + 2, *bursts_p,
			(((fn + 26 - 7) % 26) >> 2) & 1, chan_state->codec,
			chan_state->codecs, &chan_state->ul_ft,
			&chan_state->ul_cmr, &n_errors, &n_bits_total);
		if (rc)
			trx_loop_amr_input(l1t,
				trx_chan_desc[chan].chan_nr | tn, chan_state,
				(float)n_errors/(float)n_bits_total);
		amr = 2; /* we store tch_data + 2 header bytes */
		/* only good speech frames get rtp header */
		if (rc != GSM_MACBLOCK_LEN && rc >= 4) {
			rc = osmo_amr_rtp_enc(tch_data,
				chan_state->codec[chan_state->ul_cmr],
				chan_state->codec[chan_state->ul_ft], AMR_GOOD);
		}
		break;
	default:
		LOGP(DL1C, LOGL_ERROR, "TCH mode %u invalid, please fix!\n",
			tch_mode);
		return -EINVAL;
	}
	memcpy(*bursts_p, *bursts_p + 464, 464);

	/* Send uplink measurement information to L2 */
	l1if_process_meas_res(l1t->trx, tn, fn, trx_chan_desc[chan].chan_nr|tn,
		n_errors, n_bits_total, rssi, toa);

	/* Check if the frame is bad */
	if (rc < 0) {
		LOGP(DL1C, LOGL_NOTICE, "Received bad TCH frame ending at "
			"fn=%u for %s\n", fn, trx_chan_desc[chan].name);
		goto bfi;
	}
	if (rc < 4) {
		LOGP(DL1C, LOGL_NOTICE, "Received bad TCH frame ending at "
			"fn=%u for %s with codec mode %d (out of range)\n",
			fn, trx_chan_desc[chan].name, rc);
		goto bfi;
	}

	/* FACCH */
	if (rc == GSM_MACBLOCK_LEN) {
		uint16_t ber10k = compute_ber10k(n_bits_total, n_errors);
		_sched_compose_ph_data_ind(l1t, tn, (fn + GSM_HYPERFRAME - 7) % GSM_HYPERFRAME, chan,
			tch_data + amr, GSM_MACBLOCK_LEN, rssi, 4 * toa, 0,
					   ber10k, PRES_INFO_UNKNOWN);
bfi:
		if (rsl_cmode == RSL_CMOD_SPD_SPEECH) {
			/* indicate bad frame */
			switch (tch_mode) {
			case GSM48_CMODE_SPEECH_V1: /* FR */
				if (lchan->tch.dtx.ul_sid)
					return 0; /* DTXu: pause in progress */
				memset(tch_data, 0, GSM_FR_BYTES);
				rc = GSM_FR_BYTES;
				break;
			case GSM48_CMODE_SPEECH_EFR: /* EFR */
				memset(tch_data, 0, GSM_EFR_BYTES);
				rc = GSM_EFR_BYTES;
				break;
			case GSM48_CMODE_SPEECH_AMR: /* AMR */
				rc = osmo_amr_rtp_enc(tch_data,
					chan_state->codec[chan_state->dl_cmr],
					chan_state->codec[chan_state->dl_ft],
					AMR_BAD);
				if (rc < 2)
					break;
				memset(tch_data + 2, 0, rc - 2);
				break;
			default:
				LOGP(DL1C, LOGL_ERROR, "TCH mode invalid, "
					"please fix!\n");
				return -EINVAL;
			}
		}
	}

	if (rsl_cmode != RSL_CMOD_SPD_SPEECH)
		return 0;

	/* TCH or BFI */
	return _sched_compose_tch_ind(l1t, tn, (fn + GSM_HYPERFRAME - 7) % GSM_HYPERFRAME, chan,
		tch_data, rc);
}

/*! \brief a single TCH/H burst was received by the PHY, process it */
int rx_tchh_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, sbit_t *bits, uint16_t nbits,
	int8_t rssi, float toa)
{
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);
	struct l1sched_chan_state *chan_state = &l1ts->chan_state[chan];
	sbit_t *burst, **bursts_p = &chan_state->ul_bursts;
	uint8_t *mask = &chan_state->ul_mask;
	uint8_t rsl_cmode = chan_state->rsl_cmode;
	uint8_t tch_mode = chan_state->tch_mode;
	uint8_t tch_data[128]; /* just to be safe */
	int rc, amr = 0;
	int n_errors, n_bits_total;
	struct gsm_lchan *lchan =
		get_lchan_by_chan_nr(l1t->trx, trx_chan_desc[chan].chan_nr | tn);
	/* Note on FN-10: If we are at FN 10, we decoded an even aligned
	 * TCH/FACCH frame, because our burst buffer carries 6 bursts.
	 * Even FN ending at: 10,11,19,20,2,3
	 */
	int fn_is_odd = (((fn + 26 - 10) % 26) >> 2) & 1;

	/* handle RACH, if handover RACH detection is turned on */
	if (chan_state->ho_rach_detect == 1)
		return rx_rach_fn(l1t, tn, fn, chan, bid, bits, GSM_BURST_LEN, rssi, toa);

	LOGP(DL1C, LOGL_DEBUG, "TCH/H received %s fn=%u ts=%u trx=%u bid=%u\n",
		trx_chan_desc[chan].name, fn, tn, l1t->trx->nr, bid);

	/* allocate burst memory, if not already */
	if (!*bursts_p) {
		*bursts_p = talloc_zero_size(tall_bts_ctx, 696);
		if (!*bursts_p)
			return -ENOMEM;
	}

	/* clear burst */
	if (bid == 0) {
		memset(*bursts_p + 464, 0, 232);
		*mask = 0x0;
	}

	/* update mask */
	*mask |= (1 << bid);

	/* copy burst to end of buffer of 6 bursts */
	burst = *bursts_p + bid * 116 + 464;
	memcpy(burst, bits + 3, 58);
	memcpy(burst + 58, bits + 87, 58);

	/* wait until complete set of bursts */
	if (bid != 1)
		return 0;

	/* check for complete set of bursts */
	if ((*mask & 0x3) != 0x3) {
		LOGP(DL1C, LOGL_NOTICE, "Received incomplete TCH frame ending "
			"at fn=%u (%u/%u) for %s\n", fn,
			fn % l1ts->mf_period, l1ts->mf_period,
			trx_chan_desc[chan].name);
	}
	*mask = 0x0;

	/* skip second of two TCH frames of FACCH was received */
	if (chan_state->ul_ongoing_facch) {
		chan_state->ul_ongoing_facch = 0;
		memcpy(*bursts_p, *bursts_p + 232, 232);
		memcpy(*bursts_p + 232, *bursts_p + 464, 232);
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
		if (rc) /* DTXu */
			lchan_set_marker(osmo_hr_check_sid(tch_data, rc), lchan);
		break;
	case GSM48_CMODE_SPEECH_AMR: /* AMR */
		/* the first FN 0,8,17 or 1,9,18 defines that CMI is included
		 * in frame, the first FN 4,13,21 or 5,14,22 defines that CMR
		 * is included in frame.
		 */
		rc = gsm0503_tch_ahs_decode(tch_data + 2, *bursts_p,
			fn_is_odd, fn_is_odd, chan_state->codec,
			chan_state->codecs, &chan_state->ul_ft,
			&chan_state->ul_cmr, &n_errors, &n_bits_total);
		if (rc)
			trx_loop_amr_input(l1t,
				trx_chan_desc[chan].chan_nr | tn, chan_state,
				(float)n_errors/(float)n_bits_total);
		amr = 2; /* we store tch_data + 2 two */
		/* only good speech frames get rtp header */
		if (rc != GSM_MACBLOCK_LEN && rc >= 4) {
			rc = osmo_amr_rtp_enc(tch_data,
				chan_state->codec[chan_state->ul_cmr],
				chan_state->codec[chan_state->ul_ft], AMR_GOOD);
		}
		break;
	default:
		LOGP(DL1C, LOGL_ERROR, "TCH mode %u invalid, please fix!\n",
			tch_mode);
		return -EINVAL;
	}
	memcpy(*bursts_p, *bursts_p + 232, 232);
	memcpy(*bursts_p + 232, *bursts_p + 464, 232);

	/* Send uplink measurement information to L2 */
	l1if_process_meas_res(l1t->trx, tn, fn, trx_chan_desc[chan].chan_nr|tn,
		n_errors, n_bits_total, rssi, toa);

	/* Check if the frame is bad */
	if (rc < 0) {
		LOGP(DL1C, LOGL_NOTICE, "Received bad TCH frame ending at "
			"fn=%u for %s\n", fn, trx_chan_desc[chan].name);
		goto bfi;
	}
	if (rc < 4) {
		LOGP(DL1C, LOGL_NOTICE, "Received bad TCH frame ending at "
			"fn=%u for %s with codec mode %d (out of range)\n",
			fn, trx_chan_desc[chan].name, rc);
		goto bfi;
	}

	/* FACCH */
	if (rc == GSM_MACBLOCK_LEN) {
		chan_state->ul_ongoing_facch = 1;
		uint16_t ber10k = compute_ber10k(n_bits_total, n_errors);
		_sched_compose_ph_data_ind(l1t, tn,
			(fn + GSM_HYPERFRAME - 10 - ((fn % 26) >= 19)) % GSM_HYPERFRAME, chan,
			tch_data + amr, GSM_MACBLOCK_LEN, rssi, 4 * toa, 0,
					   ber10k, PRES_INFO_UNKNOWN);
bfi:
		if (rsl_cmode == RSL_CMOD_SPD_SPEECH) {
			/* indicate bad frame */
			switch (tch_mode) {
			case GSM48_CMODE_SPEECH_V1: /* HR */
				if (lchan->tch.dtx.ul_sid)
					return 0; /* DTXu: pause in progress */
				tch_data[0] = 0x70; /* F = 0, FT = 111 */
				memset(tch_data + 1, 0, 14);
				rc = 15;
				break;
			case GSM48_CMODE_SPEECH_AMR: /* AMR */
				rc = osmo_amr_rtp_enc(tch_data,
					chan_state->codec[chan_state->dl_cmr],
					chan_state->codec[chan_state->dl_ft],
					AMR_BAD);
				if (rc < 2)
					break;
				memset(tch_data + 2, 0, rc - 2);
				break;
			default:
				LOGP(DL1C, LOGL_ERROR, "TCH mode invalid, "
					"please fix!\n");
				return -EINVAL;
			}
		}
	}

	if (rsl_cmode != RSL_CMOD_SPD_SPEECH)
		return 0;

	/* TCH or BFI */
	/* Note on FN 19 or 20: If we received the last burst of a frame,
	 * it actually starts at FN 8 or 9. A burst starting there, overlaps
	 * with the slot 12, so an extra FN must be substracted to get correct
	 * start of frame.
	 */
	return _sched_compose_tch_ind(l1t, tn,
		(fn + GSM_HYPERFRAME - 10 - ((fn%26)==19) - ((fn%26)==20)) % GSM_HYPERFRAME,
		chan, tch_data, rc);
}

/* schedule all frames of all TRX for given FN */
static int trx_sched_fn(struct gsm_bts *bts, uint32_t fn)
{
	struct gsm_bts_trx *trx;
	uint8_t tn;
	const ubit_t *bits;
	uint8_t gain;
	uint16_t nbits;

	/* send time indication */
	l1if_mph_time_ind(bts, fn);

	/* process every TRX */
	llist_for_each_entry(trx, &bts->trx_list, list) {
		struct phy_instance *pinst = trx_phy_instance(trx);
		struct phy_link *plink = pinst->phy_link;
		struct trx_l1h *l1h = pinst->u.osmotrx.hdl;
		struct l1sched_trx *l1t = &l1h->l1s;

		/* advance frame number, so the transceiver has more
		 * time until it must be transmitted. */
		fn = (fn + plink->u.osmotrx.clock_advance) % GSM_HYPERFRAME;

		/* we don't schedule, if power is off */
		if (!trx_if_powered(l1h))
			continue;

		/* process every TS of TRX */
		for (tn = 0; tn < ARRAY_SIZE(l1t->ts); tn++) {
			/* ready-to-send */
			_sched_rts(l1t, tn,
				(fn + plink->u.osmotrx.rts_advance) % GSM_HYPERFRAME);
			/* get burst for FN */
			bits = _sched_dl_burst(l1t, tn, fn, &nbits);
			if (!bits) {
				/* if no bits, send no burst */
				continue;
			} else
				gain = 0;
			if (nbits)
				trx_if_data(l1h, tn, fn, gain, bits, nbits);
		}
	}

	return 0;
}


/*
 * frame clock
 */

#define FRAME_DURATION_uS	4615
#define MAX_FN_SKEW		50
#define TRX_LOSS_FRAMES		400

extern int quit;
/* this timer fires for every FN to be processed */
static void trx_ctrl_timer_cb(void *data)
{
	struct gsm_bts *bts = data;
	struct timeval tv_now, *tv_clock = &transceiver_clock_tv;
	int32_t elapsed;

	/* check if transceiver is still alive */
	if (transceiver_lost++ == TRX_LOSS_FRAMES) {
		struct gsm_bts_trx *trx;

		LOGP(DL1C, LOGL_NOTICE, "No more clock from transceiver\n");

no_clock:
		transceiver_available = 0;

		bts_shutdown(bts, "No clock from osmo-trx");

		return;
	}

	gettimeofday(&tv_now, NULL);

	elapsed = (tv_now.tv_sec - tv_clock->tv_sec) * 1000000
		+ (tv_now.tv_usec - tv_clock->tv_usec);

	/* if someone played with clock, or if the process stalled */
	if (elapsed > FRAME_DURATION_uS * MAX_FN_SKEW || elapsed < 0) {
		LOGP(DL1C, LOGL_NOTICE, "PC clock skew: elapsed uS %d\n",
			elapsed);
		goto no_clock;
	}

	/* schedule next FN clock */
	while (elapsed > FRAME_DURATION_uS / 2) {
		tv_clock->tv_usec += FRAME_DURATION_uS;
		if (tv_clock->tv_usec >= 1000000) {
			tv_clock->tv_sec++;
			tv_clock->tv_usec -= 1000000;
		}
		transceiver_last_fn = (transceiver_last_fn + 1) % GSM_HYPERFRAME;
		trx_sched_fn(bts, transceiver_last_fn);
		elapsed -= FRAME_DURATION_uS;
	}
	osmo_timer_schedule(&transceiver_clock_timer, 0,
		FRAME_DURATION_uS - elapsed);
}


/* receive clock from transceiver */
int trx_sched_clock(struct gsm_bts *bts, uint32_t fn)
{
	struct timeval tv_now, *tv_clock = &transceiver_clock_tv;
	int32_t elapsed;
	int32_t elapsed_fn;

	if (quit)
		return 0;

	/* reset lost counter */
	transceiver_lost = 0;

	gettimeofday(&tv_now, NULL);

	/* clock becomes valid */
	if (!transceiver_available) {
		LOGP(DL1C, LOGL_NOTICE, "initial GSM clock received: fn=%u\n",
			fn);

		transceiver_available = 1;

		/* start provisioning transceiver */
		l1if_provision_transceiver(bts);

		/* tell BSC */
		check_transceiver_availability(bts, 1);

new_clock:
		transceiver_last_fn = fn;
		trx_sched_fn(bts, transceiver_last_fn);

		/* schedule first FN clock */
		memcpy(tv_clock, &tv_now, sizeof(struct timeval));
		memset(&transceiver_clock_timer, 0,
			sizeof(transceiver_clock_timer));
		transceiver_clock_timer.cb = trx_ctrl_timer_cb;
	        transceiver_clock_timer.data = bts;
		osmo_timer_schedule(&transceiver_clock_timer, 0,
			FRAME_DURATION_uS);

		return 0;
	}

	osmo_timer_del(&transceiver_clock_timer);

	/* calculate elapsed time since last_fn */
	elapsed = (tv_now.tv_sec - tv_clock->tv_sec) * 1000000
		+ (tv_now.tv_usec - tv_clock->tv_usec);

	/* how much frames have been elapsed since last fn processed */
	elapsed_fn = (fn + GSM_HYPERFRAME - transceiver_last_fn) % GSM_HYPERFRAME;
	if (elapsed_fn >= 135774)
		elapsed_fn -= GSM_HYPERFRAME;

	/* check for max clock skew */
	if (elapsed_fn > MAX_FN_SKEW || elapsed_fn < -MAX_FN_SKEW) {
		LOGP(DL1C, LOGL_NOTICE, "GSM clock skew: old fn=%u, "
			"new fn=%u\n", transceiver_last_fn, fn);
		goto new_clock;
	}

	LOGP(DL1C, LOGL_INFO, "GSM clock jitter: %d\n",
		elapsed_fn * FRAME_DURATION_uS - elapsed);

	/* too many frames have been processed already */
	if (elapsed_fn < 0) {
		/* set clock to the time or last FN should have been
		 * transmitted. */
		tv_clock->tv_sec = tv_now.tv_sec;
		tv_clock->tv_usec = tv_now.tv_usec +
			(0 - elapsed_fn) * FRAME_DURATION_uS;
		if (tv_clock->tv_usec >= 1000000) {
			tv_clock->tv_sec++;
			tv_clock->tv_usec -= 1000000;
		}
		/* set time to the time our next FN has to be transmitted */
		osmo_timer_schedule(&transceiver_clock_timer, 0,
			FRAME_DURATION_uS * (1 - elapsed_fn));

		return 0;
	}

	/* transmit what we still need to transmit */
	while (fn != transceiver_last_fn) {
		transceiver_last_fn = (transceiver_last_fn + 1) % GSM_HYPERFRAME;
		trx_sched_fn(bts, transceiver_last_fn);
	}

	/* schedule next FN to be transmitted */
	memcpy(tv_clock, &tv_now, sizeof(struct timeval));
	osmo_timer_schedule(&transceiver_clock_timer, 0, FRAME_DURATION_uS);

	return 0;
}

void _sched_act_rach_det(struct l1sched_trx *l1t, uint8_t tn, uint8_t ss, int activate)
{
	struct phy_instance *pinst = trx_phy_instance(l1t->trx);
	struct trx_l1h *l1h = pinst->u.osmotrx.hdl;

	if (activate)
		trx_if_cmd_handover(l1h, tn, ss);
	else
		trx_if_cmd_nohandover(l1h, tn, ss);
}
