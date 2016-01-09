/* Scheduler worker functiosn for Virtua OsmoBTS */

/* (C) 2015 by Harald Welte <laforge@gnumonks.org>
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
#include <osmocom/core/bits.h>
#include <osmocom/core/gsmtap_util.h>

#include <osmocom/netif/rtp.h>

#include <osmo-bts/gsm_data.h>
#include <osmo-bts/phy_link.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/rsl.h>
#include <osmo-bts/l1sap.h>
#include <osmo-bts/amr.h>
#include <osmo-bts/scheduler.h>
#include <osmo-bts/scheduler_backend.h>

#include "virtual_um.h"

extern void *tall_bts_ctx;



static void tx_to_virt_um(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
			  enum trx_chan_type chan, struct msgb *msg)
{
	const struct trx_chan_desc *chdesc = &trx_chan_desc[chan];
	uint8_t ss = 0; //FIXME(chdesc);
	uint8_t gsmtap_chan;
	struct msgb *outmsg;

	gsmtap_chan = chantype_rsl2gsmtap(chdesc->chan_nr, chdesc->link_id);
	outmsg = gsmtap_makemsg(l1t->trx->arfcn, tn, gsmtap_chan, ss, fn,
				0, 0, msgb_l2(msg), msgb_l2len(msg));
	if (outmsg) {
		struct phy_instance *pinst = trx_phy_instance(l1t->trx);
		struct virt_um_inst *virt_um = pinst->phy_link->u.virt.virt_um;
		virt_um_write_msg(virt_um, outmsg);
	}

	/* free message */
	msgb_free(msg);
}

/*
 * TX on downlink
 */

/* an IDLE burst returns nothing. on C0 it is replaced by dummy burst */
ubit_t *tx_idle_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid)
{
	return NULL;
}

ubit_t *tx_fcch_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid)
{
	return NULL;
}

ubit_t *tx_sch_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid)
{
	return NULL;
}

ubit_t *tx_data_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid)
{
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);
	struct gsm_bts_trx_ts *ts = &l1t->trx->ts[tn];
	struct msgb *msg;

	if (bid > 0)
		return NULL;

	/* get mac block from queue */
	msg = _sched_dequeue_prim(l1t, tn, fn, chan);
	if (msg)
		goto got_msg;

	LOGP(DL1P, LOGL_INFO, "%s has not been served !! No prim for "
		"trx=%u ts=%u at fn=%u to transmit.\n", 
		trx_chan_desc[chan].name, l1t->trx->nr, tn, fn);

no_msg:
	return NULL;

got_msg:
	/* check validity of message */
	if (msgb_l2len(msg) != GSM_MACBLOCK_LEN) {
		LOGP(DL1P, LOGL_FATAL, "Prim not 23 bytes, please FIX! "
			"(len=%d)\n", msgb_l2len(msg));
		/* free message */
		msgb_free(msg);
		goto no_msg;
	}

	tx_to_virt_um(l1t, tn, fn, chan, msg);

	return NULL;
}

ubit_t *tx_pdtch_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid)
{
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);
	struct gsm_bts_trx_ts *ts = &l1t->trx->ts[tn];
	struct msgb *msg = NULL; /* make GCC happy */
	int rc;

	if (bid > 0)
		return NULL;

	/* get mac block from queue */
	msg = _sched_dequeue_prim(l1t, tn, fn, chan);
	if (msg)
		goto got_msg;

	LOGP(DL1P, LOGL_INFO, "%s has not been served !! No prim for "
		"trx=%u ts=%u at fn=%u to transmit.\n", 
		trx_chan_desc[chan].name, l1t->trx->nr, tn, fn);

no_msg:
	return NULL;

got_msg:
	tx_to_virt_um(l1t, tn, fn, chan, msg);

	return NULL;
}

static void tx_tch_common(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, struct msgb **_msg_tch,
	struct msgb **_msg_facch, int codec_mode_request)
{
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);
	struct msgb *msg1, *msg2, *msg_tch = NULL, *msg_facch = NULL;
	struct l1sched_chan_state *chan_state = &l1ts->chan_state[chan];
	uint8_t rsl_cmode = chan_state->rsl_cmode;
	uint8_t tch_mode = chan_state->tch_mode;
	struct osmo_phsap_prim *l1sap;
#if 0
	/* handle loss detection of received TCH frames */
	if (rsl_cmode == RSL_CMOD_SPD_SPEECH
	 && ++(chan_state->lost) > 5) {
		uint8_t tch_data[GSM_FR_BYTES];
		int len;

		LOGP(DL1P, LOGL_NOTICE, "Missing TCH bursts detected, sending "
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
			len = amr_compose_payload(tch_data,
				chan_state->codec[chan_state->dl_cmr],
				chan_state->codec[chan_state->dl_ft], 1);
			if (len < 2)
				break;
			memset(tch_data + 2, 0, len - 2);
			_sched_compose_tch_ind(l1t, tn, 0, chan, tch_data, len);
			break;
		default:
inval_mode1:
			LOGP(DL1P, LOGL_ERROR, "TCH mode invalid, please "
				"fix!\n");
			len = 0;
		}
		if (len)
			_sched_compose_tch_ind(l1t, tn, 0, chan, tch_data, len);
	}
#endif

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
					LOGP(DL1P, LOGL_FATAL, "TCH twice, "
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
					LOGP(DL1P, LOGL_FATAL, "FACCH twice, "
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
		LOGP(DL1P, LOGL_FATAL, "Prim not 23 bytes, please FIX! "
			"(len=%d)\n", msgb_l2len(msg_facch));
		/* free message */
		msgb_free(msg_facch);
		msg_facch = NULL;
	}

	/* check validity of message, get AMR ft and cmr */
	if (!msg_facch && msg_tch) {
		int len;
		uint8_t bfi, cmr_codec, ft_codec;
		int cmr, ft, i;

		if (rsl_cmode != RSL_CMOD_SPD_SPEECH) {
			LOGP(DL1P, LOGL_NOTICE, "%s Dropping speech frame, "
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
					LOGP(DL1P, LOGL_NOTICE, "%s "
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
				LOGP(DL1P, LOGL_NOTICE, "%s Transmitting 'bad "
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
				LOGP(DL1P, LOGL_NOTICE, "%s Transmitting 'bad "
					"EFR frame' trx=%u ts=%u at fn=%u.\n",
					trx_chan_desc[chan].name,
					l1t->trx->nr, tn, fn);
				goto free_bad_msg;
			}
			break;
		case GSM48_CMODE_SPEECH_AMR: /* AMR */
#if 0
			len = amr_decompose_payload(msg_tch->l2h,
				msgb_l2len(msg_tch), &cmr_codec, &ft_codec,
				&bfi);
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
				LOGP(DL1P, LOGL_ERROR, "%s Codec (FT = %d) "
					" of RTP frame not in list. "
					"trx=%u ts=%u\n",
					trx_chan_desc[chan].name, ft_codec,
					l1t->trx->nr, tn);
				goto free_bad_msg;
			}
			if (codec_mode_request && chan_state->dl_ft != ft) {
				LOGP(DL1P, LOGL_NOTICE, "%s Codec (FT = %d) "
					" of RTP cannot be changed now, but in "
					"next frame. trx=%u ts=%u\n",
					trx_chan_desc[chan].name, ft_codec,
					l1t->trx->nr, tn);
				goto free_bad_msg;
			}
			chan_state->dl_ft = ft;
			if (bfi) {
				LOGP(DL1P, LOGL_NOTICE, "%s Transmitting 'bad "
					"AMR frame' trx=%u ts=%u at fn=%u.\n",
					trx_chan_desc[chan].name,
					l1t->trx->nr, tn, fn);
				goto free_bad_msg;
			}
#endif
			break;
		default:
inval_mode2:
			LOGP(DL1P, LOGL_ERROR, "TCH mode invalid, please "
				"fix!\n");
			goto free_bad_msg;
		}
		if (len < 0) {
			LOGP(DL1P, LOGL_ERROR, "Cannot send invalid AMR "
				"payload\n");
			goto free_bad_msg;
		}
		if (msgb_l2len(msg_tch) != len) {
			LOGP(DL1P, LOGL_ERROR, "Cannot send payload with "
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

ubit_t *tx_tchf_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid)
{
	struct msgb *msg_tch = NULL, *msg_facch = NULL;
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);
	struct gsm_bts_trx_ts *ts = &l1t->trx->ts[tn];
	struct l1sched_chan_state *chan_state = &l1ts->chan_state[chan];
	uint8_t tch_mode = chan_state->tch_mode;

	if (bid > 0)
		return NULL;

	tx_tch_common(l1t, tn, fn, chan, bid, &msg_tch, &msg_facch,
		(((fn + 4) % 26) >> 2) & 1);

	/* no message at all */
	if (!msg_tch && !msg_facch) {
		LOGP(DL1P, LOGL_INFO, "%s has not been served !! No prim for "
			"trx=%u ts=%u at fn=%u to transmit.\n", 
			trx_chan_desc[chan].name, l1t->trx->nr, tn, fn);
		goto send_burst;
	}

	if (msg_facch) {
		tx_to_virt_um(l1t, tn, fn, chan, msg_facch);
		msgb_free(msg_tch);
	} else
		tx_to_virt_um(l1t, tn, fn, chan, msg_tch);

send_burst:

	return NULL;
}

ubit_t *tx_tchh_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid)
{
	struct msgb *msg_tch = NULL, *msg_facch = NULL;
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);
	struct gsm_bts_trx_ts *ts = &l1t->trx->ts[tn];
	struct l1sched_chan_state *chan_state = &l1ts->chan_state[chan];
	uint8_t tch_mode = chan_state->tch_mode;

	/* send burst, if we already got a frame */
	if (bid > 0)
		return NULL;

	/* get TCH and/or FACCH */
	tx_tch_common(l1t, tn, fn, chan, bid, &msg_tch, &msg_facch,
		(((fn + 4) % 26) >> 2) & 1);

	/* check for FACCH alignment */
	if (msg_facch && ((((fn + 4) % 26) >> 2) & 1)) {
		LOGP(DL1P, LOGL_ERROR, "%s Cannot transmit FACCH starting on "
			"even frames, please fix RTS!\n",
			trx_chan_desc[chan].name);
		msgb_free(msg_facch);
		msg_facch = NULL;
	}

	/* no message at all */
	if (!msg_tch && !msg_facch && !chan_state->dl_ongoing_facch) {
		LOGP(DL1P, LOGL_INFO, "%s has not been served !! No prim for "
			"trx=%u ts=%u at fn=%u to transmit.\n", 
			trx_chan_desc[chan].name, l1t->trx->nr, tn, fn);
		goto send_burst;
	}

	if (msg_facch) {
		tx_to_virt_um(l1t, tn, fn, chan, msg_facch);
		msgb_free(msg_tch);
	} else
		tx_to_virt_um(l1t, tn, fn, chan, msg_tch);

send_burst:
	return NULL;
}


/***********************************************************************
 * RX on uplink (indication to upper layer)
 ***********************************************************************/

/* we don't use those functions, as we feed the MAC frames from GSMTAP
 * directly into the L1SAP, bypassing the TDMA multiplex logic oriented
 * towards receiving bursts */

int rx_rach_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, sbit_t *bits, int8_t rssi,
	float toa)
{
	return 0;
}

/*! \brief a single burst was received by the PHY, process it */
int rx_data_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, sbit_t *bits, int8_t rssi,
	float toa)
{
	return 0;
}

int rx_pdtch_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, sbit_t *bits, int8_t rssi,
	float toa)
{
	return 0;
}

int rx_tchf_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, sbit_t *bits, int8_t rssi,
	float toa)
{
	return 0;
}

int rx_tchh_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, sbit_t *bits, int8_t rssi,
	float toa)
{
	return 0;
}

void _sched_act_rach_det(struct l1sched_trx *l1t, uint8_t tn, uint8_t ss, int activate)
{
}

/***********************************************************************
 * main scheduler function
 ***********************************************************************/

#define RTS_ADVANCE		5	/* about 20ms */
#define FRAME_DURATION_uS	4615

static int vbts_sched_fn(struct gsm_bts *bts, uint32_t fn)
{
	struct gsm_bts_trx *trx;

	/* send time indication */
	l1if_mph_time_ind(bts, fn);

	/* advance the frame number? */

	llist_for_each_entry(trx, &bts->trx_list, list) {
		struct phy_instance *pinst = trx_phy_instance(trx);
		struct l1sched_trx *l1t = &pinst->u.virt.sched;
		int tn;

		for (tn = 0; tn < ARRAY_SIZE(l1t->ts); tn++) {
			/* Generate RTS.ind to higher layers */
			_sched_rts(l1t, tn, (fn + RTS_ADVANCE) % GSM_HYPERFRAME);
			/* schedule transmit backend functions */
			_sched_dl_burst(l1t, tn, fn);
		}
	}

	return 0;
}

static void vbts_fn_timer_cb(void *data)
{
	struct gsm_bts *bts = data;
	struct gsm_bts_role_bts *btsb = bts_role_bts(bts);
	struct timeval tv_now;
	struct timeval *tv_clock = &btsb->vbts.tv_clock;
	int32_t elapsed_us;

	gettimeofday(&tv_now, NULL);

	elapsed_us = (tv_now.tv_sec - tv_clock->tv_sec) * 1000000 +
		     (tv_now.tv_usec - tv_clock->tv_usec);

	if (elapsed_us > 2*FRAME_DURATION_uS)
		LOGP(DL1P, LOGL_NOTICE, "vbts_fn_timer_cb after %d us\n", elapsed_us);

	while (elapsed_us > FRAME_DURATION_uS / 2) {
		const struct timeval tv_frame = {
			.tv_sec = 0,
			.tv_usec = FRAME_DURATION_uS,
		};
		timeradd(tv_clock, &tv_frame, tv_clock);
		btsb->vbts.last_fn = (btsb->vbts.last_fn + 1) % GSM_HYPERFRAME;
		vbts_sched_fn(bts, btsb->vbts.last_fn);
		elapsed_us -= FRAME_DURATION_uS;
	}

	/* re-schedule the timer */
	osmo_timer_schedule(&btsb->vbts.fn_timer, 0, FRAME_DURATION_uS - elapsed_us);
}

int vbts_sched_start(struct gsm_bts *bts)
{
	struct gsm_bts_role_bts *btsb = bts_role_bts(bts);

	LOGP(DL1P, LOGL_NOTICE, "starting VBTS scheduler\n");

	memset(&btsb->vbts.fn_timer, 0, sizeof(btsb->vbts.fn_timer));
	btsb->vbts.fn_timer.cb = vbts_fn_timer_cb;
	btsb->vbts.fn_timer.data = bts;

	gettimeofday(&btsb->vbts.tv_clock, NULL);
	osmo_timer_schedule(&btsb->vbts.fn_timer, 0, FRAME_DURATION_uS);

	return 0;
}
