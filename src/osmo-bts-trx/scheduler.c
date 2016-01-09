/* Scheduler for OsmoBTS-TRX */

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
#include <osmocom/core/bits.h>
#include <osmocom/gsm/a5.h>

#include <osmo-bts/gsm_data.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/rsl.h>
#include <osmo-bts/l1sap.h>

#include "l1_if.h"
#include "scheduler.h"
#include "scheduler_backend.h"
#include "trx_if.h"

extern void *tall_bts_ctx;

static struct gsm_bts *bts;

/* clock states */
static uint32_t transceiver_lost;
uint32_t transceiver_last_fn;
static struct timeval transceiver_clock_tv;
static struct osmo_timer_list transceiver_clock_timer;

/* clock advance for the transceiver */
uint32_t trx_clock_advance = 20;

/* advance RTS to give some time for data processing. (especially PCU) */
uint32_t trx_rts_advance = 5; /* about 20ms */

static int rts_data_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan);
static int rts_tchf_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan);
static int rts_tchh_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan);
/*! \brief Dummy Burst (TS 05.02 Chapter 5.2.6) */
static const ubit_t dummy_burst[148] = {
	0,0,0,
	1,1,1,1,1,0,1,1,0,1,1,1,0,1,1,0,0,0,0,0,1,0,1,0,0,1,0,0,1,1,1,0,
	0,0,0,0,1,0,0,1,0,0,0,1,0,0,0,0,0,0,0,1,1,1,1,1,0,0,0,1,1,1,0,0,
	0,1,0,1,1,1,0,0,0,1,0,1,1,1,0,0,0,1,0,1,0,1,1,1,0,1,0,0,1,0,1,0,
	0,0,1,1,0,0,1,1,0,0,1,1,1,0,0,1,1,1,1,0,1,0,0,1,1,1,1,1,0,0,0,1,
	0,0,1,0,1,1,1,1,1,0,1,0,1,0,
	0,0,0,
};

/*! \brief FCCH Burst (TS 05.02 Chapter 5.2.4) */
const ubit_t _sched_fcch_burst[148] = {
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
};

/*! \brief Training Sequences (TS 05.02 Chapter 5.2.3) */
const ubit_t _sched_tsc[8][26] = {
	{ 0,0,1,0,0,1,0,1,1,1,0,0,0,0,1,0,0,0,1,0,0,1,0,1,1,1, },
	{ 0,0,1,0,1,1,0,1,1,1,0,1,1,1,1,0,0,0,1,0,1,1,0,1,1,1, },
	{ 0,1,0,0,0,0,1,1,1,0,1,1,1,0,1,0,0,1,0,0,0,0,1,1,1,0, },
	{ 0,1,0,0,0,1,1,1,1,0,1,1,0,1,0,0,0,1,0,0,0,1,1,1,1,0, },
	{ 0,0,0,1,1,0,1,0,1,1,1,0,0,1,0,0,0,0,0,1,1,0,1,0,1,1, },
	{ 0,1,0,0,1,1,1,0,1,0,1,1,0,0,0,0,0,1,0,0,1,1,1,0,1,0, },
	{ 1,0,1,0,0,1,1,1,1,1,0,1,1,0,0,0,1,0,1,0,0,1,1,1,1,1, },
	{ 1,1,1,0,1,1,1,1,0,0,0,1,0,0,1,0,1,1,1,0,1,1,1,1,0,0, },
};

/*! \brief SCH trainign sequence (TS 05.02 Chapter 5.2.5) */
const ubit_t _sched_sch_train[64] = {
	1,0,1,1,1,0,0,1,0,1,1,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,1,1,1,
	0,0,1,0,1,1,0,1,0,1,0,0,0,1,0,1,0,1,1,1,0,1,1,0,0,0,0,1,1,0,1,1,
};

/*
 * subchannel description structure
 */

const struct trx_chan_desc trx_chan_desc[_TRX_CHAN_MAX] = {
      {	0,	TRXC_IDLE,	0,	0,	"IDLE",		NULL,		tx_idle_fn,	NULL,		1 },
      {	0,	TRXC_FCCH,	0,	0,	"FCCH",		NULL,		tx_fcch_fn,	NULL,		1 },
      {	0,	TRXC_SCH,	0,	0,	"SCH",		NULL,		tx_sch_fn,	NULL,		1 },
      {	0,	TRXC_BCCH,	0x80,	0x00,	"BCCH",		rts_data_fn,	tx_data_fn,	NULL,		1 },
      {	0,	TRXC_RACH,	0x88,	0x00,	"RACH",		NULL,		NULL,		rx_rach_fn,	1 },
      {	0,	TRXC_CCCH,	0x90,	0x00,	"CCCH",		rts_data_fn,	tx_data_fn,	NULL,		1 },
      {	0,	TRXC_TCHF,	0x08,	0x00,	"TCH/F",	rts_tchf_fn,	tx_tchf_fn,	rx_tchf_fn,	0 },
      {	0,	TRXC_TCHH_0,	0x10,	0x00,	"TCH/H(0)",	rts_tchh_fn,	tx_tchh_fn,	rx_tchh_fn,	0 },
      {	0,	TRXC_TCHH_1,	0x18,	0x00,	"TCH/H(1)",	rts_tchh_fn,	tx_tchh_fn,	rx_tchh_fn,	0 },
      {	0,	TRXC_SDCCH4_0,	0x20,	0x00,	"SDCCH/4(0)",	rts_data_fn,	tx_data_fn,	rx_data_fn,	0 },
      {	0,	TRXC_SDCCH4_1,	0x28,	0x00,	"SDCCH/4(1)",	rts_data_fn,	tx_data_fn,	rx_data_fn,	0 },
      {	0,	TRXC_SDCCH4_2,	0x30,	0x00,	"SDCCH/4(2)",	rts_data_fn,	tx_data_fn,	rx_data_fn,	0 },
      {	0,	TRXC_SDCCH4_3,	0x38,	0x00,	"SDCCH/4(3)",	rts_data_fn,	tx_data_fn,	rx_data_fn,	0 },
      {	0,	TRXC_SDCCH8_0,	0x40,	0x00,	"SDCCH/8(0)",	rts_data_fn,	tx_data_fn,	rx_data_fn,	0 },
      {	0,	TRXC_SDCCH8_1,	0x48,	0x00,	"SDCCH/8(1)",	rts_data_fn,	tx_data_fn,	rx_data_fn,	0 },
      {	0,	TRXC_SDCCH8_2,	0x50,	0x00,	"SDCCH/8(2)",	rts_data_fn,	tx_data_fn,	rx_data_fn,	0 },
      {	0,	TRXC_SDCCH8_3,	0x58,	0x00,	"SDCCH/8(3)",	rts_data_fn,	tx_data_fn,	rx_data_fn,	0 },
      {	0,	TRXC_SDCCH8_4,	0x60,	0x00,	"SDCCH/8(4)",	rts_data_fn,	tx_data_fn,	rx_data_fn,	0 },
      {	0,	TRXC_SDCCH8_5,	0x68,	0x00,	"SDCCH/8(5)",	rts_data_fn,	tx_data_fn,	rx_data_fn,	0 },
      {	0,	TRXC_SDCCH8_6,	0x70,	0x00,	"SDCCH/8(6)",	rts_data_fn,	tx_data_fn,	rx_data_fn,	0 },
      {	0,	TRXC_SDCCH8_7,	0x78,	0x00,	"SDCCH/8(7)",	rts_data_fn,	tx_data_fn,	rx_data_fn,	0 },
      {	0,	TRXC_SACCHTF,	0x08,	0x40,	"SACCH/TF",	rts_data_fn,	tx_data_fn,	rx_data_fn,	0 },
      {	0,	TRXC_SACCHTH_0,	0x10,	0x40,	"SACCH/TH(0)",	rts_data_fn,	tx_data_fn,	rx_data_fn,	0 },
      {	0,	TRXC_SACCHTH_1,	0x18,	0x40,	"SACCH/TH(1)",	rts_data_fn,	tx_data_fn,	rx_data_fn,	0 },
      {	0,	TRXC_SACCH4_0, 	0x20,	0x40,	"SACCH/4(0)",	rts_data_fn,	tx_data_fn,	rx_data_fn,	0 },
      {	0,	TRXC_SACCH4_1,	0x28,	0x40,	"SACCH/4(1)",	rts_data_fn,	tx_data_fn,	rx_data_fn,	0 },
      {	0,	TRXC_SACCH4_2,	0x30,	0x40,	"SACCH/4(2)",	rts_data_fn,	tx_data_fn,	rx_data_fn,	0 },
      {	0,	TRXC_SACCH4_3,	0x38,	0x40,	"SACCH/4(3)",	rts_data_fn,	tx_data_fn,	rx_data_fn,	0 },
      {	0,	TRXC_SACCH8_0,	0x40,	0x40,	"SACCH/8(0)",	rts_data_fn,	tx_data_fn,	rx_data_fn,	0 },
      {	0,	TRXC_SACCH8_1,	0x48,	0x40,	"SACCH/8(1)",	rts_data_fn,	tx_data_fn,	rx_data_fn,	0 },
      {	0,	TRXC_SACCH8_2,	0x50,	0x40,	"SACCH/8(2)",	rts_data_fn,	tx_data_fn,	rx_data_fn,	0 },
      {	0,	TRXC_SACCH8_3,	0x58,	0x40,	"SACCH/8(3)",	rts_data_fn,	tx_data_fn,	rx_data_fn,	0 },
      {	0,	TRXC_SACCH8_4,	0x60,	0x40,	"SACCH/8(4)",	rts_data_fn,	tx_data_fn,	rx_data_fn,	0 },
      {	0,	TRXC_SACCH8_5,	0x68,	0x40,	"SACCH/8(5)",	rts_data_fn,	tx_data_fn,	rx_data_fn,	0 },
      {	0,	TRXC_SACCH8_6,	0x70,	0x40,	"SACCH/8(6)",	rts_data_fn,	tx_data_fn,	rx_data_fn,	0 },
      {	0,	TRXC_SACCH8_7,	0x78,	0x40,	"SACCH/8(7)",	rts_data_fn,	tx_data_fn,	rx_data_fn,	0 },
      {	1,	TRXC_PDTCH,	0x08,	0x00,	"PDTCH",	rts_data_fn,	tx_pdtch_fn,	rx_pdtch_fn,	0 },
      {	1,	TRXC_PTCCH,	0x08,	0x00,	"PTCCH",	rts_data_fn,	tx_data_fn,	rx_data_fn,	0 },
};


/*
 * init / exit
 */

int trx_sched_init(struct l1sched_trx *l1t)
{
	uint8_t tn;
	int i;

	LOGP(DL1C, LOGL_NOTICE, "Init scheduler for trx=%u\n", l1t->trx->nr);

	for (tn = 0; tn < ARRAY_SIZE(l1t->ts); tn++) {
		struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);

		l1ts->mf_index = 0;
		l1ts->mf_last_fn = 0;
		INIT_LLIST_HEAD(&l1ts->dl_prims);
		for (i = 0; i < ARRAY_SIZE(&l1ts->chan_state); i++) {
			struct l1sched_chan_state *chan_state;
			chan_state = &l1ts->chan_state[i];
			chan_state->active = 0;
		}
	}

	return 0;
}

void trx_sched_exit(struct l1sched_trx *l1t)
{
	struct gsm_bts_trx_ts *ts;
	uint8_t tn;
	int i;

	LOGP(DL1C, LOGL_NOTICE, "Exit scheduler for trx=%u\n", l1t->trx->nr);

	for (tn = 0; tn < ARRAY_SIZE(l1t->ts); tn++) {
		struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);
		msgb_queue_flush(&l1ts->dl_prims);
		for (i = 0; i < _TRX_CHAN_MAX; i++) {
			struct l1sched_chan_state *chan_state;
			chan_state = &l1ts->chan_state[i];
			if (chan_state->dl_bursts) {
				talloc_free(chan_state->dl_bursts);
				chan_state->dl_bursts = NULL;
			}
			if (chan_state->ul_bursts) {
				talloc_free(chan_state->ul_bursts);
				chan_state->ul_bursts = NULL;
			}
		}
		/* clear lchan channel states */
		ts = &l1t->trx->ts[tn];
		for (i = 0; i < ARRAY_SIZE(ts->lchan); i++)
			lchan_set_state(&ts->lchan[i], LCHAN_S_NONE);
	}
}

/* close all logical channels and reset timeslots */
void trx_sched_reset(struct l1sched_trx *l1t)
{
	trx_sched_exit(l1t);
	trx_sched_init(l1t);
}

struct msgb *_sched_dequeue_prim(struct l1sched_trx *l1t, int8_t tn, uint32_t fn,
				 enum trx_chan_type chan)
{
	struct msgb *msg, *msg2;
	struct osmo_phsap_prim *l1sap;
	uint32_t prim_fn;
	uint8_t chan_nr, link_id;
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);

	/* get prim of current fn from queue */
	llist_for_each_entry_safe(msg, msg2, &l1ts->dl_prims, list) {
		l1sap = msgb_l1sap_prim(msg);
		if (l1sap->oph.operation != PRIM_OP_REQUEST) {
wrong_type:
			LOGP(DL1C, LOGL_ERROR, "Prim for ts=%u at fn=%u has "
				"wrong type.\n", tn, fn);
free_msg:
			/* unlink and free message */
			llist_del(&msg->list);
			msgb_free(msg);
			return NULL;
		}
		switch (l1sap->oph.primitive) {
		case PRIM_PH_DATA:
			chan_nr = l1sap->u.data.chan_nr;
			link_id = l1sap->u.data.link_id;
			prim_fn = ((l1sap->u.data.fn + GSM_HYPERFRAME - fn) % GSM_HYPERFRAME);
			break;
		case PRIM_TCH:
			chan_nr = l1sap->u.tch.chan_nr;
			link_id = 0;
			prim_fn = ((l1sap->u.tch.fn + GSM_HYPERFRAME - fn) % GSM_HYPERFRAME);
			break;
		default:
			goto wrong_type;
		}
		if (prim_fn > 100) {
			LOGP(DL1C, LOGL_NOTICE, "Prim for trx=%u ts=%u at fn=%u "
				"is out of range, or channel already disabled. "
				"If this happens in conjunction with PCU, "
				"increase 'rts-advance' by 5. (current fn=%u)\n",
				l1t->trx->nr, tn, l1sap->u.data.fn, fn);
			/* unlink and free message */
			llist_del(&msg->list);
			msgb_free(msg);
			continue;
		}
		if (prim_fn > 0)
			continue;

		goto found_msg;
	}

	return NULL;

found_msg:
	if ((chan_nr ^ (trx_chan_desc[chan].chan_nr | tn))
	 || ((link_id & 0xc0) ^ trx_chan_desc[chan].link_id)) {
		LOGP(DL1C, LOGL_ERROR, "Prim for ts=%u at fn=%u has wrong "
			"chan_nr=%02x link_id=%02x, expecting chan_nr=%02x "
			"link_id=%02x.\n", tn, fn, chan_nr, link_id,
			trx_chan_desc[chan].chan_nr | tn,
			trx_chan_desc[chan].link_id);
		goto free_msg;
	}

	/* unlink and return message */
	llist_del(&msg->list);
	return msg;
}

int _sched_compose_ph_data_ind(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
				enum trx_chan_type chan, uint8_t *l2, uint8_t l2_len, float rssi)
{
	struct msgb *msg;
	struct osmo_phsap_prim *l1sap;
	uint8_t chan_nr = trx_chan_desc[chan].chan_nr | tn;
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);

	/* compose primitive */
	msg = l1sap_msgb_alloc(l2_len);
	l1sap = msgb_l1sap_prim(msg);
	osmo_prim_init(&l1sap->oph, SAP_GSM_PH, PRIM_PH_DATA,
		PRIM_OP_INDICATION, msg);
	l1sap->u.data.chan_nr = chan_nr;
	l1sap->u.data.link_id = trx_chan_desc[chan].link_id;
	l1sap->u.data.fn = fn;
	l1sap->u.data.rssi = (int8_t) (rssi);
	msg->l2h = msgb_put(msg, l2_len);
	if (l2_len)
		memcpy(msg->l2h, l2, l2_len);

	if (L1SAP_IS_LINK_SACCH(trx_chan_desc[chan].link_id))
		l1ts->chan_state[chan].lost = 0;

	/* forward primitive */
	l1sap_up(l1t->trx, l1sap);

	return 0;
}

int _sched_compose_tch_ind(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
		    enum trx_chan_type chan, uint8_t *tch, uint8_t tch_len)
{
	struct msgb *msg;
	struct osmo_phsap_prim *l1sap;
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);

	/* compose primitive */
	msg = l1sap_msgb_alloc(tch_len);
	l1sap = msgb_l1sap_prim(msg);
	osmo_prim_init(&l1sap->oph, SAP_GSM_PH, PRIM_TCH,
		PRIM_OP_INDICATION, msg);
	l1sap->u.tch.chan_nr = trx_chan_desc[chan].chan_nr | tn;
	l1sap->u.tch.fn = fn;
	msg->l2h = msgb_put(msg, tch_len);
	if (tch_len)
		memcpy(msg->l2h, tch, tch_len);

	if (l1ts->chan_state[chan].lost)
		l1ts->chan_state[chan].lost--;

	/* forward primitive */
	l1sap_up(l1t->trx, l1sap);

	return 0;
}



/*
 * data request (from upper layer)
 */

int trx_sched_ph_data_req(struct l1sched_trx *l1t, struct osmo_phsap_prim *l1sap)
{
	uint8_t tn = l1sap->u.data.chan_nr & 7;
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);

	LOGP(DL1C, LOGL_INFO, "PH-DATA.req: chan_nr=0x%02x link_id=0x%02x "
		"fn=%u ts=%u trx=%u\n", l1sap->u.data.chan_nr,
		l1sap->u.data.link_id, l1sap->u.data.fn, tn, l1t->trx->nr);

	if (!l1sap->oph.msg)
		abort();

	/* ignore empty frame */
	if (!msgb_l2len(l1sap->oph.msg)) {
		msgb_free(l1sap->oph.msg);
		return 0;
	}

	msgb_enqueue(&l1ts->dl_prims, l1sap->oph.msg);

	return 0;
}

int trx_sched_tch_req(struct l1sched_trx *l1t, struct osmo_phsap_prim *l1sap)
{
	uint8_t tn = l1sap->u.tch.chan_nr & 7;
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);

	LOGP(DL1C, LOGL_INFO, "TCH.req: chan_nr=0x%02x "
		"fn=%u ts=%u trx=%u\n", l1sap->u.tch.chan_nr,
		l1sap->u.tch.fn, tn, l1t->trx->nr);

	if (!l1sap->oph.msg)
		abort();

	/* ignore empty frame */
	if (!msgb_l2len(l1sap->oph.msg)) {
		msgb_free(l1sap->oph.msg);
		return 0;
	}

	msgb_enqueue(&l1ts->dl_prims, l1sap->oph.msg);

	return 0;
}


/* 
 * ready-to-send indication (to upper layer)
 */

/* RTS for data frame */
static int rts_data_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan)
{
	uint8_t chan_nr, link_id;
	struct msgb *msg;
	struct osmo_phsap_prim *l1sap;

	/* get data for RTS indication */
	chan_nr = trx_chan_desc[chan].chan_nr | tn;
	link_id = trx_chan_desc[chan].link_id;

	if (!chan_nr) {
		LOGP(DL1C, LOGL_FATAL, "RTS func for %s with non-existing "
			"chan_nr %d\n", trx_chan_desc[chan].name, chan_nr);
		return -ENODEV;
	}

	LOGP(DL1C, LOGL_INFO, "PH-RTS.ind: chan=%s chan_nr=0x%02x "
		"link_id=0x%02x fn=%u ts=%u trx=%u\n", trx_chan_desc[chan].name,
		chan_nr, link_id, fn, tn, l1t->trx->nr);

	/* generate prim */
	msg = l1sap_msgb_alloc(200);
	if (!msg)
		return -ENOMEM;
	l1sap = msgb_l1sap_prim(msg);
	osmo_prim_init(&l1sap->oph, SAP_GSM_PH, PRIM_PH_RTS,
	                                PRIM_OP_INDICATION, msg);
	l1sap->u.data.chan_nr = chan_nr;
	l1sap->u.data.link_id = link_id;
	l1sap->u.data.fn = fn;

	return l1sap_up(l1t->trx, l1sap);
}

static int rts_tch_common(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, int facch)
{
	uint8_t chan_nr, link_id;
	struct msgb *msg;
	struct osmo_phsap_prim *l1sap;
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);
	int rc = 0;

	/* get data for RTS indication */
	chan_nr = trx_chan_desc[chan].chan_nr | tn;
	link_id = trx_chan_desc[chan].link_id;

	if (!chan_nr) {
		LOGP(DL1C, LOGL_FATAL, "RTS func for %s with non-existing "
			"chan_nr %d\n", trx_chan_desc[chan].name, chan_nr);
		return -ENODEV;
	}

	LOGP(DL1C, LOGL_INFO, "TCH RTS.ind: chan=%s chan_nr=0x%02x "
		"fn=%u ts=%u trx=%u\n", trx_chan_desc[chan].name,
		chan_nr, fn, tn, l1t->trx->nr);

	/* only send, if FACCH is selected */
	if (facch) {
		/* generate prim */
		msg = l1sap_msgb_alloc(200);
		if (!msg)
			return -ENOMEM;
		l1sap = msgb_l1sap_prim(msg);
		osmo_prim_init(&l1sap->oph, SAP_GSM_PH, PRIM_PH_RTS,
						PRIM_OP_INDICATION, msg);
		l1sap->u.data.chan_nr = chan_nr;
		l1sap->u.data.link_id = link_id;
		l1sap->u.data.fn = fn;

		rc = l1sap_up(l1t->trx, l1sap);
	}

	/* dont send, if TCH is in signalling only mode */
	if (l1ts->chan_state[chan].rsl_cmode != RSL_CMOD_SPD_SIGN) {
		/* generate prim */
		msg = l1sap_msgb_alloc(200);
		if (!msg)
			return -ENOMEM;
		l1sap = msgb_l1sap_prim(msg);
		osmo_prim_init(&l1sap->oph, SAP_GSM_PH, PRIM_TCH_RTS,
						PRIM_OP_INDICATION, msg);
		l1sap->u.tch.chan_nr = chan_nr;
		l1sap->u.tch.fn = fn;

		return l1sap_up(l1t->trx, l1sap);
	}

	return rc;
}

/* RTS for full rate traffic frame */
static int rts_tchf_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan)
{
	/* TCH/F may include FACCH on every 4th burst */
	return rts_tch_common(l1t, tn, fn, chan, 1);
}


/* RTS for half rate traffic frame */
static int rts_tchh_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan)
{
	/* the FN 4/5, 13/14, 21/22 defines that FACCH may be included. */
	return rts_tch_common(l1t, tn, fn, chan, ((fn % 26) >> 2) & 1);
}

/*
 * multiframe structure
 */

/* frame structures */
struct trx_sched_frame {
	/*! \brief downlink TRX channel type */
	enum trx_chan_type		dl_chan;
	/*! \brief downlink block ID */
	uint8_t				dl_bid;
	/*! \breff uplink TRX channel type */
	enum trx_chan_type		ul_chan;
	/*! \brief uplink block ID */
	uint8_t				ul_bid;
};

static const struct trx_sched_frame frame_bcch[51] = {
/*	dl_chan		dl_bid	ul_chan		ul_bid */
      {	TRXC_FCCH,	0,	TRXC_RACH,	0 },
      {	TRXC_SCH,	0,	TRXC_RACH,	0 },
      { TRXC_BCCH,	0,	TRXC_RACH,	0 }, { TRXC_BCCH,	1,	TRXC_RACH,	0 }, { TRXC_BCCH,	2,	TRXC_RACH,	0 }, { TRXC_BCCH,	3,	TRXC_RACH,	0 },
      { TRXC_CCCH,	0,	TRXC_RACH,	0 }, { TRXC_CCCH,	1,	TRXC_RACH,	0 }, { TRXC_CCCH,	2,	TRXC_RACH,	0 }, { TRXC_CCCH,	3,	TRXC_RACH,	0 },
      {	TRXC_FCCH,	0,	TRXC_RACH,	0 },
      {	TRXC_SCH,	0,	TRXC_RACH,	0 },
      { TRXC_CCCH,	0,	TRXC_RACH,	0 }, { TRXC_CCCH,	1,	TRXC_RACH,	0 }, { TRXC_CCCH,	2,	TRXC_RACH,	0 }, { TRXC_CCCH,	3,	TRXC_RACH,	0 },
      { TRXC_CCCH,	0,	TRXC_RACH,	0 }, { TRXC_CCCH,	1,	TRXC_RACH,	0 }, { TRXC_CCCH,	2,	TRXC_RACH,	0 }, { TRXC_CCCH,	3,	TRXC_RACH,	0 },
      {	TRXC_FCCH,	0,	TRXC_RACH,	0 },
      {	TRXC_SCH,	0,	TRXC_RACH,	0 },
      { TRXC_CCCH,	0,	TRXC_RACH,	0 }, { TRXC_CCCH,	1,	TRXC_RACH,	0 }, { TRXC_CCCH,	2,	TRXC_RACH,	0 }, { TRXC_CCCH,	3,	TRXC_RACH,	0 },
      { TRXC_CCCH,	0,	TRXC_RACH,	0 }, { TRXC_CCCH,	1,	TRXC_RACH,	0 }, { TRXC_CCCH,	2,	TRXC_RACH,	0 }, { TRXC_CCCH,	3,	TRXC_RACH,	0 },
      {	TRXC_FCCH,	0,	TRXC_RACH,	0 },
      {	TRXC_SCH,	0,	TRXC_RACH,	0 },
      { TRXC_CCCH,	0,	TRXC_RACH,	0 }, { TRXC_CCCH,	1,	TRXC_RACH,	0 }, { TRXC_CCCH,	2,	TRXC_RACH,	0 }, { TRXC_CCCH,	3,	TRXC_RACH,	0 },
      { TRXC_CCCH,	0,	TRXC_RACH,	0 }, { TRXC_CCCH,	1,	TRXC_RACH,	0 }, { TRXC_CCCH,	2,	TRXC_RACH,	0 }, { TRXC_CCCH,	3,	TRXC_RACH,	0 },
      {	TRXC_FCCH,	0,	TRXC_RACH,	0 },
      {	TRXC_SCH,	0,	TRXC_RACH,	0 },
      { TRXC_CCCH,	0,	TRXC_RACH,	0 }, { TRXC_CCCH,	1,	TRXC_RACH,	0 }, { TRXC_CCCH,	2,	TRXC_RACH,	0 }, { TRXC_CCCH,	3,	TRXC_RACH,	0 },
      { TRXC_CCCH,	0,	TRXC_RACH,	0 }, { TRXC_CCCH,	1,	TRXC_RACH,	0 }, { TRXC_CCCH,	2,	TRXC_RACH,	0 }, { TRXC_CCCH,	3,	TRXC_RACH,	0 },
      {	TRXC_IDLE,	0,	TRXC_RACH,	0 },
};

static const struct trx_sched_frame frame_bcch_sdcch4[102] = {
/*	dl_chan		dl_bid	ul_chan		ul_bid */
      {	TRXC_FCCH,	0,	TRXC_SDCCH4_3,	0 },
      {	TRXC_SCH,	0,	TRXC_SDCCH4_3,	1 },
      { TRXC_BCCH,	0,	TRXC_SDCCH4_3,	2 },
      { TRXC_BCCH,	1,	TRXC_SDCCH4_3,	3 },
      { TRXC_BCCH,	2,	TRXC_RACH,	0 },
      { TRXC_BCCH,	3,	TRXC_RACH,	0 },
      { TRXC_CCCH,	0,	TRXC_SACCH4_2,	0 },
      { TRXC_CCCH,	1,	TRXC_SACCH4_2,	1 },
      { TRXC_CCCH,	2,	TRXC_SACCH4_2,	2 },
      { TRXC_CCCH,	3,	TRXC_SACCH4_2,	3 },
      {	TRXC_FCCH,	0,	TRXC_SACCH4_3,	0 },
      {	TRXC_SCH,	0,	TRXC_SACCH4_3,	1 },
      { TRXC_CCCH,	0,	TRXC_SACCH4_3,	2 },
      { TRXC_CCCH,	1,	TRXC_SACCH4_3,	3 },
      { TRXC_CCCH,	2,	TRXC_RACH,	0 },
      { TRXC_CCCH,	3,	TRXC_RACH,	0 },
      { TRXC_CCCH,	0,	TRXC_RACH,	0 },
      { TRXC_CCCH,	1,	TRXC_RACH,	0 },
      { TRXC_CCCH,	2,	TRXC_RACH,	0 },
      { TRXC_CCCH,	3,	TRXC_RACH,	0 },
      {	TRXC_FCCH,	0,	TRXC_RACH,	0 },
      {	TRXC_SCH,	0,	TRXC_RACH,	0 },
      { TRXC_SDCCH4_0,	0,	TRXC_RACH,	0 },
      { TRXC_SDCCH4_0,	1,	TRXC_RACH,	0 },
      { TRXC_SDCCH4_0,	2,	TRXC_RACH,	0 },
      { TRXC_SDCCH4_0,	3,	TRXC_RACH,	0 },
      { TRXC_SDCCH4_1,	0,	TRXC_RACH,	0 },
      { TRXC_SDCCH4_1,	1,	TRXC_RACH,	0 },
      { TRXC_SDCCH4_1,	2,	TRXC_RACH,	0 },
      { TRXC_SDCCH4_1,	3,	TRXC_RACH,	0 },
      {	TRXC_FCCH,	0,	TRXC_RACH,	0 },
      {	TRXC_SCH,	0,	TRXC_RACH,	0 },
      { TRXC_SDCCH4_2,	0,	TRXC_RACH,	0 },
      { TRXC_SDCCH4_2,	1,	TRXC_RACH,	0 },
      { TRXC_SDCCH4_2,	2,	TRXC_RACH,	0 },
      { TRXC_SDCCH4_2,	3,	TRXC_RACH,	0 },
      { TRXC_SDCCH4_3,	0,	TRXC_RACH,	0 },
      { TRXC_SDCCH4_3,	1,	TRXC_SDCCH4_0,	0 },
      { TRXC_SDCCH4_3,	2,	TRXC_SDCCH4_0,	1 },
      { TRXC_SDCCH4_3,	3,	TRXC_SDCCH4_0,	2 },
      {	TRXC_FCCH,	0,	TRXC_SDCCH4_0,	3 },
      {	TRXC_SCH,	0,	TRXC_SDCCH4_1,	0 },
      { TRXC_SACCH4_0,	0,	TRXC_SDCCH4_1,	1 },
      { TRXC_SACCH4_0,	1,	TRXC_SDCCH4_1,	2 },
      { TRXC_SACCH4_0,	2,	TRXC_SDCCH4_1,	3 },
      { TRXC_SACCH4_0,	3,	TRXC_RACH,	0 },
      { TRXC_SACCH4_1,	0,	TRXC_RACH,	0 },
      { TRXC_SACCH4_1,	1,	TRXC_SDCCH4_2,	0 },
      { TRXC_SACCH4_1,	2,	TRXC_SDCCH4_2,	1 },
      { TRXC_SACCH4_1,	3,	TRXC_SDCCH4_2,	2 },
      {	TRXC_IDLE,	0,	TRXC_SDCCH4_2,	3 },

      {	TRXC_FCCH,	0,	TRXC_SDCCH4_3,	0 },
      {	TRXC_SCH,	0,	TRXC_SDCCH4_3,	1 },
      { TRXC_BCCH,	0,	TRXC_SDCCH4_3,	2 },
      { TRXC_BCCH,	1,	TRXC_SDCCH4_3,	3 },
      { TRXC_BCCH,	2,	TRXC_RACH,	0 },
      { TRXC_BCCH,	3,	TRXC_RACH,	0 },
      { TRXC_CCCH,	0,	TRXC_SACCH4_0,	0 },
      { TRXC_CCCH,	1,	TRXC_SACCH4_0,	1 },
      { TRXC_CCCH,	2,	TRXC_SACCH4_0,	2 },
      { TRXC_CCCH,	3,	TRXC_SACCH4_0,	3 },
      {	TRXC_FCCH,	0,	TRXC_SACCH4_1,	0 },
      {	TRXC_SCH,	0,	TRXC_SACCH4_1,	1 },
      { TRXC_CCCH,	0,	TRXC_SACCH4_1,	2 },
      { TRXC_CCCH,	1,	TRXC_SACCH4_1,	3 },
      { TRXC_CCCH,	2,	TRXC_RACH,	0 },
      { TRXC_CCCH,	3,	TRXC_RACH,	0 },
      { TRXC_CCCH,	0,	TRXC_RACH,	0 },
      { TRXC_CCCH,	1,	TRXC_RACH,	0 },
      { TRXC_CCCH,	2,	TRXC_RACH,	0 },
      { TRXC_CCCH,	3,	TRXC_RACH,	0 },
      {	TRXC_FCCH,	0,	TRXC_RACH,	0 },
      {	TRXC_SCH,	0,	TRXC_RACH,	0 },
      { TRXC_SDCCH4_0,	0,	TRXC_RACH,	0 },
      { TRXC_SDCCH4_0,	1,	TRXC_RACH,	0 },
      { TRXC_SDCCH4_0,	2,	TRXC_RACH,	0 },
      { TRXC_SDCCH4_0,	3,	TRXC_RACH,	0 },
      { TRXC_SDCCH4_1,	0,	TRXC_RACH,	0 },
      { TRXC_SDCCH4_1,	1,	TRXC_RACH,	0 },
      { TRXC_SDCCH4_1,	2,	TRXC_RACH,	0 },
      { TRXC_SDCCH4_1,	3,	TRXC_RACH,	0 },
      {	TRXC_FCCH,	0,	TRXC_RACH,	0 },
      {	TRXC_SCH,	0,	TRXC_RACH,	0 },
      { TRXC_SDCCH4_2,	0,	TRXC_RACH,	0 },
      { TRXC_SDCCH4_2,	1,	TRXC_RACH,	0 },
      { TRXC_SDCCH4_2,	2,	TRXC_RACH,	0 },
      { TRXC_SDCCH4_2,	3,	TRXC_RACH,	0 },
      { TRXC_SDCCH4_3,	0,	TRXC_RACH,	0 },
      { TRXC_SDCCH4_3,	1,	TRXC_SDCCH4_0,	0 },
      { TRXC_SDCCH4_3,	2,	TRXC_SDCCH4_0,	1 },
      { TRXC_SDCCH4_3,	3,	TRXC_SDCCH4_0,	2 },
      {	TRXC_FCCH,	0,	TRXC_SDCCH4_0,	3 },
      {	TRXC_SCH,	0,	TRXC_SDCCH4_1,	0 },
      { TRXC_SACCH4_2,	0,	TRXC_SDCCH4_1,	1 },
      { TRXC_SACCH4_2,	1,	TRXC_SDCCH4_1,	2 },
      { TRXC_SACCH4_2,	2,	TRXC_SDCCH4_1,	3 },
      { TRXC_SACCH4_2,	3,	TRXC_RACH,	0 },
      { TRXC_SACCH4_3,	0,	TRXC_RACH,	0 },
      { TRXC_SACCH4_3,	1,	TRXC_SDCCH4_2,	0 },
      { TRXC_SACCH4_3,	2,	TRXC_SDCCH4_2,	1 },
      { TRXC_SACCH4_3,	3,	TRXC_SDCCH4_2,	2 },
      {	TRXC_IDLE,	0,	TRXC_SDCCH4_2,	3 },
};

static const struct trx_sched_frame frame_sdcch8[102] = {
/*	dl_chan		dl_bid	ul_chan		ul_bid */
      { TRXC_SDCCH8_0,	0,	TRXC_SACCH8_5,	0 },
      { TRXC_SDCCH8_0,	1,	TRXC_SACCH8_5,	1 },
      { TRXC_SDCCH8_0,	2,	TRXC_SACCH8_5,	2 },
      { TRXC_SDCCH8_0,	3,	TRXC_SACCH8_5,	3 },
      { TRXC_SDCCH8_1,	0,	TRXC_SACCH8_6,	0 },
      { TRXC_SDCCH8_1,	1,	TRXC_SACCH8_6,	1 },
      { TRXC_SDCCH8_1,	2,	TRXC_SACCH8_6,	2 },
      { TRXC_SDCCH8_1,	3,	TRXC_SACCH8_6,	3 },
      { TRXC_SDCCH8_2,	0,	TRXC_SACCH8_7,	0 },
      { TRXC_SDCCH8_2,	1,	TRXC_SACCH8_7,	1 },
      { TRXC_SDCCH8_2,	2,	TRXC_SACCH8_7,	2 },
      { TRXC_SDCCH8_2,	3,	TRXC_SACCH8_7,	3 },
      { TRXC_SDCCH8_3,	0,	TRXC_IDLE,	0 },
      { TRXC_SDCCH8_3,	1,	TRXC_IDLE,	0 },
      { TRXC_SDCCH8_3,	2,	TRXC_IDLE,	0 },
      { TRXC_SDCCH8_3,	3,	TRXC_SDCCH8_0,	0 },
      { TRXC_SDCCH8_4,	0,	TRXC_SDCCH8_0,	1 },
      { TRXC_SDCCH8_4,	1,	TRXC_SDCCH8_0,	2 },
      { TRXC_SDCCH8_4,	2,	TRXC_SDCCH8_0,	3 },
      { TRXC_SDCCH8_4,	3,	TRXC_SDCCH8_1,	0 },
      { TRXC_SDCCH8_5,	0,	TRXC_SDCCH8_1,	1 },
      { TRXC_SDCCH8_5,	1,	TRXC_SDCCH8_1,	2 },
      { TRXC_SDCCH8_5,	2,	TRXC_SDCCH8_1,	3 },
      { TRXC_SDCCH8_5,	3,	TRXC_SDCCH8_2,	0 },
      { TRXC_SDCCH8_6,	0,	TRXC_SDCCH8_2,	1 },
      { TRXC_SDCCH8_6,	1,	TRXC_SDCCH8_2,	2 },
      { TRXC_SDCCH8_6,	2,	TRXC_SDCCH8_2,	3 },
      { TRXC_SDCCH8_6,	3,	TRXC_SDCCH8_3,	0 },
      { TRXC_SDCCH8_7,	0,	TRXC_SDCCH8_3,	1 },
      { TRXC_SDCCH8_7,	1,	TRXC_SDCCH8_3,	2 },
      { TRXC_SDCCH8_7,	2,	TRXC_SDCCH8_3,	3 },
      { TRXC_SDCCH8_7,	3,	TRXC_SDCCH8_4,	0 },
      { TRXC_SACCH8_0,	0,	TRXC_SDCCH8_4,	1 },
      { TRXC_SACCH8_0,	1,	TRXC_SDCCH8_4,	2 },
      { TRXC_SACCH8_0,	2,	TRXC_SDCCH8_4,	3 },
      { TRXC_SACCH8_0,	3,	TRXC_SDCCH8_5,	0 },
      { TRXC_SACCH8_1,	0,	TRXC_SDCCH8_5,	1 },
      { TRXC_SACCH8_1,	1,	TRXC_SDCCH8_5,	2 },
      { TRXC_SACCH8_1,	2,	TRXC_SDCCH8_5,	3 },
      { TRXC_SACCH8_1,	3,	TRXC_SDCCH8_6,	0 },
      { TRXC_SACCH8_2,	0,	TRXC_SDCCH8_6,	1 },
      { TRXC_SACCH8_2,	1,	TRXC_SDCCH8_6,	2 },
      { TRXC_SACCH8_2,	2,	TRXC_SDCCH8_6,	3 },
      { TRXC_SACCH8_2,	3,	TRXC_SDCCH8_7,	0 },
      { TRXC_SACCH8_3,	0,	TRXC_SDCCH8_7,	1 },
      { TRXC_SACCH8_3,	1,	TRXC_SDCCH8_7,	2 },
      { TRXC_SACCH8_3,	2,	TRXC_SDCCH8_7,	3 },
      { TRXC_SACCH8_3,	3,	TRXC_SACCH8_0,	0 },
      { TRXC_IDLE,	0,	TRXC_SACCH8_0,	1 },
      { TRXC_IDLE,	0,	TRXC_SACCH8_0,	2 },
      { TRXC_IDLE,	0,	TRXC_SACCH8_0,	3 },

      { TRXC_SDCCH8_0,	0,	TRXC_SACCH8_1,	0 },
      { TRXC_SDCCH8_0,	1,	TRXC_SACCH8_1,	1 },
      { TRXC_SDCCH8_0,	2,	TRXC_SACCH8_1,	2 },
      { TRXC_SDCCH8_0,	3,	TRXC_SACCH8_1,	3 },
      { TRXC_SDCCH8_1,	0,	TRXC_SACCH8_2,	0 },
      { TRXC_SDCCH8_1,	1,	TRXC_SACCH8_2,	1 },
      { TRXC_SDCCH8_1,	2,	TRXC_SACCH8_2,	2 },
      { TRXC_SDCCH8_1,	3,	TRXC_SACCH8_2,	3 },
      { TRXC_SDCCH8_2,	0,	TRXC_SACCH8_3,	0 },
      { TRXC_SDCCH8_2,	1,	TRXC_SACCH8_3,	1 },
      { TRXC_SDCCH8_2,	2,	TRXC_SACCH8_3,	2 },
      { TRXC_SDCCH8_2,	3,	TRXC_SACCH8_3,	3 },
      { TRXC_SDCCH8_3,	0,	TRXC_IDLE,	0 },
      { TRXC_SDCCH8_3,	1,	TRXC_IDLE,	0 },
      { TRXC_SDCCH8_3,	2,	TRXC_IDLE,	0 },
      { TRXC_SDCCH8_3,	3,	TRXC_SDCCH8_0,	0 },
      { TRXC_SDCCH8_4,	0,	TRXC_SDCCH8_0,	1 },
      { TRXC_SDCCH8_4,	1,	TRXC_SDCCH8_0,	2 },
      { TRXC_SDCCH8_4,	2,	TRXC_SDCCH8_0,	3 },
      { TRXC_SDCCH8_4,	3,	TRXC_SDCCH8_1,	0 },
      { TRXC_SDCCH8_5,	0,	TRXC_SDCCH8_1,	1 },
      { TRXC_SDCCH8_5,	1,	TRXC_SDCCH8_1,	2 },
      { TRXC_SDCCH8_5,	2,	TRXC_SDCCH8_1,	3 },
      { TRXC_SDCCH8_5,	3,	TRXC_SDCCH8_2,	0 },
      { TRXC_SDCCH8_6,	0,	TRXC_SDCCH8_2,	1 },
      { TRXC_SDCCH8_6,	1,	TRXC_SDCCH8_2,	2 },
      { TRXC_SDCCH8_6,	2,	TRXC_SDCCH8_2,	3 },
      { TRXC_SDCCH8_6,	3,	TRXC_SDCCH8_3,	0 },
      { TRXC_SDCCH8_7,	0,	TRXC_SDCCH8_3,	1 },
      { TRXC_SDCCH8_7,	1,	TRXC_SDCCH8_3,	2 },
      { TRXC_SDCCH8_7,	2,	TRXC_SDCCH8_3,	3 },
      { TRXC_SDCCH8_7,	3,	TRXC_SDCCH8_4,	0 },
      { TRXC_SACCH8_4,	0,	TRXC_SDCCH8_4,	1 },
      { TRXC_SACCH8_4,	1,	TRXC_SDCCH8_4,	2 },
      { TRXC_SACCH8_4,	2,	TRXC_SDCCH8_4,	3 },
      { TRXC_SACCH8_4,	3,	TRXC_SDCCH8_5,	0 },
      { TRXC_SACCH8_5,	0,	TRXC_SDCCH8_5,	1 },
      { TRXC_SACCH8_5,	1,	TRXC_SDCCH8_5,	2 },
      { TRXC_SACCH8_5,	2,	TRXC_SDCCH8_5,	3 },
      { TRXC_SACCH8_5,	3,	TRXC_SDCCH8_6,	0 },
      { TRXC_SACCH8_6,	0,	TRXC_SDCCH8_6,	1 },
      { TRXC_SACCH8_6,	1,	TRXC_SDCCH8_6,	2 },
      { TRXC_SACCH8_6,	2,	TRXC_SDCCH8_6,	3 },
      { TRXC_SACCH8_6,	3,	TRXC_SDCCH8_7,	0 },
      { TRXC_SACCH8_7,	0,	TRXC_SDCCH8_7,	1 },
      { TRXC_SACCH8_7,	1,	TRXC_SDCCH8_7,	2 },
      { TRXC_SACCH8_7,	2,	TRXC_SDCCH8_7,	3 },
      { TRXC_SACCH8_7,	3,	TRXC_SACCH8_4,	0 },
      { TRXC_IDLE,	0,	TRXC_SACCH8_4,	1 },
      { TRXC_IDLE,	0,	TRXC_SACCH8_4,	2 },
      { TRXC_IDLE,	0,	TRXC_SACCH8_4,	3 },
};

static const struct trx_sched_frame frame_tchf_ts0[104] = {
/*	dl_chan		dl_bid	ul_chan		ul_bid */
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_SACCHTF,	0,	TRXC_SACCHTF,	0 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_IDLE,	0,	TRXC_IDLE,	0 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_SACCHTF,	1,	TRXC_SACCHTF,	1 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_IDLE,	0,	TRXC_IDLE,	0 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_SACCHTF,	2,	TRXC_SACCHTF,	2 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_IDLE,	0,	TRXC_IDLE,	0 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_SACCHTF,	3,	TRXC_SACCHTF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_IDLE,	0,	TRXC_IDLE,	0 },
};

static const struct trx_sched_frame frame_tchf_ts1[104] = {
/*	dl_chan		dl_bid	ul_chan		ul_bid */
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_IDLE,	0,	TRXC_IDLE,	0 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_SACCHTF,	0,	TRXC_SACCHTF,	0 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_IDLE,	0,	TRXC_IDLE,	0 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_SACCHTF,	1,	TRXC_SACCHTF,	1 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_IDLE,	0,	TRXC_IDLE,	0 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_SACCHTF,	2,	TRXC_SACCHTF,	2 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_IDLE,	0,	TRXC_IDLE,	0 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_SACCHTF,	3,	TRXC_SACCHTF,	3 },
};

static const struct trx_sched_frame frame_tchf_ts2[104] = {
/*	dl_chan		dl_bid	ul_chan		ul_bid */
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_SACCHTF,	3,	TRXC_SACCHTF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_IDLE,	0,	TRXC_IDLE,	0 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_SACCHTF,	0,	TRXC_SACCHTF,	0 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_IDLE,	0,	TRXC_IDLE,	0 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_SACCHTF,	1,	TRXC_SACCHTF,	1 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_IDLE,	0,	TRXC_IDLE,	0 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_SACCHTF,	2,	TRXC_SACCHTF,	2 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_IDLE,	0,	TRXC_IDLE,	0 },
};

static const struct trx_sched_frame frame_tchf_ts3[104] = {
/*	dl_chan		dl_bid	ul_chan		ul_bid */
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_IDLE,	0,	TRXC_IDLE,	0 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_SACCHTF,	3,	TRXC_SACCHTF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_IDLE,	0,	TRXC_IDLE,	0 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_SACCHTF,	0,	TRXC_SACCHTF,	0 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_IDLE,	0,	TRXC_IDLE,	0 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_SACCHTF,	1,	TRXC_SACCHTF,	1 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_IDLE,	0,	TRXC_IDLE,	0 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_SACCHTF,	2,	TRXC_SACCHTF,	2 },
};

static const struct trx_sched_frame frame_tchf_ts4[104] = {
/*	dl_chan		dl_bid	ul_chan		ul_bid */
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_SACCHTF,	2,	TRXC_SACCHTF,	2 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_IDLE,	0,	TRXC_IDLE,	0 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_SACCHTF,	3,	TRXC_SACCHTF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_IDLE,	0,	TRXC_IDLE,	0 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_SACCHTF,	0,	TRXC_SACCHTF,	0 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_IDLE,	0,	TRXC_IDLE,	0 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_SACCHTF,	1,	TRXC_SACCHTF,	1 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_IDLE,	0,	TRXC_IDLE,	0 },
};

static const struct trx_sched_frame frame_tchf_ts5[104] = {
/*	dl_chan		dl_bid	ul_chan		ul_bid */
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_IDLE,	0,	TRXC_IDLE,	0 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_SACCHTF,	2,	TRXC_SACCHTF,	2 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_IDLE,	0,	TRXC_IDLE,	0 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_SACCHTF,	3,	TRXC_SACCHTF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_IDLE,	0,	TRXC_IDLE,	0 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_SACCHTF,	0,	TRXC_SACCHTF,	0 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_IDLE,	0,	TRXC_IDLE,	0 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_SACCHTF,	1,	TRXC_SACCHTF,	1 },
};

static const struct trx_sched_frame frame_tchf_ts6[104] = {
/*	dl_chan		dl_bid	ul_chan		ul_bid */
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_SACCHTF,	1,	TRXC_SACCHTF,	1 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_IDLE,	0,	TRXC_IDLE,	0 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_SACCHTF,	2,	TRXC_SACCHTF,	2 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_IDLE,	0,	TRXC_IDLE,	0 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_SACCHTF,	3,	TRXC_SACCHTF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_IDLE,	0,	TRXC_IDLE,	0 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_SACCHTF,	0,	TRXC_SACCHTF,	0 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_IDLE,	0,	TRXC_IDLE,	0 },
};

static const struct trx_sched_frame frame_tchf_ts7[104] = {
/*	dl_chan		dl_bid	ul_chan		ul_bid */
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_IDLE,	0,	TRXC_IDLE,	0 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_SACCHTF,	1,	TRXC_SACCHTF,	1 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_IDLE,	0,	TRXC_IDLE,	0 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_SACCHTF,	2,	TRXC_SACCHTF,	2 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_IDLE,	0,	TRXC_IDLE,	0 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_SACCHTF,	3,	TRXC_SACCHTF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_IDLE,	0,	TRXC_IDLE,	0 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_TCHF,	0,	TRXC_TCHF,	0 }, { TRXC_TCHF,	1,	TRXC_TCHF,	1 }, { TRXC_TCHF,	2,	TRXC_TCHF,	2 }, { TRXC_TCHF,	3,	TRXC_TCHF,	3 },
      { TRXC_SACCHTF,	0,	TRXC_SACCHTF,	0 },
};

static const struct trx_sched_frame frame_tchh_ts01[104] = {
/*	dl_chan		dl_bid	ul_chan		ul_bid */
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_SACCHTH_0,	0,	TRXC_SACCHTH_0,	0 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_SACCHTH_1,	0,	TRXC_SACCHTH_1,	0 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_SACCHTH_0,	1,	TRXC_SACCHTH_0,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_SACCHTH_1,	1,	TRXC_SACCHTH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_SACCHTH_0,	2,	TRXC_SACCHTH_0,	2 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_SACCHTH_1,	2,	TRXC_SACCHTH_1,	2 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_SACCHTH_0,	3,	TRXC_SACCHTH_0,	3 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_SACCHTH_1,	3,	TRXC_SACCHTH_1,	3 },
};

static const struct trx_sched_frame frame_tchh_ts23[104] = {
/*	dl_chan		dl_bid	ul_chan		ul_bid */
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_SACCHTH_0,	3,	TRXC_SACCHTH_0,	3 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_SACCHTH_1,	3,	TRXC_SACCHTH_1,	3 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_SACCHTH_0,	0,	TRXC_SACCHTH_0,	0 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_SACCHTH_1,	0,	TRXC_SACCHTH_1,	0 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_SACCHTH_0,	1,	TRXC_SACCHTH_0,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_SACCHTH_1,	1,	TRXC_SACCHTH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_SACCHTH_0,	2,	TRXC_SACCHTH_0,	2 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_SACCHTH_1,	2,	TRXC_SACCHTH_1,	2 },
};

static const struct trx_sched_frame frame_tchh_ts45[104] = {
/*	dl_chan		dl_bid	ul_chan		ul_bid */
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_SACCHTH_0,	2,	TRXC_SACCHTH_0,	2 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_SACCHTH_1,	2,	TRXC_SACCHTH_1,	2 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_SACCHTH_0,	3,	TRXC_SACCHTH_0,	3 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_SACCHTH_1,	3,	TRXC_SACCHTH_1,	3 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_SACCHTH_0,	0,	TRXC_SACCHTH_0,	0 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_SACCHTH_1,	0,	TRXC_SACCHTH_1,	0 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_SACCHTH_0,	1,	TRXC_SACCHTH_0,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_SACCHTH_1,	1,	TRXC_SACCHTH_1,	1 },
};

static const struct trx_sched_frame frame_tchh_ts67[104] = {
/*	dl_chan		dl_bid	ul_chan		ul_bid */
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_SACCHTH_0,	1,	TRXC_SACCHTH_0,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_SACCHTH_1,	1,	TRXC_SACCHTH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_SACCHTH_0,	2,	TRXC_SACCHTH_0,	2 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_SACCHTH_1,	2,	TRXC_SACCHTH_1,	2 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_SACCHTH_0,	3,	TRXC_SACCHTH_0,	3 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_SACCHTH_1,	3,	TRXC_SACCHTH_1,	3 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_SACCHTH_0,	0,	TRXC_SACCHTH_0,	0 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_TCHH_0,	0,	TRXC_TCHH_0,	0 }, { TRXC_TCHH_1,	0,	TRXC_TCHH_1,	0 }, { TRXC_TCHH_0,	1,	TRXC_TCHH_0,	1 }, { TRXC_TCHH_1,	1,	TRXC_TCHH_1,	1 },
      { TRXC_SACCHTH_1,	0,	TRXC_SACCHTH_1,	0 },
};

static const struct trx_sched_frame frame_pdch[104] = {
/*	dl_chan		dl_bid	ul_chan		ul_bid */
      { TRXC_PDTCH,	0,	TRXC_PDTCH,	0 }, { TRXC_PDTCH,	1,	TRXC_PDTCH,	1 }, { TRXC_PDTCH,	2,	TRXC_PDTCH,	2 }, { TRXC_PDTCH,	3,	TRXC_PDTCH,	3 },
      { TRXC_PDTCH,	0,	TRXC_PDTCH,	0 }, { TRXC_PDTCH,	1,	TRXC_PDTCH,	1 }, { TRXC_PDTCH,	2,	TRXC_PDTCH,	2 }, { TRXC_PDTCH,	3,	TRXC_PDTCH,	3 },
      { TRXC_PDTCH,	0,	TRXC_PDTCH,	0 }, { TRXC_PDTCH,	1,	TRXC_PDTCH,	1 }, { TRXC_PDTCH,	2,	TRXC_PDTCH,	2 }, { TRXC_PDTCH,	3,	TRXC_PDTCH,	3 },
      { TRXC_PTCCH,	0,	TRXC_PTCCH,	0 },
      { TRXC_PDTCH,	0,	TRXC_PDTCH,	0 }, { TRXC_PDTCH,	1,	TRXC_PDTCH,	1 }, { TRXC_PDTCH,	2,	TRXC_PDTCH,	2 }, { TRXC_PDTCH,	3,	TRXC_PDTCH,	3 },
      { TRXC_PDTCH,	0,	TRXC_PDTCH,	0 }, { TRXC_PDTCH,	1,	TRXC_PDTCH,	1 }, { TRXC_PDTCH,	2,	TRXC_PDTCH,	2 }, { TRXC_PDTCH,	3,	TRXC_PDTCH,	3 },
      { TRXC_PDTCH,	0,	TRXC_PDTCH,	0 }, { TRXC_PDTCH,	1,	TRXC_PDTCH,	1 }, { TRXC_PDTCH,	2,	TRXC_PDTCH,	2 }, { TRXC_PDTCH,	3,	TRXC_PDTCH,	3 },
      { TRXC_IDLE,	0,	TRXC_IDLE,	0 },
      { TRXC_PDTCH,	0,	TRXC_PDTCH,	0 }, { TRXC_PDTCH,	1,	TRXC_PDTCH,	1 }, { TRXC_PDTCH,	2,	TRXC_PDTCH,	2 }, { TRXC_PDTCH,	3,	TRXC_PDTCH,	3 },
      { TRXC_PDTCH,	0,	TRXC_PDTCH,	0 }, { TRXC_PDTCH,	1,	TRXC_PDTCH,	1 }, { TRXC_PDTCH,	2,	TRXC_PDTCH,	2 }, { TRXC_PDTCH,	3,	TRXC_PDTCH,	3 },
      { TRXC_PDTCH,	0,	TRXC_PDTCH,	0 }, { TRXC_PDTCH,	1,	TRXC_PDTCH,	1 }, { TRXC_PDTCH,	2,	TRXC_PDTCH,	2 }, { TRXC_PDTCH,	3,	TRXC_PDTCH,	3 },
      { TRXC_PTCCH,	1,	TRXC_PTCCH,	1 },
      { TRXC_PDTCH,	0,	TRXC_PDTCH,	0 }, { TRXC_PDTCH,	1,	TRXC_PDTCH,	1 }, { TRXC_PDTCH,	2,	TRXC_PDTCH,	2 }, { TRXC_PDTCH,	3,	TRXC_PDTCH,	3 },
      { TRXC_PDTCH,	0,	TRXC_PDTCH,	0 }, { TRXC_PDTCH,	1,	TRXC_PDTCH,	1 }, { TRXC_PDTCH,	2,	TRXC_PDTCH,	2 }, { TRXC_PDTCH,	3,	TRXC_PDTCH,	3 },
      { TRXC_PDTCH,	0,	TRXC_PDTCH,	0 }, { TRXC_PDTCH,	1,	TRXC_PDTCH,	1 }, { TRXC_PDTCH,	2,	TRXC_PDTCH,	2 }, { TRXC_PDTCH,	3,	TRXC_PDTCH,	3 },
      { TRXC_IDLE,	0,	TRXC_IDLE,	0 },
      { TRXC_PDTCH,	0,	TRXC_PDTCH,	0 }, { TRXC_PDTCH,	1,	TRXC_PDTCH,	1 }, { TRXC_PDTCH,	2,	TRXC_PDTCH,	2 }, { TRXC_PDTCH,	3,	TRXC_PDTCH,	3 },
      { TRXC_PDTCH,	0,	TRXC_PDTCH,	0 }, { TRXC_PDTCH,	1,	TRXC_PDTCH,	1 }, { TRXC_PDTCH,	2,	TRXC_PDTCH,	2 }, { TRXC_PDTCH,	3,	TRXC_PDTCH,	3 },
      { TRXC_PDTCH,	0,	TRXC_PDTCH,	0 }, { TRXC_PDTCH,	1,	TRXC_PDTCH,	1 }, { TRXC_PDTCH,	2,	TRXC_PDTCH,	2 }, { TRXC_PDTCH,	3,	TRXC_PDTCH,	3 },
      { TRXC_PTCCH,	2,	TRXC_PTCCH,	2 },
      { TRXC_PDTCH,	0,	TRXC_PDTCH,	0 }, { TRXC_PDTCH,	1,	TRXC_PDTCH,	1 }, { TRXC_PDTCH,	2,	TRXC_PDTCH,	2 }, { TRXC_PDTCH,	3,	TRXC_PDTCH,	3 },
      { TRXC_PDTCH,	0,	TRXC_PDTCH,	0 }, { TRXC_PDTCH,	1,	TRXC_PDTCH,	1 }, { TRXC_PDTCH,	2,	TRXC_PDTCH,	2 }, { TRXC_PDTCH,	3,	TRXC_PDTCH,	3 },
      { TRXC_PDTCH,	0,	TRXC_PDTCH,	0 }, { TRXC_PDTCH,	1,	TRXC_PDTCH,	1 }, { TRXC_PDTCH,	2,	TRXC_PDTCH,	2 }, { TRXC_PDTCH,	3,	TRXC_PDTCH,	3 },
      { TRXC_IDLE,	0,	TRXC_IDLE,	0 },
      { TRXC_PDTCH,	0,	TRXC_PDTCH,	0 }, { TRXC_PDTCH,	1,	TRXC_PDTCH,	1 }, { TRXC_PDTCH,	2,	TRXC_PDTCH,	2 }, { TRXC_PDTCH,	3,	TRXC_PDTCH,	3 },
      { TRXC_PDTCH,	0,	TRXC_PDTCH,	0 }, { TRXC_PDTCH,	1,	TRXC_PDTCH,	1 }, { TRXC_PDTCH,	2,	TRXC_PDTCH,	2 }, { TRXC_PDTCH,	3,	TRXC_PDTCH,	3 },
      { TRXC_PDTCH,	0,	TRXC_PDTCH,	0 }, { TRXC_PDTCH,	1,	TRXC_PDTCH,	1 }, { TRXC_PDTCH,	2,	TRXC_PDTCH,	2 }, { TRXC_PDTCH,	3,	TRXC_PDTCH,	3 },
      { TRXC_PTCCH,	3,	TRXC_PTCCH,	3 },
      { TRXC_PDTCH,	0,	TRXC_PDTCH,	0 }, { TRXC_PDTCH,	1,	TRXC_PDTCH,	1 }, { TRXC_PDTCH,	2,	TRXC_PDTCH,	2 }, { TRXC_PDTCH,	3,	TRXC_PDTCH,	3 },
      { TRXC_PDTCH,	0,	TRXC_PDTCH,	0 }, { TRXC_PDTCH,	1,	TRXC_PDTCH,	1 }, { TRXC_PDTCH,	2,	TRXC_PDTCH,	2 }, { TRXC_PDTCH,	3,	TRXC_PDTCH,	3 },
      { TRXC_PDTCH,	0,	TRXC_PDTCH,	0 }, { TRXC_PDTCH,	1,	TRXC_PDTCH,	1 }, { TRXC_PDTCH,	2,	TRXC_PDTCH,	2 }, { TRXC_PDTCH,	3,	TRXC_PDTCH,	3 },
      { TRXC_IDLE,	0,	TRXC_IDLE,	0 },
};

/* multiframe structure */
struct trx_sched_multiframe {
	/*! \brief physical channel config (channel combination) */
	enum gsm_phys_chan_config	pchan;
	/*! \brief applies to which timeslots? */
	uint8_t				slotmask;
	/*! \brief repeats how many frames */
	uint8_t				period;
	/*! \brief pointer to scheduling structure */
	const struct trx_sched_frame	*frames;
	/*! \brife human-readable name */
	const char 			*name;
};

static const struct trx_sched_multiframe trx_sched_multiframes[] = {
	{ GSM_PCHAN_NONE,		0xff,	0,	NULL,			"NONE"},
	{ GSM_PCHAN_CCCH,		0xff,	51,	frame_bcch,		"BCCH+CCCH" },
	{ GSM_PCHAN_CCCH_SDCCH4,	0xff,	102,	frame_bcch_sdcch4,	"BCCH+CCCH+SDCCH/4+SACCH/4" },
	{ GSM_PCHAN_SDCCH8_SACCH8C,	0xff,	102,	frame_sdcch8,		"SDCCH/8+SACCH/8" },
	{ GSM_PCHAN_TCH_F,		0x01,	104,	frame_tchf_ts0,		"TCH/F+SACCH" },
	{ GSM_PCHAN_TCH_F,		0x02,	104,	frame_tchf_ts1,		"TCH/F+SACCH" },
	{ GSM_PCHAN_TCH_F,		0x04,	104,	frame_tchf_ts2,		"TCH/F+SACCH" },
	{ GSM_PCHAN_TCH_F,		0x08,	104,	frame_tchf_ts3,		"TCH/F+SACCH" },
	{ GSM_PCHAN_TCH_F,		0x10,	104,	frame_tchf_ts4,		"TCH/F+SACCH" },
	{ GSM_PCHAN_TCH_F,		0x20,	104,	frame_tchf_ts5,		"TCH/F+SACCH" },
	{ GSM_PCHAN_TCH_F,		0x40,	104,	frame_tchf_ts6,		"TCH/F+SACCH" },
	{ GSM_PCHAN_TCH_F,		0x80,	104,	frame_tchf_ts7,		"TCH/F+SACCH" },
	{ GSM_PCHAN_TCH_H,		0x03,	104,	frame_tchh_ts01,	"TCH/H+SACCH" },
	{ GSM_PCHAN_TCH_H,		0x0c,	104,	frame_tchh_ts23,	"TCH/H+SACCH" },
	{ GSM_PCHAN_TCH_H,		0x30,	104,	frame_tchh_ts45,	"TCH/H+SACCH" },
	{ GSM_PCHAN_TCH_H,		0xc0,	104,	frame_tchh_ts67,	"TCH/H+SACCH" },
	{ GSM_PCHAN_PDCH,		0xff,	104,	frame_pdch,		"PDCH" },
};


/*
 * scheduler functions
 */

/* set multiframe scheduler to given pchan */
int trx_sched_set_pchan(struct l1sched_trx *l1t, uint8_t tn,
	enum gsm_phys_chan_config pchan)
{
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);
	int i;

	for (i = 0; i < ARRAY_SIZE(trx_sched_multiframes); i++) {
		if (trx_sched_multiframes[i].pchan == pchan
		 && (trx_sched_multiframes[i].slotmask & (1 << tn))) {
			l1ts->mf_index = i;
			l1ts->mf_period = trx_sched_multiframes[i].period;
			l1ts->mf_frames = trx_sched_multiframes[i].frames;
			LOGP(DL1C, LOGL_NOTICE, "Configuring multiframe with "
				"%s trx=%d ts=%d\n",
				trx_sched_multiframes[i].name,
				l1t->trx->nr, tn);
			return 0;
		}
	}

	LOGP(DL1C, LOGL_NOTICE, "Failed to configuring multiframe "
		"trx=%d ts=%d\n", l1t->trx->nr, tn);

	return -ENOTSUP;
}

/* setting all logical channels given attributes to active/inactive */
int trx_sched_set_lchan(struct l1sched_trx *l1t, uint8_t chan_nr, uint8_t link_id,
	int active)
{
	uint8_t tn = L1SAP_CHAN2TS(chan_nr);
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);
	uint8_t ss = l1sap_chan2ss(chan_nr);
	int i;
	int rc = -EINVAL;

	/* look for all matching chan_nr/link_id */
	for (i = 0; i < _TRX_CHAN_MAX; i++) {
		struct l1sched_chan_state *chan_state;
		chan_state = &l1ts->chan_state[i];
		/* skip if pchan type does not match pdch flag */
		if ((trx_sched_multiframes[l1ts->mf_index].pchan
							== GSM_PCHAN_PDCH)
						!= trx_chan_desc[i].pdch)
			continue;
		if (trx_chan_desc[i].chan_nr == (chan_nr & 0xf8)
		 && trx_chan_desc[i].link_id == link_id) {
			rc = 0;
			if (chan_state->active == active)
				continue;
			LOGP(DL1C, LOGL_NOTICE, "%s %s on trx=%d ts=%d\n",
				(active) ? "Activating" : "Deactivating",
				trx_chan_desc[i].name, l1t->trx->nr, tn);
			if (active)
				memset(chan_state, 0, sizeof(*chan_state));
			chan_state->active = active;
			/* free burst memory, to cleanly start with burst 0 */
			if (chan_state->dl_bursts) {
				talloc_free(chan_state->dl_bursts);
				chan_state->dl_bursts = NULL;
			}
			if (chan_state->ul_bursts) {
				talloc_free(chan_state->ul_bursts);
				chan_state->ul_bursts = NULL;
			}
			if (!active)
				chan_state->ho_rach_detect = 0;
		}
	}

	/* disable handover detection (on deactivation) */
	if (!active) {
		struct trx_l1h *l1h = trx_l1h_hdl(l1t->trx);
		trx_if_cmd_nohandover(l1h, tn, ss);
	}

	return rc;
}

/* setting all logical channels given attributes to active/inactive */
int trx_sched_set_mode(struct l1sched_trx *l1t, uint8_t chan_nr, uint8_t rsl_cmode,
	uint8_t tch_mode, int codecs, uint8_t codec0, uint8_t codec1,
	uint8_t codec2, uint8_t codec3, uint8_t initial_id, uint8_t handover)
{
	uint8_t tn = L1SAP_CHAN2TS(chan_nr);
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);
	struct trx_l1h *l1h = trx_l1h_hdl(l1t->trx);
	uint8_t ss = l1sap_chan2ss(chan_nr);
	int i;
	int rc = -EINVAL;
	struct l1sched_chan_state *chan_state;

	/* no mode for PDCH */
	if (trx_sched_multiframes[l1ts->mf_index].pchan == GSM_PCHAN_PDCH)
		return 0;

	/* look for all matching chan_nr/link_id */
	for (i = 0; i < _TRX_CHAN_MAX; i++) {
		if (trx_chan_desc[i].chan_nr == (chan_nr & 0xf8)
		 && trx_chan_desc[i].link_id == 0x00) {
			chan_state = &l1ts->chan_state[i];
			LOGP(DL1C, LOGL_NOTICE, "Set mode %u, %u, handover %u "
				"on %s of trx=%d ts=%d\n", rsl_cmode, tch_mode,
				handover, trx_chan_desc[i].name, l1t->trx->nr,
				tn);
			chan_state->rsl_cmode = rsl_cmode;
			chan_state->tch_mode = tch_mode;
			chan_state->ho_rach_detect = handover;
			if (rsl_cmode == RSL_CMOD_SPD_SPEECH
			 && tch_mode == GSM48_CMODE_SPEECH_AMR) {
				chan_state->codecs = codecs;
				chan_state->codec[0] = codec0;
				chan_state->codec[1] = codec1;
				chan_state->codec[2] = codec2;
				chan_state->codec[3] = codec3;
				chan_state->ul_ft = initial_id;
				chan_state->dl_ft = initial_id;
				chan_state->ul_cmr = initial_id;
				chan_state->dl_cmr = initial_id;
				chan_state->ber_sum = 0;
				chan_state->ber_num = 0;
			}
			rc = 0;
		}
	}

	/* command rach detection
	 * always enable handover, even if state is still set (due to loss
	 * of transceiver link).
	 * disable handover, if state is still set, since we might not know
	 * the actual state of transceiver (due to loss of link) */
	if (handover) {
		trx_if_cmd_handover(l1h, tn, ss);
	} else {
		trx_if_cmd_nohandover(l1h, tn, ss);
	}

	return rc;
}

/* setting cipher on logical channels */
int trx_sched_set_cipher(struct l1sched_trx *l1t, uint8_t chan_nr, int downlink,
	int algo, uint8_t *key, int key_len)
{
	uint8_t tn = L1SAP_CHAN2TS(chan_nr);
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);
	int i;
	int rc = -EINVAL;
	struct l1sched_chan_state *chan_state;

	/* no cipher for PDCH */
	if (trx_sched_multiframes[l1ts->mf_index].pchan == GSM_PCHAN_PDCH)
		return 0;

	/* no algorithm given means a5/0 */
	if (algo <= 0)
		algo = 0;
	else if (key_len != 8) {
		LOGP(DL1C, LOGL_ERROR, "Algo A5/%d not supported with given "
			"key len=%d\n", algo, key_len);
		return -ENOTSUP;
	}

	/* look for all matching chan_nr */
	for (i = 0; i < _TRX_CHAN_MAX; i++) {
		/* skip if pchan type */
		if (trx_chan_desc[i].pdch)
			continue;
		if (trx_chan_desc[i].chan_nr == (chan_nr & 0xf8)) {
			chan_state = &l1ts->chan_state[i];
			LOGP(DL1C, LOGL_NOTICE, "Set a5/%d %s for %s on trx=%d "
				"ts=%d\n", algo,
				(downlink) ? "downlink" : "uplink",
				trx_chan_desc[i].name, l1t->trx->nr, tn);
			if (downlink) {
				chan_state->dl_encr_algo = algo;
				memcpy(chan_state->dl_encr_key, key, key_len);
				chan_state->dl_encr_key_len = key_len;
			} else {
				chan_state->ul_encr_algo = algo;
				memcpy(chan_state->ul_encr_key, key, key_len);
				chan_state->ul_encr_key_len = key_len;
			}
			rc = 0;
		}
	}

	return rc;
}

/* process ready-to-send */
static int trx_sched_rts(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn)
{
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);
	const struct trx_sched_frame *frame;
	uint8_t offset, period, bid;
	trx_sched_rts_func *func;
	enum trx_chan_type chan;

	/* no multiframe set */
	if (!l1ts->mf_index)
		return 0;

	/* get frame from multiframe */
	period = l1ts->mf_period;
	offset = fn % period;
	frame = l1ts->mf_frames + offset;

	chan = frame->dl_chan;
	bid = frame->dl_bid;
	func = trx_chan_desc[frame->dl_chan].rts_fn;

	/* only on bid == 0 */
	if (bid != 0)
		return 0;

	/* no RTS function */
	if (!func)
		return 0;

	/* check if channel is active */
	if (!trx_chan_desc[chan].auto_active
	 && !l1ts->chan_state[chan].active)
	 	return -EINVAL;

	return func(l1t, tn, fn, frame->dl_chan);
}

/* process downlink burst */
static const ubit_t *trx_sched_dl_burst(struct l1sched_trx *l1t, uint8_t tn,
	uint32_t fn)
{
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);
	struct l1sched_chan_state *l1cs;
	const struct trx_sched_frame *frame;
	uint8_t offset, period, bid;
	trx_sched_dl_func *func;
	enum trx_chan_type chan;
	ubit_t *bits = NULL;

	if (!l1ts->mf_index)
		goto no_data;

	/* get frame from multiframe */
	period = l1ts->mf_period;
	offset = fn % period;
	frame = l1ts->mf_frames + offset;

	chan = frame->dl_chan;
	bid = frame->dl_bid;
	func = trx_chan_desc[chan].dl_fn;

	l1cs = &l1ts->chan_state[chan];

	/* check if channel is active */
	if (!trx_chan_desc[chan].auto_active && !l1cs->active)
	 	goto no_data;

	/* get burst from function */
	bits = func(l1t, tn, fn, chan, bid);

	/* encrypt */
	if (bits && l1cs->dl_encr_algo) {
		ubit_t ks[114];
		int i;

		osmo_a5(l1cs->dl_encr_algo, l1cs->dl_encr_key, fn, ks, NULL);
		for (i = 0; i < 57; i++) {
			bits[i + 3] ^= ks[i];
			bits[i + 88] ^= ks[i + 57];
		}
	}

no_data:
	/* in case of C0, we need a dummy burst to maintain RF power */
	if (bits == NULL && l1t->trx == l1t->trx->bts->c0) {
if (0)		if (chan != TRXC_IDLE) // hack
		LOGP(DL1C, LOGL_DEBUG, "No burst data for %s fn=%u ts=%u "
			"burst=%d on C0, so filling with dummy burst\n",
			trx_chan_desc[chan].name, fn, tn, bid);
		bits = (ubit_t *) dummy_burst;
	}

	return bits;
}

/* process uplink burst */
int trx_sched_ul_burst(struct l1sched_trx *l1t, uint8_t tn, uint32_t current_fn,
	sbit_t *bits, int8_t rssi, float toa)
{
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);
	struct l1sched_chan_state *l1cs;
	const struct trx_sched_frame *frame;
	uint8_t offset, period, bid;
	trx_sched_ul_func *func;
	enum trx_chan_type chan;
	uint32_t fn, elapsed;

	if (!l1ts->mf_index)
		return -EINVAL;

	/* calculate how many frames have been elapsed */
	elapsed = (current_fn + GSM_HYPERFRAME - l1ts->mf_last_fn) % GSM_HYPERFRAME;

	/* start counting from last fn + 1, but only if not too many fn have
	 * been elapsed */
	if (elapsed < 10)
		fn = (l1ts->mf_last_fn + 1) % GSM_HYPERFRAME;
	else
		fn = current_fn;

	while (42) {
		/* get frame from multiframe */
		period = l1ts->mf_period;
		offset = fn % period;
		frame = l1ts->mf_frames + offset;

		chan = frame->ul_chan;
		bid = frame->ul_bid;
		func = trx_chan_desc[chan].ul_fn;

		l1cs = &l1ts->chan_state[chan];

		/* check if channel is active */
		if (!trx_chan_desc[chan].auto_active && !l1cs->active)
			goto next_frame;

		/* omit bursts which have no handler, like IDLE bursts */
		if (!func)
			goto next_frame;

		/* put burst to function */
		if (fn == current_fn) {
			/* decrypt */
			if (bits && l1cs->ul_encr_algo) {
				ubit_t ks[114];
				int i;

				osmo_a5(l1cs->ul_encr_algo,
					l1cs->ul_encr_key,
					fn, NULL, ks);
				for (i = 0; i < 57; i++) {
					if (ks[i])
						bits[i + 3] = - bits[i + 3];
					if (ks[i + 57])
						bits[i + 88] = - bits[i + 88];
				}
			}

			func(l1t, tn, fn, chan, bid, bits, rssi, toa);
		} else if (chan != TRXC_RACH && !l1cs->ho_rach_detect) {
			sbit_t spare[148];

			memset(spare, 0, 148);
			func(l1t, tn, fn, chan, bid, spare, -128, 0);
		}

next_frame:
		/* reached current fn */
		if (fn == current_fn)
			break;

		fn = (fn + 1) % GSM_HYPERFRAME;
	}

	l1ts->mf_last_fn = fn;

	return 0;
}

/* schedule all frames of all TRX for given FN */
static int trx_sched_fn(uint32_t fn)
{
	struct gsm_bts_trx *trx;
	uint8_t tn;
	const ubit_t *bits;
	uint8_t gain;

	/* send time indication */
	l1if_mph_time_ind(bts, fn);

	/* advance frame number, so the transceiver has more time until
	 * it must be transmitted. */
	fn = (fn + trx_clock_advance) % GSM_HYPERFRAME;

	/* process every TRX */
	llist_for_each_entry(trx, &bts->trx_list, list) {
		struct trx_l1h *l1h = trx_l1h_hdl(trx);
		struct l1sched_trx *l1t = trx_l1sched_hdl(trx);

		/* we don't schedule, if power is off */
		if (!l1h->config.poweron)
			continue;

		/* process every TS of TRX */
		for (tn = 0; tn < ARRAY_SIZE(l1t->ts); tn++) {
			/* ready-to-send */
			trx_sched_rts(l1t, tn,
				(fn + trx_rts_advance) % GSM_HYPERFRAME);
			/* get burst for FN */
			bits = trx_sched_dl_burst(l1t, tn, fn);
			if (!bits) {
				/* if no bits, send no burst */
				continue;
			} else
				gain = 0;
			trx_if_data(l1h, tn, fn, gain, bits);
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

		/* flush pending messages of transceiver */
		/* close all logical channels and reset timeslots */
		llist_for_each_entry(trx, &bts->trx_list, list) {
			trx_if_flush(trx_l1h_hdl(trx));
			trx_sched_reset(trx_l1sched_hdl(trx));
			if (trx->nr == 0)
				trx_if_cmd_poweroff(trx_l1h_hdl(trx));
		}

		/* tell BSC */
		check_transceiver_availability(bts, 0);

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
		trx_sched_fn(transceiver_last_fn);
		elapsed -= FRAME_DURATION_uS;
	}
	osmo_timer_schedule(&transceiver_clock_timer, 0,
		FRAME_DURATION_uS - elapsed);
}


/* receive clock from transceiver */
int trx_sched_clock(uint32_t fn)
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
		trx_sched_fn(transceiver_last_fn);

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
		trx_sched_fn(transceiver_last_fn);
	}

	/* schedule next FN to be transmitted */
	memcpy(tv_clock, &tv_now, sizeof(struct timeval));
	osmo_timer_schedule(&transceiver_clock_timer, 0, FRAME_DURATION_uS);

	return 0;
}

struct l1sched_ts *l1sched_trx_get_ts(struct l1sched_trx *l1t, uint8_t tn)
{
	OSMO_ASSERT(tn < ARRAY_SIZE(l1t->ts));
	return &l1t->ts[tn];
}
