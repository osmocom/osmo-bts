/* Scheduler for OsmoBTS-TRX */

/* (C) 2013 by Andreas Eversberg <jolly@eversberg.eu>
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
#include <osmo-bts/amr.h>

#include "l1_if.h"
#include "scheduler.h"
#include "gsm0503_coding.h"
#include "trx_if.h"
#include "loops.h"
#include "amr.h"
#include "loops.h"

/* Enable this to multiply TOA of RACH by 10.
 * This usefull to check tenth of timing advances with RSSI test tool.
 * Note that regular phones will not work when using this test! */
//#define TA_TEST

void *tall_bts_ctx;

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

typedef int trx_sched_rts_func(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan);
typedef ubit_t *trx_sched_dl_func(struct trx_l1h *l1h, uint8_t tn,
	uint32_t fn, enum trx_chan_type chan, uint8_t bid);
typedef int trx_sched_ul_func(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, sbit_t *bits, int8_t rssi,
	float toa);

static int rts_data_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan);
static int rts_tchf_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan);
static int rts_tchh_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan);
static ubit_t *tx_idle_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid);
static ubit_t *tx_fcch_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid);
static ubit_t *tx_sch_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid);
static ubit_t *tx_data_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid);
static ubit_t *tx_pdtch_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid);
static ubit_t *tx_tchf_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid);
static ubit_t *tx_tchh_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid);
static int rx_rach_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, sbit_t *bits, int8_t rssi,
	float toa);
static int rx_data_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, sbit_t *bits, int8_t rssi,
	float toa);
static int rx_pdtch_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, sbit_t *bits, int8_t rssi,
	float toa);
static int rx_tchf_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, sbit_t *bits, int8_t rssi,
	float toa);
static int rx_tchh_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, sbit_t *bits, int8_t rssi,
	float toa);

static ubit_t dummy_burst[148] = {
	0,0,0,
	1,1,1,1,1,0,1,1,0,1,1,1,0,1,1,0,0,0,0,0,1,0,1,0,0,1,0,0,1,1,1,0,
	0,0,0,0,1,0,0,1,0,0,0,1,0,0,0,0,0,0,0,1,1,1,1,1,0,0,0,1,1,1,0,0,
	0,1,0,1,1,1,0,0,0,1,0,1,1,1,0,0,0,1,0,1,0,1,1,1,0,1,0,0,1,0,1,0,
	0,0,1,1,0,0,1,1,0,0,1,1,1,0,0,1,1,1,1,0,1,0,0,1,1,1,1,1,0,0,0,1,
	0,0,1,0,1,1,1,1,1,0,1,0,1,0,
	0,0,0,
};

static ubit_t fcch_burst[148] = {
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
};

static const ubit_t tsc[8][26] = {
	{ 0,0,1,0,0,1,0,1,1,1,0,0,0,0,1,0,0,0,1,0,0,1,0,1,1,1, },
	{ 0,0,1,0,1,1,0,1,1,1,0,1,1,1,1,0,0,0,1,0,1,1,0,1,1,1, },
	{ 0,1,0,0,0,0,1,1,1,0,1,1,1,0,1,0,0,1,0,0,0,0,1,1,1,0, },
	{ 0,1,0,0,0,1,1,1,1,0,1,1,0,1,0,0,0,1,0,0,0,1,1,1,1,0, },
	{ 0,0,0,1,1,0,1,0,1,1,1,0,0,1,0,0,0,0,0,1,1,0,1,0,1,1, },
	{ 0,1,0,0,1,1,1,0,1,0,1,1,0,0,0,0,0,1,0,0,1,1,1,0,1,0, },
	{ 1,0,1,0,0,1,1,1,1,1,0,1,1,0,0,0,1,0,1,0,0,1,1,1,1,1, },
	{ 1,1,1,0,1,1,1,1,0,0,0,1,0,0,1,0,1,1,1,0,1,1,1,1,0,0, },
};

static const ubit_t sch_train[64] = {
	1,0,1,1,1,0,0,1,0,1,1,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,1,1,1,
	0,0,1,0,1,1,0,1,0,1,0,0,0,1,0,1,0,1,1,1,0,1,1,0,0,0,0,1,1,0,1,1,
};

/*
 * subchannel description structure
 */

struct trx_chan_desc {
	int			pdch;
	enum trx_chan_type	chan;
	uint8_t			chan_nr;
	uint8_t			link_id;
	const char		*name;
	trx_sched_rts_func	*rts_fn;
	trx_sched_dl_func	*dl_fn;
	trx_sched_ul_func	*ul_fn;
	int			auto_active;
};
struct trx_chan_desc trx_chan_desc[_TRX_CHAN_MAX] = {
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
      {	0,	TRXC_SACCH8_7,	0x68,	0x40,	"SACCH/8(7)",	rts_data_fn,	tx_data_fn,	rx_data_fn,	0 },
      {	1,	TRXC_PDTCH,	0x08,	0x00,	"PDTCH",	rts_data_fn,	tx_pdtch_fn,	rx_pdtch_fn,	0 },
      {	1,	TRXC_PTCCH,	0x08,	0x00,	"PTCCH",	rts_data_fn,	tx_data_fn,	rx_data_fn,	0 },
};


/*
 * init / exit
 */

int trx_sched_init(struct trx_l1h *l1h)
{
	uint8_t tn;
	int i;
	struct trx_chan_state *chan_state;

	LOGP(DL1C, LOGL_NOTICE, "Init scheduler for trx=%u\n", l1h->trx->nr);

	/* hack to get bts */
	bts = l1h->trx->bts;

	for (tn = 0; tn < 8; tn++) {
		l1h->mf_index[tn] = 0;
		l1h->mf_last_fn[tn] = 0;
		INIT_LLIST_HEAD(&l1h->dl_prims[tn]);
		for (i = 0; i < _TRX_CHAN_MAX; i++) {
			chan_state = &l1h->chan_states[tn][i];
			chan_state->active = 0;
		}
	}

	return 0;
}

void trx_sched_exit(struct trx_l1h *l1h)
{
	uint8_t tn;
	int i;
	struct trx_chan_state *chan_state;

	LOGP(DL1C, LOGL_NOTICE, "Exit scheduler for trx=%u\n", l1h->trx->nr);

	for (tn = 0; tn < 8; tn++) {
		msgb_queue_flush(&l1h->dl_prims[tn]);
		for (i = 0; i < _TRX_CHAN_MAX; i++) {
			chan_state = &l1h->chan_states[tn][i];
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
		for (i = 0; i < 8; i++)
			l1h->trx->ts[tn].lchan[i].state = LCHAN_S_NONE;
	}
}

/* close all logical channels and reset timeslots */
void trx_sched_reset(struct trx_l1h *l1h)
{
	trx_sched_exit(l1h);
	trx_sched_init(l1h);
}


/*
 * data request (from upper layer)
 */

int trx_sched_ph_data_req(struct trx_l1h *l1h, struct osmo_phsap_prim *l1sap)
{
	uint8_t tn = l1sap->u.data.chan_nr & 7;

	LOGP(DL1C, LOGL_INFO, "PH-DATA.req: chan_nr=0x%02x link_id=0x%02x "
		"fn=%u ts=%u trx=%u\n", l1sap->u.data.chan_nr,
		l1sap->u.data.link_id, l1sap->u.data.fn, tn, l1h->trx->nr);

	if (!l1sap->oph.msg)
		abort();

	/* ignore empty frame */
	if (!msgb_l2len(l1sap->oph.msg)) {
		msgb_free(l1sap->oph.msg);
		return 0;
	}

	msgb_enqueue(&l1h->dl_prims[tn], l1sap->oph.msg);

	return 0;
}

int trx_sched_tch_req(struct trx_l1h *l1h, struct osmo_phsap_prim *l1sap)
{
	uint8_t tn = l1sap->u.tch.chan_nr & 7;

	LOGP(DL1C, LOGL_INFO, "TCH.req: chan_nr=0x%02x "
		"fn=%u ts=%u trx=%u\n", l1sap->u.tch.chan_nr,
		l1sap->u.tch.fn, tn, l1h->trx->nr);

	if (!l1sap->oph.msg)
		abort();

	/* ignore empty frame */
	if (!msgb_l2len(l1sap->oph.msg)) {
		msgb_free(l1sap->oph.msg);
		return 0;
	}

	msgb_enqueue(&l1h->dl_prims[tn], l1sap->oph.msg);

	return 0;
}


/* 
 * ready-to-send indication (to upper layer)
 */

/* RTS for data frame */
static int rts_data_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
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
		chan_nr, link_id, fn, tn, l1h->trx->nr);

	/* send clock information to loops process */
	if (L1SAP_IS_LINK_SACCH(link_id))
		trx_loop_sacch_clock(l1h, chan_nr, &l1h->chan_states[tn][chan]);

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

	return l1sap_up(l1h->trx, l1sap);
}

static int rts_tch_common(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, int facch)
{
	uint8_t chan_nr, link_id;
	struct msgb *msg;
	struct osmo_phsap_prim *l1sap;
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
		chan_nr, fn, tn, l1h->trx->nr);

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

		rc = l1sap_up(l1h->trx, l1sap);
	}

	/* dont send, if TCH is in signalling only mode */
	if (l1h->chan_states[tn][chan].rsl_cmode != RSL_CMOD_SPD_SIGN) {
		/* generate prim */
		msg = l1sap_msgb_alloc(200);
		if (!msg)
			return -ENOMEM;
		l1sap = msgb_l1sap_prim(msg);
		osmo_prim_init(&l1sap->oph, SAP_GSM_PH, PRIM_TCH_RTS,
						PRIM_OP_INDICATION, msg);
		l1sap->u.tch.chan_nr = chan_nr;
		l1sap->u.tch.fn = fn;

		return l1sap_up(l1h->trx, l1sap);
	}

	return rc;
}

/* RTS for full rate traffic frame */
static int rts_tchf_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan)
{
	/* TCH/F may include FACCH on every 4th burst */
	return rts_tch_common(l1h, tn, fn, chan, 1);
}


/* RTS for half rate traffic frame */
static int rts_tchh_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan)
{
	/* the FN 4/5, 13/14, 21/22 defines that FACCH may be included. */
	return rts_tch_common(l1h, tn, fn, chan, ((fn % 26) >> 2) & 1);
}


/*
 * TX on donlink
 */

/* an IDLE burst returns nothing. on C0 it is replaced by dummy burst */
static ubit_t *tx_idle_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid)
{
	LOGP(DL1C, LOGL_DEBUG, "Transmitting %s fn=%u ts=%u trx=%u\n",
		trx_chan_desc[chan].name, fn, tn, l1h->trx->nr);

	return NULL;
}

static ubit_t *tx_fcch_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid)
{
	LOGP(DL1C, LOGL_DEBUG, "Transmitting %s fn=%u ts=%u trx=%u\n",
		trx_chan_desc[chan].name, fn, tn, l1h->trx->nr);

	return fcch_burst;
}

static ubit_t *tx_sch_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid)
{
	static ubit_t bits[148], burst[78];
	uint8_t sb_info[4];
	struct	gsm_time t;
	uint8_t t3p, bsic;
	
	LOGP(DL1C, LOGL_DEBUG, "Transmitting %s fn=%u ts=%u trx=%u\n",
		trx_chan_desc[chan].name, fn, tn, l1h->trx->nr);

	/* create SB info from GSM time and BSIC */
	gsm_fn2gsmtime(&t, fn);
	t3p = t.t3 / 10;
	bsic = l1h->trx->bts->bsic;
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
	sch_encode(burst, sb_info);

	/* compose burst */
	memset(bits, 0, 3);
	memcpy(bits + 3, burst, 39);
	memcpy(bits + 42, sch_train, 64);
	memcpy(bits + 106, burst + 39, 39);
	memset(bits + 145, 0, 3);

	return bits;
}

static struct msgb *dequeue_prim(struct trx_l1h *l1h, int8_t tn,uint32_t fn,
	enum trx_chan_type chan)
{
	struct msgb *msg, *msg2;
	struct osmo_phsap_prim *l1sap;
	uint32_t prim_fn;
	uint8_t chan_nr, link_id;

	/* get prim of current fn from queue */
	llist_for_each_entry_safe(msg, msg2, &l1h->dl_prims[tn], list) {
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
			prim_fn = ((l1sap->u.data.fn + 2715648 - fn) % 2715648);
			break;
		case PRIM_TCH:
			chan_nr = l1sap->u.tch.chan_nr;
			link_id = 0;
			prim_fn = ((l1sap->u.tch.fn + 2715648 - fn) % 2715648);
			break;
		default:
			goto wrong_type;
		}
		if (prim_fn > 20) {
			LOGP(DL1C, LOGL_NOTICE, "Prim for trx=%u ts=%u at fn=%u "
				"is out of range, or channel already disabled. "
				"(current fn=%u)\n", l1h->trx->nr, tn, prim_fn,
				fn);
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

static int compose_ph_data_ind(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t *l2, uint8_t l2_len, float toa,
	float ber, float rssi)
{
	struct msgb *msg;
	struct osmo_phsap_prim *l1sap;
	uint8_t chan_nr = trx_chan_desc[chan].chan_nr | tn;

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
		l1h->chan_states[tn][chan].lost = 0;

	/* forward primitive */
	l1sap_up(l1h->trx, l1sap);

	/* process measurement */
	if (L1SAP_IS_LINK_SACCH(trx_chan_desc[chan].link_id))
		l1if_process_meas_res(l1h->trx, chan_nr,
			(l1h->trx->ts[tn].lchan[l1sap_chan2ss(chan_nr)].rqd_ta + toa) * 4,
			ber, rssi);

	return 0;
}

static int compose_tch_ind(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t *tch, uint8_t tch_len)
{
	struct msgb *msg;
	struct osmo_phsap_prim *l1sap;

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

	if (l1h->chan_states[tn][chan].lost)
		l1h->chan_states[tn][chan].lost--;

	/* forward primitive */
	l1sap_up(l1h->trx, l1sap);

	return 0;
}

static ubit_t *tx_data_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid)
{
	struct msgb *msg = NULL; /* make GCC happy */
	ubit_t *burst, **bursts_p = &l1h->chan_states[tn][chan].dl_bursts;
	static ubit_t bits[148];

	/* send burst, if we already got a frame */
	if (bid > 0) {
		if (!*bursts_p)
			return NULL;
		goto send_burst;
	}

	/* get burst from queue */
	msg = dequeue_prim(l1h, tn, fn, chan);
	if (msg)
		goto got_msg;

	LOGP(DL1C, LOGL_NOTICE, "%s has not been served !! No prim for "
		"trx=%u ts=%u at fn=%u to transmit.\n", 
		trx_chan_desc[chan].name, l1h->trx->nr, tn, fn);

no_msg:
	/* free burst memory */
	if (*bursts_p) {
		talloc_free(*bursts_p);
		*bursts_p = NULL;
	}
	return NULL;

got_msg:
	/* check validity of message */
	if (msgb_l2len(msg) != 23) {
		LOGP(DL1C, LOGL_FATAL, "Prim not 23 bytes, please FIX! "
			"(len=%d)\n", msgb_l2len(msg));
		/* free message */
		msgb_free(msg);
		goto no_msg;
	}

	/* handle loss detection of sacch */
	if (L1SAP_IS_LINK_SACCH(trx_chan_desc[chan].link_id)) {
		/* count and send BFI */
		if (++(l1h->chan_states[tn][chan].lost) > 1)
			compose_ph_data_ind(l1h, tn, 0, chan, NULL, 0, 0, 0,
				-110);
	}

	/* alloc burst memory, if not already */
	if (!*bursts_p) {
		*bursts_p = talloc_zero_size(tall_bts_ctx, 464);
		if (!*bursts_p)
			return NULL;
	}

	/* encode bursts */
	xcch_encode(*bursts_p, msg->l2h);

	/* free message */
	msgb_free(msg);

send_burst:
	/* compose burst */
	burst = *bursts_p + bid * 116;
	memset(bits, 0, 3);
	memcpy(bits + 3, burst, 58);
	memcpy(bits + 61, tsc[l1h->config.tsc], 26);
	memcpy(bits + 87, burst + 58, 58);
	memset(bits + 145, 0, 3);

	LOGP(DL1C, LOGL_DEBUG, "Transmitting %s fn=%u ts=%u trx=%u burst=%u\n",
		trx_chan_desc[chan].name, fn, tn, l1h->trx->nr, bid);

	return bits;
}

static ubit_t *tx_pdtch_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid)
{
	struct msgb *msg = NULL; /* make GCC happy */
	ubit_t *burst, **bursts_p = &l1h->chan_states[tn][chan].dl_bursts;
	static ubit_t bits[148];
	int rc;

	/* send burst, if we already got a frame */
	if (bid > 0) {
		if (!*bursts_p)
			return NULL;
		goto send_burst;
	}

	/* get burst from queue */
	msg = dequeue_prim(l1h, tn, fn, chan);
	if (msg)
		goto got_msg;

	LOGP(DL1C, LOGL_NOTICE, "%s has not been served !! No prim for "
		"trx=%u ts=%u at fn=%u to transmit.\n", 
		trx_chan_desc[chan].name, l1h->trx->nr, tn, fn);

no_msg:
	/* free burst memory */
	if (*bursts_p) {
		talloc_free(*bursts_p);
		*bursts_p = NULL;
	}
	return NULL;

got_msg:
	/* alloc burst memory, if not already */
	if (!*bursts_p) {
		*bursts_p = talloc_zero_size(tall_bts_ctx, 464);
		if (!*bursts_p)
			return NULL;
	}

	/* encode bursts */
	rc = pdtch_encode(*bursts_p, msg->l2h, msg->tail - msg->l2h);

	/* check validity of message */
	if (rc) {
		LOGP(DL1C, LOGL_FATAL, "Prim invalid length, please FIX! "
			"(len=%d)\n", rc);
		/* free message */
		msgb_free(msg);
		goto no_msg;
	}

	/* free message */
	msgb_free(msg);

send_burst:
	/* compose burst */
	burst = *bursts_p + bid * 116;
	memset(bits, 0, 3);
	memcpy(bits + 3, burst, 58);
	memcpy(bits + 61, tsc[l1h->config.tsc], 26);
	memcpy(bits + 87, burst + 58, 58);
	memset(bits + 145, 0, 3);

	LOGP(DL1C, LOGL_DEBUG, "Transmitting %s fn=%u ts=%u trx=%u burst=%u\n",
		trx_chan_desc[chan].name, fn, tn, l1h->trx->nr, bid);

	return bits;
}

static void tx_tch_common(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, struct msgb **_msg_tch,
	struct msgb **_msg_facch, int codec_mode_request)
{
	struct msgb *msg1, *msg2, *msg_tch = NULL, *msg_facch = NULL;
	struct trx_chan_state *chan_state = &l1h->chan_states[tn][chan];
	uint8_t rsl_cmode = chan_state->rsl_cmode;
	uint8_t tch_mode = chan_state->tch_mode;
	struct osmo_phsap_prim *l1sap;

	/* handle loss detection of received TCH frames */
	if (rsl_cmode == RSL_CMOD_SPD_SPEECH
	 && ++(l1h->chan_states[tn][chan].lost) > 5) {
		uint8_t tch_data[33];
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
			memset(tch_data, 0, 33);
			len = 33;
			break;
		case GSM48_CMODE_SPEECH_EFR: /* EFR */
			if (chan != TRXC_TCHF)
				goto inval_mode1;
			memset(tch_data, 0, 31);
			len = 31;
			break;
		case GSM48_CMODE_SPEECH_AMR: /* AMR */
			len = amr_compose_payload(tch_data,
				chan_state->codec[chan_state->dl_cmr],
				chan_state->codec[chan_state->dl_ft], 1);
			if (len < 2)
				break;
			memset(tch_data + 2, 0, len - 2);
			compose_tch_ind(l1h, tn, 0, chan, tch_data, len);
			break;
		default:
inval_mode1:
			LOGP(DL1C, LOGL_ERROR, "TCH mode invalid, please "
				"fix!\n");
			len = 0;
		}
		if (len)
			compose_tch_ind(l1h, tn, 0, chan, tch_data, len);
	}

	/* get frame and unlink from queue */
	msg1 = dequeue_prim(l1h, tn, fn, chan);
	msg2 = dequeue_prim(l1h, tn, fn, chan);
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
	if (msg_facch && msgb_l2len(msg_facch) != 23) {
		LOGP(DL1C, LOGL_FATAL, "Prim not 23 bytes, please FIX! "
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
			LOGP(DL1C, LOGL_NOTICE, "%s Dropping speech frame, "
				"because we are not in speech mode trx=%u "
				"ts=%u at fn=%u.\n", trx_chan_desc[chan].name,
				l1h->trx->nr, tn, fn);
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
						l1h->trx->nr, tn, fn);
					goto free_bad_msg;
				}
				break;
			}
			len = 33;
			if (msgb_l2len(msg_tch) >= 1
			 && (msg_tch->l2h[0] >> 4) != 0xd) {
				LOGP(DL1C, LOGL_NOTICE, "%s Transmitting 'bad "
					"FR frame' trx=%u ts=%u at fn=%u.\n",
					trx_chan_desc[chan].name,
					l1h->trx->nr, tn, fn);
				goto free_bad_msg;
			}
			break;
		case GSM48_CMODE_SPEECH_EFR: /* EFR */
			if (chan != TRXC_TCHF)
				goto inval_mode2;
			len = 31;
			if (msgb_l2len(msg_tch) >= 1
			 && (msg_tch->l2h[0] >> 4) != 0xc) {
				LOGP(DL1C, LOGL_NOTICE, "%s Transmitting 'bad "
					"EFR frame' trx=%u ts=%u at fn=%u.\n",
					trx_chan_desc[chan].name,
					l1h->trx->nr, tn, fn);
				goto free_bad_msg;
			}
			break;
		case GSM48_CMODE_SPEECH_AMR: /* AMR */
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
				LOGP(DL1C, LOGL_ERROR, "%s Codec (FT = %d) "
					" of RTP frame not in list. "
					"trx=%u ts=%u\n",
					trx_chan_desc[chan].name, ft_codec,
					l1h->trx->nr, tn);
				goto free_bad_msg;
			}
			if (codec_mode_request && chan_state->dl_ft != ft) {
				LOGP(DL1C, LOGL_NOTICE, "%s Codec (FT = %d) "
					" of RTP cannot be changed now, but in "
					"next frame. trx=%u ts=%u\n",
					trx_chan_desc[chan].name, ft_codec,
					l1h->trx->nr, tn);
				goto free_bad_msg;
			}
			chan_state->dl_ft = ft;
			if (bfi) {
				LOGP(DL1C, LOGL_NOTICE, "%s Transmitting 'bad "
					"AMR frame' trx=%u ts=%u at fn=%u.\n",
					trx_chan_desc[chan].name,
					l1h->trx->nr, tn, fn);
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

static ubit_t *tx_tchf_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid)
{
	struct msgb *msg_tch = NULL, *msg_facch = NULL;
	struct trx_chan_state *chan_state = &l1h->chan_states[tn][chan];
	uint8_t tch_mode = chan_state->tch_mode;
	ubit_t *burst, **bursts_p = &chan_state->dl_bursts;
	static ubit_t bits[148];

	/* send burst, if we already got a frame */
	if (bid > 0) {
		if (!*bursts_p)
			return NULL;
		goto send_burst;
	}

	tx_tch_common(l1h, tn, fn, chan, bid, &msg_tch, &msg_facch,
		(((fn + 4) % 26) >> 2) & 1);

	/* alloc burst memory, if not already,
	 * otherwise shift buffer by 4 bursts for interleaving */
	if (!*bursts_p) {
		*bursts_p = talloc_zero_size(tall_bts_ctx, 928);
		if (!*bursts_p)
			return NULL;
	} else {
		memcpy(*bursts_p, *bursts_p + 464, 464);
		memset(*bursts_p + 464, 0, 464);
	}

	/* mo message at all */
	if (!msg_tch && !msg_facch) {
		LOGP(DL1C, LOGL_NOTICE, "%s has not been served !! No prim for "
			"trx=%u ts=%u at fn=%u to transmit.\n", 
			trx_chan_desc[chan].name, l1h->trx->nr, tn, fn);
		goto send_burst;
	}

	/* encode bursts (priorize FACCH) */
	if (msg_facch)
		tch_fr_encode(*bursts_p, msg_facch->l2h, msgb_l2len(msg_facch),
			1);
	else if (tch_mode == GSM48_CMODE_SPEECH_AMR)
		/* the first FN 4,13,21 defines that CMI is included in frame,
		 * the first FN 0,8,17 defines that CMR is included in frame.
		 */
		tch_afs_encode(*bursts_p, msg_tch->l2h + 2,
			msgb_l2len(msg_tch) - 2, (((fn + 4) % 26) >> 2) & 1,
			chan_state->codec, chan_state->codecs,
			chan_state->dl_ft,
			chan_state->dl_cmr);
	else
		tch_fr_encode(*bursts_p, msg_tch->l2h, msgb_l2len(msg_tch), 1);

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
	memcpy(bits + 61, tsc[l1h->config.tsc], 26);
	memcpy(bits + 87, burst + 58, 58);
	memset(bits + 145, 0, 3);

	LOGP(DL1C, LOGL_DEBUG, "Transmitting %s fn=%u ts=%u trx=%u burst=%u\n",
		trx_chan_desc[chan].name, fn, tn, l1h->trx->nr, bid);

	return bits;
}

static ubit_t *tx_tchh_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid)
{
	struct msgb *msg_tch = NULL, *msg_facch = NULL;
	struct trx_chan_state *chan_state = &l1h->chan_states[tn][chan];
	uint8_t tch_mode = chan_state->tch_mode;
	ubit_t *burst, **bursts_p = &chan_state->dl_bursts;
	static ubit_t bits[148];

	/* send burst, if we already got a frame */
	if (bid > 0) {
		if (!*bursts_p)
			return NULL;
		goto send_burst;
	}

	/* get TCH and/or FACCH */
	tx_tch_common(l1h, tn, fn, chan, bid, &msg_tch, &msg_facch,
		(((fn + 4) % 26) >> 2) & 1);

	/* check for FACCH alignment */
	if (msg_facch && ((((fn + 4) % 26) >> 2) & 1)) {
		LOGP(DL1C, LOGL_ERROR, "%s Cannot transmit FACCH starting on "
			"even frames, please fix RTS!\n",
			trx_chan_desc[chan].name);
		msgb_free(msg_facch);
		msg_facch = NULL;
	}

	/* alloc burst memory, if not already,
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

	/* mo message at all */
	if (!msg_tch && !msg_facch && !chan_state->dl_ongoing_facch) {
		LOGP(DL1C, LOGL_NOTICE, "%s has not been served !! No prim for "
			"trx=%u ts=%u at fn=%u to transmit.\n", 
			trx_chan_desc[chan].name, l1h->trx->nr, tn, fn);
		goto send_burst;
	}

	/* encode bursts (priorize FACCH) */
	if (msg_facch) {
		tch_hr_encode(*bursts_p, msg_facch->l2h, msgb_l2len(msg_facch));
		chan_state->dl_ongoing_facch = 1; /* first of two tch frames */
	} else if (chan_state->dl_ongoing_facch) /* second of two tch frames */
		chan_state->dl_ongoing_facch = 0; /* we are done with FACCH */
	else if (tch_mode == GSM48_CMODE_SPEECH_AMR)
		/* the first FN 4,13,21 or 5,14,22 defines that CMI is included
		 * in frame, the first FN 0,8,17 or 1,9,18 defines that CMR is
		 * included in frame. */
		tch_ahs_encode(*bursts_p, msg_tch->l2h + 2,
			msgb_l2len(msg_tch) - 2, (((fn + 4) % 26) >> 2) & 1,
			chan_state->codec, chan_state->codecs,
			chan_state->dl_ft,
			chan_state->dl_cmr);
	else
		tch_hr_encode(*bursts_p, msg_tch->l2h, msgb_l2len(msg_tch));

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
	memcpy(bits + 61, tsc[l1h->config.tsc], 26);
	memcpy(bits + 87, burst + 58, 58);
	memset(bits + 145, 0, 3);

	LOGP(DL1C, LOGL_DEBUG, "Transmitting %s fn=%u ts=%u trx=%u burst=%u\n",
		trx_chan_desc[chan].name, fn, tn, l1h->trx->nr, bid);

	return bits;
}


/*
 * RX on uplink (indication to upper layer)
 */

static int rx_rach_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, sbit_t *bits, int8_t rssi,
	float toa)
{
	uint8_t chan_nr;
	struct osmo_phsap_prim l1sap;
	uint8_t ra;
	int rc;

	chan_nr = trx_chan_desc[chan].chan_nr | tn;

	LOGP(DL1C, LOGL_NOTICE, "Received Access Burst on %s fn=%u toa=%.2f\n",
		trx_chan_desc[chan].name, fn, toa);

	/* decode */
	rc = rach_decode(&ra, bits + 8 + 41, l1h->trx->bts->bsic);
	if (rc) {
		LOGP(DL1C, LOGL_NOTICE, "Received bad AB frame at fn=%u "
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

	/* forward primitive */
	l1sap_up(l1h->trx, &l1sap);

	return 0;
}

static int rx_data_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, sbit_t *bits, int8_t rssi,
	float toa)
{
	struct trx_chan_state *chan_state = &l1h->chan_states[tn][chan];
	sbit_t *burst, **bursts_p = &chan_state->ul_bursts;
	uint32_t *first_fn = &chan_state->ul_first_fn;
	uint8_t *mask = &chan_state->ul_mask;
	float *rssi_sum = &chan_state->rssi_sum;
	uint8_t *rssi_num = &chan_state->rssi_num;
	float *toa_sum = &chan_state->toa_sum;
	uint8_t *toa_num = &chan_state->toa_num;
	uint8_t l2[23], l2_len;
	int rc;

	/* handle rach, if handover rach detection is turned on */
	if (chan_state->ho_rach_detect == 1)
		return rx_rach_fn(l1h, tn, fn, chan, bid, bits, rssi, toa);

	LOGP(DL1C, LOGL_DEBUG, "Data received %s fn=%u ts=%u trx=%u bid=%u\n",
		trx_chan_desc[chan].name, fn, tn, l1h->trx->nr, bid);

	/* alloc burst memory, if not already */
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

	/* update mask + rssi */
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
		trx_loop_sacch_input(l1h, trx_chan_desc[chan].chan_nr | tn,
			chan_state, rssi, toa);
	}

	/* wait until complete set of bursts */
	if (bid != 3)
		return 0;

	/* check for complete set of bursts */
	if ((*mask & 0xf) != 0xf) {
		LOGP(DL1C, LOGL_NOTICE, "Received incomplete data frame at "
			"fn=%u (%u/%u) for %s\n", *first_fn,
			(*first_fn) % l1h->mf_period[tn], l1h->mf_period[tn],
			trx_chan_desc[chan].name);

		/* we require first burst to have correct FN */
		if (!(*mask & 0x1)) {
			*mask = 0x0;
			return 0;
		}
	}
	*mask = 0x0;

	/* decode */
	rc = xcch_decode(l2, *bursts_p);
	if (rc) {
		LOGP(DL1C, LOGL_NOTICE, "Received bad data frame at fn=%u "
			"(%u/%u) for %s\n", *first_fn,
			(*first_fn) % l1h->mf_period[tn], l1h->mf_period[tn],
			trx_chan_desc[chan].name);
		l2_len = 0;
	} else
		l2_len = 23;

	return compose_ph_data_ind(l1h, tn, *first_fn, chan, l2, l2_len,
		*toa_sum / *toa_num, 0, *rssi_sum / *rssi_num);
}

static int rx_pdtch_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, sbit_t *bits, int8_t rssi,
	float toa)
{
	struct trx_chan_state *chan_state = &l1h->chan_states[tn][chan];
	sbit_t *burst, **bursts_p = &chan_state->ul_bursts;
	uint8_t *mask = &chan_state->ul_mask;
	float *rssi_sum = &chan_state->rssi_sum;
	uint8_t *rssi_num = &chan_state->rssi_num;
	float *toa_sum = &chan_state->toa_sum;
	uint8_t *toa_num = &chan_state->toa_num;
	uint8_t l2[54+1];
	int rc;

	LOGP(DL1C, LOGL_DEBUG, "PDTCH received %s fn=%u ts=%u trx=%u bid=%u\n", 
		trx_chan_desc[chan].name, fn, tn, l1h->trx->nr, bid);

	/* alloc burst memory, if not already */
	if (!*bursts_p) {
		*bursts_p = talloc_zero_size(tall_bts_ctx, 464);
		if (!*bursts_p)
			return -ENOMEM;
	}

	/* clear burst */
	if (bid == 0) {
		memset(*bursts_p, 0, 464);
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
	burst = *bursts_p + bid * 116;
	memcpy(burst, bits + 3, 58);
	memcpy(burst + 58, bits + 87, 58);

	/* wait until complete set of bursts */
	if (bid != 3)
		return 0;

	/* check for complete set of bursts */
	if ((*mask & 0xf) != 0xf) {
		LOGP(DL1C, LOGL_NOTICE, "Received incomplete PDTCH block "
			"ending at fn=%u (%u/%u) for %s\n", fn,
			fn % l1h->mf_period[tn], l1h->mf_period[tn],
			trx_chan_desc[chan].name);
	}
	*mask = 0x0;

	/* decode */
	rc = pdtch_decode(l2 + 1, *bursts_p, NULL);
	if (rc <= 0) {
		LOGP(DL1C, LOGL_NOTICE, "Received bad PDTCH block ending at "
			"fn=%u (%u/%u) for %s\n", fn, fn % l1h->mf_period[tn],
			l1h->mf_period[tn], trx_chan_desc[chan].name);
		return 0;
	}

	l2[0] = 7; /* valid frame */

	return compose_ph_data_ind(l1h, tn, (fn + 2715648 - 3) % 2715648, chan,
		l2, rc + 1, *toa_sum / *toa_num, 0, *rssi_sum / *rssi_num);
}

static int rx_tchf_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, sbit_t *bits, int8_t rssi,
	float toa)
{
	struct trx_chan_state *chan_state = &l1h->chan_states[tn][chan];
	sbit_t *burst, **bursts_p = &chan_state->ul_bursts;
	uint8_t *mask = &chan_state->ul_mask;
	uint8_t rsl_cmode = chan_state->rsl_cmode;
	uint8_t tch_mode = chan_state->tch_mode;
	uint8_t tch_data[128]; /* just to be safe */
	int rc, amr = 0;
	float ber;

	/* handle rach, if handover rach detection is turned on */
	if (chan_state->ho_rach_detect == 1)
		return rx_rach_fn(l1h, tn, fn, chan, bid, bits, rssi, toa);

	LOGP(DL1C, LOGL_DEBUG, "TCH/F received %s fn=%u ts=%u trx=%u bid=%u\n", 
		trx_chan_desc[chan].name, fn, tn, l1h->trx->nr, bid);

	/* alloc burst memory, if not already */
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
			fn % l1h->mf_period[tn], l1h->mf_period[tn],
			trx_chan_desc[chan].name);
	}
	*mask = 0x0;

	/* decode
	 * also shift buffer by 4 bursts for interleaving */
	switch ((rsl_cmode != RSL_CMOD_SPD_SPEECH) ? GSM48_CMODE_SPEECH_V1
								: tch_mode) {
	case GSM48_CMODE_SPEECH_V1: /* FR */
		rc = tch_fr_decode(tch_data, *bursts_p, 1, 0);
		break;
	case GSM48_CMODE_SPEECH_EFR: /* EFR */
		rc = tch_fr_decode(tch_data, *bursts_p, 1, 1);
		break;
	case GSM48_CMODE_SPEECH_AMR: /* AMR */
		/* the first FN 0,8,17 defines that CMI is included in frame,
		 * the first FN 4,13,21 defines that CMR is included in frame.
		 * NOTE: A frame ends 7 FN after start.
		 */
		rc = tch_afs_decode(tch_data + 2, *bursts_p,
			(((fn + 26 - 7) % 26) >> 2) & 1, chan_state->codec,
			chan_state->codecs, &chan_state->ul_ft,
			&chan_state->ul_cmr, &ber);
		if (rc)
			trx_loop_amr_input(l1h,
				trx_chan_desc[chan].chan_nr | tn, chan_state,
				ber);
		amr = 2; /* we store tch_data + 2 header bytes */
		/* only good speech frames get rtp header */
		if (rc != 23 && rc >= 4) {
			rc = amr_compose_payload(tch_data,
				chan_state->codec[chan_state->ul_cmr],
				chan_state->codec[chan_state->ul_ft], 0);
		}
		break;
	default:
		LOGP(DL1C, LOGL_ERROR, "TCH mode %u invalid, please fix!\n",
			tch_mode);
		return -EINVAL;
	}
	memcpy(*bursts_p, *bursts_p + 464, 464);
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
	if (rc == 23) {
		compose_ph_data_ind(l1h, tn, (fn + 2715648 - 7) % 2715648, chan,
			tch_data + amr, 23, 0, 0, 0);
bfi:
		if (rsl_cmode == RSL_CMOD_SPD_SPEECH) {
			/* indicate bad frame */
			switch (tch_mode) {
			case GSM48_CMODE_SPEECH_V1: /* FR */
				memset(tch_data, 0, 33);
				rc = 33;
				break;
			case GSM48_CMODE_SPEECH_EFR: /* EFR */
				memset(tch_data, 0, 31);
				rc = 31;
				break;
			case GSM48_CMODE_SPEECH_AMR: /* AMR */
				rc = amr_compose_payload(tch_data,
					chan_state->codec[chan_state->dl_cmr],
					chan_state->codec[chan_state->dl_ft],
					1);
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
	return compose_tch_ind(l1h, tn, (fn + 2715648 - 7) % 2715648, chan,
		tch_data, rc);
}

static int rx_tchh_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, sbit_t *bits, int8_t rssi,
	float toa)
{
	struct trx_chan_state *chan_state = &l1h->chan_states[tn][chan];
	sbit_t *burst, **bursts_p = &chan_state->ul_bursts;
	uint8_t *mask = &chan_state->ul_mask;
	uint8_t rsl_cmode = chan_state->rsl_cmode;
	uint8_t tch_mode = chan_state->tch_mode;
	uint8_t tch_data[128]; /* just to be safe */
	int rc, amr = 0;
	float ber;

	/* handle rach, if handover rach detection is turned on */
	if (chan_state->ho_rach_detect == 1)
		return rx_rach_fn(l1h, tn, fn, chan, bid, bits, rssi, toa);

	LOGP(DL1C, LOGL_DEBUG, "TCH/H received %s fn=%u ts=%u trx=%u bid=%u\n", 
		trx_chan_desc[chan].name, fn, tn, l1h->trx->nr, bid);

	/* alloc burst memory, if not already */
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
			fn % l1h->mf_period[tn], l1h->mf_period[tn],
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
		rc = tch_hr_decode(tch_data, *bursts_p,
			(((fn + 26 - 10) % 26) >> 2) & 1);
		break;
	case GSM48_CMODE_SPEECH_AMR: /* AMR */
		/* the first FN 0,8,17 or 1,9,18 defines that CMI is included
		 * in frame, the first FN 4,13,21 or 5,14,22 defines that CMR
		 * is included in frame.
		 */
		rc = tch_ahs_decode(tch_data + 2, *bursts_p,
			(((fn + 26 - 10) % 26) >> 2) & 1,
			(((fn + 26 - 10) % 26) >> 2) & 1, chan_state->codec,
			chan_state->codecs, &chan_state->ul_ft,
			&chan_state->ul_cmr, &ber);
		if (rc)
			trx_loop_amr_input(l1h,
				trx_chan_desc[chan].chan_nr | tn, chan_state,
				ber);
		amr = 2; /* we store tch_data + 2 two */
		/* only good speech frames get rtp header */
		if (rc != 23 && rc >= 4) {
			rc = amr_compose_payload(tch_data,
				chan_state->codec[chan_state->ul_cmr],
				chan_state->codec[chan_state->ul_ft], 0);
		}
		break;
	default:
		LOGP(DL1C, LOGL_ERROR, "TCH mode %u invalid, please fix!\n",
			tch_mode);
		return -EINVAL;
	}
	memcpy(*bursts_p, *bursts_p + 232, 232);
	memcpy(*bursts_p + 232, *bursts_p + 464, 232);
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
	if (rc == 23) {
		chan_state->ul_ongoing_facch = 1;
		compose_ph_data_ind(l1h, tn,
			(fn + 2715648 - 10 - ((fn % 26) >= 19)) % 2715648, chan,
			tch_data + amr, 23, 0, 0, 0); 
bfi:
		if (rsl_cmode == RSL_CMOD_SPD_SPEECH) {
			/* indicate bad frame */
			switch (tch_mode) {
			case GSM48_CMODE_SPEECH_V1: /* HR */
				tch_data[0] = 0x70; /* F = 0, FT = 111 */
				memset(tch_data + 1, 0, 14);
				rc = 15;
				break;
			case GSM48_CMODE_SPEECH_AMR: /* AMR */
				rc = amr_compose_payload(tch_data,
					chan_state->codec[chan_state->dl_cmr],
					chan_state->codec[chan_state->dl_ft],
					1);
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
	return compose_tch_ind(l1h, tn,
		(fn + 2715648 - 10 - ((fn%26)==19) - ((fn%26)==20)) % 2715648,
		chan, tch_data, rc);
}


/*
 * multiframe structure
 */

/* frame structures */
struct trx_sched_frame {
	enum trx_chan_type		dl_chan;
	uint8_t				dl_bid;
	enum trx_chan_type		ul_chan;
	uint8_t				ul_bid;
};

static struct trx_sched_frame frame_bcch[51] = {
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

static struct trx_sched_frame frame_bcch_sdcch4[102] = {
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

static struct trx_sched_frame frame_sdcch8[102] = {
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

static struct trx_sched_frame frame_tchf_ts0[104] = {
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

static struct trx_sched_frame frame_tchf_ts1[104] = {
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

static struct trx_sched_frame frame_tchf_ts2[104] = {
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

static struct trx_sched_frame frame_tchf_ts3[104] = {
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

static struct trx_sched_frame frame_tchf_ts4[104] = {
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

static struct trx_sched_frame frame_tchf_ts5[104] = {
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

static struct trx_sched_frame frame_tchf_ts6[104] = {
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

static struct trx_sched_frame frame_tchf_ts7[104] = {
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

static struct trx_sched_frame frame_tchh_ts01[104] = {
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

static struct trx_sched_frame frame_tchh_ts23[104] = {
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

static struct trx_sched_frame frame_tchh_ts45[104] = {
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

static struct trx_sched_frame frame_tchh_ts67[104] = {
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

static struct trx_sched_frame frame_pdch[104] = {
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
	enum gsm_phys_chan_config	pchan;
	uint8_t				slotmask;
	uint8_t				period;
	struct trx_sched_frame		*frames;
	const char 			*name;
};

static struct trx_sched_multiframe trx_sched_multiframes[] = {
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
int trx_sched_set_pchan(struct trx_l1h *l1h, uint8_t tn,
	enum gsm_phys_chan_config pchan)
{
	int i;

	/* ignore disabled slots */
	if (!(l1h->config.slotmask & (1 << tn)))
		return -ENOTSUP;

	for (i = 0; i < ARRAY_SIZE(trx_sched_multiframes); i++) {
		if (trx_sched_multiframes[i].pchan == pchan
		 && (trx_sched_multiframes[i].slotmask & (1 << tn))) {
			l1h->mf_index[tn] = i;
			l1h->mf_period[tn] = trx_sched_multiframes[i].period;
			l1h->mf_frames[tn] = trx_sched_multiframes[i].frames;
			LOGP(DL1C, LOGL_NOTICE, "Configuring multiframe with "
				"%s trx=%d ts=%d\n",
				trx_sched_multiframes[i].name,
				l1h->trx->nr, tn);
			return 0;
		}
	}

	LOGP(DL1C, LOGL_NOTICE, "Failed to configuring multiframe "
		"trx=%d ts=%d\n", l1h->trx->nr, tn);

	return -ENOTSUP;
}

/* setting all logical channels given attributes to active/inactive */
int trx_sched_set_lchan(struct trx_l1h *l1h, uint8_t chan_nr, uint8_t link_id,
	int active)
{
	uint8_t tn = L1SAP_CHAN2TS(chan_nr);
	uint8_t ss = l1sap_chan2ss(chan_nr);
	int i;
	int rc = -EINVAL;
	struct trx_chan_state *chan_state;

	/* look for all matching chan_nr/link_id */
	for (i = 0; i < _TRX_CHAN_MAX; i++) {
		/* skip if pchan type does not match pdch flag */
		if ((trx_sched_multiframes[l1h->mf_index[tn]].pchan
							== GSM_PCHAN_PDCH)
						!= trx_chan_desc[i].pdch)
			continue;
		if (trx_chan_desc[i].chan_nr == (chan_nr & 0xf8)
		 && trx_chan_desc[i].link_id == link_id) {
			chan_state = &l1h->chan_states[tn][i];
			rc = 0;
			if (chan_state->active == active)
				continue;
			LOGP(DL1C, LOGL_NOTICE, "%s %s on trx=%d ts=%d\n",
				(active) ? "Activating" : "Deactivating",
				trx_chan_desc[i].name, l1h->trx->nr, tn);
			if (active)
				memset(chan_state, 0, sizeof(*chan_state));
			chan_state->active = active;
		}
	}

	/* disable handover detection (on deactivation) */
	if (l1h->ho_rach_detect[tn][ss]) {
		l1h->ho_rach_detect[tn][ss] = 0;
		trx_if_cmd_nohandover(l1h, tn, ss);
	}

	return rc;
}

/* setting all logical channels given attributes to active/inactive */
int trx_sched_set_mode(struct trx_l1h *l1h, uint8_t chan_nr, uint8_t rsl_cmode,
	uint8_t tch_mode, int codecs, uint8_t codec0, uint8_t codec1,
	uint8_t codec2, uint8_t codec3, uint8_t initial_id, uint8_t handover)
{
	uint8_t tn = L1SAP_CHAN2TS(chan_nr);
	uint8_t ss = l1sap_chan2ss(chan_nr);
	int i;
	int rc = -EINVAL;
	struct trx_chan_state *chan_state;

	/* look for all matching chan_nr/link_id */
	for (i = 0; i < _TRX_CHAN_MAX; i++) {
		if (trx_chan_desc[i].chan_nr == (chan_nr & 0xf8)
		 && trx_chan_desc[i].link_id == 0x00) {
			chan_state = &l1h->chan_states[tn][i];
			LOGP(DL1C, LOGL_NOTICE, "Set mode %u, %u, handover %u "
				"on %s of trx=%d ts=%d\n", rsl_cmode, tch_mode,
				handover, trx_chan_desc[i].name, l1h->trx->nr,
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
		l1h->ho_rach_detect[tn][ss] = 1;
		trx_if_cmd_handover(l1h, tn, ss);
	} else if (l1h->ho_rach_detect[tn][ss]) {
		l1h->ho_rach_detect[tn][ss] = 0;
		trx_if_cmd_nohandover(l1h, tn, ss);
	}

	return rc;
}

/* setting cipher on logical channels */
int trx_sched_set_cipher(struct trx_l1h *l1h, uint8_t chan_nr, int downlink,
	int algo, uint8_t *key, int key_len)
{
	uint8_t tn = L1SAP_CHAN2TS(chan_nr);
	int i;
	int rc = -EINVAL;
	struct trx_chan_state *chan_state;

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
			chan_state = &l1h->chan_states[tn][i];
			LOGP(DL1C, LOGL_NOTICE, "Set a5/%d %s for %s on trx=%d "
				"ts=%d\n", algo,
				(downlink) ? "downlink" : "uplink",
				trx_chan_desc[i].name, l1h->trx->nr, tn);
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
static int trx_sched_rts(struct trx_l1h *l1h, uint8_t tn, uint32_t fn)
{
	struct trx_sched_frame *frame;
	uint8_t offset, period, bid;
	trx_sched_rts_func *func;
	enum trx_chan_type chan;

	/* no multiframe set */
	if (!l1h->mf_index[tn])
		return 0;

	/* get frame from multiframe */
	period = l1h->mf_period[tn];
	offset = fn % period;
	frame = l1h->mf_frames[tn] + offset;

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
	 && !l1h->chan_states[tn][chan].active)
	 	return -EINVAL;

	return func(l1h, tn, fn, frame->dl_chan);
}

/* process downlink burst */
static const ubit_t *trx_sched_dl_burst(struct trx_l1h *l1h, uint8_t tn,
	uint32_t fn)
{
	struct trx_sched_frame *frame;
	uint8_t offset, period, bid;
	trx_sched_dl_func *func;
	enum trx_chan_type chan;
	ubit_t *bits = NULL;

	if (!l1h->mf_index[tn])
		goto no_data;

	/* get frame from multiframe */
	period = l1h->mf_period[tn];
	offset = fn % period;
	frame = l1h->mf_frames[tn] + offset;
  
	chan = frame->dl_chan;
	bid = frame->dl_bid;
	func = trx_chan_desc[chan].dl_fn;

	/* check if channel is active */
	if (!trx_chan_desc[chan].auto_active
	 && !l1h->chan_states[tn][chan].active)
	 	goto no_data;

	/* get burst from function */
	bits = func(l1h, tn, fn, chan, bid);

	/* encrypt */
	if (bits && l1h->chan_states[tn][chan].dl_encr_algo) {
		ubit_t ks[114];
		int i;

		osmo_a5(l1h->chan_states[tn][chan].dl_encr_algo,
			l1h->chan_states[tn][chan].dl_encr_key, fn, ks, NULL);
		for (i = 0; i < 57; i++) {
			bits[i + 3] ^= ks[i];
			bits[i + 88] ^= ks[i + 57];
		}
	}

no_data:
	/* in case of C0, we need a dummy burst to maintain RF power */
	if (bits == NULL && l1h->trx == l1h->trx->bts->c0) {
if (0)		if (chan != TRXC_IDLE) // hack
		LOGP(DL1C, LOGL_DEBUG, "No burst data for %s fn=%u ts=%u "
			"burst=%d on C0, so filling with dummy burst\n",
			trx_chan_desc[chan].name, fn, tn, bid);
		bits = dummy_burst;
	}

	return bits;
}

/* process uplink burst */
int trx_sched_ul_burst(struct trx_l1h *l1h, uint8_t tn, uint32_t current_fn,
	sbit_t *bits, int8_t rssi, float toa)
{
	struct trx_sched_frame *frame;
	uint8_t offset, period, bid;
	trx_sched_ul_func *func;
	enum trx_chan_type chan;
	uint32_t fn, elapsed;

	if (!l1h->mf_index[tn])
		return -EINVAL;

	/* calculate how many frames have been elapsed */
	elapsed = (current_fn + 2715648 - l1h->mf_last_fn[tn]) % 2715648;

	/* start counting from last fn + 1, but only if not too many fn have
	 * been elapsed */
	if (elapsed < 10)
		fn = (l1h->mf_last_fn[tn] + 1) % 2715648;
	else
		fn = current_fn;

	while (42) {
		/* get frame from multiframe */
		period = l1h->mf_period[tn];
		offset = fn % period;
		frame = l1h->mf_frames[tn] + offset;

		chan = frame->ul_chan;
		bid = frame->ul_bid;
		func = trx_chan_desc[chan].ul_fn;

		/* check if channel is active */
		if (!trx_chan_desc[chan].auto_active
		 && !l1h->chan_states[tn][chan].active)
			goto next_frame;

		/* omit bursts which have no handler, like IDLE bursts */
		if (!func)
			goto next_frame;

		/* put burst to function */
		if (fn == current_fn) {
			/* decrypt */
			if (bits && l1h->chan_states[tn][chan].ul_encr_algo) {
				ubit_t ks[114];
				int i;

				osmo_a5(l1h->chan_states[tn][chan].ul_encr_algo,
					l1h->chan_states[tn][chan].ul_encr_key,
					fn, NULL, ks);
				for (i = 0; i < 57; i++) {
					if (ks[i])
						bits[i + 3] = - bits[i + 3];
					if (ks[i + 57])
						bits[i + 88] = - bits[i + 88];
				}
			}

			func(l1h, tn, fn, chan, bid, bits, rssi, toa);
		} else if (chan != TRXC_RACH
		        && !l1h->chan_states[tn][chan].ho_rach_detect) {
			sbit_t spare[148];

			memset(spare, 0, 148);
			func(l1h, tn, fn, chan, bid, spare, -128, 0);
		}

next_frame:
		/* reached current fn */
		if (fn == current_fn)
			break;

		fn = (fn + 1) % 2715648;
	}

	l1h->mf_last_fn[tn] = fn;

	return 0;
}

/* schedule all frames of all TRX for given FN */
static int trx_sched_fn(uint32_t fn)
{
	struct gsm_bts_trx *trx;
	struct trx_l1h *l1h;
	uint8_t tn;
	const ubit_t *bits;
	uint8_t gain;

	/* send time indication */
	l1if_mph_time_ind(bts, fn);

	/* advance frame number, so the transceiver has more time until
	 * it must be transmitted. */
	fn = (fn + trx_clock_advance) % 2715648;

	/* process every TRX */
	llist_for_each_entry(trx, &bts->trx_list, list) {
		l1h = trx_l1h_hdl(trx);

		/* we don't schedule, if power is off */
		if (!l1h->config.poweron)
			continue;

		/* process every TS of TRX */
		for (tn = 0; tn < 8; tn++) {
			/* ignore disabled slots */
			if (!(l1h->config.slotmask & (1 << tn)))
				continue;
			/* ready-to-send */
			trx_sched_rts(l1h, tn,
				(fn + trx_rts_advance) % 2715648);
			/* get burst for FN */
			bits = trx_sched_dl_burst(l1h, tn, fn);
			if (!bits) {
				/* if no bits, send dummy burst with no gain */
				bits = dummy_burst;
				gain = 128;
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
	struct timeval tv_now, *tv_clock = &transceiver_clock_tv;
	int32_t elapsed;

	/* check if transceiver is still alive */
	if (transceiver_lost++ == TRX_LOSS_FRAMES) {
		struct gsm_bts_trx *trx;

		LOGP(DL1C, LOGL_NOTICE, "No more clock from traneiver\n");

no_clock:
		transceiver_available = 0;

		/* flush pending messages of transceiver */
		/* close all logical channels and reset timeslots */
		llist_for_each_entry(trx, &bts->trx_list, list) {
			trx_if_flush(trx_l1h_hdl(trx));
			trx_sched_reset(trx_l1h_hdl(trx));
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
		transceiver_last_fn = (transceiver_last_fn + 1) % 2715648;
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
	elapsed_fn = (fn + 2715648 - transceiver_last_fn) % 2715648;
	if (elapsed_fn >= 135774)
		elapsed_fn -= 2715648;

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
		/* set time to the time our next FN hast to be transmitted */
		osmo_timer_schedule(&transceiver_clock_timer, 0,
			FRAME_DURATION_uS * (1 - elapsed_fn));

		return 0;
	}

	/* transmit what we still need to transmit */
	while (fn != transceiver_last_fn) {
		transceiver_last_fn = (transceiver_last_fn + 1) % 2715648;
		trx_sched_fn(transceiver_last_fn);
	}

	/* schedule next FN to be transmitted */
	memcpy(tv_clock, &tv_now, sizeof(struct timeval));
	osmo_timer_schedule(&transceiver_clock_timer, 0, FRAME_DURATION_uS);

	return 0;
}

