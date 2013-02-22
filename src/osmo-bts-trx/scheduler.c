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

#include "l1_if.h"
#include "scheduler.h"
#include "gsm0503_coding.h"
#include "trx_if.h"

/* Enable this to multiply TOA of RACH by 10.
 * This usefull to check tenth of timing advances with RSSI test tool.
 * Note that regular phones will not work when using this test! */
//#define TA_TEST

void *tall_bts_ctx;

static struct gsm_bts *bts;

/* clock states */
static uint32_t tranceiver_lost;
uint32_t tranceiver_last_fn;
static struct timeval tranceiver_clock_tv;
static struct osmo_timer_list tranceiver_clock_timer;

/* clock advance for the tranceiver */
uint32_t trx_clock_advance = 20;

/* advance RTS to give some time for data processing. (especially PCU) */
uint32_t trx_rts_advance = 5; /* about 20ms */

typedef int trx_sched_rts_func(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan);
typedef ubit_t *trx_sched_dl_func(struct trx_l1h *l1h, uint8_t tn,
	uint32_t fn, enum trx_chan_type chan, uint8_t bid);
typedef int trx_sched_ul_func(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, sbit_t *bits, float toa);

static int rts_data_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan);
static int rts_tch_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
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
	enum trx_chan_type chan, uint8_t bid, sbit_t *bits, float toa);
static int rx_data_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, sbit_t *bits, float toa);
static int rx_pdtch_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, sbit_t *bits, float toa);
static int rx_tchf_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, sbit_t *bits, float toa);
static int rx_tchh_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, sbit_t *bits, float toa);

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
      {	0,	TRXC_TCHF,	0x08,	0x00,	"TCH/F",	rts_tch_fn,	tx_tchf_fn,	rx_tchf_fn,	0 },
      {	0,	TRXC_TCHH_0,	0x10,	0x00,	"TCH/H(0)",	rts_tch_fn,	tx_tchh_fn,	rx_tchh_fn,	0 },
      {	0,	TRXC_TCHH_1,	0x18,	0x00,	"TCH/H(1)",	rts_tch_fn,	tx_tchh_fn,	rx_tchh_fn,	0 },
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
			chan_state->dl_active = 0;
			chan_state->ul_active = 0;
			chan_state->ul_mask = 0x00;
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

/* RTS for traffic frame */ 
static int rts_tch_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
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

	LOGP(DL1C, LOGL_INFO, "TCH RTS.ind: chan=%s chan_nr=0x%02x "
		"fn=%u ts=%u trx=%u\n", trx_chan_desc[chan].name,
		chan_nr, fn, tn, l1h->trx->nr);

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

	/* stop here, if TCH is in signalling only mode */
	if (l1h->chan_states[tn][chan].rsl_cmode == RSL_CMOD_SPD_SIGN)
		return l1sap_up(l1h->trx, l1sap);
	l1sap_up(l1h->trx, l1sap);

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
			LOGP(DL1C, LOGL_ERROR, "Prim for trx=%u ts=%u at fn=%u "
				"is out of range. (current fn=%u)\n",
				l1h->trx->nr, tn, prim_fn, fn);
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
	enum trx_chan_type chan, uint8_t *l2, uint8_t l2_len)
{
	struct msgb *msg;
	struct osmo_phsap_prim *l1sap;

	/* compose primitive */
	msg = l1sap_msgb_alloc(l2_len);
	l1sap = msgb_l1sap_prim(msg);
	osmo_prim_init(&l1sap->oph, SAP_GSM_PH, PRIM_PH_DATA,
		PRIM_OP_INDICATION, msg);
	l1sap->u.data.chan_nr = trx_chan_desc[chan].chan_nr | tn;
	l1sap->u.data.link_id = trx_chan_desc[chan].link_id;
	l1sap->u.data.fn = fn;
	msg->l2h = msgb_put(msg, l2_len);
	if (l2_len)
		memcpy(msg->l2h, l2, l2_len);

	if (L1SAP_IS_LINK_SACCH(trx_chan_desc[chan].link_id))
		l1h->chan_states[tn][chan].lost = 0;

	/* forward primitive */
	l1sap_up(l1h->trx, l1sap);

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
			compose_ph_data_ind(l1h, tn, 0, chan, NULL, 0);
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

static ubit_t *tx_tchf_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid)
{
	struct msgb *msg1, *msg2, *msg_tch = NULL, *msg_facch = NULL;
	ubit_t *burst, **bursts_p = &l1h->chan_states[tn][chan].dl_bursts;
	static ubit_t bits[148];
	struct osmo_phsap_prim *l1sap;

	/* send burst, if we already got a frame */
	if (bid > 0) {
		if (!*bursts_p)
			return NULL;
		goto send_burst;
	}

	/* handle loss detection of received TCH frames */
	if (++(l1h->chan_states[tn][chan].lost) > 5) {
		uint8_t tch_data[33];

		LOGP(DL1C, LOGL_NOTICE, "Missing TCH bursts detected, sending "
			"BFI for %s\n", trx_chan_desc[chan].name);

		/* indicate bad frame */
		memset(tch_data, 0, sizeof(tch_data));
		// FIXME length depends on codec
		compose_tch_ind(l1h, tn, 0, chan, tch_data, 33);
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
	if (msg_tch && msgb_l2len(msg_tch) != 33) {
		LOGP(DL1C, LOGL_FATAL, "Prim not 33 bytes, please FIX! "
			"(len=%d)\n", msgb_l2len(msg_tch));
		/* free message */
		msgb_free(msg_tch);
		msg_tch = NULL;
	}
	if (msg_facch && msgb_l2len(msg_facch) != 23) {
		LOGP(DL1C, LOGL_FATAL, "Prim not 23 bytes, please FIX! "
			"(len=%d)\n", msgb_l2len(msg_facch));
		/* free message */
		msgb_free(msg_facch);
		msg_facch = NULL;
	}

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

	/* bad frame */
	if (msg_tch && !msg_facch && (msg_tch->l2h[0] >> 4) != 0xd) {
		LOGP(DL1C, LOGL_NOTICE, "%s Transmitting 'bad frame' trx=%u "
			"ts=%u at fn=%u to transmit.\n",
			trx_chan_desc[chan].name, l1h->trx->nr, tn, fn);
		goto send_burst;
	}

	/* encode bursts (priorize FACCH) */
	if (msg_facch)
		tch_fr_encode(*bursts_p, msg_facch->l2h, msgb_l2len(msg_facch),
			1);
	else
		tch_fr_encode(*bursts_p, msg_tch->l2h, msgb_l2len(msg_tch), 1);

	/* unlink and free message */
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
	// FIXME
	return NULL;
}



/*
 * RX on uplink (indication to upper layer)
 */

static int rx_rach_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, sbit_t *bits, float toa)
{
	struct osmo_phsap_prim l1sap;
	uint8_t ra;
	int rc;

	LOGP(DL1C, LOGL_NOTICE, "Received %s fn=%u toa=%.2f\n",
		trx_chan_desc[chan].name, fn, toa);

	/* decode */
	rc = rach_decode(&ra, bits + 8 + 41, l1h->trx->bts->bsic);
	if (rc) {
		LOGP(DL1C, LOGL_NOTICE, "Received bad rach frame at fn=%u "
			"(%u/51)\n", fn, fn % 51);
		return 0;
	}

	/* compose primitive */
	/* generate prim */
	memset(&l1sap, 0, sizeof(l1sap));
	osmo_prim_init(&l1sap.oph, SAP_GSM_PH, PRIM_PH_RACH, PRIM_OP_INDICATION,
		NULL);
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
	enum trx_chan_type chan, uint8_t bid, sbit_t *bits, float toa)
{
	struct trx_chan_state *chan_state = &l1h->chan_states[tn][chan];
	sbit_t *burst, **bursts_p = &chan_state->ul_bursts;
	uint32_t *first_fn = &chan_state->ul_first_fn;
	uint8_t *mask = &chan_state->ul_mask;
	uint8_t l2[23], l2_len;
	int rc;

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
	}

	/* update mask */
	*mask |= (1 << bid);

	/* copy burst to buffer of 4 bursts */
	burst = *bursts_p + bid * 116;
	memcpy(burst, bits + 3, 58);
	memcpy(burst + 58, bits + 87, 58);

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

	return compose_ph_data_ind(l1h, tn, *first_fn, chan, l2, l2_len);
}

static int rx_pdtch_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, sbit_t *bits, float toa)
{
	struct trx_chan_state *chan_state = &l1h->chan_states[tn][chan];
	sbit_t *burst, **bursts_p = &chan_state->ul_bursts;
	uint8_t *mask = &chan_state->ul_mask;
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
	}

	/* update mask */
	*mask |= (1 << bid);

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
		l2, rc + 1);
}

static int rx_tchf_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, sbit_t *bits, float toa)
{
	struct trx_chan_state *chan_state = &l1h->chan_states[tn][chan];
	sbit_t *burst, **bursts_p = &chan_state->ul_bursts;
	uint8_t *mask = &chan_state->ul_mask;
	uint8_t tch_data[33];
	int rc;

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
	rc = tch_fr_decode(tch_data, *bursts_p, 1);
	memcpy(*bursts_p, *bursts_p + 464, 464);
	if (rc < 0) {
		LOGP(DL1C, LOGL_NOTICE, "Received bad TCH frame ending at "
			"fn=%u for %s\n", fn, trx_chan_desc[chan].name);
		goto bfi;
	}

	/* FACCH */
	if (rc == 23) {
		compose_ph_data_ind(l1h, tn, (fn + 2715648 - 7) % 2715648, chan,
			tch_data, 23);
bfi:
		// FIXME length depends on codec
		rc = 33;
		/* indicate bad tch frame */
		memset(tch_data, 0, sizeof(tch_data));
	}

	/* TCH or BFI */
	return compose_tch_ind(l1h, tn, (fn + 2715648 - 7) % 2715648, chan,
		tch_data, rc);
}

static int rx_tchh_fn(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, sbit_t *bits, float toa)
{
	LOGP(DL1C, LOGL_DEBUG, "TCH/H Received %s fn=%u ts=%u trx=%u bid=%u\n", 
		trx_chan_desc[chan].name, fn, tn, l1h->trx->nr, bid);

	// FIXME
	return 0;
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
	int downlink, int active)
{
	uint8_t tn = L1SAP_CHAN2TS(chan_nr);
	int i;
	int rc = -EINVAL;

	/* look for all matching chan_nr/link_id */
	for (i = 0; i < _TRX_CHAN_MAX; i++) {
		/* skip if pchan type does not match pdch flag */
		if ((trx_sched_multiframes[l1h->mf_index[tn]].pchan
							== GSM_PCHAN_PDCH)
						!= trx_chan_desc[i].pdch)
			continue;
		if (trx_chan_desc[i].chan_nr == (chan_nr & 0xf8)
		 && trx_chan_desc[i].link_id == link_id) {
			LOGP(DL1C, LOGL_NOTICE, "%s %s %s on trx=%d ts=%d\n",
				(active) ? "Activating" : "Deactivating",
				(downlink) ? "downlink" : "uplink",
				trx_chan_desc[i].name, l1h->trx->nr, tn);
		 	if (downlink) {
				l1h->chan_states[tn][i].dl_active = active;
				l1h->chan_states[tn][i].dl_active = active;
			} else {
				l1h->chan_states[tn][i].ul_active = active;
				l1h->chan_states[tn][i].ul_active = active;
			}
			l1h->chan_states[tn][i].lost = 0;
			rc = 0;
		}
	}

	return rc;
}

/* setting all logical channels given attributes to active/inactive */
int trx_sched_set_mode(struct trx_l1h *l1h, uint8_t chan_nr, uint8_t rsl_cmode,
	uint8_t tch_mode)
{
	uint8_t tn = L1SAP_CHAN2TS(chan_nr);
	int i;
	int rc = -EINVAL;

	/* look for all matching chan_nr/link_id */
	for (i = 0; i < _TRX_CHAN_MAX; i++) {
		if (trx_chan_desc[i].chan_nr == (chan_nr & 0xf8)
		 && trx_chan_desc[i].link_id == 0x00) {
			LOGP(DL1C, LOGL_NOTICE, "Set mode %u, %u on "
				"%s of trx=%d ts=%d\n", rsl_cmode, tch_mode,
				trx_chan_desc[i].name, l1h->trx->nr, tn);
			l1h->chan_states[tn][i].rsl_cmode = rsl_cmode;
			l1h->chan_states[tn][i].tch_mode = tch_mode;
			rc = 0;
		}
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

	if (algo < 0 || key_len > 8 || (algo && key_len != 8)) {
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
	 && !l1h->chan_states[tn][chan].dl_active)
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
	 && !l1h->chan_states[tn][chan].dl_active)
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
		 && !l1h->chan_states[tn][chan].ul_active)
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

			func(l1h, tn, fn, chan, bid, bits, toa);
		} else {
			sbit_t spare[148];

			memset(spare, 0, 148);
			func(l1h, tn, fn, chan, bid, spare, toa);
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

	/* advance frame number, so the tranceiver has more time until
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
	struct timeval tv_now, *tv_clock = &tranceiver_clock_tv;
	int32_t elapsed;

	/* check if tranceiver is still alive */
	if (tranceiver_lost++ == TRX_LOSS_FRAMES) {
		struct gsm_bts_trx *trx;

		LOGP(DL1C, LOGL_NOTICE, "No more clock from traneiver\n");

no_clock:
		tranceiver_available = 0;

		/* flush pending messages of transceiver */
		/* close all logical channels and reset timeslots */
		llist_for_each_entry(trx, &bts->trx_list, list) {
			trx_if_flush(trx_l1h_hdl(trx));
			trx_sched_reset(trx_l1h_hdl(trx));
			if (trx->nr == 0)
				trx_if_cmd_poweroff(trx_l1h_hdl(trx));
		}

		/* tell BSC */
		check_tranceiver_availability(bts, 0);

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
		tranceiver_last_fn = (tranceiver_last_fn + 1) % 2715648;
		trx_sched_fn(tranceiver_last_fn);
		elapsed -= FRAME_DURATION_uS;
	}
	osmo_timer_schedule(&tranceiver_clock_timer, 0,
		FRAME_DURATION_uS - elapsed);
}


/* receive clock from tranceiver */
int trx_sched_clock(uint32_t fn)
{
	struct timeval tv_now, *tv_clock = &tranceiver_clock_tv;
	int32_t elapsed;
	int32_t elapsed_fn;

	if (quit)
		return 0;

	/* reset lost counter */
	tranceiver_lost = 0;

	gettimeofday(&tv_now, NULL);

	/* clock becomes valid */
	if (!tranceiver_available) {
		LOGP(DL1C, LOGL_NOTICE, "initial GSM clock received: fn=%u\n",
			fn);

		tranceiver_available = 1;

		/* start provisioning tranceiver */
		l1if_provision_tranceiver(bts);

		/* tell BSC */
		check_tranceiver_availability(bts, 1);

new_clock:
		tranceiver_last_fn = fn;
		trx_sched_fn(tranceiver_last_fn);

		/* schedule first FN clock */
		memcpy(tv_clock, &tv_now, sizeof(struct timeval));
		memset(&tranceiver_clock_timer, 0,
			sizeof(tranceiver_clock_timer));
		tranceiver_clock_timer.cb = trx_ctrl_timer_cb;
	        tranceiver_clock_timer.data = bts;
		osmo_timer_schedule(&tranceiver_clock_timer, 0,
			FRAME_DURATION_uS);

		return 0;
	}

	osmo_timer_del(&tranceiver_clock_timer);

	/* calculate elapsed time since last_fn */
	elapsed = (tv_now.tv_sec - tv_clock->tv_sec) * 1000000
		+ (tv_now.tv_usec - tv_clock->tv_usec);

	/* how much frames have been elapsed since last fn processed */
	elapsed_fn = (fn + 2715648 - tranceiver_last_fn) % 2715648;
	if (elapsed_fn >= 135774)
		elapsed_fn -= 2715648;

	/* check for max clock skew */
	if (elapsed_fn > MAX_FN_SKEW || elapsed_fn < -MAX_FN_SKEW) {
		LOGP(DL1C, LOGL_NOTICE, "GSM clock skew: old fn=%u, "
			"new fn=%u\n", tranceiver_last_fn, fn);
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
		osmo_timer_schedule(&tranceiver_clock_timer, 0,
			FRAME_DURATION_uS * (1 - elapsed_fn));

		return 0;
	}

	/* transmit what we still need to transmit */
	while (fn != tranceiver_last_fn) {
		tranceiver_last_fn = (tranceiver_last_fn + 1) % 2715648;
		trx_sched_fn(tranceiver_last_fn);
	}

	/* schedule next FN to be transmitted */
	memcpy(tv_clock, &tv_now, sizeof(struct timeval));
	osmo_timer_schedule(&tranceiver_clock_timer, 0, FRAME_DURATION_uS);

	return 0;
}

