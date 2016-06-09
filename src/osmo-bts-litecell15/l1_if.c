/* Interface handler for NuRAN Wireless Litecell 1.5 L1 */

/* Copyright (C) 2015 by Yves Godin <support@nuranwireless.com>
 * Copyright (C) 2016 by Harald Welte <laforge@gnumonks.org>
 * 
 * Based on sysmoBTS:
 *     (C) 2011-2014 by Harald Welte <laforge@gnumonks.org>
 *     (C) 2014 by Holger Hans Peter Freyther
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
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>

#include <sys/types.h>
#include <sys/stat.h>

#include <osmocom/core/talloc.h>
#include <osmocom/core/utils.h>
#include <osmocom/core/select.h>
#include <osmocom/core/timer.h>
#include <osmocom/core/write_queue.h>
#include <osmocom/gsm/gsm_utils.h>
#include <osmocom/gsm/lapdm.h>

#include <osmo-bts/logging.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/oml.h>
#include <osmo-bts/rsl.h>
#include <osmo-bts/gsm_data.h>
#include <osmo-bts/phy_link.h>
#include <osmo-bts/paging.h>
#include <osmo-bts/measurement.h>
#include <osmo-bts/pcu_if.h>
#include <osmo-bts/handover.h>
#include <osmo-bts/cbch.h>
#include <osmo-bts/bts_model.h>
#include <osmo-bts/l1sap.h>

#include <nrw/litecell15/litecell15.h>
#include <nrw/litecell15/gsml1prim.h>
#include <nrw/litecell15/gsml1const.h>
#include <nrw/litecell15/gsml1types.h>

#include "lc15bts.h"
#include "l1_if.h"
#include "l1_transp.h"
#include "hw_misc.h"
#include "misc/lc15bts_par.h"
#include "misc/lc15bts_bid.h"
#include "utils.h"

struct wait_l1_conf {
	struct llist_head list;		/* internal linked list */
	struct osmo_timer_list timer;	/* timer for L1 timeout */
	unsigned int conf_prim_id;	/* primitive we expect in response */
	unsigned int is_sys_prim;	/* is this a system (1) or L1 (0) primitive */
	l1if_compl_cb *cb;
	void *cb_data;
};

static void release_wlc(struct wait_l1_conf *wlc)
{
	osmo_timer_del(&wlc->timer);
	talloc_free(wlc);
}

static void l1if_req_timeout(void *data)
{
	struct wait_l1_conf *wlc = data;

	if (wlc->is_sys_prim)
		LOGP(DL1C, LOGL_FATAL, "Timeout waiting for SYS primitive %s\n",
			get_value_string(lc15bts_sysprim_names, wlc->conf_prim_id));
	else
		LOGP(DL1C, LOGL_FATAL, "Timeout waiting for L1 primitive %s\n",
			get_value_string(lc15bts_l1prim_names, wlc->conf_prim_id));
	exit(23);
}

static int _l1if_req_compl(struct lc15l1_hdl *fl1h, struct msgb *msg,
		   int is_system_prim, l1if_compl_cb *cb, void *data)
{
	struct wait_l1_conf *wlc;
	struct osmo_wqueue *wqueue;
	unsigned int timeout_secs;

	/* allocate new wsc and store reference to mutex and conf_id */
	wlc = talloc_zero(fl1h, struct wait_l1_conf);
	wlc->cb = cb;
	wlc->cb_data = data;

	/* Make sure we actually have received a REQUEST type primitive */
	if (is_system_prim == 0) {
		GsmL1_Prim_t *l1p = msgb_l1prim(msg);

		LOGP(DL1P, LOGL_INFO, "Tx L1 prim %s\n",
			get_value_string(lc15bts_l1prim_names, l1p->id));

		if (lc15bts_get_l1prim_type(l1p->id) != L1P_T_REQ) {
			LOGP(DL1C, LOGL_ERROR, "L1 Prim %s is not a Request!\n",
				get_value_string(lc15bts_l1prim_names, l1p->id));
			talloc_free(wlc);
			return -EINVAL;
		}
		wlc->is_sys_prim = 0;
		wlc->conf_prim_id = lc15bts_get_l1prim_conf(l1p->id);
		wqueue = &fl1h->write_q[MQ_L1_WRITE];
		timeout_secs = 30;
	} else {
		Litecell15_Prim_t *sysp = msgb_sysprim(msg);

		LOGP(DL1C, LOGL_INFO, "Tx SYS prim %s\n",
			get_value_string(lc15bts_sysprim_names, sysp->id));

		if (lc15bts_get_sysprim_type(sysp->id) != L1P_T_REQ) {
			LOGP(DL1C, LOGL_ERROR, "SYS Prim %s is not a Request!\n",
				get_value_string(lc15bts_sysprim_names, sysp->id));
			talloc_free(wlc);
			return -EINVAL;
		}
		wlc->is_sys_prim = 1;
		wlc->conf_prim_id = lc15bts_get_sysprim_conf(sysp->id);
		wqueue = &fl1h->write_q[MQ_SYS_WRITE];
		timeout_secs = 30;
	}

	/* enqueue the message in the queue and add wsc to list */
	if (osmo_wqueue_enqueue(wqueue, msg) != 0) {
		/* So we will get a timeout but the log message might help */
		LOGP(DL1C, LOGL_ERROR, "Write queue for %s full. dropping msg.\n",
			is_system_prim ? "system primitive" : "gsm");
		msgb_free(msg);
	}
	llist_add(&wlc->list, &fl1h->wlc_list);

	/* schedule a timer for timeout_secs seconds. If DSP fails to respond, we terminate */
	wlc->timer.data = wlc;
	wlc->timer.cb = l1if_req_timeout;
	osmo_timer_schedule(&wlc->timer, timeout_secs, 0);

	return 0;
}

/* send a request primitive to the L1 and schedule completion call-back */
int l1if_req_compl(struct lc15l1_hdl *fl1h, struct msgb *msg,
		   l1if_compl_cb *cb, void *data)
{
	return _l1if_req_compl(fl1h, msg, 1, cb, data);
}

int l1if_gsm_req_compl(struct lc15l1_hdl *fl1h, struct msgb *msg,
		   l1if_compl_cb *cb, void *data)
{
	return _l1if_req_compl(fl1h, msg, 0, cb, data);
}

/* allocate a msgb containing a GsmL1_Prim_t */
struct msgb *l1p_msgb_alloc(void)
{
	struct msgb *msg = msgb_alloc(sizeof(GsmL1_Prim_t), "l1_prim");

	if (msg)
		msg->l1h = msgb_put(msg, sizeof(GsmL1_Prim_t));

	return msg;
}

/* allocate a msgb containing a Litecell15_Prim_t */
struct msgb *sysp_msgb_alloc(void)
{
	struct msgb *msg = msgb_alloc(sizeof(Litecell15_Prim_t), "sys_prim");

	if (msg)
		msg->l1h = msgb_put(msg, sizeof(Litecell15_Prim_t));

	return msg;
}

static GsmL1_PhDataReq_t *
data_req_from_rts_ind(GsmL1_Prim_t *l1p,
		const GsmL1_PhReadyToSendInd_t *rts_ind)
{
	GsmL1_PhDataReq_t *data_req = &l1p->u.phDataReq;

	l1p->id = GsmL1_PrimId_PhDataReq;

	/* copy fields from PH-RSS.ind */
	data_req->hLayer1	= rts_ind->hLayer1;
	data_req->u8Tn 		= rts_ind->u8Tn;
	data_req->u32Fn		= rts_ind->u32Fn;
	data_req->sapi		= rts_ind->sapi;
	data_req->subCh		= rts_ind->subCh;
	data_req->u8BlockNbr	= rts_ind->u8BlockNbr;

	return data_req;
}

static GsmL1_PhEmptyFrameReq_t *
empty_req_from_rts_ind(GsmL1_Prim_t *l1p,
			const GsmL1_PhReadyToSendInd_t *rts_ind)
{
	GsmL1_PhEmptyFrameReq_t *empty_req = &l1p->u.phEmptyFrameReq;

	l1p->id = GsmL1_PrimId_PhEmptyFrameReq;

	empty_req->hLayer1 = rts_ind->hLayer1;
	empty_req->u8Tn = rts_ind->u8Tn;
	empty_req->u32Fn = rts_ind->u32Fn;
	empty_req->sapi = rts_ind->sapi;
	empty_req->subCh = rts_ind->subCh;
	empty_req->u8BlockNbr = rts_ind->u8BlockNbr;

	return empty_req;
}

static const uint8_t fill_frame[GSM_MACBLOCK_LEN] = {
	0x03, 0x03, 0x01, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B,
	0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B,
	0x2B, 0x2B, 0x2B
};

/* fill PH-DATA.req from l1sap primitive */
static GsmL1_PhDataReq_t *
data_req_from_l1sap(GsmL1_Prim_t *l1p, struct lc15l1_hdl *fl1,
		uint8_t tn, uint32_t fn, uint8_t sapi, uint8_t sub_ch,
		uint8_t block_nr, uint8_t len)
{
	GsmL1_PhDataReq_t *data_req = &l1p->u.phDataReq;

	l1p->id = GsmL1_PrimId_PhDataReq;

	/* copy fields from PH-RSS.ind */
	data_req->hLayer1	= (HANDLE)fl1->hLayer1;
	data_req->u8Tn 		= tn;
	data_req->u32Fn		= fn;
	data_req->sapi		= sapi;
	data_req->subCh		= sub_ch;
	data_req->u8BlockNbr	= block_nr;

	data_req->msgUnitParam.u8Size = len;

	return data_req;
}

/* fill PH-EMPTY_FRAME.req from l1sap primitive */
static GsmL1_PhEmptyFrameReq_t *
empty_req_from_l1sap(GsmL1_Prim_t *l1p, struct lc15l1_hdl *fl1,
		     uint8_t tn, uint32_t fn, uint8_t sapi,
		     uint8_t subch, uint8_t block_nr)
{
	GsmL1_PhEmptyFrameReq_t *empty_req = &l1p->u.phEmptyFrameReq;

	l1p->id = GsmL1_PrimId_PhEmptyFrameReq;

	empty_req->hLayer1 = (HANDLE)fl1->hLayer1;
	empty_req->u8Tn = tn;
	empty_req->u32Fn = fn;
	empty_req->sapi = sapi;
	empty_req->subCh = subch;
	empty_req->u8BlockNbr = block_nr;

	return empty_req;
}

static int ph_data_req(struct gsm_bts_trx *trx, struct msgb *msg,
		       struct osmo_phsap_prim *l1sap)
{
	struct lc15l1_hdl *fl1 = trx_lc15l1_hdl(trx);
	struct msgb *l1msg = l1p_msgb_alloc();
	uint32_t u32Fn;
	uint8_t u8Tn, subCh, u8BlockNbr = 0, sapi = 0;
	uint8_t chan_nr, link_id;
	int len;

	if (!msg) {
		LOGP(DL1C, LOGL_FATAL, "PH-DATA.req without msg. "
			"Please fix!\n");
		abort();
	}

	len = msgb_l2len(msg);

	chan_nr = l1sap->u.data.chan_nr;
	link_id = l1sap->u.data.link_id;
	u32Fn = l1sap->u.data.fn;
	u8Tn = L1SAP_CHAN2TS(chan_nr);
	subCh = 0x1f;
	if (L1SAP_IS_LINK_SACCH(link_id)) {
		sapi = GsmL1_Sapi_Sacch;
		if (!L1SAP_IS_CHAN_TCHF(chan_nr))
			subCh = l1sap_chan2ss(chan_nr);
	} else if (L1SAP_IS_CHAN_TCHF(chan_nr)) {
		if (trx->ts[u8Tn].pchan == GSM_PCHAN_PDCH) {
			if (L1SAP_IS_PTCCH(u32Fn)) {
				sapi = GsmL1_Sapi_Ptcch;
				u8BlockNbr = L1SAP_FN2PTCCHBLOCK(u32Fn);
			} else {
				sapi = GsmL1_Sapi_Pdtch;
				u8BlockNbr = L1SAP_FN2MACBLOCK(u32Fn);
			}
		} else {
			sapi = GsmL1_Sapi_FacchF;
			u8BlockNbr = (u32Fn % 13) >> 2;
		}
	} else if (L1SAP_IS_CHAN_TCHH(chan_nr)) {
		subCh = L1SAP_CHAN2SS_TCHH(chan_nr);
		sapi = GsmL1_Sapi_FacchH;
		u8BlockNbr = (u32Fn % 26) >> 3;
	} else if (L1SAP_IS_CHAN_SDCCH4(chan_nr)) {
		subCh = L1SAP_CHAN2SS_SDCCH4(chan_nr);
		sapi = GsmL1_Sapi_Sdcch;
	} else if (L1SAP_IS_CHAN_SDCCH8(chan_nr)) {
		subCh = L1SAP_CHAN2SS_SDCCH8(chan_nr);
		sapi = GsmL1_Sapi_Sdcch;
	} else if (L1SAP_IS_CHAN_BCCH(chan_nr)) {
		sapi = GsmL1_Sapi_Bcch;
	} else if (L1SAP_IS_CHAN_AGCH_PCH(chan_nr)) {
		/* The sapi depends on DSP configuration, not
		 * on the actual SYSTEM INFORMATION 3. */
		u8BlockNbr = L1SAP_FN2CCCHBLOCK(u32Fn);
		if (u8BlockNbr >= 1)
			sapi = GsmL1_Sapi_Pch;
		else
			sapi = GsmL1_Sapi_Agch;
	} else {
		LOGP(DL1C, LOGL_NOTICE, "unknown prim %d op %d "
			"chan_nr %d link_id %d\n", l1sap->oph.primitive,
			l1sap->oph.operation, chan_nr, link_id);
		return -EINVAL;
	}

	/* convert l1sap message to GsmL1 primitive, keep payload */
	if (len) {
		/* data request */
		GsmL1_Prim_t *l1p = msgb_l1prim(l1msg);

		data_req_from_l1sap(l1p, fl1, u8Tn, u32Fn, sapi, subCh, u8BlockNbr, len);

		OSMO_ASSERT(msgb_l2len(msg) <= sizeof(l1p->u.phDataReq.msgUnitParam.u8Buffer));
		memcpy(l1p->u.phDataReq.msgUnitParam.u8Buffer, msg->l2h, msgb_l2len(msg));
		LOGP(DL1P, LOGL_DEBUG, "PH-DATA.req(%s)\n",
			osmo_hexdump(l1p->u.phDataReq.msgUnitParam.u8Buffer,
				     l1p->u.phDataReq.msgUnitParam.u8Size));
	} else {
		/* empty frame */
		GsmL1_Prim_t *l1p = msgb_l1prim(l1msg);

		empty_req_from_l1sap(l1p, fl1, u8Tn, u32Fn, sapi, subCh, u8BlockNbr);
	}

	/* free the msgb holding the L1SAP primitive */
	msgb_free(msg);

	/* send message to DSP's queue */
	if (osmo_wqueue_enqueue(&fl1->write_q[MQ_L1_WRITE], l1msg) != 0) {
		LOGP(DL1P, LOGL_ERROR, "MQ_L1_WRITE queue full. Dropping msg.\n");
		msgb_free(msg);
	}

	return 0;
}

static int ph_tch_req(struct gsm_bts_trx *trx, struct msgb *msg,
		       struct osmo_phsap_prim *l1sap)
{
	struct lc15l1_hdl *fl1 = trx_lc15l1_hdl(trx);
	struct gsm_lchan *lchan;
	uint32_t u32Fn;
	uint8_t u8Tn, subCh, u8BlockNbr = 0, sapi, ss;
	uint8_t chan_nr;
	GsmL1_Prim_t *l1p;
	struct msgb *nmsg = NULL;

	chan_nr = l1sap->u.tch.chan_nr;
	u32Fn = l1sap->u.tch.fn;
	u8Tn = L1SAP_CHAN2TS(chan_nr);
	u8BlockNbr = (u32Fn % 13) >> 2;
	if (L1SAP_IS_CHAN_TCHH(chan_nr)) {
		ss = subCh = L1SAP_CHAN2SS_TCHH(chan_nr);
		sapi = GsmL1_Sapi_TchH;
	} else {
		subCh = 0x1f;
		ss = 0;
		sapi = GsmL1_Sapi_TchF;
	}

	lchan = &trx->ts[u8Tn].lchan[ss];

	/* create new message and fill data */
	if (msg) {
		msgb_pull(msg, sizeof(*l1sap));
		/* create new message */
		nmsg = l1p_msgb_alloc();
		if (!nmsg)
			return -ENOMEM;
		l1p = msgb_l1prim(nmsg);
		l1if_tch_encode(lchan,
			l1p->u.phDataReq.msgUnitParam.u8Buffer,
			&l1p->u.phDataReq.msgUnitParam.u8Size,
			msg->data, msg->len);
	}

	/* no message/data, we generate an empty traffic msg */
	if (!nmsg)
		nmsg = gen_empty_tch_msg(lchan);

	/* no traffic message, we generate an empty msg */
	if (!nmsg) {
		nmsg = l1p_msgb_alloc();
		if (!nmsg)
			return -ENOMEM;
	}

	l1p = msgb_l1prim(nmsg);

	/* if we provide data, or if data is already in nmsg */
	if (l1p->u.phDataReq.msgUnitParam.u8Size) {
		/* data request */
		data_req_from_l1sap(l1p, fl1, u8Tn, u32Fn, sapi, subCh,
				    u8BlockNbr,
				    l1p->u.phDataReq.msgUnitParam.u8Size);
	} else {
		/* empty frame */
		if (trx->bts->dtxd && trx != trx->bts->c0)
			lchan->tch.dtxd_active = true;
		empty_req_from_l1sap(l1p, fl1, u8Tn, u32Fn, sapi, subCh, u8BlockNbr);
	}
	/* send message to DSP's queue */
	osmo_wqueue_enqueue(&fl1->write_q[MQ_L1_WRITE], nmsg);

	msgb_free(msg);
	return 0;
}

static int mph_info_req(struct gsm_bts_trx *trx, struct msgb *msg,
		        struct osmo_phsap_prim *l1sap)
{
	struct lc15l1_hdl *fl1 = trx_lc15l1_hdl(trx);
	uint8_t u8Tn, ss;
	uint8_t chan_nr;
	struct gsm_lchan *lchan;
	int rc = 0;

	switch (l1sap->u.info.type) {
	case PRIM_INFO_ACT_CIPH:
		chan_nr = l1sap->u.info.u.ciph_req.chan_nr;
		u8Tn = L1SAP_CHAN2TS(chan_nr);
		ss = l1sap_chan2ss(chan_nr);
		lchan = &trx->ts[u8Tn].lchan[ss];
		if (l1sap->u.info.u.ciph_req.uplink) {
			l1if_set_ciphering(fl1, lchan, 0);
			lchan->ciph_state = LCHAN_CIPH_RX_REQ;
		}
		if (l1sap->u.info.u.ciph_req.downlink) {
			l1if_set_ciphering(fl1, lchan, 1);
			lchan->ciph_state = LCHAN_CIPH_RX_CONF_TX_REQ;
		}
		if (l1sap->u.info.u.ciph_req.downlink
		 && l1sap->u.info.u.ciph_req.uplink)
			lchan->ciph_state = LCHAN_CIPH_RXTX_REQ;
		break;
	case PRIM_INFO_ACTIVATE:
	case PRIM_INFO_DEACTIVATE:
	case PRIM_INFO_MODIFY:
		chan_nr = l1sap->u.info.u.act_req.chan_nr;
		u8Tn = L1SAP_CHAN2TS(chan_nr);
		ss = l1sap_chan2ss(chan_nr);
		lchan = &trx->ts[u8Tn].lchan[ss];
		if (l1sap->u.info.type == PRIM_INFO_ACTIVATE)
			l1if_rsl_chan_act(lchan);
		else if (l1sap->u.info.type == PRIM_INFO_MODIFY) {
			if (lchan->ho.active == HANDOVER_WAIT_FRAME)
				l1if_rsl_chan_mod(lchan);
			else
				l1if_rsl_mode_modify(lchan);
		} else if (l1sap->u.info.u.act_req.sacch_only)
			l1if_rsl_deact_sacch(lchan);
		else
			l1if_rsl_chan_rel(lchan);
		msgb_free(msg);
		break;
	default:
		LOGP(DL1C, LOGL_NOTICE, "unknown MPH-INFO.req %d\n",
			l1sap->u.info.type);
		rc = -EINVAL;
	}

	return rc;
}

/* primitive from common part */
int bts_model_l1sap_down(struct gsm_bts_trx *trx, struct osmo_phsap_prim *l1sap)
{
	struct msgb *msg = l1sap->oph.msg;
	int rc = 0;

	switch (OSMO_PRIM_HDR(&l1sap->oph)) {
	case OSMO_PRIM(PRIM_PH_DATA, PRIM_OP_REQUEST):
		rc = ph_data_req(trx, msg, l1sap);
		break;
	case OSMO_PRIM(PRIM_TCH, PRIM_OP_REQUEST):
		rc = ph_tch_req(trx, msg, l1sap);
		break;
	case OSMO_PRIM(PRIM_MPH_INFO, PRIM_OP_REQUEST):
		rc = mph_info_req(trx, msg, l1sap);
		break;
	default:
		LOGP(DL1C, LOGL_NOTICE, "unknown prim %d op %d\n",
			l1sap->oph.primitive, l1sap->oph.operation);
		rc = -EINVAL;
	}

	if (rc)
		msgb_free(msg);
	return rc;
}

static int handle_mph_time_ind(struct lc15l1_hdl *fl1,
				GsmL1_MphTimeInd_t *time_ind)
{
	struct gsm_bts_trx *trx = lc15l1_hdl_trx(fl1);
	struct gsm_bts *bts = trx->bts;
	struct osmo_phsap_prim l1sap;
	uint32_t fn;

	/* increment the primitive count for the alive timer */
	fl1->alive_prim_cnt++;

	/* ignore every time indication, except for c0 */
	if (trx != bts->c0) {
		return 0;
	}

	fn = time_ind->u32Fn;

	memset(&l1sap, 0, sizeof(l1sap));
	osmo_prim_init(&l1sap.oph, SAP_GSM_PH, PRIM_MPH_INFO,
		PRIM_OP_INDICATION, NULL);
	l1sap.u.info.type = PRIM_INFO_TIME;
	l1sap.u.info.u.time_ind.fn = fn;

	return l1sap_up(trx, &l1sap);
}

static uint8_t chan_nr_by_sapi(enum gsm_phys_chan_config pchan,
			       GsmL1_Sapi_t sapi, GsmL1_SubCh_t subCh,
			       uint8_t u8Tn, uint32_t u32Fn)
{
	uint8_t cbits = 0;
	switch (sapi) {
	case GsmL1_Sapi_Bcch:
		cbits = 0x10;
		break;
	case GsmL1_Sapi_Sacch:
		switch(pchan) {
		case GSM_PCHAN_TCH_F:
			cbits = 0x01;
			break;
		case GSM_PCHAN_TCH_H:
			cbits = 0x02 + subCh;
			break;
		case GSM_PCHAN_CCCH_SDCCH4:
			cbits = 0x04 + subCh;
			break;
		case GSM_PCHAN_SDCCH8_SACCH8C:
			cbits = 0x08 + subCh;
			break;
		default:
			LOGP(DL1C, LOGL_ERROR, "SACCH for pchan %d?\n",
				pchan);
			return 0;
		}
		break;
	case GsmL1_Sapi_Sdcch:
		switch(pchan) {
		case GSM_PCHAN_CCCH_SDCCH4:
			cbits = 0x04 + subCh;
			break;
		case GSM_PCHAN_SDCCH8_SACCH8C:
			cbits = 0x08 + subCh;
			break;
		default:
			LOGP(DL1C, LOGL_ERROR, "SDCCH for pchan %d?\n",
				pchan);
			return 0;
		}
		break;
	case GsmL1_Sapi_Agch:
	case GsmL1_Sapi_Pch:
		cbits = 0x12;
		break;
	case GsmL1_Sapi_Pdtch:
	case GsmL1_Sapi_Pacch:
		switch(pchan) {
		case GSM_PCHAN_PDCH:
			cbits = 0x01;
			break;
		default:
			LOGP(DL1C, LOGL_ERROR, "PDTCH for pchan %d?\n",
				pchan);
			return 0;
		}
		break;
	case GsmL1_Sapi_TchF:
		cbits = 0x01;
		break;
	case GsmL1_Sapi_TchH:
		cbits = 0x02 + subCh;
		break;
	case GsmL1_Sapi_FacchF:
		cbits = 0x01;
		break;
	case GsmL1_Sapi_FacchH:
		cbits = 0x02 + subCh;
		break;
	case GsmL1_Sapi_Ptcch:
		if (!L1SAP_IS_PTCCH(u32Fn)) {
			LOGP(DL1C, LOGL_FATAL, "Not expecting PTCCH at frame "
				"number other than 12, got it at %u (%u). "
				"Please fix!\n", u32Fn % 52, u32Fn);
			abort();
		}
		switch(pchan) {
		case GSM_PCHAN_PDCH:
			cbits = 0x01;
			break;
		default:
			LOGP(DL1C, LOGL_ERROR, "PTCCH for pchan %d?\n",
				pchan);
			return 0;
		}
		break;
	default:
		return 0;
	}

	/* not reached due to default case above */
	return (cbits << 3) | u8Tn;
}

static int handle_ph_readytosend_ind(struct lc15l1_hdl *fl1,
				     GsmL1_PhReadyToSendInd_t *rts_ind,
				     struct msgb *l1p_msg)
{
	struct gsm_bts_trx *trx = lc15l1_hdl_trx(fl1);
	struct gsm_bts *bts = trx->bts;
	struct msgb *resp_msg;
	GsmL1_PhDataReq_t *data_req;
	GsmL1_MsgUnitParam_t *msu_param;
	struct gsm_time g_time;
	uint32_t t3p;
	int rc;
	struct osmo_phsap_prim *l1sap;
	uint8_t chan_nr, link_id;
	uint32_t fn;

	/* check if primitive should be handled by common part */
	chan_nr = chan_nr_by_sapi(trx->ts[rts_ind->u8Tn].pchan, rts_ind->sapi,
		rts_ind->subCh, rts_ind->u8Tn, rts_ind->u32Fn);
	if (chan_nr) {
		fn = rts_ind->u32Fn;
		if (rts_ind->sapi == GsmL1_Sapi_Sacch)
			link_id = 0x40;
		else
			link_id = 0;
		rc = msgb_trim(l1p_msg, sizeof(*l1sap));
		if (rc < 0)
			MSGB_ABORT(l1p_msg, "No room for primitive\n");
		l1sap = msgb_l1sap_prim(l1p_msg);
		if (rts_ind->sapi == GsmL1_Sapi_TchF
		 || rts_ind->sapi == GsmL1_Sapi_TchH) {
			osmo_prim_init(&l1sap->oph, SAP_GSM_PH, PRIM_TCH_RTS,
				PRIM_OP_INDICATION, l1p_msg);
			l1sap->u.tch.chan_nr = chan_nr;
			l1sap->u.tch.fn = fn;
		} else {
			osmo_prim_init(&l1sap->oph, SAP_GSM_PH, PRIM_PH_RTS,
				PRIM_OP_INDICATION, l1p_msg);
			l1sap->u.data.link_id = link_id;
			l1sap->u.data.chan_nr = chan_nr;
			l1sap->u.data.fn = fn;
		}

		return l1sap_up(trx, l1sap);
	}

	gsm_fn2gsmtime(&g_time, rts_ind->u32Fn);

	DEBUGP(DL1P, "Rx PH-RTS.ind %02u/%02u/%02u SAPI=%s\n",
		g_time.t1, g_time.t2, g_time.t3,
		get_value_string(lc15bts_l1sapi_names, rts_ind->sapi));

	/* in all other cases, we need to allocate a new PH-DATA.ind
	 * primitive msgb and start to fill it */
	resp_msg = l1p_msgb_alloc();
	data_req = data_req_from_rts_ind(msgb_l1prim(resp_msg), rts_ind);
	msu_param = &data_req->msgUnitParam;

	/* set default size */
	msu_param->u8Size = GSM_MACBLOCK_LEN;

	switch (rts_ind->sapi) {
	case GsmL1_Sapi_Sch:
		/* compute T3prime */
		t3p = (g_time.t3 - 1) / 10;
		/* fill SCH burst with data */
		msu_param->u8Size = 4;
		msu_param->u8Buffer[0] = (bts->bsic << 2) | (g_time.t1 >> 9);
		msu_param->u8Buffer[1] = (g_time.t1 >> 1);
		msu_param->u8Buffer[2] = (g_time.t1 << 7) | (g_time.t2 << 2) | (t3p >> 1);
		msu_param->u8Buffer[3] = (t3p & 1);
		break;
	case GsmL1_Sapi_Prach:
		goto empty_frame;
		break;
	case GsmL1_Sapi_Cbch:
		/* get them from bts->si_buf[] */
		bts_cbch_get(bts, msu_param->u8Buffer, &g_time);
		break;
	default:
		memcpy(msu_param->u8Buffer, fill_frame, GSM_MACBLOCK_LEN);
		break;
	}
tx:

	/* transmit */
	if (osmo_wqueue_enqueue(&fl1->write_q[MQ_L1_WRITE], resp_msg) != 0) {
		LOGP(DL1C, LOGL_ERROR, "MQ_L1_WRITE queue full. Dropping msg.\n");
		msgb_free(resp_msg);
	}

	msgb_free(l1p_msg);
	return 0;

empty_frame:
	/* in case we decide to send an empty frame... */
	empty_req_from_rts_ind(msgb_l1prim(resp_msg), rts_ind);

	goto tx;
}

static void dump_meas_res(int ll, GsmL1_MeasParam_t *m)
{
	LOGPC(DL1C, ll, ", Meas: RSSI %-3.2f dBm,  Qual %-3.2f dB,  "
		"BER %-3.2f,  Timing %d\n", m->fRssi, m->fLinkQuality,
		m->fBer, m->i16BurstTiming);
}

static int process_meas_res(struct gsm_bts_trx *trx, uint8_t chan_nr,
				GsmL1_MeasParam_t *m)
{
	struct osmo_phsap_prim l1sap;
	memset(&l1sap, 0, sizeof(l1sap));
	osmo_prim_init(&l1sap.oph, SAP_GSM_PH, PRIM_MPH_INFO,
		PRIM_OP_INDICATION, NULL);
	l1sap.u.info.type = PRIM_INFO_MEAS;
	l1sap.u.info.u.meas_ind.chan_nr = chan_nr;
	l1sap.u.info.u.meas_ind.ta_offs_qbits = m->i16BurstTiming;
	l1sap.u.info.u.meas_ind.ber10k = (unsigned int) (m->fBer * 100);
	l1sap.u.info.u.meas_ind.inv_rssi = (uint8_t) (m->fRssi * -1);

	return l1sap_up(trx, &l1sap);
}

static int handle_ph_data_ind(struct lc15l1_hdl *fl1, GsmL1_PhDataInd_t *data_ind,
			      struct msgb *l1p_msg)
{
	struct gsm_bts_trx *trx = lc15l1_hdl_trx(fl1);
	struct gsm_bts_role_bts *btsb = bts_role_bts(trx->bts);
	uint8_t chan_nr, link_id;
	struct osmo_phsap_prim *l1sap;
	uint32_t fn;
	uint8_t *data, len;
	int rc = 0;
	int8_t rssi;

	chan_nr = chan_nr_by_sapi(trx->ts[data_ind->u8Tn].pchan, data_ind->sapi,
		data_ind->subCh, data_ind->u8Tn, data_ind->u32Fn);
	if (!chan_nr) {
		LOGP(DL1C, LOGL_ERROR, "PH-DATA-INDICATION for unknown sapi "
			"%d\n", data_ind->sapi);
		msgb_free(l1p_msg);
		return ENOTSUP;
	}
	fn = data_ind->u32Fn;
	link_id =  (data_ind->sapi == GsmL1_Sapi_Sacch) ? 0x40 : 0x00;

	process_meas_res(trx, chan_nr, &data_ind->measParam);

	if (data_ind->measParam.fLinkQuality < btsb->min_qual_norm
	 && data_ind->msgUnitParam.u8Size != 0) {
		msgb_free(l1p_msg);
		return 0;
	}

	DEBUGP(DL1C, "Rx PH-DATA.ind %s (hL2 %08x): %s",
		get_value_string(lc15bts_l1sapi_names, data_ind->sapi),
		(uint32_t)data_ind->hLayer2,
		osmo_hexdump(data_ind->msgUnitParam.u8Buffer,
			     data_ind->msgUnitParam.u8Size));
	dump_meas_res(LOGL_DEBUG, &data_ind->measParam);

	/* check for TCH */
	if (data_ind->sapi == GsmL1_Sapi_TchF
	 || data_ind->sapi == GsmL1_Sapi_TchH) {
		/* TCH speech frame handling */
		rc = l1if_tch_rx(trx, chan_nr, l1p_msg);
		msgb_free(l1p_msg);
		return rc;
	}

	/* get rssi */
	rssi = (int8_t) (data_ind->measParam.fRssi);
	/* get data pointer and length */
	data = data_ind->msgUnitParam.u8Buffer;
	len = data_ind->msgUnitParam.u8Size;
	/* pull lower header part before data */
	msgb_pull(l1p_msg, data - l1p_msg->data);
	/* trim remaining data to it's size, to get rid of upper header part */
	rc = msgb_trim(l1p_msg, len);
	if (rc < 0)
		MSGB_ABORT(l1p_msg, "No room for primitive data\n");
	l1p_msg->l2h = l1p_msg->data;
	/* push new l1 header */
	l1p_msg->l1h = msgb_push(l1p_msg, sizeof(*l1sap));
	/* fill header */
	l1sap = msgb_l1sap_prim(l1p_msg);
	osmo_prim_init(&l1sap->oph, SAP_GSM_PH, PRIM_PH_DATA,
		PRIM_OP_INDICATION, l1p_msg);
	l1sap->u.data.link_id = link_id;
	l1sap->u.data.chan_nr = chan_nr;
	l1sap->u.data.fn = fn;
	l1sap->u.data.rssi = rssi;

	return l1sap_up(trx, l1sap);
}

static int handle_ph_ra_ind(struct lc15l1_hdl *fl1, GsmL1_PhRaInd_t *ra_ind,
			    struct msgb *l1p_msg)
{
	struct gsm_bts_trx *trx = lc15l1_hdl_trx(fl1);
	struct gsm_bts *bts = trx->bts;
	struct gsm_bts_role_bts *btsb = bts->role;
	struct gsm_lchan *lchan;
	struct osmo_phsap_prim *l1sap;
	uint32_t fn;
	uint8_t ra, acc_delay = 0;
	int rc;

	/* increment number of busy RACH slots, if required */
	if (trx == bts->c0 &&
	    ra_ind->measParam.fRssi >= btsb->load.rach.busy_thresh)
		btsb->load.rach.busy++;

	if (ra_ind->measParam.fLinkQuality < btsb->min_qual_rach) {
		msgb_free(l1p_msg);
		return 0;
	}

	if (ra_ind->measParam.i16BurstTiming > 0)
		acc_delay = ra_ind->measParam.i16BurstTiming >> 2;

	/* increment number of RACH slots with valid non-handover RACH burst */
	lchan = l1if_hLayer_to_lchan(trx, (uint32_t)ra_ind->hLayer2);
	if (trx == bts->c0 && !(lchan && lchan->ho.active == HANDOVER_ENABLED))
		btsb->load.rach.access++;

	dump_meas_res(LOGL_DEBUG, &ra_ind->measParam);

	if (ra_ind->msgUnitParam.u8Size != 1) {
		LOGP(DL1C, LOGL_ERROR, "PH-RACH-INDICATION has %d bits\n",
			ra_ind->sapi);
		msgb_free(l1p_msg);
		return 0;
	}

	fn = ra_ind->u32Fn;
	ra = ra_ind->msgUnitParam.u8Buffer[0];
	rc = msgb_trim(l1p_msg, sizeof(*l1sap));
	if (rc < 0)
		MSGB_ABORT(l1p_msg, "No room for primitive data\n");
	l1sap = msgb_l1sap_prim(l1p_msg);
	osmo_prim_init(&l1sap->oph, SAP_GSM_PH, PRIM_PH_RACH, PRIM_OP_INDICATION,
		l1p_msg);
	l1sap->u.rach_ind.ra = ra;
	l1sap->u.rach_ind.acc_delay = acc_delay;
	l1sap->u.rach_ind.fn = fn;
	if (!lchan || lchan->ts->pchan == GSM_PCHAN_CCCH ||
	    lchan->ts->pchan == GSM_PCHAN_CCCH_SDCCH4)
		l1sap->u.rach_ind.chan_nr = 0x88;
	else
		l1sap->u.rach_ind.chan_nr = gsm_lchan2chan_nr(lchan);

	return l1sap_up(trx, l1sap);
}

/* handle any random indication from the L1 */
static int l1if_handle_ind(struct lc15l1_hdl *fl1, struct msgb *msg)
{
	GsmL1_Prim_t *l1p = msgb_l1prim(msg);
	int rc = 0;

	switch (l1p->id) {
	case GsmL1_PrimId_MphTimeInd:
		rc = handle_mph_time_ind(fl1, &l1p->u.mphTimeInd);
		break;
	case GsmL1_PrimId_MphSyncInd:
		break;
	case GsmL1_PrimId_PhConnectInd:
		break;
	case GsmL1_PrimId_PhReadyToSendInd:
		return handle_ph_readytosend_ind(fl1, &l1p->u.phReadyToSendInd,
					       msg);
	case GsmL1_PrimId_PhDataInd:
		return handle_ph_data_ind(fl1, &l1p->u.phDataInd, msg);
	case GsmL1_PrimId_PhRaInd:
		return handle_ph_ra_ind(fl1, &l1p->u.phRaInd, msg);
		break;
	default:
		break;
	}

	/* Special return value '1' means: do not free */
	if (rc != 1)
		msgb_free(msg);

	return rc;
}

static inline int is_prim_compat(GsmL1_Prim_t *l1p, struct wait_l1_conf *wlc)
{
	/* the limitation here is that we cannot have multiple callers
	 * sending the same primitive */
	if (wlc->is_sys_prim != 0)
		return 0;
	if (l1p->id != wlc->conf_prim_id)
		return 0;
	return 1;
}

int l1if_handle_l1prim(int wq, struct lc15l1_hdl *fl1h, struct msgb *msg)
{
	GsmL1_Prim_t *l1p = msgb_l1prim(msg);
	struct wait_l1_conf *wlc;
	int rc;

	switch (l1p->id) {
	case GsmL1_PrimId_MphTimeInd:
		/* silent, don't clog the log file */
		break;
	default:
		LOGP(DL1P, LOGL_DEBUG, "Rx L1 prim %s on queue %d\n",
			get_value_string(lc15bts_l1prim_names, l1p->id), wq);
	}

	/* check if this is a resposne to a sync-waiting request */
	llist_for_each_entry(wlc, &fl1h->wlc_list, list) {
		if (is_prim_compat(l1p, wlc)) {
			llist_del(&wlc->list);
			if (wlc->cb)
				rc = wlc->cb(lc15l1_hdl_trx(fl1h), msg,
					     wlc->cb_data);
			else {
				rc = 0;
				msgb_free(msg);
			}
			release_wlc(wlc);
			return rc;
		}
	}

	/* if we reach here, it is not a Conf for a pending Req */
	return l1if_handle_ind(fl1h, msg);
}

int l1if_handle_sysprim(struct lc15l1_hdl *fl1h, struct msgb *msg)
{
	Litecell15_Prim_t *sysp = msgb_sysprim(msg);
	struct wait_l1_conf *wlc;
	int rc;

	LOGP(DL1P, LOGL_DEBUG, "Rx SYS prim %s\n",
		get_value_string(lc15bts_sysprim_names, sysp->id));

	/* check if this is a resposne to a sync-waiting request */
	llist_for_each_entry(wlc, &fl1h->wlc_list, list) {
		/* the limitation here is that we cannot have multiple callers
		 * sending the same primitive */
		if (wlc->is_sys_prim && sysp->id == wlc->conf_prim_id) {
			llist_del(&wlc->list);
			if (wlc->cb)
				rc = wlc->cb(lc15l1_hdl_trx(fl1h), msg,
					     wlc->cb_data);
			else {
				rc = 0;
				msgb_free(msg);
			}
			release_wlc(wlc);
			return rc;
		}
	}
	/* if we reach here, it is not a Conf for a pending Req */
	return l1if_handle_ind(fl1h, msg);
}

#if 0
/* called by RSL if the BCCH SI has been modified */
int sysinfo_has_changed(struct gsm_bts *bts, int si)
{
	/* FIXME: Determine BS_AG_BLKS_RES and 
	 *  	* set cfgParams.u.agch.u8NbrOfAgch
	 *	* determine implications on paging
	 */
	/* FIXME: Check for Extended BCCH presence */
	/* FIXME: Check for CCCH_CONF */
	/* FIXME: Check for BS_PA_MFRMS: update paging */

	return 0;
}
#endif

static int activate_rf_compl_cb(struct gsm_bts_trx *trx, struct msgb *resp,
				void *data)
{
	Litecell15_Prim_t *sysp = msgb_sysprim(resp);
	GsmL1_Status_t status;
	int on = 0;
	unsigned int i;

	if (sysp->id == Litecell15_PrimId_ActivateRfCnf)
		on = 1;

	if (on)
		status = sysp->u.activateRfCnf.status;
	else
		status = sysp->u.deactivateRfCnf.status;

	LOGP(DL1C, LOGL_INFO, "Rx RF-%sACT.conf (status=%s)\n", on ? "" : "DE",
		get_value_string(lc15bts_l1status_names, status));


	if (on) {
		if (status != GsmL1_Status_Success) {
			LOGP(DL1C, LOGL_FATAL, "RF-ACT.conf with status %s\n",
				get_value_string(lc15bts_l1status_names, status));
			bts_shutdown(trx->bts, "RF-ACT failure");
		} else
			bts_update_status(BTS_STATUS_RF_ACTIVE, 1);

		/* signal availability */
		oml_mo_state_chg(&trx->mo, NM_OPSTATE_DISABLED, NM_AVSTATE_OK);
		oml_mo_tx_sw_act_rep(&trx->mo);
		oml_mo_state_chg(&trx->bb_transc.mo, -1, NM_AVSTATE_OK);
		oml_mo_tx_sw_act_rep(&trx->bb_transc.mo);

		for (i = 0; i < ARRAY_SIZE(trx->ts); i++)
			oml_mo_state_chg(&trx->ts[i].mo, NM_OPSTATE_DISABLED, NM_AVSTATE_DEPENDENCY);
	} else {
		bts_update_status(BTS_STATUS_RF_ACTIVE, 0);
		oml_mo_state_chg(&trx->mo, NM_OPSTATE_DISABLED, NM_AVSTATE_OFF_LINE);
		oml_mo_state_chg(&trx->bb_transc.mo, NM_OPSTATE_DISABLED, NM_AVSTATE_OFF_LINE);
	}

	msgb_free(resp);

	return 0;
}

/* activate or de-activate the entire RF-Frontend */
int l1if_activate_rf(struct lc15l1_hdl *hdl, int on)
{
	struct msgb *msg = sysp_msgb_alloc();
	Litecell15_Prim_t *sysp = msgb_sysprim(msg);

	if (on) {
		sysp->id = Litecell15_PrimId_ActivateRfReq;
		sysp->u.activateRfReq.msgq.u8UseTchMsgq = 0;
		sysp->u.activateRfReq.msgq.u8UsePdtchMsgq = pcu_direct;

		sysp->u.activateRfReq.u8UnusedTsMode = 0;
		sysp->u.activateRfReq.u8McCorrMode = 0;

		/* maximum cell size in quarter-bits, 90 == 12.456 km */
		sysp->u.activateRfReq.u8MaxCellSize = 90;
	} else {
		sysp->id = Litecell15_PrimId_DeactivateRfReq;
	}

	return l1if_req_compl(hdl, msg, activate_rf_compl_cb, NULL);
}

static void mute_handle_ts(struct gsm_bts_trx_ts *ts, int is_muted)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ts->lchan); i++) {
		struct gsm_lchan *lchan = &ts->lchan[i];

		if (!is_muted)
			continue;

		if (lchan->state != LCHAN_S_ACTIVE)
			continue;

		/* skip channels that might be active for another reason */
		if (lchan->type == GSM_LCHAN_CCCH)
			continue;
		if (lchan->type == GSM_LCHAN_PDTCH)
			continue;

		if (lchan->s <= 0)
			continue;

		lchan->s = 0;
		rsl_tx_conn_fail(lchan, RSL_ERR_RADIO_LINK_FAIL);
	}
}

static int mute_rf_compl_cb(struct gsm_bts_trx *trx, struct msgb *resp,
			    void *data)
{
	struct lc15l1_hdl *fl1h = trx_lc15l1_hdl(trx);
	Litecell15_Prim_t *sysp = msgb_sysprim(resp);
	GsmL1_Status_t status;

	status = sysp->u.muteRfCnf.status;

	if (status != GsmL1_Status_Success) {
		LOGP(DL1C, LOGL_ERROR, "Rx RF-MUTE.conf with status %s\n",
		     get_value_string(lc15bts_l1status_names, status));
		oml_mo_rf_lock_chg(&trx->mo, fl1h->last_rf_mute, 0);
	} else {
		int i;

		LOGP(DL1C, LOGL_INFO, "Rx RF-MUTE.conf with status=%s\n",
		     get_value_string(lc15bts_l1status_names, status));
		bts_update_status(BTS_STATUS_RF_MUTE, fl1h->last_rf_mute[0]);
		oml_mo_rf_lock_chg(&trx->mo, fl1h->last_rf_mute, 1);

		osmo_static_assert(
			ARRAY_SIZE(trx->ts) >= ARRAY_SIZE(fl1h->last_rf_mute),
			ts_array_size);

		for (i = 0; i < ARRAY_SIZE(fl1h->last_rf_mute); ++i)
			mute_handle_ts(&trx->ts[i], fl1h->last_rf_mute[i]);
	}

	msgb_free(resp);

	return 0;
}

/* mute/unmute RF time slots */
int l1if_mute_rf(struct lc15l1_hdl *hdl, uint8_t mute[8], l1if_compl_cb *cb)
{
	struct msgb *msg = sysp_msgb_alloc();
	Litecell15_Prim_t *sysp = msgb_sysprim(msg);

	LOGP(DL1C, LOGL_INFO, "Tx RF-MUTE.req (%d, %d, %d, %d, %d, %d, %d, %d)\n",
	     mute[0], mute[1], mute[2], mute[3],
	     mute[4], mute[5], mute[6], mute[7]
	    );

	sysp->id = Litecell15_PrimId_MuteRfReq;
	memcpy(sysp->u.muteRfReq.u8Mute, mute, sizeof(sysp->u.muteRfReq.u8Mute));
	/* save for later use */
	memcpy(hdl->last_rf_mute, mute, sizeof(hdl->last_rf_mute));

	return l1if_req_compl(hdl, msg, cb ? cb : mute_rf_compl_cb, NULL);
}

/* call-back on arrival of DSP+FPGA version + band capability */
static int info_compl_cb(struct gsm_bts_trx *trx, struct msgb *resp,
			 void *data)
{
	Litecell15_Prim_t *sysp = msgb_sysprim(resp);
	Litecell15_SystemInfoCnf_t *sic = &sysp->u.systemInfoCnf;
	struct lc15l1_hdl *fl1h = trx_lc15l1_hdl(trx);
	int rc;

	fl1h->hw_info.dsp_version[0] = sic->dspVersion.major;
	fl1h->hw_info.dsp_version[1] = sic->dspVersion.minor;
	fl1h->hw_info.dsp_version[2] = sic->dspVersion.build;

	fl1h->hw_info.fpga_version[0] = sic->fpgaVersion.major;
	fl1h->hw_info.fpga_version[1] = sic->fpgaVersion.minor;
	fl1h->hw_info.fpga_version[2] = sic->fpgaVersion.build;

	LOGP(DL1C, LOGL_INFO, "DSP v%u.%u.%u, FPGA v%u.%u.%u\nn",
		sic->dspVersion.major, sic->dspVersion.minor,
		sic->dspVersion.build, sic->fpgaVersion.major,
		sic->fpgaVersion.minor, sic->fpgaVersion.build);

	if (!(fl1h->hw_info.band_support & trx->bts->band))
		LOGP(DL1C, LOGL_FATAL, "BTS band %s not supported by hw\n",
		     gsm_band_name(trx->bts->band));

	/* Request the activation */
	l1if_activate_rf(fl1h, 1);

	/* load calibration tables */
	rc = calib_load(fl1h);
	if (rc < 0)
		LOGP(DL1C, LOGL_ERROR, "Operating without calibration; "
			"unable to load tables!\n");

	msgb_free(resp);
	return 0;
}

/* request DSP+FPGA code versions */
static int l1if_get_info(struct lc15l1_hdl *hdl)
{
	struct msgb *msg = sysp_msgb_alloc();
	Litecell15_Prim_t *sysp = msgb_sysprim(msg);

	sysp->id = Litecell15_PrimId_SystemInfoReq;

	return l1if_req_compl(hdl, msg, info_compl_cb, NULL);
}

static int reset_compl_cb(struct gsm_bts_trx *trx, struct msgb *resp,
			  void *data)
{
	struct lc15l1_hdl *fl1h = trx_lc15l1_hdl(trx);
	Litecell15_Prim_t *sysp = msgb_sysprim(resp);
	GsmL1_Status_t status = sysp->u.layer1ResetCnf.status;

	LOGP(DL1C, LOGL_NOTICE, "Rx L1-RESET.conf (status=%s)\n",
		get_value_string(lc15bts_l1status_names, status));

	msgb_free(resp);

	/* If we're coming out of reset .. */
	if (status != GsmL1_Status_Success) {
		LOGP(DL1C, LOGL_FATAL, "L1-RESET.conf with status %s\n",
			get_value_string(lc15bts_l1status_names, status));
		bts_shutdown(trx->bts, "L1-RESET failure");
	}

	/* as we cannot get the current DSP trace flags, we simply
	 * set them to zero (or whatever dsp_trace_f has been initialized to */
	l1if_set_trace_flags(fl1h, fl1h->dsp_trace_f);

	/* obtain version information on DSP/FPGA and band capabilities */
	l1if_get_info(fl1h);

	return 0;
}

int l1if_reset(struct lc15l1_hdl *hdl)
{
	struct msgb *msg = sysp_msgb_alloc();
	Litecell15_Prim_t *sysp = msgb_sysprim(msg);
	sysp->id = Litecell15_PrimId_Layer1ResetReq;

	return l1if_req_compl(hdl, msg, reset_compl_cb, NULL);
}

/* set the trace flags within the DSP */
int l1if_set_trace_flags(struct lc15l1_hdl *hdl, uint32_t flags)
{
	struct msgb *msg = sysp_msgb_alloc();
	Litecell15_Prim_t *sysp = msgb_sysprim(msg);

	LOGP(DL1C, LOGL_INFO, "Tx SET-TRACE-FLAGS.req (0x%08x)\n",
		flags);

	sysp->id = Litecell15_PrimId_SetTraceFlagsReq;
	sysp->u.setTraceFlagsReq.u32Tf = flags;

	hdl->dsp_trace_f = flags;

	/* There is no confirmation we could wait for */
	if (osmo_wqueue_enqueue(&hdl->write_q[MQ_SYS_WRITE], msg) != 0) {
		LOGP(DL1C, LOGL_ERROR, "MQ_SYS_WRITE queue full. Dropping msg\n");
		msgb_free(msg);
		return -EAGAIN;
	}
	return 0;
}

static int get_hwinfo(struct lc15l1_hdl *fl1h)
{
	int rc;

	rc = lc15bts_rev_get();
	if (rc < 0)
		return rc;
	fl1h->hw_info.ver_major = rc;

	rc = lc15bts_model_get();
	if (rc < 0)
		return rc;
	fl1h->hw_info.ver_minor = rc;

	rc = lc15bts_option_get(LC15BTS_OPTION_BAND);
	if (rc < 0) 
		return rc;

	switch (rc) {
	case LC15BTS_BAND_850: 
		fl1h->hw_info.band_support = GSM_BAND_850; 
		break;
	case LC15BTS_BAND_900: 
		fl1h->hw_info.band_support = GSM_BAND_900; 
		break;
	case LC15BTS_BAND_1800: 
		fl1h->hw_info.band_support = GSM_BAND_1800; 
		break;
	case LC15BTS_BAND_1900: 
		fl1h->hw_info.band_support = GSM_BAND_1900; 
		break;
	default:
		return -1;
	}
	return 0;
}

struct lc15l1_hdl *l1if_open(struct phy_instance *pinst)
{
	struct lc15l1_hdl *fl1h;
	int rc;

	LOGP(DL1C, LOGL_INFO, "Litecell 1.5 BTS L1IF compiled against API headers "
			"v%u.%u.%u\n", LITECELL15_API_VERSION >> 16,
			(LITECELL15_API_VERSION >> 8) & 0xff,
			 LITECELL15_API_VERSION & 0xff);

	fl1h = talloc_zero(pinst, struct lc15l1_hdl);
	if (!fl1h)
		return NULL;
	INIT_LLIST_HEAD(&fl1h->wlc_list);

	fl1h->phy_inst = pinst;
	fl1h->dsp_trace_f = pinst->u.lc15.dsp_trace_f;

	get_hwinfo(fl1h);

	rc = l1if_transport_open(MQ_SYS_WRITE, fl1h);
	if (rc < 0) {
		talloc_free(fl1h);
		return NULL;
	}

	rc = l1if_transport_open(MQ_L1_WRITE, fl1h);
	if (rc < 0) {
		l1if_transport_close(MQ_SYS_WRITE, fl1h);
		talloc_free(fl1h);
		return NULL;
	}

	return fl1h;
}

int l1if_close(struct lc15l1_hdl *fl1h)
{
	l1if_transport_close(MQ_L1_WRITE, fl1h);
	l1if_transport_close(MQ_SYS_WRITE, fl1h);
	return 0;
}

int bts_model_phy_link_open(struct phy_link *plink)
{
	struct phy_instance *pinst = phy_instance_by_num(plink, 0);

	OSMO_ASSERT(pinst);

	if (!pinst->trx) {
		LOGP(DL1C, LOGL_NOTICE, "Ignoring phy link %d instance %d "
		     "because no TRX is associated with it\n", plink->num, pinst->num);
		return 0;
	}
	phy_link_state_set(plink, PHY_LINK_CONNECTING);

	pinst->u.lc15.hdl = l1if_open(pinst);
	if (!pinst->u.lc15.hdl) {
		LOGP(DL1C, LOGL_FATAL, "Cannot open L1 interface\n");
		return -EIO;
	}

	l1if_reset(pinst->u.lc15.hdl);

	phy_link_state_set(plink, PHY_LINK_CONNECTED);

	return 0;
}

