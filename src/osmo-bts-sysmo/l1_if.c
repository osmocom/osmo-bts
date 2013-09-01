/* Interface handler for Sysmocom L1 */

/* (C) 2011 by Harald Welte <laforge@gnumonks.org>
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
#include <osmocom/core/gsmtap.h>
#include <osmocom/core/gsmtap_util.h>
#include <osmocom/gsm/gsm_utils.h>
#include <osmocom/gsm/lapdm.h>

#include <osmo-bts/logging.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/oml.h>
#include <osmo-bts/rsl.h>
#include <osmo-bts/gsm_data.h>
#include <osmo-bts/paging.h>
#include <osmo-bts/measurement.h>
#include <osmo-bts/pcu_if.h>
#include <osmo-bts/l1sap.h>

#include <sysmocom/femtobts/superfemto.h>
#include <sysmocom/femtobts/gsml1prim.h>
#include <sysmocom/femtobts/gsml1const.h>
#include <sysmocom/femtobts/gsml1types.h>

#include "femtobts.h"
#include "l1_if.h"
#include "l1_transp.h"
#include "hw_misc.h"
#include "misc/sysmobts_par.h"
#include "eeprom.h"

extern int pcu_direct;

#define MIN_QUAL_RACH	 5.0f	/* at least  5 dB C/I */
#define MIN_QUAL_NORM	-0.5f	/* at least -1 dB C/I */

/* mapping from femtobts L1 SAPI to GSMTAP channel type */
static const uint8_t l1sapi2gsmtap_cht[GsmL1_Sapi_NUM] = {
	[GsmL1_Sapi_Idle] = 255,
	[GsmL1_Sapi_Fcch] = 255,
	[GsmL1_Sapi_Sch] = 255,
	[GsmL1_Sapi_Sacch] = GSMTAP_CHANNEL_SDCCH | GSMTAP_CHANNEL_ACCH,
	[GsmL1_Sapi_Sdcch] = GSMTAP_CHANNEL_SDCCH,
	[GsmL1_Sapi_Bcch] = GSMTAP_CHANNEL_BCCH,
	[GsmL1_Sapi_Pch] = GSMTAP_CHANNEL_PCH,
	[GsmL1_Sapi_Agch] = GSMTAP_CHANNEL_AGCH,
	[GsmL1_Sapi_Cbch] = GSMTAP_CHANNEL_CBCH51,
	[GsmL1_Sapi_Rach] = GSMTAP_CHANNEL_RACH,
	[GsmL1_Sapi_TchF] = 255,
	[GsmL1_Sapi_FacchF] = GSMTAP_CHANNEL_TCH_F,
	[GsmL1_Sapi_TchH] = 255,
	[GsmL1_Sapi_FacchH] = GSMTAP_CHANNEL_TCH_H,
	[GsmL1_Sapi_Nch] = GSMTAP_CHANNEL_CCCH,
	[GsmL1_Sapi_Pdtch] = GSMTAP_CHANNEL_PACCH,
	[GsmL1_Sapi_Pacch] = GSMTAP_CHANNEL_PACCH,
	[GsmL1_Sapi_Pbcch] = 255,
	[GsmL1_Sapi_Pagch] = 255,
	[GsmL1_Sapi_Ppch] = 255,
	[GsmL1_Sapi_Pnch] = 255,
	[GsmL1_Sapi_Ptcch] = GSMTAP_CHANNEL_PTCCH,
	[GsmL1_Sapi_Prach] = 255,
};

static void tx_to_gsmtap(struct femtol1_hdl *fl1h, struct msgb *msg)
{
	struct gsm_bts_trx *trx = fl1h->priv;
	GsmL1_Prim_t *l1p = msgb_l1prim(msg);
	GsmL1_PhDataReq_t *data_req = &l1p->u.phDataReq;

	if (fl1h->gsmtap) {
		uint8_t ss, chan_type;
		if (data_req->subCh == 0x1f)
			ss = 0;
		else
			ss = data_req->subCh;

		if (!(fl1h->gsmtap_sapi_mask & (1 << data_req->sapi)))
			return;

		chan_type = l1sapi2gsmtap_cht[data_req->sapi];
		if (chan_type == 255)
			return;

		gsmtap_send(fl1h->gsmtap, trx->arfcn, data_req->u8Tn,
				chan_type, ss, data_req->u32Fn, 0, 0,
				data_req->msgUnitParam.u8Buffer,
				data_req->msgUnitParam.u8Size);
	}
}

static void ul_to_gsmtap(struct femtol1_hdl *fl1h, struct msgb *msg)
{
	struct gsm_bts_trx *trx = fl1h->priv;
	GsmL1_Prim_t *l1p = msgb_l1prim(msg);
	GsmL1_PhDataInd_t *data_ind = &l1p->u.phDataInd;
	int skip = 0;

	if (fl1h->gsmtap) {
		uint8_t ss, chan_type;
		if (data_ind->subCh == 0x1f)
			ss = 0;
		else
			ss = data_ind->subCh;

		if (!(fl1h->gsmtap_sapi_mask & (1 << data_ind->sapi)))
			return;

		chan_type = l1sapi2gsmtap_cht[data_ind->sapi];
		if (chan_type == 255)
			return;
		if (chan_type == GSMTAP_CHANNEL_PACCH
		 || chan_type == GSMTAP_CHANNEL_PDCH) {
			if (data_ind->msgUnitParam.u8Buffer[0]
					!= GsmL1_PdtchPlType_Full)
				return;
			skip = 1;
		}

		gsmtap_send(fl1h->gsmtap, trx->arfcn | GSMTAP_ARFCN_F_UPLINK,
				data_ind->u8Tn, chan_type, ss, data_ind->u32Fn,
				data_ind->measParam.fRssi,
				data_ind->measParam.fLinkQuality,
				data_ind->msgUnitParam.u8Buffer + skip,
				data_ind->msgUnitParam.u8Size - skip);
	}
}


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
			get_value_string(femtobts_sysprim_names, wlc->conf_prim_id));
	else
		LOGP(DL1C, LOGL_FATAL, "Timeout waiting for L1 primitive %s\n",
			get_value_string(femtobts_l1prim_names, wlc->conf_prim_id));
	exit(23);
}

static int _l1if_req_compl(struct femtol1_hdl *fl1h, struct msgb *msg,
		   int is_system_prim, l1if_compl_cb *cb)
{
	struct wait_l1_conf *wlc;
	struct osmo_wqueue *wqueue;
	unsigned int timeout_secs;

	/* allocate new wsc and store reference to mutex and conf_id */
	wlc = talloc_zero(fl1h, struct wait_l1_conf);
	wlc->cb = cb;
	wlc->cb_data = NULL;

	/* Make sure we actually have received a REQUEST type primitive */
	if (is_system_prim == 0) {
		GsmL1_Prim_t *l1p = msgb_l1prim(msg);

		LOGP(DL1P, LOGL_INFO, "Tx L1 prim %s\n",
			get_value_string(femtobts_l1prim_names, l1p->id));

		if (femtobts_l1prim_type[l1p->id] != L1P_T_REQ) {
			LOGP(DL1C, LOGL_ERROR, "L1 Prim %s is not a Request!\n",
				get_value_string(femtobts_l1prim_names, l1p->id));
			talloc_free(wlc);
			return -EINVAL;
		}
		wlc->is_sys_prim = 0;
		wlc->conf_prim_id = femtobts_l1prim_req2conf[l1p->id];
		wqueue = &fl1h->write_q[MQ_L1_WRITE];
		timeout_secs = 30;
	} else {
		SuperFemto_Prim_t *sysp = msgb_sysprim(msg);

		LOGP(DL1C, LOGL_INFO, "Tx SYS prim %s\n",
			get_value_string(femtobts_sysprim_names, sysp->id));

		if (femtobts_sysprim_type[sysp->id] != L1P_T_REQ) {
			LOGP(DL1C, LOGL_ERROR, "SYS Prim %s is not a Request!\n",
				get_value_string(femtobts_sysprim_names, sysp->id));
			talloc_free(wlc);
			return -EINVAL;
		}
		wlc->is_sys_prim = 1;
		wlc->conf_prim_id = femtobts_sysprim_req2conf[sysp->id];
		wqueue = &fl1h->write_q[MQ_SYS_WRITE];
		timeout_secs = 30;
	}

	/* enqueue the message in the queue and add wsc to list */
	osmo_wqueue_enqueue(wqueue, msg);
	llist_add(&wlc->list, &fl1h->wlc_list);

	/* schedule a timer for timeout_secs seconds. If DSP fails to respond, we terminate */
	wlc->timer.data = wlc;
	wlc->timer.cb = l1if_req_timeout;
	osmo_timer_schedule(&wlc->timer, timeout_secs, 0);

	return 0;
}

/* send a request primitive to the L1 and schedule completion call-back */
int l1if_req_compl(struct femtol1_hdl *fl1h, struct msgb *msg,
		   l1if_compl_cb *cb)
{
	return _l1if_req_compl(fl1h, msg, 1, cb);
}

int l1if_gsm_req_compl(struct femtol1_hdl *fl1h, struct msgb *msg,
		   l1if_compl_cb *cb)
{
	return _l1if_req_compl(fl1h, msg, 0, cb);
}

/* allocate a msgb containing a GsmL1_Prim_t */
struct msgb *l1p_msgb_alloc(void)
{
	struct msgb *msg = msgb_alloc(sizeof(GsmL1_Prim_t), "l1_prim");

	if (msg)
		msg->l1h = msgb_put(msg, sizeof(GsmL1_Prim_t));

	return msg;
}

/* allocate a msgb containing a SuperFemto_Prim_t */
struct msgb *sysp_msgb_alloc(void)
{
	struct msgb *msg = msgb_alloc(sizeof(SuperFemto_Prim_t), "sys_prim");

	if (msg)
		msg->l1h = msgb_put(msg, sizeof(SuperFemto_Prim_t));

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

static int ph_data_req(struct gsm_bts_trx *trx, struct msgb *msg,
		       struct osmo_phsap_prim *l1sap)
{
	struct femtol1_hdl *fl1 = trx_femtol1_hdl(trx);
	uint32_t u32Fn;
	uint8_t u8Tn, subCh, u8BlockNbr = 0, sapi;
	uint8_t chan_nr, link_id;
	GsmL1_Prim_t *l1p;
	int len;

	if (!msg) {
		LOGP(DL1C, LOGL_FATAL, "PH-DATA.req without msg. "
			"Please fix!\n");
		abort();
	}
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

	/* pull and trim msg to start of payload */
	msgb_pull(msg, sizeof(*l1sap));
	len = msg->len;
	msgb_trim(msg, 0);

	/* convert l1sap message to GsmL1 primitive, keep payload */
	if (len) {
		/* data request */
		GsmL1_PhDataReq_t *data_req;
		GsmL1_MsgUnitParam_t *msu_param;
		uint8_t *temp;

		/* wrap zeroed l1p structure arrount payload
		 * this must be done in three steps, since the actual
		 * payload is not at the end but inside the l1p structure. */
		temp = l1p->u.phDataReq.msgUnitParam.u8Buffer;
		msgb_push(msg, temp - (uint8_t *)l1p);
		memset(msg->data, 0, msg->len);
		msgb_put(msg, len);
		memset(msg->tail, 0, sizeof(*l1p) - msg->len);
		msgb_put(msg, sizeof(*l1p) - msg->len);
		msg->l1h = msg->data;

		l1p = msgb_l1prim(msg);
		l1p->id = GsmL1_PrimId_PhDataReq;
		data_req = &l1p->u.phDataReq;
		data_req->hLayer1 = fl1->hLayer1;
		data_req->u8Tn = u8Tn;
		data_req->u32Fn = u32Fn;
		data_req->sapi = sapi;
		data_req->subCh = subCh;
		data_req->u8BlockNbr = u8BlockNbr;
		msu_param = &data_req->msgUnitParam;
		msu_param->u8Size = len;
	} else {
		/* empty frame */
		GsmL1_PhEmptyFrameReq_t *empty_req;

		/* put l1p structure */
		msgb_put(msg, sizeof(*l1p));
		memset(msg->data, 0, msg->len);
		msg->l1h = msg->data;

		l1p = msgb_l1prim(msg);
		l1p->id = GsmL1_PrimId_PhEmptyFrameReq;
		empty_req = &l1p->u.phEmptyFrameReq;
		empty_req->hLayer1 = fl1->hLayer1;
		empty_req->u8Tn = u8Tn;
		empty_req->u32Fn = u32Fn;
		empty_req->sapi = sapi;
		empty_req->subCh = subCh;
		empty_req->u8BlockNbr = u8BlockNbr;
	}

	/* send message to DSP's queue */
	osmo_wqueue_enqueue(&fl1->write_q[MQ_L1_WRITE], msg);

	return 0;
}

static int ph_tch_req(struct gsm_bts_trx *trx, struct msgb *msg,
		       struct osmo_phsap_prim *l1sap)
{
	struct femtol1_hdl *fl1 = trx_femtol1_hdl(trx);
	struct gsm_lchan *lchan;
	uint32_t u32Fn;
	uint8_t u8Tn, subCh, u8BlockNbr = 0, sapi, ss;
	uint8_t chan_nr;
	GsmL1_Prim_t *l1p;
	struct msgb *nmsg;

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
		GsmL1_PhDataReq_t *data_req = &l1p->u.phDataReq;

		l1p->id = GsmL1_PrimId_PhDataReq;

		data_req->hLayer1 = fl1->hLayer1;
		data_req->u8Tn = u8Tn;
		data_req->u32Fn = u32Fn;
		data_req->sapi = sapi;
		data_req->subCh = subCh;
		data_req->u8BlockNbr = u8BlockNbr;
	} else {
		/* empty frame */
		GsmL1_PhEmptyFrameReq_t *empty_req =
						&l1p->u.phEmptyFrameReq;

		l1p->id = GsmL1_PrimId_PhEmptyFrameReq;

		empty_req->hLayer1 = fl1->hLayer1;
		empty_req->u8Tn = u8Tn;
		empty_req->u32Fn = u32Fn;
		empty_req->sapi = sapi;
		empty_req->subCh = subCh;
		empty_req->u8BlockNbr = u8BlockNbr;
	}
	/* send message to DSP's queue */
	osmo_wqueue_enqueue(&fl1->write_q[MQ_L1_WRITE], nmsg);

	msgb_free(msg);
	return 0;
}

/* primitive from common part */
int bts_model_l1sap_down(struct gsm_bts_trx *trx, struct osmo_phsap_prim *l1sap)
{
	struct femtol1_hdl *fl1 = trx_femtol1_hdl(trx);
	struct msgb *msg = l1sap->oph.msg;
	uint8_t u8Tn, ss;
	uint8_t chan_nr;
	struct gsm_lchan *lchan;
	int rc = 0;

	switch (OSMO_PRIM_HDR(&l1sap->oph)) {
	case OSMO_PRIM(PRIM_PH_DATA, PRIM_OP_REQUEST):
		rc = ph_data_req(trx, msg, l1sap);
		break;
	case OSMO_PRIM(PRIM_TCH, PRIM_OP_REQUEST):
		rc = ph_tch_req(trx, msg, l1sap);
		break;
	case OSMO_PRIM(PRIM_MPH_INFO, PRIM_OP_REQUEST):
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
				lchan->ciph_state = LCHAN_CIPH_TXRX_REQ;
			}
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
			else if (l1sap->u.info.type == PRIM_INFO_MODIFY)
				l1if_rsl_mode_modify(lchan);
			else if (l1sap->u.info.u.act_req.sacch_only)
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

static int handle_mph_time_ind(struct femtol1_hdl *fl1,
				GsmL1_MphTimeInd_t *time_ind)
{
	struct gsm_bts_trx *trx = fl1->priv;
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

	return (cbits << 3) | u8Tn;
}

static int handle_ph_readytosend_ind(struct femtol1_hdl *fl1,
				     GsmL1_PhReadyToSendInd_t *rts_ind,
				     struct msgb *l1p_msg)
{
	struct gsm_bts_trx *trx = fl1->priv;
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
		get_value_string(femtobts_l1sapi_names, rts_ind->sapi));

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
	default:
		memcpy(msu_param->u8Buffer, fill_frame, GSM_MACBLOCK_LEN);
		break;
	}
tx:

	tx_to_gsmtap(fl1, resp_msg);

	/* transmit */
	osmo_wqueue_enqueue(&fl1->write_q[MQ_L1_WRITE], resp_msg);

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

static int process_meas_res(struct gsm_lchan *lchan, GsmL1_MeasParam_t *m)
{
	struct bts_ul_meas ulm;

	/* in the GPRS case we are not interested in measurement
	 * processing.  The PCU will take care of it */
	if (lchan->type == GSM_LCHAN_PDTCH)
		return 0;

	ulm.ta_offs_qbits = m->i16BurstTiming;
	ulm.ber10k = (unsigned int) (m->fBer * 100);
	ulm.inv_rssi = (uint8_t) (m->fRssi * -1);

	return lchan_new_ul_meas(lchan, &ulm);
}

static int handle_ph_data_ind(struct femtol1_hdl *fl1, GsmL1_PhDataInd_t *data_ind,
			      struct msgb *l1p_msg)
{
	struct gsm_bts_trx *trx = fl1->priv;
	struct gsm_lchan *lchan;
	uint8_t chan_nr, link_id;
	struct osmo_phsap_prim *l1sap;
	uint32_t fn;
	uint8_t *data, len;
	int rc = 0;

	ul_to_gsmtap(fl1, l1p_msg);

	lchan = l1if_hLayer_to_lchan(fl1->priv, data_ind->hLayer2);
	if (!lchan) {
		LOGP(DL1C, LOGL_ERROR, "unable to resolve lchan by hLayer2\n");
		msgb_free(l1p_msg);
		return -ENODEV;
	}

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

	process_meas_res(lchan, &data_ind->measParam);

	if (data_ind->measParam.fLinkQuality < fl1->min_qual_norm
	 && data_ind->msgUnitParam.u8Size != 0) {
		msgb_free(l1p_msg);
		return 0;
	}

	DEBUGP(DL1C, "Rx PH-DATA.ind %s (hL2 %08x): %s",
		get_value_string(femtobts_l1sapi_names, data_ind->sapi),
		data_ind->hLayer2,
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

	return l1sap_up(trx, l1sap);
}


static int handle_ph_ra_ind(struct femtol1_hdl *fl1, GsmL1_PhRaInd_t *ra_ind,
			    struct msgb *l1p_msg)
{
	struct gsm_bts_trx *trx = fl1->priv;
	struct gsm_bts *bts = trx->bts;
	struct gsm_bts_role_bts *btsb = bts->role;
	struct osmo_phsap_prim *l1sap;
	uint32_t fn;
	uint8_t ra, acc_delay;
	int rc;

	/* increment number of busy RACH slots, if required */
	if (trx == bts->c0 &&
	    ra_ind->measParam.fRssi >= btsb->load.rach.busy_thresh)
		btsb->load.rach.busy++;

	if (ra_ind->measParam.fLinkQuality < fl1->min_qual_rach) {
		msgb_free(l1p_msg);
		return 0;
	}

	/* increment number of RACH slots with valid RACH burst */
	if (trx == bts->c0)
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
	/* check for under/overflow / sign */
	if (ra_ind->measParam.i16BurstTiming < 0)
		acc_delay = 0;
	else
		acc_delay = ra_ind->measParam.i16BurstTiming >> 2;
	rc = msgb_trim(l1p_msg, sizeof(*l1sap));
	if (rc < 0)
		MSGB_ABORT(l1p_msg, "No room for primitive data\n");
	l1sap = msgb_l1sap_prim(l1p_msg);
	osmo_prim_init(&l1sap->oph, SAP_GSM_PH, PRIM_PH_RACH, PRIM_OP_INDICATION,
		l1p_msg);
	l1sap->u.rach_ind.ra = ra;
	l1sap->u.rach_ind.acc_delay = acc_delay;
	l1sap->u.rach_ind.fn = fn;

	return l1sap_up(trx, l1sap);
}

/* handle any random indication from the L1 */
static int l1if_handle_ind(struct femtol1_hdl *fl1, struct msgb *msg)
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

int l1if_handle_l1prim(int wq, struct femtol1_hdl *fl1h, struct msgb *msg)
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
			get_value_string(femtobts_l1prim_names, l1p->id), wq);
	}

	/* check if this is a resposne to a sync-waiting request */
	llist_for_each_entry(wlc, &fl1h->wlc_list, list) {
		if (is_prim_compat(l1p, wlc)) {
			llist_del(&wlc->list);
			if (wlc->cb)
				rc = wlc->cb(fl1h->priv, msg);
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

int l1if_handle_sysprim(struct femtol1_hdl *fl1h, struct msgb *msg)
{
	SuperFemto_Prim_t *sysp = msgb_sysprim(msg);
	struct wait_l1_conf *wlc;
	int rc;

	LOGP(DL1P, LOGL_DEBUG, "Rx SYS prim %s\n",
		get_value_string(femtobts_sysprim_names, sysp->id));

	/* check if this is a resposne to a sync-waiting request */
	llist_for_each_entry(wlc, &fl1h->wlc_list, list) {
		/* the limitation here is that we cannot have multiple callers
		 * sending the same primitive */
		if (wlc->is_sys_prim && sysp->id == wlc->conf_prim_id) {
			llist_del(&wlc->list);
			if (wlc->cb)
				rc = wlc->cb(fl1h->priv, msg);
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

static int activate_rf_compl_cb(struct gsm_bts_trx *trx, struct msgb *resp)
{
	SuperFemto_Prim_t *sysp = msgb_sysprim(resp);
	GsmL1_Status_t status;
	int on = 0;
	unsigned int i;

	if (sysp->id == SuperFemto_PrimId_ActivateRfCnf)
		on = 1;

	if (on)
		status = sysp->u.activateRfCnf.status;
	else
		status = sysp->u.deactivateRfCnf.status;

	LOGP(DL1C, LOGL_INFO, "Rx RF-%sACT.conf (status=%s)\n", on ? "" : "DE",
		get_value_string(femtobts_l1status_names, status));


	if (on) {
		if (status != GsmL1_Status_Success) {
			LOGP(DL1C, LOGL_FATAL, "RF-ACT.conf with status %s\n",
				get_value_string(femtobts_l1status_names, status));
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

static int get_clk_cal(struct femtol1_hdl *hdl)
{
#ifdef FEMTOBTS_API_VERSION
	return hdl->clk_cal;
#else
	switch (hdl->clk_src) {
	case SuperFemto_ClkSrcId_Ocxo:
	case SuperFemto_ClkSrcId_Tcxo:
		/* only for those on-board clocks it makes sense to use
		 * the calibration value */
		return hdl->clk_cal;
	default:
		/* external clocks like GPS are taken 1:1 without any
		 * modification by a local calibration value */
		LOGP(DL1C, LOGL_INFO, "Ignoring Clock Calibration for "
		     "selected %s clock\n",
		     get_value_string(femtobts_clksrc_names, hdl->clk_src));
		return 0;
	}
#endif
}

/* activate or de-activate the entire RF-Frontend */
int l1if_activate_rf(struct femtol1_hdl *hdl, int on)
{
	struct msgb *msg = sysp_msgb_alloc();
	SuperFemto_Prim_t *sysp = msgb_sysprim(msg);

	if (on) {
		sysp->id = SuperFemto_PrimId_ActivateRfReq;
#ifdef HW_SYSMOBTS_V1
		sysp->u.activateRfReq.u12ClkVc = get_clk_cal(hdl);
#else
#if SUPERFEMTO_API_VERSION >= SUPERFEMTO_API(0,2,0)
		sysp->u.activateRfReq.timing.u8TimSrc = 1; /* Master */
#endif /* 0.2.0 */
		sysp->u.activateRfReq.msgq.u8UseTchMsgq = 0;
		sysp->u.activateRfReq.msgq.u8UsePdtchMsgq = pcu_direct;
		/* Use clock from OCXO or whatever source is configured */
#if SUPERFEMTO_API_VERSION < SUPERFEMTO_API(2,1,0)
		sysp->u.activateRfReq.rfTrx.u8ClkSrc = hdl->clk_src;
#else
		sysp->u.activateRfReq.rfTrx.clkSrc = hdl->clk_src;
#endif /* 2.1.0 */
		sysp->u.activateRfReq.rfTrx.iClkCor = get_clk_cal(hdl);
#if SUPERFEMTO_API_VERSION < SUPERFEMTO_API(2,4,0)
#if SUPERFEMTO_API_VERSION < SUPERFEMTO_API(2,1,0)
		sysp->u.activateRfReq.rfRx.u8ClkSrc = hdl->clk_src;
#else
		sysp->u.activateRfReq.rfRx.clkSrc = hdl->clk_src;
#endif /* 2.1.0 */
		sysp->u.activateRfReq.rfRx.iClkCor = get_clk_cal(hdl);
#endif /* API 2.4.0 */
#endif /* !HW_SYSMOBTS_V1 */
	} else {
		sysp->id = SuperFemto_PrimId_DeactivateRfReq;
	}

	return l1if_req_compl(hdl, msg, activate_rf_compl_cb);
}

#if SUPERFEMTO_API_VERSION >= SUPERFEMTO_API(3,6,0)
static void mute_handle_ts(struct gsm_bts_trx_ts *ts, int is_muted)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ts->lchan); i++) {
		struct gsm_lchan *lchan = &ts->lchan[i];

		if (!is_muted)
			continue;

		if (lchan->state != LCHAN_S_ACTIVE)
			continue;

		if (lchan->s <= 0)
			continue;

		lchan->s = 0;
		rsl_tx_conn_fail(lchan, RSL_ERR_RADIO_LINK_FAIL);
	}
}

static int mute_rf_compl_cb(struct gsm_bts_trx *trx, struct msgb *resp)
{
	struct femtol1_hdl *fl1h = trx_femtol1_hdl(trx);
	SuperFemto_Prim_t *sysp = msgb_sysprim(resp);
	GsmL1_Status_t status;

	status = sysp->u.muteRfCnf.status;

	if (status != GsmL1_Status_Success) {
		LOGP(DL1C, LOGL_ERROR, "Rx RF-MUTE.conf with status %s\n",
		     get_value_string(femtobts_l1status_names, status));
		oml_mo_rf_lock_chg(&trx->mo, fl1h->last_rf_mute, 0);
	} else {
		int i;

		LOGP(DL1C, LOGL_INFO, "Rx RF-MUTE.conf with status=%s\n",
		     get_value_string(femtobts_l1status_names, status));
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
#endif

/* mute/unmute RF time slots */
int l1if_mute_rf(struct femtol1_hdl *hdl, uint8_t mute[8])
{
	struct msgb *msg = sysp_msgb_alloc();
	SuperFemto_Prim_t *sysp = msgb_sysprim(msg);

	LOGP(DL1C, LOGL_INFO, "Tx RF-MUTE.req (%d, %d, %d, %d, %d, %d, %d, %d)\n",
	     mute[0], mute[1], mute[2], mute[3],
	     mute[4], mute[5], mute[6], mute[7]
	    );

#if SUPERFEMTO_API_VERSION < SUPERFEMTO_API(3,6,0)
	LOGP(DL1C, LOGL_ERROR, "RF-MUTE.req not supported by SuperFemto\n");
	return -ENOTSUP;
#else
	sysp->id = SuperFemto_PrimId_MuteRfReq;
	memcpy(sysp->u.muteRfReq.u8Mute, mute, sizeof(sysp->u.muteRfReq.u8Mute));
	/* save for later use */
	memcpy(hdl->last_rf_mute, mute, sizeof(hdl->last_rf_mute));

	return l1if_req_compl(hdl, msg, mute_rf_compl_cb);
#endif /* < 3.6.0 */
}

/* call-back on arrival of DSP+FPGA version + band capability */
static int info_compl_cb(struct gsm_bts_trx *trx, struct msgb *resp)
{
	SuperFemto_Prim_t *sysp = msgb_sysprim(resp);
	SuperFemto_SystemInfoCnf_t *sic = &sysp->u.systemInfoCnf;
	struct femtol1_hdl *fl1h = trx_femtol1_hdl(trx);
	int rc;

	fl1h->hw_info.dsp_version[0] = sic->dspVersion.major;
	fl1h->hw_info.dsp_version[1] = sic->dspVersion.minor;
	fl1h->hw_info.dsp_version[2] = sic->dspVersion.build;

	fl1h->hw_info.fpga_version[0] = sic->fpgaVersion.major;
	fl1h->hw_info.fpga_version[1] = sic->fpgaVersion.minor;
	fl1h->hw_info.fpga_version[2] = sic->fpgaVersion.build;

	LOGP(DL1C, LOGL_INFO, "DSP v%u.%u.%u, FPGA v%u.%u.%u\n",
		sic->dspVersion.major, sic->dspVersion.minor,
		sic->dspVersion.build, sic->fpgaVersion.major,
		sic->fpgaVersion.minor, sic->fpgaVersion.build);

#ifdef HW_SYSMOBTS_V1
	if (sic->rfBand.gsm850)
		fl1h->hw_info.band_support |= GSM_BAND_850;
	if (sic->rfBand.gsm900)
		fl1h->hw_info.band_support |= GSM_BAND_900;
	if (sic->rfBand.dcs1800)
		fl1h->hw_info.band_support |= GSM_BAND_1800;
	if (sic->rfBand.pcs1900)
		fl1h->hw_info.band_support |= GSM_BAND_1900;
#endif

	if (!(fl1h->hw_info.band_support & trx->bts->band))
		LOGP(DL1C, LOGL_FATAL, "BTS band %s not supported by hw\n",
		     gsm_band_name(trx->bts->band));

#if SUPERFEMTO_API_VERSION >= SUPERFEMTO_API(2,4,0)
	/* load calibration tables (if we know their path) */
	rc = calib_load(fl1h);
	if (rc < 0)
		LOGP(DL1C, LOGL_ERROR, "Operating without calibration; "
			"unable to load tables!\n");
#else
	LOGP(DL1C, LOGL_NOTICE, "Operating without calibration "
		"as software was compiled against old header files\n");
#endif

	msgb_free(resp);

	/* FIXME: clock related */
	return 0;
}

/* request DSP+FPGA code versions + band capability */
static int l1if_get_info(struct femtol1_hdl *hdl)
{
	struct msgb *msg = sysp_msgb_alloc();
	SuperFemto_Prim_t *sysp = msgb_sysprim(msg);

	sysp->id = SuperFemto_PrimId_SystemInfoReq;

	return l1if_req_compl(hdl, msg, info_compl_cb);
}

static int reset_compl_cb(struct gsm_bts_trx *trx, struct msgb *resp)
{
	struct femtol1_hdl *fl1h = trx_femtol1_hdl(trx);
	SuperFemto_Prim_t *sysp = msgb_sysprim(resp);
	GsmL1_Status_t status = sysp->u.layer1ResetCnf.status;

	LOGP(DL1C, LOGL_NOTICE, "Rx L1-RESET.conf (status=%s)\n",
		get_value_string(femtobts_l1status_names, status));

	msgb_free(resp);

	/* If we're coming out of reset .. */
	if (status != GsmL1_Status_Success) {
		LOGP(DL1C, LOGL_FATAL, "L1-RESET.conf with status %s\n",
			get_value_string(femtobts_l1status_names, status));
		bts_shutdown(trx->bts, "L1-RESET failure");
	}

	/* as we cannot get the current DSP trace flags, we simply
	 * set them to zero (or whatever dsp_trace_f has been initialized to */
	l1if_set_trace_flags(fl1h, fl1h->dsp_trace_f);

	/* obtain version information on DSP/FPGA and band capabilities */
	l1if_get_info(fl1h);

	/* otherwise, request activation of RF board */
	l1if_activate_rf(fl1h, 1);

	return 0;
}

int l1if_reset(struct femtol1_hdl *hdl)
{
	struct msgb *msg = sysp_msgb_alloc();
	SuperFemto_Prim_t *sysp = msgb_sysprim(msg);
	sysp->id = SuperFemto_PrimId_Layer1ResetReq;

	return l1if_req_compl(hdl, msg, reset_compl_cb);
}

/* set the trace flags within the DSP */
int l1if_set_trace_flags(struct femtol1_hdl *hdl, uint32_t flags)
{
	struct msgb *msg = sysp_msgb_alloc();
	SuperFemto_Prim_t *sysp = msgb_sysprim(msg);

	LOGP(DL1C, LOGL_INFO, "Tx SET-TRACE-FLAGS.req (0x%08x)\n",
		flags);

	sysp->id = SuperFemto_PrimId_SetTraceFlagsReq;
	sysp->u.setTraceFlagsReq.u32Tf = flags;

	hdl->dsp_trace_f = flags;

	/* There is no confirmation we could wait for */
	return osmo_wqueue_enqueue(&hdl->write_q[MQ_SYS_WRITE], msg);
}

/* get those femtol1_hdl.hw_info elements that sre in EEPROM */
static int get_hwinfo_eeprom(struct femtol1_hdl *fl1h)
{
	eeprom_SysInfo_t sysinfo;
	int val, rc;

	rc = sysmobts_par_get_int(SYSMOBTS_PAR_MODEL_NR, &val);
	if (rc < 0)
		return rc;
	fl1h->hw_info.model_nr = val;

	rc = sysmobts_par_get_int(SYSMOBTS_PAR_MODEL_FLAGS, &val);
	if (rc < 0)
		return rc;
	fl1h->hw_info.model_flags = val;

	rc = sysmobts_par_get_int(SYSMOBTS_PAR_TRX_NR, &val);
	if (rc < 0)
		return rc;
	fl1h->hw_info.trx_nr = val;

	rc = eeprom_ReadSysInfo(&sysinfo);
	if (rc != EEPROM_SUCCESS) {
		/* some early units don't yet have the EEPROM
		 * information structure */
		LOGP(DL1C, LOGL_ERROR, "Unable to read band support "
			"from EEPROM, assuming all bands\n");
		fl1h->hw_info.band_support = GSM_BAND_850 | GSM_BAND_900 | GSM_BAND_1800 | GSM_BAND_1900;
		return 0;
	}

	if (sysinfo.u8GSM850)
		fl1h->hw_info.band_support |= GSM_BAND_850;
	if (sysinfo.u8GSM900)
		fl1h->hw_info.band_support |= GSM_BAND_900;
	if (sysinfo.u8DCS1800)
	        fl1h->hw_info.band_support |= GSM_BAND_1800;
	if (sysinfo.u8PCS1900)
		fl1h->hw_info.band_support |= GSM_BAND_1900;

	return 0;
}

struct femtol1_hdl *l1if_open(void *priv)
{
	struct femtol1_hdl *fl1h;
	int rc;

#ifndef HW_SYSMOBTS_V1
	LOGP(DL1C, LOGL_INFO, "sysmoBTSv2 L1IF compiled against API headers "
			"v%u.%u.%u\n", SUPERFEMTO_API_VERSION >> 16,
			(SUPERFEMTO_API_VERSION >> 8) & 0xff,
			 SUPERFEMTO_API_VERSION & 0xff);
#else
	LOGP(DL1C, LOGL_INFO, "sysmoBTSv1 L1IF compiled against API headers "
			"v%u.%u.%u\n", FEMTOBTS_API_VERSION >> 16,
			(FEMTOBTS_API_VERSION >> 8) & 0xff,
			 FEMTOBTS_API_VERSION & 0xff);
#endif

	fl1h = talloc_zero(priv, struct femtol1_hdl);
	if (!fl1h)
		return NULL;
	INIT_LLIST_HEAD(&fl1h->wlc_list);

	fl1h->priv = priv;
	fl1h->clk_cal = 0;
	fl1h->clk_use_eeprom = 1;
	fl1h->ul_power_target = -75;	/* dBm default */
	fl1h->min_qual_rach = MIN_QUAL_RACH;
	fl1h->min_qual_norm = MIN_QUAL_NORM;
	get_hwinfo_eeprom(fl1h);
#if SUPERFEMTO_API_VERSION >= SUPERFEMTO_API(2,1,0)
	if (fl1h->hw_info.model_nr == 2050) {
		/* On the sysmoBTS 2050, we don't have an OCXO but
		 * always slave our clock to the GPS receiver */
		fl1h->clk_src = SuperFemto_ClkSrcId_GpsPps;
		LOGP(DL1C, LOGL_INFO, "Clock source defaulting to GPS 1PPS "
			"on sysmoBTS 2050\n");
	} else {
		/* default clock source: OCXO */
		fl1h->clk_src = SuperFemto_ClkSrcId_Ocxo;
	}
#else
	fl1h->clk_src = SF_CLKSRC_OCXO;
#endif

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

	fl1h->gsmtap = gsmtap_source_init("localhost", GSMTAP_UDP_PORT, 1);
	if (fl1h->gsmtap)
		gsmtap_source_add_sink(fl1h->gsmtap);

	return fl1h;
}

int l1if_close(struct femtol1_hdl *fl1h)
{
	l1if_transport_close(MQ_L1_WRITE, fl1h);
	l1if_transport_close(MQ_SYS_WRITE, fl1h);
	return 0;
}
