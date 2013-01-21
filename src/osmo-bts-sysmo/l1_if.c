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

#include <osmocom/trau/osmo_ortp.h>

#include <osmo-bts/logging.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/oml.h>
#include <osmo-bts/gsm_data.h>
#include <osmo-bts/paging.h>
#include <osmo-bts/measurement.h>
#include <osmo-bts/pcu_if.h>

#include <sysmocom/femtobts/superfemto.h>
#include <sysmocom/femtobts/gsml1prim.h>
#include <sysmocom/femtobts/gsml1const.h>
#include <sysmocom/femtobts/gsml1types.h>

#include "femtobts.h"
#include "l1_if.h"
#include "l1_transp.h"
#include "hw_misc.h"

extern int pcu_direct;

/* FIXME: make threshold configurable */
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
				0, 0, data_ind->msgUnitParam.u8Buffer + skip,
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

/* send a request primitive to the L1 and schedule completion call-back */
int l1if_req_compl(struct femtol1_hdl *fl1h, struct msgb *msg,
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

int l1if_gsm_req_compl(struct femtol1_hdl *fl1h, struct msgb *msg,
		   l1if_compl_cb *cb)
{
	return l1if_req_compl(fl1h, msg, 0, cb, fl1h->priv);
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

/* obtain a ptr to the lapdm_channel for a given hLayer2 */
static struct lapdm_channel *
get_lapdm_chan_by_hl2(struct gsm_bts_trx *trx, uint32_t hLayer2)
{
	struct gsm_lchan *lchan;

	lchan = l1if_hLayer_to_lchan(trx, hLayer2);
	if (!lchan)
		return NULL;

	return &lchan->lapdm_ch;
}

/* check if the message is a GSM48_MT_RR_CIPH_M_CMD, and if yes, enable
 * uni-directional de-cryption on the uplink. We need this ugly layering
 * violation as we have no way of passing down L3 metadata (RSL CIPHERING CMD)
 * to this point in L1 */
static int check_for_ciph_cmd(struct femtol1_hdl *fl1h,
			      struct msgb *msg, struct gsm_lchan *lchan)
{

	/* only do this if we are in the right state */
	switch (lchan->ciph_state) {
	case LCHAN_CIPH_NONE:
	case LCHAN_CIPH_RX_REQ:
		break;
	default:
		return 0;
	}

	/* First byte (Address Field) of LAPDm header) */
	if (msg->data[0] != 0x03)
		return 0;
	/* First byte (protocol discriminator) of RR */
	if ((msg->data[3] & 0xF) != GSM48_PDISC_RR)
		return 0;
	/* 2nd byte (msg type) of RR */
	if ((msg->data[4] & 0x3F) != GSM48_MT_RR_CIPH_M_CMD)
		return 0;

	lchan->ciph_state = LCHAN_CIPH_RX_REQ;
	l1if_set_ciphering(fl1h, lchan, 0);

	return 1;
}

static const uint8_t fill_frame[GSM_MACBLOCK_LEN] = {
	0x03, 0x03, 0x01, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B,
	0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B,
	0x2B, 0x2B, 0x2B
};

static int handle_ph_readytosend_ind(struct femtol1_hdl *fl1,
				     GsmL1_PhReadyToSendInd_t *rts_ind)
{
	struct gsm_bts_trx *trx = fl1->priv;
	struct gsm_bts *bts = trx->bts;
	struct gsm_bts_role_bts *btsb = bts->role;
	struct msgb *resp_msg;
	GsmL1_PhDataReq_t *data_req;
	GsmL1_MsgUnitParam_t *msu_param;
	struct lapdm_entity *le;
	struct gsm_lchan *lchan;
	struct gsm_time g_time;
	uint32_t t3p;
	uint8_t *si;
	struct osmo_phsap_prim pp;
	int rc;

	gsm_fn2gsmtime(&g_time, rts_ind->u32Fn);

	DEBUGP(DL1P, "Rx PH-RTS.ind %02u/%02u/%02u SAPI=%s\n",
		g_time.t1, g_time.t2, g_time.t3,
		get_value_string(femtobts_l1sapi_names, rts_ind->sapi));

	/* In case of TCH downlink trasnmission, we already have a l1
	 * primitive msgb pre-allocated and pre-formatted in the
	 * dl_tch_queue.  All we need to do is to pull it off the queue
	 * and transmit it */
	switch (rts_ind->sapi) {
	case GsmL1_Sapi_TchF:
	case GsmL1_Sapi_TchH:
		/* resolve the L2 entity using rts_ind->hLayer2 */
		lchan = l1if_hLayer_to_lchan(trx, rts_ind->hLayer2);
		if (!lchan)
			break;

		if (!lchan->loopback && lchan->abis_ip.rtp_socket) {
			osmo_rtp_socket_poll(lchan->abis_ip.rtp_socket);
			/* FIXME: we _assume_ that we never miss TDMA
			 * frames and that we always get to this point
			 * for every to-be-transmitted voice frame.  A
			 * better solution would be to compute
			 * rx_user_ts based on how many TDMA frames have
			 * elapsed since the last call */
			lchan->abis_ip.rtp_socket->rx_user_ts += GSM_RTP_DURATION;
		}
		/* get a msgb from the dl_tx_queue */
		resp_msg = msgb_dequeue(&lchan->dl_tch_queue);
		/* if there is none, try to generate empty TCH frame
		 * like AMR SID_BAD */
		if (!resp_msg) {
			LOGP(DL1C, LOGL_DEBUG, "%s DL TCH Tx queue underrun\n",
				gsm_lchan_name(lchan));
			resp_msg = gen_empty_tch_msg(lchan);
			/* if there really is none, break here and send empty */
			if (!resp_msg)
				break;
		}

		/* fill header */
		data_req_from_rts_ind(msgb_l1prim(resp_msg), rts_ind);
		/* actually transmit it */
		goto tx;
		break;
	case GsmL1_Sapi_Pdtch:
	case GsmL1_Sapi_Pacch:
		return pcu_tx_rts_req(&trx->ts[rts_ind->u8Tn], 0,
			rts_ind->u32Fn, rts_ind->u16Arfcn, rts_ind->u8BlockNbr);
	case GsmL1_Sapi_Ptcch:
		return pcu_tx_rts_req(&trx->ts[rts_ind->u8Tn], 1,
			rts_ind->u32Fn, rts_ind->u16Arfcn, rts_ind->u8BlockNbr);
	default:
		break;
	}

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
	case GsmL1_Sapi_Bcch:
		/* get them from bts->si_buf[] */
		si = bts_sysinfo_get(bts, &g_time);
		if (si)
			memcpy(msu_param->u8Buffer, si, GSM_MACBLOCK_LEN);
		else
			memcpy(msu_param->u8Buffer, fill_frame, GSM_MACBLOCK_LEN);
		break;
	case GsmL1_Sapi_Sacch:
		/* resolve the L2 entity using rts_ind->hLayer2 */
		lchan = l1if_hLayer_to_lchan(trx, rts_ind->hLayer2);
		le = &lchan->lapdm_ch.lapdm_acch;
		rc = lapdm_phsap_dequeue_prim(le, &pp);
		if (rc < 0) {
			/* No SACCH data from LAPDM pending, send SACCH filling */
			uint8_t *si = lchan_sacch_get(lchan, &g_time);
			if (si) {
				/* The +2 is empty space where the DSP inserts the L1 hdr */
				memcpy(msu_param->u8Buffer+2, si, GSM_MACBLOCK_LEN-2);
			} else
				memcpy(msu_param->u8Buffer+2, fill_frame, GSM_MACBLOCK_LEN-2);
		} else {
			/* The +2 is empty space where the DSP inserts the L1 hdr */
			memcpy(msu_param->u8Buffer+2, pp.oph.msg->data, GSM_MACBLOCK_LEN-2);
			msgb_free(pp.oph.msg);
		}
		break;
	case GsmL1_Sapi_Sdcch:
		/* resolve the L2 entity using rts_ind->hLayer2 */
		lchan = l1if_hLayer_to_lchan(trx, rts_ind->hLayer2);
		le = &lchan->lapdm_ch.lapdm_dcch;
		rc = lapdm_phsap_dequeue_prim(le, &pp);
		if (rc < 0)
			memcpy(msu_param->u8Buffer, fill_frame, GSM_MACBLOCK_LEN);
		else {
			memcpy(msu_param->u8Buffer, pp.oph.msg->data, GSM_MACBLOCK_LEN);
			/* check if it is a RR CIPH MODE CMD. if yes, enable RX ciphering */
			check_for_ciph_cmd(fl1, pp.oph.msg, lchan);
			msgb_free(pp.oph.msg);
		}
		break;
	case GsmL1_Sapi_Agch:
		/* special queue of messages from IMM ASS CMD */
		{
			struct msgb *msg = bts_agch_dequeue(bts);
			if (!msg)
				memcpy(msu_param->u8Buffer, fill_frame, GSM_MACBLOCK_LEN);
			else {
				memcpy(msu_param->u8Buffer, msg->data, msg->len);
				msgb_free(msg);
			}
		}
		break;
	case GsmL1_Sapi_Pch:
		rc = paging_gen_msg(btsb->paging_state, msu_param->u8Buffer, &g_time);
		break;
	case GsmL1_Sapi_TchF:
	case GsmL1_Sapi_TchH:
		/* only hit in case we have a RTP underflow, as real TCH
		 * frames are handled way above */
		goto empty_frame;
		break;
	case GsmL1_Sapi_FacchF:
	case GsmL1_Sapi_FacchH:
		/* resolve the L2 entity using rts_ind->hLayer2 */
		lchan = l1if_hLayer_to_lchan(trx, rts_ind->hLayer2);
		le = &lchan->lapdm_ch.lapdm_dcch;
		rc = lapdm_phsap_dequeue_prim(le, &pp);
		if (rc < 0)
			goto empty_frame;
		else {
			memcpy(msu_param->u8Buffer, pp.oph.msg->data, GSM_MACBLOCK_LEN);
			/* check if it is a RR CIPH MODE CMD. if yes, enable RX ciphering */
			check_for_ciph_cmd(fl1, pp.oph.msg, lchan);
			msgb_free(pp.oph.msg);
		}
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

	return 0;

empty_frame:
	/* in case we decide to send an empty frame... */
	empty_req_from_rts_ind(msgb_l1prim(resp_msg), rts_ind);

	goto tx;
}

static int handle_mph_time_ind(struct femtol1_hdl *fl1,
				GsmL1_MphTimeInd_t *time_ind)
{
	struct gsm_bts_trx *trx = fl1->priv;
	struct gsm_bts *bts = trx->bts;
	struct gsm_bts_role_bts *btsb = bts->role;

	int frames_expired = time_ind->u32Fn - fl1->gsm_time.fn;

	/* update time on PCU interface */
	pcu_tx_time_ind(time_ind->u32Fn);

	/* Update our data structures with the current GSM time */
	gsm_fn2gsmtime(&fl1->gsm_time, time_ind->u32Fn);

	/* check if the measurement period of some lchan has ended
	 * and pre-compute the respective measurement */
	trx_meas_check_compute(fl1->priv, time_ind->u32Fn -1);

	/* increment the primitive count for the alive timer */
	fl1->alive_prim_cnt++;

	/* increment number of RACH slots that have passed by since the
	 * last time indication */
	if (trx == bts->c0) {
		unsigned int num_rach_per_frame;
		/* 27 / 51 taken from TS 05.01 Figure 3 */
		if (bts->c0->ts[0].pchan == GSM_PCHAN_CCCH_SDCCH4)
			num_rach_per_frame = 27;
		else
			num_rach_per_frame = 51;

		btsb->load.rach.total += frames_expired * num_rach_per_frame;
	}

	return 0;
}

/* determine LAPDm entity inside LAPDm channel for given L1 sapi */
static struct lapdm_entity *le_by_l1_sapi(struct lapdm_channel *lc, GsmL1_Sapi_t sapi)
{
	switch (sapi) {
	case GsmL1_Sapi_Sacch:
		return &lc->lapdm_acch;
	default:
		return &lc->lapdm_dcch;
	}
}

static uint8_t gen_link_id(GsmL1_Sapi_t l1_sapi, uint8_t lapdm_sapi)
{
	uint8_t c_bits = 0;

	if (l1_sapi == GsmL1_Sapi_Sacch)
		c_bits = 0x40;

	return c_bits | (lapdm_sapi & 7);
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
	struct osmo_phsap_prim pp;
	struct gsm_lchan *lchan;
	struct lapdm_entity *le;
	struct msgb *msg;
	int rc = 0;

	ul_to_gsmtap(fl1, l1p_msg);

	lchan = l1if_hLayer_to_lchan(fl1->priv, data_ind->hLayer2);
	if (!lchan) {
		LOGP(DL1C, LOGL_ERROR, "unable to resolve lchan by hLayer2\n");
		return -ENODEV;
	}

	process_meas_res(lchan, &data_ind->measParam);

	if (data_ind->measParam.fLinkQuality < MIN_QUAL_NORM)
		return 0;

	DEBUGP(DL1C, "Rx PH-DATA.ind %s (hL2 %08x): %s",
		get_value_string(femtobts_l1sapi_names, data_ind->sapi),
		data_ind->hLayer2,
		osmo_hexdump(data_ind->msgUnitParam.u8Buffer,
			     data_ind->msgUnitParam.u8Size));
	dump_meas_res(LOGL_DEBUG, &data_ind->measParam);

	switch (data_ind->sapi) {
	case GsmL1_Sapi_Sacch:
		/* save the SACCH L1 header in the lchan struct for RSL MEAS RES */
		if (data_ind->msgUnitParam.u8Size < 2) {
			LOGP(DL1C, LOGL_NOTICE, "SACCH with size %u<2 !?!\n",
				data_ind->msgUnitParam.u8Size);
			break;
		}
		/* Some brilliant engineer decided that the ordering of
		 * fields on the Um interface is different from the
		 * order of fields in RLS. See TS 04.04 (Chapter 7.2)
		 * vs. TS 08.58 (Chapter 9.3.10). */
		lchan->meas.l1_info[0] = data_ind->msgUnitParam.u8Buffer[0] << 3;
		lchan->meas.l1_info[0] |= ((data_ind->msgUnitParam.u8Buffer[0] >> 5) & 1) << 2;
		lchan->meas.l1_info[1] = data_ind->msgUnitParam.u8Buffer[1];
		lchan->meas.flags |= LC_UL_M_F_L1_VALID;
		/* fall-through */
	case GsmL1_Sapi_Sdcch:
	case GsmL1_Sapi_FacchF:
	case GsmL1_Sapi_FacchH:
		/* Check and Re-check for the SACCH */
		if (data_ind->msgUnitParam.u8Size == 0) {
			LOGP(DL1C, LOGL_NOTICE, "%s %s data is null.\n",
				gsm_lchan_name(lchan),
				get_value_string(femtobts_l1sapi_names, data_ind->sapi));
			break;
		}

		/* if this is the first valid message after enabling Rx
		 * decryption, we have to enable Tx encryption */
		if (lchan->ciph_state == LCHAN_CIPH_RX_CONF) {
			/* HACK: check if it's an I frame, in order to
			 * ignore some still buffered/queued UI frames received
			 * before decryption was enabled */
			if (data_ind->msgUnitParam.u8Buffer[0] == 0x01 &&
			    (data_ind->msgUnitParam.u8Buffer[1] & 0x01) == 0) {
				l1if_set_ciphering(fl1, lchan, 1);
				lchan->ciph_state = LCHAN_CIPH_TXRX_REQ;
			}
		}

		/* SDCCH, SACCH and FACCH all go to LAPDm */
		le = le_by_l1_sapi(&lchan->lapdm_ch, data_ind->sapi);
		/* allocate and fill LAPDm primitive */
		msg = msgb_alloc_headroom(128, 64, "PH-DATA.ind");
		osmo_prim_init(&pp.oph, SAP_GSM_PH, PRIM_PH_DATA,
				PRIM_OP_INDICATION, msg);

		/* copy over actual MAC block */
		msg->l2h = msgb_put(msg, data_ind->msgUnitParam.u8Size);
		memcpy(msg->l2h, data_ind->msgUnitParam.u8Buffer,
			data_ind->msgUnitParam.u8Size);

		/* LAPDm requires those... */
		pp.u.data.chan_nr = gsm_lchan2chan_nr(lchan);
		pp.u.data.link_id = gen_link_id(data_ind->sapi, 0);

		/* feed into the LAPDm code of libosmogsm */
		rc = lapdm_phsap_up(&pp.oph, le);
		break;
	case GsmL1_Sapi_TchF:
	case GsmL1_Sapi_TchH:
		/* TCH speech frame handling */
		rc = l1if_tch_rx(lchan, l1p_msg);
		break;
	case GsmL1_Sapi_Pdtch:
	case GsmL1_Sapi_Pacch:
		/* drop incomplete UL block */
		if (data_ind->msgUnitParam.u8Buffer[0]
			!= GsmL1_PdtchPlType_Full)
			break;
		/* PDTCH / PACCH frame handling */
		rc = pcu_tx_data_ind(&trx->ts[data_ind->u8Tn], 0,
			data_ind->u32Fn, data_ind->u16Arfcn,
			data_ind->u8BlockNbr,
			data_ind->msgUnitParam.u8Buffer + 1,
			data_ind->msgUnitParam.u8Size - 1);
		break;
	case GsmL1_Sapi_Ptcch:
		/* PTCCH frame handling */
		rc = pcu_tx_data_ind(&trx->ts[data_ind->u8Tn], 1,
			data_ind->u32Fn, data_ind->u16Arfcn,
			data_ind->u8BlockNbr,
			data_ind->msgUnitParam.u8Buffer,
			data_ind->msgUnitParam.u8Size);
		break;
	default:
		LOGP(DL1C, LOGL_NOTICE, "Rx PH-DATA.ind for unknown L1 SAPI %s\n",
			get_value_string(femtobts_l1sapi_names, data_ind->sapi));
		break;
	}

	return rc;
}


static int handle_ph_ra_ind(struct femtol1_hdl *fl1, GsmL1_PhRaInd_t *ra_ind)
{
	struct gsm_bts_trx *trx = fl1->priv;
	struct gsm_bts *bts = trx->bts;
	struct gsm_bts_role_bts *btsb = bts->role;
	struct osmo_phsap_prim pp;
	struct lapdm_channel *lc;
	uint8_t acc_delay;

	/* increment number of busy RACH slots, if required */
	if (trx == bts->c0 &&
	    ra_ind->measParam.fRssi >= btsb->load.rach.busy_thresh)
		btsb->load.rach.busy++;

	if (ra_ind->measParam.fLinkQuality < MIN_QUAL_RACH)
		return 0;

	/* increment number of RACH slots with valid RACH burst */
	if (trx == bts->c0)
		btsb->load.rach.access++;

	DEBUGP(DL1C, "Rx PH-RA.ind");
	dump_meas_res(LOGL_DEBUG, &ra_ind->measParam);

	lc = get_lapdm_chan_by_hl2(fl1->priv, ra_ind->hLayer2);
	if (!lc) {
		LOGP(DL1C, LOGL_ERROR, "unable to resolve LAPD channel by hLayer2\n");
		return -ENODEV;
	}

	/* check for under/overflow / sign */
	if (ra_ind->measParam.i16BurstTiming < 0)
		acc_delay = 0;
	else
		acc_delay = ra_ind->measParam.i16BurstTiming >> 2;
	if (acc_delay > btsb->max_ta) {
		LOGP(DL1C, LOGL_INFO, "ignoring RACH request %u > max_ta(%u)\n",
		     acc_delay, btsb->max_ta);
		return 0;
	}

	/* check for packet access */
	if (trx == bts->c0
	 && (ra_ind->msgUnitParam.u8Buffer[0] & 0xf0) == 0x70) {
		LOGP(DL1C, LOGL_INFO, "RACH for packet access\n");
		return pcu_tx_rach_ind(bts, ra_ind->measParam.i16BurstTiming,
			ra_ind->msgUnitParam.u8Buffer[0], ra_ind->u32Fn);
	}

	osmo_prim_init(&pp.oph, SAP_GSM_PH, PRIM_PH_RACH,
			PRIM_OP_INDICATION, NULL);

	pp.u.rach_ind.ra = ra_ind->msgUnitParam.u8Buffer[0];
	pp.u.rach_ind.fn = ra_ind->u32Fn;
	pp.u.rach_ind.acc_delay = acc_delay;

	return lapdm_phsap_up(&pp.oph, &lc->lapdm_dcch);
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
		rc = handle_ph_readytosend_ind(fl1, &l1p->u.phReadyToSendInd);
		break;
	case GsmL1_PrimId_PhDataInd:
		rc = handle_ph_data_ind(fl1, &l1p->u.phDataInd, msg);
		break;
	case GsmL1_PrimId_PhRaInd:
		rc = handle_ph_ra_ind(fl1, &l1p->u.phRaInd);
		break;
	default:
		break;
	}

	/* Special return value '1' means: do not free */
	if (rc != 1)
		msgb_free(msg);

	return rc;
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
		/* the limitation here is that we cannot have multiple callers
		 * sending the same primitive */
		if (wlc->is_sys_prim == 0 && l1p->id == wlc->conf_prim_id) {
			llist_del(&wlc->list);
			if (wlc->cb)
				rc = wlc->cb(msg, wlc->cb_data);
			else
				rc = 0;
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
				rc = wlc->cb(msg, wlc->cb_data);
			else
				rc = 0;
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

static int activate_rf_compl_cb(struct msgb *resp, void *data)
{
	SuperFemto_Prim_t *sysp = msgb_sysprim(resp);
	struct gsm_bts_trx *trx = data;
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
			sysmobts_led_set(LED_RF_ACTIVE, 1);

		/* signal availability */
		oml_mo_state_chg(&trx->mo, NM_OPSTATE_DISABLED, NM_AVSTATE_OK);
		oml_mo_tx_sw_act_rep(&trx->mo);
		oml_mo_state_chg(&trx->bb_transc.mo, -1, NM_AVSTATE_OK);
		oml_mo_tx_sw_act_rep(&trx->bb_transc.mo);

		for (i = 0; i < ARRAY_SIZE(trx->ts); i++)
			oml_mo_state_chg(&trx->ts[i].mo, NM_OPSTATE_DISABLED, NM_AVSTATE_DEPENDENCY);
	} else {
		sysmobts_led_set(LED_RF_ACTIVE, 0);
		oml_mo_state_chg(&trx->mo, NM_OPSTATE_DISABLED, NM_AVSTATE_OFF_LINE);
		oml_mo_state_chg(&trx->bb_transc.mo, NM_OPSTATE_DISABLED, NM_AVSTATE_OFF_LINE);
	}

	talloc_free(resp);

	return 0;
}

/* activate or de-activate the entire RF-Frontend */
int l1if_activate_rf(struct femtol1_hdl *hdl, int on)
{
	struct msgb *msg = sysp_msgb_alloc();
	SuperFemto_Prim_t *sysp = msgb_sysprim(msg);

	if (on) {
		sysp->id = SuperFemto_PrimId_ActivateRfReq;
#ifdef HW_SYSMOBTS_V1
		sysp->u.activateRfReq.u12ClkVc = hdl->clk_cal;
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
		sysp->u.activateRfReq.rfTrx.iClkCor = hdl->clk_cal;
#if SUPERFEMTO_API_VERSION < SUPERFEMTO_API(2,4,0)
#if SUPERFEMTO_API_VERSION < SUPERFEMTO_API(2,1,0)
		sysp->u.activateRfReq.rfRx.u8ClkSrc = hdl->clk_src;
#else
		sysp->u.activateRfReq.rfRx.clkSrc = hdl->clk_src;
#endif /* 2.1.0 */
		sysp->u.activateRfReq.rfRx.iClkCor = hdl->clk_cal;
#endif /* API 2.4.0 */
#endif /* !HW_SYSMOBTS_V1 */
	} else {
		sysp->id = SuperFemto_PrimId_DeactivateRfReq;
	}

	return l1if_req_compl(hdl, msg, 1, activate_rf_compl_cb, hdl->priv);
}

/* call-back on arrival of DSP+FPGA version + band capability */
static int info_compl_cb(struct msgb *resp, void *data)
{
	SuperFemto_Prim_t *sysp = msgb_sysprim(resp);
	SuperFemto_SystemInfoCnf_t *sic = &sysp->u.systemInfoCnf;
	struct gsm_bts_trx *trx = data;
	struct femtol1_hdl *fl1h = trx_femtol1_hdl(trx);

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
#else
	fl1h->hw_info.band_support |= GSM_BAND_850 | GSM_BAND_900 | GSM_BAND_1800 | GSM_BAND_1900;
#endif

	if (!(fl1h->hw_info.band_support & trx->bts->band))
		LOGP(DL1C, LOGL_FATAL, "BTS band %s not supported by hw\n",
		     gsm_band_name(trx->bts->band));

	/* FIXME: clock related */
	return 0;
}

/* request DSP+FPGA code versions + band capability */
static int l1if_get_info(struct femtol1_hdl *hdl)
{
	struct msgb *msg = sysp_msgb_alloc();
	SuperFemto_Prim_t *sysp = msgb_sysprim(msg);

	sysp->id = SuperFemto_PrimId_SystemInfoReq;

	return l1if_req_compl(hdl, msg, 1, info_compl_cb, hdl->priv);
}

static int reset_compl_cb(struct msgb *resp, void *data)
{
	struct gsm_bts_trx *trx = data;
	struct femtol1_hdl *fl1h = trx_femtol1_hdl(trx);
	SuperFemto_Prim_t *sysp = msgb_sysprim(resp);
	GsmL1_Status_t status = sysp->u.layer1ResetCnf.status;

	LOGP(DL1C, LOGL_NOTICE, "Rx L1-RESET.conf (status=%s)\n",
		get_value_string(femtobts_l1status_names, status));

	talloc_free(resp);

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

#if SUPERFEMTO_API_VERSION >= SUPERFEMTO_API(2,1,0)
	/* load calibration tables (if we know their path) */
	if (fl1h->calib_path)
		calib_load(fl1h);
	else
#endif
		LOGP(DL1C, LOGL_NOTICE, "Operating without calibration tables!\n");

	/* otherwise, request activation of RF board */
	l1if_activate_rf(fl1h, 1);

	return 0;
}

int l1if_reset(struct femtol1_hdl *hdl)
{
	struct msgb *msg = sysp_msgb_alloc();
	SuperFemto_Prim_t *sysp = msgb_sysprim(msg);
	sysp->id = SuperFemto_PrimId_Layer1ResetReq;

	return l1if_req_compl(hdl, msg, 1, reset_compl_cb, hdl->priv);
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

/* send packet data request to L1 */
int l1if_pdch_req(struct gsm_bts_trx_ts *ts, int is_ptcch, uint32_t fn,
	uint16_t arfcn, uint8_t block_nr, uint8_t *data, uint8_t len)
{
	struct gsm_bts_trx *trx = ts->trx;
	struct femtol1_hdl *fl1h = trx_femtol1_hdl(trx);
	struct msgb *msg;
	GsmL1_Prim_t *l1p;
	GsmL1_PhDataReq_t *data_req;
	GsmL1_MsgUnitParam_t *msu_param;
	struct gsm_time g_time;

	gsm_fn2gsmtime(&g_time, fn);

	DEBUGP(DL1P, "TX packet data %02u/%02u/%02u is_ptcch=%d trx=%d ts=%d "
		"block_nr=%d, arfcn=%d, len=%d\n", g_time.t1, g_time.t2,
		g_time.t3, is_ptcch, ts->trx->nr, ts->nr, block_nr, arfcn, len);

	msg = l1p_msgb_alloc();
	l1p = msgb_l1prim(msg);
	l1p->id = GsmL1_PrimId_PhDataReq;
	data_req = &l1p->u.phDataReq;
	data_req->hLayer1 = fl1h->hLayer1;
	data_req->sapi = (is_ptcch) ? GsmL1_Sapi_Ptcch : GsmL1_Sapi_Pdtch;
	data_req->subCh = GsmL1_SubCh_NA;
	data_req->u8BlockNbr = block_nr;
	data_req->u8Tn = ts->nr;
	data_req->u32Fn = fn;
	msu_param = &data_req->msgUnitParam;
	msu_param->u8Size = len;
	memcpy(msu_param->u8Buffer, data, len);

	tx_to_gsmtap(fl1h, msg);

	/* transmit */
	osmo_wqueue_enqueue(&fl1h->write_q[MQ_L1_WRITE], msg);

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
	fl1h->ul_power_target = -75;	/* dBm default */
	/* default clock source: OCXO */
#if SUPERFEMTO_API_VERSION >= SUPERFEMTO_API(2,1,0)
	fl1h->clk_src = SuperFemto_ClkSrcId_Ocxo;
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
