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
#include <osmocom/gsm/gsm_utils.h>
#include <osmocom/gsm/lapdm.h>

#include <osmo-bts/logging.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/oml.h>
#include <osmo-bts/gsm_data.h>
#include <osmo-bts/paging.h>
#include <osmo-bts/measurement.h>

#include <sysmocom/femtobts/femtobts.h>
#include <sysmocom/femtobts/gsml1prim.h>
#include <sysmocom/femtobts/gsml1const.h>
#include <sysmocom/femtobts/gsml1types.h>

#include "femtobts.h"
#include "l1_if.h"
#include "l1_transp.h"

/* FIXME: make threshold configurable */
#define MIN_QUAL_RACH	 1.0f	/* at least  1 dB C/I */
#define MIN_QUAL_NORM	-0.5f	/* at least -1 dB C/I */

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
		FemtoBts_Prim_t *sysp = msgb_sysprim(msg);

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

	/* schedule a timer for 10 seconds. If DSP fails to respond, we terminate */
	wlc->timer.data = wlc;
	wlc->timer.cb = l1if_req_timeout;
	osmo_timer_schedule(&wlc->timer, timeout_secs, 0);

	return 0;
}

/* allocate a msgb containing a GsmL1_Prim_t */
struct msgb *l1p_msgb_alloc(void)
{
	struct msgb *msg = msgb_alloc(sizeof(GsmL1_Prim_t), "l1_prim");

	if (msg)
		msg->l1h = msgb_put(msg, sizeof(GsmL1_Prim_t));

	return msg;
}

/* allocate a msgb containing a FemtoBts_Prim_t */
struct msgb *sysp_msgb_alloc(void)
{
	struct msgb *msg = msgb_alloc(sizeof(FemtoBts_Prim_t), "sys_prim");

	if (msg)
		msg->l1h = msgb_put(msg, sizeof(FemtoBts_Prim_t));

	return msg;
}

/* prepare a PH-DATA.req primitive in response to a PH-RTS.ind */
static struct msgb *alloc_ph_data_req(GsmL1_PhReadyToSendInd_t *rts_ind)
{
	struct msgb *msg = l1p_msgb_alloc();
	GsmL1_Prim_t *l1p = msgb_l1prim(msg);
	GsmL1_PhDataReq_t *data_req = &l1p->u.phDataReq;

	l1p->id = GsmL1_PrimId_PhDataReq;

	/* copy fields from PH-RSS.ind */
	data_req->hLayer1	= rts_ind->hLayer1;
	data_req->u8Tn 		= rts_ind->u8Tn;
	data_req->u32Fn		= rts_ind->u32Fn;
	data_req->sapi		= rts_ind->sapi;
	data_req->subCh		= rts_ind->subCh;
	data_req->u8BlockNbr	= rts_ind->u8BlockNbr;

	return msg;
}

/* obtain a ptr to the lapdm_channel for a given hLayer2 */
static struct lapdm_channel *
get_lapdm_chan_by_hl2(struct gsm_bts_trx *trx, uint32_t hLayer2)
{
	struct gsm_lchan *lchan;

	lchan = l1if_hLayer2_to_lchan(trx, hLayer2);
	if (!lchan)
		return NULL;

	return &lchan->lapdm_ch;
}


static const uint8_t fill_frame[GSM_MACBLOCK_LEN] = {
	0x01, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B,
	0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B,
	0x2B, 0x2B, 0x2B
};

static int handle_ph_readytosend_ind(struct femtol1_hdl *fl1,
				     GsmL1_PhReadyToSendInd_t *rts_ind)
{
	struct msgb *resp_msg = alloc_ph_data_req(rts_ind);
	struct gsm_bts_trx *trx = fl1->priv;
	struct gsm_bts *bts = trx->bts;
	struct gsm_bts_role_bts *btsb = bts->role;
	GsmL1_Prim_t *l1p = msgb_l1prim(resp_msg);
	GsmL1_PhDataReq_t *data_req = &l1p->u.phDataReq;
	GsmL1_PhEmptyFrameReq_t *empty_req = &l1p->u.phEmptyFrameReq;
	GsmL1_MsgUnitParam_t *msu_param = &data_req->msgUnitParam;
	struct lapdm_channel *lc;
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

	/* copy over parameters from PH-RTS.ind into PH-DATA.req */
	data_req->hLayer1 = rts_ind->hLayer1;
	data_req->u8Tn = rts_ind->u8Tn;
	data_req->u32Fn = rts_ind->u32Fn;
	data_req->sapi = rts_ind->sapi;
	data_req->subCh = rts_ind->subCh;
	data_req->u8BlockNbr = rts_ind->u8BlockNbr;
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
		lchan = l1if_hLayer2_to_lchan(trx, rts_ind->hLayer2);
		le = &lchan->lapdm_ch.lapdm_acch;
		rc = lapdm_phsap_dequeue_prim(le, &pp);
		if (rc < 0) {
			/* No SACCH data from LAPDM pending, send SACCH filling */
			uint8_t *si = lchan_sacch_get(lchan, &g_time);
			if (si) {
				/* The +2 is empty space where the DSP inserts the L1 hdr */
				memcpy(msu_param->u8Buffer+2, si, GSM_MACBLOCK_LEN-2);
			} else
				memcpy(msu_param->u8Buffer, fill_frame, GSM_MACBLOCK_LEN);
		} else {
			/* The +2 is empty space where the DSP inserts the L1 hdr */
			memcpy(msu_param->u8Buffer+2, pp.oph.msg->data, GSM_MACBLOCK_LEN-2);
			msgb_free(pp.oph.msg);
		}
		break;
	case GsmL1_Sapi_Sdcch:
		/* resolve the L2 entity using rts_ind->hLayer2 */
		lc = get_lapdm_chan_by_hl2(trx, rts_ind->hLayer2);
		le = &lc->lapdm_dcch;
		rc = lapdm_phsap_dequeue_prim(le, &pp);
		if (rc < 0)
			memcpy(msu_param->u8Buffer, fill_frame, GSM_MACBLOCK_LEN);
		else {
			memcpy(msu_param->u8Buffer, pp.oph.msg->data, GSM_MACBLOCK_LEN);
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
#warning Send actual speech data on the TCH
		goto empty_frame;
		break;
	case GsmL1_Sapi_FacchF:
		/* resolve the L2 entity using rts_ind->hLayer2 */
		lc = get_lapdm_chan_by_hl2(trx, rts_ind->hLayer2);
		le = &lc->lapdm_dcch;
		rc = lapdm_phsap_dequeue_prim(le, &pp);
		if (rc < 0)
			goto empty_frame;
		else {
			data_req->sapi = GsmL1_Sapi_FacchF;
			memcpy(msu_param->u8Buffer, pp.oph.msg->data, GSM_MACBLOCK_LEN);
			msgb_free(pp.oph.msg);
		}
		break;
		/* we should never receive a request here */
	default:
		memcpy(msu_param->u8Buffer, fill_frame, GSM_MACBLOCK_LEN);
		break;
	}
tx:
	/* transmit */
	osmo_wqueue_enqueue(&fl1->write_q[MQ_L1_WRITE], resp_msg);

	return 0;

empty_frame:
	/* in case we decide to send an empty frame... */
	memset(l1p, 0, sizeof(*l1p));
	l1p->id = GsmL1_PrimId_PhEmptyFrameReq;
	empty_req->hLayer1 = rts_ind->hLayer1;
	empty_req->u8Tn = rts_ind->u8Tn;
	empty_req->u32Fn = rts_ind->u32Fn;
	empty_req->sapi = rts_ind->sapi;
	empty_req->subCh = rts_ind->subCh;
	empty_req->u8BlockNbr = rts_ind->u8BlockNbr;

	goto tx;
}

static int handle_mph_time_ind(struct femtol1_hdl *fl1,
				GsmL1_MphTimeInd_t *time_ind)
{
	/* Update our data structures with the current GSM time */
	gsm_fn2gsmtime(&fl1->gsm_time, time_ind->u32Fn);

	/* check if the measurement period of some lchan has ended
	 * and pre-compute the respective measurement */
	trx_meas_check_compute(fl1->priv, time_ind->u32Fn -1);

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

static void dump_meas_res(GsmL1_MeasParam_t *m)
{
	DEBUGPC(DL1C, ", Meas: RSSI %-3.2f dBm,  Qual %-3.2f dB,  "
		"BER %-3.2f,  Timing %d\n", m->fRssi, m->fLinkQuality,
		m->fBer, m->i16BurstTiming);
}

static int process_meas_res(struct gsm_lchan *lchan, GsmL1_MeasParam_t *m)
{
	struct bts_ul_meas ulm;

	ulm.ta_offs_qbits = m->i16BurstTiming;
	ulm.ber10k = (unsigned int) (m->fBer * 100);
	ulm.inv_rssi = (uint8_t) (m->fRssi * -1);

	return lchan_new_ul_meas(lchan, &ulm);
}

static int handle_ph_data_ind(struct femtol1_hdl *fl1, GsmL1_PhDataInd_t *data_ind)
{
	struct osmo_phsap_prim pp;
	struct gsm_lchan *lchan;
	struct lapdm_entity *le;
	struct msgb *msg;
	int rc;

	lchan = l1if_hLayer2_to_lchan(fl1->priv, data_ind->hLayer2);
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
	dump_meas_res(&data_ind->measParam);

	switch (data_ind->sapi) {
	case GsmL1_Sapi_Sacch:
		/* save the SACCH L1 header in the lchan struct for RSL MEAS RES */
		if (data_ind->msgUnitParam.u8Size < 2)
			break;
		lchan->meas.l1_info[0] = data_ind->msgUnitParam.u8Buffer[0];
		lchan->meas.l1_info[1] = data_ind->msgUnitParam.u8Buffer[1];
		lchan->meas.flags |= LC_UL_M_F_L1_VALID;
		/* fall-through */
	case GsmL1_Sapi_Sdcch:
	case GsmL1_Sapi_FacchF:
	case GsmL1_Sapi_FacchH:
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
		/* FIXME: TCH speech frame handling */
		rc = 0;
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
	struct osmo_phsap_prim pp;
	struct lapdm_channel *lc;

	if (ra_ind->measParam.fLinkQuality < MIN_QUAL_RACH)
		return 0;

	DEBUGP(DL1C, "Rx PH-RA.ind");
	dump_meas_res(&ra_ind->measParam);

	lc = get_lapdm_chan_by_hl2(fl1->priv, ra_ind->hLayer2);
	if (!lc) {
		LOGP(DL1C, LOGL_ERROR, "unable to resolve LAPD channel by hLayer2\n");
		return -ENODEV;
	}

	osmo_prim_init(&pp.oph, SAP_GSM_PH, PRIM_PH_RACH,
			PRIM_OP_INDICATION, NULL);

	pp.u.rach_ind.ra = ra_ind->msgUnitParam.u8Buffer[0];
	pp.u.rach_ind.fn = ra_ind->u32Fn;
	/* FIXME: check for under/overflow / sign */
	if (ra_ind->measParam.i16BurstTiming <= 0 ||
	    ra_ind->measParam.i16BurstTiming > 63 * 4)
		pp.u.rach_ind.acc_delay = 0;
	else
		pp.u.rach_ind.acc_delay = ra_ind->measParam.i16BurstTiming >> 2;

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
		rc = handle_ph_data_ind(fl1, &l1p->u.phDataInd);
		break;
	case GsmL1_PrimId_PhRaInd:
		rc = handle_ph_ra_ind(fl1, &l1p->u.phRaInd);
		break;
	default:
		break;
	}

	msgb_free(msg);
	return rc;
}

int l1if_handle_l1prim(struct femtol1_hdl *fl1h, struct msgb *msg)
{
	GsmL1_Prim_t *l1p = msgb_l1prim(msg);
	struct wait_l1_conf *wlc;
	int rc;

	switch (l1p->id) {
	case GsmL1_PrimId_MphTimeInd:
		/* silent, don't clog the log file */
		break;
	default:
		LOGP(DL1P, LOGL_DEBUG, "Rx L1 prim %s\n",
			get_value_string(femtobts_l1prim_names, l1p->id));
	}

	/* check if this is a resposne to a sync-waiting request */
	llist_for_each_entry(wlc, &fl1h->wlc_list, list) {
		/* the limitation here is that we cannot have multiple callers
		 * sending the same primitive */
		if (wlc->is_sys_prim == 0 && l1p->id == wlc->conf_prim_id) {
			llist_del(&wlc->list);
			rc = wlc->cb(msg, wlc->cb_data);
			release_wlc(wlc);
			return rc;
		}
	}

	/* if we reach here, it is not a Conf for a pending Req */
	return l1if_handle_ind(fl1h, msg);
}

int l1if_handle_sysprim(struct femtol1_hdl *fl1h, struct msgb *msg)
{
	FemtoBts_Prim_t *sysp = msgb_sysprim(msg);
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
			rc = wlc->cb(msg, wlc->cb_data);
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
	FemtoBts_Prim_t *sysp = msgb_sysprim(resp);
	struct femtol1_hdl *fl1h = data;
	struct gsm_bts_trx *trx = fl1h->priv;
	GsmL1_Status_t status;
	int on = 0;
	unsigned int i;

	if (sysp->id == FemtoBts_PrimId_ActivateRfCnf)
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
		}
		/* signal availability */
		oml_mo_state_chg(&trx->mo, NM_OPSTATE_DISABLED, NM_AVSTATE_OK);
		oml_mo_tx_sw_act_rep(&trx->mo);
		oml_mo_state_chg(&trx->bb_transc.mo, -1, NM_AVSTATE_OK);
		oml_mo_tx_sw_act_rep(&trx->bb_transc.mo);

		for (i = 0; i < ARRAY_SIZE(trx->ts); i++)
			oml_mo_state_chg(&trx->ts[i].mo, NM_OPSTATE_DISABLED, NM_AVSTATE_DEPENDENCY);
	} else {
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
	FemtoBts_Prim_t *sysp = msgb_sysprim(msg);

	if (on) {
		sysp->id = FemtoBts_PrimId_ActivateRfReq;
		sysp->u.activateRfReq.u12ClkVc = 0xFFFF;
	} else {
		sysp->id = FemtoBts_PrimId_DeactivateRfReq;
	}

	return l1if_req_compl(hdl, msg, 1, activate_rf_compl_cb, hdl);
}

static int reset_compl_cb(struct msgb *resp, void *data)
{
	struct femtol1_hdl *fl1h = data;
	struct gsm_bts_trx *trx = fl1h->priv;
	FemtoBts_Prim_t *sysp = msgb_sysprim(resp);
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

	/* otherwise, request activation of RF board */
	l1if_activate_rf(fl1h, 1);

	return 0;
}

int l1if_reset(struct femtol1_hdl *hdl)
{
	struct msgb *msg = sysp_msgb_alloc();
	FemtoBts_Prim_t *sysp = msgb_sysprim(msg);
	sysp->id = FemtoBts_PrimId_Layer1ResetReq;

	return l1if_req_compl(hdl, msg, 1, reset_compl_cb, hdl);
}

/* set the trace flags within the DSP */
int l1if_set_trace_flags(struct femtol1_hdl *hdl, uint32_t flags)
{
	struct msgb *msg = sysp_msgb_alloc();
	FemtoBts_Prim_t *sysp = msgb_sysprim(msg);

	LOGP(DL1C, LOGL_INFO, "Tx SET-TRACE-FLAGS.req (0x%08x)\n",
		flags);

	sysp->id = FemtoBts_PrimId_SetTraceFlagsReq;
	sysp->u.setTraceFlagsReq.u32Tf = flags;

	hdl->dsp_trace_f = flags;

	/* There is no confirmation we could wait for */
	return osmo_wqueue_enqueue(&hdl->write_q[MQ_SYS_WRITE], msg);
}

struct femtol1_hdl *l1if_open(void *priv)
{
	struct femtol1_hdl *fl1h;
	int rc;

	fl1h = talloc_zero(priv, struct femtol1_hdl);
	if (!fl1h)
		return NULL;
	INIT_LLIST_HEAD(&fl1h->wlc_list);

	fl1h->priv = priv;

	rc = l1if_transport_open(fl1h);
	if (rc < 0) {
		talloc_free(fl1h);
		return NULL;
	}

	return fl1h;
}

int l1if_close(struct femtol1_hdl *fl1h)
{
	return l1if_transport_close(fl1h);
}
