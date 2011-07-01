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

#include <osmocom/core/talloc.h>
#include <osmocom/core/utils.h>

#include <sysmocom/femtobts/gsml1prim.h>
#include <sysmocom/femtobts/gsml1const.h>
#include <sysmocom/femtobts/gsml1types.h>

#include <osmo-bts/gsm_data.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/oml.h>

#include "l1_if.h"
#include "femtobts.h"

static const enum GsmL1_LogChComb_t pchan_to_logChComb[_GSM_PCHAN_MAX] = {
	[GSM_PCHAN_NONE]		= GsmL1_LogChComb_0,
	[GSM_PCHAN_CCCH]		= GsmL1_LogChComb_IV,
	[GSM_PCHAN_CCCH_SDCCH4] 	= GsmL1_LogChComb_V,
	[GSM_PCHAN_TCH_F]		= GsmL1_LogChComb_I,
	[GSM_PCHAN_TCH_H]		= GsmL1_LogChComb_II,
	[GSM_PCHAN_SDCCH8_SACCH8C]	= GsmL1_LogChComb_VII,
	[GSM_PCHAN_PDCH]		= GsmL1_LogChComb_XIII,
	//[GSM_PCHAN_TCH_F_PDCH]		= FIXME,
	[GSM_PCHAN_UNKNOWN]		= GsmL1_LogChComb_0,
};

static void *prim_init(GsmL1_Prim_t *prim, GsmL1_PrimId_t id, struct femtol1_hdl *gl1)
{
	prim->id = id;

	/* for some reason the hLayer1 field is not always at the same position
	 * in the GsmL1_Prim_t, so we have to have this ugly case statement here... */
	switch (id) {
	case GsmL1_PrimId_MphInitReq:
		//prim->u.mphInitReq.hLayer1 = gl1->hLayer1;
		break;
	case GsmL1_PrimId_MphCloseReq:
		prim->u.mphCloseReq.hLayer1 = gl1->hLayer1;
		break;
	case GsmL1_PrimId_MphConnectReq:
		prim->u.mphConnectReq.hLayer1 = gl1->hLayer1;
		break;
	case GsmL1_PrimId_MphDisconnectReq:
		prim->u.mphDisconnectReq.hLayer1 = gl1->hLayer1;
		break;
	case GsmL1_PrimId_MphActivateReq:
		prim->u.mphActivateReq.hLayer1 = gl1->hLayer1;
		break;
	case GsmL1_PrimId_MphDeactivateReq:
		prim->u.mphDeactivateReq.hLayer1 = gl1->hLayer1;
		break;
	case GsmL1_PrimId_MphConfigReq:
		prim->u.mphConfigReq.hLayer1 = gl1->hLayer1;
		break;
	case GsmL1_PrimId_MphMeasureReq:
		prim->u.mphMeasureReq.hLayer1 = gl1->hLayer1;
		break;
	case GsmL1_PrimId_MphInitCnf:
	case GsmL1_PrimId_MphCloseCnf:
	case GsmL1_PrimId_MphConnectCnf:
	case GsmL1_PrimId_MphDisconnectCnf:
	case GsmL1_PrimId_MphActivateCnf:
	case GsmL1_PrimId_MphDeactivateCnf:
	case GsmL1_PrimId_MphConfigCnf:
	case GsmL1_PrimId_MphMeasureCnf:
		break;
	case GsmL1_PrimId_MphTimeInd:
		break;
	case GsmL1_PrimId_MphSyncInd:
		break;
	case GsmL1_PrimId_PhEmptyFrameReq:
		prim->u.phEmptyFrameReq.hLayer1 = gl1->hLayer1;
		break;
	case GsmL1_PrimId_PhDataReq:
		prim->u.phDataReq.hLayer1 = gl1->hLayer1;
		break;
	case GsmL1_PrimId_PhConnectInd:
		break;
	case GsmL1_PrimId_PhReadyToSendInd:
		break;
	case GsmL1_PrimId_PhDataInd:
		break;
	case GsmL1_PrimId_PhRaInd:
		break;
	default:
		LOGP(DL1C, LOGL_ERROR, "unknown L1 primitive %u\n", id);
		break;
	}
	return &prim->u;
}

GsmL1_Status_t prim_status(GsmL1_Prim_t *prim)
{
	/* for some reason the Status field is not always at the same position
	 * in the GsmL1_Prim_t, so we have to have this ugly case statement here... */
	switch (prim->id) {
	case GsmL1_PrimId_MphInitCnf:
		return prim->u.mphInitCnf.status;
	case GsmL1_PrimId_MphCloseCnf:
		return prim->u.mphCloseCnf.status;
	case GsmL1_PrimId_MphConnectCnf:
		return prim->u.mphConnectCnf.status;
	case GsmL1_PrimId_MphDisconnectCnf:
		return prim->u.mphDisconnectCnf.status;
	case GsmL1_PrimId_MphActivateCnf:
		return prim->u.mphActivateCnf.status;
	case GsmL1_PrimId_MphDeactivateCnf:
		return prim->u.mphDeactivateCnf.status;
	case GsmL1_PrimId_MphConfigCnf:
		return prim->u.mphConfigCnf.status;
	case GsmL1_PrimId_MphMeasureCnf:
		return prim->u.mphMeasureCnf.status;
	default:
		break;
	}
	return GsmL1_Status_Success;
}

#if 0
static int compl_cb_send_oml_msg(struct msgb *l1_msg, void *data)
{
	struct msgb *resp_msg = data;
	GsmL1_Prim_t *l1p = msgb_l1prim(l1_msg);

	if (prim_status(l1p) != GsmL1_Status_Success) {
		LOGP(DL1C, LOGL_ERROR, "Rx %s, status: %s\n",
			get_value_string(femtobts_l1prim_names, l1p->id),
			get_value_string(femtobts_l1status_names, cc->status));
		return 0;
	}

	msgb_free(l1_msg);

	return abis_nm_sendmsg(msg);
}
#endif

int lchan_activate(struct gsm_lchan *lchan);

static int opstart_compl_cb(struct msgb *l1_msg, void *data)
{
	struct gsm_abis_mo *mo = data;
	GsmL1_Prim_t *l1p = msgb_l1prim(l1_msg);
	GsmL1_Status_t status = prim_status(l1p);

	if (status != GsmL1_Status_Success) {
		LOGP(DL1C, LOGL_ERROR, "Rx %s, status: %s\n",
			get_value_string(femtobts_l1prim_names, l1p->id),
			get_value_string(femtobts_l1status_names, status));
		return oml_mo_opstart_nack(mo, NM_NACK_CANT_PERFORM);
	}

	msgb_free(l1_msg);

	/* Set to Operational State: Enabled */
	oml_mo_state_chg(mo, NM_OPSTATE_ENABLED, NM_AVSTATE_OK);

	/* ugly hack to auto-activate all SAPIs for the BCCH/CCCH on TS0 */
	if (mo->obj_class == NM_OC_CHANNEL && mo->obj_inst.trx_nr == 0 &&
	    mo->obj_inst.ts_nr == 0) {
		DEBUGP(DL1C, "====> trying to activate lchans of BCCH\n");
		lchan_activate(&mo->bts->c0->ts[0].lchan[4]);
	}

	/* Send OPSTART ack */
	return oml_mo_opstart_ack(mo);
}

static int trx_init_compl_cb(struct msgb *l1_msg, void *data)
{
	struct femtol1_hdl *fl1h = data;
	struct gsm_bts_trx *trx = fl1h->priv;

	GsmL1_Prim_t *l1p = msgb_l1prim(l1_msg);
	GsmL1_MphInitCnf_t *ic = &l1p->u.mphInitCnf;

	LOGP(DL1C, LOGL_INFO, "Rx MPH-INIT.conf (status=%s)\n",
		get_value_string(femtobts_l1status_names, ic->status));

	/* store layer1 handle */
	if (ic->status != GsmL1_Status_Success) {
		LOGP(DL1C, LOGL_FATAL, "Rx MPH-INIT.conf status=%s\n",
			get_value_string(femtobts_l1status_names, ic->status));
		bts_shutdown(trx->bts, "MPH-INIT failure");
	}

	fl1h->hLayer1 = ic->hLayer1;

	return opstart_compl_cb(l1_msg, &trx->mo);
}

int gsm_abis_mo_check_attr(const struct gsm_abis_mo *mo, uint8_t *attr_ids,
			   unsigned int num_attr_ids)
{
	unsigned int i;

	if (!mo->nm_attr)
		return 0;

	for (i = 0; i < num_attr_ids; i++) {
		if (!TLVP_PRESENT(mo->nm_attr, attr_ids[i]))
			return 0;
	}
	return 1;
}

static const uint8_t trx_rqd_attr[] = { NM_ATT_RF_MAXPOWR_R };

/* initialize the layer1 */
static int trx_init(struct gsm_bts_trx *trx)
{
	struct femtol1_hdl *fl1h = trx_femtol1_hdl(trx);
	struct msgb *msg;
	GsmL1_MphInitReq_t *mi_req;
	GsmL1_DeviceParam_t *dev_par;

	if (!gsm_abis_mo_check_attr(&trx->mo, trx_rqd_attr,
				    ARRAY_SIZE(trx_rqd_attr))) {
		/* HACK: spec says we need to decline, but openbsc
		 * doesn't deal with this very well */
		return oml_mo_opstart_ack(&trx->mo);
		//return oml_mo_opstart_nack(&trx->mo, NM_NACK_CANT_PERFORM);
	}

	msg = l1p_msgb_alloc();
	mi_req = prim_init(msgb_l1prim(msg), GsmL1_PrimId_MphInitReq, fl1h);
	dev_par = &mi_req->deviceParam;
	dev_par->devType = GsmL1_DevType_TxdRxu;
	dev_par->freqBand = GsmL1_FreqBand_1800;
	dev_par->u16Arfcn = trx->arfcn;
	dev_par->u16BcchArfcn = trx->bts->c0->arfcn;
	dev_par->u8NbTsc = trx->bts->bsic & 7;
	dev_par->fRxPowerLevel = -75.f;
	dev_par->fTxPowerLevel = trx->nominal_power - trx->max_power_red;
	LOGP(DL1C, LOGL_NOTICE, "Init TRX (ARFCN %u, TSC %u, RxPower % 2f dBm, "
		"TxPower % 2.2f dBm\n", dev_par->u16Arfcn, dev_par->u8NbTsc,
		dev_par->fRxPowerLevel, dev_par->fTxPowerLevel);
	
	/* send MPH-INIT-REQ, wait for MPH-INIT-CNF */
	return l1if_req_compl(fl1h, msg, 0, trx_init_compl_cb, fl1h);
}

static int ts_connect(struct gsm_bts_trx_ts *ts)
{
	struct msgb *msg = l1p_msgb_alloc();
	struct femtol1_hdl *fl1h = trx_femtol1_hdl(ts->trx);
	GsmL1_MphConnectReq_t *cr;

	cr = prim_init(msgb_l1prim(msg), GsmL1_PrimId_MphConnectReq, fl1h);
	cr->u8Tn = ts->nr;
	cr->logChComb = pchan_to_logChComb[ts->pchan];
	
	return l1if_req_compl(fl1h, msg, 0, opstart_compl_cb, &ts->mo);
}

GsmL1_SubCh_t lchan_to_GsmL1_SubCh_t(const struct gsm_lchan *lchan)
{
	switch (lchan->ts->pchan) {
	case GSM_PCHAN_CCCH_SDCCH4:
		if (lchan->type == GSM_LCHAN_CCCH)
			return GsmL1_SubCh_NA;
		/* fall-through */
	case GSM_PCHAN_TCH_H:
	case GSM_PCHAN_SDCCH8_SACCH8C:
		return lchan->nr;
	case GSM_PCHAN_NONE:
	case GSM_PCHAN_CCCH:
	case GSM_PCHAN_TCH_F:
	case GSM_PCHAN_PDCH:
	case GSM_PCHAN_UNKNOWN:
	default:
		return GsmL1_SubCh_NA;
	}

	return GsmL1_SubCh_NA;
}

struct sapi_dir {
	GsmL1_Sapi_t sapi;
	GsmL1_Dir_t dir;
};

static const struct sapi_dir ccch_sapis[] = {
	{ GsmL1_Sapi_Fcch,	GsmL1_Dir_TxDownlink },
	{ GsmL1_Sapi_Sch, 	GsmL1_Dir_TxDownlink },
	{ GsmL1_Sapi_Bcch,	GsmL1_Dir_TxDownlink },
	{ GsmL1_Sapi_Agch,	GsmL1_Dir_TxDownlink },
	{ GsmL1_Sapi_Pch,	GsmL1_Dir_TxDownlink },
	{ GsmL1_Sapi_Rach,	GsmL1_Dir_RxUplink },
};

#define DIR_BOTH	(GsmL1_Dir_TxDownlink|GsmL1_Dir_RxUplink)

static const struct sapi_dir tchf_sapis[] = {
	{ GsmL1_Sapi_TchF,	DIR_BOTH },
	{ GsmL1_Sapi_FacchF,	DIR_BOTH },
	{ GsmL1_Sapi_Sacch,	DIR_BOTH },
};

static const struct sapi_dir tchh_sapis[] = {
	{ GsmL1_Sapi_TchH,	DIR_BOTH },
	{ GsmL1_Sapi_FacchH, 	DIR_BOTH },
	{ GsmL1_Sapi_Sacch,	DIR_BOTH },
};

static const struct sapi_dir sdcch_sapis[] = {
	{ GsmL1_Sapi_Sdcch, 	DIR_BOTH },
	{ GsmL1_Sapi_Sacch,	DIR_BOTH },
};

struct lchan_sapis {
	const struct sapi_dir *sapis;
	unsigned int num_sapis;
};

static const struct lchan_sapis sapis_for_lchan[_GSM_LCHAN_MAX] = {
	[GSM_LCHAN_SDCCH] = {
		.sapis = sdcch_sapis,
		.num_sapis = ARRAY_SIZE(sdcch_sapis),
	},
	[GSM_LCHAN_TCH_F] = {
		.sapis = tchf_sapis,
		.num_sapis = ARRAY_SIZE(tchf_sapis),
	},
	[GSM_LCHAN_TCH_H] = {
		.sapis = tchh_sapis,
		.num_sapis = ARRAY_SIZE(tchh_sapis),
	},
	[GSM_LCHAN_CCCH] = {
		.sapis = ccch_sapis,
		.num_sapis = ARRAY_SIZE(ccch_sapis),
	},
};

static int lchan_act_compl_cb(struct msgb *l1_msg, void *data)
{
	struct gsm_lchan *lchan = data;
	GsmL1_Prim_t *l1p = msgb_l1prim(l1_msg);
	GsmL1_MphActivateCnf_t *ic = &l1p->u.mphActivateCnf;

	LOGP(DL1C, LOGL_INFO, "%s MPH-ACTIVATE.conf\n", gsm_lchan_name(lchan));

	if (ic->status == GsmL1_Status_Success) {
		DEBUGP(DL1C, "Successful activation of L1 SAPI %s on TS %u\n",
			get_value_string(femtobts_l1sapi_names, ic->sapi), ic->u8Tn);
		lchan->state = LCHAN_S_ACTIVE;
	} else {
		LOGP(DL1C, LOGL_ERROR, "Error activating L1 SAPI %s on TS %u: %s\n",
			get_value_string(femtobts_l1sapi_names, ic->sapi), ic->u8Tn,
			get_value_string(femtobts_l1status_names, ic->status));
		lchan->state = LCHAN_S_NONE;
	}

	switch (ic->sapi) {
	case GsmL1_Sapi_Sdcch:
	case GsmL1_Sapi_TchF:
		/* FIXME: Send RSL CHAN ACT */
		break;
	default:
		break;
	}

	msgb_free(l1_msg);

	return 0;
}

uint32_t l1if_lchan_to_hLayer2(struct gsm_lchan *lchan)
{
	return (lchan->nr << 8) | (lchan->ts->nr << 16) | (lchan->ts->trx->nr << 24);
}

/* obtain a ptr to the lapdm_channel for a given hLayer2 */
struct gsm_lchan *
l1if_hLayer2_to_lchan(struct gsm_bts_trx *trx, uint32_t hLayer2)
{
	uint8_t ts_nr = (hLayer2 >> 16) & 0xff;
	uint8_t lchan_nr = (hLayer2 >> 8)& 0xff;
	struct gsm_bts_trx_ts *ts;

	/* FIXME: if we actually run on the BTS, the 32bit field is large
	 * enough to simply put a pointer inside. */
	if (ts_nr >= ARRAY_SIZE(trx->ts))
		return NULL;

	ts = &trx->ts[ts_nr];

	if (lchan_nr >= ARRAY_SIZE(ts->lchan))
		return NULL;

	return &ts->lchan[lchan_nr];
}



int lchan_activate(struct gsm_lchan *lchan)
{
	struct femtol1_hdl *fl1h = trx_femtol1_hdl(lchan->ts->trx);
	const struct lchan_sapis *s4l = &sapis_for_lchan[lchan->type];
	unsigned int i;

	for (i = 0; i < s4l->num_sapis; i++) {
		struct msgb *msg = l1p_msgb_alloc();
		GsmL1_MphActivateReq_t *act_req;
		GsmL1_LogChParam_t *lch_par;
		int j;

		act_req = prim_init(msgb_l1prim(msg), GsmL1_PrimId_MphActivateReq, fl1h);
		lch_par = &act_req->logChPrm;
		act_req->u8Tn = lchan->ts->nr;
		act_req->subCh = lchan_to_GsmL1_SubCh_t(lchan);
		act_req->dir = s4l->sapis[i].dir;
		act_req->sapi = s4l->sapis[i].sapi;
		act_req->hLayer2 = l1if_lchan_to_hLayer2(lchan);

		switch (act_req->sapi) {
		case GsmL1_Sapi_Rach:
			lch_par->rach.u8Bsic = lchan->ts->trx->bts->bsic;
			break;
		case GsmL1_Sapi_Agch:
#warning Set BS_AG_BLKS_RES
			lch_par->agch.u8NbrOfAgch = 1;
			break;
		case GsmL1_Sapi_Sacch:
			/* Only if we use manual MS power control */
			//act_req->logChPrm.sacch.u8MsPowerLevel = FIXME;
			break;
		case GsmL1_Sapi_TchH:
		case GsmL1_Sapi_TchF:
#warning Set AMR parameters for TCH
			lch_par->tch.amrCmiPhase = GsmL1_AmrCmiPhase_NA;
			lch_par->tch.amrInitCodecMode = GsmL1_AmrCodecMode_Unset;
			for (j = 0; j < ARRAY_SIZE(lch_par->tch.amrActiveCodecSet); j++)
				lch_par->tch.amrActiveCodecSet[i] = GsmL1_AmrCodec_Unset;
			break;
		default:
			break;
		}

		LOGP(DL1C, LOGL_INFO, "%s MPH-ACTIVATE.req (hL2=0x%08x)\n",
			gsm_lchan_name(lchan), act_req->hLayer2);

		/* send the primitive for all GsmL1_Sapi_* that match the LCHAN */
		l1if_req_compl(fl1h, msg, 0, lchan_act_compl_cb, lchan);

	}
	lchan->state = LCHAN_S_ACT_REQ;

	lchan_init_lapdm(lchan);

	return 0;
}

static int lchan_deact_compl_cb(struct msgb *l1_msg, void *data)
{
	struct gsm_lchan *lchan = data;
	GsmL1_Prim_t *l1p = msgb_l1prim(l1_msg);
	GsmL1_MphDeactivateCnf_t *ic = &l1p->u.mphDeactivateCnf;

	LOGP(DL1C, LOGL_INFO, "%s MPH-DEACTIVATE.conf (%s)\n",
		gsm_lchan_name(lchan),
		get_value_string(femtobts_l1sapi_names, ic->sapi));

	if (ic->status == GsmL1_Status_Success) {
		DEBUGP(DL1C, "Successful deactivation of L1 SAPI %s on TS %u\n",
			get_value_string(femtobts_l1sapi_names, ic->sapi), ic->u8Tn);
		lchan->state = LCHAN_S_ACTIVE;
	} else {
		LOGP(DL1C, LOGL_ERROR, "Error deactivating L1 SAPI %s on TS %u: %s\n",
			get_value_string(femtobts_l1sapi_names, ic->sapi), ic->u8Tn,
			get_value_string(femtobts_l1status_names, ic->status));
		lchan->state = LCHAN_S_NONE;
	}

	switch (ic->sapi) {
	case GsmL1_Sapi_Sdcch:
	case GsmL1_Sapi_TchF:
		/* FIXME: Send RSL CHAN REL ACK */
		break;
	default:
		break;
	}

	msgb_free(l1_msg);

	return 0;
}

int lchan_deactivate(struct gsm_lchan *lchan)
{
	struct femtol1_hdl *fl1h = trx_femtol1_hdl(lchan->ts->trx);
	const struct lchan_sapis *s4l = &sapis_for_lchan[lchan->type];
	unsigned int i;

	for (i = 0; i < s4l->num_sapis; i++) {
		struct msgb *msg = l1p_msgb_alloc();
		GsmL1_MphDeactivateReq_t *deact_req;

		deact_req = prim_init(msgb_l1prim(msg), GsmL1_PrimId_MphDeactivateReq, fl1h);
		deact_req->u8Tn = lchan->ts->nr;
		deact_req->subCh = lchan_to_GsmL1_SubCh_t(lchan);
		deact_req->dir = s4l->sapis[i].dir;
		deact_req->sapi = s4l->sapis[i].sapi;

		LOGP(DL1C, LOGL_INFO, "%s MPH-DEACTIVATE.req (%s)\n",
			gsm_lchan_name(lchan),
			get_value_string(femtobts_l1sapi_names, deact_req->sapi));

		/* send the primitive for all GsmL1_Sapi_* that match the LCHAN */
		l1if_req_compl(fl1h, msg, 0, lchan_deact_compl_cb, lchan);

	}
	lchan->state = LCHAN_S_ACT_REQ;

	return 0;
}

int lchan_deactivate_sacch(struct gsm_lchan *lchan)
{
	struct femtol1_hdl *fl1h = trx_femtol1_hdl(lchan->ts->trx);
	struct msgb *msg = l1p_msgb_alloc();
	GsmL1_MphDeactivateReq_t *deact_req;

	deact_req = prim_init(msgb_l1prim(msg), GsmL1_PrimId_MphDeactivateReq, fl1h);
	deact_req->u8Tn = lchan->ts->nr;
	deact_req->subCh = lchan_to_GsmL1_SubCh_t(lchan);
	deact_req->dir = DIR_BOTH;
	deact_req->sapi = GsmL1_Sapi_Sacch;

	LOGP(DL1C, LOGL_INFO, "%s MPH-DEACTIVATE.req (SACCH)\n",
		gsm_lchan_name(lchan));

	/* send the primitive for all GsmL1_Sapi_* that match the LCHAN */
	return l1if_req_compl(fl1h, msg, 0, lchan_deact_compl_cb, lchan);
}


struct gsm_time *bts_model_get_time(struct gsm_bts *bts)
{
	struct femtol1_hdl *fl1h = trx_femtol1_hdl(bts->c0);

	return &fl1h->gsm_time;
}

/* callback from OML */
int bts_model_check_oml(struct gsm_bts *bts, uint8_t msg_type,
			struct tlv_parsed *old_attr, struct tlv_parsed *new_attr,
			void *obj)
{
	/* FIXME: check if the attributes are valid */
	return 0;
}

/* callback from OML */
int bts_model_apply_oml(struct gsm_bts *bts, struct msgb *msg,
			struct tlv_parsed *new_attr, void *obj)
{
	/* FIXME: we actaully need to send a ACK or NACK for the OML message */
	return oml_fom_ack_nack(msg, 0);
}

/* callback from OML */
int bts_model_opstart(struct gsm_bts *bts, struct gsm_abis_mo *mo,
		      void *obj)
{
	int rc;

	switch (mo->obj_class) {
	case NM_OC_RADIO_CARRIER:
		rc = trx_init(obj);
		break;
	case NM_OC_CHANNEL:
		rc = ts_connect(obj);
		break;
	case NM_OC_BTS:
	case NM_OC_SITE_MANAGER:
	case NM_OC_BASEB_TRANSC:
		oml_mo_state_chg(mo, NM_OPSTATE_ENABLED, -1);
		rc = oml_mo_opstart_ack(mo);
		break;
	default:
		rc = oml_mo_opstart_nack(mo, NM_NACK_OBJCLASS_NOTSUPP);
	}
	return rc;
}

int bts_model_chg_adm_state(struct gsm_bts *bts, struct gsm_abis_mo *mo,
			    void *obj, uint8_t adm_state)
{
	/* blindly accept all state changes */
	mo->nm_state.administrative = adm_state;
	return oml_mo_fom_ack_nack(mo, NM_MT_CHG_ADM_STATE, 0);
}
