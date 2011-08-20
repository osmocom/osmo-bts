/* (C) 2011 by Harald Welte <laforge@gnumonks.org>
 * (C) 2011 by Andreas Eversberg <jolly@eversberg.eu>
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

#include <osmo-bts/gsm_data.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/oml.h>

#include "l1ctl.h"
#include "l1_if.h"

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

static int trx_init(struct gsm_bts_trx *trx)
{
#if 0
	if (!gsm_abis_mo_check_attr(&trx->mo, trx_rqd_attr,
				    ARRAY_SIZE(trx_rqd_attr))) {
		/* HACK: spec says we need to decline, but openbsc
		 * doesn't deal with this very well */
		LOGP(DL1C, LOGL_NOTICE, "WHY HERE?\n");
		return oml_mo_opstart_ack(&trx->mo);
		//return oml_mo_opstart_nack(&trx->mo, NM_NACK_CANT_PERFORM);
	}
#endif

	return l1if_setup(trx);
}

static int opstart_compl(struct gsm_abis_mo *mo, int cause, int opstate, int avstate)
{
	if (cause) {
		return oml_mo_opstart_nack(mo, cause);
	}

	/* Set to Operational State: Enabled */
	oml_mo_state_chg(mo, opstate, avstate);

	/* ugly hack to auto-activate all SAPIs for the BCCH/CCCH on TS0 */
	if (mo->obj_class == NM_OC_CHANNEL && mo->obj_inst.trx_nr == 0 &&
	    mo->obj_inst.ts_nr == 0) {
		DEBUGP(DL1C, "====> trying to activate lchans of BCCH\n");
		lchan_activate(&mo->bts->c0->ts[0].lchan[4]);
	}

	/* Send OPSTART ack */
	return oml_mo_opstart_ack(mo);
}

int trx_init_complete(struct gsm_bts_trx *trx, int cause)
{
	struct gsm_abis_mo *mo = &trx->mo;

	return opstart_compl(mo, 0, NM_OPSTATE_ENABLED, NM_AVSTATE_OK);
}

static int ts_connect(struct gsm_bts_trx_ts *ts)
{
	struct gsm_abis_mo *mo = &ts->mo;
	int cause = 0;

	//FIXME: depends on l1 capabilities
	if (ts->nr > 3)
		return opstart_compl(mo, 0, NM_OPSTATE_DISABLED, NM_AVSTATE_OFF_LINE);
	return opstart_compl(mo, 0, NM_OPSTATE_ENABLED, NM_AVSTATE_OK);
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

int lchan_deactivate(struct gsm_lchan *lchan)
{
#if 0
	struct femtol1_hdl *fl1h = trx_femtol1_hdl(lchan->ts->trx);
	const struct lchan_sapis *s4l = &sapis_for_lchan[lchan->type];
	int i;

	for (i = s4l->num_sapis-1; i >= 0; i--) {
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

		/* Stop the alive timer once we deactivate the SCH */
		if (deact_req->sapi == GsmL1_Sapi_Sch)
			osmo_timer_del(&fl1h->alive_timer);

		/* send the primitive for all GsmL1_Sapi_* that match the LCHAN */
		l1if_req_compl(fl1h, msg, 0, lchan_deact_compl_cb, lchan);

	}
#endif
	lchan->state = LCHAN_S_ACT_REQ;
	return 0;
}

int lchan_activate(struct gsm_lchan *lchan)
{
#if 0
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
		case GsmL1_Sapi_Sch:
			/* once we activate the SCH, we should get MPH-TIME.ind */
			fl1h->alive_timer.cb = alive_timer_cb;
			fl1h->alive_timer.data = fl1h;
			fl1h->alive_prim_cnt = 0;
			osmo_timer_schedule(&fl1h->alive_timer, 5, 0);
			break;
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
#endif
	lchan->state = LCHAN_S_ACT_REQ;

	lchan_init_lapdm(lchan);

	return 0;
}

struct gsm_time *bts_model_get_time(struct gsm_bts *bts)
{
//	struct femtol1_hdl *fl1h = trx_femtol1_hdl(bts->c0);

//	return &fl1h->gsm_time;
}

int lchan_deactivate_sacch(struct gsm_lchan *lchan)
{
#if 0
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
#endif
}


