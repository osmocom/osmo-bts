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

#include <stdint.h>
#include <errno.h>

#include <osmocom/core/talloc.h>
#include <osmocom/core/utils.h>
#include <osmocom/codec/codec.h>
#include <osmocom/core/fsm.h>

#include <osmo-bts/gsm_data.h>
#include <osmo-bts/phy_link.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/oml.h>
#include <osmo-bts/rsl.h>
#include <osmo-bts/amr.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/bts_model.h>
#include <osmo-bts/handover.h>
#include <osmo-bts/l1sap.h>
#include <osmo-bts/nm_common_fsm.h>

/* TODO: check if dummy method is sufficient, else implement */
int bts_model_lchan_deactivate(struct gsm_lchan *lchan)
{
	LOGP(DL1C, LOGL_NOTICE, "Unimplemented %s\n", __func__);
	return -1;
}

/* TODO: check if dummy method is sufficient, else implement */
int osmo_amr_rtp_dec(const uint8_t *rtppayload, int payload_len, uint8_t *cmr,
		     int8_t *cmi, enum osmo_amr_type *ft, enum osmo_amr_quality *bfi, int8_t *sti)
{
	LOGP(DL1C, LOGL_NOTICE, "Unimplemented %s\n", __func__);
	return -1;
}

void bts_model_trx_close(struct gsm_bts_trx *trx)
{
	bts_model_trx_close_cb(trx, 0);
}

int bts_model_adjst_ms_pwr(struct gsm_lchan *lchan)
{
	LOGP(DL1C, LOGL_NOTICE, "Unimplemented %s\n", __func__);
	return 0;
}

int bts_model_check_oml(struct gsm_bts *bts, uint8_t msg_type,
			struct tlv_parsed *old_attr, struct tlv_parsed *new_attr,
			void *obj)
{
	return 0;
}

static uint8_t vbts_set_bts(struct gsm_bts *bts)
{
	struct gsm_bts_trx *trx;

	llist_for_each_entry(trx, &bts->trx_list, list) {
		/* report availability of trx to the bts. this will trigger the rsl connection */
		osmo_fsm_inst_dispatch(trx->mo.fi, NM_EV_SW_ACT, NULL);
		osmo_fsm_inst_dispatch(trx->bb_transc.mo.fi, NM_EV_SW_ACT, NULL);
	}
	return 0;
}

static uint8_t vbts_set_trx(struct gsm_bts_trx *trx)
{
	LOGP(DL1C, LOGL_NOTICE, "Unimplemented %s\n", __func__);
	return 0;
}

static uint8_t vbts_set_ts(struct gsm_bts_trx_ts *ts)
{
	return 0;
}

int bts_model_apply_oml(struct gsm_bts *bts, struct msgb *msg,
			struct tlv_parsed *new_attr, int kind, void *obj)
{
	struct abis_om_fom_hdr *foh = msgb_l3(msg);
	int cause = 0;

	switch (foh->msg_type) {
	case NM_MT_SET_BTS_ATTR:
		cause = vbts_set_bts(obj);
		break;
	case NM_MT_SET_RADIO_ATTR:
		cause = vbts_set_trx(obj);
		break;
	case NM_MT_SET_CHAN_ATTR:
		cause = vbts_set_ts(obj);
		break;
	}
	return oml_fom_ack_nack(msg, cause);
}

/* MO: TS 12.21 Managed Object */
int bts_model_opstart(struct gsm_bts *bts, struct gsm_abis_mo *mo, void *obj)
{
	int rc;
	struct gsm_bts_bb_trx *bb_transc;
	struct gsm_bts_trx* trx;
	struct gsm_bts_trx_ts* ts;

	switch (mo->obj_class) {
	case NM_OC_SITE_MANAGER:
		rc = osmo_fsm_inst_dispatch(bts->site_mgr.mo.fi, NM_EV_OPSTART_ACK, NULL);
		break;
	case NM_OC_BTS:
		rc = osmo_fsm_inst_dispatch(bts->mo.fi, NM_EV_OPSTART_ACK, NULL);
		break;
	case NM_OC_RADIO_CARRIER:
		trx = (struct gsm_bts_trx*) obj;
		rc = osmo_fsm_inst_dispatch(trx->mo.fi, NM_EV_OPSTART_ACK, NULL);
		break;
	case NM_OC_BASEB_TRANSC:
		bb_transc = (struct gsm_bts_bb_trx *) obj;
		rc = osmo_fsm_inst_dispatch(bb_transc->mo.fi, NM_EV_OPSTART_ACK, NULL);
		break;
	case NM_OC_CHANNEL:
		ts = (struct gsm_bts_trx_ts *) obj;
		rc = osmo_fsm_inst_dispatch(ts->mo.fi, NM_EV_OPSTART_ACK, NULL);
		break;
	case NM_OC_GPRS_NSE:
	case NM_OC_GPRS_CELL:
	case NM_OC_GPRS_NSVC:
		oml_mo_state_chg(mo, NM_OPSTATE_ENABLED, NM_AVSTATE_OK, -1);
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
	mo->nm_state.administrative = adm_state;
	return oml_mo_statechg_ack(mo);
}

int bts_model_trx_deact_rf(struct gsm_bts_trx *trx)
{
	return 0;
}


int bts_model_change_power(struct gsm_bts_trx *trx, int p_trxout_mdBm)
{
	power_trx_change_compl(trx, p_trxout_mdBm);
	return 0;
}

int bts_model_ctrl_cmds_install(struct gsm_bts *bts)
{
	LOGP(DL1C, LOGL_NOTICE, "Unimplemented %s\n", __func__);
	return 0;
}

uint32_t trx_get_hlayer1(const struct gsm_bts_trx *trx)
{
	return 0;
}

int bts_model_init(struct gsm_bts *bts)
{
	bts->variant = BTS_OSMO_OMLDUMMY;
	osmo_bts_set_feature(bts->features, BTS_FEAT_CBCH);
	osmo_bts_set_feature(bts->features, BTS_FEAT_HOPPING);
	osmo_bts_set_feature(bts->features, BTS_FEAT_BCCH_POWER_RED);
	return 0;
}

int bts_model_trx_init(struct gsm_bts_trx *trx)
{
	struct trx_power_params *tpp = &trx->power_params;
	/* Speed up shutdown, we don't care about power ramping in omldummy */
	tpp->ramp.step_interval_sec = 0;
	return 0;
}

void bts_model_print_help()
{
}

void bts_model_abis_close(struct gsm_bts *bts)
{
	bts_shutdown(bts, "Abis close");
}

void bts_model_phy_link_set_defaults(struct phy_link *plink)
{
}

void bts_model_phy_instance_set_defaults(struct phy_instance *pinst)
{
}

int bts_model_oml_estab(struct gsm_bts *bts)
{
	return 0;
}

int bts_model_ts_disconnect(struct gsm_bts_trx_ts *ts)
{
	return -ENOTSUP;
}

void bts_model_ts_connect(struct gsm_bts_trx_ts *ts, enum gsm_phys_chan_config as_pchan)
{
	return;
}

int bts_model_l1sap_down(struct gsm_bts_trx *trx, struct osmo_phsap_prim *l1sap)
{
	return 0;
}

int bts_model_phy_link_open(struct phy_link *plink)
{
	return 0;
}
