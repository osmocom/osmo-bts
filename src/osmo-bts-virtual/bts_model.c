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

int bts_model_trx_close(struct gsm_bts_trx *trx)
{
	return 0;
}

int bts_model_adjst_ms_pwr(struct gsm_lchan *lchan)
{
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
	return 0;
}

static uint8_t vbts_set_trx(struct gsm_bts_trx *trx)
{
	return 0;
}

static uint8_t vbts_set_ts(struct gsm_bts_trx_ts *ts)
{
	struct phy_instance *pinst = trx_phy_instance(ts->trx);
	int rc;

	rc = trx_sched_set_pchan(&pinst->u.virt.sched, ts->nr, ts->pchan);
	if (rc)
		return NM_NACK_RES_NOTAVAIL;

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

int bts_model_opstart(struct gsm_bts *bts, struct gsm_abis_mo *mo,
		      void *obj)
{
	int rc;

	switch (mo->obj_class) {
	case NM_OC_RADIO_CARRIER:
	case NM_OC_CHANNEL:
		oml_mo_state_chg(mo, NM_OPSTATE_ENABLED, NM_AVSTATE_OK);
		rc = oml_mo_opstart_ack(mo);
		break;
	case NM_OC_BTS:
	case NM_OC_SITE_MANAGER:
	case NM_OC_BASEB_TRANSC:
	case NM_OC_GPRS_NSE:
	case NM_OC_GPRS_CELL:
	case NM_OC_GPRS_NSVC:
		oml_mo_state_chg(mo, NM_OPSTATE_ENABLED, -1);
		rc = oml_mo_opstart_ack(mo);
		if (mo->obj_class == NM_OC_BTS) {
			oml_mo_state_chg(&bts->mo, -1, NM_AVSTATE_OK);
			oml_mo_state_chg(&bts->gprs.nse.mo, -1, NM_AVSTATE_OK);
			oml_mo_state_chg(&bts->gprs.cell.mo, -1, NM_AVSTATE_OK);
			oml_mo_state_chg(&bts->gprs.nsvc[0].mo, -1, NM_AVSTATE_OK);
		}
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
	return 0;
}

int bts_model_ctrl_cmds_install(struct gsm_bts *bts)
{
	return 0;
}
