/* NM BTS FSM */

/* (C) 2020 by sysmocom - s.f.m.c. GmbH <info@sysmocom.de>
 * Author: Pau Espin Pedrol <pespin@sysmocom.de>
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
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <errno.h>
#include <unistd.h>
#include <inttypes.h>

#include <osmocom/core/fsm.h>
#include <osmocom/core/tdef.h>
#include <osmocom/gsm/protocol/gsm_12_21.h>

#include <osmo-bts/logging.h>
#include <osmo-bts/gsm_data.h>
#include <osmo-bts/bts_model.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/rsl.h>
#include <osmo-bts/nm_common_fsm.h>
#include <osmo-bts/phy_link.h>
#include <osmo-bts/cbch.h>
#include <osmo-bts/notification.h>

#define X(s) (1 << (s))

#define nm_bts_fsm_state_chg(fi, NEXT_STATE) \
	osmo_fsm_inst_state_chg(fi, NEXT_STATE, 0, 0)

static void ev_dispatch_children(struct gsm_bts *bts, uint32_t event)
{
	struct gsm_bts_trx *trx;
	llist_for_each_entry(trx, &bts->trx_list, list) {
		osmo_fsm_inst_dispatch(trx->mo.fi, event, NULL);
		osmo_fsm_inst_dispatch(trx->bb_transc.mo.fi, event, NULL);
	}
}

//////////////////////////
// FSM STATE ACTIONS
//////////////////////////

static void st_op_disabled_notinstalled_on_enter(struct osmo_fsm_inst *fi, uint32_t prev_state)
{
	struct gsm_bts *bts = (struct gsm_bts *)fi->priv;
	/* Reset state: */
	bts->si_valid = 0;
	bts->bsic_configured = false;
	bts->bsic = 0xff; /* invalid value */
	TALLOC_FREE(bts->mo.nm_attr);
	bts_cbch_reset(bts);
	bts_asci_notification_reset(bts);
	if (bts->c0_power_red_db > 0)
		bts_set_c0_pwr_red(bts, 0);

	bts->mo.setattr_success = false;
	bts->mo.opstart_success = false;
	oml_mo_state_chg(&bts->mo, NM_OPSTATE_DISABLED, NM_AVSTATE_NOT_INSTALLED, NM_STATE_LOCKED);
}

static void st_op_disabled_notinstalled(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct gsm_bts *bts = (struct gsm_bts *)fi->priv;
	struct gsm_bts_trx *trx;

	switch (event) {
	case NM_EV_OML_UP:
		/* automatic SW_ACT upon OML link establishment: */
		oml_mo_tx_sw_act_rep(&bts->mo);

		llist_for_each_entry(trx, &bts->trx_list, list) {
			if (trx->bts->variant == BTS_OSMO_OMLDUMMY) /* In OMLDUMMY, phy=NULL */
				continue;
			/* During startup, phy_links are already opened, but if we are
			 * re-connecting, phy_link was closed when disconnected from
			 * previous BSC, so let's re-open it.
			 */
			struct phy_instance *pinst = trx_phy_instance(trx);
			struct phy_link *plink = pinst->phy_link;
			if (phy_link_state_get(plink) == PHY_LINK_SHUTDOWN)
				bts_model_phy_link_open(plink);
		}

		nm_bts_fsm_state_chg(fi, NM_BTS_ST_OP_DISABLED_OFFLINE);
		ev_dispatch_children(bts, event);
		return;
	default:
		OSMO_ASSERT(0);
	}
}

static void st_op_disabled_offline_on_enter(struct osmo_fsm_inst *fi, uint32_t prev_state)
{
	struct gsm_bts *bts = (struct gsm_bts *)fi->priv;
	bts->mo.setattr_success = false;
	bts->mo.opstart_success = false;
	oml_mo_state_chg(&bts->mo, NM_OPSTATE_DISABLED, NM_AVSTATE_OFF_LINE, -1);
}

static void st_op_disabled_offline(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct gsm_bts *bts = (struct gsm_bts *)fi->priv;
	struct nm_fsm_ev_setattr_data *setattr_data;
	int rc;

	switch (event) {
	case NM_EV_RX_SETATTR:
		setattr_data = (struct nm_fsm_ev_setattr_data *)data;
		rc = bts_model_apply_oml(bts, setattr_data->msg, &bts->mo, bts);
		bts->mo.setattr_success = rc == 0;
		oml_fom_ack_nack_copy_msg(setattr_data->msg, rc);
		break;
	case NM_EV_RX_OPSTART:
		if (!bts->mo.setattr_success) {
			oml_mo_opstart_nack(&bts->mo, NM_NACK_CANT_PERFORM);
			return;
		}
		bts_model_opstart(bts, &bts->mo, bts);
		break;
	case NM_EV_OPSTART_ACK:
		bts->mo.opstart_success = true;
		oml_mo_opstart_ack(&bts->mo);
		nm_bts_fsm_state_chg(fi, NM_BTS_ST_OP_ENABLED);
		break; /* check statechg below */
	case NM_EV_OPSTART_NACK:
		bts->mo.opstart_success = false;
		oml_mo_opstart_nack(&bts->mo, (int)(intptr_t)data);
		return;
	default:
		OSMO_ASSERT(0);
	}
}

static void st_op_enabled_on_enter(struct osmo_fsm_inst *fi, uint32_t prev_state)
{
	struct gsm_bts *bts = (struct gsm_bts *)fi->priv;
	oml_mo_state_chg(&bts->mo, NM_OPSTATE_ENABLED, NM_AVSTATE_OK, -1);
}

static void st_op_enabled(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
}

static void nm_bts_allstate(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct gsm_bts *bts = (struct gsm_bts *)fi->priv;

	switch (event) {
	case NM_EV_SHUTDOWN_START:
		/* Announce we start shutting down */
		oml_mo_state_chg(&bts->mo, -1, -1, NM_STATE_SHUTDOWN);

		/* Propagate event to children: */
		ev_dispatch_children(bts, event);
		break;
	case NM_EV_SHUTDOWN_FINISH:
		/* Propagate event to children: */
		ev_dispatch_children(bts, event);
		nm_bts_fsm_state_chg(fi, NM_BTS_ST_OP_DISABLED_NOTINSTALLED);
		break;
	default:
		OSMO_ASSERT(false);
	}
}

static struct osmo_fsm_state nm_bts_fsm_states[] = {
	[NM_BTS_ST_OP_DISABLED_NOTINSTALLED] = {
		.in_event_mask =
			X(NM_EV_OML_UP),
		.out_state_mask =
			X(NM_BTS_ST_OP_DISABLED_NOTINSTALLED) |
			X(NM_BTS_ST_OP_DISABLED_OFFLINE),
		.name = "DISABLED_NOTINSTALLED",
		.onenter = st_op_disabled_notinstalled_on_enter,
		.action = st_op_disabled_notinstalled,
	},
	[NM_BTS_ST_OP_DISABLED_OFFLINE] = {
		.in_event_mask =
			X(NM_EV_RX_SETATTR) |
			X(NM_EV_RX_OPSTART) |
			X(NM_EV_OPSTART_ACK) |
			X(NM_EV_OPSTART_NACK),
		.out_state_mask =
			X(NM_BTS_ST_OP_DISABLED_NOTINSTALLED) |
			X(NM_BTS_ST_OP_ENABLED),
		.name = "DISABLED_OFFLINE",
		.onenter = st_op_disabled_offline_on_enter,
		.action = st_op_disabled_offline,
	},
	[NM_BTS_ST_OP_ENABLED] = {
		.in_event_mask = 0,
		.out_state_mask =
			X(NM_BTS_ST_OP_DISABLED_NOTINSTALLED),
		.name = "ENABLED",
		.onenter = st_op_enabled_on_enter,
		.action = st_op_enabled,
	},
};

struct osmo_fsm nm_bts_fsm = {
	.name = "NM_BTS_OP",
	.states = nm_bts_fsm_states,
	.num_states = ARRAY_SIZE(nm_bts_fsm_states),
	.event_names = nm_fsm_event_names,
	.allstate_action = nm_bts_allstate,
	.allstate_event_mask = X(NM_EV_SHUTDOWN_START) |
			       X(NM_EV_SHUTDOWN_FINISH),
	.log_subsys = DOML,
};

static __attribute__((constructor)) void nm_bts_fsm_init(void)
{
	OSMO_ASSERT(osmo_fsm_register(&nm_bts_fsm) == 0);
}
