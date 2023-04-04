/* NM Radio Carrier FSM */

/* (C) 2020 by sysmocom - s.m.f.c. GmbH <info@sysmocom.de>
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
 * GNU General Public License for more details.
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

#define X(s) (1 << (s))

#define nm_bb_transc_fsm_state_chg(fi, NEXT_STATE) \
	osmo_fsm_inst_state_chg(fi, NEXT_STATE, 0, 0)

static void ev_dispatch_children(struct gsm_bts_bb_trx *bb_transc, uint32_t event)
{
	struct gsm_bts_trx *trx = gsm_bts_bb_trx_get_trx(bb_transc);
	uint8_t tn;

	for (tn = 0; tn < TRX_NR_TS; tn++) {
		struct gsm_bts_trx_ts *ts = &trx->ts[tn];
		osmo_fsm_inst_dispatch(ts->mo.fi, event, NULL);
	}
}

//////////////////////////
// FSM STATE ACTIONS
//////////////////////////

static void st_op_disabled_notinstalled_on_enter(struct osmo_fsm_inst *fi, uint32_t prev_state)
{
	struct gsm_bts_bb_trx *bb_transc = (struct gsm_bts_bb_trx *)fi->priv;
	/* Reset state: */
	TALLOC_FREE(bb_transc->mo.nm_attr);

	bb_transc->mo.setattr_success = false;
	bb_transc->mo.opstart_success = false;
	oml_mo_state_chg(&bb_transc->mo, NM_OPSTATE_DISABLED, NM_AVSTATE_NOT_INSTALLED, NM_STATE_LOCKED);
}

static void st_op_disabled_notinstalled(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct gsm_bts_bb_trx *bb_transc = (struct gsm_bts_bb_trx *)fi->priv;
	struct gsm_bts_trx *trx = gsm_bts_bb_trx_get_trx(bb_transc);
	int i;

	switch (event) {
	case NM_EV_SW_ACT:
		oml_mo_tx_sw_act_rep(&bb_transc->mo);
		nm_bb_transc_fsm_state_chg(fi, NM_BBTRANSC_ST_OP_DISABLED_OFFLINE);
		for (i = 0; i < TRX_NR_TS; i++) {
			struct gsm_bts_trx_ts *ts = &trx->ts[i];
			osmo_fsm_inst_dispatch(ts->mo.fi, NM_EV_BBTRANSC_INSTALLED, NULL);
		}
		return;
	case NM_EV_RSL_UP:
		return;
	case NM_EV_RSL_DOWN:
		return;
	case NM_EV_PHYLINK_UP:
		return;
	case NM_EV_PHYLINK_DOWN:
		return;
	case NM_EV_DISABLE:
		return;
	default:
		OSMO_ASSERT(0);
	}
}

static void st_op_disabled_offline_on_enter(struct osmo_fsm_inst *fi, uint32_t prev_state)
{
	struct gsm_bts_bb_trx *bb_transc = (struct gsm_bts_bb_trx *)fi->priv;
	struct gsm_bts_trx *trx = gsm_bts_bb_trx_get_trx(bb_transc);
	int i;

	bb_transc->mo.setattr_success = false;
	bb_transc->mo.opstart_success = false;
	oml_mo_state_chg(&bb_transc->mo, NM_OPSTATE_DISABLED, NM_AVSTATE_OFF_LINE, -1);

	if (prev_state == NM_BBTRANSC_ST_OP_ENABLED) {
		for (i = 0; i < TRX_NR_TS; i++) {
			struct gsm_bts_trx_ts *ts = &trx->ts[i];
			osmo_fsm_inst_dispatch(ts->mo.fi, NM_EV_BBTRANSC_DISABLED, NULL);
		}
	}
}

static void st_op_disabled_offline(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct gsm_bts_bb_trx *bb_transc = (struct gsm_bts_bb_trx *)fi->priv;
	struct gsm_bts_trx *trx = gsm_bts_bb_trx_get_trx(bb_transc);
	struct nm_fsm_ev_setattr_data *setattr_data;
	bool phy_state_connected;
	bool rsl_link_connected;
	int rc;

	switch (event) {
	case NM_EV_RX_SETATTR:
		setattr_data = (struct nm_fsm_ev_setattr_data *)data;
		rc = bts_model_apply_oml(trx->bts, setattr_data->msg,
					 &bb_transc->mo, bb_transc);
		bb_transc->mo.setattr_success = rc == 0;
		oml_fom_ack_nack_copy_msg(setattr_data->msg, rc);
		break;
	case NM_EV_RX_OPSTART:
		bts_model_opstart(trx->bts, &bb_transc->mo, bb_transc);
		break;
	case NM_EV_OPSTART_ACK:
		bb_transc->mo.opstart_success = true;
		oml_mo_opstart_ack(&bb_transc->mo);
		break; /* check statechg below */
	case NM_EV_OPSTART_NACK:
		bb_transc->mo.opstart_success = false;
		oml_mo_opstart_nack(&bb_transc->mo, (int)(intptr_t)data);
		return;
	case NM_EV_RSL_UP:
		break; /* check statechg below */
	case NM_EV_RSL_DOWN:
		return;
	case NM_EV_PHYLINK_UP:
		break; /* check statechg below */
	case NM_EV_PHYLINK_DOWN:
		return;
	case NM_EV_DISABLE:
		return;
	default:
		OSMO_ASSERT(0);
	}


	if (trx->bts->variant != BTS_OSMO_OMLDUMMY) { /* In OMLDUMMY, phy=NULL */
		struct phy_instance *pinst = trx_phy_instance(trx);
		phy_state_connected = phy_link_state_get(pinst->phy_link) == PHY_LINK_CONNECTED;
		rsl_link_connected = !!trx->rsl_link;
	} else {
		phy_state_connected = true;
		rsl_link_connected = true;
	}

	/* We so far don't expect any SetAttributes for this NM object */
	if (rsl_link_connected && phy_state_connected &&
	    bb_transc->mo.opstart_success) {
		nm_bb_transc_fsm_state_chg(fi, NM_BBTRANSC_ST_OP_ENABLED);
	} else {
		LOGPFSML(fi, LOGL_INFO, "Delay switch to operative state Enabled, wait for:%s%s%s\n",
			 rsl_link_connected ? "" : " rsl",
			 phy_state_connected ? "" : " phy",
			 bb_transc->mo.opstart_success ? "" : " opstart");

	}
}

static void st_op_enabled_on_enter(struct osmo_fsm_inst *fi, uint32_t prev_state)
{
	struct gsm_bts_bb_trx *bb_transc = (struct gsm_bts_bb_trx *)fi->priv;
	struct gsm_bts_trx *trx = gsm_bts_bb_trx_get_trx(bb_transc);
	uint8_t tn;

	oml_mo_state_chg(&bb_transc->mo, NM_OPSTATE_ENABLED, NM_AVSTATE_OK, -1);
	/* Mark Dependency TS as Offline (ready to be Opstarted) */
	for (tn = 0; tn < TRX_NR_TS; tn++) {
		struct gsm_bts_trx_ts *ts = &trx->ts[tn];
		osmo_fsm_inst_dispatch(ts->mo.fi, NM_EV_BBTRANSC_ENABLED, NULL);
	}
}

static void st_op_enabled(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	switch (event) {
	case NM_EV_RSL_DOWN:
		break;
	case NM_EV_PHYLINK_DOWN:
		break;
	case NM_EV_DISABLE:
		break;
	default:
		OSMO_ASSERT(0);
	}

	nm_bb_transc_fsm_state_chg(fi, NM_BBTRANSC_ST_OP_DISABLED_OFFLINE);
}

static void nm_bb_transc_allstate(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct gsm_bts_bb_trx *bb_transc = (struct gsm_bts_bb_trx *)fi->priv;

	switch (event) {
	case NM_EV_SHUTDOWN_START:
		/* Announce we start shutting down */
		oml_mo_state_chg(&bb_transc->mo, -1, -1, NM_STATE_SHUTDOWN);

		/* Propagate event to children: */
		ev_dispatch_children(bb_transc, event);
		break;
	case NM_EV_SHUTDOWN_FINISH:
		/* Propagate event to children: */
		ev_dispatch_children(bb_transc, event);
		nm_bb_transc_fsm_state_chg(fi, NM_BBTRANSC_ST_OP_DISABLED_NOTINSTALLED);
		break;
	default:
		OSMO_ASSERT(false);
	}
}

static struct osmo_fsm_state nm_bb_transc_fsm_states[] = {
	[NM_BBTRANSC_ST_OP_DISABLED_NOTINSTALLED] = {
		.in_event_mask =
			X(NM_EV_SW_ACT) |
			X(NM_EV_RSL_UP) |
			X(NM_EV_RSL_DOWN) |
			X(NM_EV_PHYLINK_UP) |
			X(NM_EV_PHYLINK_DOWN) |
			X(NM_EV_DISABLE),
		.out_state_mask =
			X(NM_BBTRANSC_ST_OP_DISABLED_NOTINSTALLED) |
			X(NM_BBTRANSC_ST_OP_DISABLED_OFFLINE),
		.name = "DISABLED_NOTINSTALLED",
		.onenter = st_op_disabled_notinstalled_on_enter,
		.action = st_op_disabled_notinstalled,
	},
	[NM_BBTRANSC_ST_OP_DISABLED_OFFLINE] = {
		.in_event_mask =
			X(NM_EV_RX_SETATTR) |
			X(NM_EV_RX_OPSTART) |
			X(NM_EV_OPSTART_ACK) |
			X(NM_EV_OPSTART_NACK) |
			X(NM_EV_RSL_UP) |
			X(NM_EV_RSL_DOWN) |
			X(NM_EV_PHYLINK_UP) |
			X(NM_EV_PHYLINK_DOWN) |
			X(NM_EV_DISABLE),
		.out_state_mask =
			X(NM_BBTRANSC_ST_OP_DISABLED_NOTINSTALLED) |
			X(NM_BBTRANSC_ST_OP_ENABLED),
		.name = "DISABLED_OFFLINE",
		.onenter = st_op_disabled_offline_on_enter,
		.action = st_op_disabled_offline,
	},
	[NM_BBTRANSC_ST_OP_ENABLED] = {
		.in_event_mask =
			X(NM_EV_RSL_DOWN) |
			X(NM_EV_PHYLINK_DOWN) |
			X(NM_EV_DISABLE),
		.out_state_mask =
			X(NM_BBTRANSC_ST_OP_DISABLED_NOTINSTALLED) |
			X(NM_BBTRANSC_ST_OP_DISABLED_OFFLINE),
		.name = "ENABLED",
		.onenter = st_op_enabled_on_enter,
		.action = st_op_enabled,
	},
};

struct osmo_fsm nm_bb_transc_fsm = {
	.name = "NM_BBTRANSC_OP",
	.states = nm_bb_transc_fsm_states,
	.num_states = ARRAY_SIZE(nm_bb_transc_fsm_states),
	.event_names = nm_fsm_event_names,
	.allstate_action = nm_bb_transc_allstate,
	.allstate_event_mask = X(NM_EV_SHUTDOWN_START) |
			       X(NM_EV_SHUTDOWN_FINISH),
	.log_subsys = DOML,
};

static __attribute__((constructor)) void nm_bb_transc_fsm_init(void)
{
	OSMO_ASSERT(osmo_fsm_register(&nm_bb_transc_fsm) == 0);
}
