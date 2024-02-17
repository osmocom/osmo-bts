/* NM GPRS NSVC FSM */

/* (C) 2023 by sysmocom - s.f.m.c. GmbH <info@sysmocom.de>
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
#include <osmo-bts/bts_sm.h>
#include <osmo-bts/rsl.h>
#include <osmo-bts/nm_common_fsm.h>

#define X(s) (1 << (s))

#define nm_gprs_nsvc_fsm_state_chg(fi, NEXT_STATE) \
	osmo_fsm_inst_state_chg(fi, NEXT_STATE, 0, 0)

/* Can the GPRS Cell be enabled (OPSTARTed)? aka should it stay in "Disabled Dependency" state? */
static bool gprs_nsvc_can_be_enabled(struct gsm_gprs_nsvc *nsvc)
{
	return nsvc->nse->mo.nm_state.operational == NM_OPSTATE_ENABLED;
}


//////////////////////////
// FSM STATE ACTIONS
//////////////////////////

static void st_op_disabled_notinstalled_on_enter(struct osmo_fsm_inst *fi, uint32_t prev_state)
{
	struct gsm_gprs_nsvc *nsvc = (struct gsm_gprs_nsvc *)fi->priv;
	/* Reset state here: */

	nsvc->mo.setattr_success = false;
	nsvc->mo.opstart_success = false;
	oml_mo_state_chg(&nsvc->mo, NM_OPSTATE_DISABLED, NM_AVSTATE_NOT_INSTALLED, NM_STATE_LOCKED);
}

static void st_op_disabled_notinstalled(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct gsm_gprs_nsvc *nsvc = (struct gsm_gprs_nsvc *)fi->priv;

	switch (event) {
	case NM_EV_OML_UP:
		/* automatic SW_ACT upon OML link establishment: */
		oml_mo_tx_sw_act_rep(&nsvc->mo);
		if (gprs_nsvc_can_be_enabled(nsvc))
			nm_gprs_nsvc_fsm_state_chg(fi, NM_GPRS_NSVC_ST_OP_DISABLED_OFFLINE);
		else
			nm_gprs_nsvc_fsm_state_chg(fi, NM_GPRS_NSVC_ST_OP_DISABLED_DEPENDENCY);
		return;
	default:
		OSMO_ASSERT(0);
	}
}

static void st_op_disabled_dependency_on_enter(struct osmo_fsm_inst *fi, uint32_t prev_state)
{
	struct gsm_gprs_nsvc *nsvc = (struct gsm_gprs_nsvc *)fi->priv;
	nsvc->mo.setattr_success = false;
	nsvc->mo.opstart_success = false;
	oml_mo_state_chg(&nsvc->mo, NM_OPSTATE_DISABLED, NM_AVSTATE_DEPENDENCY, -1);
}

static void st_op_disabled_dependency(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct gsm_gprs_nsvc *nsvc = (struct gsm_gprs_nsvc *)fi->priv;
	struct gsm_bts *bts = gsm_gprs_nse_get_bts(nsvc->nse);
	struct nm_fsm_ev_setattr_data *setattr_data;
	int rc;

	switch (event) {
	case NM_EV_RX_SETATTR:
		setattr_data = (struct nm_fsm_ev_setattr_data *)data;
		rc = bts_model_apply_oml(bts, setattr_data->msg,
					 &nsvc->mo, nsvc);
		nsvc->mo.setattr_success = rc == 0;
		oml_fom_ack_nack_copy_msg(setattr_data->msg, rc);
		break;
	case NM_EV_RX_OPSTART:
		if (!nsvc->mo.setattr_success) {
			oml_mo_opstart_nack(&nsvc->mo, NM_NACK_CANT_PERFORM);
			return;
		}
		bts_model_opstart(bts, &nsvc->mo, nsvc);
		break;
	case NM_EV_OPSTART_ACK:
		nsvc->mo.opstart_success = true;
		oml_mo_opstart_ack(&nsvc->mo);
		nm_gprs_nsvc_fsm_state_chg(fi, NM_CHAN_ST_OP_ENABLED);
		return;
	case NM_EV_OPSTART_NACK:
		nsvc->mo.opstart_success = false;
		oml_mo_opstart_nack(&nsvc->mo, (int)(intptr_t)data);
		return;
	default:
		OSMO_ASSERT(0);
	}
}

static void st_op_disabled_offline_on_enter(struct osmo_fsm_inst *fi, uint32_t prev_state)
{
	struct gsm_gprs_nsvc *nsvc = (struct gsm_gprs_nsvc *)fi->priv;
	nsvc->mo.opstart_success = false;
	oml_mo_state_chg(&nsvc->mo, NM_OPSTATE_DISABLED, NM_AVSTATE_OFF_LINE, -1);
}

static void st_op_disabled_offline(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct gsm_gprs_nsvc *nsvc = (struct gsm_gprs_nsvc *)fi->priv;
	struct gsm_bts *bts = gsm_gprs_nse_get_bts(nsvc->nse);
	struct nm_fsm_ev_setattr_data *setattr_data;
	int rc;

	switch (event) {
	case NM_EV_RX_SETATTR:
		setattr_data = (struct nm_fsm_ev_setattr_data *)data;
		rc = bts_model_apply_oml(bts, setattr_data->msg, &nsvc->mo, bts);
		nsvc->mo.setattr_success = rc == 0;
		oml_fom_ack_nack_copy_msg(setattr_data->msg, rc);
		break;
	case NM_EV_RX_OPSTART:
		if (!nsvc->mo.setattr_success) {
			oml_mo_opstart_nack(&nsvc->mo, NM_NACK_CANT_PERFORM);
			return;
		}
		bts_model_opstart(bts, &nsvc->mo, bts);
		break;
	case NM_EV_OPSTART_ACK:
		nsvc->mo.opstart_success = true;
		oml_mo_opstart_ack(&nsvc->mo);
		nm_gprs_nsvc_fsm_state_chg(fi, NM_GPRS_NSVC_ST_OP_ENABLED);
		break; /* check statechg below */
	case NM_EV_OPSTART_NACK:
		nsvc->mo.opstart_success = false;
		oml_mo_opstart_nack(&nsvc->mo, (int)(intptr_t)data);
		return;
	default:
		OSMO_ASSERT(0);
	}
}

static void st_op_enabled_on_enter(struct osmo_fsm_inst *fi, uint32_t prev_state)
{
	struct gsm_gprs_nsvc *nsvc = (struct gsm_gprs_nsvc *)fi->priv;
	oml_mo_state_chg(&nsvc->mo, NM_OPSTATE_ENABLED, NM_AVSTATE_OK, -1);
}

static void st_op_enabled(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
}

static void nm_gprs_nsvc_allstate(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct gsm_gprs_nsvc *nsvc = (struct gsm_gprs_nsvc *)fi->priv;

	switch (event) {
	case NM_EV_SHUTDOWN_START:
		/* Announce we start shutting down */
		oml_mo_state_chg(&nsvc->mo, -1, -1, NM_STATE_SHUTDOWN);
		break;
	case NM_EV_SHUTDOWN_FINISH:
		nm_gprs_nsvc_fsm_state_chg(fi, NM_GPRS_NSVC_ST_OP_DISABLED_NOTINSTALLED);
		break;
	default:
		OSMO_ASSERT(false);
	}
}

static struct osmo_fsm_state nm_gprs_nsvc_fsm_states[] = {
	[NM_GPRS_NSVC_ST_OP_DISABLED_NOTINSTALLED] = {
		.in_event_mask =
			X(NM_EV_OML_UP),
		.out_state_mask =
			X(NM_GPRS_NSVC_ST_OP_DISABLED_NOTINSTALLED) |
			X(NM_GPRS_NSVC_ST_OP_DISABLED_DEPENDENCY) |
			X(NM_GPRS_NSVC_ST_OP_DISABLED_OFFLINE),
		.name = "DISABLED_NOTINSTALLED",
		.onenter = st_op_disabled_notinstalled_on_enter,
		.action = st_op_disabled_notinstalled,
	},
	[NM_GPRS_NSVC_ST_OP_DISABLED_DEPENDENCY] = {
		.in_event_mask =
			X(NM_EV_RX_SETATTR) |
			X(NM_EV_RX_OPSTART) | /* backward compatibility, buggy BSC */
			X(NM_EV_OPSTART_ACK) | /* backward compatibility, buggy BSC */
			X(NM_EV_OPSTART_NACK), /* backward compatibility, buggy BSC */
		.out_state_mask =
			X(NM_CHAN_ST_OP_DISABLED_NOTINSTALLED) |
			X(NM_CHAN_ST_OP_DISABLED_OFFLINE) |
			X(NM_CHAN_ST_OP_ENABLED), /* backward compatibility, buggy BSC */
		.name = "DISABLED_DEPENDENCY",
		.onenter = st_op_disabled_dependency_on_enter,
		.action = st_op_disabled_dependency,
	},
	[NM_GPRS_NSVC_ST_OP_DISABLED_OFFLINE] = {
		.in_event_mask =
			X(NM_EV_RX_SETATTR) |
			X(NM_EV_RX_OPSTART) |
			X(NM_EV_OPSTART_ACK) |
			X(NM_EV_OPSTART_NACK),
		.out_state_mask =
			X(NM_GPRS_NSVC_ST_OP_DISABLED_NOTINSTALLED) |
			X(NM_GPRS_NSVC_ST_OP_ENABLED),
		.name = "DISABLED_OFFLINE",
		.onenter = st_op_disabled_offline_on_enter,
		.action = st_op_disabled_offline,
	},
	[NM_GPRS_NSVC_ST_OP_ENABLED] = {
		.in_event_mask = 0,
		.out_state_mask =
			X(NM_GPRS_NSVC_ST_OP_DISABLED_NOTINSTALLED),
		.name = "ENABLED",
		.onenter = st_op_enabled_on_enter,
		.action = st_op_enabled,
	},
};

struct osmo_fsm nm_gprs_nsvc_fsm = {
	.name = "NM_GPRS_NSVC_OP",
	.states = nm_gprs_nsvc_fsm_states,
	.num_states = ARRAY_SIZE(nm_gprs_nsvc_fsm_states),
	.event_names = nm_fsm_event_names,
	.allstate_action = nm_gprs_nsvc_allstate,
	.allstate_event_mask = X(NM_EV_SHUTDOWN_START) |
			       X(NM_EV_SHUTDOWN_FINISH),
	.log_subsys = DOML,
};

static __attribute__((constructor)) void nm_gprs_nsvc_fsm_init(void)
{
	OSMO_ASSERT(osmo_fsm_register(&nm_gprs_nsvc_fsm) == 0);
}
