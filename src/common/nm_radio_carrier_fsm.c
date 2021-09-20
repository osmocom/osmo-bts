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

#define nm_rcarrier_fsm_state_chg(fi, NEXT_STATE) \
	osmo_fsm_inst_state_chg(fi, NEXT_STATE, 0, 0)

//////////////////////////
// FSM STATE ACTIONS
//////////////////////////

static void st_op_disabled_notinstalled_on_enter(struct osmo_fsm_inst *fi, uint32_t prev_state)
{
	struct gsm_bts_trx *trx = (struct gsm_bts_trx *)fi->priv;
	trx->mo.opstart_success = false;
	oml_mo_state_chg(&trx->mo, NM_OPSTATE_DISABLED, NM_AVSTATE_NOT_INSTALLED);
}

static void st_op_disabled_notinstalled(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct gsm_bts_trx *trx = (struct gsm_bts_trx *)fi->priv;

	switch (event) {
	case NM_EV_SW_ACT:
		oml_mo_tx_sw_act_rep(&trx->mo);
		nm_rcarrier_fsm_state_chg(fi, NM_RCARRIER_ST_OP_DISABLED_OFFLINE);
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
	struct gsm_bts_trx *trx = (struct gsm_bts_trx *)fi->priv;
	unsigned int i;

	trx->mo.opstart_success = false;
	oml_mo_state_chg(&trx->mo, NM_OPSTATE_DISABLED, NM_AVSTATE_OFF_LINE);

	if (prev_state == NM_RCARRIER_ST_OP_ENABLED) {
		for (i = 0; i < TRX_NR_TS; i++) {
			struct gsm_bts_trx_ts *ts = &trx->ts[i];
			osmo_fsm_inst_dispatch(ts->mo.fi, NM_EV_RCARRIER_DISABLED, NULL);
		}
	}
}

static void st_op_disabled_offline(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct gsm_bts_trx *trx = (struct gsm_bts_trx *)fi->priv;
	bool phy_state_connected;
	bool rsl_link_connected;

	switch (event) {
	case NM_EV_OPSTART_ACK:
		trx->mo.opstart_success = true;
		oml_mo_opstart_ack(&trx->mo);
		break; /* check statechg below */
	case NM_EV_OPSTART_NACK:
		trx->mo.opstart_success = false;
		oml_mo_opstart_nack(&trx->mo, (int)(intptr_t)data);
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

	if (rsl_link_connected && phy_state_connected &&
	    trx->mo.opstart_success) {
		nm_rcarrier_fsm_state_chg(fi, NM_RCARRIER_ST_OP_ENABLED);
	} else {
		LOGPFSML(fi, LOGL_INFO, "Delay switch to operative state Enabled, wait for:%s%s%s\n",
			 rsl_link_connected ? "" : " rsl",
			 phy_state_connected ? "" : " phy",
			 trx->mo.opstart_success ? "" : " opstart");

	}
}

static void st_op_enabled_on_enter(struct osmo_fsm_inst *fi, uint32_t prev_state)
{
	struct gsm_bts_trx *trx = (struct gsm_bts_trx *)fi->priv;
	unsigned int tn;

	oml_mo_state_chg(&trx->mo, NM_OPSTATE_ENABLED, NM_AVSTATE_OK);
	/* Mark Dependency TS as Offline (ready to be Opstarted) */
	for (tn = 0; tn < TRX_NR_TS; tn++) {
		struct gsm_bts_trx_ts *ts = &trx->ts[tn];
		osmo_fsm_inst_dispatch(ts->mo.fi, NM_EV_RCARRIER_ENABLED, NULL);
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

	nm_rcarrier_fsm_state_chg(fi, NM_RCARRIER_ST_OP_DISABLED_OFFLINE);
}

static struct osmo_fsm_state nm_rcarrier_fsm_states[] = {
	[NM_RCARRIER_ST_OP_DISABLED_NOTINSTALLED] = {
		.in_event_mask =
			X(NM_EV_SW_ACT) |
			X(NM_EV_RSL_UP) |
			X(NM_EV_RSL_DOWN) |
			X(NM_EV_PHYLINK_UP) |
			X(NM_EV_PHYLINK_DOWN),
		.out_state_mask =
			X(NM_RCARRIER_ST_OP_DISABLED_OFFLINE),
		.name = "DISABLED_NOTINSTALLED",
		.onenter = st_op_disabled_notinstalled_on_enter,
		.action = st_op_disabled_notinstalled,
	},
	[NM_RCARRIER_ST_OP_DISABLED_OFFLINE] = {
		.in_event_mask =
			X(NM_EV_OPSTART_ACK) |
			X(NM_EV_OPSTART_NACK) |
			X(NM_EV_RSL_UP) |
			X(NM_EV_RSL_DOWN) |
			X(NM_EV_PHYLINK_UP) |
			X(NM_EV_PHYLINK_DOWN),
		.out_state_mask =
			X(NM_RCARRIER_ST_OP_ENABLED),
		.name = "DISABLED_OFFLINE",
		.onenter = st_op_disabled_offline_on_enter,
		.action = st_op_disabled_offline,
	},
	[NM_RCARRIER_ST_OP_ENABLED] = {
		.in_event_mask =
			X(NM_EV_RSL_DOWN) |
			X(NM_EV_PHYLINK_DOWN),
		.out_state_mask =
			X(NM_RCARRIER_ST_OP_DISABLED_OFFLINE),
		.name = "ENABLED",
		.onenter = st_op_enabled_on_enter,
		.action = st_op_enabled,
	},
};

struct osmo_fsm nm_rcarrier_fsm = {
	.name = "NM_RCARRIER_OP",
	.states = nm_rcarrier_fsm_states,
	.num_states = ARRAY_SIZE(nm_rcarrier_fsm_states),
	.event_names = nm_fsm_event_names,
	.log_subsys = DOML,
};

static __attribute__((constructor)) void nm_rcarrier_fsm_init(void)
{
	OSMO_ASSERT(osmo_fsm_register(&nm_rcarrier_fsm) == 0);
}
