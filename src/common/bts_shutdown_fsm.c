/* BTS shutdown FSM */

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

#include <osmocom/core/fsm.h>
#include <osmocom/core/tdef.h>
#include <osmocom/gsm/protocol/gsm_12_21.h>

#include <osmo-bts/bts_shutdown_fsm.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/gsm_data.h>
#include <osmo-bts/bts_model.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/nm_common_fsm.h>

#define X(s) (1 << (s))

#define BTS_SHUTDOWN_POWER_RAMP_TGT -10

static const struct osmo_tdef_state_timeout bts_shutdown_fsm_timeouts[32] = {
	[BTS_SHUTDOWN_ST_WAIT_RAMP_DOWN_COMPL] = { .T = -1 },
	[BTS_SHUTDOWN_ST_WAIT_TRX_CLOSED] = { .T = -2 },
};

#define bts_shutdown_fsm_state_chg(fi, NEXT_STATE) \
	osmo_tdef_fsm_inst_state_chg(fi, NEXT_STATE, bts_shutdown_fsm_timeouts, ((struct gsm_bts *)fi->priv)->T_defs, -1)

static unsigned int count_trx_operational(struct gsm_bts *bts) {
	unsigned int count = 0;
	struct gsm_bts_trx *trx;
	llist_for_each_entry(trx, &bts->trx_list, list) {
		if (trx->mo.nm_state.operational == NM_OPSTATE_ENABLED)
			count++;
	}
	return count;
}

static void st_none(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct gsm_bts *bts = (struct gsm_bts *)fi->priv;
	unsigned int count;
	switch(event) {
	case BTS_SHUTDOWN_EV_START:
		/* Firt announce to NM objects that we are starting a shutdown procedure: */
		osmo_fsm_inst_dispatch(bts->site_mgr.mo.fi, NM_EV_SHUTDOWN_START, NULL);

		count = count_trx_operational(bts);
		if (count) {
			bts_shutdown_fsm_state_chg(fi, BTS_SHUTDOWN_ST_WAIT_RAMP_DOWN_COMPL);
		} else {
			/* we can skip ramp down since no TRX is running anyway.
			 * Let's jump into WAIT_TRX_CLOSED to make sure we
			 * tell lower layer to close all TRX in case there's some
			 * open() WIP */
			LOGPFSML(fi, LOGL_INFO, "No TRX is operational, skipping power ramp down\n");
			bts_shutdown_fsm_state_chg(fi, BTS_SHUTDOWN_ST_WAIT_TRX_CLOSED);
		}
		break;
	}
}

static void ramp_down_compl_cb(struct gsm_bts_trx *trx) {
       osmo_fsm_inst_dispatch(trx->bts->shutdown_fi, BTS_SHUTDOWN_EV_TRX_RAMP_COMPL, trx);
}

static void st_wait_ramp_down_compl_on_enter(struct osmo_fsm_inst *fi, uint32_t prev_state)
{
	struct gsm_bts *bts = (struct gsm_bts *)fi->priv;
	struct gsm_bts_trx *trx;

	llist_for_each_entry(trx, &bts->trx_list, list) {
		if (trx->mo.nm_state.operational != NM_OPSTATE_ENABLED)
			continue;
		power_ramp_start(trx, to_mdB(BTS_SHUTDOWN_POWER_RAMP_TGT), 1, ramp_down_compl_cb);
	}
}

static void st_wait_ramp_down_compl(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct gsm_bts *bts = (struct gsm_bts *)fi->priv;
	struct gsm_bts_trx *src_trx;
	unsigned int remaining = 0;
	struct gsm_bts_trx *trx;

	switch(event) {
	case BTS_SHUTDOWN_EV_TRX_RAMP_COMPL:
		src_trx = (struct gsm_bts_trx *)data;

		llist_for_each_entry(trx, &bts->trx_list, list) {
			if (trx->mo.nm_state.operational == NM_OPSTATE_ENABLED &&
			    trx->power_params.p_total_cur_mdBm > BTS_SHUTDOWN_POWER_RAMP_TGT)
				remaining++;
		}

		LOGPFSML(fi, LOGL_INFO, "%s Ramping down complete, %u TRX remaining\n",
			 gsm_trx_name(src_trx), remaining);
		if (remaining == 0)
			bts_shutdown_fsm_state_chg(fi, BTS_SHUTDOWN_ST_WAIT_TRX_CLOSED);
		break;
	}
}

static void st_wait_trx_closed_on_enter(struct osmo_fsm_inst *fi, uint32_t prev_state)
{
	struct gsm_bts *bts = (struct gsm_bts *)fi->priv;
	struct gsm_bts_trx *trx;
	llist_for_each_entry_reverse(trx, &bts->trx_list, list) {
		bts_model_trx_deact_rf(trx);
	}
	llist_for_each_entry_reverse(trx, &bts->trx_list, list) {
		bts_model_trx_close(trx);
	}
	/* Now wait until all TRX are closed asynchronously, we'll get feedback through events... */
}

static void st_wait_trx_closed(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct gsm_bts *bts = (struct gsm_bts *)fi->priv;
	struct gsm_bts_trx *src_trx;
	unsigned int remaining;

	switch(event) {
	case BTS_SHUTDOWN_EV_TRX_CLOSED:
		src_trx = (struct gsm_bts_trx *)data;
		remaining = count_trx_operational(bts);

		LOGPFSML(fi, LOGL_INFO, "%s TRX closed, %u TRX remaining\n",
			 gsm_trx_name(src_trx), remaining);
		if (remaining == 0)
			bts_shutdown_fsm_state_chg(fi, BTS_SHUTDOWN_ST_EXIT);
		break;
	}
}

static void st_exit_on_enter(struct osmo_fsm_inst *fi, uint32_t prev_state)
{
	LOGPFSML(fi, LOGL_NOTICE, "Shutdown process completed successfuly, exiting process\n");
	exit(0);
}

static struct osmo_fsm_state bts_shutdown_fsm_states[] = {
	[BTS_SHUTDOWN_ST_NONE] = {
		.in_event_mask =
			X(BTS_SHUTDOWN_EV_START),
		.out_state_mask =
			X(BTS_SHUTDOWN_ST_WAIT_RAMP_DOWN_COMPL) |
			X(BTS_SHUTDOWN_ST_WAIT_TRX_CLOSED),
		.name = "NONE",
		.action = st_none,
	},
	[BTS_SHUTDOWN_ST_WAIT_RAMP_DOWN_COMPL] = {
		.in_event_mask =
			X(BTS_SHUTDOWN_EV_TRX_RAMP_COMPL),
		.out_state_mask =
			X(BTS_SHUTDOWN_ST_WAIT_TRX_CLOSED),
		.name = "WAIT_RAMP_DOWN_COMPL",
		.onenter = st_wait_ramp_down_compl_on_enter,
		.action = st_wait_ramp_down_compl,
	},
	[BTS_SHUTDOWN_ST_WAIT_TRX_CLOSED] = {
		.in_event_mask =
			X(BTS_SHUTDOWN_EV_TRX_CLOSED),
		.out_state_mask =
			X(BTS_SHUTDOWN_ST_EXIT),
		.name = "WAIT_TRX_CLOSED",
		.onenter = st_wait_trx_closed_on_enter,
		.action = st_wait_trx_closed,
	},
	[BTS_SHUTDOWN_ST_EXIT] = {
		.name = "EXIT",
		.onenter = st_exit_on_enter,
	}
};

const struct value_string bts_shutdown_fsm_event_names[] = {
	OSMO_VALUE_STRING(BTS_SHUTDOWN_EV_START),
	OSMO_VALUE_STRING(BTS_SHUTDOWN_EV_TRX_RAMP_COMPL),
	OSMO_VALUE_STRING(BTS_SHUTDOWN_ST_WAIT_TRX_CLOSED),
	{ 0, NULL }
};

int bts_shutdown_fsm_timer_cb(struct osmo_fsm_inst *fi)
{
	switch (fi->state) {
	case BTS_SHUTDOWN_ST_WAIT_RAMP_DOWN_COMPL:
		LOGPFSML(fi, LOGL_ERROR, "Timer expired waiting for ramp down complete\n");
		bts_shutdown_fsm_state_chg(fi, BTS_SHUTDOWN_ST_WAIT_TRX_CLOSED);
		break;
	case BTS_SHUTDOWN_ST_WAIT_TRX_CLOSED:
		LOGPFSML(fi, LOGL_ERROR, "Timer expired waiting for TRX close\n");
		bts_shutdown_fsm_state_chg(fi, BTS_SHUTDOWN_ST_EXIT);
		break;
	default:
		OSMO_ASSERT(false);
	}
	return 0;
}

struct osmo_fsm bts_shutdown_fsm = {
	.name = "BTS_SHUTDOWN",
	.states = bts_shutdown_fsm_states,
	.num_states = ARRAY_SIZE(bts_shutdown_fsm_states),
	.event_names = bts_shutdown_fsm_event_names,
	.log_subsys = DOML,
	.timer_cb = bts_shutdown_fsm_timer_cb,
};

static __attribute__((constructor)) void bts_shutdown_fsm_init(void)
{
	OSMO_ASSERT(osmo_fsm_register(&bts_shutdown_fsm) == 0);
}

void bts_shutdown(struct gsm_bts *bts, const char *reason)
{
	struct osmo_fsm_inst *fi = bts->shutdown_fi;
	if (fi->state != BTS_SHUTDOWN_ST_NONE) {
		LOGPFSML(fi, LOGL_NOTICE, "BTS is already being shutdown.\n");
		return;
	}

	LOGPFSML(fi, LOGL_NOTICE, "Shutting down BTS, reason: %s\n", reason);
	osmo_fsm_inst_dispatch(fi, BTS_SHUTDOWN_EV_START, NULL);
}

void bts_model_trx_close_cb(struct gsm_bts_trx *trx, int rc)
{
	struct osmo_fsm_inst *fi = trx->bts->shutdown_fi;
	LOGPFSML(fi, LOGL_DEBUG, "%s Received TRX close cb rc=%d\n", gsm_trx_name(trx), rc);
	osmo_fsm_inst_dispatch(fi, BTS_SHUTDOWN_EV_TRX_CLOSED, trx);
}
