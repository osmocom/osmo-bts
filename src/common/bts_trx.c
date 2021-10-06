/* (C) 2008-2010 by Harald Welte <laforge@gnumonks.org>
 * (C) 2020-2021 by sysmocom - s.f.m.c. GmbH <info@sysmocom.de>
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

#include <osmocom/core/fsm.h>

#include <osmocom/gsm/abis_nm.h>

#include <osmo-bts/logging.h>
#include <osmo-bts/gsm_data.h>
#include <osmo-bts/bts_trx.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/bts_model.h>
#include <osmo-bts/rsl.h>
#include <osmo-bts/phy_link.h>
#include <osmo-bts/nm_common_fsm.h>

static int gsm_bts_trx_talloc_destructor(struct gsm_bts_trx *trx)
{
	unsigned int i;

	if (trx->bb_transc.mo.fi) {
		osmo_fsm_inst_free(trx->bb_transc.mo.fi);
		trx->bb_transc.mo.fi = NULL;
	}
	if (trx->mo.fi) {
		osmo_fsm_inst_free(trx->mo.fi);
		trx->mo.fi = NULL;
	}
	for (i = 0; i < TRX_NR_TS; i++) {
		struct gsm_bts_trx_ts *ts = &trx->ts[i];
		if (ts->mo.fi) {
			osmo_fsm_inst_free(ts->mo.fi);
			ts->mo.fi = NULL;
		}
	}
	return 0;
}

/* Initialize all logical channels of the given timeslot */
static void gsm_bts_trx_ts_init_lchan(struct gsm_bts_trx_ts *ts)
{
	unsigned int ln;

	for (ln = 0; ln < ARRAY_SIZE(ts->lchan); ln++) {
		struct gsm_lchan *lchan = &ts->lchan[ln];
		gsm_lchan_init(lchan, ts, ln);
	}
}

/* Initialize primary timeslots of the given transceiver */
static void gsm_bts_trx_init_ts(struct gsm_bts_trx *trx)
{
	unsigned int tn;

	for (tn = 0; tn < ARRAY_SIZE(trx->ts); tn++) {
		struct gsm_bts_trx_ts *ts = &trx->ts[tn];

		ts->trx = trx;
		ts->nr = tn;

		ts->mo.fi = osmo_fsm_inst_alloc(&nm_chan_fsm, trx, ts,
						LOGL_INFO, NULL);
		osmo_fsm_inst_update_id_f(ts->mo.fi, "%s-ts%u",
					  trx->bb_transc.mo.fi->id, ts->nr);
		gsm_mo_init(&ts->mo, trx->bts, NM_OC_CHANNEL,
			    trx->bts->nr, trx->nr, ts->nr);

		gsm_bts_trx_ts_init_lchan(ts);
	}
}

/* Initialize shadow timeslots of the given transceiver */
void gsm_bts_trx_init_shadow_ts(struct gsm_bts_trx *trx)
{
	unsigned int tn;

	for (tn = 0; tn < ARRAY_SIZE(trx->ts); tn++) {
		struct gsm_bts_trx_ts *ts;

		ts = talloc_zero(trx, struct gsm_bts_trx_ts);
		OSMO_ASSERT(ts != NULL);

		ts->trx = trx;
		ts->nr = tn;

		/* Link both primary and shadow */
		trx->ts[tn].vamos.peer = ts;
		ts->vamos.peer = &trx->ts[tn];
		ts->vamos.is_shadow = true;

		/* Shadow timeslot uses the primary's NM FSM */
		OSMO_ASSERT(trx->ts[tn].mo.fi != NULL);
		ts->mo.fi = trx->ts[tn].mo.fi;

		gsm_bts_trx_ts_init_lchan(ts);
	}
}

struct gsm_bts_trx *gsm_bts_trx_alloc(struct gsm_bts *bts)
{
	struct gsm_bts_trx *trx = talloc_zero(bts, struct gsm_bts_trx);

	if (!trx)
		return NULL;

	talloc_set_destructor(trx, gsm_bts_trx_talloc_destructor);

	trx->bts = bts;
	trx->nr = bts->num_trx++;

	trx->mo.fi = osmo_fsm_inst_alloc(&nm_rcarrier_fsm, trx, trx,
						  LOGL_INFO, NULL);
	osmo_fsm_inst_update_id_f(trx->mo.fi, "bts%d-trx%d", bts->nr, trx->nr);
	gsm_mo_init(&trx->mo, bts, NM_OC_RADIO_CARRIER,
		    bts->nr, trx->nr, 0xff);

	trx->bb_transc.mo.fi = osmo_fsm_inst_alloc(&nm_bb_transc_fsm, trx, &trx->bb_transc,
						  LOGL_INFO, NULL);
	osmo_fsm_inst_update_id_f(trx->bb_transc.mo.fi, "bts%d-trx%d", bts->nr, trx->nr);
	gsm_mo_init(&trx->bb_transc.mo, bts, NM_OC_BASEB_TRANSC,
		    bts->nr, trx->nr, 0xff);

	gsm_bts_trx_init_ts(trx);

	if (trx->nr != 0)
		trx->nominal_power = bts->c0->nominal_power;

	/* Default values for the power adjustments */
	trx->power_params.ramp.max_initial_pout_mdBm = to_mdB(0);
	trx->power_params.ramp.step_size_mdB = to_mdB(2);
	trx->power_params.ramp.step_interval_sec = 1;

	/* Default (fall-back) Dynamic Power Control parameters */
	trx->bs_dpc_params = &bts->bs_dpc_params;
	trx->ms_dpc_params = &bts->ms_dpc_params;

	/* IF BTS model doesn't DSP/HW support MS Power Control Loop, enable osmo algo by default: */
	if (!bts_internal_flag_get(trx->bts, BTS_INTERNAL_FLAG_MS_PWR_CTRL_DSP))
		trx->ms_pwr_ctl_soft = true;

	llist_add_tail(&trx->list, &bts->trx_list);

	return trx;
}

struct gsm_bts_trx *gsm_bts_trx_num(const struct gsm_bts *bts, int num)
{
	struct gsm_bts_trx *trx;

	if (num >= bts->num_trx)
		return NULL;

	llist_for_each_entry(trx, &bts->trx_list, list) {
		if (trx->nr == num)
			return trx;
	}

	return NULL;
}

static char ts2str[255];

char *gsm_trx_name(const struct gsm_bts_trx *trx)
{
	if (!trx)
		snprintf(ts2str, sizeof(ts2str), "(trx=NULL)");
	else
		snprintf(ts2str, sizeof(ts2str), "(bts=%d,trx=%d)",
			 trx->bts->nr, trx->nr);

	return ts2str;
}

const char *gsm_trx_unit_id(struct gsm_bts_trx *trx)
{
	static char buf[23];

	snprintf(buf, sizeof(buf), "%u/%u/%u", trx->bts->ip_access.site_id,
		trx->bts->ip_access.bts_id, trx->nr);
	return buf;
}

/* RSL link is established, send status report */
int trx_link_estab(struct gsm_bts_trx *trx)
{
	int rc;

	LOGPTRX(trx, DSUM, LOGL_INFO, "RSL link up\n");

	osmo_fsm_inst_dispatch(trx->mo.fi, NM_EV_RSL_UP, NULL);
	osmo_fsm_inst_dispatch(trx->bb_transc.mo.fi, NM_EV_RSL_UP, NULL);

	if ((rc = rsl_tx_rf_res(trx)) < 0)
		oml_tx_failure_event_rep(&trx->bb_transc.mo, NM_SEVER_MAJOR, OSMO_EVT_MAJ_RSL_FAIL,
					 "Failed to establish RSL link (%d)", rc);

	if (trx == trx->bts->c0)
		load_timer_start(trx->bts);

	return 0;
}


bool trx_ms_pwr_ctrl_is_osmo(const struct gsm_bts_trx *trx)
{
	return trx->ms_pwr_ctl_soft;
}
