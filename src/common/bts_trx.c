/* (C) 2008-2010 by Harald Welte <laforge@gnumonks.org>
 * (C) 2020 by sysmocom - s.f.m.c. GmbH <info@sysmocom.de>
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

struct gsm_bts_trx *gsm_bts_trx_alloc(struct gsm_bts *bts)
{
	struct gsm_bts_trx *trx = talloc_zero(bts, struct gsm_bts_trx);
	int k;

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

	for (k = 0; k < TRX_NR_TS; k++) {
		struct gsm_bts_trx_ts *ts = &trx->ts[k];
		int l;

		ts->trx = trx;
		ts->nr = k;
		ts->pchan = GSM_PCHAN_NONE;
		ts->dyn.pchan_is = GSM_PCHAN_NONE;
		ts->dyn.pchan_want = GSM_PCHAN_NONE;

		ts->mo.fi = osmo_fsm_inst_alloc(&nm_chan_fsm, trx, ts,
						     LOGL_INFO, NULL);
		osmo_fsm_inst_update_id_f(ts->mo.fi, "bts%d-trx%d-ts%d",
					  bts->nr, trx->nr, ts->nr);
		gsm_mo_init(&ts->mo, bts, NM_OC_CHANNEL,
			    bts->nr, trx->nr, ts->nr);

		for (l = 0; l < TS_MAX_LCHAN; l++) {
			struct gsm_lchan *lchan;
			lchan = &ts->lchan[l];

			lchan->ts = ts;
			lchan->nr = l;
			lchan->type = GSM_LCHAN_NONE;
			gsm_lchan_name_update(lchan);

			INIT_LLIST_HEAD(&lchan->sapi_cmds);
			INIT_LLIST_HEAD(&lchan->dl_tch_queue);
		}
	}

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
	struct e1inp_sign_link *link = trx->rsl_link;
	int rc;

	LOGPTRX(trx, DSUM, LOGL_INFO, "RSL link %s\n",
		link ? "up" : "down");

	osmo_fsm_inst_dispatch(trx->mo.fi, link ? NM_EV_RSL_UP : NM_EV_RSL_DOWN, NULL);
	osmo_fsm_inst_dispatch(trx->bb_transc.mo.fi, link ? NM_EV_RSL_UP : NM_EV_RSL_DOWN, NULL);

	if (link)
		rc = rsl_tx_rf_res(trx);
	else
		rc = bts_model_trx_deact_rf(trx);
	if (rc < 0) {
		oml_tx_failure_event_rep(&trx->bb_transc.mo, NM_SEVER_MAJOR, OSMO_EVT_MAJ_RSL_FAIL,
					 link ?
					 "Failed to establish RSL link (%d)" :
					 "Failed to deactivate RF (%d)", rc);
	}

	return 0;
}


bool trx_ms_pwr_ctrl_is_osmo(const struct gsm_bts_trx *trx)
{
	return trx->ms_pwr_ctl_soft;
}
