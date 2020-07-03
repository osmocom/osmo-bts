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

#include <osmo-bts/logging.h>
#include <osmo-bts/gsm_data.h>
#include <osmo-bts/bts_trx.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/bts_model.h>
#include <osmo-bts/rsl.h>

struct gsm_bts_trx *gsm_bts_trx_alloc(struct gsm_bts *bts)
{
	struct gsm_bts_trx *trx = talloc_zero(bts, struct gsm_bts_trx);
	int k;

	if (!trx)
		return NULL;

	trx->bts = bts;
	trx->nr = bts->num_trx++;

	gsm_mo_init(&trx->mo, bts, NM_OC_RADIO_CARRIER,
		    bts->nr, trx->nr, 0xff);

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
		ts->tsc = -1;

		gsm_mo_init(&ts->mo, bts, NM_OC_CHANNEL,
			    bts->nr, trx->nr, ts->nr);

		for (l = 0; l < TS_MAX_LCHAN; l++) {
			struct gsm_lchan *lchan;
			char *name;
			lchan = &ts->lchan[l];

			lchan->ts = ts;
			lchan->nr = l;
			lchan->type = GSM_LCHAN_NONE;

			name = gsm_lchan_name_compute(lchan);
			lchan->name = talloc_strdup(trx, name);
			INIT_LLIST_HEAD(&lchan->sapi_cmds);
		}
	}

	if (trx->nr != 0)
		trx->nominal_power = bts->c0->nominal_power;

	llist_add_tail(&trx->list, &bts->trx_list);

	return trx;
}

/* Initialize the TRX data structures, called before config
 * file reading */
int bts_trx_init(struct gsm_bts_trx *trx)
{
	/* initialize bts data structure */
	struct trx_power_params *tpp = &trx->power_params;
	int rc, i;

	for (i = 0; i < ARRAY_SIZE(trx->ts); i++) {
		struct gsm_bts_trx_ts *ts = &trx->ts[i];
		int k;

		for (k = 0; k < ARRAY_SIZE(ts->lchan); k++) {
			struct gsm_lchan *lchan = &ts->lchan[k];
			INIT_LLIST_HEAD(&lchan->dl_tch_queue);
		}
	}
	/* Default values for the power adjustments */
	tpp->ramp.max_initial_pout_mdBm = to_mdB(0);
	tpp->ramp.step_size_mdB = to_mdB(2);
	tpp->ramp.step_interval_sec = 1;

	/* IF BTS model doesn't DSP/HW support MS Power Control Loop, enable osmo algo by default: */
	if (!bts_internal_flag_get(trx->bts, BTS_INTERNAL_FLAG_MS_PWR_CTRL_DSP))
		trx->ms_pwr_ctl_soft = true;

	rc = bts_model_trx_init(trx);
	if (rc < 0) {
		llist_del(&trx->list);
		return rc;
	}
	return 0;
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
	uint8_t radio_state = link ?  NM_OPSTATE_ENABLED : NM_OPSTATE_DISABLED;
	int rc;

	LOGP(DSUM, LOGL_INFO, "RSL link (TRX %02x) state changed to %s, sending Status'.\n",
		trx->nr, link ? "up" : "down");

	oml_mo_state_chg(&trx->mo, radio_state, NM_AVSTATE_OK);

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

/* set the availability of the TRX (used by PHY driver) */
int trx_set_available(struct gsm_bts_trx *trx, int avail)
{
	int tn;

	LOGP(DSUM, LOGL_INFO, "TRX(%d): Setting available = %d\n",
		trx->nr, avail);
	if (avail) {
		int op_state = trx->rsl_link ?  NM_OPSTATE_ENABLED : NM_OPSTATE_DISABLED;
		oml_mo_state_chg(&trx->mo, op_state, NM_AVSTATE_OK);
		oml_mo_state_chg(&trx->bb_transc.mo, -1, NM_AVSTATE_OK);
		for (tn = 0; tn < ARRAY_SIZE(trx->ts); tn++)
			oml_mo_state_chg(&trx->ts[tn].mo, op_state, NM_AVSTATE_OK);
	} else {
		oml_mo_state_chg(&trx->mo,  NM_OPSTATE_DISABLED, NM_AVSTATE_NOT_INSTALLED);
		oml_mo_state_chg(&trx->bb_transc.mo, -1, NM_AVSTATE_NOT_INSTALLED);

		for (tn = 0; tn < ARRAY_SIZE(trx->ts); tn++)
			oml_mo_state_chg(&trx->ts[tn].mo, NM_OPSTATE_DISABLED, NM_AVSTATE_NOT_INSTALLED);
	}
	return 0;
}


bool trx_ms_pwr_ctrl_is_osmo(struct gsm_bts_trx *trx)
{
	return trx->ms_pwr_ctl_soft;
}
