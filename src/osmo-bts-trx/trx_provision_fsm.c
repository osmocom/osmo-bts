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

#include "l1_if.h"
#include "trx_provision_fsm.h"

#define X(s) (1 << (s))

#define trx_prov_fsm_state_chg(fi, NEXT_STATE) \
	osmo_fsm_inst_state_chg(fi, NEXT_STATE, 0, 0)

static void l1if_poweronoff_cb(struct trx_l1h *l1h, bool poweronoff, int rc)
{
	struct phy_instance *pinst = l1h->phy_inst;
	struct phy_link *plink = pinst->phy_link;

	plink->u.osmotrx.powered = poweronoff;
	plink->u.osmotrx.poweronoff_sent = false;

	if (poweronoff)
		osmo_fsm_inst_dispatch(l1h->provision_fi, TRX_PROV_EV_POWERON_CNF, (void*)(intptr_t)rc);
	else
		osmo_fsm_inst_dispatch(l1h->provision_fi, TRX_PROV_EV_POWEROFF_CNF, (void*)(intptr_t)rc);
}

static void l1if_getnompower_cb(struct trx_l1h *l1h, int nominal_power, int rc)
{
	struct phy_instance *pinst = l1h->phy_inst;
	struct gsm_bts_trx *trx = pinst->trx;

	LOGPPHI(pinst, DL1C, LOGL_DEBUG, "l1if_getnompower_cb(nominal_power=%d, rc=%d)\n", nominal_power, rc);

	l1if_trx_set_nominal_power(trx, nominal_power);
}

/*
 * transceiver provisioning
 */
int l1if_provision_transceiver_trx(struct trx_l1h *l1h)
{
	struct phy_instance *pinst = l1h->phy_inst;
	struct phy_link *plink = pinst->phy_link;

	/* During setup, pinst may still not be associated to a TRX nr */
	if (!pinst->trx) {
		LOGPPHI(pinst, DL1C, LOGL_INFO,
			"Delaying provision, TRX not yet assigned to phy instance\n");
		return -EIO;
	}

	if (phy_link_state_get(plink) == PHY_LINK_SHUTDOWN) {
		LOGPPHI(pinst, DL1C, LOGL_INFO,
			"Delaying provision, TRX not yet available\n");
		return -EIO;
	}

	if (l1h->config.enabled
	 && l1h->config.tsc_valid
	 && l1h->config.bsic_valid
	 && l1h->config.arfcn_valid) {
		/* before power on */
		if (!l1h->config.arfcn_sent) {
			trx_if_cmd_rxtune(l1h, l1h->config.arfcn);
			trx_if_cmd_txtune(l1h, l1h->config.arfcn);
			/* After TXTUNE is sent to TRX, get the tx nominal power
			 * (which may vary precisly on band/arfcn. Avoid sending
			 * it if we are forced by VTY to use a specific nominal
			 * power (because TRX may not support the command or
			 * provide broken values) */
			if (!l1h->config.nominal_power_set_by_vty)
				trx_if_cmd_getnompower(l1h, l1if_getnompower_cb);
			l1h->config.arfcn_sent = 1;
		}
		if (!l1h->config.tsc_sent) {
			trx_if_cmd_settsc(l1h, l1h->config.tsc);
			l1h->config.tsc_sent = 1;
		}
		if (!l1h->config.bsic_sent) {
			trx_if_cmd_setbsic(l1h, l1h->config.bsic);
			l1h->config.bsic_sent = 1;
		}

		/* Ask transceiver to use the newest TRXD header version if not using it yet */
		if (!l1h->config.setformat_sent) {
			if (l1h->config.trxd_hdr_ver_use != plink->u.osmotrx.trxd_hdr_ver_max) {
				trx_if_cmd_setformat(l1h, plink->u.osmotrx.trxd_hdr_ver_max);
				l1h->config.trxd_hdr_ver_req = plink->u.osmotrx.trxd_hdr_ver_max;
			} else {
				LOGPPHI(pinst, DL1C, LOGL_INFO,
					"No need to negotiate TRXD version, "
					"already using maximum configured one: %" PRIu8 "\n",
					l1h->config.trxd_hdr_ver_use);
			}
			l1h->config.setformat_sent = 1;
		}

		if (pinst->num == 0 && !plink->u.osmotrx.powered && !plink->u.osmotrx.poweronoff_sent) {
			trx_if_cmd_poweron(l1h, l1if_poweronoff_cb);
			plink->u.osmotrx.poweronoff_sent = true;
		}

		return 0;
	}
	LOGPPHI(pinst, DL1C, LOGL_INFO, "Delaying provision, TRX attributes not yet received from BSC:%s%s%s%s\n",
		l1h->config.enabled ? "" :" enable",
		l1h->config.tsc_valid ? "" : " tsc",
		l1h->config.bsic_valid ? "" : " bsic",
		l1h->config.arfcn_valid ? "" : " arfcn");
	return 1;
}

static void l1if_setslot_cb(struct trx_l1h *l1h, uint8_t tn, uint8_t type, int rc)
{
	struct phy_instance *pinst = l1h->phy_inst;
	struct gsm_bts_trx *trx = pinst->trx;
	struct gsm_bts_trx_ts *ts;
	enum gsm_phys_chan_config pchan;

	if (tn >= TRX_NR_TS) {
		LOGPPHI(pinst, DL1C, LOGL_ERROR, "transceiver SETSLOT invalid param TN (%" PRIu8 ")\n",
			tn);
		return;
	}

	pchan = transceiver_chan_type_2_pchan(type);
	if (pchan == GSM_PCHAN_UNKNOWN) {
		LOGPPHI(pinst, DL1C, LOGL_ERROR, "transceiver SETSLOT invalid param TS_TYPE (%" PRIu8 ")\n",
			type);
		return;
	}

	ts = &trx->ts[tn];
	LOGPPHI(pinst, DL1C, LOGL_DEBUG, "%s l1if_setslot_cb(as_pchan=%s),"
	     " calling cb_ts_connected(rc=%d)\n",
	     gsm_ts_name(ts), gsm_pchan_name(pchan), rc);
	cb_ts_connected(ts, rc);
}

/* Returns true if any TS changed, false otherwise */
static bool update_ts_data(struct trx_l1h *l1h, struct trx_prov_ev_cfg_ts_data* ts_data) {

	if (l1h->config.slottype[ts_data->tn] != ts_data->slottype ||
	    !l1h->config.slottype_valid[ts_data->tn]) {
		l1h->config.slottype[ts_data->tn] = ts_data->slottype;
		l1h->config.slottype_valid[ts_data->tn] = 1;
		l1h->config.slottype_sent[ts_data->tn] = 0;
		return true;
	}
	return false;
}


//////////////////////////
// FSM STATE ACTIONS
//////////////////////////

static void st_closed(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct trx_l1h *l1h = (struct trx_l1h *)fi->priv;

	switch(event) {
	case TRX_PROV_EV_OPEN:
		/* enable all slots */
		l1h->config.slotmask = 0xff;
		if (l1h->phy_inst->num == 0)
			trx_if_cmd_poweroff(l1h, NULL); /* TODO: jump to poweroff upon cb received */
		trx_prov_fsm_state_chg(fi, TRX_PROV_ST_OPEN_POWEROFF);
		break;
	}
}

static void st_open_poweroff(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct trx_l1h *l1h = (struct trx_l1h *)fi->priv;
	uint8_t bsic;
	uint16_t arfcn;
	uint16_t tsc;

	switch(event) {
	case TRX_PROV_EV_CFG_ENABLE:
		l1h->config.enabled =(bool)data;
		break;
	case TRX_PROV_EV_CFG_BSIC:
		bsic = (uint8_t)(intptr_t)data;
		if (l1h->config.bsic != bsic || !l1h->config.bsic_valid) {
			l1h->config.bsic = bsic;
			l1h->config.bsic_valid = 1;
			l1h->config.bsic_sent = 0;
		}
		break;
	case TRX_PROV_EV_CFG_ARFCN:
		arfcn = (uint16_t)(intptr_t)data;
		if (l1h->config.arfcn != arfcn || !l1h->config.arfcn_valid) {
			l1h->config.arfcn = arfcn;
			l1h->config.arfcn_valid = 1;
			l1h->config.arfcn_sent = 0;
		}
		break;
	case TRX_PROV_EV_CFG_TSC:
		tsc = (uint16_t)(intptr_t)data;
		if (l1h->config.tsc != tsc || !l1h->config.tsc_valid) {
			l1h->config.tsc = tsc;
			l1h->config.tsc_valid = 1;
			l1h->config.tsc_sent = 0;
		}
		break;
	case TRX_PROV_EV_CFG_TS:
		update_ts_data(l1h, (struct trx_prov_ev_cfg_ts_data*)data);
		break;
	}

	/* 0 = if we gathered all date and could go forward :*/
	if (l1if_provision_transceiver_trx(l1h) == 0) {
		if (l1h->phy_inst->num == 0)
			trx_prov_fsm_state_chg(fi, TRX_PROV_ST_OPEN_WAIT_POWERON_CNF);
		else
			trx_prov_fsm_state_chg(fi, TRX_PROV_ST_OPEN_POWERON);
	}
}

static void st_open_wait_power_cnf(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct trx_l1h *l1h = (struct trx_l1h *)fi->priv;
	struct phy_instance *pinst = l1h->phy_inst;
	struct phy_link *plink = pinst->phy_link;
	int rc;

	switch(event) {
	case TRX_PROV_EV_POWERON_CNF:
		rc = (uint16_t)(intptr_t)data;
		if (rc == 0 && plink->state != PHY_LINK_CONNECTED) {
			trx_sched_clock_started(pinst->trx->bts);
			phy_link_state_set(plink, PHY_LINK_CONNECTED);

			/* Begin to ramp up the power on all TRX associated with this phy */
			llist_for_each_entry(pinst, &plink->instances, list) {
				if (pinst->trx->mo.nm_state.administrative == NM_STATE_UNLOCKED)
					l1if_trx_start_power_ramp(pinst->trx, NULL);
			}

			trx_prov_fsm_state_chg(fi, TRX_PROV_ST_OPEN_POWERON);
		} else if (rc != 0 && plink->state != PHY_LINK_SHUTDOWN) {
			trx_sched_clock_stopped(pinst->trx->bts);
			phy_link_state_set(plink, PHY_LINK_SHUTDOWN);
		}
		break;
	case TRX_PROV_EV_CFG_TS:
		update_ts_data(l1h, (struct trx_prov_ev_cfg_ts_data*)data);
		break;
	}
}

static void st_open_poweron_on_enter(struct osmo_fsm_inst *fi, uint32_t prev_state)
{
	struct trx_l1h *l1h = (struct trx_l1h *)fi->priv;
	uint8_t tn;

	/* after power on */
	if (l1h->config.rxgain_valid && !l1h->config.rxgain_sent) {
		trx_if_cmd_setrxgain(l1h, l1h->config.rxgain);
		l1h->config.rxgain_sent = 1;
	}
	if (l1h->config.maxdly_valid && !l1h->config.maxdly_sent) {
		trx_if_cmd_setmaxdly(l1h, l1h->config.maxdly);
		l1h->config.maxdly_sent = 1;
	}
	if (l1h->config.maxdlynb_valid && !l1h->config.maxdlynb_sent) {
		trx_if_cmd_setmaxdlynb(l1h, l1h->config.maxdlynb);
		l1h->config.maxdlynb_sent = 1;
	}

	for (tn = 0; tn < TRX_NR_TS; tn++) {
		if (l1h->config.slottype_valid[tn]
		 && !l1h->config.slottype_sent[tn]) {
			trx_if_cmd_setslot(l1h, tn,
				l1h->config.slottype[tn], l1if_setslot_cb);
			l1h->config.slottype_sent[tn] = 1;
		}
	}
}

static void st_open_poweron(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct trx_l1h *l1h = (struct trx_l1h *)fi->priv;
	struct phy_instance *pinst = l1h->phy_inst;
	struct phy_link *plink = pinst->phy_link;
	struct trx_prov_ev_cfg_ts_data* ts_data;
	uint8_t tn;

	switch(event) {
	case TRX_PROV_EV_CLOSE:
		/* power off transceiver, if not already */
		if (l1h->config.enabled) {
			if (pinst->num == 0 && plink->u.osmotrx.powered && !plink->u.osmotrx.poweronoff_sent) {
				trx_if_cmd_poweroff(l1h, l1if_poweronoff_cb);
				plink->u.osmotrx.poweronoff_sent = true;
			}
			l1h->config.rxgain_sent = 0;
			l1h->config.maxdly_sent = 0;
			l1h->config.maxdlynb_sent = 0;
			for (tn = 0; tn < TRX_NR_TS; tn++)
				l1h->config.slottype_sent[tn] = 0;
		} else if (!pinst->phy_link->u.osmotrx.poweronoff_sent) {
			bts_model_trx_close_cb(pinst->trx, 0);
		} /* else: poweroff in progress, cb will be called upon TRXC RSP */

		if (pinst->num == 0)
			trx_prov_fsm_state_chg(fi, TRX_PROV_ST_OPEN_WAIT_POWEROFF_CNF);
		else
			trx_prov_fsm_state_chg(fi, TRX_PROV_ST_OPEN_POWEROFF);
		break;
	case TRX_PROV_EV_CFG_TS:
		ts_data = (struct trx_prov_ev_cfg_ts_data*)data;
		if (update_ts_data(l1h, ts_data)) {
			trx_if_cmd_setslot(l1h, ts_data->tn,
				l1h->config.slottype[ ts_data->tn], l1if_setslot_cb);
			l1h->config.slottype_sent[ ts_data->tn] = 1;
		}

		break;
	}
}

static void st_open_wait_poweroff_cnf(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct trx_l1h *l1h = (struct trx_l1h *)fi->priv;
	struct phy_instance *pinst = l1h->phy_inst;
	struct phy_link *plink = pinst->phy_link;
	int rc;

	switch(event) {
	case TRX_PROV_EV_POWEROFF_CNF:
		rc = (uint16_t)(intptr_t)data;
		if (plink->state != PHY_LINK_SHUTDOWN) {
			trx_sched_clock_stopped(pinst->trx->bts);
			phy_link_state_set(plink, PHY_LINK_SHUTDOWN);

			/* Notify TRX close on all TRX associated with this phy */
			llist_for_each_entry(pinst, &plink->instances, list) {
				bts_model_trx_close_cb(pinst->trx, rc);
			}
		}
		break;
	}
}

static struct osmo_fsm_state trx_prov_fsm_states[] = {
	[TRX_PROV_ST_CLOSED] = {
		.in_event_mask =
			X(TRX_PROV_EV_OPEN),
		.out_state_mask =
			X(TRX_PROV_ST_OPEN_POWEROFF),
		.name = "CLOSED",
		.action = st_closed,
	},
	[TRX_PROV_ST_OPEN_POWEROFF] = {
		.in_event_mask =
			X(TRX_PROV_EV_CFG_ENABLE) |
			X(TRX_PROV_EV_CFG_BSIC) |
			X(TRX_PROV_EV_CFG_ARFCN) |
			X(TRX_PROV_EV_CFG_TSC) |
			X(TRX_PROV_EV_CFG_TS),
		.out_state_mask =
			X(TRX_PROV_ST_OPEN_WAIT_POWERON_CNF) |
			X(TRX_PROV_ST_OPEN_POWERON),
		.name = "OPEN_POWEROFF",
		.action = st_open_poweroff,
	},
	[TRX_PROV_ST_OPEN_WAIT_POWERON_CNF] = {
		.in_event_mask =
			X(TRX_PROV_EV_POWERON_CNF) |
			X(TRX_PROV_EV_CFG_TS),
		.out_state_mask =
			X(TRX_PROV_ST_OPEN_POWERON),
		.name = "OPEN_WAIT_POWERON_CNF",
		.action = st_open_wait_power_cnf,
	},
	[TRX_PROV_ST_OPEN_POWERON] = {
		.in_event_mask =
			X(TRX_PROV_EV_POWEROFF) |
			X(TRX_PROV_EV_CFG_TS),
		.out_state_mask =
			X(TRX_PROV_ST_OPEN_WAIT_POWEROFF_CNF) |
			X(TRX_PROV_ST_OPEN_POWEROFF),
		.name = "OPEN_POWERON",
		.onenter = st_open_poweron_on_enter,
		.action = st_open_poweron,
	},
	[TRX_PROV_ST_OPEN_WAIT_POWEROFF_CNF] = {
		.in_event_mask =
			X(TRX_PROV_EV_POWEROFF_CNF),
		.out_state_mask =
			X(TRX_PROV_ST_OPEN_POWEROFF),
		.name = "OPEN_WAIT_POWEROFF_CNF",
		.action = st_open_wait_poweroff_cnf,
	},
};

const struct value_string trx_prov_fsm_event_names[] = {
	OSMO_VALUE_STRING(TRX_PROV_EV_OPEN),
	OSMO_VALUE_STRING(TRX_PROV_EV_CFG_ENABLE),
	OSMO_VALUE_STRING(TRX_PROV_EV_CFG_BSIC),
	OSMO_VALUE_STRING(TRX_PROV_EV_CFG_ARFCN),
	OSMO_VALUE_STRING(TRX_PROV_EV_CFG_TSC),
	OSMO_VALUE_STRING(TRX_PROV_EV_CFG_TS),
	OSMO_VALUE_STRING(TRX_PROV_EV_CFG_RXGAIN),
	OSMO_VALUE_STRING(TRX_PROV_EV_CFG_SETMAXDLY),
	OSMO_VALUE_STRING(TRX_PROV_EV_POWERON_CNF),
	OSMO_VALUE_STRING(TRX_PROV_EV_POWEROFF),
	OSMO_VALUE_STRING(TRX_PROV_EV_POWEROFF_CNF),
	OSMO_VALUE_STRING(TRX_PROV_EV_CLOSE),
	{ 0, NULL }
};

struct osmo_fsm trx_prov_fsm = {
	.name = "TRX_PROV",
	.states = trx_prov_fsm_states,
	.num_states = ARRAY_SIZE(trx_prov_fsm_states),
	.event_names = trx_prov_fsm_event_names,
	.log_subsys = DL1C,
};

static __attribute__((constructor)) void trx_prov_fsm_init(void)
{
	OSMO_ASSERT(osmo_fsm_register(&trx_prov_fsm) == 0);
}
