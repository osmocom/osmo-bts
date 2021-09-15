/* TRX provision FSM */

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


void l1if_rxtune_cb(struct trx_l1h *l1h, int rc)
{
	osmo_fsm_inst_dispatch(l1h->provision_fi, TRX_PROV_EV_RXTUNE_CNF, (void*)(intptr_t)rc);
}

void l1if_txtune_cb(struct trx_l1h *l1h, int rc)
{
	osmo_fsm_inst_dispatch(l1h->provision_fi, TRX_PROV_EV_TXTUNE_CNF, (void*)(intptr_t)rc);
}

void l1if_settsc_cb(struct trx_l1h *l1h, int rc)
{
	osmo_fsm_inst_dispatch(l1h->provision_fi, TRX_PROV_EV_SETTSC_CNF, (void*)(intptr_t)rc);
}

void l1if_setbsic_cb(struct trx_l1h *l1h, int rc)
{
	osmo_fsm_inst_dispatch(l1h->provision_fi, TRX_PROV_EV_SETBSIC_CNF, (void*)(intptr_t)rc);
}

static void l1if_getnompower_cb(struct trx_l1h *l1h, int nominal_power, int rc)
{
	struct phy_instance *pinst = l1h->phy_inst;
	LOGPPHI(pinst, DL1C, LOGL_DEBUG, "l1if_getnompower_cb(nominal_power=%d, rc=%d)\n", nominal_power, rc);
	osmo_fsm_inst_dispatch(l1h->provision_fi, TRX_PROV_EV_NOMTXPOWER_CNF, (void*)(intptr_t)nominal_power);
}

void l1if_setformat_cb(struct trx_l1h *l1h, int rc)
{
	osmo_fsm_inst_dispatch(l1h->provision_fi, TRX_PROV_EV_SETFORMAT_CNF, (void*)(intptr_t)rc);
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

	/* before power on */
	if (l1h->config.arfcn_valid) {
		if (!l1h->config.rxtune_sent) {
			trx_if_cmd_rxtune(l1h, l1h->config.arfcn, l1if_rxtune_cb);
			l1h->config.rxtune_sent = true;
			l1h->config.rxtune_acked = false;
		}
		if (!l1h->config.txtune_sent) {
			trx_if_cmd_txtune(l1h, l1h->config.arfcn, l1if_txtune_cb);
			l1h->config.txtune_sent = true;
			l1h->config.txtune_acked = false;
		}
		if (l1h->config.txtune_acked) {
			/* After TXTUNE is sent to TRX, get the tx nominal power
			 * (which may vary precisly on band/arfcn. Avoid sending
			 * it if we are forced by VTY to use a specific nominal
			 * power (because TRX may not support the command or
			 * provide broken values) */
			if (!l1h->config.nominal_power_set_by_vty && !l1h->config.nomtxpower_sent) {
				trx_if_cmd_getnompower(l1h, l1if_getnompower_cb);
				l1h->config.nomtxpower_sent = true;
				l1h->config.nomtxpower_acked = false;
			}
		}
	}
	if (!pinst->phy_link->u.osmotrx.use_legacy_setbsic &&
	    l1h->config.tsc_valid && !l1h->config.tsc_sent) {
		trx_if_cmd_settsc(l1h, l1h->config.tsc, l1if_settsc_cb);
		l1h->config.tsc_sent = true;
		l1h->config.tsc_acked = false;
	}
	if (pinst->phy_link->u.osmotrx.use_legacy_setbsic &&
	    l1h->config.bsic_valid && !l1h->config.bsic_sent) {
		trx_if_cmd_setbsic(l1h, l1h->config.bsic, l1if_setbsic_cb);
		l1h->config.bsic_sent = true;
		l1h->config.bsic_acked = false;
	}

	/* Ask transceiver to use the newest TRXD PDU version if not using it yet */
	if (!l1h->config.setformat_sent) {
		l1h->config.setformat_sent = true;
		if (plink->u.osmotrx.trxd_pdu_ver_max == 0) {
			LOGPPHI(pinst, DL1C, LOGL_INFO,
				"No need to negotiate max TRXD version 0");
			l1h->config.trxd_pdu_ver_use = 0;
			l1h->config.setformat_acked = true;
		} else {
			trx_if_cmd_setformat(l1h, l1h->config.trxd_pdu_ver_req, l1if_setformat_cb);
			l1h->config.setformat_acked = false;
		}
	}
	return 0;
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

static void update_ts_data(struct trx_l1h *l1h, struct trx_prov_ev_cfg_ts_data *data)
{
	l1h->config.setslot[data->tn].slottype = data->slottype;
	l1h->config.setslot[data->tn].tsc_set = data->tsc_set;
	l1h->config.setslot[data->tn].tsc_val = data->tsc_val;
	l1h->config.setslot[data->tn].tsc_valid = data->tsc_valid;

	l1h->config.setslot_valid[data->tn] = true;
	l1h->config.setslot_sent[data->tn] = false;
}

/* Whether a given TRX is fully configured and can be powered on */
static bool trx_is_provisioned(struct trx_l1h *l1h)
{
	struct phy_instance *pinst = l1h->phy_inst;
	if (l1h->config.enabled && l1h->config.rxtune_acked && l1h->config.txtune_acked &&
	    (l1h->config.bsic_acked || !pinst->phy_link->u.osmotrx.use_legacy_setbsic) &&
	    (l1h->config.tsc_acked || pinst->phy_link->u.osmotrx.use_legacy_setbsic) &&
	    (l1h->config.nomtxpower_acked || l1h->config.nominal_power_set_by_vty) &&
	    (l1h->config.setformat_acked)) {
		    return true;
	    }
	return false;
}


static void trx_signal_ready_trx0(struct trx_l1h *l1h)
{
	struct phy_instance *pinst = l1h->phy_inst;
	struct phy_instance *pinst_it;

	llist_for_each_entry(pinst_it, &pinst->phy_link->instances, list) {
		struct trx_l1h *l1h_it = pinst_it->u.osmotrx.hdl;
		if (l1h_it->phy_inst->num != 0)
			continue;
		osmo_fsm_inst_dispatch(l1h_it->provision_fi, TRX_PROV_EV_OTHER_TRX_READY, NULL);
		return;
	}
}

/* Called from TRX0 to check if other TRX are already configured and powered so POWERON can be sent */
static bool trx_other_trx0_ready(struct trx_l1h *l1h)
{
	struct phy_instance *pinst = l1h->phy_inst;
	struct phy_instance *pinst_it;

	/* Don't POWERON until all trx are ready */
	llist_for_each_entry(pinst_it, &pinst->phy_link->instances, list) {
		struct trx_l1h *l1h_it = pinst_it->u.osmotrx.hdl;
		if (l1h_it->phy_inst->num == 0)
			continue;
		if (l1h_it->provision_fi->state != TRX_PROV_ST_OPEN_POWERON)
			return false;
	}
	return true;
}

//////////////////////////
// FSM STATE ACTIONS
//////////////////////////

static void st_closed(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct trx_l1h *l1h = (struct trx_l1h *)fi->priv;

	switch (event) {
	case TRX_PROV_EV_OPEN:
		/* enable all slots */
		l1h->config.slotmask = 0xff;
		if (l1h->phy_inst->num == 0)
			trx_if_cmd_poweroff(l1h, NULL); /* TODO: jump to poweroff upon cb received */
		trx_prov_fsm_state_chg(fi, TRX_PROV_ST_OPEN_POWEROFF);
		break;
	default:
		OSMO_ASSERT(0);
	}
}

static void st_open_poweroff_on_enter(struct osmo_fsm_inst *fi, uint32_t prev_state)
{
	struct trx_l1h *l1h = (struct trx_l1h *)fi->priv;
	struct phy_instance *pinst = l1h->phy_inst;

	l1h->config.trxd_pdu_ver_req = pinst->phy_link->u.osmotrx.trxd_pdu_ver_max;

	/* Apply initial RFMUTE state */
	if (pinst->trx != NULL)
		trx_if_cmd_rfmute(l1h, pinst->trx->mo.nm_state.administrative != NM_STATE_UNLOCKED);
	else
		trx_if_cmd_rfmute(l1h, true);
}

static void st_open_poweroff(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct trx_l1h *l1h = (struct trx_l1h *)fi->priv;
	struct phy_instance *pinst = l1h->phy_inst;
	struct gsm_bts_trx *trx = pinst->trx;
	uint16_t arfcn;
	int nominal_power;
	int status;
	bool others_ready;

	switch (event) {
	case TRX_PROV_EV_CFG_ENABLE:
		l1h->config.enabled =(bool)data;
		break;
	case TRX_PROV_EV_CFG_BSIC:
		/* We always get BSIC from the BSC, TSC can be derived from the BCC */
		if (!pinst->phy_link->u.osmotrx.use_legacy_setbsic) {
			const uint8_t tsc = BSIC2BCC((uint8_t)(intptr_t)data);
			if (l1h->config.tsc != tsc || !l1h->config.tsc_valid) {
				l1h->config.tsc = tsc;
				l1h->config.tsc_valid = true;
				l1h->config.tsc_sent = false;
			}
		} else {
			const uint8_t bsic = (uint8_t)(intptr_t)data;
			if (l1h->config.bsic != bsic || !l1h->config.bsic_valid) {
				l1h->config.bsic = bsic;
				l1h->config.bsic_valid = true;
				l1h->config.bsic_sent = false;
			}
		}
		break;
	case TRX_PROV_EV_CFG_ARFCN:
		arfcn = (uint16_t)(intptr_t)data;
		if (l1h->config.arfcn != arfcn || !l1h->config.arfcn_valid) {
			l1h->config.arfcn = arfcn;
			l1h->config.arfcn_valid = true;
			l1h->config.txtune_sent = false;
			l1h->config.rxtune_sent = false;
			l1h->config.nomtxpower_sent = false;
		}
		break;
	case TRX_PROV_EV_CFG_TS:
		update_ts_data(l1h, (struct trx_prov_ev_cfg_ts_data*)data);
		break;

	/* CONFIRMATIONS FROM TRXC */
	case TRX_PROV_EV_RXTUNE_CNF:
		if (l1h->config.rxtune_sent)
			l1h->config.rxtune_acked = true;
		break;
	case TRX_PROV_EV_TXTUNE_CNF:
		if (l1h->config.txtune_sent)
			l1h->config.txtune_acked = true;
		break;
	case TRX_PROV_EV_NOMTXPOWER_CNF:
		nominal_power = (int)(intptr_t)data;
		if (l1h->config.nomtxpower_sent)
			l1h->config.nomtxpower_acked = true;
		l1if_trx_set_nominal_power(trx, nominal_power);
		break;
	case TRX_PROV_EV_SETBSIC_CNF:
		if (l1h->config.bsic_sent)
			l1h->config.bsic_acked = true;
		break;
	case TRX_PROV_EV_SETTSC_CNF:
		if (l1h->config.tsc_sent)
			l1h->config.tsc_acked = true;
		break;
	case TRX_PROV_EV_SETFORMAT_CNF:
		status = (int)(intptr_t)data;
		/* Transceiver may suggest a lower version (than requested) */
		if (status == l1h->config.trxd_pdu_ver_req) {
			l1h->config.trxd_pdu_ver_use = status;
			l1h->config.setformat_acked = true;
			LOGPPHI(l1h->phy_inst, DTRX, LOGL_INFO,
				"Using TRXD PDU version %u\n",
				l1h->config.trxd_pdu_ver_use);
		} else {
			LOGPPHI(l1h->phy_inst, DTRX, LOGL_DEBUG,
				"Transceiver suggests TRXD PDU version %u (requested %u)\n",
				status, l1h->config.trxd_pdu_ver_req);
			/* Send another SETFORMAT with suggested version */
			l1h->config.trxd_pdu_ver_req = status;
			l1h->config.setformat_sent = false;
		}
		break;
	case TRX_PROV_EV_OTHER_TRX_READY:
		OSMO_ASSERT(pinst->num == 0);
		/* Do nothing here, we were triggered to see if we can finally poweron TRX0 below */
		break;
	default:
		OSMO_ASSERT(0);
	}

	l1if_provision_transceiver_trx(l1h);

	if (l1h->phy_inst->num == 0)
		others_ready = trx_other_trx0_ready(l1h);
	else
		others_ready = true; /* we don't care about others in TRX!=0 */

	/* if we gathered all data and could go forward. For TRX0, only after
	 * all other TRX are prepared, since it will send POWERON commad */
	if (trx_is_provisioned(l1h) &&
	    (l1h->phy_inst->num != 0 || others_ready)) {
		if (l1h->phy_inst->num != 0) {
			trx_prov_fsm_state_chg(fi, TRX_PROV_ST_OPEN_POWERON);
			/* Once we are powered on, signal TRX0 in case it was waiting for us */
			trx_signal_ready_trx0(l1h);
		} else {
			trx_prov_fsm_state_chg(fi, TRX_PROV_ST_OPEN_WAIT_POWERON_CNF);
		}
	} else {
		LOGPFSML(fi, LOGL_INFO, "Delay poweron, wait for:%s%s%s%s%s%s%s%s\n",
			l1h->config.enabled ? "" :" enable",
			pinst->phy_link->u.osmotrx.use_legacy_setbsic ?
				(l1h->config.bsic_valid ? (l1h->config.bsic_acked ? "" : " bsic-ack") : " bsic") :
				(l1h->config.tsc_valid ? (l1h->config.tsc_acked ? "" : " tsc-ack") : " tsc"),
			l1h->config.arfcn_valid ? "" : " arfcn",
			l1h->config.rxtune_acked ? "" : " rxtune-ack",
			l1h->config.txtune_acked ? "" : " txtune-ack",
			l1h->config.nominal_power_set_by_vty ? "" : (l1h->config.nomtxpower_acked ? "" : " nomtxpower-ack"),
			l1h->config.setformat_acked ? "" : " setformat-ack",
			(l1h->phy_inst->num != 0 || others_ready) ? "" : " other-trx"
			);
	}
}


static void st_open_wait_power_cnf_on_enter(struct osmo_fsm_inst *fi, uint32_t prev_state)
{
	struct trx_l1h *l1h = (struct trx_l1h *)fi->priv;
	struct phy_instance *pinst = l1h->phy_inst;

	trx_if_cmd_poweron(l1h, l1if_poweronoff_cb);
	pinst->phy_link->u.osmotrx.poweronoff_sent = true;
}

static void st_open_wait_power_cnf(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct trx_l1h *l1h = (struct trx_l1h *)fi->priv;
	struct phy_instance *pinst = l1h->phy_inst;
	struct phy_link *plink = pinst->phy_link;
	int rc;

	switch (event) {
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
	default:
		OSMO_ASSERT(0);
	}
}

static void st_open_poweron_on_enter(struct osmo_fsm_inst *fi, uint32_t prev_state)
{
	struct trx_l1h *l1h = (struct trx_l1h *)fi->priv;
	uint8_t tn;

	/* after power on */
	if (l1h->config.rxgain_valid && !l1h->config.rxgain_sent) {
		trx_if_cmd_setrxgain(l1h, l1h->config.rxgain);
		l1h->config.rxgain_sent = true;
	}
	if (l1h->config.maxdly_valid && !l1h->config.maxdly_sent) {
		trx_if_cmd_setmaxdly(l1h, l1h->config.maxdly);
		l1h->config.maxdly_sent = true;
	}
	if (l1h->config.maxdlynb_valid && !l1h->config.maxdlynb_sent) {
		trx_if_cmd_setmaxdlynb(l1h, l1h->config.maxdlynb);
		l1h->config.maxdlynb_sent = true;
	}

	for (tn = 0; tn < TRX_NR_TS; tn++) {
		if (l1h->config.setslot_valid[tn]
		 && !l1h->config.setslot_sent[tn]) {
			trx_if_cmd_setslot(l1h, tn, l1if_setslot_cb);
			l1h->config.setslot_sent[tn] = true;
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

	switch (event) {
	case TRX_PROV_EV_CLOSE:
		/* power off transceiver, if not already */
		if (l1h->config.enabled) {
			if (pinst->num == 0 && plink->u.osmotrx.powered && !plink->u.osmotrx.poweronoff_sent) {
				trx_if_cmd_poweroff(l1h, l1if_poweronoff_cb);
				plink->u.osmotrx.poweronoff_sent = true;
			}
			l1h->config.rxgain_sent = false;
			l1h->config.maxdly_sent = false;
			l1h->config.maxdlynb_sent = false;
			for (tn = 0; tn < TRX_NR_TS; tn++)
				l1h->config.setslot_sent[tn] = false;
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
		update_ts_data(l1h, ts_data);
		/* While in this state we can send SETSLOT immediately */
		trx_if_cmd_setslot(l1h, ts_data->tn, l1if_setslot_cb);
		l1h->config.setslot_sent[ts_data->tn] = true;
		break;
	default:
		OSMO_ASSERT(0);
	}
}

static void st_open_wait_poweroff_cnf(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct trx_l1h *l1h = (struct trx_l1h *)fi->priv;
	struct phy_instance *pinst = l1h->phy_inst;
	struct phy_link *plink = pinst->phy_link;
	int rc;

	switch (event) {
	case TRX_PROV_EV_POWEROFF_CNF:
		rc = (uint16_t)(intptr_t)data;
		if (plink->state != PHY_LINK_SHUTDOWN) {
			trx_sched_clock_stopped(pinst->trx->bts);
			phy_link_state_set(plink, PHY_LINK_SHUTDOWN);

			/* Notify TRX close on all TRX associated with this phy */
			llist_for_each_entry(pinst, &plink->instances, list) {
				bts_model_trx_close_cb(pinst->trx, rc);
			}
			trx_prov_fsm_state_chg(fi, TRX_PROV_ST_OPEN_POWEROFF);
		}
		break;
	default:
		OSMO_ASSERT(0);
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
			X(TRX_PROV_EV_OTHER_TRX_READY) |
			X(TRX_PROV_EV_CFG_ENABLE) |
			X(TRX_PROV_EV_CFG_BSIC) |
			X(TRX_PROV_EV_CFG_ARFCN) |
			X(TRX_PROV_EV_CFG_TS) |
			X(TRX_PROV_EV_RXTUNE_CNF) |
			X(TRX_PROV_EV_TXTUNE_CNF) |
			X(TRX_PROV_EV_NOMTXPOWER_CNF) |
			X(TRX_PROV_EV_SETBSIC_CNF) |
			X(TRX_PROV_EV_SETTSC_CNF) |
			X(TRX_PROV_EV_SETFORMAT_CNF),
		.out_state_mask =
			X(TRX_PROV_ST_OPEN_WAIT_POWERON_CNF) |
			X(TRX_PROV_ST_OPEN_POWERON),
		.name = "OPEN_POWEROFF",
		.onenter = st_open_poweroff_on_enter,
		.action = st_open_poweroff,
	},
	[TRX_PROV_ST_OPEN_WAIT_POWERON_CNF] = {
		.in_event_mask =
			X(TRX_PROV_EV_POWERON_CNF) |
			X(TRX_PROV_EV_CFG_TS),
		.out_state_mask =
			X(TRX_PROV_ST_OPEN_POWERON),
		.name = "OPEN_WAIT_POWERON_CNF",
		.onenter = st_open_wait_power_cnf_on_enter,
		.action = st_open_wait_power_cnf,
	},
	[TRX_PROV_ST_OPEN_POWERON] = {
		.in_event_mask =
			X(TRX_PROV_EV_CLOSE) |
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
	OSMO_VALUE_STRING(TRX_PROV_EV_OTHER_TRX_READY),
	OSMO_VALUE_STRING(TRX_PROV_EV_OPEN),
	OSMO_VALUE_STRING(TRX_PROV_EV_CFG_ENABLE),
	OSMO_VALUE_STRING(TRX_PROV_EV_CFG_BSIC),
	OSMO_VALUE_STRING(TRX_PROV_EV_CFG_ARFCN),
	OSMO_VALUE_STRING(TRX_PROV_EV_CFG_TS),
	OSMO_VALUE_STRING(TRX_PROV_EV_CFG_RXGAIN),
	OSMO_VALUE_STRING(TRX_PROV_EV_CFG_SETMAXDLY),
	OSMO_VALUE_STRING(TRX_PROV_EV_RXTUNE_CNF),
	OSMO_VALUE_STRING(TRX_PROV_EV_TXTUNE_CNF),
	OSMO_VALUE_STRING(TRX_PROV_EV_NOMTXPOWER_CNF),
	OSMO_VALUE_STRING(TRX_PROV_EV_SETBSIC_CNF),
	OSMO_VALUE_STRING(TRX_PROV_EV_SETTSC_CNF),
	OSMO_VALUE_STRING(TRX_PROV_EV_SETFORMAT_CNF),
	OSMO_VALUE_STRING(TRX_PROV_EV_POWERON_CNF),
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
