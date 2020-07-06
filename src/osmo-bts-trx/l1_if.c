/*
 * layer 1 primitive handling and interface
 *
 * Copyright (C) 2013  Andreas Eversberg <jolly@eversberg.eu>
 * Copyright (C) 2015  Alexander Chemeris <Alexander.Chemeris@fairwaves.co>
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
 */

#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>
#include <errno.h>
#include <inttypes.h>

#include <osmocom/core/talloc.h>
#include <osmocom/core/bits.h>
#include <osmocom/core/fsm.h>
#include <osmocom/codec/ecu.h>
#include <osmocom/gsm/abis_nm.h>

#include <osmo-bts/logging.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/oml.h>
#include <osmo-bts/rsl.h>
#include <osmo-bts/l1sap.h>
#include <osmo-bts/bts_model.h>
#include <osmo-bts/amr.h>
#include <osmo-bts/abis.h>
#include <osmo-bts/scheduler.h>
#include <osmo-bts/pcu_if.h>

#include "l1_if.h"
#include "trx_if.h"
#include "trx_provision_fsm.h"

#define RF_DISABLED_mdB to_mdB(-10)

static const uint8_t transceiver_chan_types[_GSM_PCHAN_MAX] = {
	[GSM_PCHAN_NONE]                = 8,
	[GSM_PCHAN_CCCH]                = 4,
	[GSM_PCHAN_CCCH_SDCCH4]         = 5,
	[GSM_PCHAN_CCCH_SDCCH4_CBCH]    = 5,
	[GSM_PCHAN_TCH_F]               = 1,
	[GSM_PCHAN_TCH_H]               = 3,
	[GSM_PCHAN_SDCCH8_SACCH8C]      = 7,
	[GSM_PCHAN_SDCCH8_SACCH8C_CBCH] = 7,
	[GSM_PCHAN_PDCH]                = 13,
	/* [GSM_PCHAN_TCH_F_PDCH] not needed here, see trx_set_ts_as_pchan() */
	[GSM_PCHAN_UNKNOWN]             = 0,
};

enum gsm_phys_chan_config transceiver_chan_type_2_pchan(uint8_t type)
{
	int i;
	for (i = 0; i < _GSM_PCHAN_MAX; i++) {
		if (transceiver_chan_types[i] == type)
			return (enum gsm_phys_chan_config) i;
	}
	return GSM_PCHAN_UNKNOWN;
}

struct trx_l1h *trx_l1h_alloc(void *tall_ctx, struct phy_instance *pinst)
{
	struct trx_l1h *l1h;
	l1h = talloc_zero(tall_ctx, struct trx_l1h);
	l1h->phy_inst = pinst;
	l1h->provision_fi = osmo_fsm_inst_alloc(&trx_prov_fsm, l1h, l1h, LOGL_INFO, NULL);
	OSMO_ASSERT(osmo_fsm_inst_update_id_f_sanitize(l1h->provision_fi, '-', phy_instance_name(pinst)) == 0);
	trx_if_init(l1h);
	return l1h;
}

static void check_transceiver_availability_trx(struct trx_l1h *l1h, int avail)
{
	struct phy_instance *pinst = l1h->phy_inst;
	struct gsm_bts_trx *trx = pinst->trx;
	uint8_t tn;

	/* HACK, we should change state when we receive first clock from
	 * transceiver */
	if (avail) {
		/* signal availability */
		oml_mo_state_chg(&trx->mo, NM_OPSTATE_DISABLED, NM_AVSTATE_OK);
		oml_mo_state_chg(&trx->bb_transc.mo, -1, NM_AVSTATE_OK);
		if (!pinst->u.osmotrx.sw_act_reported) {
			oml_mo_tx_sw_act_rep(&trx->mo);
			oml_mo_tx_sw_act_rep(&trx->bb_transc.mo);
			pinst->u.osmotrx.sw_act_reported = true;
		}

		for (tn = 0; tn < TRX_NR_TS; tn++)
			oml_mo_state_chg(&trx->ts[tn].mo, NM_OPSTATE_DISABLED,
				(l1h->config.slotmask & (1 << tn)) ?
					NM_AVSTATE_DEPENDENCY :
					NM_AVSTATE_NOT_INSTALLED);
	} else {
		oml_mo_state_chg(&trx->mo, NM_OPSTATE_DISABLED,
			NM_AVSTATE_OFF_LINE);
		oml_mo_state_chg(&trx->bb_transc.mo, NM_OPSTATE_DISABLED,
			NM_AVSTATE_OFF_LINE);

		for (tn = 0; tn < TRX_NR_TS; tn++)
			oml_mo_state_chg(&trx->ts[tn].mo, NM_OPSTATE_DISABLED,
				NM_AVSTATE_OFF_LINE);
	}
}

int bts_model_lchan_deactivate(struct gsm_lchan *lchan)
{
	struct phy_instance *pinst = trx_phy_instance(lchan->ts->trx);
	struct trx_l1h *l1h = pinst->u.osmotrx.hdl;

	if (lchan->rel_act_kind == LCHAN_REL_ACT_REACT) {
		lchan->rel_act_kind = LCHAN_REL_ACT_RSL;
		/* FIXME: perform whatever is needed (if any) to set proper PCH/AGCH allocation according to
		   3GPP TS 44.018 Table 10.5.2.11.1 using num_agch(lchan->ts->trx, "TRX L1"); function */
		return 0;
	}
	/* set lchan inactive */
	lchan_set_state(lchan, LCHAN_S_NONE);

	return trx_sched_set_lchan(&l1h->l1s, gsm_lchan2chan_nr(lchan), LID_DEDIC, false);
}

int bts_model_lchan_deactivate_sacch(struct gsm_lchan *lchan)
{
	struct phy_instance *pinst = trx_phy_instance(lchan->ts->trx);
	struct trx_l1h *l1h = pinst->u.osmotrx.hdl;
	return trx_sched_set_lchan(&l1h->l1s, gsm_lchan2chan_nr(lchan), LID_SACCH, false);
}

int l1if_trx_start_power_ramp(struct gsm_bts_trx *trx, ramp_compl_cb_t ramp_compl_cb)
{
	struct phy_instance *pinst = trx_phy_instance(trx);
	struct trx_l1h *l1h = pinst->u.osmotrx.hdl;

	if (l1h->config.forced_max_power_red == -1)
		return power_ramp_start(trx, get_p_nominal_mdBm(trx), 0, ramp_compl_cb);
	else
		return power_ramp_start(trx, get_p_max_out_mdBm(trx) - to_mdB(l1h->config.forced_max_power_red), 1, ramp_compl_cb);
}

/* Sets the nominal power, in dB */
void l1if_trx_set_nominal_power(struct gsm_bts_trx *trx, int nominal_power)
{
	struct phy_instance *pinst = trx_phy_instance(trx);
	bool nom_pwr_changed = trx->nominal_power != nominal_power;

	trx->nominal_power = nominal_power;
	trx->power_params.trx_p_max_out_mdBm = to_mdB(nominal_power);
	/* If we receive ultra-low  nominal Tx power (<0dBm), make sure to update where we are */
	trx->power_params.p_total_cur_mdBm = OSMO_MIN(trx->power_params.p_total_cur_mdBm,
						      trx->power_params.trx_p_max_out_mdBm);

	/* If TRX is not yet powered, delay ramping until it's ON */
	if (!nom_pwr_changed || !pinst->phy_link->u.osmotrx.powered ||
	    trx->mo.nm_state.administrative == NM_STATE_UNLOCKED)
		return;

	/* We are already ON and we got new information about nominal power, so
	 * let's make sure we adapt the tx power to it
	 */
	l1if_trx_start_power_ramp(trx, NULL);
}

static void l1if_setpower_att_cb(struct trx_l1h *l1h, int power_att_db, int rc)
{
	struct phy_instance *pinst = l1h->phy_inst;
	struct gsm_bts_trx *trx = pinst->trx;

	LOGPPHI(pinst, DL1C, LOGL_DEBUG, "l1if_setpower_att_cb(power_att_db=%d, rc=%d)\n", power_att_db, rc);

	power_trx_change_compl(trx, get_p_max_out_mdBm(trx) - to_mdB(power_att_db));
}

/*
 * activation/configuration/deactivation of transceiver's TRX
 */

/* initialize the layer1 */
static int trx_init(struct gsm_bts_trx *trx)
{
	struct phy_instance *pinst = trx_phy_instance(trx);
	struct trx_l1h *l1h = pinst->u.osmotrx.hdl;

	osmo_fsm_inst_dispatch(l1h->provision_fi, TRX_PROV_EV_CFG_ENABLE, (void*)(intptr_t)true);

	if (trx == trx->bts->c0)
		lchan_init_lapdm(&trx->ts[0].lchan[CCCH_LCHAN]);

	/* Send OPSTART ack */
	return oml_mo_opstart_ack(&trx->mo);
}

/* Deact RF on transceiver */
int bts_model_trx_deact_rf(struct gsm_bts_trx *trx)
{
	struct phy_instance *pinst = trx_phy_instance(trx);
	struct trx_l1h *l1h = pinst->u.osmotrx.hdl;
	enum gsm_phys_chan_config pchan = trx->ts[0].pchan;

	/* close all logical channels and reset timeslots */
	trx_sched_reset(&l1h->l1s);

	/* deactivate lchan for CCCH */
	if (pchan == GSM_PCHAN_CCCH || pchan == GSM_PCHAN_CCCH_SDCCH4 ||
	    pchan == GSM_PCHAN_CCCH_SDCCH4_CBCH) {
		lchan_set_state(&trx->ts[0].lchan[CCCH_LCHAN], LCHAN_S_INACTIVE);
	}

	return 0;
}

/* deactivate transceiver */
void bts_model_trx_close(struct gsm_bts_trx *trx)
{
	struct phy_instance *pinst = trx_phy_instance(trx);
	struct trx_l1h *l1h = pinst->u.osmotrx.hdl;

	osmo_fsm_inst_dispatch(l1h->provision_fi, TRX_PROV_EV_POWEROFF, NULL);

	/* Set to Operational State: Disabled */
	check_transceiver_availability_trx(l1h, 0);
}

/* on RSL failure, deactivate transceiver */
void bts_model_abis_close(struct gsm_bts *bts)
{
	bts_shutdown(bts, "Abis close");
}

int bts_model_adjst_ms_pwr(struct gsm_lchan *lchan)
{
	/* we always implement the power control loop in osmo-bts software, as
	 * there is no automatism in the underlying osmo-trx */
	return 0;
}

/* set bts attributes */
static uint8_t trx_set_bts(struct gsm_bts *bts, struct tlv_parsed *new_attr)
{
	struct gsm_bts_trx *trx;
	uint8_t bsic = bts->bsic;

	llist_for_each_entry(trx, &bts->trx_list, list) {
		struct phy_instance *pinst = trx_phy_instance(trx);
		struct phy_link *plink = pinst->phy_link;
		struct trx_l1h *l1h = pinst->u.osmotrx.hdl;
		osmo_fsm_inst_dispatch(l1h->provision_fi, TRX_PROV_EV_CFG_BSIC, (void*)(intptr_t)bsic);
		check_transceiver_availability_trx(l1h, phy_link_state_get(plink) != PHY_LINK_SHUTDOWN);
	}

	return 0;
}

/* set trx attributes */
static uint8_t trx_set_trx(struct gsm_bts_trx *trx)
{
	struct phy_instance *pinst = trx_phy_instance(trx);
	struct trx_l1h *l1h = pinst->u.osmotrx.hdl;
	struct phy_link *plink = pinst->phy_link;
	uint16_t arfcn = trx->arfcn;

	osmo_fsm_inst_dispatch(l1h->provision_fi, TRX_PROV_EV_CFG_ARFCN, (void*)(intptr_t)arfcn);

	/* Begin to ramp up the power if power reduction is set by OML and TRX
	   is already running. Otherwise skip, power ramping will be started
	   after TRX is running */
	if (plink->u.osmotrx.powered && l1h->config.forced_max_power_red == -1 &&
	    trx->mo.nm_state.administrative == NM_STATE_UNLOCKED)
		power_ramp_start(pinst->trx, get_p_nominal_mdBm(pinst->trx), 0, NULL);

	return 0;
}

/* set ts attributes */
static uint8_t trx_set_ts_as_pchan(struct gsm_bts_trx_ts *ts,
				   enum gsm_phys_chan_config pchan)
{
	struct phy_instance *pinst = trx_phy_instance(ts->trx);
	struct trx_l1h *l1h = pinst->u.osmotrx.hdl;
	uint8_t tn = ts->nr;
	uint16_t tsc = ts->tsc;
	uint8_t slottype;
	int rc;

	/* all TSC of all timeslots must be equal, because transceiver only
	 * supports one TSC per TRX */

	osmo_fsm_inst_dispatch(l1h->provision_fi, TRX_PROV_EV_CFG_TSC, (void*)(intptr_t)tsc);

	/* ignore disabled slots */
	if (!(l1h->config.slotmask & (1 << tn)))
		return NM_NACK_RES_NOTAVAIL;

	/* set physical channel. For dynamic timeslots, the caller should have
	 * decided on a more specific PCHAN type already. */
	OSMO_ASSERT(pchan != GSM_PCHAN_TCH_F_PDCH);
	OSMO_ASSERT(pchan != GSM_PCHAN_TCH_F_TCH_H_PDCH);
	rc = trx_sched_set_pchan(&l1h->l1s, tn, pchan);
	if (rc)
		return NM_NACK_RES_NOTAVAIL;

	/* activate lchan for CCCH */
	if (pchan == GSM_PCHAN_CCCH || pchan == GSM_PCHAN_CCCH_SDCCH4 ||
	    pchan == GSM_PCHAN_CCCH_SDCCH4_CBCH) {
		ts->lchan[CCCH_LCHAN].rel_act_kind = LCHAN_REL_ACT_OML;
		lchan_set_state(&ts->lchan[CCCH_LCHAN], LCHAN_S_ACTIVE);
	}

	slottype = transceiver_chan_types[pchan];


	struct trx_prov_ev_cfg_ts_data data = { .tn = tn, .slottype = slottype };
	osmo_fsm_inst_dispatch(l1h->provision_fi, TRX_PROV_EV_CFG_TS, &data);

	return 0;
}

static uint8_t trx_set_ts(struct gsm_bts_trx_ts *ts)
{
	enum gsm_phys_chan_config pchan;

	/* For dynamic timeslots, pick the pchan type that should currently be
	 * active. This should only be called during init, PDCH transitions
	 * will call trx_set_ts_as_pchan() directly. */
	switch (ts->pchan) {
	case GSM_PCHAN_TCH_F_PDCH:
		OSMO_ASSERT((ts->flags & TS_F_PDCH_PENDING_MASK) == 0);
		pchan = (ts->flags & TS_F_PDCH_ACTIVE)? GSM_PCHAN_PDCH
			                              : GSM_PCHAN_TCH_F;
		break;
	case GSM_PCHAN_TCH_F_TCH_H_PDCH:
		OSMO_ASSERT(ts->dyn.pchan_is == ts->dyn.pchan_want);
		pchan = ts->dyn.pchan_is;
		break;
	default:
		pchan = ts->pchan;
		break;
	}

	return trx_set_ts_as_pchan(ts, pchan);
}


/*
 * primitive handling
 */

/* enable ciphering */
static int l1if_set_ciphering(struct trx_l1h *l1h, struct gsm_lchan *lchan,
	uint8_t chan_nr, int downlink)
{
	/* ciphering already enabled in both directions */
	if (lchan->ciph_state == LCHAN_CIPH_RXTX_CONF)
		return -EINVAL;

	if (!downlink) {
		/* set uplink */
		trx_sched_set_cipher(&l1h->l1s, chan_nr, 0, lchan->encr.alg_id - 1,
			lchan->encr.key, lchan->encr.key_len);
		lchan->ciph_state = LCHAN_CIPH_RX_CONF;
	} else {
		/* set downlink and also set uplink, if not already */
		if (lchan->ciph_state != LCHAN_CIPH_RX_CONF) {
			trx_sched_set_cipher(&l1h->l1s, chan_nr, 0,
				lchan->encr.alg_id - 1, lchan->encr.key,
				lchan->encr.key_len);
		}
		trx_sched_set_cipher(&l1h->l1s, chan_nr, 1, lchan->encr.alg_id - 1,
			lchan->encr.key, lchan->encr.key_len);
		lchan->ciph_state = LCHAN_CIPH_RXTX_CONF;
	}

	return 0;
}

static int mph_info_chan_confirm(struct trx_l1h *l1h, uint8_t chan_nr,
	enum osmo_mph_info_type type, uint8_t cause)
{
	struct phy_instance *pinst = l1h->phy_inst;
	struct osmo_phsap_prim l1sap;

	memset(&l1sap, 0, sizeof(l1sap));
	osmo_prim_init(&l1sap.oph, SAP_GSM_PH, PRIM_MPH_INFO, PRIM_OP_CONFIRM,
		NULL);
	l1sap.u.info.type = type;
	l1sap.u.info.u.act_cnf.chan_nr = chan_nr;
	l1sap.u.info.u.act_cnf.cause = cause;

	return l1sap_up(pinst->trx, &l1sap);
}

int l1if_mph_time_ind(struct gsm_bts *bts, uint32_t fn)
{
	struct osmo_phsap_prim l1sap;

	memset(&l1sap, 0, sizeof(l1sap));
	osmo_prim_init(&l1sap.oph, SAP_GSM_PH, PRIM_MPH_INFO,
		PRIM_OP_INDICATION, NULL);
	l1sap.u.info.type = PRIM_INFO_TIME;
	l1sap.u.info.u.time_ind.fn = fn;

	if (!bts->c0)
		return -EINVAL;

	return l1sap_up(bts->c0, &l1sap);
}

/* primitive from common part */
int bts_model_l1sap_down(struct gsm_bts_trx *trx, struct osmo_phsap_prim *l1sap)
{
	struct phy_instance *pinst = trx_phy_instance(trx);
	struct trx_l1h *l1h = pinst->u.osmotrx.hdl;
	struct msgb *msg = l1sap->oph.msg;
	uint8_t chan_nr;
	int rc = 0;
	struct gsm_lchan *lchan;

	switch (OSMO_PRIM_HDR(&l1sap->oph)) {
	case OSMO_PRIM(PRIM_PH_DATA, PRIM_OP_REQUEST):
		if (!msg)
			break;
		/* put data into scheduler's queue */
		return trx_sched_ph_data_req(&l1h->l1s, l1sap);
	case OSMO_PRIM(PRIM_TCH, PRIM_OP_REQUEST):
		if (!msg)
			break;
		/* put data into scheduler's queue */
		return trx_sched_tch_req(&l1h->l1s, l1sap);
	case OSMO_PRIM(PRIM_MPH_INFO, PRIM_OP_REQUEST):
		switch (l1sap->u.info.type) {
		case PRIM_INFO_ACT_CIPH:
			chan_nr = l1sap->u.info.u.ciph_req.chan_nr;
			lchan = get_lchan_by_chan_nr(trx, chan_nr);
			if (l1sap->u.info.u.ciph_req.uplink)
				l1if_set_ciphering(l1h, lchan, chan_nr, 0);
			if (l1sap->u.info.u.ciph_req.downlink)
				l1if_set_ciphering(l1h, lchan, chan_nr, 1);
			break;
		case PRIM_INFO_ACTIVATE:
		case PRIM_INFO_DEACTIVATE:
		case PRIM_INFO_MODIFY:
			chan_nr = l1sap->u.info.u.act_req.chan_nr;
			lchan = get_lchan_by_chan_nr(trx, chan_nr);
			if (l1sap->u.info.type == PRIM_INFO_ACTIVATE) {
				if ((chan_nr & 0xE0) == 0x80) {
					LOGP(DL1C, LOGL_ERROR, "Cannot activate"
						" chan_nr 0x%02x\n", chan_nr);
					break;
				}

				/* attempt to allocate an Error Concealment Unit instance, if available */
				lchan->ecu_state = osmo_ecu_init(trx, lchan2ecu_codec(lchan));

				/* trx_chan_desc[] in scheduler.c uses the RSL_CHAN_OSMO_PDCH cbits
				 * (0xc0) to indicate the need for PDTCH and PTCCH SAPI activation.
				 * However, 0xc0 is a cbits pattern exclusively used for Osmocom style
				 * dyn TS (a non-standard RSL Chan Activ mod); hence, for IPA style dyn
				 * TS, the chan_nr will never reflect 0xc0 and we would omit the
				 * PDTCH,PTTCH SAPIs. To properly de-/activate the PDTCH SAPIs in
				 * scheduler.c, make sure the 0xc0 cbits are set for de-/activating PDTCH
				 * lchans, i.e. both Osmocom and IPA style dyn TS. (For Osmocom style dyn
				 * TS, the chan_nr typically already reflects 0xc0, while it doesn't for
				 * IPA style.) */
				if (lchan->type == GSM_LCHAN_PDTCH)
					chan_nr = RSL_CHAN_OSMO_PDCH | (chan_nr & ~RSL_CHAN_NR_MASK);

				/* activate dedicated channel */
				trx_sched_set_lchan(&l1h->l1s, chan_nr, LID_DEDIC, true);
				/* activate associated channel */
				trx_sched_set_lchan(&l1h->l1s, chan_nr, LID_SACCH, true);
				/* set mode */
				trx_sched_set_mode(&l1h->l1s, chan_nr,
					lchan->rsl_cmode, lchan->tch_mode,
					lchan->tch.amr_mr.num_modes,
					lchan->tch.amr_mr.bts_mode[0].mode,
					lchan->tch.amr_mr.bts_mode[1].mode,
					lchan->tch.amr_mr.bts_mode[2].mode,
					lchan->tch.amr_mr.bts_mode[3].mode,
					amr_get_initial_mode(lchan),
					(lchan->ho.active == 1));
				/* init lapdm */
				lchan_init_lapdm(lchan);
				/* set lchan active */
				lchan_set_state(lchan, LCHAN_S_ACTIVE);
				/* set initial ciphering */
				l1if_set_ciphering(l1h, lchan, chan_nr, 0);
				l1if_set_ciphering(l1h, lchan, chan_nr, 1);
				if (lchan->encr.alg_id)
					lchan->ciph_state = LCHAN_CIPH_RXTX_CONF;
				else
					lchan->ciph_state = LCHAN_CIPH_NONE;

				/* confirm */
				mph_info_chan_confirm(l1h, chan_nr,
					PRIM_INFO_ACTIVATE, 0);
				break;
			}
			if (l1sap->u.info.type == PRIM_INFO_MODIFY) {
				/* ECU for possibly new codec */
				if (lchan->ecu_state)
					osmo_ecu_destroy(lchan->ecu_state);
				lchan->ecu_state = osmo_ecu_init(trx, lchan2ecu_codec(lchan));
				/* change mode */
				trx_sched_set_mode(&l1h->l1s, chan_nr,
					lchan->rsl_cmode, lchan->tch_mode,
					lchan->tch.amr_mr.num_modes,
					lchan->tch.amr_mr.bts_mode[0].mode,
					lchan->tch.amr_mr.bts_mode[1].mode,
					lchan->tch.amr_mr.bts_mode[2].mode,
					lchan->tch.amr_mr.bts_mode[3].mode,
					amr_get_initial_mode(lchan),
					0);
				break;
			}
			/* here, type == PRIM_INFO_DEACTIVATE */
			if ((chan_nr & 0xE0) == 0x80) {
				LOGP(DL1C, LOGL_ERROR, "Cannot deactivate "
					"chan_nr 0x%02x\n", chan_nr);
				break;
			}
			/* clear ECU state (if any) */
			if (lchan->ecu_state) {
				osmo_ecu_destroy(lchan->ecu_state);
				lchan->ecu_state = NULL;
			}
			/* deactivate associated channel */
			bts_model_lchan_deactivate_sacch(lchan);
			if (!l1sap->u.info.u.act_req.sacch_only) {
				/* deactivate dedicated channel */
				lchan_deactivate(lchan);
				/* confirm only on dedicated channel */
				mph_info_chan_confirm(l1h, chan_nr,
					PRIM_INFO_DEACTIVATE, 0);
			}
			break;
		default:
			LOGP(DL1C, LOGL_NOTICE, "unknown MPH-INFO.req %d\n",
				l1sap->u.info.type);
			rc = -EINVAL;
			goto done;
		}
		break;
	default:
		LOGP(DL1C, LOGL_NOTICE, "unknown prim %d op %d\n",
			l1sap->oph.primitive, l1sap->oph.operation);
		rc = -EINVAL;
		goto done;
	}

done:
	if (msg)
		msgb_free(msg);
	return rc;
}


/*
 * oml handling
 */

/* callback from OML */
int bts_model_check_oml(struct gsm_bts *bts, uint8_t msg_type,
			struct tlv_parsed *old_attr, struct tlv_parsed *new_attr,
			void *obj)
{
	/* FIXME: check if the attributes are valid */
	return 0;
}

/* callback from OML */
int bts_model_apply_oml(struct gsm_bts *bts, struct msgb *msg,
			struct tlv_parsed *new_attr, int kind, void *obj)
{
	struct abis_om_fom_hdr *foh = msgb_l3(msg);
	int cause = 0;

	switch (foh->msg_type) {
	case NM_MT_SET_BTS_ATTR:
		cause = trx_set_bts(obj, new_attr);
		break;
	case NM_MT_SET_RADIO_ATTR:
		cause = trx_set_trx(obj);
		break;
	case NM_MT_SET_CHAN_ATTR:
		cause = trx_set_ts(obj);
		break;
	}

	return oml_fom_ack_nack(msg, cause);
}

/* callback from OML */
int bts_model_opstart(struct gsm_bts *bts, struct gsm_abis_mo *mo,
		      void *obj)
{
	int rc;
	LOGP(DOML, LOGL_DEBUG, "bts_model_opstart: %s received\n",
	     get_value_string(abis_nm_obj_class_names, mo->obj_class));
	switch (mo->obj_class) {
	case NM_OC_RADIO_CARRIER:
		/* activate transceiver */
		rc = trx_init(obj);
		break;
	case NM_OC_CHANNEL:
	case NM_OC_BTS:
	case NM_OC_SITE_MANAGER:
	case NM_OC_BASEB_TRANSC:
	case NM_OC_GPRS_NSE:
	case NM_OC_GPRS_CELL:
	case NM_OC_GPRS_NSVC:
		oml_mo_state_chg(mo, NM_OPSTATE_ENABLED, NM_AVSTATE_OK);
		rc = oml_mo_opstart_ack(mo);
		break;
	default:
		rc = oml_mo_opstart_nack(mo, NM_NACK_OBJCLASS_NOTSUPP);
	}
	return rc;
}

static void bts_model_chg_adm_state_ramp_compl_cb(struct gsm_bts_trx *trx)
{
	LOGPTRX(trx, DL1C, LOGL_INFO, "power ramp due to ADM STATE change finished\n");
	trx->mo.procedure_pending = 0;
	if (trx->mo.nm_state.administrative == NM_STATE_LOCKED) {
		bts_model_trx_deact_rf(trx);
		pcu_tx_info_ind();
	}
}

int bts_model_chg_adm_state(struct gsm_bts *bts, struct gsm_abis_mo *mo,
			    void *obj, uint8_t adm_state)
{
	struct gsm_bts_trx *trx;
	struct phy_instance *pinst;
	int i, rc = 0;

	switch (mo->obj_class) {
	case NM_OC_RADIO_CARRIER:
		trx = (struct gsm_bts_trx *) obj;
		pinst = trx_phy_instance(trx);

		/* Begin to ramp the power if TRX is already running. Otherwise
		 * skip, power ramping will be started after TRX is running */
		if (!pinst->phy_link->u.osmotrx.powered)
			break;

		if (mo->procedure_pending) {
			LOGPTRX(trx, DL1C, LOGL_ERROR, "Discarding adm change command: "
				"pending procedure on RC %d\n",
				((struct gsm_bts_trx *)obj)->nr);
			rc = -1;
			break;
		}
		switch (adm_state) {
		case NM_STATE_LOCKED:
			mo->procedure_pending = 1;
			rc = power_ramp_start(trx, RF_DISABLED_mdB, 1, bts_model_chg_adm_state_ramp_compl_cb);
			break;
		case NM_STATE_UNLOCKED:
			mo->procedure_pending = 1;
			/* Activate timeslots in scheduler and start power ramp up */
			for (i = 0; i < ARRAY_SIZE(trx->ts); i++) {
				struct gsm_bts_trx_ts *ts = &trx->ts[i];
				trx_set_ts(ts);
			}
			rc = l1if_trx_start_power_ramp(trx, bts_model_chg_adm_state_ramp_compl_cb);
			if (rc == 0) {
				mo->nm_state.administrative = adm_state;
				pcu_tx_info_ind();
				return oml_mo_statechg_ack(mo);
			}
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}

	if (rc == 0) {
		mo->nm_state.administrative = adm_state;
		return oml_mo_statechg_ack(mo);
	} else
		return oml_mo_statechg_nack(mo, NM_NACK_REQ_NOT_GRANT);
}

int bts_model_oml_estab(struct gsm_bts *bts)
{
	return 0;
}

int bts_model_change_power(struct gsm_bts_trx *trx, int p_trxout_mdBm)
{
	struct phy_instance *pinst = trx_phy_instance(trx);
	struct trx_l1h *l1h = pinst->u.osmotrx.hdl;
	int power_att = (get_p_max_out_mdBm(trx) - p_trxout_mdBm) / 1000;
	return trx_if_cmd_setpower_att(l1h, power_att, l1if_setpower_att_cb);
}

int bts_model_ts_disconnect(struct gsm_bts_trx_ts *ts)
{
	/* no action required, signal completion right away. */
	cb_ts_disconnected(ts);
	return 0;
}

void bts_model_ts_connect(struct gsm_bts_trx_ts *ts,
			 enum gsm_phys_chan_config as_pchan)
{
	int rc;
	LOGP(DL1C, LOGL_DEBUG, "%s bts_model_ts_connect(as_pchan=%s)\n",
	     gsm_ts_name(ts), gsm_pchan_name(as_pchan));

	rc = trx_set_ts_as_pchan(ts, as_pchan);
	if (rc)
		cb_ts_connected(ts, rc);

	/* cb_ts_connected will be called in l1if_setslot_cb once we receive RSP SETSLOT */
}
