/*
 * layer 1 primitive handling and interface
 *
 * Copyright (C) 2013  Andreas Eversberg <jolly@eversberg.eu>
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
#include <errno.h>

#include <osmocom/core/talloc.h>
#include <osmocom/core/bits.h>

#include <osmo-bts/logging.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/oml.h>
#include <osmo-bts/rsl.h>
#include <osmo-bts/l1sap.h>
#include <osmo-bts/bts_model.h>

#include "l1_if.h"
#include "trx_if.h"
#include "scheduler.h"


static const uint8_t tranceiver_chan_types[_GSM_PCHAN_MAX] = {
	[GSM_PCHAN_NONE]                = 8,
	[GSM_PCHAN_CCCH]                = 6,
	[GSM_PCHAN_CCCH_SDCCH4]         = 5,
	[GSM_PCHAN_TCH_F]               = 1,
	[GSM_PCHAN_TCH_H]               = 2,
	[GSM_PCHAN_SDCCH8_SACCH8C]      = 7,
	[GSM_PCHAN_PDCH]                = 13,
	//[GSM_PCHAN_TCH_F_PDCH]                = FIXME,
	[GSM_PCHAN_UNKNOWN]             = 0,
};


/*
 * create destroy trx l1 instance
 */

struct trx_l1h *l1if_open(struct gsm_bts_trx *trx)
{
	struct trx_l1h *l1h;
	int rc;

	l1h = talloc_zero(tall_bts_ctx, struct trx_l1h);
	if (!l1h)
		return NULL;
	l1h->trx = trx;
	trx->role_bts.l1h = l1h;

	trx_sched_init(l1h);

	rc = trx_if_open(l1h);
	if (rc < 0) {
		LOGP(DL1C, LOGL_FATAL, "Cannot initialize scheduler\n");
		goto err;
	}

	return l1h;

err:
	l1if_close(l1h);
	return NULL;
}

void l1if_close(struct trx_l1h *l1h)
{
	trx_if_close(l1h);
	trx_sched_exit(l1h);
	talloc_free(l1h);
}

void l1if_reset(struct trx_l1h *l1h)
{
}

static void check_tranceiver_availability_trx(struct trx_l1h *l1h, int avail)
{
	struct gsm_bts_trx *trx = l1h->trx;
	uint8_t tn;

	/* HACK, we should change state when we receive first clock from
	 * tranceiver */
	if (avail) {
		/* signal availability */
		oml_mo_state_chg(&trx->mo, NM_OPSTATE_DISABLED, NM_AVSTATE_OK);
		oml_mo_tx_sw_act_rep(&trx->mo);
		oml_mo_state_chg(&trx->bb_transc.mo, -1, NM_AVSTATE_OK);
		oml_mo_tx_sw_act_rep(&trx->bb_transc.mo);

		for (tn = 0; tn < 8; tn++)
			oml_mo_state_chg(&trx->ts[tn].mo, NM_OPSTATE_DISABLED,
				(l1h->config.slotmask & (1 << tn)) ?
					NM_AVSTATE_DEPENDENCY :
					NM_AVSTATE_NOT_INSTALLED);
	} else {
		oml_mo_state_chg(&trx->mo, NM_OPSTATE_DISABLED,
			NM_AVSTATE_OFF_LINE);
		oml_mo_state_chg(&trx->bb_transc.mo, NM_OPSTATE_DISABLED,
			NM_AVSTATE_OFF_LINE);

		for (tn = 0; tn < 8; tn++)
			oml_mo_state_chg(&trx->ts[tn].mo, NM_OPSTATE_DISABLED,
				NM_AVSTATE_OFF_LINE);
	}
}

int check_tranceiver_availability(struct gsm_bts *bts, int avail)
{
	struct gsm_bts_trx *trx;
	struct trx_l1h *l1h;

	llist_for_each_entry(trx, &bts->trx_list, list) {
		l1h = trx_l1h_hdl(trx);
		check_tranceiver_availability_trx(l1h, avail);
	}
	return 0;
}


/*
 * tranceiver provisioning
 */
int l1if_provision_tranceiver_trx(struct trx_l1h *l1h)
{
	uint8_t tn;

	if (!tranceiver_available)
		return -EIO;

	if (l1h->config.poweron
	 && l1h->config.tsc_valid
	 && l1h->config.bsic_valid
	 && l1h->config.arfcn_valid) {
	 	/* before power on */
		if (l1h->config.arfcn_valid && !l1h->config.arfcn_sent) {
			trx_if_cmd_rxtune(l1h, l1h->config.arfcn);
			trx_if_cmd_txtune(l1h, l1h->config.arfcn);
			l1h->config.arfcn_sent = 1;
		}
		if (l1h->config.tsc_valid && !l1h->config.tsc_sent) {
			trx_if_cmd_settsc(l1h, l1h->config.tsc);
			l1h->config.tsc_sent = 1;
		}
		if (l1h->config.bsic_valid && !l1h->config.bsic_sent) {
			trx_if_cmd_setbsic(l1h, l1h->config.bsic);
			l1h->config.bsic_sent = 1;
		}

		if (!l1h->config.poweron_sent) {
			trx_if_cmd_poweron(l1h);
			l1h->config.poweron_sent = 1;
		}

		/* after power on */
		if (l1h->config.rxgain_valid && !l1h->config.rxgain_sent) {
			trx_if_cmd_setrxgain(l1h, l1h->config.rxgain);
			l1h->config.rxgain_sent = 1;
		}
		if (l1h->config.power_valid && !l1h->config.power_sent) {
			trx_if_cmd_setpower(l1h, l1h->config.power);
			l1h->config.power_sent = 1;
		}
		if (l1h->config.maxdly_valid && !l1h->config.maxdly_sent) {
			trx_if_cmd_setmaxdly(l1h, l1h->config.maxdly);
			l1h->config.maxdly_sent = 1;
		}
		for (tn = 0; tn < 8; tn++) {
			if (l1h->config.slottype_valid[tn]
			 && !l1h->config.slottype_sent[tn]) {
				trx_if_cmd_setslot(l1h, tn,
					l1h->config.slottype[tn]);
				l1h->config.slottype_sent[tn] = 1;
			}
		}
		return 0;
	}

	if (!l1h->config.poweron && !l1h->config.poweron_sent) {
		trx_if_cmd_poweroff(l1h);
		l1h->config.poweron_sent = 1;
		l1h->config.rxgain_sent = 0;
		l1h->config.power_sent = 0;
		l1h->config.maxdly_sent = 0;
		for (tn = 0; tn < 8; tn++)
			l1h->config.slottype_sent[tn] = 0;
	}

	return 0;
}

int l1if_provision_tranceiver(struct gsm_bts *bts)
{
	struct gsm_bts_trx *trx;
	struct trx_l1h *l1h;

	llist_for_each_entry(trx, &bts->trx_list, list) {
		l1h = trx_l1h_hdl(trx);
		l1h->config.arfcn_sent = 0;
		l1h->config.tsc_sent = 0;
		l1h->config.bsic_sent = 0;
		l1h->config.poweron_sent = 0;
		l1h->config.rxgain_sent = 0;
		l1h->config.power_sent = 0;
		l1h->config.maxdly_sent = 0;
		l1if_provision_tranceiver_trx(l1h);
	}
	return 0;
}

/*
 * activation/configuration/deactivation of tranceiver's TRX
 */

/* initialize the layer1 */
static int trx_init(struct gsm_bts_trx *trx)
{
	struct trx_l1h *l1h = trx_l1h_hdl(trx);

	/* power on tranceiver, if not already */
	if (!l1h->config.poweron) {
		l1h->config.poweron = 1;
		l1h->config.poweron_sent = 0;
		l1if_provision_tranceiver_trx(l1h);
	}

	if (trx == trx->bts->c0)
		lchan_init_lapdm(&trx->ts[0].lchan[4]);

	/* Set to Operational State: Enabled */
	oml_mo_state_chg(&trx->mo, NM_OPSTATE_ENABLED, NM_AVSTATE_OK);

	/* Send OPSTART ack */
	return oml_mo_opstart_ack(&trx->mo);
}

/* deactivate tranceiver */
static int trx_close(struct gsm_bts_trx *trx)
{
	struct trx_l1h *l1h = trx_l1h_hdl(trx);

	/* close all logical channels and reset timeslots */
	trx_sched_reset(l1h);

	/* power off tranceiver, if not already */
	if (l1h->config.poweron) {
		l1h->config.poweron = 0;
		l1h->config.poweron_sent = 0;
		l1if_provision_tranceiver_trx(l1h);
	}

	/* Set to Operational State: Disabled */
	check_tranceiver_availability_trx(l1h, 0);

	return 0;
}

/* set bts attributes */
static uint8_t trx_set_bts(struct gsm_bts *bts)
{
	struct gsm_bts_trx *trx;
	struct trx_l1h *l1h;
	uint8_t bsic = bts->bsic;

	llist_for_each_entry(trx, &bts->trx_list, list) {
		l1h = trx_l1h_hdl(trx);
		if (l1h->config.bsic != bsic || !l1h->config.bsic_valid) {
			l1h->config.bsic = bsic;
			l1h->config.bsic_valid = 1;
			l1h->config.bsic_sent = 0;
			l1if_provision_tranceiver_trx(l1h);
		}
	}
	check_tranceiver_availability(bts, tranceiver_available);


	return 0;
}

/* set trx attributes */
static uint8_t trx_set_trx(struct gsm_bts_trx *trx)
{
	struct trx_l1h *l1h = trx_l1h_hdl(trx);
	uint16_t arfcn = trx->arfcn;

	if (l1h->config.arfcn != arfcn || !l1h->config.arfcn_valid) {
		l1h->config.arfcn = arfcn;
		l1h->config.arfcn_valid = 1;
		l1h->config.arfcn_sent = 0;
		l1if_provision_tranceiver_trx(l1h);
	}

	return 0;
}

/* set ts attributes */
static uint8_t trx_set_ts(struct gsm_bts_trx_ts *ts)
{
	struct trx_l1h *l1h = trx_l1h_hdl(ts->trx);
	uint8_t tn = ts->nr;
	uint16_t tsc = ts->tsc;
	enum gsm_phys_chan_config pchan = ts->pchan;
	uint8_t slottype;
	int rc;

	/* all TSC of all timeslots must be equal, because tranceiver only
	 * supports one TSC per TRX */

	if (l1h->config.tsc != tsc || !l1h->config.tsc_valid) {
		l1h->config.tsc = tsc;
		l1h->config.tsc_valid = 1;
		l1h->config.tsc_sent = 0;
		l1if_provision_tranceiver_trx(l1h);
	}

	/* set physical channel */
	rc = trx_sched_set_pchan(l1h, tn, pchan);
	if (rc)
		return NM_NACK_RES_NOTAVAIL;

	slottype = tranceiver_chan_types[pchan];
	
	if (l1h->config.slottype[tn] != slottype
	 || !l1h->config.slottype_valid[tn]) {
		l1h->config.slottype[tn] = slottype;
		l1h->config.slottype_valid[tn] = 1;
		l1h->config.slottype_sent[tn] = 0;
		l1if_provision_tranceiver_trx(l1h);
	}

	return 0;
}


/*
 * primitive handling
 */

/* enable ciphering */
static int l1if_set_ciphering(struct trx_l1h *l1h, struct gsm_lchan *lchan,
	uint8_t chan_nr, int downlink)
{
	/* ciphering already enabled in both directions */
	if (lchan->ciph_state == LCHAN_CIPH_TXRX_CONF)
		return -EINVAL;

	if (!downlink) {
		/* set uplink */
		trx_sched_set_cipher(l1h, chan_nr, 0, lchan->encr.alg_id - 1,
			lchan->encr.key, lchan->encr.key_len);
		lchan->ciph_state = LCHAN_CIPH_RX_CONF;
	} else {
		/* set downlink and also set uplink, if not already */
		if (lchan->ciph_state != LCHAN_CIPH_RX_CONF) {
			trx_sched_set_cipher(l1h, chan_nr, 0,
				lchan->encr.alg_id - 1, lchan->encr.key,
				lchan->encr.key_len);
		}
		trx_sched_set_cipher(l1h, chan_nr, 1, lchan->encr.alg_id - 1,
			lchan->encr.key, lchan->encr.key_len);
		lchan->ciph_state = LCHAN_CIPH_TXRX_CONF;
	}

	return 0;
}

static int mph_info_chan_confirm(struct trx_l1h *l1h, uint8_t chan_nr,
	enum osmo_mph_info_type type, uint8_t cause)
{
	struct osmo_phsap_prim l1sap;

	memset(&l1sap, 0, sizeof(l1sap));
	osmo_prim_init(&l1sap.oph, SAP_GSM_PH, PRIM_MPH_INFO, PRIM_OP_CONFIRM,
		NULL);
	l1sap.u.info.type = type;
	l1sap.u.info.u.act_cnf.chan_nr = chan_nr;
	l1sap.u.info.u.act_cnf.cause = cause;

	return l1sap_up(l1h->trx, &l1sap);
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
	struct trx_l1h *l1h = trx_l1h_hdl(trx);
	struct msgb *msg = l1sap->oph.msg;
	uint8_t chan_nr;
	uint8_t tn, ss;
	int rc = 0;
	struct gsm_lchan *lchan;

	switch (OSMO_PRIM_HDR(&l1sap->oph)) {
	case OSMO_PRIM(PRIM_PH_DATA, PRIM_OP_REQUEST):
		if (!msg)
			break;
		/* put data into scheduler's queue */
		return trx_sched_ph_data_req(l1h, l1sap);
	case OSMO_PRIM(PRIM_TCH, PRIM_OP_REQUEST):
		if (!msg)
			break;
		/* put data into scheduler's queue */
		return trx_sched_tch_req(l1h, l1sap);
	case OSMO_PRIM(PRIM_MPH_INFO, PRIM_OP_REQUEST):
		switch (l1sap->u.info.type) {
		case PRIM_INFO_ACT_CIPH:
			chan_nr = l1sap->u.info.u.ciph_req.chan_nr;
			tn = L1SAP_CHAN2TS(chan_nr);
			ss = l1sap_chan2ss(chan_nr);
			lchan = &trx->ts[tn].lchan[ss];
			if (l1sap->u.info.u.ciph_req.uplink)
				l1if_set_ciphering(l1h, lchan, chan_nr, 0);
			if (l1sap->u.info.u.ciph_req.downlink)
				l1if_set_ciphering(l1h, lchan, chan_nr, 1);
			break;
		case PRIM_INFO_ACTIVATE:
		case PRIM_INFO_DEACTIVATE:
		case PRIM_INFO_MODIFY:
			chan_nr = l1sap->u.info.u.act_req.chan_nr;
			tn = L1SAP_CHAN2TS(chan_nr);
			ss = l1sap_chan2ss(chan_nr);
			lchan = &trx->ts[tn].lchan[ss];
			if (l1sap->u.info.type == PRIM_INFO_ACTIVATE) {
				if ((chan_nr & 0x80)) {
					LOGP(DL1C, LOGL_ERROR, "Cannot activate"
						" chan_nr 0x%02x\n", chan_nr);
					break;
				}
				/* activate dedicated channel */
				trx_sched_set_lchan(l1h, chan_nr, 0x00, 0, 1);
				trx_sched_set_lchan(l1h, chan_nr, 0x00, 1, 1);
				/* activate assoicated channel */
				trx_sched_set_lchan(l1h, chan_nr, 0x40, 0, 1);
				trx_sched_set_lchan(l1h, chan_nr, 0x40, 1, 1);
				/* set mode */
				trx_sched_set_mode(l1h, chan_nr,
					lchan->rsl_cmode, lchan->tch_mode);
				/* init lapdm */
				lchan_init_lapdm(lchan);
				/* confirm */
				mph_info_chan_confirm(l1h, chan_nr,
					PRIM_INFO_ACTIVATE, 0);
				break;
			}
			if (l1sap->u.info.type == PRIM_INFO_MODIFY) {
				/* change mode */
				trx_sched_set_mode(l1h, chan_nr,
					lchan->rsl_cmode, lchan->tch_mode);
				break;
			}
			if ((chan_nr & 0x80)) {
				LOGP(DL1C, LOGL_ERROR, "Cannot deactivate "
					"chan_nr 0x%02x\n", chan_nr);
				break;
			}
			/* deactivate assoicated channel */
			trx_sched_set_lchan(l1h, chan_nr, 0x40, 0, 0);
			trx_sched_set_lchan(l1h, chan_nr, 0x40, 1, 0);
			/* deactivate dedicated channel */
			if (!l1sap->u.info.u.act_req.sacch_only) {
				trx_sched_set_lchan(l1h, chan_nr, 0x00, 0, 0);
				trx_sched_set_lchan(l1h, chan_nr, 0x00, 1, 0);
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
		cause = trx_set_bts(obj);
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

	switch (mo->obj_class) {
	case NM_OC_RADIO_CARRIER:
		/* activate tranceiver */
		rc = trx_init(obj);
		break;
	case NM_OC_CHANNEL:
		/* configure timeslot */
		rc = 0; //ts_connect(obj);

		/* Set to Operational State: Enabled */
		oml_mo_state_chg(mo, NM_OPSTATE_ENABLED, NM_AVSTATE_OK);

		/* Send OPSTART ack */
		rc = oml_mo_opstart_ack(mo);

		break;
	case NM_OC_BTS:
	case NM_OC_SITE_MANAGER:
	case NM_OC_BASEB_TRANSC:
	case NM_OC_GPRS_NSE:
	case NM_OC_GPRS_CELL:
	case NM_OC_GPRS_NSVC:
		oml_mo_state_chg(mo, NM_OPSTATE_ENABLED, -1);
		rc = oml_mo_opstart_ack(mo);
		break;
	default:
		rc = oml_mo_opstart_nack(mo, NM_NACK_OBJCLASS_NOTSUPP);
	}
	return rc;
}

int bts_model_chg_adm_state(struct gsm_bts *bts, struct gsm_abis_mo *mo,
			    void *obj, uint8_t adm_state)
{
	/* blindly accept all state changes */
	mo->nm_state.administrative = adm_state;
	return oml_mo_statechg_ack(mo);
}

int bts_model_trx_deact_rf(struct gsm_bts_trx *trx)
{
	return trx_close(trx);
}

int bts_model_oml_estab(struct gsm_bts *bts)
{
	return 0;
}

