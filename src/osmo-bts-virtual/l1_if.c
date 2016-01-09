/* Virtual BTS layer 1 primitive handling and interface
 *
 * Copyright (C) 2015 Harald Welte <laforge@gnumonks.org>
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
#include <osmocom/core/linuxlist.h>

#include <osmo-bts/logging.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/oml.h>
#include <osmo-bts/rsl.h>
#include <osmo-bts/l1sap.h>
#include <osmo-bts/bts_model.h>
#include <osmo-bts/phy_link.h>
#include <osmo-bts/amr.h>
#include <osmo-bts/abis.h>
#include <osmo-bts/scheduler.h>

#include "virtual_um.h"

static void virt_um_rcv_cb(struct virt_um_inst *vui, struct msgb *msg)
{
	/* FIXME: Handle msg from MS */
}


/* called by common part once OML link is established */
int bts_model_oml_estab(struct gsm_bts *bts)
{
	return 0;
}

int bts_model_phy_link_open(struct phy_link *plink)
{
	struct phy_instance *pinst;

	//OSMO_ASSERT(plink->type == PHY_LINK_T_VIRTUAL);

	if (plink->u.virt.virt_um)
		virt_um_destroy(plink->u.virt.virt_um);

	phy_link_state_set(plink, PHY_LINK_CONNECTING);

	plink->u.virt.virt_um = virt_um_init(plink, plink->u.virt.mcast_group,
				      plink->u.virt.mcast_port,
				      plink->u.virt.mcast_dev, plink,
				      virt_um_rcv_cb);
	if (!plink->u.virt.virt_um) {
		phy_link_state_set(plink, PHY_LINK_SHUTDOWN);
		return -1;
	}

	/* iterate over list of PHY instances and initialize the
	 * scheduler */
	llist_for_each_entry(pinst, &plink->instances, list) {
		trx_sched_init(&pinst->u.virt.sched, pinst->trx);
		if (pinst->trx == pinst->trx->bts->c0)
			vbts_sched_start(pinst->trx->bts);
	}

	/* this will automatically update the MO state of all associated
	 * TRX objects */
	phy_link_state_set(plink, PHY_LINK_CONNECTED);

	return 0;
}


/*
 * primitive handling
 */

/* enable ciphering */
static int l1if_set_ciphering(struct gsm_lchan *lchan, uint8_t chan_nr, int downlink)
{
	struct gsm_bts_trx *trx = lchan->ts->trx;
	struct phy_instance *pinst = trx_phy_instance(trx);
	struct l1sched_trx *sched = &pinst->u.virt.sched;

	/* ciphering already enabled in both directions */
	if (lchan->ciph_state == LCHAN_CIPH_RXTX_CONF)
		return -EINVAL;

	if (!downlink) {
		/* set uplink */
		trx_sched_set_cipher(sched, chan_nr, 0, lchan->encr.alg_id - 1,
			lchan->encr.key, lchan->encr.key_len);
		lchan->ciph_state = LCHAN_CIPH_RX_CONF;
	} else {
		/* set downlink and also set uplink, if not already */
		if (lchan->ciph_state != LCHAN_CIPH_RX_CONF) {
			trx_sched_set_cipher(sched, chan_nr, 0,
				lchan->encr.alg_id - 1, lchan->encr.key,
				lchan->encr.key_len);
		}
		trx_sched_set_cipher(sched, chan_nr, 1, lchan->encr.alg_id - 1,
			lchan->encr.key, lchan->encr.key_len);
		lchan->ciph_state = LCHAN_CIPH_RXTX_CONF;
	}

	return 0;
}

static int mph_info_chan_confirm(struct gsm_bts_trx *trx, uint8_t chan_nr,
	enum osmo_mph_info_type type, uint8_t cause)
{
	struct osmo_phsap_prim l1sap;

	memset(&l1sap, 0, sizeof(l1sap));
	osmo_prim_init(&l1sap.oph, SAP_GSM_PH, PRIM_MPH_INFO, PRIM_OP_CONFIRM,
		NULL);
	l1sap.u.info.type = type;
	l1sap.u.info.u.act_cnf.chan_nr = chan_nr;
	l1sap.u.info.u.act_cnf.cause = cause;

	return l1sap_up(trx, &l1sap);
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


static void l1if_fill_meas_res(struct osmo_phsap_prim *l1sap, uint8_t chan_nr, float ta,
				float ber, float rssi)
{
	memset(l1sap, 0, sizeof(*l1sap));
	osmo_prim_init(&l1sap->oph, SAP_GSM_PH, PRIM_MPH_INFO,
		PRIM_OP_INDICATION, NULL);
	l1sap->u.info.type = PRIM_INFO_MEAS;
	l1sap->u.info.u.meas_ind.chan_nr = chan_nr;
	l1sap->u.info.u.meas_ind.ta_offs_qbits = (int16_t)(ta*4);
	l1sap->u.info.u.meas_ind.ber10k = (unsigned int) (ber * 10000);
	l1sap->u.info.u.meas_ind.inv_rssi = (uint8_t) (rssi * -1);
}

int l1if_process_meas_res(struct gsm_bts_trx *trx, uint8_t tn, uint32_t fn, uint8_t chan_nr,
	int n_errors, int n_bits_total, float rssi, float toa)
{
	struct gsm_lchan *lchan = &trx->ts[tn].lchan[l1sap_chan2ss(chan_nr)];
	struct osmo_phsap_prim l1sap;
	/* 100% BER is n_bits_total is 0 */
	float ber = n_bits_total==0 ? 1.0 : (float)n_errors / (float)n_bits_total;

	LOGP(DMEAS, LOGL_DEBUG, "RX L1 frame %s fn=%u chan_nr=0x%02x MS pwr=%ddBm rssi=%.1f dBFS "
		"ber=%.2f%% (%d/%d bits) L1_ta=%d rqd_ta=%d toa=%.2f\n",
		gsm_lchan_name(lchan), fn, chan_nr, ms_pwr_dbm(lchan->ts->trx->bts->band, lchan->ms_power),
		rssi, ber*100, n_errors, n_bits_total, lchan->meas.l1_info[1], lchan->rqd_ta, toa);

	l1if_fill_meas_res(&l1sap, chan_nr, lchan->rqd_ta + toa, ber, rssi);

	return l1sap_up(trx, &l1sap);
}



/* primitive from common part */
int bts_model_l1sap_down(struct gsm_bts_trx *trx, struct osmo_phsap_prim *l1sap)
{
	struct phy_instance *pinst = trx_phy_instance(trx);
	struct l1sched_trx *sched = &pinst->u.virt.sched;
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
		return trx_sched_ph_data_req(sched, l1sap);
	case OSMO_PRIM(PRIM_TCH, PRIM_OP_REQUEST):
		if (!msg)
			break;
		/* put data into scheduler's queue */
		return trx_sched_tch_req(sched, l1sap);
	case OSMO_PRIM(PRIM_MPH_INFO, PRIM_OP_REQUEST):
		switch (l1sap->u.info.type) {
		case PRIM_INFO_ACT_CIPH:
			chan_nr = l1sap->u.info.u.ciph_req.chan_nr;
			tn = L1SAP_CHAN2TS(chan_nr);
			ss = l1sap_chan2ss(chan_nr);
			lchan = &trx->ts[tn].lchan[ss];
			if (l1sap->u.info.u.ciph_req.uplink)
				l1if_set_ciphering(lchan, chan_nr, 0);
			if (l1sap->u.info.u.ciph_req.downlink)
				l1if_set_ciphering(lchan, chan_nr, 1);
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
				trx_sched_set_lchan(sched, chan_nr, 0x00, 1);
				/* activate associated channel */
				trx_sched_set_lchan(sched, chan_nr, 0x40, 1);
				/* set mode */
				trx_sched_set_mode(sched, chan_nr,
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
				l1if_set_ciphering(lchan, chan_nr, 0);
				l1if_set_ciphering(lchan, chan_nr, 1);
				if (lchan->encr.alg_id)
					lchan->ciph_state = LCHAN_CIPH_RXTX_CONF;
				else
					lchan->ciph_state = LCHAN_CIPH_NONE;

				/* confirm */
				mph_info_chan_confirm(trx, chan_nr,
					PRIM_INFO_ACTIVATE, 0);
				break;
			}
			if (l1sap->u.info.type == PRIM_INFO_MODIFY) {
				/* change mode */
				trx_sched_set_mode(sched, chan_nr,
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
			if ((chan_nr & 0x80)) {
				LOGP(DL1C, LOGL_ERROR, "Cannot deactivate "
					"chan_nr 0x%02x\n", chan_nr);
				break;
			}
			/* deactivate associated channel */
			trx_sched_set_lchan(sched, chan_nr, 0x40, 0);
			if (!l1sap->u.info.u.act_req.sacch_only) {
				/* set lchan inactive */
				lchan_set_state(lchan, LCHAN_S_NONE);
				/* deactivate dedicated channel */
				trx_sched_set_lchan(sched, chan_nr, 0x00, 0);
				/* confirm only on dedicated channel */
				mph_info_chan_confirm(trx, chan_nr,
					PRIM_INFO_DEACTIVATE, 0);
				lchan->ciph_state = 0; /* FIXME: do this in common/\*.c */
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
