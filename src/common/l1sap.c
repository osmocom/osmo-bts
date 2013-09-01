/* L1 SAP primitives */

/* (C) 2011 by Harald Welte <laforge@gnumonks.org>
 * (C) 2013 by Andreas Eversberg <jolly@eversberg.eu>
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

#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>

#include <sys/types.h>
#include <sys/stat.h>

#include <osmocom/core/msgb.h>
#include <osmocom/gsm/l1sap.h>
#include <osmocom/core/gsmtap.h>
#include <osmocom/core/gsmtap_util.h>
#include <osmocom/core/utils.h>

#include <osmocom/trau/osmo_ortp.h>

#include <osmo-bts/logging.h>
#include <osmo-bts/gsm_data.h>
#include <osmo-bts/l1sap.h>
#include <osmo-bts/pcu_if.h>
#include <osmo-bts/measurement.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/rsl.h>
#include <osmo-bts/bts_model.h>

static int l1sap_down(struct gsm_bts_trx *trx, struct osmo_phsap_prim *l1sap);

static const uint8_t fill_frame[GSM_MACBLOCK_LEN] = {
        0x03, 0x03, 0x01, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B,
        0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B,
        0x2B, 0x2B, 0x2B
};

/* allocate a msgb containing a osmo_phsap_prim + optional l2 data
 * in order to wrap femtobts header arround l2 data, there must be enough space
 * in front and behind data pointer */
struct msgb *l1sap_msgb_alloc(unsigned int l2_len)
{
	struct msgb *msg = msgb_alloc_headroom(512, 128, "l1sap_prim");

	if (!msg)
		return NULL;

	msg->l1h = msgb_put(msg, sizeof(struct osmo_phsap_prim));

	return msg;
}

/* time information received from bts model */
static int l1sap_info_time_ind(struct gsm_bts_trx *trx,
	struct osmo_phsap_prim *l1sap,
	struct info_time_ind_param *info_time_ind)
{
	struct gsm_bts *bts = trx->bts;
	struct gsm_bts_role_bts *btsb = bts->role;

	DEBUGP(DL1P, "MPH_INFO time ind %u\n", info_time_ind->fn);

	/* Update our data structures with the current GSM time */
	gsm_fn2gsmtime(&btsb->gsm_time, info_time_ind->fn);

	/* Update time on PCU interface */
	pcu_tx_time_ind(info_time_ind->fn);

	/* check if the measurement period of some lchan has ended
	 * and pre-compute the respective measurement */
	trx_meas_check_compute(trx, info_time_ind->fn - 1);

	/* increment 'total' for every possible rach */
	if (bts->c0->ts[0].pchan != GSM_PCHAN_CCCH_SDCCH4
	 || (info_time_ind->fn % 51) < 27)
		btsb->load.rach.total++;

	return 0;
}

/* any L1 MPH_INFO indication prim recevied from bts model */
static int l1sap_mph_info_ind(struct gsm_bts_trx *trx,
	 struct osmo_phsap_prim *l1sap, struct mph_info_param *info)
{
	int rc = 0;

	switch (info->type) {
	case PRIM_INFO_TIME:
		rc = l1sap_info_time_ind(trx, l1sap, &info->u.time_ind);
		break;
	default:
		LOGP(DL1P, LOGL_NOTICE, "unknown MPH_INFO ind type %d\n",
			info->type);
		break;
	}

	return rc;
}

/* activation confirm received from bts model */
static int l1sap_info_act_cnf(struct gsm_bts_trx *trx,
	struct osmo_phsap_prim *l1sap,
	struct info_act_cnf_param *info_act_cnf)
{
	struct gsm_lchan *lchan;

	LOGP(DL1P, LOGL_INFO, "activate confirm chan_nr=%02x trx=%d\n",
		info_act_cnf->chan_nr, trx->nr);

	lchan = &trx->ts[L1SAP_CHAN2TS(info_act_cnf->chan_nr)]
				.lchan[l1sap_chan2ss(info_act_cnf->chan_nr)];

	if (info_act_cnf->cause)
		rsl_tx_chan_act_nack(lchan, info_act_cnf->cause);
	else
		rsl_tx_chan_act_ack(lchan);

	return 0;
}

/* activation confirm received from bts model */
static int l1sap_info_rel_cnf(struct gsm_bts_trx *trx,
	struct osmo_phsap_prim *l1sap,
	struct info_act_cnf_param *info_act_cnf)
{
	struct gsm_lchan *lchan;

	LOGP(DL1P, LOGL_INFO, "deactivate confirm chan_nr=%02x trx=%d\n",
		info_act_cnf->chan_nr, trx->nr);

	lchan = &trx->ts[L1SAP_CHAN2TS(info_act_cnf->chan_nr)]
				.lchan[l1sap_chan2ss(info_act_cnf->chan_nr)];

	rsl_tx_rf_rel_ack(lchan);

	return 0;
}

/* any L1 MPH_INFO confirm prim recevied from bts model */
static int l1sap_mph_info_cnf(struct gsm_bts_trx *trx,
	 struct osmo_phsap_prim *l1sap, struct mph_info_param *info)
{
	int rc = 0;

	switch (info->type) {
	case PRIM_INFO_ACTIVATE:
		rc = l1sap_info_act_cnf(trx, l1sap, &info->u.act_cnf);
		break;
	case PRIM_INFO_DEACTIVATE:
		rc = l1sap_info_rel_cnf(trx, l1sap, &info->u.act_cnf);
		break;
	default:
		LOGP(DL1P, LOGL_NOTICE, "unknown MPH_INFO cnf type %d\n",
			info->type);
		break;
	}

	return rc;
}

/* PH-RTS-IND prim recevied from bts model */
static int l1sap_ph_rts_ind(struct gsm_bts_trx *trx,
	struct osmo_phsap_prim *l1sap, struct ph_data_param *rts_ind)
{
	struct msgb *msg = l1sap->oph.msg;
	struct gsm_time g_time;
	uint8_t chan_nr, link_id;
	uint8_t tn;
	uint32_t fn;
	uint8_t *p, *si;
	int rc;

	chan_nr = rts_ind->chan_nr;
	link_id = rts_ind->link_id;
	fn = rts_ind->fn;
	tn = L1SAP_CHAN2TS(chan_nr);

	gsm_fn2gsmtime(&g_time, fn);

	DEBUGP(DL1P, "Rx PH-RTS.ind %02u/%02u/%02u chan_nr=%d link_id=%d\n",
		g_time.t1, g_time.t2, g_time.t3, chan_nr, link_id);

	if (trx->ts[tn].pchan == GSM_PCHAN_PDCH) {
		if (L1SAP_IS_PTCCH(rts_ind->fn)) {
			pcu_tx_rts_req(&trx->ts[tn], 1, fn, 1 /* ARFCN */,
				L1SAP_FN2PTCCHBLOCK(fn));

			return 0;
		}
		pcu_tx_rts_req(&trx->ts[tn], 0, fn, 0 /* ARFCN */,
			L1SAP_FN2MACBLOCK(fn));

		return 0;
	}

	/* reuse PH-RTS.ind for PH-DATA.req */
	if (!msg) {
		LOGP(DL1P, LOGL_FATAL, "RTS without msg to be reused. Please "
			"fix!\n");
		abort();
	}
	msgb_trim(msg, sizeof(*l1sap));
	osmo_prim_init(&l1sap->oph, SAP_GSM_PH, PRIM_PH_DATA, PRIM_OP_REQUEST,
		msg);
	msg->l2h = msg->l1h + sizeof(*l1sap);

	if (L1SAP_IS_CHAN_BCCH(chan_nr)) {
		p = msgb_put(msg, GSM_MACBLOCK_LEN);
		/* get them from bts->si_buf[] */
		si = bts_sysinfo_get(trx->bts, &g_time);
		if (si)
			memcpy(p, si, GSM_MACBLOCK_LEN);
		else
			memcpy(p, fill_frame, GSM_MACBLOCK_LEN);
	} else if (L1SAP_IS_CHAN_AGCH_PCH(chan_nr)) {
		p = msgb_put(msg, GSM_MACBLOCK_LEN);
		/* if CCCH block is 0, it is AGCH */
		rc = bts_ccch_copy_msg(trx->bts, p, &g_time,
			(L1SAP_FN2CCCHBLOCK(fn) < 1));
		if (rc <= 0)
			memcpy(p, fill_frame, GSM_MACBLOCK_LEN);
	}

	DEBUGP(DL1P, "Tx PH-DATA.req %02u/%02u/%02u chan_nr=%d link_id=%d\n",
		g_time.t1, g_time.t2, g_time.t3, chan_nr, link_id);

	l1sap_down(trx, l1sap);

	/* don't free, because we forwarded data */
	return 1;
}

static int check_acc_delay(struct ph_rach_ind_param *rach_ind,
	struct gsm_bts_role_bts *btsb, uint8_t *acc_delay)
{
	*acc_delay = rach_ind->acc_delay;
	return *acc_delay <= btsb->max_ta;
}

/* special case where handover RACH is detected */
static int l1sap_handover_rach(struct gsm_bts_trx *trx,
	struct osmo_phsap_prim *l1sap, struct ph_rach_ind_param *rach_ind)
{
	struct gsm_lchan *lchan;
	uint8_t chan_nr;
	uint8_t tn, ss;

	chan_nr = rach_ind->chan_nr;
	tn = L1SAP_CHAN2TS(chan_nr);
	ss = l1sap_chan2ss(chan_nr);
	lchan = &trx->ts[tn].lchan[ss];

	handover_rach(lchan, rach_ind->ra, rach_ind->acc_delay);

	/* must return 0, so in case of msg at l1sap, it will be freed */
	return 0;
}

/* DATA received from bts model */
static int l1sap_ph_data_ind(struct gsm_bts_trx *trx,
	 struct osmo_phsap_prim *l1sap, struct ph_data_param *data_ind)
{
	struct msgb *msg = l1sap->oph.msg;
	struct gsm_time g_time;
	uint8_t *data = msg->l2h;
	int len = msgb_l2len(msg);
	uint8_t chan_nr, link_id;
	uint8_t tn, ss;
	uint32_t fn;
	int8_t rssi;

	rssi = data_ind->rssi;
	chan_nr = data_ind->chan_nr;
	link_id = data_ind->link_id;
	fn = data_ind->fn;
	tn = L1SAP_CHAN2TS(chan_nr);
	ss = l1sap_chan2ss(chan_nr);

	gsm_fn2gsmtime(&g_time, fn);

	DEBUGP(DL1P, "Rx PH-DATA.ind %02u/%02u/%02u chan_nr=%d link_id=%d\n",
		g_time.t1, g_time.t2, g_time.t3, chan_nr, link_id);

	if (trx->ts[tn].pchan == GSM_PCHAN_PDCH) {
		if (len == 0)
			return -EINVAL;
		if (L1SAP_IS_PTCCH(fn)) {
			pcu_tx_data_ind(&trx->ts[tn], 1, fn,
				0 /* ARFCN */, L1SAP_FN2PTCCHBLOCK(fn),
				data, len, rssi);

			return 0;
		}
		/* drop incomplete UL block */
		if (data[0] != 7)
			return 0;
		/* PDTCH / PACCH frame handling */
		pcu_tx_data_ind(&trx->ts[tn], 0, fn, 0 /* ARFCN */,
			L1SAP_FN2MACBLOCK(fn), data + 1, len - 1, rssi);

		return 0;
	}

	return 0;
}

/* RACH received from bts model */
static int l1sap_ph_rach_ind(struct gsm_bts_trx *trx,
	 struct osmo_phsap_prim *l1sap, struct ph_rach_ind_param *rach_ind)
{
	struct gsm_bts *bts = trx->bts;
	struct gsm_bts_role_bts *btsb = bts->role;
	struct lapdm_channel *lc;
	uint8_t acc_delay;

	DEBUGP(DL1P, "Rx PH-RA.ind");

	lc = &trx->ts[0].lchan[4].lapdm_ch;

	/* check for under/overflow / sign */
	if (!check_acc_delay(rach_ind, btsb, &acc_delay)) {
		LOGP(DL1C, LOGL_INFO, "ignoring RACH request %u > max_ta(%u)\n",
		     acc_delay, btsb->max_ta);
		return 0;
	}

	/* check for handover rach */
	if (trx != bts->c0 && rach_ind->chan_nr != 0x88)
		return l1sap_handover_rach(trx, l1sap, rach_ind);

	/* check for packet access */
	if (trx == bts->c0
	 && L1SAP_IS_PACKET_RACH(rach_ind->ra)) {
		LOGP(DL1P, LOGL_INFO, "RACH for packet access\n");
		pcu_tx_rach_ind(bts, rach_ind->acc_delay << 2,
			rach_ind->ra, rach_ind->fn);
		return 0;
	}

	LOGP(DL1P, LOGL_INFO, "RACH for RR access (toa=%d, ra=%d)\n",
		rach_ind->acc_delay, rach_ind->ra);
	lapdm_phsap_up(&l1sap->oph, &lc->lapdm_dcch);

	return 0;
}

/* any L1 prim received from bts model */
int l1sap_up(struct gsm_bts_trx *trx, struct osmo_phsap_prim *l1sap)
{
	struct msgb *msg = l1sap->oph.msg;
	int rc = 0;

	switch (OSMO_PRIM_HDR(&l1sap->oph)) {
	case OSMO_PRIM(PRIM_MPH_INFO, PRIM_OP_INDICATION):
		rc = l1sap_mph_info_ind(trx, l1sap, &l1sap->u.info);
		break;
	case OSMO_PRIM(PRIM_MPH_INFO, PRIM_OP_CONFIRM):
		rc = l1sap_mph_info_cnf(trx, l1sap, &l1sap->u.info);
		break;
	case OSMO_PRIM(PRIM_PH_RTS, PRIM_OP_INDICATION):
		rc = l1sap_ph_rts_ind(trx, l1sap, &l1sap->u.data);
		break;
	case OSMO_PRIM(PRIM_PH_DATA, PRIM_OP_INDICATION):
		rc = l1sap_ph_data_ind(trx, l1sap, &l1sap->u.data);
		break;
	case OSMO_PRIM(PRIM_PH_RACH, PRIM_OP_INDICATION):
		rc = l1sap_ph_rach_ind(trx, l1sap, &l1sap->u.rach_ind);
		break;
	default:
		LOGP(DL1P, LOGL_NOTICE, "unknown prim %d op %d\n",
			l1sap->oph.primitive, l1sap->oph.operation);
		break;
	}

	/* Special return value '1' means: do not free */
	if (rc != 1)
		msgb_free(msg);

	return rc;
}

/* any L1 prim sent to bts model */
static int l1sap_down(struct gsm_bts_trx *trx, struct osmo_phsap_prim *l1sap)
{
	return bts_model_l1sap_down(trx, l1sap);
}

/* pcu (socket interface) sends us a data request primitive */
int l1sap_pdch_req(struct gsm_bts_trx_ts *ts, int is_ptcch, uint32_t fn,
	uint16_t arfcn, uint8_t block_nr, uint8_t *data, uint8_t len)
{
	struct msgb *msg;
	struct osmo_phsap_prim *l1sap;
	struct gsm_time g_time;

	gsm_fn2gsmtime(&g_time, fn);

	DEBUGP(DL1P, "TX packet data %02u/%02u/%02u is_ptcch=%d trx=%d ts=%d "
		"block_nr=%d, arfcn=%d, len=%d\n", g_time.t1, g_time.t2,
		g_time.t3, is_ptcch, ts->trx->nr, ts->nr, block_nr, arfcn, len);

	msg = l1sap_msgb_alloc(len);
	l1sap = msgb_l1sap_prim(msg);
	osmo_prim_init(&l1sap->oph, SAP_GSM_PH, PRIM_PH_DATA, PRIM_OP_REQUEST,
		msg);
	l1sap->u.data.chan_nr = 0x08 | ts->nr;
	l1sap->u.data.link_id = 0x00;
	l1sap->u.data.fn = fn;
	msg->l2h = msgb_put(msg, len);
	memcpy(msg->l2h, data, len);

	return l1sap_down(ts->trx, l1sap);
}

static int l1sap_chan_act_dact_modify(struct gsm_bts_trx *trx, uint8_t chan_nr,
		enum osmo_mph_info_type type, uint8_t sacch_only)
{
	struct osmo_phsap_prim l1sap;

	memset(&l1sap, 0, sizeof(l1sap));
	osmo_prim_init(&l1sap.oph, SAP_GSM_PH, PRIM_MPH_INFO, PRIM_OP_REQUEST,
		NULL);
	l1sap.u.info.type = type;
	l1sap.u.info.u.act_req.chan_nr = chan_nr;
	l1sap.u.info.u.act_req.sacch_only = sacch_only;

	return l1sap_down(trx, &l1sap);
}

int l1sap_chan_act(struct gsm_bts_trx *trx, uint8_t chan_nr, struct tlv_parsed *tp)
{
	struct gsm_bts_role_bts *btsb = trx->bts->role;
	struct gsm_lchan *lchan = &trx->ts[L1SAP_CHAN2TS(chan_nr)]
				.lchan[l1sap_chan2ss(chan_nr)];
	struct gsm48_chan_desc *cd;
	int rc;

	LOGP(DL1P, LOGL_INFO, "activating channel chan_nr=%02x trx=%d\n",
		chan_nr, trx->nr);

	if (tp && TLVP_PRESENT(tp, GSM48_IE_CHANDESC_2) &&
	    TLVP_LEN(tp, GSM48_IE_CHANDESC_2) >= sizeof(*cd)) {
		cd = (struct gsm48_chan_desc *)
		TLVP_VAL(tp, GSM48_IE_CHANDESC_2);

		/* our L1 only supports one global TSC for all channels
		 * one one TRX, so we need to make sure not to activate
		 * channels with a different TSC!! */
		if (cd->h0.tsc != (lchan->ts->trx->bts->bsic & 7)) {
			LOGP(DRSL, LOGL_ERROR, "lchan TSC %u != BSIC-TSC %u\n",
				cd->h0.tsc, lchan->ts->trx->bts->bsic & 7);
			return -RSL_ERR_SERV_OPT_UNIMPL;
		}
	}

	lchan->sacch_deact = 0;
	lchan->s = btsb->radio_link_timeout;

	rc = l1sap_chan_act_dact_modify(trx, chan_nr, PRIM_INFO_ACTIVATE, 0);
	if (rc)
		return -RSL_ERR_EQUIPMENT_FAIL;
	return 0;
}

int l1sap_chan_rel(struct gsm_bts_trx *trx, uint8_t chan_nr)
{
	LOGP(DL1P, LOGL_INFO, "deactivating channel chan_nr=%02x trx=%d\n",
		chan_nr, trx->nr);

	return l1sap_chan_act_dact_modify(trx, chan_nr, PRIM_INFO_DEACTIVATE,
		0);
}

int l1sap_chan_deact_sacch(struct gsm_bts_trx *trx, uint8_t chan_nr)
{
	struct gsm_lchan *lchan = &trx->ts[L1SAP_CHAN2TS(chan_nr)]
				.lchan[l1sap_chan2ss(chan_nr)];

	LOGP(DL1P, LOGL_INFO, "deactivating sacch chan_nr=%02x trx=%d\n",
		chan_nr, trx->nr);

	lchan->sacch_deact = 1;

	return l1sap_chan_act_dact_modify(trx, chan_nr, PRIM_INFO_DEACTIVATE,
		1);
}

int l1sap_chan_modify(struct gsm_bts_trx *trx, uint8_t chan_nr)
{
	LOGP(DL1P, LOGL_INFO, "modifying channel chan_nr=%02x trx=%d\n",
		chan_nr, trx->nr);

	return l1sap_chan_act_dact_modify(trx, chan_nr, PRIM_INFO_MODIFY, 0);
}
