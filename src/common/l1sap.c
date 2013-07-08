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
#include <osmo-bts/handover.h>

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

static int l1sap_tx_ciph_req(struct gsm_bts_trx *trx, uint8_t chan_nr,
	uint8_t downlink, uint8_t uplink)
{
	struct osmo_phsap_prim l1sap_ciph;

	osmo_prim_init(&l1sap_ciph.oph, SAP_GSM_PH, PRIM_MPH_INFO,
		PRIM_OP_REQUEST, NULL);
	l1sap_ciph.u.info.type = PRIM_INFO_ACT_CIPH;
	l1sap_ciph.u.info.u.ciph_req.chan_nr = chan_nr;
	l1sap_ciph.u.info.u.ciph_req.downlink = downlink;
	l1sap_ciph.u.info.u.ciph_req.uplink = uplink;

	return l1sap_down(trx, &l1sap_ciph);
}


/* check if the message is a GSM48_MT_RR_CIPH_M_CMD, and if yes, enable
 * uni-directional de-cryption on the uplink. We need this ugly layering
 * violation as we have no way of passing down L3 metadata (RSL CIPHERING CMD)
 * to this point in L1 */
static int check_for_ciph_cmd(struct msgb *msg, struct gsm_lchan *lchan,
	uint8_t chan_nr)
{

	/* only do this if we are in the right state */
	switch (lchan->ciph_state) {
	case LCHAN_CIPH_NONE:
	case LCHAN_CIPH_RX_REQ:
		break;
	default:
		return 0;
	}

	/* First byte (Address Field) of LAPDm header) */
	if (msg->data[0] != 0x03)
		return 0;
	/* First byte (protocol discriminator) of RR */
	if ((msg->data[3] & 0xF) != GSM48_PDISC_RR)
		return 0;
	/* 2nd byte (msg type) of RR */
	if ((msg->data[4] & 0x3F) != GSM48_MT_RR_CIPH_M_CMD)
		return 0;

	l1sap_tx_ciph_req(lchan->ts->trx, chan_nr, 0, 1);

	return 1;
}

struct gsmtap_inst *gsmtap = NULL;
uint32_t gsmtap_sapi_mask = 0;
uint8_t gsmtap_sapi_acch = 0;

const struct value_string gsmtap_sapi_names[] = {
	{ GSMTAP_CHANNEL_BCCH,	"BCCH" },
	{ GSMTAP_CHANNEL_CCCH,	"CCCH" },
	{ GSMTAP_CHANNEL_RACH,	"RACH" },
	{ GSMTAP_CHANNEL_AGCH,	"AGCH" },
	{ GSMTAP_CHANNEL_PCH,	"PCH" },
	{ GSMTAP_CHANNEL_SDCCH,	"SDCCH" },
	{ GSMTAP_CHANNEL_TCH_F,	"TCH/F" },
	{ GSMTAP_CHANNEL_TCH_H,	"TCH/H" },
	{ GSMTAP_CHANNEL_PACCH,	"PACCH" },
	{ GSMTAP_CHANNEL_PDCH,	"PDTCH" },
	{ GSMTAP_CHANNEL_PTCCH,	"PTCCH" },
	{ GSMTAP_CHANNEL_CBCH51,"CBCH" },
	{ GSMTAP_CHANNEL_ACCH,  "SACCH" },
	{ 0, NULL }
};

/* send primitive as gsmtap */
static int gsmtap_ph_data(struct osmo_phsap_prim *l1sap, uint8_t *chan_type,
	uint8_t *tn, uint8_t *ss, uint32_t *fn, uint8_t **data, int *len)
{
	struct msgb *msg = l1sap->oph.msg;
	uint8_t chan_nr, link_id;

	*data = msg->data + sizeof(struct osmo_phsap_prim);
	*len = msg->len - sizeof(struct osmo_phsap_prim);
	*fn = l1sap->u.data.fn;
	*tn = L1SAP_CHAN2TS(l1sap->u.data.chan_nr);
	chan_nr = l1sap->u.data.chan_nr;
	link_id = l1sap->u.data.link_id;

	if (L1SAP_IS_CHAN_TCHF(chan_nr)) {
		*chan_type = GSMTAP_CHANNEL_TCH_F;
	} else if (L1SAP_IS_CHAN_TCHH(chan_nr)) {
		*ss = L1SAP_CHAN2SS_TCHH(chan_nr);
		*chan_type = GSMTAP_CHANNEL_TCH_H;
	} else if (L1SAP_IS_CHAN_SDCCH4(chan_nr)) {
		*ss = L1SAP_CHAN2SS_SDCCH4(chan_nr);
		*chan_type = GSMTAP_CHANNEL_SDCCH;
	} else if (L1SAP_IS_CHAN_SDCCH8(chan_nr)) {
		*ss = L1SAP_CHAN2SS_SDCCH8(chan_nr);
		*chan_type = GSMTAP_CHANNEL_SDCCH;
	} else if (L1SAP_IS_CHAN_BCCH(chan_nr)) {
		*chan_type = GSMTAP_CHANNEL_BCCH;
	} else if (L1SAP_IS_CHAN_AGCH_PCH(chan_nr)) {
#warning Set BS_AG_BLKS_RES
		/* The sapi depends on DSP configuration, not
		 * on the actual SYSTEM INFORMATION 3. */
		if (L1SAP_FN2CCCHBLOCK(*fn) >= 1)
			*chan_type = GSMTAP_CHANNEL_PCH;
		else
			*chan_type = GSMTAP_CHANNEL_AGCH;
	}
	if (L1SAP_IS_LINK_SACCH(link_id))
		*chan_type |= GSMTAP_CHANNEL_ACCH;

	return 0;
}

static int gsmtap_pdch(struct osmo_phsap_prim *l1sap, uint8_t *chan_type,
	uint8_t *tn, uint8_t *ss, uint32_t *fn, uint8_t **data, int *len)
{
	struct msgb *msg = l1sap->oph.msg;

	*data = msg->data + sizeof(struct osmo_phsap_prim);
	*len = msg->len - sizeof(struct osmo_phsap_prim);
	*fn = l1sap->u.data.fn;
	*tn = L1SAP_CHAN2TS(l1sap->u.data.chan_nr);

	if (L1SAP_IS_PTCCH(*fn)) {
		*chan_type = GSMTAP_CHANNEL_PTCCH;
		*ss = L1SAP_FN2PTCCHBLOCK(*fn);
		if (l1sap->oph.primitive
				== PRIM_OP_INDICATION) {
			if ((*data[0]) == 7)
				return -EINVAL;
			(*data)++;
			(*len)--;
		}
	} else
		*chan_type = GSMTAP_CHANNEL_PACCH;

	return 0;
}

static int gsmtap_ph_rach(struct osmo_phsap_prim *l1sap, uint8_t *chan_type,
	uint8_t *tn, uint8_t *ss, uint32_t *fn, uint8_t **data, int *len)
{
	uint8_t chan_nr;

	*chan_type = GSMTAP_CHANNEL_RACH;
	*fn = l1sap->u.rach_ind.fn;
	*tn = L1SAP_CHAN2TS(l1sap->u.rach_ind.chan_nr);
	chan_nr = l1sap->u.rach_ind.chan_nr;
	if (L1SAP_IS_CHAN_TCHH(chan_nr))
		*ss = L1SAP_CHAN2SS_TCHH(chan_nr);
	else if (L1SAP_IS_CHAN_SDCCH4(chan_nr))
		*ss = L1SAP_CHAN2SS_SDCCH4(chan_nr);
	else if (L1SAP_IS_CHAN_SDCCH8(chan_nr))
		*ss = L1SAP_CHAN2SS_SDCCH8(chan_nr);
	*data = &l1sap->u.rach_ind.ra;
	*len = 1;

	return 0;
}

static int to_gsmtap(struct gsm_bts_trx *trx, struct osmo_phsap_prim *l1sap)
{
	uint8_t *data;
	int len;
	uint8_t chan_type = 0, tn = 0, ss = 0;
	uint32_t fn;
	uint16_t uplink = GSMTAP_ARFCN_F_UPLINK;
	int rc;

	if (!gsmtap)
		return 0;

	switch (OSMO_PRIM_HDR(&l1sap->oph)) {
	case OSMO_PRIM(PRIM_PH_DATA, PRIM_OP_REQUEST):
		uplink = 0;
		/* fall through */
	case OSMO_PRIM(PRIM_PH_DATA, PRIM_OP_INDICATION):
		if (trx->ts[tn].pchan == GSM_PCHAN_PDCH)
			rc = gsmtap_pdch(l1sap, &chan_type, &tn, &ss, &fn, &data,
				&len);
		else
			rc = gsmtap_ph_data(l1sap, &chan_type, &tn, &ss, &fn,
				&data, &len);
		break;
	case OSMO_PRIM(PRIM_PH_RACH, PRIM_OP_INDICATION):
		rc = gsmtap_ph_rach(l1sap, &chan_type, &tn, &ss, &fn, &data,
			&len);
		break;
	default:
		rc = -ENOTSUP;
	}

	if (rc)
		return rc;

	if (len == 0)
		return 0;
	if ((chan_type & GSMTAP_CHANNEL_ACCH)) {
		if (!gsmtap_sapi_acch)
			return 0;
	} else {
		if (!((1 << (chan_type & 31)) & gsmtap_sapi_mask))
			return 0;
	}

	gsmtap_send(gsmtap, trx->arfcn | uplink, tn, chan_type, ss, fn, 0, 0,
		data, len);

	return 0;
}

/* time information received from bts model */
static int l1sap_info_time_ind(struct gsm_bts *bts,
	struct osmo_phsap_prim *l1sap,
	struct info_time_ind_param *info_time_ind)
{
	struct gsm_bts_trx *trx;
	struct gsm_bts_role_bts *btsb = bts->role;

	DEBUGP(DL1P, "MPH_INFO time ind %u\n", info_time_ind->fn);

	/* Update our data structures with the current GSM time */
	gsm_fn2gsmtime(&btsb->gsm_time, info_time_ind->fn);

	/* Update time on PCU interface */
	pcu_tx_time_ind(info_time_ind->fn);

	/* check if the measurement period of some lchan has ended
	 * and pre-compute the respective measurement */
	llist_for_each_entry(trx, &bts->trx_list, list)
		trx_meas_check_compute(trx, info_time_ind->fn - 1);

	/* increment 'total' for every possible rach */
	if (bts->c0->ts[0].pchan != GSM_PCHAN_CCCH_SDCCH4
	 || (info_time_ind->fn % 51) < 27)
		btsb->load.rach.total++;

	return 0;
}

/* measurement information received from bts model */
static int l1sap_info_meas_ind(struct gsm_bts_trx *trx,
	struct osmo_phsap_prim *l1sap,
	struct info_meas_ind_param *info_meas_ind)
{
	struct bts_ul_meas ulm;
	struct gsm_lchan *lchan;

	DEBUGP(DL1P, "MPH_INFO meas ind chan_nr=%02x\n",
		info_meas_ind->chan_nr);

	lchan = &trx->ts[L1SAP_CHAN2TS(info_meas_ind->chan_nr)]
				.lchan[l1sap_chan2ss(info_meas_ind->chan_nr)];

	/* in the GPRS case we are not interested in measurement
	 * processing.  The PCU will take care of it */
	if (lchan->type == GSM_LCHAN_PDTCH)
		return 0;

	memset(&ulm, 0, sizeof(ulm));
	ulm.ta_offs_qbits = info_meas_ind->ta_offs_qbits;
	ulm.ber10k = info_meas_ind->ber10k;
	ulm.inv_rssi = info_meas_ind->inv_rssi;

	lchan_new_ul_meas(lchan, &ulm);

	return 0;
}

/* any L1 MPH_INFO indication prim recevied from bts model */
static int l1sap_mph_info_ind(struct gsm_bts_trx *trx,
	 struct osmo_phsap_prim *l1sap, struct mph_info_param *info)
{
	int rc = 0;

	switch (info->type) {
	case PRIM_INFO_TIME:
		rc = l1sap_info_time_ind(trx->bts, l1sap, &info->u.time_ind);
		break;
	case PRIM_INFO_MEAS:
		rc = l1sap_info_meas_ind(trx, l1sap, &info->u.meas_ind);
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
	struct gsm_lchan *lchan;
	uint8_t chan_nr, link_id;
	uint8_t tn, ss;
	uint32_t fn;
	uint8_t *p, *si;
	struct lapdm_entity *le;
	struct osmo_phsap_prim pp;
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
	} else if (!(chan_nr & 0x80)) { /* only TCH/F, TCH/H, SDCCH/4 and SDCCH/8 have C5 bit cleared */
		if (L1SAP_IS_CHAN_TCHH(chan_nr))
			ss = L1SAP_CHAN2SS_TCHH(chan_nr); /* TCH/H */
		else if (L1SAP_IS_CHAN_SDCCH4(chan_nr))
			ss = L1SAP_CHAN2SS_SDCCH4(chan_nr); /* SDCCH/4 */
		else if (L1SAP_IS_CHAN_SDCCH8(chan_nr))
			ss = L1SAP_CHAN2SS_SDCCH8(chan_nr); /* SDCCH/8 */
		else
			ss = 0; /* TCH/F */
		lchan = &trx->ts[tn].lchan[ss];
		if (L1SAP_IS_LINK_SACCH(link_id)) {
			p = msgb_put(msg, GSM_MACBLOCK_LEN);
			/* L1-header, if not set/modified by layer 1 */
			p[0] = lchan->ms_power;
			p[1] = lchan->rqd_ta;
			le = &lchan->lapdm_ch.lapdm_acch;
		} else
			le = &lchan->lapdm_ch.lapdm_dcch;
		rc = lapdm_phsap_dequeue_prim(le, &pp);
		if (rc < 0) {
			if (L1SAP_IS_LINK_SACCH(link_id)) {
				/* No SACCH data from LAPDM pending, send SACCH filling */
				uint8_t *si = lchan_sacch_get(lchan, &g_time);
				if (si) {
					/* The +2 is empty space where the DSP inserts the L1 hdr */
					memcpy(p + 2, si, GSM_MACBLOCK_LEN - 2);
				} else
					memcpy(p + 2, fill_frame, GSM_MACBLOCK_LEN - 2);
			} else if ((!L1SAP_IS_CHAN_TCHF(chan_nr) && !L1SAP_IS_CHAN_TCHH(chan_nr))
				|| lchan->rsl_cmode == RSL_CMOD_SPD_SIGN) {
				/* send fill frame only, if not TCH/x != Signalling, otherwise send empty frame */
				p = msgb_put(msg, GSM_MACBLOCK_LEN);
				memcpy(p, fill_frame, GSM_MACBLOCK_LEN);
			} /* else the message remains empty, so TCH frames are sent */
		} else {
			/* The +2 is empty space where the DSP inserts the L1 hdr */
			if (L1SAP_IS_LINK_SACCH(link_id))
				memcpy(p + 2, pp.oph.msg->data + 2, GSM_MACBLOCK_LEN - 2);
			else {
				p = msgb_put(msg, GSM_MACBLOCK_LEN);
				memcpy(p, pp.oph.msg->data, GSM_MACBLOCK_LEN);
				/* check if it is a RR CIPH MODE CMD. if yes, enable RX ciphering */
				check_for_ciph_cmd(pp.oph.msg, lchan, chan_nr);
			}
			msgb_free(pp.oph.msg);
		}
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

/* TCH-RTS-IND prim recevied from bts model */
static int l1sap_tch_rts_ind(struct gsm_bts_trx *trx,
	struct osmo_phsap_prim *l1sap, struct ph_tch_param *rts_ind)
{
	struct msgb *resp_msg;
	struct osmo_phsap_prim *resp_l1sap, empty_l1sap;
	struct gsm_time g_time;
	struct gsm_lchan *lchan;
	uint8_t chan_nr;
	uint8_t tn, ss;
	uint32_t fn;

	chan_nr = rts_ind->chan_nr;
	fn = rts_ind->fn;
	tn = L1SAP_CHAN2TS(chan_nr);

	gsm_fn2gsmtime(&g_time, fn);

	DEBUGP(DL1P, "Rx TCH-RTS.ind %02u/%02u/%02u chan_nr=%d\n",
		g_time.t1, g_time.t2, g_time.t3, chan_nr);

	/* get timeslot and subslot */
	tn = L1SAP_CHAN2TS(chan_nr);
	if (L1SAP_IS_CHAN_TCHH(chan_nr))
		ss = L1SAP_CHAN2SS_TCHH(chan_nr); /* TCH/H */
	else
		ss = 0; /* TCH/F */
	lchan = &trx->ts[tn].lchan[ss];

	if (!lchan->loopback && lchan->abis_ip.rtp_socket) {
		osmo_rtp_socket_poll(lchan->abis_ip.rtp_socket);
		/* FIXME: we _assume_ that we never miss TDMA
		 * frames and that we always get to this point
		 * for every to-be-transmitted voice frame.  A
		 * better solution would be to compute
		 * rx_user_ts based on how many TDMA frames have
		 * elapsed since the last call */
		lchan->abis_ip.rtp_socket->rx_user_ts += GSM_RTP_DURATION;
	}
	/* get a msgb from the dl_tx_queue */
	resp_msg = msgb_dequeue(&lchan->dl_tch_queue);
	if (!resp_msg) {
		LOGP(DL1P, LOGL_DEBUG, "%s DL TCH Tx queue underrun\n",
			gsm_lchan_name(lchan));
		resp_l1sap = &empty_l1sap;
	} else {
		resp_msg->l2h = resp_msg->data;
		msgb_push(resp_msg, sizeof(*resp_l1sap));
		resp_msg->l1h = resp_msg->data;
		resp_l1sap = msgb_l1sap_prim(resp_msg);
	}

	memset(resp_l1sap, 0, sizeof(*resp_l1sap));
	osmo_prim_init(&resp_l1sap->oph, SAP_GSM_PH, PRIM_TCH, PRIM_OP_REQUEST,
		resp_msg);
	resp_l1sap->u.tch.chan_nr = chan_nr;
	resp_l1sap->u.tch.fn = fn;

	DEBUGP(DL1P, "Tx TCH.req %02u/%02u/%02u chan_nr=%d\n",
		g_time.t1, g_time.t2, g_time.t3, chan_nr);

	l1sap_down(trx, resp_l1sap);

	return 0;
}

/* process radio link timeout counter S */
static void radio_link_timeout(struct gsm_lchan *lchan, int bad_frame)
{
	struct gsm_bts_role_bts *btsb = lchan->ts->trx->bts->role;

	/* if link loss criterion already reached */
	if (lchan->s == 0) {
		DEBUGP(DMEAS, "%s radio link counter S already 0.\n",
			gsm_lchan_name(lchan));
		return;
	}

	if (bad_frame) {
		/* count down radio link counter S */
		lchan->s--;
		DEBUGP(DMEAS, "%s counting down radio link counter S=%d\n",
			gsm_lchan_name(lchan), lchan->s);
		if (lchan->s == 0)
			rsl_tx_conn_fail(lchan, RSL_ERR_RADIO_LINK_FAIL);
		return;
	}

	if (lchan->s < btsb->radio_link_timeout) {
		/* count up radio link counter S */
		lchan->s += 2;
		if (lchan->s > btsb->radio_link_timeout)
			lchan->s = btsb->radio_link_timeout;
		DEBUGP(DMEAS, "%s counting up radio link counter S=%d\n",
			gsm_lchan_name(lchan), lchan->s);
	}
}

/* DATA received from bts model */
static int l1sap_ph_data_ind(struct gsm_bts_trx *trx,
	 struct osmo_phsap_prim *l1sap, struct ph_data_param *data_ind)
{
	struct msgb *msg = l1sap->oph.msg;
	struct gsm_time g_time;
	struct gsm_lchan *lchan;
	struct lapdm_entity *le;
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

	lchan = &trx->ts[tn].lchan[ss];

	/* bad frame */
	if (len == 0) {
		if (L1SAP_IS_LINK_SACCH(link_id))
			radio_link_timeout(lchan, 1);
		return -EINVAL;
	}

	/* report first valid received frame to handover process */
	if (lchan->ho.active == HANDOVER_WAIT_FRAME)
		handover_frame(lchan);

	if (L1SAP_IS_LINK_SACCH(link_id)) {
		radio_link_timeout(lchan, 0);
		le = &lchan->lapdm_ch.lapdm_acch;
		/* save the SACCH L1 header in the lchan struct for RSL MEAS RES */
		if (len < 2) {
			LOGP(DL1P, LOGL_NOTICE, "SACCH with size %u<2 !?!\n",
				len);
			return -EINVAL;
		}
		/* Some brilliant engineer decided that the ordering of
		 * fields on the Um interface is different from the
		 * order of fields in RLS. See TS 04.04 (Chapter 7.2)
		 * vs. TS 08.58 (Chapter 9.3.10). */
		lchan->meas.l1_info[0] = data[0] << 3;
		lchan->meas.l1_info[0] |= ((data[0] >> 5) & 1) << 2;
		lchan->meas.l1_info[1] = data[1];
		lchan->meas.flags |= LC_UL_M_F_L1_VALID;
	} else
		le = &lchan->lapdm_ch.lapdm_dcch;

	/* if this is the first valid message after enabling Rx
	 * decryption, we have to enable Tx encryption */
	if (lchan->ciph_state == LCHAN_CIPH_RX_CONF) {
		/* HACK: check if it's an I frame, in order to
		 * ignore some still buffered/queued UI frames received
		 * before decryption was enabled */
		if (data[0] == 0x01 && (data[1] & 0x01) == 0) {
			l1sap_tx_ciph_req(trx, chan_nr, 1, 0);
		}
	}

	/* SDCCH, SACCH and FACCH all go to LAPDm */
	msgb_pull(msg, (msg->l2h - msg->data));
	msg->l1h = NULL;
	lapdm_phsap_up(&l1sap->oph, le);

	/* don't free, because we forwarded data */
	return 1;
}

/* TCH received from bts model */
static int l1sap_tch_ind(struct gsm_bts_trx *trx, struct osmo_phsap_prim *l1sap,
	struct ph_tch_param *tch_ind)
{
	struct msgb *msg = l1sap->oph.msg;
	struct gsm_time g_time;
	struct gsm_lchan *lchan;
	uint8_t tn, ss, chan_nr;
	uint32_t fn;

	chan_nr = tch_ind->chan_nr;
	fn = tch_ind->fn;
	tn = L1SAP_CHAN2TS(chan_nr);
	if (L1SAP_IS_CHAN_TCHH(chan_nr))
		ss = L1SAP_CHAN2SS_TCHH(chan_nr);
	else
		ss = 0;
	lchan = &trx->ts[tn].lchan[ss];

	gsm_fn2gsmtime(&g_time, fn);

	DEBUGP(DL1P, "Rx TCH.ind %02u/%02u/%02u chan_nr=%d\n",
		g_time.t1, g_time.t2, g_time.t3, chan_nr);

	msgb_pull(msg, sizeof(*l1sap));

	/* hand msg to RTP code for transmission */
	if (lchan->abis_ip.rtp_socket)
		osmo_rtp_send_frame(lchan->abis_ip.rtp_socket,
			msg->data, msg->len, 160);

	/* if loopback is enabled, also queue received RTP data */
	if (lchan->loopback) {
		struct msgb *tmp;
		int count = 0;

		 /* make sure the queue doesn't get too long */
		llist_for_each_entry(tmp, &lchan->dl_tch_queue, list)
		count++;
		while (count >= 1) {
			tmp = msgb_dequeue(&lchan->dl_tch_queue);
			msgb_free(tmp);
			count--;
		}

		msgb_enqueue(&lchan->dl_tch_queue, msg);
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
	case OSMO_PRIM(PRIM_TCH_RTS, PRIM_OP_INDICATION):
		rc = l1sap_tch_rts_ind(trx, l1sap, &l1sap->u.tch);
		break;
	case OSMO_PRIM(PRIM_PH_DATA, PRIM_OP_INDICATION):
		to_gsmtap(trx, l1sap);
		rc = l1sap_ph_data_ind(trx, l1sap, &l1sap->u.data);
		break;
	case OSMO_PRIM(PRIM_TCH, PRIM_OP_INDICATION):
		rc = l1sap_tch_ind(trx, l1sap, &l1sap->u.tch);
		break;
	case OSMO_PRIM(PRIM_PH_RACH, PRIM_OP_INDICATION):
		to_gsmtap(trx, l1sap);
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
	if (OSMO_PRIM_HDR(&l1sap->oph) ==
				 OSMO_PRIM(PRIM_PH_DATA, PRIM_OP_REQUEST))
		to_gsmtap(trx, l1sap);

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

/*! \brief call-back function for incoming RTP */
void l1sap_rtp_rx_cb(struct osmo_rtp_socket *rs, const uint8_t *rtp_pl,
                     unsigned int rtp_pl_len)
{
	struct gsm_lchan *lchan = rs->priv;
	struct msgb *msg, *tmp;
	struct osmo_phsap_prim *l1sap;
	int count = 0;

	msg = l1sap_msgb_alloc(rtp_pl_len);
	if (!msg)
		return;
	memcpy(msgb_put(msg, rtp_pl_len), rtp_pl, rtp_pl_len);
	msgb_pull(msg, sizeof(*l1sap));


	 /* make sure the queue doesn't get too long */
	llist_for_each_entry(tmp, &lchan->dl_tch_queue, list)
	count++;
	while (count >= 2) {
		tmp = msgb_dequeue(&lchan->dl_tch_queue);
		msgb_free(tmp);
		count--;
	}

	msgb_enqueue(&lchan->dl_tch_queue, msg);
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
