/* pcu_sock.c: Connect from PCU via unix domain socket */

/* (C) 2008-2019 by Harald Welte <laforge@gnumonks.org>
 * (C) 2009-2012 by Andreas Eversberg <jolly@eversberg.eu>
 * (C) 2012 by Holger Hans Peter Freyther
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <inttypes.h>

#include <osmocom/core/talloc.h>
#include <osmocom/core/utils.h>
#include <osmocom/core/select.h>
#include <osmocom/core/socket.h>
#include <osmocom/core/write_queue.h>
#include <osmocom/gsm/gsm23003.h>
#include <osmocom/gsm/abis_nm.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/gsm_data.h>
#include <osmo-bts/pcu_if.h>
#include <osmo-bts/pcuif_proto.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/bts_sm.h>
#include <osmo-bts/rsl.h>
#include <osmo-bts/signal.h>
#include <osmo-bts/l1sap.h>
#include <osmo-bts/oml.h>
#include <osmo-bts/abis_osmo.h>

uint32_t trx_get_hlayer1(const struct gsm_bts_trx *trx);

int pcu_direct = 0;
static int avail_lai = 0, avail_nse = 0, avail_cell = 0, avail_nsvc[2] = {0, 0};

static const char *sapi_string[] = {
	[PCU_IF_SAPI_RACH] =	"RACH",
	[PCU_IF_SAPI_BCCH] =	"BCCH",
	[PCU_IF_SAPI_PDTCH] =	"PDTCH",
	[PCU_IF_SAPI_PRACH] =	"PRACH",
	[PCU_IF_SAPI_PTCCH] = 	"PTCCH",
	[PCU_IF_SAPI_PCH_2] =	"PCH_2",
	[PCU_IF_SAPI_AGCH_2] =	"AGCH_2",
};

/*
 * PCU messages
 */

struct msgb *pcu_msgb_alloc(uint8_t msg_type, uint8_t bts_nr)
{
	struct msgb *msg;
	struct gsm_pcu_if *pcu_prim;

	msg = msgb_alloc(sizeof(struct gsm_pcu_if), "pcu_sock_tx");
	if (!msg)
		return NULL;
	msgb_put(msg, sizeof(struct gsm_pcu_if));
	pcu_prim = (struct gsm_pcu_if *) msg->data;
	pcu_prim->msg_type = msg_type;
	pcu_prim->bts_nr = bts_nr;

	return msg;
}

static bool ts_should_be_pdch(const struct gsm_bts_trx_ts *ts)
{
	switch (ts->pchan) {
	case GSM_PCHAN_PDCH:
		return true;
	case GSM_PCHAN_TCH_F_PDCH:
		/* When we're busy deactivating the PDCH, we first set
		 * DEACT_PENDING, tell the PCU about it and wait for a
		 * response. So DEACT_PENDING means "no PDCH" to the PCU.
		 * Similarly, when we're activating PDCH, we set the
		 * ACT_PENDING and wait for an activation response from the
		 * PCU, so ACT_PENDING means "is PDCH". */
		if (ts->flags & TS_F_PDCH_ACTIVE)
			return !(ts->flags & TS_F_PDCH_DEACT_PENDING);
		else
			return (ts->flags & TS_F_PDCH_ACT_PENDING);
	case GSM_PCHAN_OSMO_DYN:
		/*
		 * When we're busy de-/activating the PDCH, we first set
		 * ts->dyn.pchan_want, tell the PCU about it and wait for a
		 * response. To make it available to PCU, we want to make sure
		 * it's already configured by phy (pchan_is==PDCH) and that we
		 * are not in progress of removing it (pchan_want=None).
		 */

		return ts->dyn.pchan_is == GSM_PCHAN_PDCH && ts->dyn.pchan_want == GSM_PCHAN_PDCH;
	default:
		return false;
	}
}

/* As a BTS, we do not (and neither need to) know the Mobile Allocation, because
 * in CS domain it's responsibility of the BSC to encode RR messages containing
 * this IE.  However, a BTS co-located PCU needs to know all hopping parameters,
 * including the Mobile Allocation, because it's responsible for encoding of the
 * packet resource assignment messages.
 *
 * This function, similar to generate_ma_for_ts() in osmo-bsc, computes the
 * Mobile Allocation bit-mask and populates the given part of INFO.ind with
 * the hopping parameters for the given timeslot. */
static void info_ind_fill_fhp(struct gsm_pcu_if_info_trx_ts *ts_info,
			      const struct gsm_bts_trx_ts *ts)
{
	const struct gsm_bts *bts = ts->trx->bts;
	const struct gsm_bts_trx *trx;
	uint8_t ca_buf[1024 / 8] = { 0 };
	uint8_t sa_buf[1024 / 8] = { 0 };
	struct bitvec ca, sa, ma;
	unsigned int i;

	ts_info->maio = ts->hopping.maio;
	ts_info->hsn = ts->hopping.hsn;
	ts_info->hopping = 0x01;

	/* Cell Allocation bit-mask */
	ca = (struct bitvec) {
		.data_len = sizeof(ca_buf),
		.data = &ca_buf[0],
	};

	llist_for_each_entry(trx, &bts->trx_list, list) {
		/* Skip non-provisioned transceivers */
		if (trx->mo.nm_attr == NULL) {
			LOGPTRX(trx, DPCU, LOGL_NOTICE, "not (yet) provisioned\n");
			continue;
		}

		bitvec_set_bit_pos(&ca, trx->arfcn, ONE);
		ts_info->ma_bit_len++;
	}

	/* Slot Allocation bit-mask */
	sa = (struct bitvec) {
		.data_len = sizeof(sa_buf),
		.data = &sa_buf[0],
	};

	for (i = 0; i < ts->hopping.arfcn_num; i++) {
		bitvec_set_bit_pos(&sa, ts->hopping.arfcn_list[i], ONE);
		if (bitvec_get_bit_pos(&ca, ts->hopping.arfcn_list[i]) != ONE) {
			LOGP(DPCU, LOGL_NOTICE, "A transceiver with ARFCN %u "
			     "is not (yet) provisioned\n", ts->hopping.arfcn_list[i]);
			bitvec_set_bit_pos(&ca, ts->hopping.arfcn_list[i], ONE);
			ts_info->ma_bit_len++;
		}
	}

	/* Mobile Allocation bit-mask */
	ma = (struct bitvec) {
		.cur_bit = sizeof(ts_info->ma) * 8 - 1,
		.data_len = sizeof(ts_info->ma),
		.data = &ts_info->ma[0],
	};

	/* Skip ARFCN 0, it goes to the end of MA bit-mask */
	for (i = 1; i < sizeof(ca_buf) * 8; i++) {
		if (bitvec_get_bit_pos(&ca, i) != ONE)
			continue;
		if (bitvec_get_bit_pos(&sa, i) == ONE)
			bitvec_set_bit_pos(&ma, ma.cur_bit, ONE);
		ma.cur_bit--;
	}

	if (bitvec_get_bit_pos(&sa, 0) == ONE)
		bitvec_set_bit_pos(&ma, ma.cur_bit, ONE);
}

static void info_ind_fill_trx(struct gsm_pcu_if_info_trx *trx_info,
			      const struct gsm_bts_trx *trx)
{
	unsigned int tn;

	trx_info->pdch_mask = 0;
	trx_info->arfcn = trx->arfcn;
	trx_info->hlayer1 = trx_get_hlayer1(trx);

	if (trx->mo.nm_state.operational != NM_OPSTATE_ENABLED ||
	    trx->mo.nm_state.administrative != NM_STATE_UNLOCKED) {
		LOGPTRX(trx, DPCU, LOGL_INFO, "unavailable for PCU (op=%s adm=%s)\n",
		    abis_nm_opstate_name(trx->mo.nm_state.operational),
		    abis_nm_admin_name(trx->mo.nm_state.administrative));
		return;
	}

	for (tn = 0; tn < ARRAY_SIZE(trx->ts); tn++) {
		const struct gsm_bts_trx_ts *ts = &trx->ts[tn];

		if (ts->mo.nm_state.operational != NM_OPSTATE_ENABLED)
			continue;
		if (!ts_should_be_pdch(ts))
			continue;

		trx_info->pdch_mask |= (1 << tn);
		trx_info->ts[tn].tsc = ts->tsc;

		if (ts->hopping.enabled)
			info_ind_fill_fhp(&trx_info->ts[tn], ts);

		LOGPTRX(trx, DPCU, LOGL_INFO, "PDCH on ts=%u is available "
			"(tsc=%u ", ts->nr, trx_info->ts[tn].tsc);
		if (ts->hopping.enabled) {
			LOGPC(DPCU, LOGL_INFO, "hopping=yes hsn=%u maio=%u ma_bit_len=%u)\n",
			      ts->hopping.hsn, ts->hopping.maio, trx_info->ts[tn].ma_bit_len);
		} else {
			LOGPC(DPCU, LOGL_INFO, "hopping=no arfcn=%u)\n", trx->arfcn);
		}
	}
}

static enum gsm_pcuif_bts_model bts_model_from_variant(enum gsm_bts_type_variant variant)
{
	switch (variant) {
	case BTS_OSMO_LITECELL15:
		return PCU_IF_BTS_MODEL_LC15;
	case BTS_OSMO_OC2G:
		return PCU_IF_BTS_MODEL_OC2G;
	case BTS_OSMO_OCTPHY:
		return PCU_IF_BTS_MODEL_OCTPHY;
	case BTS_OSMO_SYSMO:
		return PCU_IF_BTS_MODEL_SYSMO;
	case BTS_OSMO_TRX:
	case BTS_OSMO_VIRTUAL:
		return PCU_IF_BTS_MODEL_TRX;
	default:
		return PCU_IF_BTS_MODEL_UNSPEC;
	}
}

int pcu_tx_info_ind(void)
{
	struct msgb *msg;
	struct gsm_pcu_if *pcu_prim;
	struct gsm_pcu_if_info_ind *info_ind;
	struct gsm_bts *bts;
	struct gprs_rlc_cfg *rlcc;
	struct gsm_bts_trx *trx;
	int i;
	struct gsm_gprs_nse *nse;

	LOGP(DPCU, LOGL_INFO, "Sending info\n");

	nse = &g_bts_sm->gprs.nse;
	/* FIXME: allow multiple BTS */
	bts = llist_entry(g_bts_sm->bts_list.next, struct gsm_bts, list);
	rlcc = &bts->gprs.cell.rlc_cfg;

	msg = pcu_msgb_alloc(PCU_IF_MSG_INFO_IND, bts->nr);
	if (!msg)
		return -ENOMEM;
	pcu_prim = (struct gsm_pcu_if *) msg->data;
	info_ind = &pcu_prim->u.info_ind;
	info_ind->version = PCU_IF_VERSION;

	if (avail_lai && avail_nse && avail_cell && avail_nsvc[0]) {
		info_ind->flags |= PCU_IF_FLAG_ACTIVE;
		LOGP(DPCU, LOGL_INFO, "BTS is up\n");
	} else
		LOGP(DPCU, LOGL_INFO, "BTS is down\n");

	if (pcu_direct)
		info_ind->flags |= PCU_IF_FLAG_DIRECT_PHY;

	info_ind->bsic = bts->bsic;
	/* RAI */
	info_ind->mcc = g_bts_sm->plmn.mcc;
	info_ind->mnc = g_bts_sm->plmn.mnc;
	info_ind->mnc_3_digits = g_bts_sm->plmn.mnc_3_digits;
	info_ind->lac = bts->location_area_code;
	info_ind->rac = bts->gprs.rac;

	/* NSE */
	info_ind->nsei = nse->nsei;
	memcpy(info_ind->nse_timer, nse->timer, 7);
	memcpy(info_ind->cell_timer, bts->gprs.cell.timer, 11);

	/* cell attributes */
	info_ind->cell_id = bts->cell_identity;
	info_ind->repeat_time = rlcc->paging.repeat_time;
	info_ind->repeat_count = rlcc->paging.repeat_count;
	info_ind->bvci = bts->gprs.cell.bvci;
	info_ind->t3142 = rlcc->parameter[RLC_T3142];
	info_ind->t3169 = rlcc->parameter[RLC_T3169];
	info_ind->t3191 = rlcc->parameter[RLC_T3191];
	info_ind->t3193_10ms = rlcc->parameter[RLC_T3193];
	info_ind->t3195 = rlcc->parameter[RLC_T3195];
	info_ind->n3101 = rlcc->parameter[RLC_N3101];
	info_ind->n3103 = rlcc->parameter[RLC_N3103];
	info_ind->n3105 = rlcc->parameter[RLC_N3105];
	info_ind->cv_countdown = rlcc->parameter[CV_COUNTDOWN];
	if (rlcc->cs_mask & (1 << GPRS_CS1))
		info_ind->flags |= PCU_IF_FLAG_CS1;
	if (rlcc->cs_mask & (1 << GPRS_CS2))
		info_ind->flags |= PCU_IF_FLAG_CS2;
	if (rlcc->cs_mask & (1 << GPRS_CS3))
		info_ind->flags |= PCU_IF_FLAG_CS3;
	if (rlcc->cs_mask & (1 << GPRS_CS4))
		info_ind->flags |= PCU_IF_FLAG_CS4;
	if (rlcc->cs_mask & (1 << GPRS_MCS1))
		info_ind->flags |= PCU_IF_FLAG_MCS1;
	if (rlcc->cs_mask & (1 << GPRS_MCS2))
		info_ind->flags |= PCU_IF_FLAG_MCS2;
	if (rlcc->cs_mask & (1 << GPRS_MCS3))
		info_ind->flags |= PCU_IF_FLAG_MCS3;
	if (rlcc->cs_mask & (1 << GPRS_MCS4))
		info_ind->flags |= PCU_IF_FLAG_MCS4;
	if (rlcc->cs_mask & (1 << GPRS_MCS5))
		info_ind->flags |= PCU_IF_FLAG_MCS5;
	if (rlcc->cs_mask & (1 << GPRS_MCS6))
		info_ind->flags |= PCU_IF_FLAG_MCS6;
	if (rlcc->cs_mask & (1 << GPRS_MCS7))
		info_ind->flags |= PCU_IF_FLAG_MCS7;
	if (rlcc->cs_mask & (1 << GPRS_MCS8))
		info_ind->flags |= PCU_IF_FLAG_MCS8;
	if (rlcc->cs_mask & (1 << GPRS_MCS9))
		info_ind->flags |= PCU_IF_FLAG_MCS9;
	/* FIXME: isn't dl_tbf_ext wrong?: * 10 and no ntohs */
	info_ind->dl_tbf_ext = rlcc->parameter[T_DL_TBF_EXT];
	/* FIXME: isn't ul_tbf_ext wrong?: * 10 and no ntohs */
	info_ind->ul_tbf_ext = rlcc->parameter[T_UL_TBF_EXT];
	info_ind->initial_cs = rlcc->initial_cs;
	info_ind->initial_mcs = rlcc->initial_mcs;

	/* NSVC */
	for (i = 0; i < ARRAY_SIZE(nse->nsvc); i++) {
		const struct gsm_gprs_nsvc *nsvc = &nse->nsvc[i];
		info_ind->nsvci[i] = nsvc->nsvci;
		/* PCUIF beauty: the NSVC addresses are sent in the network byte order,
		 * while the port numbers need to be send in the host order.  Sigh. */
		info_ind->local_port[i] = ntohs(nsvc->local.u.sin.sin_port);
		info_ind->remote_port[i] = ntohs(nsvc->remote.u.sin.sin_port);
		switch (nsvc->remote.u.sas.ss_family) {
		case AF_INET:
			info_ind->address_type[i] = PCU_IF_ADDR_TYPE_IPV4;
			info_ind->remote_ip[i].v4 = nsvc->remote.u.sin.sin_addr;
			break;
		case AF_INET6:
			info_ind->address_type[i] = PCU_IF_ADDR_TYPE_IPV6;
			info_ind->remote_ip[i].v6 = nsvc->remote.u.sin6.sin6_addr;
			break;
		default:
			info_ind->address_type[i] = PCU_IF_ADDR_TYPE_UNSPEC;
			break;
		}
	}

	llist_for_each_entry(trx, &bts->trx_list, list) {
		if (trx->nr >= ARRAY_SIZE(info_ind->trx)) {
			LOGPTRX(trx, DPCU, LOGL_NOTICE, "PCU interface (version %u) "
				"cannot handle more than %zu transceivers => skipped\n",
				PCU_IF_VERSION, ARRAY_SIZE(info_ind->trx));
			break;
		}

		info_ind_fill_trx(&info_ind->trx[trx->nr], trx);
	}

	info_ind->bts_model = bts_model_from_variant(bts->variant);

	return pcu_sock_send(msg);
}

static int pcu_if_signal_cb(unsigned int subsys, unsigned int signal,
	void *hdlr_data, void *signal_data)
{
	struct gsm_gprs_nsvc *nsvc;
	struct gsm_bts *bts;
	struct gsm48_system_information_type_3 *si3;
	int id;

	if (subsys != SS_GLOBAL)
		return -EINVAL;

	switch(signal) {
	case S_NEW_SYSINFO:
		bts = signal_data;
		if (!(bts->si_valid & (1 << SYSINFO_TYPE_3)))
			break;
		si3 = (struct gsm48_system_information_type_3 *)
						bts->si_buf[SYSINFO_TYPE_3];
		osmo_plmn_from_bcd(si3->lai.digits, &g_bts_sm->plmn);
		bts->location_area_code = ntohs(si3->lai.lac);
		bts->cell_identity = ntohs(si3->cell_identity);
		avail_lai = 1;
		break;
	case S_NEW_NSE_ATTR:
		bts = signal_data;
		avail_nse = 1;
		break;
	case S_NEW_CELL_ATTR:
		bts = signal_data;
		avail_cell = 1;
		break;
	case S_NEW_NSVC_ATTR:
		nsvc = signal_data;
		id = nsvc->id;
		if (id < 0 || id > 1)
			return -EINVAL;
		avail_nsvc[id] = 1;
		break;
	case S_NEW_OP_STATE:
		break;
	default:
		return -EINVAL;
	}

	/* Do not send INFO.ind if PCU is not connected */
	if (!pcu_connected())
		return 0;

	/* If all infos have been received, of if one info is updated after
	 * all infos have been received, transmit info update. */
	if (avail_lai && avail_nse && avail_cell && avail_nsvc[0])
		pcu_tx_info_ind();
	return 0;
}

int pcu_tx_app_info_req(struct gsm_bts *bts, uint8_t app_type, uint8_t len, const uint8_t *app_data)
{
	struct gsm_pcu_if_app_info_req *ai_req;
	struct gsm_pcu_if *pcu_prim;
	struct msgb *msg;

	if (app_type & 0xF0 || len > sizeof(ai_req->data))
		return -EINVAL;

	msg = pcu_msgb_alloc(PCU_IF_MSG_APP_INFO_REQ, bts->nr);
	if (!msg)
		return -ENOMEM;
	pcu_prim = (struct gsm_pcu_if *) msg->data;
	ai_req = &pcu_prim->u.app_info_req;

	ai_req->application_type = app_type;
	ai_req->len = len;
	memcpy(ai_req->data, app_data, ai_req->len);

	return pcu_sock_send(msg);
}

int pcu_tx_rts_req(struct gsm_bts_trx_ts *ts, uint8_t is_ptcch, uint32_t fn,
	uint16_t arfcn, uint8_t block_nr)
{
	struct msgb *msg;
	struct gsm_pcu_if *pcu_prim;
	struct gsm_pcu_if_rts_req *rts_req;
	struct gsm_bts *bts = ts->trx->bts;

	LOGP(DPCU, LOGL_DEBUG, "Sending rts request: is_ptcch=%d arfcn=%d "
		"block=%d\n", is_ptcch, arfcn, block_nr);

	msg = pcu_msgb_alloc(PCU_IF_MSG_RTS_REQ, bts->nr);
	if (!msg)
		return -ENOMEM;
	pcu_prim = (struct gsm_pcu_if *) msg->data;
	rts_req = &pcu_prim->u.rts_req;

	rts_req->sapi = (is_ptcch) ? PCU_IF_SAPI_PTCCH : PCU_IF_SAPI_PDTCH;
	rts_req->fn = fn;
	rts_req->arfcn = arfcn;
	rts_req->trx_nr = ts->trx->nr;
	rts_req->ts_nr = ts->nr;
	rts_req->block_nr = block_nr;

	return pcu_sock_send(msg);
}

int pcu_tx_data_ind(struct gsm_bts_trx_ts *ts, uint8_t sapi, uint32_t fn,
	uint16_t arfcn, uint8_t block_nr, uint8_t *data, uint8_t len,
	int8_t rssi, uint16_t ber10k, int16_t bto, int16_t lqual)
{
	struct msgb *msg;
	struct gsm_pcu_if *pcu_prim;
	struct gsm_pcu_if_data *data_ind;
	struct gsm_bts *bts = ts->trx->bts;

	LOGP(DPCU, LOGL_DEBUG, "Sending data indication: sapi=%s arfcn=%d block=%d data=%s\n",
	     sapi_string[sapi], arfcn, block_nr, osmo_hexdump(data, len));

	msg = pcu_msgb_alloc(PCU_IF_MSG_DATA_IND, bts->nr);
	if (!msg)
		return -ENOMEM;
	pcu_prim = (struct gsm_pcu_if *) msg->data;
	data_ind = &pcu_prim->u.data_ind;

	data_ind->sapi = sapi;
	data_ind->rssi = rssi;
	data_ind->fn = fn;
	data_ind->arfcn = arfcn;
	data_ind->trx_nr = ts->trx->nr;
	data_ind->ts_nr = ts->nr;
	data_ind->block_nr = block_nr;
	data_ind->ber10k = ber10k;
	data_ind->ta_offs_qbits = bto;
	data_ind->lqual_cb = lqual;
	if (len)
		memcpy(data_ind->data, data, len);
	data_ind->len = len;

	return pcu_sock_send(msg);
}

int pcu_tx_rach_ind(uint8_t bts_nr, uint8_t trx_nr, uint8_t ts_nr,
		    int16_t qta, uint16_t ra, uint32_t fn, uint8_t is_11bit,
		    enum ph_burst_type burst_type, uint8_t sapi)
{
	struct msgb *msg;
	struct gsm_pcu_if *pcu_prim;
	struct gsm_pcu_if_rach_ind *rach_ind;

	LOGP(DPCU, LOGL_INFO, "Sending RACH indication: qta=%d, ra=%d, "
		"fn=%d\n", qta, ra, fn);

	msg = pcu_msgb_alloc(PCU_IF_MSG_RACH_IND, bts_nr);
	if (!msg)
		return -ENOMEM;
	pcu_prim = (struct gsm_pcu_if *) msg->data;
	rach_ind = &pcu_prim->u.rach_ind;

	rach_ind->sapi = sapi;
	rach_ind->ra = ra;
	rach_ind->qta = qta;
	rach_ind->fn = fn;
	rach_ind->is_11bit = is_11bit;
	rach_ind->burst_type = burst_type;
	rach_ind->trx_nr = trx_nr;
	rach_ind->ts_nr = ts_nr;

	return pcu_sock_send(msg);
}

int pcu_tx_time_ind(uint32_t fn)
{
	struct msgb *msg;
	struct gsm_pcu_if *pcu_prim;
	struct gsm_pcu_if_time_ind *time_ind;
	uint8_t fn13 = fn % 13;

	/* omit frame numbers not starting at a MAC block */
	if (fn13 != 0 && fn13 != 4 && fn13 != 8)
		return 0;

	msg = pcu_msgb_alloc(PCU_IF_MSG_TIME_IND, 0);
	if (!msg)
		return -ENOMEM;
	pcu_prim = (struct gsm_pcu_if *) msg->data;
	time_ind = &pcu_prim->u.time_ind;

	time_ind->fn = fn;

	return pcu_sock_send(msg);
}

int pcu_tx_interf_ind(const struct gsm_bts_trx *trx, uint32_t fn)
{
	struct gsm_pcu_if_interf_ind *interf_ind;
	struct gsm_pcu_if *pcu_prim;
	struct msgb *msg;
	unsigned int tn;

	msg = pcu_msgb_alloc(PCU_IF_MSG_INTERF_IND, trx->bts->nr);
	if (!msg)
		return -ENOMEM;
	pcu_prim = (struct gsm_pcu_if *) msg->data;
	interf_ind = &pcu_prim->u.interf_ind;

	interf_ind->trx_nr = trx->nr;
	interf_ind->fn = fn;

	for (tn = 0; tn < ARRAY_SIZE(trx->ts); tn++) {
		const struct gsm_bts_trx_ts *ts = &trx->ts[tn];
		const struct gsm_lchan *lchan = &ts->lchan[0];

		if (ts->mo.nm_state.operational != NM_OPSTATE_ENABLED)
			continue;
		if (ts->mo.nm_state.availability != NM_AVSTATE_OK)
			continue;
		if (ts_pchan(ts) != GSM_PCHAN_PDCH)
			continue;

		interf_ind->interf[tn] = -1 * lchan->meas.interf_meas_avg_dbm;
	}

	return pcu_sock_send(msg);
}

int pcu_tx_pag_req(const uint8_t *identity_lv, uint8_t chan_needed)
{
	struct pcu_sock_state *state = g_bts_sm->gprs.pcu_state;
	struct msgb *msg;
	struct gsm_pcu_if *pcu_prim;
	struct gsm_pcu_if_pag_req *pag_req;

	/* check if identity does not fit: length > sizeof(lv) - 1 */
	if (identity_lv[0] >= sizeof(pag_req->identity_lv)) {
		LOGP(DPCU, LOGL_ERROR, "Paging identity too large (%d)\n",
			identity_lv[0]);
		return -EINVAL;
	}

	/* socket not created */
	if (!state) {
		LOGP(DPCU, LOGL_DEBUG, "PCU socket not created, ignoring "
			"paging message\n");
		return 0;
	}

	msg = pcu_msgb_alloc(PCU_IF_MSG_PAG_REQ, 0);
	if (!msg)
		return -ENOMEM;
	pcu_prim = (struct gsm_pcu_if *) msg->data;
	pag_req = &pcu_prim->u.pag_req;

	pag_req->chan_needed = chan_needed;
	memcpy(pag_req->identity_lv, identity_lv, identity_lv[0] + 1);

	return pcu_sock_send(msg);
}

int pcu_tx_data_cnf(uint32_t msg_id, uint8_t sapi)
{
	struct gsm_bts *bts;
	struct msgb *msg;
	struct gsm_pcu_if *pcu_prim;

	/* FIXME: allow multiple BTS */
	bts = llist_entry(g_bts_sm->bts_list.next, struct gsm_bts, list);

	LOGP(DPCU, LOGL_DEBUG, "Sending DATA.cnf: sapi=%s msg_id=%08x\n",
	     sapi_string[sapi], msg_id);

	msg = pcu_msgb_alloc(PCU_IF_MSG_DATA_CNF_2, bts->nr);
	if (!msg)
		return -ENOMEM;
	pcu_prim = (struct gsm_pcu_if *) msg->data;
	pcu_prim->u.data_cnf2 = (struct gsm_pcu_if_data_cnf) {
		.sapi = sapi,
		.msg_id = msg_id,
	};

	return pcu_sock_send(msg);
}

/* forward data from a RR GPRS SUSPEND REQ towards PCU */
int pcu_tx_susp_req(struct gsm_lchan *lchan, uint32_t tlli, const uint8_t *ra_id, uint8_t cause)
{
	struct msgb *msg;
	struct gsm_pcu_if *pcu_prim;

	msg = pcu_msgb_alloc(PCU_IF_MSG_SUSP_REQ, lchan->ts->trx->bts->nr);
	if (!msg)
		return -ENOMEM;
	pcu_prim = (struct gsm_pcu_if *) msg->data;
	pcu_prim->u.susp_req.tlli = tlli;
	memcpy(pcu_prim->u.susp_req.ra_id, ra_id, sizeof(pcu_prim->u.susp_req.ra_id));
	pcu_prim->u.susp_req.cause = cause;

	return pcu_sock_send(msg);
}

static int pcu_rx_data_req(struct gsm_bts *bts, uint8_t msg_type,
	const struct gsm_pcu_if_data *data_req)
{
	uint8_t is_ptcch;
	struct gsm_bts_trx *trx;
	struct gsm_bts_trx_ts *ts;
	struct msgb *msg;
	int rc = 0;

	LOGP(DPCU, LOGL_DEBUG, "Data request received: sapi=%s arfcn=%d "
		"block=%d data=%s\n", sapi_string[data_req->sapi],
		data_req->arfcn, data_req->block_nr,
		osmo_hexdump(data_req->data, data_req->len));

	switch (data_req->sapi) {
	case PCU_IF_SAPI_PCH_2:
	{
		const struct gsm_pcu_if_pch *gsm_pcu_if_pch;

		if (OSMO_UNLIKELY(data_req->len != sizeof(*gsm_pcu_if_pch))) {
			LOGP(DPCU, LOGL_ERROR, "Rx malformed DATA.req for PCH\n");
			rc = -EINVAL;
			break;
		}

		gsm_pcu_if_pch = (struct gsm_pcu_if_pch *)data_req->data;
		rc = paging_add_macblock(bts->paging_state, gsm_pcu_if_pch->msg_id,
					 gsm_pcu_if_pch->imsi, gsm_pcu_if_pch->confirm, gsm_pcu_if_pch->data);
		break;
	}
	case PCU_IF_SAPI_AGCH_2:
	{
		const struct gsm_pcu_if_agch *gsm_pcu_if_agch;
		struct bts_agch_msg_cb *msg_cb;

		gsm_pcu_if_agch = (struct gsm_pcu_if_agch *)data_req->data;

		msg = msgb_alloc(GSM_MACBLOCK_LEN, "pcu_agch");
		if (!msg) {
			rc = -ENOMEM;
			break;
		}
		msg->l3h = msgb_put(msg, GSM_MACBLOCK_LEN);
		memcpy(msg->l3h, gsm_pcu_if_agch->data, GSM_MACBLOCK_LEN);

		msg_cb = (struct bts_agch_msg_cb *) msg->cb;
		msg_cb->confirm = gsm_pcu_if_agch->confirm;
		msg_cb->msg_id = gsm_pcu_if_agch->msg_id;
		if (bts_agch_enqueue(bts, msg) < 0) {
			msgb_free(msg);
			rc = -EIO;
		}
		break;
	}
	case PCU_IF_SAPI_PDTCH:
	case PCU_IF_SAPI_PTCCH:
		trx = gsm_bts_trx_num(bts, data_req->trx_nr);
		if (!trx) {
			LOGP(DPCU, LOGL_ERROR, "Received PCU data request with "
				"not existing TRX %d\n", data_req->trx_nr);
			rc = -EINVAL;
			break;
		}
		if (data_req->ts_nr >= ARRAY_SIZE(trx->ts)) {
			LOGP(DPCU, LOGL_ERROR, "Received PCU data request with "
				"not existing TS %u\n", data_req->ts_nr);
			rc = -EINVAL;
			break;
		}
		ts = &trx->ts[data_req->ts_nr];
		if (!ts_should_be_pdch(ts)) {
			LOGP(DPCU, LOGL_ERROR, "%s: Received PCU DATA request for non-PDCH TS\n",
				gsm_ts_name(ts));
			rc = -EINVAL;
			break;
		}
		if (ts->lchan[0].state != LCHAN_S_ACTIVE) {
			LOGP(DPCU, LOGL_ERROR, "%s: Received PCU DATA request for inactive lchan\n",
				gsm_ts_name(ts));
			rc = -EINVAL;
			break;
		}
		is_ptcch = (data_req->sapi == PCU_IF_SAPI_PTCCH);
		rc = l1sap_pdch_req(ts, is_ptcch, data_req->fn, data_req->arfcn,
			data_req->block_nr, data_req->data, data_req->len);
		break;
	default:
		LOGP(DPCU, LOGL_ERROR, "Received PCU data request with "
			"unsupported sapi %d\n", data_req->sapi);
		rc = -EINVAL;
	}

	return rc;
}

static int pcu_rx_pag_req(struct gsm_bts *bts, uint8_t msg_type,
	const struct gsm_pcu_if_pag_req *pag_req)
{
	int rc = 0;

	OSMO_ASSERT(msg_type == PCU_IF_MSG_PAG_REQ);

	/* FIXME: Add function to schedule paging request.
	 * At present, osmo-pcu sends paging requests in PCU_IF_MSG_DATA_REQ
	 * messages which are processed by pcu_rx_data_req().
	 * This code path is not triggered in practice. */
	LOGP(DPCU, LOGL_NOTICE, "Paging request received: chan_needed=%d length=%d "
	     "(dropping message because support for PCU_IF_MSG_PAG_REQ is not yet implemented)\n",
	     pag_req->chan_needed, pag_req->identity_lv[0]);

	return rc;
}

int pcu_tx_si(const struct gsm_bts *bts, enum osmo_sysinfo_type si_type,
	      bool enable)
{
	/* the SI is per-BTS so it doesn't matter which TRX we use */
	struct gsm_bts_trx *trx = gsm_bts_trx_num(bts, 0);

	uint8_t si_buf[GSM_MACBLOCK_LEN];
	uint8_t len;
	int rc;

	if (enable) {
		memcpy(si_buf, GSM_BTS_SI(bts, si_type), GSM_MACBLOCK_LEN);
		len = GSM_MACBLOCK_LEN;
		LOGP(DPCU, LOGL_DEBUG, "Updating SI%s to PCU: %s\n",
		     get_value_string(osmo_sitype_strs, si_type),
		     osmo_hexdump_nospc(si_buf, GSM_MACBLOCK_LEN));
	} else {
		si_buf[0] = si_type;
		len = 1;

		/* Note: SI13 is the only system information type that is revked
		 * by just sending a completely empty message. This is due to
		 * historical reasons */
		if (si_type != SYSINFO_TYPE_13)
			len = 0;

		LOGP(DPCU, LOGL_DEBUG, "Revoking SI%s from PCU\n",
		     get_value_string(osmo_sitype_strs, si_buf[0]));
	}

	/* The low-level data like FN, ARFCN etc will be ignored but we have to
	 * set lqual high enough to bypass the check at lower levels */
	rc = pcu_tx_data_ind(&trx->ts[0], PCU_IF_SAPI_BCCH, 0, 0, 0, si_buf, len,
			     0, 0, 0, INT16_MAX);
	if (rc < 0)
		LOGP(DPCU, LOGL_NOTICE, "Failed to send SI%s to PCU: rc=%d\n",
		     get_value_string(osmo_sitype_strs, si_type), rc);

	return rc;
}

static int pcu_tx_si_all(struct gsm_bts *bts)
{
	const enum osmo_sysinfo_type si_types[] =
	    { SYSINFO_TYPE_1, SYSINFO_TYPE_2, SYSINFO_TYPE_3, SYSINFO_TYPE_13 };
	unsigned int i;
	int rc = 0;

	for (i = 0; i < ARRAY_SIZE(si_types); i++) {
		if (GSM_BTS_HAS_SI(bts, si_types[i])) {
			rc = pcu_tx_si(bts, si_types[i], true);
			if (rc < 0)
				return rc;
		} else {
			LOGP(DPCU, LOGL_INFO,
			     "SI%s is not available on PCU connection\n",
			     get_value_string(osmo_sitype_strs, si_types[i]));
		}
	}

	return 0;
}

static int pcu_rx_txt_ind(struct gsm_bts *bts,
			  struct gsm_pcu_if_txt_ind *txt)
{
	int rc = 0;

	switch (txt->type) {
	case PCU_VERSION:
		LOGP(DPCU, LOGL_INFO, "OsmoPCU version %s connected\n",
		     txt->text);

		/* we use the reception of the PCU_VERSION as a trigger to make the PCU available for
		 * all BTSs handled by this process (currently this is exactly one BTS, see FIXME notes) */
		llist_for_each_entry(bts, &g_bts_sm->bts_list, list) {
			oml_tx_failure_event_rep(&bts->gprs.cell.mo, NM_SEVER_CEASED, OSMO_EVT_PCU_VERS, txt->text);
			osmo_strlcpy(bts->pcu_version, txt->text, MAX_VERSION_LENGTH);

			/* patch SI to advertise GPRS, *if* the SI sent by BSC said so */
			regenerate_si3_restoctets(bts);
			regenerate_si4_restoctets(bts);

			if (pcu_tx_si_all(bts) < 0)
				rc = -EINVAL;
		}
		if (rc < 0)
			return rc;
		break;
	case PCU_OML_ALERT:
		OSMO_ASSERT(bts);
		oml_tx_failure_event_rep(&bts->gprs.cell.mo, NM_SEVER_INDETERMINATE, OSMO_EVT_EXT_ALARM,
					 txt->text);
		break;
	default:
		LOGP(DPCU, LOGL_ERROR, "Unknown TXT_IND type %u received\n",
		     txt->type);
		return -EINVAL;
	}

	return 0;
}

static int pcu_rx_act_req(struct gsm_bts *bts,
	const struct gsm_pcu_if_act_req *act_req)
{
	struct gsm_bts_trx *trx;
	struct gsm_lchan *lchan;

	LOGP(DPCU, LOGL_INFO, "%s request received: TRX=%d TS=%d\n",
		(act_req->activate) ? "Activate" : "Deactivate",
		act_req->trx_nr, act_req->ts_nr);

	trx = gsm_bts_trx_num(bts, act_req->trx_nr);
	if (!trx || act_req->ts_nr >= 8)
		return -EINVAL;

	lchan = trx->ts[act_req->ts_nr].lchan;
	lchan->rel_act_kind = LCHAN_REL_ACT_PCU;
	if (lchan->type != GSM_LCHAN_PDTCH) {
		LOGP(DPCU, LOGL_ERROR,
		     "%s request, but lchan is not of type PDTCH (is %s)\n",
		     (act_req->activate) ? "Activate" : "Deactivate",
		     gsm_lchant_name(lchan->type));
		return -EINVAL;
	}
	if (lchan->ts->pchan == GSM_PCHAN_OSMO_DYN &&
	    lchan->ts->dyn.pchan_is != GSM_PCHAN_PDCH) {
		LOGP(DPCU, LOGL_ERROR,
		     "%s request, but lchan in dyn TS is not configured as PDCH in lower layers (is %s)\n",
		     (act_req->activate) ? "Activate" : "Deactivate",
		     gsm_pchan_name(lchan->ts->dyn.pchan_is));
		return -EINVAL;
	}
	if (act_req->activate)
		l1sap_chan_act(trx, gsm_lchan2chan_nr(lchan));
	else
		l1sap_chan_rel(trx, gsm_lchan2chan_nr(lchan));

	return 0;
}

#define CHECK_IF_MSG_SIZE(prim_len, prim_msg) \
	do { \
		size_t _len = PCUIF_HDR_SIZE + sizeof(prim_msg); \
		if (prim_len < _len) { \
			LOGP(DPCU, LOGL_ERROR, "Received %zu bytes on PCU Socket, but primitive %s " \
			     "size is %zu, discarding\n", prim_len, #prim_msg, _len); \
			return -EINVAL; \
		} \
	} while (0)

#define ENSURE_BTS_OBJECT(bts) \
	do { \
		if ((bts = gsm_bts_num(g_bts_sm, pcu_prim->bts_nr)) == NULL) { \
			LOGP(DPCU, LOGL_ERROR, "Received PCU Prim for non-existent BTS %u\n", pcu_prim->bts_nr); \
			return -EINVAL; \
		} \
	} while (0)

static int pcu_rx(uint8_t msg_type, struct gsm_pcu_if *pcu_prim, size_t prim_len)
{
	int rc = 0;
	struct gsm_bts *bts;
	size_t exp_len;

	switch (msg_type) {
	case PCU_IF_MSG_DATA_REQ:
		CHECK_IF_MSG_SIZE(prim_len, pcu_prim->u.data_req);
		ENSURE_BTS_OBJECT(bts);
		rc = pcu_rx_data_req(bts, msg_type, &pcu_prim->u.data_req);
		break;
	case PCU_IF_MSG_PAG_REQ:
		CHECK_IF_MSG_SIZE(prim_len, pcu_prim->u.pag_req);
		ENSURE_BTS_OBJECT(bts);
		rc = pcu_rx_pag_req(bts, msg_type, &pcu_prim->u.pag_req);
		break;
	case PCU_IF_MSG_ACT_REQ:
		CHECK_IF_MSG_SIZE(prim_len, pcu_prim->u.act_req);
		ENSURE_BTS_OBJECT(bts);
		rc = pcu_rx_act_req(bts, &pcu_prim->u.act_req);
		break;
	case PCU_IF_MSG_TXT_IND:
		CHECK_IF_MSG_SIZE(prim_len, pcu_prim->u.txt_ind);
		if (pcu_prim->u.txt_ind.type == PCU_VERSION) {
			/* A TXT indication that carries the PCU_VERSION is always addressed to the
			 * receiving process as a whole, which means we will not resolve a specific
			 * BTS object in this case. */
			rc = pcu_rx_txt_ind(NULL, &pcu_prim->u.txt_ind);
		} else {
			ENSURE_BTS_OBJECT(bts);
			rc = pcu_rx_txt_ind(bts, &pcu_prim->u.txt_ind);
		}
		break;
	case PCU_IF_MSG_CONTAINER:
		CHECK_IF_MSG_SIZE(prim_len, pcu_prim->u.container);
		ENSURE_BTS_OBJECT(bts);
		/* ^ check if we can access container fields, v check with container data length */
		exp_len = PCUIF_HDR_SIZE + sizeof(pcu_prim->u.container) + osmo_load16be(&pcu_prim->u.container.length);
		if (prim_len < exp_len) {
			LOGP(DPCU, LOGL_ERROR, "Received %zu bytes on PCU Socket, but primitive "
			     "container size is %zu, discarding\n", prim_len, exp_len);
			return -EINVAL;
		}
		rc = abis_osmo_pcu_tx_container(bts, &pcu_prim->u.container);
		break;
	default:
		LOGP(DPCU, LOGL_ERROR, "Received unknown PCU msg type %d\n",
			msg_type);
		rc = -EINVAL;
	}

	return rc;
}

/*
 * PCU socket interface
 */

struct pcu_sock_state {
	struct osmo_fd listen_bfd;	/* fd for listen socket */
	struct osmo_wqueue upqueue;	/* For sending messages; has fd for conn. to PCU */
};

static void pcu_sock_close(struct pcu_sock_state *state);

int pcu_sock_send(struct msgb *msg)
{
	struct pcu_sock_state *state = g_bts_sm->gprs.pcu_state;
	struct osmo_fd *conn_bfd;
	struct gsm_pcu_if *pcu_prim = (struct gsm_pcu_if *) msg->data;
	int rc;

	if (!state) {
		if (pcu_prim->msg_type != PCU_IF_MSG_TIME_IND &&
		    pcu_prim->msg_type != PCU_IF_MSG_INTERF_IND)
			LOGP(DPCU, LOGL_INFO, "PCU socket not created, "
				"dropping message\n");
		msgb_free(msg);
		return -EINVAL;
	}
	conn_bfd = &state->upqueue.bfd;
	if (conn_bfd->fd <= 0) {
		if (pcu_prim->msg_type != PCU_IF_MSG_TIME_IND &&
		    pcu_prim->msg_type != PCU_IF_MSG_INTERF_IND)
			LOGP(DPCU, LOGL_NOTICE, "PCU socket not connected, "
				"dropping message\n");
		msgb_free(msg);
		return -EIO;
	}

	rc = osmo_wqueue_enqueue(&state->upqueue, msg);
	if (rc < 0) {
		if (rc == -ENOSPC)
			LOGP(DPCU, LOGL_NOTICE, "PCU not reacting (more than %u messages waiting). Closing connection\n",
			     state->upqueue.max_length);
		pcu_sock_close(state);
		msgb_free(msg);
		return rc;
	}
	return 0;
}

static void pcu_sock_close(struct pcu_sock_state *state)
{
	struct osmo_fd *bfd = &state->upqueue.bfd;
	struct gsm_bts *bts;
	struct gsm_bts_trx *trx;
	unsigned int tn;

	/* FIXME: allow multiple BTS */
	bts = llist_entry(g_bts_sm->bts_list.next, struct gsm_bts, list);

	LOGP(DPCU, LOGL_NOTICE, "PCU socket has LOST connection\n");
	oml_tx_failure_event_rep(&bts->gprs.cell.mo, NM_SEVER_MAJOR, OSMO_EVT_PCU_VERS,
				 "PCU socket has LOST connection");

	bts->pcu_version[0] = '\0';

	osmo_fd_unregister(bfd);
	close(bfd->fd);
	bfd->fd = -1;

	/* patch SI3 to remove GPRS indicator */
	regenerate_si3_restoctets(bts);
	regenerate_si4_restoctets(bts);

	/* re-enable the generation of ACCEPT for new connections */
	osmo_fd_read_enable(&state->listen_bfd);

#if 0
	/* remove si13, ... */
	bts->si_valid &= ~(1 << SYSINFO_TYPE_13);
	osmo_signal_dispatch(SS_GLOBAL, S_NEW_SYSINFO, bts);
#endif

	/* Deactivate all active PDCH timeslots */
	llist_for_each_entry(trx, &bts->trx_list, list) {
		for (tn = 0; tn < 8; tn++) {
			struct gsm_bts_trx_ts *ts = &trx->ts[tn];

			if (ts->mo.nm_state.operational != NM_OPSTATE_ENABLED)
				continue;
			if (!ts_should_be_pdch(ts))
				continue;

			ts->lchan[0].rel_act_kind = LCHAN_REL_ACT_PCU;
			l1sap_chan_rel(trx, gsm_lchan2chan_nr(&ts->lchan[0]));
		}
	}

	osmo_wqueue_clear(&state->upqueue);
}

static int pcu_sock_read(struct osmo_fd *bfd)
{
	struct pcu_sock_state *state = (struct pcu_sock_state *)bfd->data;
	struct gsm_pcu_if *pcu_prim;
	struct msgb *msg;
	int rc;

	msg = msgb_alloc(sizeof(*pcu_prim) + 1000, "pcu_sock_rx");
	if (!msg)
		return -ENOMEM;

	pcu_prim = (struct gsm_pcu_if *) msg->tail;

	rc = recv(bfd->fd, msg->tail, msgb_tailroom(msg), 0);
	if (rc == 0)
		goto close;

	if (rc < 0) {
		if (errno == EAGAIN) {
			msgb_free(msg);
			return 0;
		}
		goto close;
	}

	if (rc < PCUIF_HDR_SIZE) {
		LOGP(DPCU, LOGL_ERROR, "Received %d bytes on PCU Socket, but primitive hdr size "
		     "is %zu, discarding\n", rc, PCUIF_HDR_SIZE);
		msgb_free(msg);
		return 0;
	}

	rc = pcu_rx(pcu_prim->msg_type, pcu_prim, rc);

	/* as we always synchronously process the message in pcu_rx() and
	 * its callbacks, we can free the message here. */
	msgb_free(msg);

	return rc;

close:
	msgb_free(msg);
	pcu_sock_close(state);
	return -1;
}

static int pcu_sock_write(struct osmo_fd *bfd, struct msgb *msg)
{
	struct pcu_sock_state *state = bfd->data;
	int rc;

	/* bug hunter 8-): maybe someone forgot msgb_put(...) ? */
	OSMO_ASSERT(msgb_length(msg) > 0);
	/* try to send it over the socket */
	rc = write(bfd->fd, msgb_data(msg), msgb_length(msg));
	if (OSMO_UNLIKELY(rc == 0))
		goto close;
	if (OSMO_UNLIKELY(rc < 0)) {
		if (errno == EAGAIN)
			return -EAGAIN;
		return -1;
	}
	return 0;

close:
	pcu_sock_close(state);
	return -1;
}

/* accept connection coming from PCU */
static int pcu_sock_accept(struct osmo_fd *bfd, unsigned int flags)
{
	struct pcu_sock_state *state = (struct pcu_sock_state *)bfd->data;
	struct osmo_fd *conn_bfd = &state->upqueue.bfd;
	struct sockaddr_un un_addr;
	socklen_t len;
	int fd;

	len = sizeof(un_addr);
	fd = accept(bfd->fd, (struct sockaddr *)&un_addr, &len);
	if (fd < 0) {
		LOGP(DPCU, LOGL_ERROR, "Failed to accept a new connection\n");
		return -1;
	}

	if (conn_bfd->fd >= 0) {
		LOGP(DPCU, LOGL_NOTICE, "PCU connects but we already have another active connection ?!?\n");
		/* We already have one PCU connected, this is all we support */
		osmo_fd_read_disable(&state->listen_bfd);
		close(fd);
		return 0;
	}

	osmo_fd_setup(conn_bfd, fd, OSMO_FD_READ, osmo_wqueue_bfd_cb, state, 0);

	if (osmo_fd_register(conn_bfd) != 0) {
		LOGP(DPCU, LOGL_ERROR, "Failed to register new connection fd\n");
		close(conn_bfd->fd);
		conn_bfd->fd = -1;
		return -1;
	}

	LOGP(DPCU, LOGL_NOTICE, "PCU socket connected to external PCU\n");

	/* send current info */
	pcu_tx_info_ind();

	return 0;
}

int pcu_sock_init(const char *path, int qlength_max)
{
	struct pcu_sock_state *state;
	struct osmo_fd *bfd;
	int rc;

	state = talloc_zero(g_bts_sm, struct pcu_sock_state);
	if (!state)
		return -ENOMEM;

	osmo_wqueue_init(&state->upqueue, qlength_max);
	state->upqueue.read_cb = pcu_sock_read;
	state->upqueue.write_cb = pcu_sock_write;
	state->upqueue.bfd.fd = -1;

	bfd = &state->listen_bfd;

	rc = osmo_sock_unix_init(SOCK_SEQPACKET, 0, path, OSMO_SOCK_F_BIND);
	if (rc < 0) {
		LOGP(DPCU, LOGL_ERROR, "Could not create %s unix socket: %s\n",
		     path, strerror(errno));
		talloc_free(state);
		return -1;
	}

	osmo_fd_setup(bfd, rc, OSMO_FD_READ, pcu_sock_accept, state, 0);

	rc = osmo_fd_register(bfd);
	if (rc < 0) {
		LOGP(DPCU, LOGL_ERROR, "Could not register listen fd: %d\n",
			rc);
		close(bfd->fd);
		talloc_free(state);
		return rc;
	}

	osmo_signal_register_handler(SS_GLOBAL, pcu_if_signal_cb, NULL);

	g_bts_sm->gprs.pcu_state = state;

	LOGP(DPCU, LOGL_INFO, "Started listening on PCU socket (PCU IF v%u): %s\n", PCU_IF_VERSION, path);

	return 0;
}

void pcu_sock_exit(void)
{
	struct pcu_sock_state *state = g_bts_sm->gprs.pcu_state;
	struct osmo_fd *bfd, *conn_bfd;

	if (!state)
		return;

	osmo_signal_unregister_handler(SS_GLOBAL, pcu_if_signal_cb, NULL);
	conn_bfd = &state->upqueue.bfd;
	if (conn_bfd->fd > 0)
		pcu_sock_close(state);
	bfd = &state->listen_bfd;
	close(bfd->fd);
	osmo_fd_unregister(bfd);
	talloc_free(state);
	g_bts_sm->gprs.pcu_state = NULL;
}

bool pcu_connected(void) {
	struct pcu_sock_state *state = g_bts_sm->gprs.pcu_state;

	if (!state)
		return false;
	if (state->upqueue.bfd.fd <= 0)
		return false;
	return true;
}
