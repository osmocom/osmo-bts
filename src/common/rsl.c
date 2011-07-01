/* GSM TS 08.58 RSL, BTS Side */

/* (C) 2011 by Andreas Eversberg <jolly@eversberg.eu>
 * (C) 2011 by Harald Welte <laforge@gnumonks.org>
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

#include <stdio.h>
#include <errno.h>
#include <sys/types.h>
#include <arpa/inet.h>

#include <osmocom/core/msgb.h>
#include <osmocom/gsm/rsl.h>
#include <osmocom/gsm/lapdm.h>
#include <osmocom/gsm/protocol/gsm_12_21.h>

#include <osmo-bts/logging.h>
#include <osmo-bts/gsm_data.h>
#include <osmo-bts/abis.h>
#include <osmo-bts/rtp.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/rsl.h>
#include <osmo-bts/oml.h>
#include <osmo-bts/signal.h>
#include <osmo-bts/bts_model.h>
#include <osmo-bts/measurement.h>

static int rsl_tx_error_report(struct gsm_bts_trx *trx, uint8_t cause);

/* list of RSL SI types that can occur on the SACCH */
static const unsigned int rsl_sacch_sitypes[] = {
	RSL_SYSTEM_INFO_5,
	RSL_SYSTEM_INFO_6,
	RSL_SYSTEM_INFO_5bis,
	RSL_SYSTEM_INFO_5ter,
	RSL_EXT_MEAS_ORDER,
	RSL_MEAS_INFO,
};

/* FIXME: move this to libosmocore */
int osmo_in_array(unsigned int search, const unsigned int *arr, unsigned int size)
{
	unsigned int i;
	for (i = 0; i < size; i++) {
		if (arr[i] == search)
			return 1;
	}
	return 0;
}
#define OSMO_IN_ARRAY(search, arr) osmo_in_array(search, arr, ARRAY_SIZE(arr))

/* FIXME: move this to libosmocore */
void gsm48_gen_starting_time(uint8_t *out, struct gsm_time *gtime)
{
	uint8_t t1p = gtime->t1 % 32;
	out[0] = (t1p << 3) | (gtime->t3 >> 3);
	out[1] = (gtime->t3 << 5) | gtime->t2;
}



/*
 * support
 */


#warning merge lchan_lookup with OpenBSC
/* determine logical channel based on TRX and channel number IE */
struct gsm_lchan *rsl_lchan_lookup(struct gsm_bts_trx *trx, uint8_t chan_nr)
{
	struct gsm_lchan *lchan;
	uint8_t ts_nr = chan_nr & 0x07;
	uint8_t cbits = chan_nr >> 3;
	uint8_t lch_idx;
	struct gsm_bts_trx_ts *ts = &trx->ts[ts_nr];

	if (cbits == 0x01) {
		lch_idx = 0;	/* TCH/F */	
		if (ts->pchan != GSM_PCHAN_TCH_F &&
		    ts->pchan != GSM_PCHAN_PDCH &&
		    ts->pchan != GSM_PCHAN_TCH_F_PDCH)
			LOGP(DRSL, LOGL_ERROR, "chan_nr=0x%02x but pchan=%u\n",
				chan_nr, ts->pchan);
	} else if ((cbits & 0x1e) == 0x02) {
		lch_idx = cbits & 0x1;	/* TCH/H */
		if (ts->pchan != GSM_PCHAN_TCH_H)
			LOGP(DRSL, LOGL_ERROR, "chan_nr=0x%02x but pchan=%u\n",
				chan_nr, ts->pchan);
	} else if ((cbits & 0x1c) == 0x04) {
		lch_idx = cbits & 0x3;	/* SDCCH/4 */
		if (ts->pchan != GSM_PCHAN_CCCH_SDCCH4)
			LOGP(DRSL, LOGL_ERROR, "chan_nr=0x%02x but pchan=%u\n",
				chan_nr, ts->pchan);
	} else if ((cbits & 0x18) == 0x08) {
		lch_idx = cbits & 0x7;	/* SDCCH/8 */
		if (ts->pchan != GSM_PCHAN_SDCCH8_SACCH8C)
			LOGP(DRSL, LOGL_ERROR, "chan_nr=0x%02x but pchan=%u\n",
				chan_nr, ts->pchan);
	} else if (cbits == 0x10 || cbits == 0x11 || cbits == 0x12) {
		lch_idx = 0;
		if (ts->pchan != GSM_PCHAN_CCCH &&
		    ts->pchan != GSM_PCHAN_CCCH_SDCCH4)
			LOGP(DRSL, LOGL_ERROR, "chan_nr=0x%02x but pchan=%u\n",
				chan_nr, ts->pchan);
		/* FIXME: we should not return first sdcch4 !!! */
	} else {
		LOGP(DRSL, LOGL_ERROR, "unknown chan_nr=0x%02x\n", chan_nr);
		return NULL;
	}

	lchan = &ts->lchan[lch_idx];
#if 0
	log_set_context(BSC_CTX_LCHAN, lchan);
	if (lchan->conn)
		log_set_context(BSC_CTX_SUBSCR, lchan->conn->subscr);
#endif

	return lchan;
}

static struct msgb *rsl_msgb_alloc(int hdr_size)
{
	struct msgb *nmsg;

	nmsg = abis_msgb_alloc(hdr_size);
	if (!nmsg)
		return NULL;
	nmsg->l3h = nmsg->data;
	return nmsg;
}

static void rsl_trx_push_hdr(struct msgb *msg, uint8_t msg_type)
{
	struct abis_rsl_common_hdr *th;

	th = (struct abis_rsl_common_hdr *) msgb_push(msg, sizeof(*th));
	th->msg_discr = ABIS_RSL_MDISC_TRX;
	th->msg_type = msg_type;
}

static void rsl_cch_push_hdr(struct msgb *msg, uint8_t msg_type, uint8_t chan_nr)
{
	struct abis_rsl_cchan_hdr *cch;

	cch = (struct abis_rsl_cchan_hdr *) msgb_push(msg, sizeof(*cch));
	cch->c.msg_discr = ABIS_RSL_MDISC_COM_CHAN;
	cch->c.msg_type = msg_type;
	cch->chan_nr = chan_nr;
}

static void rsl_dch_push_hdr(struct msgb *msg, uint8_t msg_type, uint8_t chan_nr)
{
	struct abis_rsl_dchan_hdr *dch;

	dch = (struct abis_rsl_dchan_hdr *) msgb_push(msg, sizeof(*dch));
	dch->c.msg_discr = ABIS_RSL_MDISC_DED_CHAN;
	dch->c.msg_type = msg_type;
	dch->ie_chan = RSL_IE_CHAN_NR;
	dch->chan_nr = chan_nr;
}


/*
 * TRX related messages
 */

/* 8.6.4 sending ERROR REPORT */
static int rsl_tx_error_report(struct gsm_bts_trx *trx, uint8_t cause)
{
	struct msgb *nmsg;

	LOGP(DRSL, LOGL_NOTICE, "Tx RSL Error Report: cause = 0x%02x\n", cause);

	nmsg = rsl_msgb_alloc(sizeof(struct abis_rsl_common_hdr));
	if (!nmsg)
		return -ENOMEM;
	msgb_tlv_put(nmsg, RSL_IE_CAUSE, 1, &cause);
	rsl_trx_push_hdr(nmsg, RSL_MT_ERROR_REPORT);
	nmsg->trx = trx;

	return abis_rsl_sendmsg(nmsg);
}

/* 8.6.1 sending RF RESOURCE INDICATION */
int rsl_tx_rf_res(struct gsm_bts_trx *trx)
{
	struct msgb *nmsg;

	LOGP(DRSL, LOGL_INFO, "Tx RSL RF RESource INDication\n");

	nmsg = rsl_msgb_alloc(sizeof(struct abis_rsl_common_hdr));
	if (!nmsg)
		return -ENOMEM;
	// FIXME: add interference levels of TRX
	rsl_trx_push_hdr(nmsg, RSL_MT_RF_RES_IND);
	nmsg->trx = trx;

	return abis_rsl_sendmsg(nmsg);
}

/* 
 * common channel releated messages
 */

/* 8.5.1 BCCH INFOrmation is received */
static int rsl_rx_bcch_info(struct gsm_bts_trx *trx, struct msgb *msg)
{
	struct gsm_bts *bts = trx->bts;
	struct tlv_parsed tp;
	uint8_t rsl_si;
	enum osmo_sysinfo_type osmo_si;

	rsl_tlv_parse(&tp, msgb_l3(msg), msgb_l3len(msg));

	/* 9.3.30 System Info Type */
	if (!TLVP_PRESENT(&tp, RSL_IE_SYSINFO_TYPE))
		return rsl_tx_error_report(trx, RSL_ERR_MAND_IE_ERROR);

	rsl_si = *TLVP_VAL(&tp, RSL_IE_SYSINFO_TYPE);
	if (OSMO_IN_ARRAY(rsl_si, rsl_sacch_sitypes))
		return rsl_tx_error_report(trx, RSL_ERR_IE_CONTENT);

	osmo_si = osmo_rsl2sitype(rsl_si);
	if (osmo_si == SYSINFO_TYPE_NONE) {
		LOGP(DRSL, LOGL_NOTICE, " Rx RSL SI 0x%02x not supported.\n", rsl_si);
		return rsl_tx_error_report(trx, RSL_ERR_IE_CONTENT);
	}
	/* 9.3.39 Full BCCH Information */
	if (TLVP_PRESENT(&tp, RSL_IE_FULL_BCCH_INFO)) {
		uint8_t len = TLVP_LEN(&tp, RSL_IE_FULL_BCCH_INFO);
		if (len > sizeof(sysinfo_buf_t))
			len = sizeof(sysinfo_buf_t);
		bts->si_valid |= (1 << osmo_si);
		memcpy(bts->si_buf[osmo_si],
			TLVP_VAL(&tp, RSL_IE_FULL_BCCH_INFO), len);
		LOGP(DRSL, LOGL_INFO, " Rx RSL BCCH INFO (SI%s)\n",
			get_value_string(osmo_sitype_strs, osmo_si));
	} else if (TLVP_PRESENT(&tp, RSL_IE_L3_INFO)) {
		uint16_t len = TLVP_LEN(&tp, RSL_IE_L3_INFO);
		if (len > sizeof(sysinfo_buf_t))
			len = sizeof(sysinfo_buf_t);
		bts->si_valid |= (1 << osmo_si);
		memcpy(bts->si_buf[osmo_si],
			TLVP_VAL(&tp, RSL_IE_L3_INFO), len);
		LOGP(DRSL, LOGL_INFO, " Rx RSL BCCH INFO (SI%s)\n",
			get_value_string(osmo_sitype_strs, osmo_si));
	} else {
		bts->si_valid &= (1 << osmo_si);
		LOGP(DRSL, LOGL_INFO, " RX RSL Disabling BCCH INFO (SI%s)\n",
			get_value_string(osmo_sitype_strs, osmo_si));
	}
	osmo_signal_dispatch(SS_GLOBAL, S_NEW_SYSINFO, bts);

	return 0;
}

/* 8.5.2 CCCH Load Indication (PCH) */
int rsl_tx_ccch_load_ind_pch(struct gsm_bts *bts, uint16_t paging_avail)
{
	struct msgb *msg;

	msg = rsl_msgb_alloc(sizeof(struct abis_rsl_common_hdr));
	if (!msg)
		return -ENOMEM;
	rsl_trx_push_hdr(msg, RSL_MT_CCCH_LOAD_IND);
	msgb_tv16_put(msg, RSL_IE_PAGING_LOAD, paging_avail);
	msg->trx = bts->c0;

	return abis_rsl_sendmsg(msg);
}

/* 8.5.5 PAGING COMMAND */
static int rsl_rx_paging_cmd(struct gsm_bts_trx *trx, struct msgb *msg)
{
	struct gsm_bts_role_bts *btsb = trx->bts->role;
	struct tlv_parsed tp;
	uint8_t chan_needed = 0, paging_group;
	const uint8_t *identity_lv;
	int rc;

	rsl_tlv_parse(&tp, msgb_l3(msg), msgb_l3len(msg));

	if (!TLVP_PRESENT(&tp, RSL_IE_PAGING_GROUP) ||
	    !TLVP_PRESENT(&tp, RSL_IE_MS_IDENTITY))
		return rsl_tx_error_report(trx, RSL_ERR_MAND_IE_ERROR);

	paging_group = *TLVP_VAL(&tp, RSL_IE_PAGING_GROUP);
	identity_lv = TLVP_VAL(&tp, RSL_IE_MS_IDENTITY)-1;

	if (TLVP_PRESENT(&tp, RSL_IE_CHAN_NEEDED))
		chan_needed = *TLVP_VAL(&tp, RSL_IE_CHAN_NEEDED);

	rc = paging_add_identity(btsb->paging_state, paging_group,
				 identity_lv, chan_needed);
	if (rc < 0) {
		/* FIXME: notfiy the BSC somehow ?*/
	}

	return 0;
}

int rsl_tx_ccch_load_ind_rach(struct gsm_bts *bts, uint16_t rach_slots,
			      uint16_t rach_busy, uint16_t rach_access)
{
	struct msgb *msg;
	uint16_t payload[3];

	payload[0] = htons(rach_slots);
	payload[1] = htons(rach_busy);
	payload[2] = htons(rach_access);

	msg = rsl_msgb_alloc(sizeof(struct abis_rsl_common_hdr));
	if (!msg)
		return -ENOMEM;

	msgb_tlv_put(msg, RSL_IE_RACH_LOAD, 6, (uint8_t *)payload);
	rsl_trx_push_hdr(msg, RSL_MT_CCCH_LOAD_IND);
	msg->trx = bts->c0;

	return abis_rsl_sendmsg(msg);
}

/* 8.6.2 SACCH FILLING */
static int rsl_rx_sacch_fill(struct gsm_bts_trx *trx, struct msgb *msg)
{
	struct gsm_bts *bts = trx->bts;
	struct tlv_parsed tp;
	uint8_t rsl_si;
	enum osmo_sysinfo_type osmo_si;

	rsl_tlv_parse(&tp, msgb_l3(msg), msgb_l3len(msg));

	/* 9.3.30 System Info Type */
	if (!TLVP_PRESENT(&tp, RSL_IE_SYSINFO_TYPE))
		return rsl_tx_error_report(trx, RSL_ERR_MAND_IE_ERROR);

	rsl_si = *TLVP_VAL(&tp, RSL_IE_SYSINFO_TYPE);
	if (!OSMO_IN_ARRAY(rsl_si, rsl_sacch_sitypes))
		return rsl_tx_error_report(trx, RSL_ERR_IE_CONTENT);

	osmo_si = osmo_rsl2sitype(rsl_si);
	if (osmo_si == SYSINFO_TYPE_NONE) {
		LOGP(DRSL, LOGL_NOTICE, " Rx SACCH SI 0x%02x not supported.\n", rsl_si);
		return rsl_tx_error_report(trx, RSL_ERR_IE_CONTENT);
	}
	if (TLVP_PRESENT(&tp, RSL_IE_L3_INFO)) {
		uint16_t len = TLVP_LEN(&tp, RSL_IE_L3_INFO);
		/* We have to pre-fix with the two-byte LAPDM UI header */
		if (len > sizeof(sysinfo_buf_t)-2)
			len = sizeof(sysinfo_buf_t)-2;
		bts->si_valid |= (1 << osmo_si);
		bts->si_buf[osmo_si][0] = 0x00;
		bts->si_buf[osmo_si][1] = 0x03;
		memcpy(bts->si_buf[osmo_si]+2,
			TLVP_VAL(&tp, RSL_IE_L3_INFO), len);
		LOGP(DRSL, LOGL_INFO, " Rx RSL SACCH FILLING (SI%s)\n",
			get_value_string(osmo_sitype_strs, osmo_si));
	} else {
		bts->si_valid &= (1 << osmo_si);
		LOGP(DRSL, LOGL_INFO, " Rx RSL Disabling SACCH FILLING (SI%s)\n",
			get_value_string(osmo_sitype_strs, osmo_si));
	}
	osmo_signal_dispatch(SS_GLOBAL, S_NEW_SYSINFO, bts);

	return 0;

}

/* 8.5.6 IMMEDIATE ASSIGN COMMAND is received */
static int rsl_rx_imm_ass(struct gsm_bts_trx *trx, struct msgb *msg)
{
	struct tlv_parsed tp;

	rsl_tlv_parse(&tp, msgb_l3(msg), msgb_l3len(msg));

	if (!TLVP_PRESENT(&tp, RSL_IE_FULL_IMM_ASS_INFO))
		return rsl_tx_error_report(trx, RSL_ERR_MAND_IE_ERROR);

	/* cut down msg to the 04.08 RR part */
	msg->data = (uint8_t *) TLVP_VAL(&tp, RSL_IE_FULL_IMM_ASS_INFO);
	msg->len = TLVP_LEN(&tp, RSL_IE_FULL_IMM_ASS_INFO);

	/* put into the AGCH queue of the BTS */
	if (bts_agch_enqueue(trx->bts, msg) < 0) {
		/* if there is no space in the queue: send DELETE IND */
		msgb_free(msg);
	}

	/* return 1 means: don't msgb_free() the msg */
	return 1;
}

/*
 * dedicated channel related messages
 */

/* 8.4.19 sebdubg RF CHANnel RELease ACKnowledge */
int rsl_tx_rf_rel_ack(struct gsm_lchan *lchan)
{
	struct msgb *msg = rsl_msgb_alloc(sizeof(struct abis_rsl_dchan_hdr));
	uint8_t chan_nr = gsm_lchan2chan_nr(lchan);

	LOGP(DRSL, LOGL_NOTICE, "%s Tx RF CHAN REL ACK\n", gsm_lchan_name(lchan));

	rsl_dch_push_hdr(msg, RSL_MT_RF_CHAN_REL_ACK, chan_nr);
	msg->trx = lchan->ts->trx;

	return abis_rsl_sendmsg(msg);
}

/* 8.4.2 sending CHANnel ACTIVation ACKnowledge */
int rsl_tx_chan_act_ack(struct gsm_lchan *lchan, struct gsm_time *gtime)
{
	struct msgb *msg = rsl_msgb_alloc(sizeof(struct abis_rsl_dchan_hdr));
	uint8_t chan_nr = gsm_lchan2chan_nr(lchan);
	uint8_t ie[2];

	LOGP(DRSL, LOGL_NOTICE, "%s Tx CHAN ACT ACK\n", gsm_lchan_name(lchan));

	gsm48_gen_starting_time(ie, gtime);
	msgb_tv_fixed_put(msg, RSL_IE_FRAME_NUMBER, 2, ie);
	rsl_dch_push_hdr(msg, RSL_MT_CHAN_ACTIV_ACK, chan_nr);
	msg->trx = lchan->ts->trx;

	/* since activation was successful, do some lchan initialization */
	lchan->meas.res_nr = 0;

	return abis_rsl_sendmsg(msg);
}

/* 8.4.3 sending CHANnel ACTIVation Negative ACK */
static int rsl_tx_chan_nack(struct gsm_bts_trx *trx, struct msgb *msg, uint8_t cause)
{
	struct abis_rsl_dchan_hdr *dch = msgb_l2(msg);
	uint8_t chan_nr = dch->chan_nr;

	LOGP(DRSL, LOGL_NOTICE, "Sending Channel Activated NACK: cause = 0x%02x\n", cause);

	msg->len = 0;
	msg->data = msg->tail = msg->l3h;

	/* 9.3.26 Cause */
	msgb_tlv_put(msg, RSL_IE_CAUSE, 1, &cause);
	rsl_dch_push_hdr(msg, RSL_MT_CHAN_ACTIV_NACK, chan_nr);
	msg->trx = trx;

	return abis_rsl_sendmsg(msg);
}

/* 8.5.3 sending CHANnel ReQuireD */
int rsl_tx_chan_rqd(struct gsm_bts_trx *trx, struct gsm_time *gtime,
		    uint8_t ra, uint8_t acc_delay)
{
	struct msgb *nmsg;
	uint8_t payload[3];

	LOGP(DRSL, LOGL_NOTICE, "Sending Channel Required\n");

	nmsg = rsl_msgb_alloc(sizeof(struct abis_rsl_cchan_hdr));
	if (!nmsg)
		return -ENOMEM;

	/* 9.3.19 Request Reference */
	payload[0] = ra;
	gsm48_gen_starting_time(payload+1, gtime);
	msgb_tv_fixed_put(nmsg, RSL_IE_REQ_REFERENCE, 3, payload);

	/* 9.3.17 Access Delay */
	msgb_tv_put(nmsg, RSL_IE_ACCESS_DELAY, acc_delay);

	rsl_cch_push_hdr(nmsg, RSL_MT_CHAN_RQD, 0x88); // FIXME
	nmsg->trx = trx;

	return abis_rsl_sendmsg(nmsg);
}

/* copy the SACCH related sysinfo from BTS global buffer to lchan specific buffer */
static void copy_sacch_si_to_lchan(struct gsm_lchan *lchan)
{
	struct gsm_bts *bts = lchan->ts->trx->bts;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(rsl_sacch_sitypes); i++) {
		uint8_t rsl_si = rsl_sacch_sitypes[i];
		uint8_t osmo_si = osmo_rsl2sitype(rsl_si);
		uint8_t osmo_si_shifted = (1 << osmo_si);
		if (osmo_si == SYSINFO_TYPE_NONE)
			continue;
		if (!(bts->si_valid & osmo_si_shifted)) {
			lchan->si.valid &= ~osmo_si_shifted;
			continue;
		}
		lchan->si.valid |= osmo_si_shifted;
		memcpy(lchan->si.buf[osmo_si], bts->si_buf[osmo_si],
			sizeof(sysinfo_buf_t));
	}
}


/* 8.4.1 CHANnel ACTIVation is received */
static int rsl_rx_chan_activ(struct msgb *msg)
{
	struct abis_rsl_dchan_hdr *dch = msgb_l2(msg);
	struct gsm_lchan *lchan = msg->lchan;
	struct tlv_parsed tp;
	uint8_t type, mode;

	rsl_tlv_parse(&tp, msgb_l3(msg), msgb_l3len(msg));

	/* 9.3.3 Activation Type */
	if (!TLVP_PRESENT(&tp, RSL_IE_ACT_TYPE)) {
		LOGP(DRSL, LOGL_NOTICE, "missing Activation Type\n");
		msgb_free(msg);
		return rsl_tx_chan_nack(msg->trx, msg, RSL_ERR_MAND_IE_ERROR);
	}
	type = *TLVP_VAL(&tp, RSL_IE_ACT_TYPE);

	/* 9.3.6 Channel Mode */
	if (!TLVP_PRESENT(&tp, RSL_IE_CHAN_MODE)) {
		LOGP(DRSL, LOGL_NOTICE, "missing Channel Mode\n");
		msgb_free(msg);
		return rsl_tx_chan_nack(msg->trx, msg, RSL_ERR_MAND_IE_ERROR);
	}
	mode = *TLVP_VAL(&tp, RSL_IE_CHAN_MODE);

	/* 9.3.7 Encryption Information */
	if (TLVP_PRESENT(&tp, RSL_IE_ENCR_INFO)) {
		uint8_t len = TLVP_LEN(&tp, RSL_IE_ENCR_INFO);
		const uint8_t *val = TLVP_VAL(&tp, RSL_IE_ENCR_INFO);
		lchan->encr.alg_id = *val++;
		lchan->encr.key_len = len -1;
		if (lchan->encr.key_len > sizeof(lchan->encr.key))
			lchan->encr.key_len = sizeof(lchan->encr.key);
		memcpy(lchan->encr.key, val, lchan->encr.key_len);
	}

	/* 9.3.9 Handover Reference */

	/* 9.3.4 BS Power */
	if (TLVP_PRESENT(&tp, RSL_IE_BS_POWER))
		lchan->bs_power = *TLVP_VAL(&tp, RSL_IE_BS_POWER);
	/* 9.3.13 MS Power */
	if (TLVP_PRESENT(&tp, RSL_IE_MS_POWER))
		lchan->bs_power = *TLVP_VAL(&tp, RSL_IE_MS_POWER);
	/* 9.3.24 Timing Advance */
	if (TLVP_PRESENT(&tp, RSL_IE_TIMING_ADVANCE))
		lchan->rqd_ta = *TLVP_VAL(&tp, RSL_IE_TIMING_ADVANCE);

	/* 9.3.32 BS Power Parameters */
	/* 9.3.31 MS Power Parameters */
	/* 9.3.16 Physical Context */

	/* 9.3.29 SACCH Information */
	if (TLVP_PRESENT(&tp, RSL_IE_SACCH_INFO)) {
		uint8_t tot_len = TLVP_LEN(&tp, RSL_IE_SACCH_INFO);
		const uint8_t *val = TLVP_VAL(&tp, RSL_IE_SACCH_INFO);
		const uint8_t *cur = val;
		uint8_t num_msgs = *cur++;
		unsigned int i;
		for (i = 0; i < num_msgs; i++) {
			uint8_t rsl_si = *cur++;
			uint8_t si_len = *cur++;
			uint8_t osmo_si;
			uint8_t copy_len;

			if (!OSMO_IN_ARRAY(rsl_si, rsl_sacch_sitypes))
				return rsl_tx_error_report(msg->trx, RSL_ERR_IE_CONTENT);

			osmo_si = osmo_rsl2sitype(rsl_si);
			if (osmo_si == SYSINFO_TYPE_NONE) {
				LOGP(DRSL, LOGL_NOTICE, " Rx SACCH SI 0x%02x not supported.\n", rsl_si);
				return rsl_tx_error_report(msg->trx, RSL_ERR_IE_CONTENT);
			}

			copy_len = si_len;
			/* We have to pre-fix with the two-byte LAPDM UI header */
			if (copy_len > sizeof(sysinfo_buf_t)-2)
				copy_len = sizeof(sysinfo_buf_t)-2;
			lchan->si.valid |= (1 << osmo_si);
			lchan->si.buf[osmo_si][0] = 0x00;
			lchan->si.buf[osmo_si][1] = 0x03;
			memcpy(lchan->si.buf[osmo_si]+2, cur, copy_len);

			cur += si_len;
			if (cur >= val + tot_len) {
				LOGP(DRSL, LOGL_ERROR, "Error parsing SACCH INFO IE\n");
				return rsl_tx_error_report(msg->trx, RSL_ERR_IE_CONTENT);
			}
		}
	} else {
		/* use standard SACCH filling of the BTS */
		copy_sacch_si_to_lchan(lchan);
	}
	/* 9.3.52 MultiRate Configuration */
	/* 9.3.53 MultiRate Control */
	/* 9.3.54 Supported Codec Types */

	LOGP(DRSL, LOGL_INFO, " chan_nr=0x%02x type=0x%02x mode=0x%02x\n", dch->chan_nr, type, mode);

	/* actually activate the channel in the BTS */
	return  bts_model_rsl_chan_act(msg->lchan, &tp);
}

/* 8.4.14 RF CHANnel RELease is received */
static int rsl_rx_rf_chan_rel(struct msgb *msg)
{
	int rc;
#if 0
	if (lchan->rtp.socket_created)
		rsl_tx_ipac_dlcx_ind(lchan, RSL_ERR_NORMAL_UNSPEC);
#endif
	rc = bts_model_rsl_chan_rel(msg->lchan);

	lapdm_channel_reset(&msg->lchan->lapdm_ch);

	return rc;
}

/* 8.4.20 SACCH INFO MODify */
static int rsl_rx_sacch_inf_mod(struct msgb *msg)
{
	struct gsm_lchan *lchan = msg->lchan;
	struct tlv_parsed tp;
	uint8_t rsl_si, osmo_si;

	rsl_tlv_parse(&tp, msgb_l3(msg), msgb_l3len(msg));

	if (TLVP_PRESENT(&tp, RSL_IE_STARTNG_TIME)) {
		LOGP(DRSL, LOGL_NOTICE, "Starting time not supported\n");
		return rsl_tx_error_report(msg->trx, RSL_ERR_SERV_OPT_UNIMPL);
	}

	/* 9.3.30 System Info Type */
	if (!TLVP_PRESENT(&tp, RSL_IE_SYSINFO_TYPE))
		return rsl_tx_error_report(msg->trx, RSL_ERR_MAND_IE_ERROR);

	rsl_si = *TLVP_VAL(&tp, RSL_IE_SYSINFO_TYPE);
	if (!OSMO_IN_ARRAY(rsl_si, rsl_sacch_sitypes))
		return rsl_tx_error_report(msg->trx, RSL_ERR_IE_CONTENT);

	osmo_si = osmo_rsl2sitype(rsl_si);
	if (osmo_si == SYSINFO_TYPE_NONE) {
		LOGP(DRSL, LOGL_NOTICE, "%s Rx SACCH SI 0x%02x not supported.\n",
			gsm_lchan_name(lchan), rsl_si);
		return rsl_tx_error_report(msg->trx, RSL_ERR_IE_CONTENT);
	}
	if (TLVP_PRESENT(&tp, RSL_IE_L3_INFO)) {
		uint16_t len = TLVP_LEN(&tp, RSL_IE_L3_INFO);
		/* We have to pre-fix with the two-byte LAPDM UI header */
		if (len > sizeof(sysinfo_buf_t)-2)
			len = sizeof(sysinfo_buf_t)-2;
		lchan->si.valid |= (1 << osmo_si);
		lchan->si.buf[osmo_si][0] = 0x00;
		lchan->si.buf[osmo_si][1] = 0x03;
		memcpy(lchan->si.buf[osmo_si]+2,
			TLVP_VAL(&tp, RSL_IE_L3_INFO), len);
		LOGP(DRSL, LOGL_INFO, "%s Rx RSL SACCH FILLING (SI%s)\n",
			gsm_lchan_name(lchan),
			get_value_string(osmo_sitype_strs, osmo_si));
	} else {
		lchan->si.valid &= (1 << osmo_si);
		LOGP(DRSL, LOGL_INFO, "%s Rx RSL Disabling SACCH FILLING (SI%s)\n",
			gsm_lchan_name(lchan),
			get_value_string(osmo_sitype_strs, osmo_si));
	}

	return 0;
}

#if 0
/*
 * ip.access related messages
 */

int rsl_tx_ipac_dlcx_ind(struct osmobts_lchan *lchan, uint8_t cause)
{
	struct msgb *nmsg;
	struct gsm_bts_trx *trx = lchan->slot->trx;

	LOGP(DRSL, LOGL_NOTICE, "Sending RTP delete indication: cause=%d\n", cause);

	nmsg = rsl_msgb_alloc(sizeof(struct abis_rsl_common_hdr));
	if (!nmsg)
		return -ENOMEM;
	msgb_tlv_put(nmsg, RSL_IE_CAUSE, 1, &cause);
	rsl_trx_push_hdr(nmsg, RSL_MT_IPAC_DLCX_IND);
	msg->trx = trx;

	return abis_rsl_sendmsg(nmsg);
}

static int rsl_tx_ipac_cx_ack(struct gsm_bts_trx *trx, struct msgb *msg, uint32_t ip, uint16_t port, uint16_t *conn_id)
{
	struct abis_rsl_dchan_hdr *dch = msgb_l2(msg);
	uint8_t chan_nr = dch->chan_nr;
	uint8_t msg_type = dch->c.msg_type;
	uint8_t *ie;

	LOGP(DRSL, LOGL_NOTICE, "Sending RTP connection request ACK\n");

	msg->len = 0;
	msg->data = msg->tail = msg->l3h;

	if (conn_id)
		msgb_tv16_put(msg, RSL_IE_IPAC_CONN_ID, htons(*conn_id));
	ie = msgb_put(msg, 5);
	ie[0] = RSL_IE_IPAC_LOCAL_IP;
	*((uint32_t *)(ie + 1)) = htonl(ip);
	msgb_tv16_put(msg, RSL_IE_IPAC_LOCAL_PORT, htons(port));

	rsl_dch_push_hdr(msg, msg_type + 1, chan_nr);
	msg->trx = trx;

	return abis_rsl_sendmsg(msg);
}

static int rsl_tx_ipac_cx_nack(struct gsm_bts_trx *trx, struct msgb *msg, uint8_t cause)
{
	struct abis_rsl_dchan_hdr *dch = msgb_l2(msg);
	uint8_t chan_nr = dch->chan_nr;
	uint8_t msg_type = dch->c.msg_type;

	LOGP(DRSL, LOGL_NOTICE, "Sending RTP connection request NACK: cause=%d\n", cause);

	msg->len = 0;
	msg->data = msg->tail = msg->l3h;

	/* 9.3.26 Cause */
	msgb_tlv_put(msg, RSL_IE_CAUSE, 1, &cause);
	rsl_dch_push_hdr(msg, msg_type + 2, chan_nr);
	msg->trx = trx;

	return abis_rsl_sendmsg(msg);
}

static int rsl_rx_ipac_crcx_mdcx(struct gsm_bts_trx *trx, struct msgb *msg)
{
	struct abis_rsl_dchan_hdr *dch = msgb_l2(msg);
	struct tlv_parsed tp;
	struct osmobts_lchan *lchan;
	uint32_t *ip = NULL;
	uint16_t *port = NULL;
	uint8_t *speech_mode = NULL, *payload_type = NULL;
	uint16_t conn_id;
	int rc;

	if (dch->c.msg_type == RSL_MT_IPAC_CRCX)
		LOGP(DRSL, LOGL_INFO, "Request of creating RTP connection:\n");
	else
		LOGP(DRSL, LOGL_INFO, "Request of modding RTP connection:\n");

	lchan = rsl_lchan_lookup(trx, dch->chan_nr);
	if (!lchan)
		return rsl_tx_ipac_cx_nack(trx, msg, RSL_ERR_MAND_IE_ERROR);

	rsl_tlv_parse(&tp, msgb_l3(msg), msgb_l3len(msg));

	if (TLVP_PRESENT(&tp, RSL_IE_IPAC_SPEECH_MODE)) {
		speech_mode = (uint8_t *) TLVP_VAL(&tp, RSL_IE_IPAC_SPEECH_MODE);
	}
	if (TLVP_PRESENT(&tp, RSL_IE_IPAC_RTP_PAYLOAD)) {
		payload_type = (uint8_t *) TLVP_VAL(&tp, RSL_IE_IPAC_RTP_PAYLOAD);
	}
	if (TLVP_PRESENT(&tp, RSL_IE_IPAC_REMOTE_IP)) {
		ip = (uint32_t *) TLVP_VAL(&tp, RSL_IE_IPAC_REMOTE_IP);
	}
	if (TLVP_PRESENT(&tp, RSL_IE_IPAC_REMOTE_PORT)) {
		port = (uint16_t *) TLVP_VAL(&tp, RSL_IE_IPAC_REMOTE_PORT);
	}

	if (!lchan->rtp.socket_created) {
		if (dch->c.msg_type == RSL_MT_IPAC_CRCX && !payload_type) {
			LOGP(DRSL, LOGL_ERROR, "Missing payload type IE.\n");
			return rsl_tx_ipac_cx_nack(trx, msg, RSL_ERR_MAND_IE_ERROR);
		}
		rc = rtp_create_socket(lchan, &lchan->rtp);
		if (rc < 0) {
			LOGP(DRSL, LOGL_ERROR, "Failed to create RTP/RTCP sockets.\n");
			return rsl_tx_ipac_cx_nack(trx, msg, RSL_ERR_RES_UNAVAIL);
		}

		rc = rtp_bind_socket(&lchan->rtp);
		if (rc < 0) {
			LOGP(DRSL, LOGL_ERROR, "Failed to bind RTP/RTCP sockets.\n");
			rtp_close_socket(&lchan->rtp);
			return rsl_tx_ipac_cx_nack(trx, msg, RSL_ERR_RES_UNAVAIL);
		}
	}
	if (payload_type)
		lchan->rtp.payload_type = *payload_type;

	if (ip && port) {
		rc = rtp_connect_socket(&lchan->rtp, ntohl(*ip), ntohs(*port));
		if (rc < 0) {
			LOGP(DRSL, LOGL_ERROR, "Failed to connect RTP/RTCP sockets.\n");
			rtp_close_socket(&lchan->rtp);
			return rsl_tx_ipac_cx_nack(trx, msg, RSL_ERR_RES_UNAVAIL);
		}
	}

	conn_id = 0; // FIXME: what the hack is conn_id for?
	return rsl_tx_ipac_cx_ack(trx, msg, ntohl(lchan->rtp.rtp_udp.sin_local.sin_addr.s_addr), ntohs(lchan->rtp.rtp_udp.sin_local.sin_port), &conn_id);
}

static int rsl_rx_ipac_dlcx(struct gsm_bts_trx *trx, struct msgb *msg)
{
	struct abis_rsl_dchan_hdr *dch = msgb_l2(msg);
	struct tlv_parsed tp;
	struct osmobts_lchan *lchan;

	LOGP(DRSL, LOGL_INFO, "Request of deleting RTP connection:\n");

	lchan = rsl_lchan_lookup(trx, dch->chan_nr);
	if (!lchan)
		return rsl_tx_ipac_cx_nack(trx, msg, RSL_ERR_MAND_IE_ERROR);

	rsl_tlv_parse(&tp, msgb_l3(msg), msgb_l3len(msg));

	rtp_close_socket(&lchan->rtp);

	return rsl_tx_ipac_cx_ack(trx, msg, ntohl(lchan->rtp.rtp_udp.sin_local.sin_addr.s_addr), ntohs(lchan->rtp.rtp_udp.sin_local.sin_port), NULL);
}
#endif

/*
 * selecting message
 */

static int rsl_rx_rll(struct gsm_bts_trx *trx, struct msgb *msg)
{
	struct abis_rsl_rll_hdr *rh = msgb_l2(msg);
	struct gsm_lchan *lchan;

	if (msgb_l2len(msg) < sizeof(*rh)) {
		LOGP(DRSL, LOGL_NOTICE, "RSL Radio Link Layer message too short\n");
		msgb_free(msg);
		return -EIO;
	}
	msg->l3h = (unsigned char *)rh + sizeof(*rh);

	lchan = rsl_lchan_lookup(trx, rh->chan_nr);
	if (!lchan) {
		LOGP(DRLL, LOGL_NOTICE, "Rx RLL %s for unknown lchan\n",
			rsl_msg_name(rh->c.msg_type));
		return rsl_tx_chan_nack(trx, msg, RSL_ERR_MAND_IE_ERROR);
	}

	DEBUGP(DRLL, "%s Rx RLL %s Abis -> LAPDm\n", gsm_lchan_name(lchan),
		rsl_msg_name(rh->c.msg_type));

	/* exception: RLL messages are _NOT_ freed as they are now
	 * owned by LAPDm which might have queued them */
	return lapdm_rslms_recvmsg(msg, &lchan->lapdm_ch);
}

static inline int rsl_link_id_is_sacch(uint8_t link_id)
{
	if (link_id >> 6 == 1)
		return 1;
	else
		return 0;
}

static int rslms_is_meas_rep(struct msgb *msg)
{
	struct abis_rsl_common_hdr *rh = msgb_l2(msg);
	struct abis_rsl_rll_hdr *rllh;
	struct gsm48_hdr *gh;

	if ((rh->msg_discr & 0xfe) != ABIS_RSL_MDISC_RLL)
		return 0;

	if (rh->msg_type != RSL_MT_UNIT_DATA_IND)
		return 0;

	rllh = msgb_l2(msg);
	if (rsl_link_id_is_sacch(rllh->link_id) == 0)
		return 0;

	gh = msgb_l3(msg);
	if (gh->proto_discr != GSM48_PDISC_RR)
		return 0;

	switch (gh->msg_type) {
	case GSM48_MT_RR_MEAS_REP:
	case GSM48_MT_RR_EXT_MEAS_REP:
		return 1;
	default:
		break;
	}

	/* FIXME: this does not cover the Bter frame format and the associated
	 * short RR protocol descriptor for ENHANCED MEASUREMENT REPORT */

	return 0;
}

/* 8.4.8 MEASUREMENT RESult */
static int rsl_tx_meas_res(struct gsm_lchan *lchan, uint8_t *l3, int l3_len)
{
	struct msgb *msg = rsl_msgb_alloc(sizeof(struct abis_rsl_dchan_hdr));
	uint8_t chan_nr = gsm_lchan2chan_nr(lchan);

	LOGP(DRSL, LOGL_NOTICE, "%s Tx MEAS RES\n", gsm_lchan_name(lchan));

	msgb_tv_put(msg, RSL_IE_MEAS_RES_NR, lchan->meas.res_nr++);
	if (lchan->meas.flags & LC_UL_M_F_RES_VALID) {
		uint8_t meas_res[16];
		int ie_len = lchan_build_rsl_ul_meas(lchan, meas_res);
		if (ie_len >= 3) {
			msgb_tlv_put(msg, RSL_IE_UPLINK_MEAS, ie_len, meas_res);
			lchan->meas.flags &= ~LC_UL_M_F_RES_VALID;
		}
	}
	msgb_tv_put(msg, RSL_IE_BS_POWER, lchan->meas.bts_tx_pwr);
	if (lchan->meas.flags & LC_UL_M_F_L1_VALID) {
		msgb_tv_fixed_put(msg, RSL_IE_L1_INFO, 2, lchan->meas.l1_info);
		lchan->meas.flags &= ~LC_UL_M_F_L1_VALID;
	}
	msgb_tl16v_put(msg, RSL_IE_L3_INFO, l3_len, l3);
		//msgb_tv_put(msg, RSL_IE_MS_TIMING_OFFSET, FIXME);

	rsl_dch_push_hdr(msg, RSL_MT_MEAS_RES, chan_nr);
	msg->trx = lchan->ts->trx;

	return abis_rsl_sendmsg(msg);
}

/* call-back for LAPDm code, called when it wants to send msgs UP */
int lapdm_rll_tx_cb(struct msgb *msg, struct lapdm_entity *le, void *ctx)
{
	struct gsm_lchan *lchan = ctx;
	struct abis_rsl_common_hdr *rh = msgb_l2(msg);

	msg->trx = lchan->ts->trx;

	/* check if this is a measurement report from SACCH which needs special
	 * processing before forwarding */
	if (rslms_is_meas_rep(msg)) {
		int rc;

		LOGP(DRSL, LOGL_INFO, "%s Handing RLL msg %s from LAPDm to MEAS REP\n",
			gsm_lchan_name(lchan), rsl_msg_name(rh->msg_type));

		rc = rsl_tx_meas_res(lchan, msgb_l3(msg), msgb_l3len(msg));
		msgb_free(msg);
		return rc;
	} else {
		LOGP(DRSL, LOGL_INFO, "%s Fwd RLL msg %s from LAPDm to A-bis\n",
			gsm_lchan_name(lchan), rsl_msg_name(rh->msg_type));

		return abis_rsl_sendmsg(msg);
	}
}

static int rsl_rx_cchan(struct gsm_bts_trx *trx, struct msgb *msg)
{
	struct abis_rsl_cchan_hdr *cch = msgb_l2(msg);
	int ret = 0;

	if (msgb_l2len(msg) < sizeof(*cch)) {
		LOGP(DRSL, LOGL_NOTICE, "RSL Common Channel Management message too short\n");
		msgb_free(msg);
		return -EIO;
	}
	msg->l3h = (unsigned char *)cch + sizeof(*cch);

	msg->lchan = rsl_lchan_lookup(trx, cch->chan_nr);
	if (!msg->lchan) {
		LOGP(DRSL, LOGL_ERROR, "Rx RSL %s for unknow lchan\n",
			rsl_msg_name(cch->c.msg_type));
		//return rsl_tx_chan_nack(trx, msg, RSL_ERR_MAND_IE_ERROR);
	}

	LOGP(DRSL, LOGL_INFO, "%s Rx RSL %s\n", gsm_lchan_name(msg->lchan),
		rsl_msg_name(cch->c.msg_type));

	switch (cch->c.msg_type) {
	case RSL_MT_BCCH_INFO:
		ret = rsl_rx_bcch_info(trx, msg);
		break;
	case RSL_MT_IMMEDIATE_ASSIGN_CMD:
		ret = rsl_rx_imm_ass(trx, msg);
		break;
	case RSL_MT_PAGING_CMD:
		ret = rsl_rx_paging_cmd(trx, msg);
		break;
	case RSL_MT_SMS_BC_REQ:
	case RSL_MT_SMS_BC_CMD:
	case RSL_MT_NOT_CMD:
		LOGP(DRSL, LOGL_NOTICE, "unimplemented RSL cchan msg_type %s\n",
			rsl_msg_name(cch->c.msg_type));
		break;
	default:
		LOGP(DRSL, LOGL_NOTICE, "undefined RSL cchan msg_type 0x%02x\n",
			cch->c.msg_type);
		ret = -EINVAL;
		break;
	}

	if (ret != 1)
		msgb_free(msg);

	return ret;
}

static int rsl_rx_dchan(struct gsm_bts_trx *trx, struct msgb *msg)
{
	struct abis_rsl_dchan_hdr *dch = msgb_l2(msg);
	int ret = 0;

	if (msgb_l2len(msg) < sizeof(*dch)) {
		LOGP(DRSL, LOGL_NOTICE, "RSL Dedicated Channel Management message too short\n");
		msgb_free(msg);
		return -EIO;
	}
	msg->l3h = (unsigned char *)dch + sizeof(*dch);

	msg->lchan = rsl_lchan_lookup(trx, dch->chan_nr);
	if (!msg->lchan) {
		LOGP(DRSL, LOGL_ERROR, "Rx RSL %s for unknow lchan\n",
			rsl_msg_name(dch->c.msg_type));
		//return rsl_tx_chan_nack(trx, msg, RSL_ERR_MAND_IE_ERROR);
	}

	LOGP(DRSL, LOGL_INFO, "%s Rx RSL %s\n", gsm_lchan_name(msg->lchan),
		rsl_msg_name(dch->c.msg_type));

	switch (dch->c.msg_type) {
	case RSL_MT_CHAN_ACTIV:
		ret = rsl_rx_chan_activ(msg);
		break;
	case RSL_MT_RF_CHAN_REL:
		ret = rsl_rx_rf_chan_rel(msg);
		break;
	case RSL_MT_SACCH_INFO_MODIFY:
		ret = rsl_rx_sacch_inf_mod(msg);
		break;
	case RSL_MT_DEACTIVATE_SACCH:
		ret = bts_model_rsl_deact_sacch(msg->lchan);
		break;
	case RSL_MT_ENCR_CMD:
	case RSL_MT_MODE_MODIFY_REQ:
	case RSL_MT_PHY_CONTEXT_REQ:
	case RSL_MT_PREPROC_CONFIG:
	case RSL_MT_RTD_REP:
	case RSL_MT_PRE_HANDO_NOTIF:
	case RSL_MT_MR_CODEC_MOD_REQ:
	case RSL_MT_TFO_MOD_REQ:
		LOGP(DRSL, LOGL_NOTICE, "unimplemented RSL dchan msg_type %s\n",
			rsl_msg_name(dch->c.msg_type));
		break;
	default:
		LOGP(DRSL, LOGL_NOTICE, "undefined RSL dchan msg_type 0x%02x\n",
			dch->c.msg_type);
		ret = -EINVAL;
	}

	if (ret != 1)
		msgb_free(msg);

	return ret;
}

static int rsl_rx_trx(struct gsm_bts_trx *trx, struct msgb *msg)
{
	struct abis_rsl_common_hdr *th = msgb_l2(msg);
	int ret = 0;

	if (msgb_l2len(msg) < sizeof(*th)) {
		LOGP(DRSL, LOGL_NOTICE, "RSL TRX message too short\n");
		msgb_free(msg);
		return -EIO;
	}
	msg->l3h = (unsigned char *)th + sizeof(*th);

	switch (th->msg_type) {
	case RSL_MT_SACCH_FILL:
		ret = rsl_rx_sacch_fill(trx, msg);
		break;
	default:
		LOGP(DRSL, LOGL_NOTICE, "undefined RSL TRX msg_type 0x%02x\n",
			th->msg_type);
		ret = -EINVAL;
	}

	if (ret != 1)
		msgb_free(msg);

	return ret;
}

static int rsl_rx_ipaccess(struct gsm_bts_trx *trx, struct msgb *msg)
{
	struct abis_rsl_dchan_hdr *dch = msgb_l2(msg);
	int ret = 0;

	if (msgb_l2len(msg) < sizeof(*dch)) {
		LOGP(DRSL, LOGL_NOTICE, "RSL ip.access message too short\n");
		msgb_free(msg);
		return -EIO;
	}
	msg->l3h = (unsigned char *)dch + sizeof(*dch);

	switch (dch->c.msg_type) {
#if 0
	case RSL_MT_IPAC_CRCX:
	case RSL_MT_IPAC_MDCX:
		return rsl_rx_ipac_crcx_mdcx(trx, msg);
	case RSL_MT_IPAC_DLCX:
		return rsl_rx_ipac_dlcx(trx, msg);
#endif
	default:
		LOGP(DRSL, LOGL_NOTICE, "unsupported RSL ip.access msg_type 0x%02x\n",
			dch->c.msg_type);
		ret = -EINVAL;
	}

	if (ret != 1)
		msgb_free(msg);
	return ret;
}

int down_rsl(struct gsm_bts_trx *trx, struct msgb *msg)
{
	struct abis_rsl_common_hdr *rslh = msgb_l2(msg);
	int ret = 0;

	if (msgb_l2len(msg) < sizeof(*rslh)) {
		LOGP(DRSL, LOGL_NOTICE, "RSL message too short\n");
		msgb_free(msg);
		return -EIO;
	}

	switch (rslh->msg_discr & 0xfe) {
	case ABIS_RSL_MDISC_RLL:
		ret = rsl_rx_rll(trx, msg);
		/* exception: RLL messages are _NOT_ freed as they are now
		 * owned by LAPDm which might have queued them */
		break;
	case ABIS_RSL_MDISC_COM_CHAN:
		ret = rsl_rx_cchan(trx, msg);
		break;
	case ABIS_RSL_MDISC_DED_CHAN:
		ret = rsl_rx_dchan(trx, msg);
		break;
	case ABIS_RSL_MDISC_TRX:
		ret = rsl_rx_trx(trx, msg);
		break;
	case ABIS_RSL_MDISC_IPACCESS:
		ret = rsl_rx_ipaccess(trx, msg);
		break;
	default:
		LOGP(DRSL, LOGL_NOTICE, "unknown RSL msg_discr 0x%02x\n",
			rslh->msg_discr);
		msgb_free(msg);
		ret = -EINVAL;
	}

	/* we don't free here, as rsl_rx{cchan,dchan,trx,ipaccess,rll} are
	 * responsible for owning the msg */

	return ret;
}
