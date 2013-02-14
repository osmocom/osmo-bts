/* GSM TS 08.58 RSL, BTS Side */

/* (C) 2011 by Andreas Eversberg <jolly@eversberg.eu>
 * (C) 2011-2013 by Harald Welte <laforge@gnumonks.org>
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
#include <netdb.h>
#include <sys/types.h>
#include <arpa/inet.h>

#include <osmocom/core/msgb.h>
#include <osmocom/core/talloc.h>
#include <osmocom/gsm/rsl.h>
#include <osmocom/gsm/lapdm.h>
#include <osmocom/gsm/protocol/gsm_12_21.h>
#include <osmocom/gsm/protocol/ipaccess.h>
#include <osmocom/trau/osmo_ortp.h>

#include <osmo-bts/logging.h>
#include <osmo-bts/gsm_data.h>
#include <osmo-bts/abis.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/rsl.h>
#include <osmo-bts/oml.h>
#include <osmo-bts/amr.h>
#include <osmo-bts/signal.h>
#include <osmo-bts/bts_model.h>
#include <osmo-bts/measurement.h>
#include <osmo-bts/pcu_if.h>
#include <osmo-bts/handover.h>

//#define FAKE_CIPH_MODE_COMPL

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

int msgb_queue_flush(struct llist_head *list)
{
	struct msgb *msg, *msg2;
	int count = 0;

	llist_for_each_entry_safe(msg, msg2, list, list) {
		msgb_free(msg);
		count++;
	}

	return count;
}

/* FIXME: move this to libosmocore */
void gsm48_gen_starting_time(uint8_t *out, struct gsm_time *gtime)
{
	uint8_t t1p = gtime->t1 % 32;
	out[0] = (t1p << 3) | (gtime->t3 >> 3);
	out[1] = (gtime->t3 << 5) | gtime->t2;
}

/* compute lchan->rsl_cmode and lchan->tch_mode from RSL CHAN MODE IE */
static void lchan_tchmode_from_cmode(struct gsm_lchan *lchan,
				     struct rsl_ie_chan_mode *cm)
{
	lchan->rsl_cmode = cm->spd_ind;
	switch (cm->chan_rate) {
	case RSL_CMOD_SP_GSM1:
		lchan->tch_mode = GSM48_CMODE_SPEECH_V1;
		break;
	case RSL_CMOD_SP_GSM2:
		lchan->tch_mode = GSM48_CMODE_SPEECH_EFR;
		break;
	case RSL_CMOD_SP_GSM3:
		lchan->tch_mode = GSM48_CMODE_SPEECH_AMR;
		break;
	case RSL_CMOD_SP_NT_14k5:
		lchan->tch_mode = GSM48_CMODE_DATA_14k5;
		break;
	case RSL_CMOD_SP_NT_12k0:
		lchan->tch_mode = GSM48_CMODE_DATA_12k0;
		break;
	case RSL_CMOD_SP_NT_6k0:
		lchan->tch_mode = GSM48_CMODE_DATA_6k0;
		break;
	}
}


/*
 * support
 */

/**
 * Handle GSM 08.58 7 Error Handling for the given input. This method will
 * send either a CHANNEL ACTIVATION NACK, MODE MODIFY NACK or ERROR REPORT
 * depending on the input of the method.
 *
 * TODO: actually make the decision
 */
static int report_error(struct gsm_bts_trx *trx)
{
	return rsl_tx_error_report(trx, RSL_ERR_IE_CONTENT);
}

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

	hdr_size += sizeof(struct ipaccess_head);

	nmsg = msgb_alloc_headroom(600+hdr_size, hdr_size, "RSL");
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
	cch->ie_chan = RSL_IE_CHAN_NR;
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

static void rsl_ipa_push_hdr(struct msgb *msg, uint8_t msg_type, uint8_t chan_nr)
{
	struct abis_rsl_dchan_hdr *dch;

	dch = (struct abis_rsl_dchan_hdr *) msgb_push(msg, sizeof(*dch));
	dch->c.msg_discr = ABIS_RSL_MDISC_IPACCESS;
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

	return abis_bts_rsl_sendmsg(nmsg);
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

	return abis_bts_rsl_sendmsg(nmsg);
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
		memset(bts->si_buf[osmo_si], 0x2b, sizeof(sysinfo_buf_t));
		memcpy(bts->si_buf[osmo_si],
			TLVP_VAL(&tp, RSL_IE_FULL_BCCH_INFO), len);
		LOGP(DRSL, LOGL_INFO, " Rx RSL BCCH INFO (SI%s)\n",
			get_value_string(osmo_sitype_strs, osmo_si));
	} else if (TLVP_PRESENT(&tp, RSL_IE_L3_INFO)) {
		uint16_t len = TLVP_LEN(&tp, RSL_IE_L3_INFO);
		if (len > sizeof(sysinfo_buf_t))
			len = sizeof(sysinfo_buf_t);
		bts->si_valid |= (1 << osmo_si);
		memset(bts->si_buf[osmo_si], 0x2b, sizeof(sysinfo_buf_t));
		memcpy(bts->si_buf[osmo_si],
			TLVP_VAL(&tp, RSL_IE_L3_INFO), len);
		LOGP(DRSL, LOGL_INFO, " Rx RSL BCCH INFO (SI%s)\n",
			get_value_string(osmo_sitype_strs, osmo_si));
	} else {
		bts->si_valid &= ~(1 << osmo_si);
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

	msg = rsl_msgb_alloc(sizeof(struct abis_rsl_cchan_hdr));
	if (!msg)
		return -ENOMEM;
	rsl_cch_push_hdr(msg, RSL_MT_CCCH_LOAD_IND, RSL_CHAN_PCH_AGCH);
	msgb_tv16_put(msg, RSL_IE_PAGING_LOAD, paging_avail);
	msg->trx = bts->c0;

	return abis_bts_rsl_sendmsg(msg);
}

/* 8.5.2 CCCH Load Indication (RACH) */
int rsl_tx_ccch_load_ind_rach(struct gsm_bts *bts, uint16_t total,
			      uint16_t busy, uint16_t access)
{
	struct msgb *msg;

	msg = rsl_msgb_alloc(sizeof(struct abis_rsl_cchan_hdr));
	if (!msg)
		return -ENOMEM;
	rsl_cch_push_hdr(msg, RSL_MT_CCCH_LOAD_IND, RSL_CHAN_RACH);
	/* tag and length */
	msgb_tv_put(msg, RSL_IE_RACH_LOAD, 6);
	/* content of the IE */
	msgb_put_u16(msg, total);
	msgb_put_u16(msg, busy);
	msgb_put_u16(msg, access);

	msg->trx = bts->c0;

	return abis_bts_rsl_sendmsg(msg);
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

	pcu_tx_pag_req(identity_lv, chan_needed);

	return 0;
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
		bts->si_buf[osmo_si][0] = 0x03;	/* C/R + EA */
		bts->si_buf[osmo_si][1] = 0x03;	/* UI frame */
		memset(bts->si_buf[osmo_si]+2, 0x2b, sizeof(sysinfo_buf_t)-2);
		memcpy(bts->si_buf[osmo_si]+2,
			TLVP_VAL(&tp, RSL_IE_L3_INFO), len);
		LOGP(DRSL, LOGL_INFO, " Rx RSL SACCH FILLING (SI%s)\n",
			get_value_string(osmo_sitype_strs, osmo_si));
	} else {
		bts->si_valid &= ~(1 << osmo_si);
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
	msg->l3h = (uint8_t *) TLVP_VAL(&tp, RSL_IE_FULL_IMM_ASS_INFO);
	msg->data = msg->l3h;
	msg->l2h = NULL;
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

/* 8.4.19 sending RF CHANnel RELease ACKnowledge */
int rsl_tx_rf_rel_ack(struct gsm_lchan *lchan)
{
	struct msgb *msg;
	uint8_t chan_nr = gsm_lchan2chan_nr(lchan);

	if (lchan->rel_act_kind != LCHAN_REL_ACT_RSL) {
		LOGP(DRSL, LOGL_NOTICE, "%s not sending REL ACK\n",
			gsm_lchan_name(lchan));
		return 0;
	}

	LOGP(DRSL, LOGL_NOTICE, "%s Tx RF CHAN REL ACK\n", gsm_lchan_name(lchan));

	/*
	 * Free the LAPDm resources now that the BTS
	 * has released all the resources.
	 */
	lapdm_channel_exit(&lchan->lapdm_ch);

	msg = rsl_msgb_alloc(sizeof(struct abis_rsl_dchan_hdr));
	if (!msg)
		return -ENOMEM;

	rsl_dch_push_hdr(msg, RSL_MT_RF_CHAN_REL_ACK, chan_nr);
	msg->trx = lchan->ts->trx;

	return abis_bts_rsl_sendmsg(msg);
}

/* 8.4.2 sending CHANnel ACTIVation ACKnowledge */
int rsl_tx_chan_act_ack(struct gsm_lchan *lchan, struct gsm_time *gtime)
{
	struct msgb *msg;
	uint8_t chan_nr = gsm_lchan2chan_nr(lchan);
	uint8_t ie[2];

	if (lchan->rel_act_kind != LCHAN_REL_ACT_RSL) {
		LOGP(DRSL, LOGL_NOTICE, "%s not sending CHAN ACT ACK\n",
			gsm_lchan_name(lchan));
		return 0;
	}

	LOGP(DRSL, LOGL_NOTICE, "%s Tx CHAN ACT ACK\n", gsm_lchan_name(lchan));

	msg = rsl_msgb_alloc(sizeof(struct abis_rsl_dchan_hdr));
	if (!msg)
		return -ENOMEM;

	gsm48_gen_starting_time(ie, gtime);
	msgb_tv_fixed_put(msg, RSL_IE_FRAME_NUMBER, 2, ie);
	rsl_dch_push_hdr(msg, RSL_MT_CHAN_ACTIV_ACK, chan_nr);
	msg->trx = lchan->ts->trx;

	/* since activation was successful, do some lchan initialization */
	lchan->meas.res_nr = 0;

	return abis_bts_rsl_sendmsg(msg);
}

/* 8.4.7 sending HANDOver DETection */
int rsl_tx_hando_det(struct gsm_lchan *lchan, uint8_t *ho_delay)
{
	struct msgb *msg;
	uint8_t chan_nr = gsm_lchan2chan_nr(lchan);

	LOGP(DRSL, LOGL_INFO, "Sending HANDOver DETect\n");

	msg = rsl_msgb_alloc(sizeof(struct abis_rsl_dchan_hdr));
	if (!msg)
		return -ENOMEM;

	/* 9.3.17 Access Delay */
	if (ho_delay)
		msgb_tv_put(msg, RSL_IE_ACCESS_DELAY, *ho_delay);

	rsl_dch_push_hdr(msg, RSL_MT_HANDO_DET, chan_nr);
	msg->trx = lchan->ts->trx;

	return abis_bts_rsl_sendmsg(msg);
}

/* 8.4.3 sending CHANnel ACTIVation Negative ACK */
int rsl_tx_chan_act_nack(struct gsm_lchan *lchan, uint8_t cause)
{
	struct msgb *msg;
	uint8_t chan_nr = gsm_lchan2chan_nr(lchan);

	if (lchan->rel_act_kind != LCHAN_REL_ACT_RSL) {
		LOGP(DRSL, LOGL_DEBUG, "%s not sending CHAN ACT NACK.\n",
			gsm_lchan_name(lchan));
		return 0;
	}

	LOGP(DRSL, LOGL_NOTICE,
		"%s Sending Channel Activated NACK: cause = 0x%02x\n",
		gsm_lchan_name(lchan), cause);

	msg = rsl_msgb_alloc(sizeof(struct abis_rsl_dchan_hdr));
	if (!msg)
		return -ENOMEM;

	/* 9.3.26 Cause */
	msgb_tlv_put(msg, RSL_IE_CAUSE, 1, &cause);
	rsl_dch_push_hdr(msg, RSL_MT_CHAN_ACTIV_NACK, chan_nr);
	msg->trx = lchan->ts->trx;

	return abis_bts_rsl_sendmsg(msg);
}

/* 8.4.4 sending CONNection FAILure */
int rsl_tx_conn_fail(struct gsm_lchan *lchan, uint8_t cause)
{
	struct msgb *msg;
	uint8_t chan_nr = gsm_lchan2chan_nr(lchan);

	LOGP(DRSL, LOGL_NOTICE,
		"%s Sending Connection Failure: cause = 0x%02x\n",
		gsm_lchan_name(lchan), cause);

	msg = rsl_msgb_alloc(sizeof(struct abis_rsl_dchan_hdr));
	if (!msg)
		return -ENOMEM;

	/* 9.3.26 Cause */
	msgb_tlv_put(msg, RSL_IE_CAUSE, 1, &cause);
	rsl_dch_push_hdr(msg, RSL_MT_CONN_FAIL, chan_nr);
	msg->trx = lchan->ts->trx;

	return abis_bts_rsl_sendmsg(msg);
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

	return abis_bts_rsl_sendmsg(nmsg);
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


static int encr_info2lchan(struct gsm_lchan *lchan,
			   const uint8_t *val, uint8_t len)
{
	int rc;
	struct gsm_bts_role_bts *btsb = bts_role_bts(lchan->ts->trx->bts);

	/* check if the encryption algorithm sent by BSC is supported! */
	rc = bts_supports_cipher(btsb, *val);
	if (rc != 1)
		return rc;

	/* length can be '1' in case of no ciphering */
	if (len < 1)
		return -EINVAL;

	lchan->encr.alg_id = *val++;
	lchan->encr.key_len = len -1;
	if (lchan->encr.key_len > sizeof(lchan->encr.key))
		lchan->encr.key_len = sizeof(lchan->encr.key);
	memcpy(lchan->encr.key, val, lchan->encr.key_len);

	return 0;
}

/* 8.4.1 CHANnel ACTIVation is received */
static int rsl_rx_chan_activ(struct msgb *msg)
{
	struct abis_rsl_dchan_hdr *dch = msgb_l2(msg);
	struct gsm_lchan *lchan = msg->lchan;
	struct rsl_ie_chan_mode *cm;
	struct tlv_parsed tp;
	uint8_t type;
	int rc;

	if (lchan->state != LCHAN_S_NONE) {
		LOGP(DRSL, LOGL_ERROR,
		     "%s: error lchan is not available state: %s.\n",
		     gsm_lchan_name(lchan), gsm_lchans_name(lchan->state));
		return rsl_tx_chan_act_nack(lchan, RSL_ERR_EQUIPMENT_FAIL);
	}

	rsl_tlv_parse(&tp, msgb_l3(msg), msgb_l3len(msg));

	/* 9.3.3 Activation Type */
	if (!TLVP_PRESENT(&tp, RSL_IE_ACT_TYPE)) {
		LOGP(DRSL, LOGL_NOTICE, "missing Activation Type\n");
		return rsl_tx_chan_act_nack(lchan, RSL_ERR_MAND_IE_ERROR);
	}
	type = *TLVP_VAL(&tp, RSL_IE_ACT_TYPE);

	/* 9.3.6 Channel Mode */
	if (!TLVP_PRESENT(&tp, RSL_IE_CHAN_MODE)) {
		LOGP(DRSL, LOGL_NOTICE, "missing Channel Mode\n");
		return rsl_tx_chan_act_nack(lchan, RSL_ERR_MAND_IE_ERROR);
	}
	cm = (struct rsl_ie_chan_mode *) TLVP_VAL(&tp, RSL_IE_CHAN_MODE);
	lchan_tchmode_from_cmode(lchan, cm);

	/* 9.3.7 Encryption Information */
	if (TLVP_PRESENT(&tp, RSL_IE_ENCR_INFO)) {
		uint8_t len = TLVP_LEN(&tp, RSL_IE_ENCR_INFO);
		const uint8_t *val = TLVP_VAL(&tp, RSL_IE_ENCR_INFO);

		if (encr_info2lchan(lchan, val, len) < 0)
			 return rsl_tx_error_report(msg->trx, RSL_ERR_IE_CONTENT);
	} else
		memset(&lchan->encr, 0, sizeof(lchan->encr));

	/* 9.3.9 Handover Reference */
	if ((type == RSL_ACT_INTER_ASYNC ||
	     type == RSL_ACT_INTER_SYNC) &&
	    TLVP_PRESENT(&tp, RSL_IE_HANDO_REF)) {
		lchan->ho.active = HANDOVER_ENABLED;
		lchan->ho.ref = *TLVP_VAL(&tp, RSL_IE_HANDO_REF);
	}

	/* 9.3.4 BS Power */
	if (TLVP_PRESENT(&tp, RSL_IE_BS_POWER))
		lchan->bs_power = *TLVP_VAL(&tp, RSL_IE_BS_POWER);
	/* 9.3.13 MS Power */
	if (TLVP_PRESENT(&tp, RSL_IE_MS_POWER))
		lchan->ms_power = *TLVP_VAL(&tp, RSL_IE_MS_POWER);
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
			lchan->si.buf[osmo_si][0] = 0x03;
			lchan->si.buf[osmo_si][1] = 0x03;
			memset(lchan->si.buf[osmo_si]+2, 0x2b, sizeof(sysinfo_buf_t)-2);
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
	if (TLVP_PRESENT(&tp, RSL_IE_MR_CONFIG)) {
		if (TLVP_LEN(&tp, RSL_IE_MR_CONFIG) > sizeof(lchan->mr_conf)) {
			LOGP(DRSL, LOGL_ERROR, "Error parsing MultiRate conf IE\n");
			return rsl_tx_error_report(msg->trx, RSL_ERR_IE_CONTENT);
		}
		memcpy(&lchan->mr_conf, TLVP_VAL(&tp, RSL_IE_MR_CONFIG),
		       TLVP_LEN(&tp, RSL_IE_MR_CONFIG));
		amr_parse_mr_conf(&lchan->tch.amr_mr, TLVP_VAL(&tp, RSL_IE_MR_CONFIG),
				  TLVP_LEN(&tp, RSL_IE_MR_CONFIG));
		amr_log_mr_conf(DRTP, LOGL_DEBUG, gsm_lchan_name(lchan),
				&lchan->tch.amr_mr);
		lchan->tch.last_cmr = AMR_CMR_NONE;
	}
	/* 9.3.53 MultiRate Control */
	/* 9.3.54 Supported Codec Types */

	LOGP(DRSL, LOGL_INFO, " chan_nr=0x%02x type=0x%02x mode=0x%02x\n",
		dch->chan_nr, type, lchan->tch_mode);

	/* actually activate the channel in the BTS */
	lchan->rel_act_kind = LCHAN_REL_ACT_RSL;
	rc = bts_model_rsl_chan_act(msg->lchan, &tp);
	if (rc < 0)
		return rsl_tx_chan_act_nack(lchan, -rc);

	return 0;
}

/* 8.4.14 RF CHANnel RELease is received */
static int rsl_rx_rf_chan_rel(struct gsm_lchan *lchan)
{
	int rc;

	if (lchan->abis_ip.rtp_socket) {
		rsl_tx_ipac_dlcx_ind(lchan, RSL_ERR_NORMAL_UNSPEC);
		osmo_rtp_socket_free(lchan->abis_ip.rtp_socket);
		lchan->abis_ip.rtp_socket = NULL;
		msgb_queue_flush(&lchan->dl_tch_queue);
	}

	/* release handover state */
	handover_reset(lchan);

	lchan->rel_act_kind = LCHAN_REL_ACT_RSL;
	rc = bts_model_rsl_chan_rel(lchan);

	return rc;
}

#ifdef FAKE_CIPH_MODE_COMPL
/* ugly hack to send a fake CIPH MODE COMPLETE back to the BSC */
#include <osmocom/gsm/protocol/gsm_04_08.h>
#include <osmocom/gsm/gsm48.h>
static int tx_ciph_mod_compl_hack(struct gsm_lchan *lchan, uint8_t link_id,
				  const char *imeisv)
{
	struct msgb *fake_msg;
	struct gsm48_hdr *g48h;
	uint8_t mid_buf[11];
	int rc;

	fake_msg = rsl_msgb_alloc(128);
	if (!fake_msg)
		return -ENOMEM;

	/* generate 04.08 RR message */
	g48h = (struct gsm48_hdr *) msgb_put(fake_msg, sizeof(*g48h));
	g48h->proto_discr = GSM48_PDISC_RR;
	g48h->msg_type = GSM48_MT_RR_CIPH_M_COMPL;

	/* add IMEISV, if requested */
	if (imeisv) {
		rc = gsm48_generate_mid_from_imsi(mid_buf, imeisv);
		if (rc > 0) {
			mid_buf[2] = (mid_buf[2] & 0xf8) | GSM_MI_TYPE_IMEISV;
			memcpy(msgb_put(fake_msg, rc), mid_buf, rc);
		}
	}

	rsl_rll_push_l3(fake_msg, RSL_MT_DATA_IND, gsm_lchan2chan_nr(lchan),
			link_id, 1);

	fake_msg->lchan = lchan;
	fake_msg->trx = lchan->ts->trx;

	/* send it back to the BTS */
	return abis_bts_rsl_sendmsg(fake_msg);
}

struct ciph_mod_compl {
	struct osmo_timer_list timer;
	struct gsm_lchan *lchan;
	int send_imeisv;
	uint8_t link_id;
};

static void cmc_timer_cb(void *data)
{
	struct ciph_mod_compl *cmc = data;
	const char *imeisv = NULL;

	LOGP(DRSL, LOGL_NOTICE,
	     "%s Sending FAKE CIPHERING MODE COMPLETE to BSC (Alg %u)\n",
	     gsm_lchan_name(cmc->lchan), cmc->lchan->encr.alg_id);

	if (cmc->send_imeisv)
		imeisv = "0123456789012345";

	/* We have no clue whatsoever that this lchan still exists! */
	tx_ciph_mod_compl_hack(cmc->lchan, cmc->link_id, imeisv);

	talloc_free(cmc);
}
#endif


/* 8.4.6 ENCRYPTION COMMAND */
static int rsl_rx_encr_cmd(struct msgb *msg)
{
	struct gsm_lchan *lchan = msg->lchan;
	struct abis_rsl_dchan_hdr *dch = msgb_l2(msg);
	struct tlv_parsed tp;
	uint8_t link_id;

	if (rsl_tlv_parse(&tp, msgb_l3(msg), msgb_l3len(msg)) < 0)
		return rsl_tx_error_report(msg->trx, RSL_ERR_IE_CONTENT);

	if (!TLVP_PRESENT(&tp, RSL_IE_ENCR_INFO) ||
	    !TLVP_PRESENT(&tp, RSL_IE_L3_INFO) ||
	    !TLVP_PRESENT(&tp, RSL_IE_LINK_IDENT))
		return rsl_tx_error_report(msg->trx, RSL_ERR_MAND_IE_ERROR);

	/* 9.3.7 Encryption Information */
	if (TLVP_PRESENT(&tp, RSL_IE_ENCR_INFO)) {
		uint8_t len = TLVP_LEN(&tp, RSL_IE_ENCR_INFO);
		const uint8_t *val = TLVP_VAL(&tp, RSL_IE_ENCR_INFO);

		if (encr_info2lchan(lchan, val, len) < 0)
			 return rsl_tx_error_report(msg->trx, RSL_ERR_IE_CONTENT);
	}

	/* 9.3.2 Link Identifier */
	link_id = *TLVP_VAL(&tp, RSL_IE_LINK_IDENT);

	/* we have to set msg->l3h as rsl_rll_push_l3 will use it to
	 * determine the length field of the L3_INFO IE */
	msg->l3h = (uint8_t *) TLVP_VAL(&tp, RSL_IE_L3_INFO);

	/* pop the RSL dchan header, but keep L3 TLV */
	msgb_pull(msg, msg->l3h - msg->data);

	/* push a fake RLL DATA REQ header */
	rsl_rll_push_l3(msg, RSL_MT_DATA_REQ, dch->chan_nr, link_id, 1);


#ifdef FAKE_CIPH_MODE_COMPL
	if (lchan->encr.alg_id != RSL_ENC_ALG_A5(0)) {
		struct ciph_mod_compl *cmc;
		struct gsm48_hdr *g48h = (struct gsm48_hdr *) msg->l3h;

		cmc = talloc_zero(NULL, struct ciph_mod_compl);
		if (g48h->data[0] & 0x10)
			cmc->send_imeisv = 1;
		cmc->lchan = lchan;
		cmc->link_id = link_id;
		cmc->timer.cb = cmc_timer_cb;
		cmc->timer.data = cmc;
		osmo_timer_schedule(&cmc->timer, 1, 0);

		/* FIXME: send fake CM SERVICE ACCEPT to MS */

		return 0;
	} else
#endif
	{
	LOGP(DRSL, LOGL_INFO, "%s Fwd RSL ENCR CMD (Alg %u) to LAPDm\n",
		gsm_lchan_name(lchan), lchan->encr.alg_id);
	/* hand it into RSLms for transmission of L3_INFO to the MS */
	lapdm_rslms_recvmsg(msg, &lchan->lapdm_ch);
	/* return 1 to make sure the msgb is not free'd */
	return 1;
	}
}

/* 8.4.11 MODE MODIFY NEGATIVE ACKNOWLEDGE */
static int rsl_tx_mode_modif_nack(struct gsm_lchan *lchan, uint8_t cause)
{
	struct msgb *msg;
	uint8_t chan_nr = gsm_lchan2chan_nr(lchan);

	LOGP(DRSL, LOGL_NOTICE, "%s Tx MODE MODIFY NACK (cause = 0x%02x)\n",
	     gsm_lchan_name(lchan), cause);

	msg = rsl_msgb_alloc(sizeof(struct abis_rsl_dchan_hdr));
	if (!msg)
		return -ENOMEM;

	msg->len = 0;
	msg->data = msg->tail = msg->l3h;

	/* 9.3.26 Cause */
	msgb_tlv_put(msg, RSL_IE_CAUSE, 1, &cause);
	rsl_dch_push_hdr(msg, RSL_MT_MODE_MODIFY_NACK, chan_nr);
	msg->lchan = lchan;

	return abis_bts_rsl_sendmsg(msg);
}

/* 8.4.10 MODE MODIFY ACK */
static int rsl_tx_mode_modif_ack(struct gsm_lchan *lchan)
{
	struct msgb *msg;
	uint8_t chan_nr = gsm_lchan2chan_nr(lchan);

	LOGP(DRSL, LOGL_INFO, "%s Tx MODE MODIF ACK\n", gsm_lchan_name(lchan));

	msg = rsl_msgb_alloc(sizeof(struct abis_rsl_dchan_hdr));
	if (!msg)
		return -ENOMEM;

	rsl_dch_push_hdr(msg, RSL_MT_MODE_MODIFY_ACK, chan_nr);
	msg->trx = lchan->ts->trx;

	return abis_bts_rsl_sendmsg(msg);
}

/* 8.4.9 MODE MODIFY */
static int rsl_rx_mode_modif(struct msgb *msg)
{
	struct gsm_lchan *lchan = msg->lchan;
	struct rsl_ie_chan_mode *cm;
	struct tlv_parsed tp;
	int rc;

	rsl_tlv_parse(&tp, msgb_l3(msg), msgb_l3len(msg));

	/* 9.3.6 Channel Mode */
	if (!TLVP_PRESENT(&tp, RSL_IE_CHAN_MODE)) {
		LOGP(DRSL, LOGL_NOTICE, "missing Channel Mode\n");
		msgb_free(msg);
		return rsl_tx_mode_modif_nack(lchan, RSL_ERR_MAND_IE_ERROR);
	}
	cm = (struct rsl_ie_chan_mode *) TLVP_VAL(&tp, RSL_IE_CHAN_MODE);
	lchan_tchmode_from_cmode(lchan, cm);

	/* 9.3.7 Encryption Information */
	if (TLVP_PRESENT(&tp, RSL_IE_ENCR_INFO)) {
		uint8_t len = TLVP_LEN(&tp, RSL_IE_ENCR_INFO);
		const uint8_t *val = TLVP_VAL(&tp, RSL_IE_ENCR_INFO);

		if (encr_info2lchan(lchan, val, len) < 0)
			 return rsl_tx_error_report(msg->trx, RSL_ERR_IE_CONTENT);
	}

	/* 9.3.45 Main channel reference */

	/* 9.3.52 MultiRate Configuration */
	if (TLVP_PRESENT(&tp, RSL_IE_MR_CONFIG)) {
		if (TLVP_LEN(&tp, RSL_IE_MR_CONFIG) > sizeof(lchan->mr_conf)) {
			LOGP(DRSL, LOGL_ERROR, "Error parsing MultiRate conf IE\n");
			return rsl_tx_error_report(msg->trx, RSL_ERR_IE_CONTENT);
		}
		memcpy(&lchan->mr_conf, TLVP_VAL(&tp, RSL_IE_MR_CONFIG),
			TLVP_LEN(&tp, RSL_IE_MR_CONFIG));
		amr_parse_mr_conf(&lchan->tch.amr_mr, TLVP_VAL(&tp, RSL_IE_MR_CONFIG),
				  TLVP_LEN(&tp, RSL_IE_MR_CONFIG));
		amr_log_mr_conf(DRTP, LOGL_DEBUG, gsm_lchan_name(lchan),
				&lchan->tch.amr_mr);
		lchan->tch.last_cmr = AMR_CMR_NONE;
	}
	/* 9.3.53 MultiRate Control */
	/* 9.3.54 Supported Codec Types */

	rc = bts_model_rsl_mode_modify(msg->lchan);

	/* FIXME: delay this until L1 says OK? */
	rsl_tx_mode_modif_ack(msg->lchan);

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
		lchan->si.buf[osmo_si][0] = 0x03;
		lchan->si.buf[osmo_si][1] = 0x03;
		memset(lchan->si.buf[osmo_si]+2, 0x2b, sizeof(sysinfo_buf_t)-2);
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

/*
 * ip.access related messages
 */

int rsl_tx_ipac_dlcx_ind(struct gsm_lchan *lchan, uint8_t cause)
{
	struct msgb *nmsg;

	LOGP(DRSL, LOGL_NOTICE, "%s Sending RTP delete indication: cause=%d\n",
		gsm_lchan_name(lchan), cause);

	nmsg = rsl_msgb_alloc(sizeof(struct abis_rsl_dchan_hdr));
	if (!nmsg)
		return -ENOMEM;

	msgb_tlv_put(nmsg, RSL_IE_CAUSE, 1, &cause);
	rsl_ipa_push_hdr(nmsg, RSL_MT_IPAC_DLCX_IND, gsm_lchan2chan_nr(lchan));

	nmsg->trx = lchan->ts->trx;

	return abis_bts_rsl_sendmsg(nmsg);
}

/* transmit an CRCX ACK for the lchan */
static int rsl_tx_ipac_XXcx_ack(struct gsm_lchan *lchan, int inc_pt2,
				  uint8_t orig_msgt)
{
	struct msgb *msg;
	uint8_t chan_nr = gsm_lchan2chan_nr(lchan);
	const char *name;
	struct in_addr ia;

	if (orig_msgt == RSL_MT_IPAC_CRCX)
		name = "CRCX";
	else
		name = "MDCX";

	ia.s_addr = htonl(lchan->abis_ip.bound_ip);
	LOGP(DRSL, LOGL_INFO, "%s RSL Tx IPAC_%s_ACK (local %s:%u, ",
	     gsm_lchan_name(lchan), name,
	     inet_ntoa(ia), lchan->abis_ip.bound_port);
	ia.s_addr = htonl(lchan->abis_ip.connect_ip);
	LOGPC(DRSL, LOGL_INFO, "remote %s:%u)\n",
		inet_ntoa(ia), lchan->abis_ip.connect_port);

	msg = rsl_msgb_alloc(sizeof(struct abis_rsl_dchan_hdr));
	if (!msg)
		return -ENOMEM;


	/* Connection ID */
	msgb_tv16_put(msg, RSL_IE_IPAC_CONN_ID, htons(lchan->abis_ip.conn_id));

	/* locally bound IP */
	msgb_v_put(msg, RSL_IE_IPAC_LOCAL_IP);
	msgb_put_u32(msg, lchan->abis_ip.bound_ip);

	/* locally bound port */
	msgb_tv16_put(msg, RSL_IE_IPAC_LOCAL_PORT,
		      lchan->abis_ip.bound_port);

	if (inc_pt2) {
		/* RTP Payload Type 2 */
		msgb_tv_put(msg, RSL_IE_IPAC_RTP_PAYLOAD2,
					lchan->abis_ip.rtp_payload2);
	}

	/* push the header in front */
	rsl_ipa_push_hdr(msg, orig_msgt + 1, chan_nr);
	msg->trx = lchan->ts->trx;

	return abis_bts_rsl_sendmsg(msg);
}

static int rsl_tx_ipac_dlcx_ack(struct gsm_lchan *lchan, int inc_conn_id)
{
	struct msgb *msg;
	uint8_t chan_nr = gsm_lchan2chan_nr(lchan);

	LOGP(DRSL, LOGL_INFO, "%s RSL Tx IPAC_DLCX_ACK\n",
		gsm_lchan_name(lchan));

	msg = rsl_msgb_alloc(sizeof(struct abis_rsl_dchan_hdr));
	if (!msg)
		return -ENOMEM;

	if (inc_conn_id)
		msgb_tv_put(msg, RSL_IE_IPAC_CONN_ID, lchan->abis_ip.conn_id);

	rsl_ipa_push_hdr(msg, RSL_MT_IPAC_DLCX_ACK, chan_nr);
	msg->trx = lchan->ts->trx;

	return abis_bts_rsl_sendmsg(msg);
}

static int rsl_tx_ipac_dlcx_nack(struct gsm_lchan *lchan, int inc_conn_id,
				 uint8_t cause)
{
	struct msgb *msg;
	uint8_t chan_nr = gsm_lchan2chan_nr(lchan);

	LOGP(DRSL, LOGL_INFO, "%s RSL Tx IPAC_DLCX_NACK\n",
		gsm_lchan_name(lchan));

	msg = rsl_msgb_alloc(sizeof(struct abis_rsl_dchan_hdr));
	if (!msg)
		return -ENOMEM;

	if (inc_conn_id)
		msgb_tv_put(msg, RSL_IE_IPAC_CONN_ID, lchan->abis_ip.conn_id);

	msgb_tlv_put(msg, RSL_IE_CAUSE, 1, &cause);

	rsl_ipa_push_hdr(msg, RSL_MT_IPAC_DLCX_NACK, chan_nr);
	msg->trx = lchan->ts->trx;

	return abis_bts_rsl_sendmsg(msg);

}


/* transmit an CRCX NACK for the lchan */
static int tx_ipac_XXcx_nack(struct gsm_lchan *lchan, uint8_t cause,
			     int inc_ipport, uint8_t orig_msgtype)
{
	struct msgb *msg;
	uint8_t chan_nr = gsm_lchan2chan_nr(lchan);

	/* FIXME: allocate new msgb and copy old over */
	LOGP(DRSL, LOGL_NOTICE, "%s RSL Tx IPAC_BIND_NACK\n",
		gsm_lchan_name(lchan));

	msg = rsl_msgb_alloc(sizeof(struct abis_rsl_dchan_hdr));
	if (!msg)
		return -ENOMEM;

	if (inc_ipport) {
		/* remote IP */
		msgb_v_put(msg, RSL_IE_IPAC_REMOTE_IP);
		msgb_put_u32(msg, lchan->abis_ip.connect_ip);

		/* remote port */
		msgb_tv16_put(msg, RSL_IE_IPAC_REMOTE_PORT,
				htons(lchan->abis_ip.connect_port));
	}

	/* 9.3.26 Cause */
	msgb_tlv_put(msg, RSL_IE_CAUSE, 1, &cause);

	/* push the header in front */
	rsl_ipa_push_hdr(msg, orig_msgtype + 2, chan_nr);
	msg->trx = lchan->ts->trx;

	return abis_bts_rsl_sendmsg(msg);
}

static char *get_rsl_local_ip(struct gsm_bts_trx *trx)
{
	struct e1inp_ts *ts = trx->rsl_link->ts;
	struct sockaddr_storage ss;
	socklen_t sa_len = sizeof(ss);
	static char hostbuf[256];
	int rc;

	rc = getsockname(ts->driver.ipaccess.fd.fd, (struct sockaddr *) &ss,
			 &sa_len);
	if (rc < 0)
		return NULL;

	rc = getnameinfo((struct sockaddr *)&ss, sa_len,
			 hostbuf, sizeof(hostbuf), NULL, 0,
			 NI_NUMERICHOST);
	if (rc < 0)
		return NULL;

	return hostbuf;
}

static int rsl_rx_ipac_XXcx(struct msgb *msg)
{
	struct abis_rsl_dchan_hdr *dch = msgb_l2(msg);
	struct tlv_parsed tp;
	struct gsm_lchan *lchan = msg->lchan;
	struct gsm_bts_role_bts *btsb = bts_role_bts(msg->lchan->ts->trx->bts);
	const uint8_t *payload_type, *speech_mode, *payload_type2;
	uint32_t connect_ip = 0;
	uint16_t connect_port = 0;
	int rc, inc_ip_port = 0, port;
	char *name;
	struct in_addr ia;

	if (dch->c.msg_type == RSL_MT_IPAC_CRCX)
		name = "CRCX";
	else
		name = "MDCX";

	/* check the kind of channel and reject */
	if (lchan->type != GSM_LCHAN_TCH_F && lchan->type != GSM_LCHAN_TCH_H)
		return tx_ipac_XXcx_nack(lchan, 0x52,
					 0, dch->c.msg_type);

	rc = rsl_tlv_parse(&tp, msgb_l3(msg), msgb_l3len(msg));
	if (rc < 0)
		return tx_ipac_XXcx_nack(lchan, RSL_ERR_MAND_IE_ERROR,
					 0, dch->c.msg_type);

	/* any of these can be NULL!! */
	speech_mode = TLVP_VAL(&tp, RSL_IE_IPAC_SPEECH_MODE);
	payload_type = TLVP_VAL(&tp, RSL_IE_IPAC_RTP_PAYLOAD);
	payload_type2 = TLVP_VAL(&tp, RSL_IE_IPAC_RTP_PAYLOAD2);

	if (TLVP_PRESENT(&tp, RSL_IE_IPAC_REMOTE_IP))
		connect_ip = tlvp_val32_unal(&tp, RSL_IE_IPAC_REMOTE_IP);
	else
		LOGP(DRSL, LOGL_NOTICE, "CRCX does not specify a remote IP\n");

	if (TLVP_PRESENT(&tp, RSL_IE_IPAC_REMOTE_PORT))
		connect_port = tlvp_val16_unal(&tp, RSL_IE_IPAC_REMOTE_PORT);
	else
		LOGP(DRSL, LOGL_NOTICE, "CRCX does not specify a remote port\n");

	if (dch->c.msg_type == RSL_MT_IPAC_CRCX && connect_ip && connect_port)
		inc_ip_port = 1;

	if (payload_type && payload_type2) {
		LOGP(DRSL, LOGL_ERROR, "%s Rx RSL IPAC %s, "
			"RTP_PT and RTP_PT2 in same msg !?!\n",
			gsm_lchan_name(lchan), name);
		return tx_ipac_XXcx_nack(lchan, RSL_ERR_MAND_IE_ERROR,
					 inc_ip_port, dch->c.msg_type);
	}

	if (dch->c.msg_type == RSL_MT_IPAC_CRCX) {
		char *ipstr = NULL;
		if (lchan->abis_ip.rtp_socket) {
			LOGP(DRSL, LOGL_ERROR, "%s Rx RSL IPAC CRCX, "
				"but we already have socket!\n",
				gsm_lchan_name(lchan));
			return tx_ipac_XXcx_nack(lchan, RSL_ERR_RES_UNAVAIL,
						 inc_ip_port, dch->c.msg_type);
		}
		/* FIXME: select default value depending on speech_mode */
		//if (!payload_type)
		lchan->abis_ip.rtp_socket = osmo_rtp_socket_create(lchan->ts->trx,
								OSMO_RTP_F_POLL);
		if (!lchan->abis_ip.rtp_socket) {
			LOGP(DRSL, LOGL_ERROR,
			     "%s IPAC Failed to create RTP/RTCP sockets\n",
			     gsm_lchan_name(lchan));
			return tx_ipac_XXcx_nack(lchan, RSL_ERR_RES_UNAVAIL,
						 inc_ip_port, dch->c.msg_type);
		}
		osmo_rtp_socket_set_param(lchan->abis_ip.rtp_socket,
					  OSMO_RTP_P_JITBUF,
					  btsb->rtp_jitter_buf_ms);
		lchan->abis_ip.rtp_socket->priv = lchan;
		lchan->abis_ip.rtp_socket->rx_cb = &bts_model_rtp_rx_cb;

		if (connect_ip && connect_port) {
			/* if CRCX specifies a remote IP, we can bind()
			 * here to 0.0.0.0 and wait for the connect()
			 * below, after which the kernel will have
			 * selected the local IP address.  */
			ipstr = "0.0.0.0";
		} else {
			/* if CRCX does not specify a remote IP, we will
			 * not do any connect() below, and thus the
			 * local socket will remain bound to 0.0.0.0 -
			 * which however we cannot legitimately report
			 * back to the BSC in the CRCX_ACK */
			ipstr = get_rsl_local_ip(lchan->ts->trx);
		}
		rc = osmo_rtp_socket_bind(lchan->abis_ip.rtp_socket,
					  ipstr, -1);
		if (rc < 0) {
			LOGP(DRSL, LOGL_ERROR,
			     "%s IPAC Failed to bind RTP/RTCP sockets\n",
			     gsm_lchan_name(lchan));
			osmo_rtp_socket_free(lchan->abis_ip.rtp_socket);
			lchan->abis_ip.rtp_socket = NULL;
			msgb_queue_flush(&lchan->dl_tch_queue);
			return tx_ipac_XXcx_nack(lchan, RSL_ERR_RES_UNAVAIL,
						 inc_ip_port, dch->c.msg_type);
		}
		/* FIXME: multiplex connection, BSC proxy */
	} else {
		/* MDCX */
		if (!lchan->abis_ip.rtp_socket) {
			LOGP(DRSL, LOGL_ERROR, "%s Rx RSL IPAC MDCX, "
				"but we have no RTP socket!\n",
				gsm_lchan_name(lchan));
			return tx_ipac_XXcx_nack(lchan, RSL_ERR_RES_UNAVAIL,
						 inc_ip_port, dch->c.msg_type);
		}
	}


	/* Special rule: If connect_ip == 0.0.0.0, use RSL IP
	 * address */
	if (connect_ip == 0) {
		struct e1inp_sign_link *sign_link =
					lchan->ts->trx->rsl_link;

		ia.s_addr = htonl(get_signlink_remote_ip(sign_link));
	} else
		ia.s_addr = connect_ip;
	rc = osmo_rtp_socket_connect(lchan->abis_ip.rtp_socket,
				     inet_ntoa(ia), ntohs(connect_port));
	if (rc < 0) {
		LOGP(DRSL, LOGL_ERROR,
		     "%s Failed to connect RTP/RTCP sockets\n",
		     gsm_lchan_name(lchan));
		osmo_rtp_socket_free(lchan->abis_ip.rtp_socket);
		lchan->abis_ip.rtp_socket = NULL;
		msgb_queue_flush(&lchan->dl_tch_queue);
		return tx_ipac_XXcx_nack(lchan, RSL_ERR_RES_UNAVAIL,
					 inc_ip_port, dch->c.msg_type);
	}
	/* save IP address and port number */
	lchan->abis_ip.connect_ip = ntohl(ia.s_addr);
	lchan->abis_ip.connect_port = ntohs(connect_port);

	rc = osmo_rtp_get_bound_ip_port(lchan->abis_ip.rtp_socket,
					&lchan->abis_ip.bound_ip,
					&port);
	if (rc < 0)
		LOGP(DRSL, LOGL_ERROR, "%s IPAC cannot obtain "
		     "locally bound IP/port: %d\n",
		     gsm_lchan_name(lchan), rc);
	lchan->abis_ip.bound_port = port;

	/* Everything has succeeded, we can store new values in lchan */
	if (payload_type) {
		lchan->abis_ip.rtp_payload = *payload_type;
		if (lchan->abis_ip.rtp_socket)
			osmo_rtp_socket_set_pt(lchan->abis_ip.rtp_socket,
						*payload_type);
	}
	if (payload_type2) {
		lchan->abis_ip.rtp_payload2 = *payload_type2;
		if (lchan->abis_ip.rtp_socket)
			osmo_rtp_socket_set_pt(lchan->abis_ip.rtp_socket,
						*payload_type2);
	}
	if (speech_mode)
		lchan->abis_ip.speech_mode = *speech_mode;

	/* FIXME: CSD, jitterbuffer, compression */

	return rsl_tx_ipac_XXcx_ack(lchan, payload_type2 ? 1 : 0,
				    dch->c.msg_type);
}

static int rsl_rx_ipac_dlcx(struct msgb *msg)
{
	struct tlv_parsed tp;
	struct gsm_lchan *lchan = msg->lchan;
	int rc, inc_conn_id = 0;

	rc = rsl_tlv_parse(&tp, msgb_l3(msg), msgb_l3len(msg));
	if (rc < 0)
		return rsl_tx_ipac_dlcx_nack(lchan, 0, RSL_ERR_MAND_IE_ERROR);

	if (TLVP_PRESENT(&tp, RSL_IE_IPAC_CONN_ID))
		inc_conn_id = 1;

	osmo_rtp_socket_free(lchan->abis_ip.rtp_socket);
	lchan->abis_ip.rtp_socket = NULL;
	msgb_queue_flush(&lchan->dl_tch_queue);

	return rsl_tx_ipac_dlcx_ack(lchan, inc_conn_id);
}

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
		msgb_free(msg);
		return report_error(trx);
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
	struct msgb *msg;
	uint8_t chan_nr = gsm_lchan2chan_nr(lchan);

	LOGP(DRSL, LOGL_NOTICE, "%s Tx MEAS RES\n", gsm_lchan_name(lchan));

	msg = rsl_msgb_alloc(sizeof(struct abis_rsl_dchan_hdr));
	if (!msg)
		return -ENOMEM;

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

	return abis_bts_rsl_sendmsg(msg);
}

/* call-back for LAPDm code, called when it wants to send msgs UP */
int lapdm_rll_tx_cb(struct msgb *msg, struct lapdm_entity *le, void *ctx)
{
	struct gsm_lchan *lchan = ctx;
	struct abis_rsl_common_hdr *rh = msgb_l2(msg);

	if (lchan->state != LCHAN_S_ACTIVE) {
		LOGP(DRSL, LOGL_INFO, "%s(%s) is not active . Dropping message.\n",
			gsm_lchan_name(lchan), gsm_lchans_name(lchan->state));
		msgb_free(msg);
		return 0;
	}

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

		return abis_bts_rsl_sendmsg(msg);
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
		msgb_free(msg);
		return report_error(trx);
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
		msgb_free(msg);
		return report_error(trx);
	}

	LOGP(DRSL, LOGL_INFO, "%s Rx RSL %s\n", gsm_lchan_name(msg->lchan),
		rsl_msg_name(dch->c.msg_type));

	switch (dch->c.msg_type) {
	case RSL_MT_CHAN_ACTIV:
		ret = rsl_rx_chan_activ(msg);
		break;
	case RSL_MT_RF_CHAN_REL:
		ret = rsl_rx_rf_chan_rel(msg->lchan);
		break;
	case RSL_MT_SACCH_INFO_MODIFY:
		ret = rsl_rx_sacch_inf_mod(msg);
		break;
	case RSL_MT_DEACTIVATE_SACCH:
		ret = bts_model_rsl_deact_sacch(msg->lchan);
		break;
	case RSL_MT_ENCR_CMD:
		ret = rsl_rx_encr_cmd(msg);
		break;
	case RSL_MT_MODE_MODIFY_REQ:
		ret = rsl_rx_mode_modif(msg);
		break;
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

	msg->lchan = rsl_lchan_lookup(trx, dch->chan_nr);
	if (!msg->lchan) {
		LOGP(DRSL, LOGL_ERROR, "Rx RSL %s for unknow lchan\n",
			rsl_msg_name(dch->c.msg_type));
		msgb_free(msg);
		return report_error(trx);
	}

	LOGP(DRSL, LOGL_INFO, "%s Rx RSL %s\n", gsm_lchan_name(msg->lchan),
		rsl_ipac_msg_name(dch->c.msg_type));

	switch (dch->c.msg_type) {
	case RSL_MT_IPAC_CRCX:
	case RSL_MT_IPAC_MDCX:
		ret = rsl_rx_ipac_XXcx(msg);
		break;
	case RSL_MT_IPAC_DLCX:
		ret = rsl_rx_ipac_dlcx(msg);
		break;
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
