/*
 * (C) 2011 by Andreas Eversberg <jolly@eversberg.eu>
 *
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

/*
 * Radio Link Layer Messages
 */

#include <stdio.h>
#include <errno.h>
#include <sys/types.h>
#include <arpa/inet.h>

#include <osmocore/msgb.h>
#include <osmocore/rsl.h>
#include <osmocore/protocol/gsm_12_21.h>
#include <osmo-bts/logging.h>
//#include <osmocom/bb/common/osmocom_data.h>
#include <osmo-bts/abis.h>
#include <osmo-bts/rtp.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/rsl.h>
#include <osmo-bts/oml.h>
#include <osmo-bts/support.h>

static int rsl_tx_error_report(struct osmobts_trx *trx, uint8_t cause);

/*
 * support
 */

static uint8_t bts_si_list[BTS_SI_NUM] = BTS_SI_LIST;

struct osmobts_lchan *rsl_get_chan(struct osmobts_trx *trx, uint8_t chan_nr)
{
	uint8_t ts_nr = chan_nr & 0x07;
	uint8_t cbits = chan_nr >> 3;
	struct bts_support *sup = &bts_support;
	struct osmobts_slot *slot = &trx->slot[ts_nr];
	struct osmobts_lchan *lchan = NULL;

	if (!slot->tx_ms) {
		LOGP(DRSL, LOGL_NOTICE, "Given slot not available: %d\n", ts_nr);
		return NULL;
	}

	if (cbits == 0x01) {
		/* TCH/F */
		if (!sup->chan_comb[NM_CHANC_TCHFull]) {
			LOGP(DRSL, LOGL_NOTICE, "TCH/F not not supported on slot: %d\n", ts_nr);
			return NULL;
		}
		if (slot->chan_comb != NM_CHANC_TCHFull) {
			LOGP(DRSL, LOGL_NOTICE, "Given channel type not TCH/F: %d\n", ts_nr);
			return NULL;
		}
		lchan = slot->lchan[0];
	} else if ((cbits & 0x1e) == 0x02) {
		/* TCH/H */
		if (!sup->chan_comb[NM_CHANC_TCHHalf]) {
			LOGP(DRSL, LOGL_NOTICE, "TCH/H not not supported on slot: %d\n", ts_nr);
			return NULL;
		}
		if (slot->chan_comb != NM_CHANC_TCHHalf) {
			LOGP(DRSL, LOGL_NOTICE, "Given channel type not TCH/H: %d\n", ts_nr);
			return NULL;
		}
		lchan = slot->lchan[cbits & 0x01];;
	} else if ((cbits & 0x1c) == 0x04) {
		/* BCCH+SDCCH4 */
		if (!sup->chan_comb[NM_CHANC_BCCHComb]) {
			LOGP(DRSL, LOGL_NOTICE, "Combined BCCH+SDCCH/4 not not supported on slot: %d\n", ts_nr);
			return NULL;
		}
		if (slot->chan_comb != NM_CHANC_BCCHComb) {
			LOGP(DRSL, LOGL_NOTICE, "Given channel type not Combined BCCH+SDCCH/4: %d\n", ts_nr);
			return NULL;
		}
		lchan = slot->lchan[cbits & 0x03];;
	} else if ((cbits & 0x18) == 0x08) {
		/* SDCCH8 */
		if (!sup->chan_comb[NM_CHANC_SDCCH]) {
			LOGP(DRSL, LOGL_NOTICE, "SDCCH/8 not not supported on slot: %d\n", ts_nr);
			return NULL;
		}
		if (slot->chan_comb != NM_CHANC_SDCCH) {
			LOGP(DRSL, LOGL_NOTICE, "Given channel type not SDCCH/8: %d\n", ts_nr);
			return NULL;
		}
		lchan = slot->lchan[cbits & 0x07];
	} else {
		LOGP(DRSL, LOGL_NOTICE, "Given chan_nr unknown: %d\n", chan_nr);
		return NULL;
	}

	if (!lchan)
		LOGP(DRSL, LOGL_ERROR, "Lchan not created.\n");

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
	dch->chan_nr = chan_nr;
}


/*
 * TRX related messages
 */

/* 8.6.4 sending ERROR REPORT */
static int rsl_tx_error_report(struct osmobts_trx *trx, uint8_t cause)
{
	struct msgb *nmsg;
	uint8_t *ie;

	LOGP(DRSL, LOGL_NOTICE, "Sending Error Report: cause = 0x%02x\n", cause);

	nmsg = rsl_msgb_alloc(sizeof(struct abis_rsl_common_hdr));
	if (!nmsg)
		return -ENOMEM;
	ie = msgb_put(nmsg, 3);
	ie[0] = RSL_IE_CAUSE;
	ie[1] = 1;
	ie[2] = cause;
	rsl_trx_push_hdr(nmsg, RSL_MT_ERROR_REPORT);
	abis_push_ipa(nmsg, IPA_PROTO_RSL);

	return abis_tx(&trx->link, nmsg);
}

/* 8.6.1 sending RF RESOURCE INDICATION */
int rsl_tx_rf_res(struct osmobts_trx *trx)
{
	struct msgb *nmsg;

	LOGP(DRSL, LOGL_INFO, "Sending RF RESource INDication\n");

	nmsg = rsl_msgb_alloc(sizeof(struct abis_rsl_common_hdr));
	if (!nmsg)
		return -ENOMEM;
	// FIXME: add interference levels of TRX
	rsl_trx_push_hdr(nmsg, RSL_MT_RF_RES_IND);
	abis_push_ipa(nmsg, IPA_PROTO_RSL);

	return abis_tx(&trx->link, nmsg);
}

/* 
 * common channel releated messages
 */

/* 8.5.1 BCCH INFOrmation is received */
static int rsl_rx_bcch_info(struct osmobts_trx *trx, struct msgb *msg)
{
	struct tlv_parsed tp;
	uint8_t si;
	int i;

	LOGP(DRSL, LOGL_INFO, "RSL BCCH Information:\n");

	rsl_tlv_parse(&tp, msgb_l3(msg), msgb_l3len(msg));

	/* 9.3.30 System Info Type */
	if (!TLVP_PRESENT(&tp, RSL_IE_SYSINFO_TYPE)) {
		return rsl_tx_error_report(trx, RSL_ERR_MAND_IE_ERROR);
	}
	si = *TLVP_VAL(&tp, RSL_IE_SYSINFO_TYPE);
	i = 0;
	while(i < BTS_SI_NUM) {
		if (bts_si_list[i] == si)
			break;
		i++;
	}
	if (i == BTS_SI_NUM) {
		LOGP(DRSL, LOGL_NOTICE, " SI 0x%02x not supported.\n", si);
		return rsl_tx_error_report(trx, RSL_ERR_IE_CONTENT);
	}
	/* 9.3.39 Full BCCH Information */
	if (TLVP_PRESENT(&tp, RSL_IE_FULL_BCCH_INFO)) {
		trx->si.flags[i] |= BTS_SI_USE;
		memcpy(trx->si.si[i], TLVP_VAL(&tp, RSL_IE_FULL_BCCH_INFO), 23);
		LOGP(DRSL, LOGL_INFO, " Got new SYSTEM INFORMATION 0x%02x.\n",si);
	} else if (TLVP_PRESENT(&tp, RSL_IE_L3_INFO)) {
		trx->si.flags[i] |= BTS_SI_USE;
		memcpy(trx->si.si[i], TLVP_VAL(&tp, RSL_IE_L3_INFO), 23);
		LOGP(DRSL, LOGL_INFO, " Got new SYSTEM INFORMATION 0x%02x.\n",si);
	} else {
		trx->si.flags[i] &= ~BTS_SI_USE;
		LOGP(DRSL, LOGL_INFO, " Removing SYSTEM INFORMATION 0x%02x.\n",si);
	}
	trx->si.flags[i] |= BTS_SI_NEW;
	bts_new_si(trx);

	return 0;
}

/* 8.5.6 IMMEDIATE ASSIGN COMMAND is received */
static int rsl_rx_imm_ass(struct osmobts_trx *trx, struct msgb *msg)
{
	struct tlv_parsed tp;
	uint8_t *data;

	LOGP(DRSL, LOGL_INFO, "Immidiate Assignment Command:\n");

	rsl_tlv_parse(&tp, msgb_l3(msg), msgb_l3len(msg));

	/* 9.3.30 System Info Type */
	if (!TLVP_PRESENT(&tp, RSL_IE_FULL_IMM_ASS_INFO)) {
		return rsl_tx_error_report(trx, RSL_ERR_MAND_IE_ERROR);
	}
	data = (uint8_t *) TLVP_VAL(&tp, RSL_IE_FULL_IMM_ASS_INFO);
	LOGP(DRSL, LOGL_INFO, " length = %d\n", data[-1]);

#warning HACK
	{
		struct msgb *nmsg = rsl_msgb_alloc(64);
		struct l1ctl_info_dl *dl;
		uint8_t lupd[23] = {0x01,0x03f,0x3d,
				0x05,0x08,0x12,0x62,0xf2,0x10,0x31,0x04,0x33,0x05,0xf4,0x87,0x16,0xb3,0xf0,
				0x2b, 0x2b, 0x2b, 0x2b, 0x2b};

		memcpy(msgb_put(nmsg, sizeof(lupd)), lupd, sizeof(lupd));

		nmsg->l3h = nmsg->data;
		dl = (struct l1ctl_info_dl *)(nmsg->l1h = msgb_push(nmsg, sizeof(*dl)));
		dl->chan_nr = trx->slot[2].lchan[0]->chan_nr;
		dl->link_id = 0x00;
		msgb_push(nmsg, sizeof(struct l1ctl_hdr));
		printf("%p '%s'\n", trx->slot[2].tx_ms, trx->slot[2].tx_ms->ms.name);
		rx_ph_data_ind(&trx->slot[2].tx_ms->ms, nmsg);
	}

	return 0;
}

/*
 * dedicated channel related messages
 */

/* 8.4.19 sebdubg RF CHANnel RELease ACKnowledge */
static int rsl_tx_rf_rel_ack(struct osmobts_trx *trx, struct msgb *msg, uint8_t t1, uint8_t t2, uint8_t t3)
{
	struct abis_rsl_dchan_hdr *dch = msgb_l2(msg);
	uint8_t chan_nr = dch->chan_nr;

	LOGP(DRSL, LOGL_NOTICE, "Sending Channel Release ACK\n");

	msg->len = 0;
	msg->data = msg->tail = msg->l3h;

	rsl_dch_push_hdr(msg, RSL_MT_RF_CHAN_REL_ACK, chan_nr);
	abis_push_ipa(msg, IPA_PROTO_RSL);

	return abis_tx(&trx->link, msg);
}

/* 8.4.2 sending CHANnel ACTIVation ACKnowledge */
static int rsl_tx_chan_ack(struct osmobts_trx *trx, struct msgb *msg, uint8_t t1, uint8_t t2, uint8_t t3)
{
	struct abis_rsl_dchan_hdr *dch = msgb_l2(msg);
	uint8_t *ie;
	uint8_t chan_nr = dch->chan_nr;

	LOGP(DRSL, LOGL_NOTICE, "Sending Channel Activated ACK\n");

	msg->len = 0;
	msg->data = msg->tail = msg->l3h;

	ie = msgb_put(msg, 3);
	ie[0] = RSL_IE_FRAME_NUMBER;
	ie[1] = (t1 << 3) | (t3 >> 3);
	ie[2] = (t3 & 0x07) | (t2 & 0x1f);
	rsl_dch_push_hdr(msg, RSL_MT_CHAN_ACTIV_ACK, chan_nr);
	abis_push_ipa(msg, IPA_PROTO_RSL);

	return abis_tx(&trx->link, msg);
}

/* 8.4.3 sending CHANnel ACTIVation Negative ACK */
static int rsl_tx_chan_nack(struct osmobts_trx *trx, struct msgb *msg, uint8_t cause)
{
	struct abis_rsl_dchan_hdr *dch = msgb_l2(msg);
	uint8_t *ie;
	uint8_t chan_nr = dch->chan_nr;

	LOGP(DRSL, LOGL_NOTICE, "Sending Channel Activated NACK: cause = 0x%02x\n", cause);

	msg->len = 0;
	msg->data = msg->tail = msg->l3h;

	ie = msgb_put(msg, 3);
	/* 9.3.26 Cause */
	ie[0] = RSL_IE_CAUSE;
	ie[1] = 1;
	ie[2] = cause;
	rsl_dch_push_hdr(msg, RSL_MT_CHAN_ACTIV_NACK, chan_nr);
	abis_push_ipa(msg, IPA_PROTO_RSL);

	return abis_tx(&trx->link, msg);
}

/* 8.5.3 sending CHANnel ReQuireD */
int rsl_tx_chan_rqd(struct osmobts_trx *trx)
{
	struct msgb *nmsg;
	uint8_t *ie;

	LOGP(DRSL, LOGL_NOTICE, "Sending Channel Required\n");

	nmsg = rsl_msgb_alloc(sizeof(struct abis_rsl_cchan_hdr));
	if (!nmsg)
		return -ENOMEM;
	ie = msgb_put(nmsg, 4);
	/* 9.3.19 Request Reference */
	ie[0] = RSL_IE_REQ_REFERENCE;
	ie[1] = 0xe0; // FIXME
	ie[2] = 0x00;
	ie[3] = 0x00;
	/* 9.3.17 Access Delay */
	ie = msgb_put(nmsg, 2);
	ie[0] = RSL_IE_ACCESS_DELAY;
	ie[1] = 0x00; // FIXME
	rsl_cch_push_hdr(nmsg, RSL_MT_CHAN_RQD, 0x88); // FIXME
	abis_push_ipa(nmsg, IPA_PROTO_RSL);

	return abis_tx(&trx->link, nmsg);
}

/* 8.4.1 CHANnel ACTIVation is received */
static int rsl_rx_chan_activ(struct osmobts_trx *trx, struct msgb *msg)
{
	struct abis_rsl_dchan_hdr *dch = msgb_l2(msg);
	struct tlv_parsed tp;
	uint8_t type, mode;
	struct osmobts_lchan *lchan;

	LOGP(DRSL, LOGL_INFO, "Channel Activation:\n");

	lchan = rsl_get_chan(trx, dch->chan_nr);
	if (!lchan)
		return rsl_tx_chan_nack(trx, msg, RSL_ERR_MAND_IE_ERROR);

	rsl_tlv_parse(&tp, msgb_l3(msg), msgb_l3len(msg));

	/* 9.3.3 Activation Type */
	if (!TLVP_PRESENT(&tp, RSL_IE_ACT_TYPE)) {
		LOGP(DRSL, LOGL_NOTICE, "missing Activation Type\n");
		msgb_free(msg);
		return rsl_tx_chan_nack(trx, msg, RSL_ERR_MAND_IE_ERROR);
	}
	type = *TLVP_VAL(&tp, RSL_IE_ACT_TYPE);
	/* 9.3.6 Channel Mode */
	if (!TLVP_PRESENT(&tp, RSL_IE_CHAN_MODE)) {
		LOGP(DRSL, LOGL_NOTICE, "missing Channel Mode\n");
		msgb_free(msg);
		return rsl_tx_chan_nack(trx, msg, RSL_ERR_MAND_IE_ERROR);
	}
	mode = *TLVP_VAL(&tp, RSL_IE_CHAN_MODE);

	LOGP(DRSL, LOGL_INFO, " chan_nr=0x%02x type=0x%02x mode=0x%02x\n", dch->chan_nr, type, mode);

	return rsl_tx_chan_ack(trx, msg, 0, 0, 0);
}

/* 8.4.14 RF CHANnel RELease is received */
static int rsl_rx_rf_chan_rel(struct osmobts_trx *trx, struct msgb *msg)
{
	struct abis_rsl_dchan_hdr *dch = msgb_l2(msg);
	struct osmobts_lchan *lchan;
	int rc;

	LOGP(DRSL, LOGL_INFO, "Channel Release:\n");

	lchan = rsl_get_chan(trx, dch->chan_nr);
	if (!lchan)
		return rsl_tx_chan_nack(trx, msg, RSL_ERR_MAND_IE_ERROR);

	LOGP(DRSL, LOGL_INFO, " chan_nr=0x%02x\n", dch->chan_nr);

	lapdm_reset(&lchan->l2_entity.lapdm_dcch);
	lapdm_reset(&lchan->l2_entity.lapdm_acch);

	rc = rsl_tx_rf_rel_ack(trx, msg, 0, 0, 0);

	if (lchan->rtp.socket_created)
		rsl_tx_ipac_dlcx_ind(lchan, RSL_ERR_NORMAL_UNSPEC);

	return rc;
}

/*
 * ip.access related messages
 */

int rsl_tx_ipac_dlcx_ind(struct osmobts_lchan *lchan, uint8_t cause)
{
	struct msgb *nmsg;
	struct osmobts_trx *trx = lchan->slot->trx;
	uint8_t *ie;

	LOGP(DRSL, LOGL_NOTICE, "Sending RTP delete indication: cause=%d\n", cause);

	nmsg = rsl_msgb_alloc(sizeof(struct abis_rsl_common_hdr));
	if (!nmsg)
		return -ENOMEM;
	ie = msgb_put(nmsg, 3);
	ie[0] = RSL_IE_CAUSE;
	ie[1] = 1;
	ie[2] = cause;
	rsl_trx_push_hdr(nmsg, RSL_MT_IPAC_DLCX_IND);
	abis_push_ipa(nmsg, IPA_PROTO_RSL);

	return abis_tx(&trx->link, nmsg);
}

static int rsl_tx_ipac_cx_ack(struct osmobts_trx *trx, struct msgb *msg, uint32_t ip, uint16_t port, uint16_t *conn_id)
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
	abis_push_ipa(msg, IPA_PROTO_RSL);

	return abis_tx(&trx->link, msg);
}

static int rsl_tx_ipac_cx_nack(struct osmobts_trx *trx, struct msgb *msg, uint8_t cause)
{
	struct abis_rsl_dchan_hdr *dch = msgb_l2(msg);
	uint8_t chan_nr = dch->chan_nr;
	uint8_t msg_type = dch->c.msg_type;
	uint8_t *ie;

	LOGP(DRSL, LOGL_NOTICE, "Sending RTP connection request NACK: cause=%d\n", cause);

	msg->len = 0;
	msg->data = msg->tail = msg->l3h;

	ie = msgb_put(msg, 3);
	/* 9.3.26 Cause */
	ie[0] = RSL_IE_CAUSE;
	ie[1] = 1;
	ie[2] = cause;
	rsl_dch_push_hdr(msg, msg_type + 2, chan_nr);
	abis_push_ipa(msg, IPA_PROTO_RSL);

	return abis_tx(&trx->link, msg);
}

static int rsl_rx_ipac_crcx_mdcx(struct osmobts_trx *trx, struct msgb *msg)
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

	lchan = rsl_get_chan(trx, dch->chan_nr);
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

static int rsl_rx_ipac_dlcx(struct osmobts_trx *trx, struct msgb *msg)
{
	struct abis_rsl_dchan_hdr *dch = msgb_l2(msg);
	struct tlv_parsed tp;
	struct osmobts_lchan *lchan;

	LOGP(DRSL, LOGL_INFO, "Request of deleting RTP connection:\n");

	lchan = rsl_get_chan(trx, dch->chan_nr);
	if (!lchan)
		return rsl_tx_ipac_cx_nack(trx, msg, RSL_ERR_MAND_IE_ERROR);

	rsl_tlv_parse(&tp, msgb_l3(msg), msgb_l3len(msg));

	rtp_close_socket(&lchan->rtp);

	return rsl_tx_ipac_cx_ack(trx, msg, ntohl(lchan->rtp.rtp_udp.sin_local.sin_addr.s_addr), ntohs(lchan->rtp.rtp_udp.sin_local.sin_port), NULL);
}

/*
 * selecting message
 */

static int rsl_rx_rll(struct osmobts_trx *trx, struct msgb *msg)
{
	struct abis_rsl_rll_hdr *rh = msgb_l2(msg);
	struct osmobts_lchan *lchan;

	if (msgb_l2len(msg) < sizeof(*rh)) {
		LOGP(DRSL, LOGL_NOTICE, "RSL Radio Link Layer message too short\n");
		msgb_free(msg);
		return -EIO;
	}
	msg->l3h = (unsigned char *)rh + sizeof(*rh);

	lchan = rsl_get_chan(trx, rh->chan_nr);
	if (!lchan)
		return rsl_tx_chan_nack(trx, msg, RSL_ERR_MAND_IE_ERROR);

	return rslms_recvmsg(msg, &lchan->l2_entity);
}

int rsl_tx_rll(struct msgb *msg, struct osmol2_entity *l2_entity)
{
	struct osmobts_lchan *lchan = container_of(l2_entity, struct osmobts_lchan, l2_entity);
	struct abis_rsl_common_hdr *rh = msgb_l2(msg);

	LOGP(DRSL, LOGL_INFO, "Got '%s' message from L2.\n", get_rsl_name(rh->msg_type));

	abis_push_ipa(msg, IPA_PROTO_RSL);

	return abis_tx(&lchan->slot->trx->link, msg);
}

static int rsl_rx_cchan(struct osmobts_trx *trx, struct msgb *msg)
{
	struct abis_rsl_cchan_hdr *cch = msgb_l2(msg);
	int ret = 0;

	if (msgb_l2len(msg) < sizeof(*cch)) {
		LOGP(DRSL, LOGL_NOTICE, "RSL Common Channel Management message too short\n");
		msgb_free(msg);
		return -EIO;
	}
	msg->l3h = (unsigned char *)cch + sizeof(*cch);

	switch (cch->c.msg_type) {
	case RSL_MT_BCCH_INFO:
		ret = rsl_rx_bcch_info(trx, msg);
		break;
	case RSL_MT_IMMEDIATE_ASSIGN_CMD:
		ret = rsl_rx_imm_ass(trx, msg);
		break;
	default:
		LOGP(DRSL, LOGL_NOTICE, "unsupported RSL cchan msg_type 0x%02x\n",
			cch->c.msg_type);
		ret = -EINVAL;
	}

	msgb_free(msg);
	return ret;
}

static int rsl_rx_dchan(struct osmobts_trx *trx, struct msgb *msg)
{
	struct abis_rsl_dchan_hdr *dch = msgb_l2(msg);
	int ret = 0;

	if (msgb_l2len(msg) < sizeof(*dch)) {
		LOGP(DRSL, LOGL_NOTICE, "RSL Dedicated Channel Management message too short\n");
		msgb_free(msg);
		return -EIO;
	}
	msg->l3h = (unsigned char *)dch + sizeof(*dch);

	switch (dch->c.msg_type) {
	case RSL_MT_CHAN_ACTIV:
		return rsl_rx_chan_activ(trx, msg);
	case RSL_MT_RF_CHAN_REL:
		return rsl_rx_rf_chan_rel(trx, msg);
	default:
		LOGP(DRSL, LOGL_NOTICE, "unsupported RSL dchan msg_type 0x%02x\n",
			dch->c.msg_type);
		ret = -EINVAL;
	}

	msgb_free(msg);
	return ret;
}

static int rsl_rx_trx(struct osmobts_trx *trx, struct msgb *msg)
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
		ret = rsl_rx_bcch_info(trx, msg);
		break;
	default:
		LOGP(DRSL, LOGL_NOTICE, "unsupported RSL TRX msg_type 0x%02x\n",
			th->msg_type);
		ret = -EINVAL;
	}

	msgb_free(msg);
	return ret;
}

static int rsl_rx_ipaccess(struct osmobts_trx *trx, struct msgb *msg)
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
	case RSL_MT_IPAC_CRCX:
	case RSL_MT_IPAC_MDCX:
		return rsl_rx_ipac_crcx_mdcx(trx, msg);
	case RSL_MT_IPAC_DLCX:
		return rsl_rx_ipac_dlcx(trx, msg);
	default:
		LOGP(DRSL, LOGL_NOTICE, "unsupported RSL ip.access msg_type 0x%02x\n",
			dch->c.msg_type);
		ret = -EINVAL;
	}

	msgb_free(msg);
	return ret;
}

int down_rsl(struct osmobts_trx *trx, struct msgb *msg)
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

	return ret;
}


