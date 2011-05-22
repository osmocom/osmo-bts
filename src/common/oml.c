/*
 * (C) 2011 by Andreas Eversberg <jolly@eversberg.eu>
 * (C) 2011 by Harald Welte <laforge@gnumonks.org>
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
 * Operation and Maintainance Messages
 */

#include <errno.h>
#include <sys/types.h>
#include <arpa/inet.h>

#include <osmocom/gsm/protocol/gsm_12_21.h>
#include <osmocom/gsm/abis_nm.h>
#include <osmo-bts/logging.h>
//#include <osmocom/bb/common/osmocom_data.h>
#include <osmo-bts/support.h>
#include <osmo-bts/abis.h>
#include <osmo-bts/rtp.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/rsl.h>
#include <osmo-bts/oml.h>

/*
 * support
 */

struct osmobts_trx *get_trx_by_nr(struct osmocom_bts *bts, uint8_t trx_nr)
{
	int max = sizeof(bts->trx) / sizeof(bts->trx[0]);
	struct osmobts_trx *trx;

	if (trx_nr >= max) {
		LOGP(DOML, LOGL_NOTICE, "Indicated TRX #%d is out of range. (max #%d)\n", trx_nr, max - 1);
		return NULL;
	}

	trx = bts->trx[trx_nr];
	if (!trx) {
		LOGP(DOML, LOGL_NOTICE, "Indicated TRX #%d does not exist.\n", trx_nr);
		return NULL;
	}

	return trx;
}

struct osmobts_slot *get_slot_by_nr(struct osmobts_trx *trx, uint8_t slot_nr)
{
	struct osmobts_slot *slot;

	if (slot_nr >= 8) {
		LOGP(DOML, LOGL_NOTICE, "Indicated Slot #%d is out of range. (max #8)\n", slot_nr);
		return NULL;
	}

	slot = &trx->slot[slot_nr];

	return slot;
}

static struct msgb *fom_msgb_alloc(void)
{
	struct msgb *nmsg;

	nmsg = abis_msgb_alloc(sizeof(struct abis_om_hdr));
	if (!nmsg)
		return NULL;
	return nmsg;
}

static void fom_push_om(struct msgb *msg, uint8_t mdisc, uint8_t placement, uint8_t sequence)
{
	struct abis_om_hdr *noh;

	msg->l3h = msg->data;
	noh = (struct abis_om_hdr *) msgb_push(msg, sizeof(*noh));
	noh->mdisc = mdisc;
	noh->placement = placement;
	noh->sequence = sequence;
	noh->length = msgb_l3len(msg);
}

static int fom_ack_nack(struct ipabis_link *link, struct msgb *msg, uint8_t cause)
{
	struct abis_om_fom_hdr *foh = msgb_l3(msg);
	uint8_t *ie;

	/* alter message type */
	if (cause) {
		LOGP(DOML, LOGL_NOTICE, "Sending FOM NACK with cause %d.\n", cause);
		foh->msg_type += 2; /* nack */
		/* add cause */
		ie = msgb_put(msg, 2);
		ie[0] = NM_ATT_NACK_CAUSES;
		ie[1] = cause;
	} else {
		LOGP(DOML, LOGL_NOTICE, "Sending FOM ACK.\n");
		foh->msg_type++; /* ack */
	}

	return abis_tx(link, msg);
}

/*
 * Formatted O&M messages
 */

/* 8.3.7 sending SW Activated Report */
int oml_tx_sw_act_rep(struct ipabis_link *link, uint8_t obj_class, uint8_t bts_nr, uint8_t trx_nr, uint8_t ts_nr)
{
	struct msgb *nmsg;
	struct abis_om_fom_hdr *nofh;

	LOGP(DOML, LOGL_INFO, "Sending SW Activated Report (%02x,%02x,%02x).\n", bts_nr, trx_nr, ts_nr);

	nmsg = fom_msgb_alloc();
	if (!nmsg)
		return -ENOMEM;
	nofh = (struct abis_om_fom_hdr *) msgb_put(nmsg, sizeof(*nofh));
	nofh->msg_type = NM_MT_SW_ACTIVATED_REP;
	nofh->obj_class = obj_class;
	nofh->obj_inst.bts_nr = bts_nr;
	nofh->obj_inst.trx_nr = trx_nr;
	nofh->obj_inst.ts_nr = ts_nr;
	fom_push_om(nmsg, ABIS_OM_MDISC_FOM, ABIS_OM_PLACEMENT_ONLY, 0);
	abis_push_ipa(nmsg, IPA_PROTO_OML);

	return abis_tx(link, nmsg);
}

/* 8.8.1 sending State Changed Event Report */
int oml_tx_state_changed(struct ipabis_link *link, uint8_t op_state, uint8_t avail_status, uint8_t obj_class, uint8_t bts_nr, uint8_t trx_nr, uint8_t ts_nr)
{
	struct msgb *nmsg;
	struct abis_om_fom_hdr *nofh;
	uint8_t *ie;

	LOGP(DOML, LOGL_INFO, "Sending state change (bts=%02x trx=%02x ts=%02x).\n", bts_nr, trx_nr, ts_nr);

	nmsg = fom_msgb_alloc();
	if (!nmsg)
		return -ENOMEM;
	nofh = (struct abis_om_fom_hdr *) msgb_put(nmsg, sizeof(*nofh));
	nofh->msg_type = NM_MT_STATECHG_EVENT_REP;
	nofh->obj_class = obj_class;
	nofh->obj_inst.bts_nr = bts_nr;
	nofh->obj_inst.trx_nr = trx_nr;
	nofh->obj_inst.ts_nr = ts_nr;
	/* 9.4.38 Operational State */
	ie = msgb_put(nmsg, 2);
	ie[0] = NM_ATT_OPER_STATE;
	ie[1] = op_state;
	/* 9.4.7 Availability Status */
	ie = msgb_put(nmsg, 4);
	ie[0] = NM_ATT_AVAIL_STATUS;
	ie[1] = 0;
	ie[2] = 1;
	ie[3] = avail_status;
	fom_push_om(nmsg, ABIS_OM_MDISC_FOM, ABIS_OM_PLACEMENT_ONLY, 0);
	abis_push_ipa(nmsg, IPA_PROTO_OML);

	return abis_tx(link, nmsg);
}

/* 8.6.1 Set BTS Attributes is received */
int oml_rx_set_bts_attr(struct osmocom_bts *bts, struct msgb *msg)
{
	struct abis_om_fom_hdr *foh = msgb_l3(msg);
	struct tlv_parsed tp;
	struct bts_support *sup = &bts_support;

	LOGP(DOML, LOGL_INFO, "BSC is setting BTS attributes:\n");

	tlv_parse(&tp, &abis_nm_att_tlvdef, foh->data, msgb_l3len(msg) - sizeof(*foh), 0, 0);
	/* 9.4.31 Maximum Timing Advance */
	if (TLVP_PRESENT(&tp, NM_ATT_MAX_TA)) {
		uint16_t *fn = (uint16_t *) TLVP_VAL(&tp, NM_ATT_START_TIME);
		bts->max_ta = ntohs(*fn);
		LOGP(DOML, LOGL_INFO, " Maximum TA = %d\n", bts->max_ta);
	}
	/* 9.4.8 BCCH ARFCN */
	if (TLVP_PRESENT(&tp, NM_ATT_BCCH_ARFCN)) {
		uint16_t *value = (uint16_t *) TLVP_VAL(&tp, NM_ATT_BCCH_ARFCN);
		uint16_t arfcn = ntohs(*value);
		LOGP(DOML, LOGL_INFO, " ARFCN = %d", bts->bcch_arfcn);
		if (arfcn > 1023 || !(sup->freq_map[arfcn >> 3] & (1 << (arfcn & 0x7)))) {
			LOGP(DOML, LOGL_NOTICE, "Given ARFCN %d is not supported.\n", arfcn);
			return fom_ack_nack(&bts->link, msg, NM_NACK_FREQ_NOTAVAIL);
		}
		bts->bcch_arfcn = arfcn;
	}
	/* 9.4.9 BSIC */
	if (TLVP_PRESENT(&tp, NM_ATT_BSIC)) {
		uint8_t *bsic = (uint8_t *) TLVP_VAL(&tp, NM_ATT_BSIC);
		bts->bcc = *bsic & 0x7;
		bts->ncc = (*bsic >> 3) & 0x7;
		LOGP(DOML, LOGL_INFO, " BCC = %d\n", bts->bcc);
		LOGP(DOML, LOGL_INFO, " NCC = %d\n", bts->ncc);
	}
	/* 9.4.52 Starting Time */
	if (TLVP_PRESENT(&tp, NM_ATT_START_TIME)) {
		uint16_t *fn = (uint16_t *) TLVP_VAL(&tp, NM_ATT_START_TIME);
		bts->start_time = ntohs(*fn);
		LOGP(DOML, LOGL_INFO, " Starting Time = %d\n", bts->start_time);
	}

	return fom_ack_nack(&bts->link, msg, 0);
}

/* 8.6.2 Set Radio Attributes is received */
int oml_rx_set_radio_attr(struct osmocom_bts *bts, struct msgb *msg)
{
	struct abis_om_fom_hdr *foh = msgb_l3(msg);
	struct tlv_parsed tp;
	struct osmobts_trx *trx;
	struct bts_support *sup = &bts_support;

	trx = get_trx_by_nr(bts, foh->obj_inst.trx_nr);
	if (!trx)
		return fom_ack_nack(&bts->link, msg, NM_NACK_TRXNR_UNKN);

	LOGP(DOML, LOGL_INFO, "BSC is setting radio attributes:\n");

	tlv_parse(&tp, &abis_nm_att_tlvdef, foh->data, msgb_l3len(msg) - sizeof(*foh), 0, 0);
	/* 9.4.47 RF Max Power Reduction */
	if (TLVP_PRESENT(&tp, NM_ATT_RF_MAXPOWR_R)) {
		trx->rf_red = *TLVP_VAL(&tp, NM_ATT_RF_MAXPOWR_R);
		LOGP(DOML, LOGL_INFO, " RF Max Power Reduction = %d\n", trx->rf_red);
	} else
		trx->rf_red = 0;
	/* 9.4.5 ARFCN List */
	if (TLVP_PRESENT(&tp, NM_ATT_ARFCN_LIST)) {
		uint16_t *value = (uint16_t *) TLVP_VAL(&tp, NM_ATT_ARFCN_LIST);
		uint16_t length = *(TLVP_VAL(&tp, NM_ATT_ARFCN_LIST) - 1);
		uint16_t arfcn;
		int max = (sizeof(trx->arfcn_list) / sizeof(trx->arfcn_list[0]));
		int i;
		if (length > max) {
			LOGP(DOML, LOGL_NOTICE, "Too many ARFCN given. (max #%d)\n", max);
			return fom_ack_nack(&bts->link, msg, NM_NACK_PARAM_RANGE);
		}
		for (i = 0; i < length; i++) {
			arfcn = ntohs(*value++);
			if (arfcn > 1023 || !(sup->freq_map[arfcn >> 3] & (1 << (arfcn & 0x7))))
				return fom_ack_nack(&bts->link, msg, NM_NACK_FREQ_NOTAVAIL);
			trx->arfcn_list[i] = arfcn;
			LOGP(DOML, LOGL_INFO, " ARFCN list = %d\n", trx->arfcn_list[i]);
		}
		trx->arfcn_num = length;
	} else
		trx->arfcn_num = 0;

	return fom_ack_nack(&bts->link, msg, 0);
}

/* 8.6.3 Set Channel Attributes is received */
int oml_rx_set_chan_attr(struct osmocom_bts *bts, struct msgb *msg)
{
	struct abis_om_fom_hdr *foh = msgb_l3(msg);
	struct tlv_parsed tp;
	struct osmobts_trx *trx;
	struct osmobts_slot *slot;
	struct bts_support *sup = &bts_support;

	trx = get_trx_by_nr(bts, foh->obj_inst.trx_nr);
	if (!trx)
		return fom_ack_nack(&bts->link, msg, NM_NACK_TRXNR_UNKN);
	slot = get_slot_by_nr(trx, foh->obj_inst.ts_nr);
	if (!slot)
		return fom_ack_nack(&bts->link, msg, NM_NACK_OBJINST_UNKN);

	LOGP(DOML, LOGL_INFO, "BSC is setting channel attributes:\n");

	tlv_parse(&tp, &abis_nm_att_tlvdef, foh->data, msgb_l3len(msg) - sizeof(*foh), 0, 0);
	/* 9.4.13 Channel Combination */
	if (TLVP_PRESENT(&tp, NM_ATT_CHAN_COMB)) {
		uint8_t comb = *TLVP_VAL(&tp, NM_ATT_CHAN_COMB);
		if (!sup->chan_comb[comb]) {
			LOGP(DOML, LOGL_NOTICE, " channel combination %d (not supported).\n", comb);
			return fom_ack_nack(&bts->link, msg, NM_NACK_SPEC_IMPL_NOTSUPP);
		}
		LOGP(DOML, LOGL_INFO, " channel combination = %s\n", bts_support_comb_name(comb));
		bts_setup_slot(slot, comb);
		slot->chan_comb = comb;
	}
	/* 9.4.21 HSN... */
	if (TLVP_PRESENT(&tp, NM_ATT_HSN)) {
		LOGP(DOML, LOGL_NOTICE, "Frequency hopping not supported.\n");
		return fom_ack_nack(&bts->link, msg, NM_NACK_SPEC_IMPL_NOTSUPP);
	}

	return fom_ack_nack(&bts->link, msg, 0);
}

/* 8.9.2 Opstart is received */
int oml_rx_opstart(struct osmocom_bts *bts, struct msgb *msg)
{
	struct abis_om_fom_hdr *foh = msgb_l3(msg);
	struct osmobts_trx *trx;
	struct osmobts_slot *slot;


	/* site manager */
	if (foh->obj_inst.bts_nr == 0xff) {
		LOGP(DOML, LOGL_INFO, "BSC is sending Opstart. (Site Manager)\n");
		oml_tx_state_changed(&bts->link, NM_OPSTATE_ENABLED, NM_AVSTATE_OK, NM_OC_SITE_MANAGER, foh->obj_inst.bts_nr, foh->obj_inst.trx_nr, foh->obj_inst.ts_nr);
		return fom_ack_nack(&bts->link, msg, 0);
	}

#warning todo: change state
	/* BTS */
	if (foh->obj_inst.trx_nr == 0xff) {
		LOGP(DOML, LOGL_INFO, "BSC is sending Opstart. (BTS)\n");
		return fom_ack_nack(&bts->link, msg, 0);
	}

	/* TRX */
	trx = get_trx_by_nr(bts, foh->obj_inst.trx_nr);
	if (!trx)
		return fom_ack_nack(&bts->link, msg, NM_NACK_TRXNR_UNKN);
	if (foh->obj_inst.ts_nr == 0xff) {
		LOGP(DOML, LOGL_INFO, "BSC is sending Opstart. (TRX %d)\n", trx->trx_nr);
		if (trx->link.state == LINK_STATE_IDLE) {
			int ret;

			/* connecting TRX */
			ret = abis_open(&trx->link, bts->link.ip);
			if (ret <= 0) {
				LOGP(DOML, LOGL_ERROR, "Failed to connect TRX.\n");
				return fom_ack_nack(&bts->link, msg, NM_NACK_CANT_PERFORM);
			}
		}
		return fom_ack_nack(&bts->link, msg, 0);
	}

	/* slot */
	slot = get_slot_by_nr(trx, foh->obj_inst.ts_nr);
	if (!slot)
		return fom_ack_nack(&bts->link, msg, NM_NACK_OBJINST_UNKN);

	LOGP(DOML, LOGL_INFO, "BSC is sending Opstart. (trx=%d ts=%d)\n", trx->trx_nr, slot->slot_nr);
	oml_tx_state_changed(&bts->link, NM_OPSTATE_ENABLED, NM_AVSTATE_OK, NM_OC_CHANNEL, foh->obj_inst.bts_nr, foh->obj_inst.trx_nr, foh->obj_inst.ts_nr);
	return fom_ack_nack(&bts->link, msg, 0);
}

static int down_fom(struct osmocom_bts *bts, struct msgb *msg)
{
	struct abis_om_fom_hdr *foh = msgb_l3(msg);
	int ret;

	if (msgb_l2len(msg) < sizeof(*foh)) {
		LOGP(DOML, LOGL_NOTICE, "Formatted O&M message too short\n");
		msgb_free(msg);
		return -EIO;
	}

	if (foh->obj_inst.bts_nr != 0 && foh->obj_inst.bts_nr != 0xff) {
		LOGP(DOML, LOGL_INFO, "Formatted O&M with BTS %d out of range.\n", foh->obj_inst.bts_nr);
		return fom_ack_nack(&bts->link, msg, NM_NACK_BTSNR_UNKN);
	}

	switch (foh->msg_type) {
	case NM_MT_SET_BTS_ATTR:
		ret = oml_rx_set_bts_attr(bts, msg);
		break;
	case NM_MT_SET_RADIO_ATTR:
		ret = oml_rx_set_radio_attr(bts, msg);
		break;
	case NM_MT_SET_CHAN_ATTR:
		ret = oml_rx_set_chan_attr(bts, msg);
		break;
	case NM_MT_OPSTART:
		ret = oml_rx_opstart(bts, msg);
		break;
	case NM_MT_CHG_ADM_STATE:
		LOGP(DOML, LOGL_INFO, "BSC is changing ADM state.\n");
		ret = fom_ack_nack(&bts->link, msg, 0);
		break;
	default:
		LOGP(DOML, LOGL_INFO, "unknown Formatted O&M msg_type 0x%02x\n",
			foh->msg_type);
		ret = fom_ack_nack(&bts->link, msg, NM_NACK_MSGTYPE_INVAL);
	}

	return ret;
}

/*
 * manufacturer related messages
 */

static int down_mom(struct osmocom_bts *bts, struct msgb *msg)
{
	struct abis_om_fom_hdr *foh = msgb_l3(msg);
	int ret;

	if (msgb_l2len(msg) < sizeof(*foh)) {
		LOGP(DOML, LOGL_NOTICE, "Manufacturer O&M message too short\n");
		msgb_free(msg);
		return -EIO;
	}

	if (foh->obj_inst.bts_nr != 0 && foh->obj_inst.bts_nr != 0xff) {
		LOGP(DOML, LOGL_INFO, "Manufacturer O&M with BTS %d out of range.\n", foh->obj_inst.bts_nr);
		return fom_ack_nack(&bts->link, msg, NM_NACK_BTSNR_UNKN);
	}

	switch (foh->msg_type) {
	default:
		LOGP(DOML, LOGL_INFO, "Manufacturer Formatted O&M msg_type 0x%02x\n",
			foh->msg_type);
		ret = fom_ack_nack(&bts->link, msg, NM_NACK_MSGTYPE_INVAL);
	}

	return ret;
}

/*
 * selecting messages
 */

int down_oml(struct osmocom_bts *bts, struct msgb *msg)
{
	struct abis_om_hdr *oh = msgb_l2(msg);
	int ret = 0;

	if (msgb_l2len(msg) < 1) {
		LOGP(DOML, LOGL_NOTICE, "OML message too short\n");
		msgb_free(msg);
		return -EIO;
	}
	msg->l3h = (unsigned char *)oh + sizeof(*oh);

	switch (oh->mdisc) {
	case ABIS_OM_MDISC_FOM:
		if (msgb_l2len(msg) < sizeof(*oh)) {
			LOGP(DOML, LOGL_NOTICE, "Formatted O&M message too short\n");
			msgb_free(msg);
			ret = -EIO;
			break;
		}
		ret = down_fom(bts, msg);
		break;
	case ABIS_OM_MDISC_MANUF:
		if (msgb_l2len(msg) < sizeof(*oh)) {
			LOGP(DOML, LOGL_NOTICE, "Manufacturer O&M message too short\n");
			msgb_free(msg);
			ret = -EIO;
			break;
		}
		ret = down_mom(bts, msg);
		break;
	default:
		LOGP(DOML, LOGL_NOTICE, "unknown OML msg_discr 0x%02x\n",
			oh->mdisc);
		msgb_free(msg);
		ret = -EINVAL;
	}

	return ret;
}


