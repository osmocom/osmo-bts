/* GSM TS 12.21 O&M / OML, BTS side */

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
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

/*
 * Operation and Maintenance Messages
 */

#include "btsconfig.h"

#include <errno.h>
#include <stdarg.h>
#include <string.h>
#include <sys/types.h>
#include <arpa/inet.h>

#include <osmocom/core/talloc.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/utils.h>
#include <osmocom/gsm/protocol/gsm_12_21.h>
#include <osmocom/gsm/abis_nm.h>
#include <osmocom/gsm/tlv.h>
#include <osmocom/abis/e1_input.h>
#include <osmocom/abis/ipaccess.h>

#include <osmo-bts/logging.h>
#include <osmo-bts/gsm_data.h>
#include <osmo-bts/abis.h>
#include <osmo-bts/oml.h>
#include <osmo-bts/bts_model.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/bts_sm.h>
#include <osmo-bts/signal.h>
#include <osmo-bts/phy_link.h>
#include <osmo-bts/nm_common_fsm.h>

#define LOGPFOH(ss, lvl, foh, fmt, args ...) LOGP(ss, lvl, "%s: " fmt, abis_nm_dump_foh(foh), ## args)
#define DEBUGPFOH(ss, foh, fmt, args ...) LOGPFOH(ss, LOGL_DEBUG, foh, fmt, ## args)

static int oml_ipa_set_attr(struct gsm_bts *bts, struct msgb *msg);

static struct tlv_definition abis_nm_att_tlvdef_ipa_local = {};

/*
 * support
 */

static int oml_tlv_parse(struct tlv_parsed *tp, const uint8_t *buf, int len)
{
	return tlv_parse(tp, &abis_nm_att_tlvdef_ipa_local, buf, len, 0, 0);
}

struct msgb *oml_msgb_alloc(void)
{
	return msgb_alloc_headroom(1024, 128, "OML");
}

/* FIXME: move to gsm_data_shared */
static char mo_buf[128];
char *gsm_abis_mo_name(const struct gsm_abis_mo *mo)
{
	snprintf(mo_buf, sizeof(mo_buf), "OC=%s INST=(%02x,%02x,%02x)",
		 get_value_string(abis_nm_obj_class_names, mo->obj_class),
		 mo->obj_inst.bts_nr, mo->obj_inst.trx_nr, mo->obj_inst.ts_nr);
	return mo_buf;
}

/* 3GPP TS 12.21 § 8.8.2 */
int oml_tx_failure_event_rep(const struct gsm_abis_mo *mo, enum abis_nm_severity severity,
			     uint16_t cause_value, const char *fmt, ...)
{
	struct msgb *nmsg;
	va_list ap;

	LOGP(DOML, LOGL_NOTICE, "%s: Sending %s to BSC: ", gsm_abis_mo_name(mo),
		get_value_string(abis_mm_event_cause_names, cause_value));
	va_start(ap, fmt);
	osmo_vlogp(DOML, LOGL_NOTICE, __FILE__, __LINE__, 1, fmt, ap);
	nmsg = abis_nm_fail_evt_vrep(NM_EVT_PROC_FAIL, severity,
				     NM_PCAUSE_T_MANUF, cause_value, fmt, ap);
	va_end(ap);
	LOGPC(DOML, LOGL_NOTICE, "\n");

	if (!nmsg)
		return -ENOMEM;

	return oml_mo_send_msg(mo, nmsg, NM_MT_FAILURE_EVENT_REP);
}

/* Push OM header in front of msgb and send it */
int oml_send_msg(struct msgb *msg, int is_manuf)
{
	struct abis_om_hdr *omh;

	if (is_manuf) {
		/* length byte, string + 0 termination */
		uint8_t *manuf = msgb_push(msg, LV_GROSS_LEN(sizeof(abis_nm_ipa_magic)));
		lv_put(manuf, sizeof(abis_nm_ipa_magic), (const uint8_t *) abis_nm_ipa_magic);
	}

	/* Push the main OML header and send it off */
	omh = (struct abis_om_hdr *) msgb_push(msg, sizeof(*omh));
	if (is_manuf)
		omh->mdisc = ABIS_OM_MDISC_MANUF;
	else
		omh->mdisc = ABIS_OM_MDISC_FOM;
	omh->placement = ABIS_OM_PLACEMENT_ONLY;
	omh->sequence = 0;
	omh->length = msgb_l3len(msg);

	msg->l2h = (uint8_t *)omh;

	return abis_oml_sendmsg(msg);
}

int oml_mo_send_msg(const struct gsm_abis_mo *mo, struct msgb *msg, uint8_t msg_type)
{
	struct abis_om_fom_hdr *foh;
	struct gsm_bts *bts;

	msg->l3h = msgb_push(msg, sizeof(*foh));
	foh = (struct abis_om_fom_hdr *) msg->l3h;
	foh->msg_type = msg_type;
	foh->obj_class = mo->obj_class;
	memcpy(&foh->obj_inst, &mo->obj_inst, sizeof(foh->obj_inst));

	/* Find and set OML TRX on msg: */
	switch (mo->obj_class) {
	case NM_OC_SITE_MANAGER:
		/* Pick the first BTS: */
		bts = gsm_bts_num(g_bts_sm, 0);
		break;
	default:
		/* Other objects should have a valid BTS available: */
		bts = gsm_bts_num(g_bts_sm, mo->obj_inst.bts_nr);
	}
	if (OSMO_UNLIKELY(!bts)) {
		LOGPFOH(DOML, LOGL_NOTICE, foh,
			"Sending FOM failed (no related BTS object found)\n");
		return -EINVAL;
	}
	msg->trx = bts->c0;

	DEBUGPFOH(DOML, foh, "Tx %s\n", get_value_string(abis_nm_msgtype_names, foh->msg_type));

	return oml_send_msg(msg, 0);
}

static inline void add_bts_attrs(struct msgb *msg, const struct gsm_bts *bts)
{
	uint16_t total_len = 0;
	uint8_t *len;

	/* Put NM_ATT_SW_CONFIG as per 9.4.61 "SW Configuration". */
	msgb_v_put(msg, NM_ATT_SW_CONFIG);

	/* We don't know the length yet, so we update it later. */
	len = msgb_put(msg, 2);

	total_len += abis_nm_put_sw_file(msg, "osmobts", PACKAGE_VERSION, true);
	total_len += abis_nm_put_sw_file(msg, btsatttr2str(BTS_TYPE_VARIANT),
					 btsvariant2str(bts->variant), true);

	if (strlen(bts->sub_model)) {
		total_len += abis_nm_put_sw_file(msg, btsatttr2str(BTS_SUB_MODEL),
						 bts->sub_model, true);
	}

	/* Finally, update the length */
	osmo_store16be(total_len, len);
}

/* Add BTS features as 3GPP TS 52.021 §9.4.30 Manufacturer Id */
static inline void add_bts_feat(struct msgb *msg, const struct gsm_bts *bts)
{
	unsigned int len = OSMO_BYTES_FOR_BITS(_NUM_BTS_FEAT);
	msgb_tl16v_put(msg, NM_ATT_MANUF_ID, len, bts->features->data);
}

static inline void add_trx_attr(struct msgb *msg, const struct gsm_bts_trx *trx)
{
	const struct phy_instance *pinst = trx_phy_instance(trx);
	const char *phy_version;
	uint16_t total_len;
	uint8_t *len;

	/* Put NM_ATT_SW_CONFIG as per 9.4.61 "SW Configuration". */
	msgb_v_put(msg, NM_ATT_SW_CONFIG);

	/* We don't know the length yet, so we update it later. */
	len = msgb_put(msg, 2);

	phy_version = pinst && strlen(pinst->version) ? pinst->version : "Unknown";
	total_len = abis_nm_put_sw_file(msg, btsatttr2str(TRX_PHY_VERSION), phy_version, true);

	/* Finally, update the length */
	osmo_store16be(total_len, len);
}

/* Handle a list of attributes requested by the BSC, compose
 * TRX-specific Get Attribute Response IE as per 9.4.64. */
static inline int handle_attrs_trx(struct msgb *out_msg, const struct gsm_bts_trx *trx,
				   const uint8_t *attr, uint16_t attr_len)
{
	uint8_t num_unsupported = 0;
	uint8_t *buf;
	int i;

	if (!trx) {
		LOGP(DOML, LOGL_ERROR, "%s: O&M Get Attributes for unknown TRX\n", gsm_trx_name(trx));
		return -NM_NACK_TRXNR_UNKN;
	}

	for (i = 0; i < attr_len; i++) {
		switch (attr[i]) {
		case NM_ATT_SW_CONFIG:
			add_trx_attr(out_msg, trx);
			break;
		default:
			LOGP(DOML, LOGL_ERROR, "%s: O&M Get Attributes [%u], %s is unsupported by TRX\n",
			     gsm_trx_name(trx), i, get_value_string(abis_nm_att_names, attr[i]));
			/* Push this tag to the list of unsupported attributes */
			buf = msgb_push(out_msg, 1);
			*buf = attr[i];
			num_unsupported++;
		}
	}

	/* Push the amount of unsupported attributes */
	buf = msgb_push(out_msg, 1);
	*buf = num_unsupported;

	return 0;
}

/* Handle a list of attributes requested by the BSC, compose
 * BTS-specific Get Attribute Response IE as per 9.4.64. */
static inline int handle_attrs_bts(struct msgb *out_msg, const struct gsm_bts *bts,
				   const uint8_t *attr, uint16_t attr_len)
{
	uint8_t num_unsupported = 0;
	uint8_t *buf;
	int i;

	if (!bts) {
		LOGP(DOML, LOGL_ERROR, "O&M Get Attributes for unknown BTS\n");
		return -NM_NACK_BTSNR_UNKN;
	}

	for (i = 0; i < attr_len; i++) {
		switch (attr[i]) {
		case NM_ATT_SW_CONFIG:
			add_bts_attrs(out_msg, bts);
			break;
		case NM_ATT_MANUF_ID:
			add_bts_feat(out_msg, bts);
			break;
		default:
			LOGP(DOML, LOGL_ERROR, "O&M Get Attributes [%u], %s is unsupported by BTS\n", i,
			     get_value_string(abis_nm_att_names, attr[i]));
			/* Push this tag to the list of unsupported attributes */
			buf = msgb_push(out_msg, 1);
			*buf = attr[i];
			num_unsupported++;
		}
	}

	/* Push the amount of unsupported attributes */
	buf = msgb_push(out_msg, 1);
	*buf = num_unsupported;

	return 0;
}

/* send 3GPP TS 52.021 §8.11.2 Get Attribute Response */
static int oml_tx_attr_resp(const struct gsm_abis_mo *mo,
			    const uint8_t *attr, uint16_t attr_len)
{
	struct msgb *nmsg = oml_msgb_alloc();
	const char *mo_name = gsm_abis_mo_name(mo);
	int rc;

	if (!nmsg)
		return -NM_NACK_CANT_PERFORM;

	switch (mo->obj_class) {
	case NM_OC_BTS:
		rc = handle_attrs_bts(nmsg, mo->bts, attr, attr_len);
		break;
	case NM_OC_BASEB_TRANSC:
		rc = handle_attrs_trx(nmsg, gsm_bts_trx_num(mo->bts, mo->obj_inst.trx_nr), attr, attr_len);
		break;
	default:
		LOGP(DOML, LOGL_ERROR, "%s: Unsupported MO class in Get Attribute Response\n",
		     mo_name);
		rc = -NM_NACK_OBJCLASS_NOTSUPP;
	}

	if (rc < 0) {
		LOGP(DOML, LOGL_ERROR, "%s: Tx Get Attribute Response FAILED with rc=%d\n",
		     mo_name, rc);
		msgb_free(nmsg);
		return rc;
	}

	/* Push Get Attribute Response Info TL (actually TV where V is L) */
	msgb_tv16_push(nmsg, NM_ATT_GET_ARI, msgb_length(nmsg));

	rc = oml_mo_send_msg(mo, nmsg, NM_MT_GET_ATTR_RESP);
	return (rc < 0) ? -NM_NACK_CANT_PERFORM : rc;
}

/* 8.8.1 sending State Changed Event Report */
int oml_tx_state_changed(const struct gsm_abis_mo *mo)
{
	struct msgb *nmsg;
	uint8_t avail_state;

	nmsg = oml_msgb_alloc();
	if (!nmsg)
		return -ENOMEM;

	/* 9.4.38 Operational State */
	msgb_tv_put(nmsg, NM_ATT_OPER_STATE, mo->nm_state.operational);

	/* 9.4.7 Availability Status */
	avail_state = (uint8_t) mo->nm_state.availability;
	msgb_tl16v_put(nmsg, NM_ATT_AVAIL_STATUS, 1, &avail_state);

	/* 9.4.4 Administrative Status -- not in spec but also sent by nanobts */
	msgb_tv_put(nmsg, NM_ATT_ADM_STATE, mo->nm_state.administrative);

	return oml_mo_send_msg(mo, nmsg, NM_MT_STATECHG_EVENT_REP);
}

/* First initialization of MO, does _not_ generate state changes */
void oml_mo_state_init(struct gsm_abis_mo *mo, int op_state, int avail_state)
{
	mo->nm_state.availability = avail_state;
	mo->nm_state.operational = op_state;
}

int oml_mo_state_chg(struct gsm_abis_mo *mo, int op_state, int avail_state, int adm_state)
{
	int rc = 0;

	if ((op_state != -1 && mo->nm_state.operational != op_state) ||
	    (avail_state != -1 && mo->nm_state.availability != avail_state) ||
	    (adm_state != -1 && mo->nm_state.administrative != adm_state)) {
		if (avail_state != -1) {
			LOGP(DOML, LOGL_INFO, "%s AVAIL STATE %s -> %s\n",
				gsm_abis_mo_name(mo),
				abis_nm_avail_name(mo->nm_state.availability),
				abis_nm_avail_name(avail_state));
			mo->nm_state.availability = avail_state;
		}
		if (op_state != -1) {
			struct nm_statechg_signal_data nsd;
			LOGP(DOML, LOGL_INFO, "%s OPER STATE %s -> %s\n",
				gsm_abis_mo_name(mo),
				abis_nm_opstate_name(mo->nm_state.operational),
				abis_nm_opstate_name(op_state));
			nsd.mo = mo;
			nsd.old_state = mo->nm_state.operational;
			nsd.new_state = op_state;
			mo->nm_state.operational = op_state;
			osmo_signal_dispatch(SS_GLOBAL, S_NEW_OP_STATE, &nsd);
		}
		if (adm_state != -1) {
			LOGP(DOML, LOGL_INFO, "%s ADMIN STATE %s -> %s\n",
				gsm_abis_mo_name(mo),
				abis_nm_admin_name(mo->nm_state.administrative),
				abis_nm_admin_name(adm_state));
			mo->nm_state.administrative = adm_state;
		}


		/* send state change report */
		rc = oml_tx_state_changed(mo);
	}
	return rc;
}

/* Send an ACK or NACK response from 'mo' to BSC, deriving message
 * type from 'orig_msg_type'. ACK is sent if cause == 0; NACK otherwise */
int oml_mo_fom_ack_nack(const struct gsm_abis_mo *mo, uint8_t orig_msg_type,
			uint8_t cause)
{
	struct msgb *msg;
	uint8_t new_msg_type;

	msg = oml_msgb_alloc();
	if (!msg)
		return -ENOMEM;

	if (cause) {
		new_msg_type = orig_msg_type + 2;
		msgb_tv_put(msg, NM_ATT_NACK_CAUSES, cause);
	} else {
		new_msg_type = orig_msg_type + 1;
	}

	return oml_mo_send_msg(mo, msg, new_msg_type);
}

int oml_mo_statechg_ack(const struct gsm_abis_mo *mo)
{
	struct msgb *msg;
	int rc = 0;

	msg = oml_msgb_alloc();
	if (!msg)
		return -ENOMEM;

	msgb_tv_put(msg, NM_ATT_ADM_STATE, mo->nm_state.administrative);

	rc = oml_mo_send_msg(mo, msg, NM_MT_CHG_ADM_STATE_ACK);
	if (rc != 0)
		return rc;

	/* Emulate behaviour of ipaccess nanobts: Send a 'State Changed Event Report' as well. */
	return oml_tx_state_changed(mo);
}

int oml_mo_statechg_nack(const struct gsm_abis_mo *mo, uint8_t nack_cause)
{
	return oml_mo_fom_ack_nack(mo, NM_MT_CHG_ADM_STATE, nack_cause);
}

int oml_mo_opstart_ack(const struct gsm_abis_mo *mo)
{
	return oml_mo_fom_ack_nack(mo, NM_MT_OPSTART, 0);
}

int oml_mo_opstart_nack(const struct gsm_abis_mo *mo, uint8_t nack_cause)
{
	return oml_mo_fom_ack_nack(mo, NM_MT_OPSTART, nack_cause);
}

/* Send an ACK or NACK response to BSC for the given OML message,
 * reusing it.  ACK is sent if cause == 0; NACK otherwise. */
int oml_fom_ack_nack(struct msgb *msg, uint8_t cause)
{
	struct abis_om_fom_hdr *foh;

	/* remove any l2/l1 that may be already present */
	msgb_pull_to_l2(msg);

	foh = (struct abis_om_fom_hdr *) msg->l3h;

	/* alter message type */
	if (cause) {
		LOGPFOH(DOML, LOGL_NOTICE, foh, "Sending FOM NACK with cause %s\n",
			abis_nm_nack_cause_name(cause));
		foh->msg_type += 2; /* nack */
		/* add cause */
		msgb_tv_put(msg, NM_ATT_NACK_CAUSES, cause);
	} else {
		LOGPFOH(DOML, LOGL_DEBUG, foh, "Sending FOM ACK\n");
		foh->msg_type++; /* ack */
	}

	/* ensure that the message length is up to date */
	struct abis_om_hdr *omh = (struct abis_om_hdr *) msgb_l2(msg);
	omh->length = msgb_l3len(msg);

	/* we cannot use oml_send_msg() as we already have the OML header */
	if (abis_oml_sendmsg(msg) != 0)
		LOGPFOH(DOML, LOGL_ERROR, foh, "Failed to send ACK/NACK\n");

	/* msgb was reused, do not free() */
	return 1;
}

/* Copy msg before calling oml_fom_ack_nack(), which takes its ownership */
int oml_fom_ack_nack_copy_msg(const struct msgb *old_msg, uint8_t cause)
{
	struct msgb *msg = msgb_copy(old_msg, "OML-ack_nack");
	msg->trx = old_msg->trx;
	oml_fom_ack_nack(msg, cause);
	return 0;
}

/*
 * Formatted O&M messages
 */

/* 8.3.7 sending SW Activated Report */
int oml_mo_tx_sw_act_rep(const struct gsm_abis_mo *mo)
{
	struct msgb *nmsg;

	nmsg = oml_msgb_alloc();
	if (!nmsg)
		return -ENOMEM;

	return oml_mo_send_msg(mo, nmsg, NM_MT_SW_ACTIVATED_REP);
}

/* The defaults below correspond to various sources/recommendations that could be found online.
 * The BSC should override this via OML anyway. */
const unsigned int oml_default_t200_ms[7] = {
        [T200_SDCCH]		= 1000,
        [T200_FACCH_F]		= 1000,
        [T200_FACCH_H]		= 1000,
        [T200_SACCH_TCH_SAPI0]	= 2000,
        [T200_SACCH_SDCCH]	= 2000,
        [T200_SDCCH_SAPI3]	= 1000,
        [T200_SACCH_TCH_SAPI3]	= 2000,
};

/* 3GPP TS 52.021 §8.11.1 Get Attributes has been received */
static int oml_rx_get_attr(struct gsm_bts *bts, struct msgb *msg)
{
	struct abis_om_fom_hdr *foh = msgb_l3(msg);
	const struct gsm_abis_mo *mo;
	struct tlv_parsed tp;
	int rc;
	enum abis_nm_nack_cause c;

	if (!foh || !bts)
		return -EINVAL;

	DEBUGPFOH(DOML, foh, "Rx GET ATTR\n");

	/* Determine which OML object is addressed */
	if ((mo = gsm_objclass2mo(bts, foh->obj_class, &foh->obj_inst, &c)) == NULL) {
		LOGPFOH(DOML, LOGL_ERROR, foh, "Get Attributes for unknown Object Instance\n");
		return oml_fom_ack_nack(msg, c);
	}

	rc = oml_tlv_parse(&tp, foh->data, msgb_l3len(msg) - sizeof(*foh));
	if (rc < 0) {
		oml_tx_failure_event_rep(mo, NM_SEVER_MINOR, OSMO_EVT_MAJ_UNSUP_ATTR,
					 "Get Attribute parsing failure");
		return oml_mo_fom_ack_nack(mo, NM_MT_GET_ATTR, NM_NACK_INCORR_STRUCT);
	}

	if (!TLVP_PRES_LEN(&tp, NM_ATT_LIST_REQ_ATTR, 1)) {
		oml_tx_failure_event_rep(mo, NM_SEVER_MINOR, OSMO_EVT_MAJ_UNSUP_ATTR,
					 "Get Attribute without Attribute List");
		return oml_mo_fom_ack_nack(mo, NM_MT_GET_ATTR, NM_NACK_INCORR_STRUCT);
	}

	rc = oml_tx_attr_resp(mo, TLVP_VAL(&tp, NM_ATT_LIST_REQ_ATTR), TLVP_LEN(&tp, NM_ATT_LIST_REQ_ATTR));
	if (rc < 0) {
		LOGPFOH(DOML, LOGL_ERROR, foh, "Responding to O&M Get Attributes message with NACK 0%x\n", -rc);
		return oml_mo_fom_ack_nack(mo, NM_MT_GET_ATTR,  -rc);
	}

	return 0;
}

/* 8.6.1 Set BTS Attributes has been received */
static int oml_rx_set_bts_attr(struct gsm_bts *bts, struct msgb *msg)
{
	struct abis_om_fom_hdr *foh = msgb_l3(msg);
	struct tlv_parsed tp, *tp_merged;
	int rc, i;
	const uint8_t *payload;
	struct nm_fsm_ev_setattr_data ev_data;

	DEBUGPFOH(DOML, foh, "Rx SET BTS ATTR\n");

	rc = oml_tlv_parse(&tp, foh->data, msgb_l3len(msg) - sizeof(*foh));
	if (rc < 0) {
		oml_tx_failure_event_rep(&bts->mo, NM_SEVER_MAJOR, OSMO_EVT_MAJ_UNSUP_ATTR,
					 "New value for Attribute not supported");
		return oml_fom_ack_nack(msg, NM_NACK_INCORR_STRUCT);
	}

	/* Test for globally unsupported stuff here */
	if (TLVP_PRES_LEN(&tp, NM_ATT_BCCH_ARFCN, 2)) {
		uint16_t arfcn = ntohs(tlvp_val16_unal(&tp, NM_ATT_BCCH_ARFCN));
		if (arfcn >= 1024) { /* 0 .. 1023 (1024 channels total) */
			oml_tx_failure_event_rep(&bts->mo, NM_SEVER_MAJOR, OSMO_EVT_WARN_SW_WARN,
						 "Given ARFCN %u is not supported",
						 arfcn);
			LOGPFOH(DOML, LOGL_ERROR, foh, "Given ARFCN %u is not supported\n", arfcn);
			return oml_fom_ack_nack(msg, NM_NACK_FREQ_NOTAVAIL);
		}
	}
	/* 9.4.52 Starting Time */
	if (TLVP_PRESENT(&tp, NM_ATT_START_TIME)) {
		oml_tx_failure_event_rep(&bts->mo, NM_SEVER_MAJOR, OSMO_EVT_MAJ_UNSUP_ATTR,
					 "NM_ATT_START_TIME Attribute not "
					 "supported");
		return oml_fom_ack_nack(msg, NM_NACK_SPEC_IMPL_NOTSUPP);
	}

	/* merge existing BTS attributes with new attributes */
	tp_merged = osmo_tlvp_copy(bts->mo.nm_attr, bts);
	talloc_set_name_const(tp_merged, "oml_bts_attr");
	osmo_tlvp_merge(tp_merged, &tp);

	/* Ask BTS driver to validate new merged attributes */
	rc = bts_model_check_oml(bts, foh->msg_type, bts->mo.nm_attr, tp_merged, bts);
	if (rc < 0) {
		talloc_free(tp_merged);
		return oml_fom_ack_nack(msg, -rc);
	}

	/* Success: replace old BTS attributes with new */
	talloc_free(bts->mo.nm_attr);
	bts->mo.nm_attr = tp_merged;

	/* ... and actually still parse them */

	/* 9.4.25 Interference Level Boundaries */
	if (TLVP_PRES_LEN(&tp, NM_ATT_INTERF_BOUND, 6)) {
		payload = TLVP_VAL(&tp, NM_ATT_INTERF_BOUND);
		for (i = 0; i < ARRAY_SIZE(bts->interference.boundary); i++) {
			const int16_t boundary = payload[i];
			bts->interference.boundary[i] = -1 * boundary;
		}
	}
	/* 9.4.24 Intave Parameter */
	if (TLVP_PRES_LEN(&tp, NM_ATT_INTAVE_PARAM, 1))
		bts->interference.intave = *TLVP_VAL(&tp, NM_ATT_INTAVE_PARAM);

	/* 9.4.14 Connection Failure Criterion */
	if (TLVP_PRES_LEN(&tp, NM_ATT_CONN_FAIL_CRIT, 1)) {
		const uint8_t *val = TLVP_VAL(&tp, NM_ATT_CONN_FAIL_CRIT);

		switch (val[0]) {
		case 0xFF: /* Osmocom specific Extension of TS 12.21 */
			LOGPFOH(DOML, LOGL_NOTICE, foh, "WARNING: Radio Link Timeout "
				"explicitly disabled, only use this for lab testing!\n");
			bts->radio_link_timeout.oml = -1;
			if (!bts->radio_link_timeout.vty_override)
				bts->radio_link_timeout.current = bts->radio_link_timeout.oml;
			break;
		case 0x01: /* Based on uplink SACCH (radio link timeout) */
			if (TLVP_LEN(&tp, NM_ATT_CONN_FAIL_CRIT) >= 2 &&
			    val[1] >= 4 && val[1] <= 64) {
				bts->radio_link_timeout.oml = val[1];
				if (!bts->radio_link_timeout.vty_override)
					bts->radio_link_timeout.current = bts->radio_link_timeout.oml;
				break;
			}
			/* fall-through */
		case 0x02: /* Based on RXLEV/RXQUAL measurements */
		default:
			LOGPFOH(DOML, LOGL_NOTICE, foh, "Given Conn. Failure Criterion "
				"not supported. Please use criterion 0x01 with "
				"RADIO_LINK_TIMEOUT value of 4..64\n");
			return oml_fom_ack_nack(msg, NM_NACK_PARAM_RANGE);
		}
	}

	/* 9.4.53 T200 */
	if (TLVP_PRES_LEN(&tp, NM_ATT_T200, ARRAY_SIZE(bts->t200_ms))) {
		payload = TLVP_VAL(&tp, NM_ATT_T200);
		for (i = 0; i < ARRAY_SIZE(bts->t200_ms); i++) {
			uint32_t t200_ms = payload[i] * abis_nm_t200_ms[i];
#if 0
			bts->t200_ms[i] = t200_ms;
			DEBUGPFOH(DOML, foh, "T200[%u]: OML=%u, mult=%u => %u ms\n",
				  i, payload[i], abis_nm_t200_ms[i],
				  bts->t200_ms[i]);
#else
                        /* we'd rather use the 1s/2s (long) defaults by
                         * libosmocore, as we appear to have some bug(s)
                         * related to handling T200 expiration in
                         * libosmogsm lapd(m) code? */
                        LOGPFOH(DOML, LOGL_NOTICE, foh, "Ignoring T200[%u] (%u ms) "
                                "as sent by BSC due to suspected LAPDm bug!\n",
                                i, t200_ms);
#endif
		}
	}

	/* 9.4.31 Maximum Timing Advance */
	if (TLVP_PRES_LEN(&tp, NM_ATT_MAX_TA, 1))
		bts->max_ta = *TLVP_VAL(&tp, NM_ATT_MAX_TA);

	/* 9.4.39 Overload Period */
	if (TLVP_PRES_LEN(&tp, NM_ATT_OVERL_PERIOD, 1))
		bts->load.overload_period = *TLVP_VAL(&tp, NM_ATT_OVERL_PERIOD);

	/* 9.4.12 CCCH Load Threshold */
	if (TLVP_PRES_LEN(&tp, NM_ATT_CCCH_L_T, 1))
		bts->load.ccch.load_ind_thresh = *TLVP_VAL(&tp, NM_ATT_CCCH_L_T);

	/* 9.4.11 CCCH Load Indication Period */
	if (TLVP_PRES_LEN(&tp, NM_ATT_CCCH_L_I_P, 1)) {
		bts->load.ccch.load_ind_period = *TLVP_VAL(&tp, NM_ATT_CCCH_L_I_P);
		if (load_timer_is_running(bts)) {
			load_timer_stop(bts);
			load_timer_start(bts);
		}
	}

	/* 9.4.44 RACH Busy Threshold */
	if (TLVP_PRES_LEN(&tp, NM_ATT_RACH_B_THRESH, 1)) {
		int16_t thresh = *TLVP_VAL(&tp, NM_ATT_RACH_B_THRESH);
		bts->load.rach.busy_thresh = -1 * thresh;
	}

	/* 9.4.45 RACH Load Averaging Slots */
	if (TLVP_PRES_LEN(&tp, NM_ATT_LDAVG_SLOTS, 2)) {
		bts->load.rach.averaging_slots =
			ntohs(tlvp_val16_unal(&tp, NM_ATT_LDAVG_SLOTS));
	}

	/* 9.4.10 BTS Air Timer */
	if (TLVP_PRES_LEN(&tp, NM_ATT_BTS_AIR_TIMER, 1)) {
		uint8_t t3105 = *TLVP_VAL(&tp, NM_ATT_BTS_AIR_TIMER);
		if (t3105 == 0) {
			LOGPFOH(DOML, LOGL_NOTICE, foh, "T3105 must have a value != 0\n");
			return oml_fom_ack_nack(msg, NM_NACK_PARAM_RANGE);
		}
		bts->t3105_ms = t3105 * 10;
	}

	/* 9.4.37 NY1 */
	if (TLVP_PRES_LEN(&tp, NM_ATT_NY1, 1))
		bts->ny1 = *TLVP_VAL(&tp, NM_ATT_NY1);

	/* 9.4.8 BCCH ARFCN */
	if (TLVP_PRES_LEN(&tp, NM_ATT_BCCH_ARFCN, 2))
		bts->c0->arfcn = ntohs(tlvp_val16_unal(&tp, NM_ATT_BCCH_ARFCN));

	/* 9.4.9 BSIC */
	if (TLVP_PRES_LEN(&tp, NM_ATT_BSIC, 1)) {
		struct gsm_bts_trx *trx;
		uint8_t bts_tsc;

		bts->bsic = *TLVP_VAL(&tp, NM_ATT_BSIC);
		bts->bsic_configured = true;
		bts_tsc = BTS_TSC(bts);

		/* Apply TSC update on each TS if required: */
		llist_for_each_entry(trx, &bts->trx_list, list) { /* C0..n */
			unsigned int tn;
			for (tn = 0; tn < ARRAY_SIZE(trx->ts); tn++) {
				struct gsm_bts_trx_ts *ts = &trx->ts[tn];

				/* First some config validation: */
				if (ts->tsc_oml_configured &&
				    ts->tsc_oml != bts_tsc &&
				    !osmo_bts_has_feature(bts->features, BTS_FEAT_MULTI_TSC)) {
					LOGPFOH(DOML, LOGL_ERROR, foh, "SET BTS ATTR: this BTS model does not "
						"support TSC %u != BSIC-BCC %u (TSC %u)\n",
						ts->tsc_oml, bts->bsic, bts_tsc);
					return oml_fom_ack_nack(msg, NM_NACK_PARAM_RANGE);
				}

				/* Now update TS TSC if needed: */
				gsm_ts_apply_configured_tsc(ts);
			}
		}
	}

	ev_data = (struct nm_fsm_ev_setattr_data){
		.msg = msg,
	};

	rc = osmo_fsm_inst_dispatch(bts->mo.fi, NM_EV_RX_SETATTR, &ev_data);
	if (rc < 0)
		return oml_fom_ack_nack(msg, NM_NACK_CANT_PERFORM);
	return rc;
}

/* 8.6.2 Set Radio Attributes has been received */
static int oml_rx_set_radio_attr(struct gsm_bts_trx *trx, struct msgb *msg)
{
	struct abis_om_fom_hdr *foh = msgb_l3(msg);
	struct tlv_parsed tp, *tp_merged;
	int rc;
	struct nm_fsm_ev_setattr_data ev_data;

	DEBUGPFOH(DOML, foh, "Rx SET RADIO CARRIER ATTR\n");

	rc = oml_tlv_parse(&tp, foh->data, msgb_l3len(msg) - sizeof(*foh));
	if (rc < 0) {
		oml_tx_failure_event_rep(&trx->mo, NM_SEVER_MAJOR, OSMO_EVT_MAJ_UNSUP_ATTR,
					 "New value for Set Radio Attribute not"
					 " supported");
		return oml_fom_ack_nack(msg, NM_NACK_INCORR_STRUCT);
	}

	/* merge existing TRX attributes with new attributes */
	tp_merged = osmo_tlvp_copy(trx->mo.nm_attr, trx);
	talloc_set_name_const(tp_merged, "oml_trx_attr");
	osmo_tlvp_merge(tp_merged, &tp);

	/* Ask BTS driver to validate new merged attributes */
	rc = bts_model_check_oml(trx->bts, foh->msg_type, trx->mo.nm_attr, tp_merged, trx);
	if (rc < 0) {
		talloc_free(tp_merged);
		return oml_fom_ack_nack(msg, -rc);
	}

	/* Success: replace old TRX attributes with new */
	talloc_free(trx->mo.nm_attr);
	trx->mo.nm_attr = tp_merged;

	/* ... and actually still parse them */

	/* 9.4.47 RF Max Power Reduction */
	if (TLVP_PRES_LEN(&tp, NM_ATT_RF_MAXPOWR_R, 1)) {
		trx->max_power_red = *TLVP_VAL(&tp, NM_ATT_RF_MAXPOWR_R) * 2;
		LOGPFOH(DOML, LOGL_INFO, foh, "Set RF Max Power Reduction to "
			"%d dBm\n", trx->max_power_red);
	}
	/* 9.4.5 ARFCN List */
#if 0
	if (TLVP_PRESENT(&tp, NM_ATT_ARFCN_LIST)) {
		uint8_t *value = TLVP_VAL(&tp, NM_ATT_ARFCN_LIST);
		uint16_t _value;
		uint16_t length = TLVP_LEN(&tp, NM_ATT_ARFCN_LIST);
		uint16_t arfcn;
		int i;
		for (i = 0; i < length; i++) {
			memcpy(&_value, value, 2);
			arfcn = ntohs(_value);
			value += 2;
			if (arfcn > 1024)
				return oml_fom_ack_nack(msg, NM_NACK_FREQ_NOTAVAIL);
			trx->arfcn_list[i] = arfcn;
			LOGPFOH(DOML, LOGL_INFO, foh, " ARFCN list = %d\n", trx->arfcn_list[i]);
		}
		trx->arfcn_num = length;
	} else
		trx->arfcn_num = 0;
#else
	if (trx != trx->bts->c0 && TLVP_PRESENT(&tp, NM_ATT_ARFCN_LIST)) {
		const uint8_t *value = TLVP_VAL(&tp, NM_ATT_ARFCN_LIST);
		uint16_t _value;
		uint16_t length = TLVP_LEN(&tp, NM_ATT_ARFCN_LIST);
		uint16_t arfcn;
		if (length != 2) {
			LOGPFOH(DOML, LOGL_ERROR, foh, "Expecting only one ARFCN, "
				"because hopping not supported\n");
			return oml_fom_ack_nack(msg, NM_NACK_MSGINCONSIST_PHYSCFG);
		}
		memcpy(&_value, value, 2);
		arfcn = ntohs(_value);
		value += 2;
		if (arfcn >= 1024) { /* 0 .. 1023 (1024 channels total) */
			oml_tx_failure_event_rep(&trx->bts->mo, NM_SEVER_MAJOR, OSMO_EVT_WARN_SW_WARN,
						 "Given ARFCN %u is unsupported", arfcn);
			LOGPFOH(DOML, LOGL_NOTICE, foh, "Given ARFCN %u is unsupported\n", arfcn);
			return oml_fom_ack_nack(msg, NM_NACK_FREQ_NOTAVAIL);
		}
		trx->arfcn = arfcn;
	}
#endif

	ev_data = (struct nm_fsm_ev_setattr_data){
		.msg = msg,
	};

	rc = osmo_fsm_inst_dispatch(trx->mo.fi, NM_EV_RX_SETATTR, &ev_data);
	if (rc < 0)
		return oml_fom_ack_nack(msg, NM_NACK_CANT_PERFORM);
	return rc;

}

static int handle_chan_comb(struct gsm_bts_trx_ts *ts, const uint8_t comb)
{
	enum gsm_phys_chan_config pchan;

	pchan = abis_nm_pchan4chcomb(comb);
	ts->pchan = pchan;

	/* RSL_MT_IPAC_PDCH_ACT style dyn PDCH */
	if (pchan == GSM_PCHAN_TCH_F_PDCH)
		pchan = ts->flags & TS_F_PDCH_ACTIVE? GSM_PCHAN_PDCH
						    : GSM_PCHAN_TCH_F;

	/* Osmocom RSL CHAN ACT style dyn TS */
	if (pchan == GSM_PCHAN_OSMO_DYN) {
		pchan = ts->dyn.pchan_is;

		/* If the dyn TS doesn't have a pchan yet, do nothing. */
		if (pchan == GSM_PCHAN_NONE)
			return 0;
	}

	return conf_lchans_as_pchan(ts, pchan);
}

static inline void lchans_type_set(struct gsm_bts_trx_ts *ts,
				   enum gsm_chan_t lchan_type,
				   unsigned int num_lchans)
{
	unsigned int i;

	for (i = 0; i < num_lchans; i++)
		ts->lchan[i].type = lchan_type;
}

int conf_lchans_as_pchan(struct gsm_bts_trx_ts *ts,
			 enum gsm_phys_chan_config pchan)
{
	/* Initialize all lchans with GSM_LCHAN_NONE first */
	lchans_type_set(ts, GSM_LCHAN_NONE, ARRAY_SIZE(ts->lchan));

	switch (pchan) {
	case GSM_PCHAN_CCCH_SDCCH4_CBCH:
	case GSM_PCHAN_CCCH_SDCCH4:
		lchans_type_set(ts, GSM_LCHAN_SDCCH, 4);
		if (pchan == GSM_PCHAN_CCCH_SDCCH4_CBCH)
			ts->lchan[2].type = GSM_LCHAN_CBCH;
		/* fallthrough */
	case GSM_PCHAN_CCCH:
		ts->lchan[CCCH_LCHAN].type = GSM_LCHAN_CCCH;
		break;
	case GSM_PCHAN_TCH_F:
		if (ts->vamos.peer != NULL) { /* VAMOS: enable shadow lchans */
			lchans_type_set(ts->vamos.peer, GSM_LCHAN_TCH_F, 1);
			ts->vamos.peer->pchan = GSM_PCHAN_TCH_F;
		}
		lchans_type_set(ts, GSM_LCHAN_TCH_F, 1);
		break;
	case GSM_PCHAN_TCH_H:
		if (ts->vamos.peer != NULL) { /* VAMOS: enable shadow lchans */
			lchans_type_set(ts->vamos.peer, GSM_LCHAN_TCH_H, 2);
			ts->vamos.peer->pchan = GSM_PCHAN_TCH_H;
		}
		lchans_type_set(ts, GSM_LCHAN_TCH_H, 2);
		break;
	case GSM_PCHAN_SDCCH8_SACCH8C_CBCH:
	case GSM_PCHAN_SDCCH8_SACCH8C:
		lchans_type_set(ts, GSM_LCHAN_SDCCH, 8);
		if (pchan == GSM_PCHAN_SDCCH8_SACCH8C_CBCH)
			ts->lchan[2].type = GSM_LCHAN_CBCH;
		break;
	case GSM_PCHAN_PDCH:
		if (ts->vamos.peer != NULL) { /* VAMOS: disable shadow lchans */
			lchans_type_set(ts->vamos.peer, GSM_LCHAN_NONE, 1);
			ts->vamos.peer->pchan = GSM_PCHAN_NONE;
		}
		lchans_type_set(ts, GSM_LCHAN_PDTCH, 1);
		break;
	default:
		LOGP(DOML, LOGL_ERROR, "Unknown/unhandled PCHAN type: %u %s\n",
		     ts->pchan, gsm_pchan_name(ts->pchan));
		return -NM_NACK_PARAM_RANGE;
	}

	return 0;
}

/* 8.6.3 Set Channel Attributes has been received */
static int oml_rx_set_chan_attr(struct gsm_bts_trx_ts *ts, struct msgb *msg)
{
	struct abis_om_fom_hdr *foh = msgb_l3(msg);
	struct gsm_bts *bts = ts->trx->bts;
	struct tlv_parsed tp, *tp_merged;
	int rc, i;
	struct nm_fsm_ev_setattr_data ev_data;

	DEBUGPFOH(DOML, foh, "Rx SET CHAN ATTR\n");

	rc = oml_tlv_parse(&tp, foh->data, msgb_l3len(msg) - sizeof(*foh));
	if (rc < 0) {
		oml_tx_failure_event_rep(&ts->mo, NM_SEVER_MAJOR, OSMO_EVT_MAJ_UNSUP_ATTR,
					 "New value for Set Channel Attribute not supported");
		return oml_fom_ack_nack(msg, NM_NACK_INCORR_STRUCT);
	}

	/* Check frequency hopping parameters (HSN, MAIO, ARFCN list) */
	if (TLVP_PRESENT(&tp, NM_ATT_HSN) || TLVP_PRESENT(&tp, NM_ATT_MAIO)) {
		if (!osmo_bts_has_feature(bts->features, BTS_FEAT_HOPPING)) {
			LOGPFOH(DOML, LOGL_ERROR, foh, "SET CHAN ATTR: Frequency hopping not supported\n");
			return oml_fom_ack_nack(msg, NM_NACK_SPEC_IMPL_NOTSUPP);
		}

		if (!TLVP_PRES_LEN(&tp, NM_ATT_HSN, 1) || !TLVP_PRES_LEN(&tp, NM_ATT_MAIO, 1)) {
			LOGPFOH(DOML, LOGL_ERROR, foh, "SET CHAN ATTR: HSN and/or MAIO is missing: "
				"hsn=%u, maio=%u\n", TLVP_LEN(&tp, NM_ATT_HSN), TLVP_LEN(&tp, NM_ATT_MAIO));
			return oml_fom_ack_nack(msg, NM_NACK_ATTRLIST_INCONSISTENT);
		}

		if (!TLVP_PRES_LEN(&tp, NM_ATT_ARFCN_LIST, 2)) { /* At least one ARFCN */
			LOGPFOH(DOML, LOGL_ERROR, foh, "SET CHAN ATTR: ARFCN list is missing\n");
			return oml_fom_ack_nack(msg, NM_NACK_ATTRLIST_INCONSISTENT);
		}

		if (TLVP_LEN(&tp, NM_ATT_ARFCN_LIST) > sizeof(ts->hopping.arfcn_list)) {
			LOGPFOH(DOML, LOGL_ERROR, foh, "SET CHAN ATTR: ARFCN list is too long\n");
			return oml_fom_ack_nack(msg, NM_NACK_ATTRLIST_INCONSISTENT);
		} else if (TLVP_LEN(&tp, NM_ATT_ARFCN_LIST) % 2 != 0) {
			LOGPFOH(DOML, LOGL_ERROR, foh, "SET CHAN ATTR: ARFCN list has odd length\n");
			return oml_fom_ack_nack(msg, NM_NACK_ATTRLIST_INCONSISTENT);
		}

		ts->hopping.enabled = true;
		ts->hopping.hsn = *TLVP_VAL(&tp, NM_ATT_HSN);
		ts->hopping.maio = *TLVP_VAL(&tp, NM_ATT_MAIO);

		ts->hopping.arfcn_num = TLVP_LEN(&tp, NM_ATT_ARFCN_LIST) / sizeof(uint16_t);
		for (i = 0; i < ts->hopping.arfcn_num; i++)
			ts->hopping.arfcn_list[i] = osmo_load16be(TLVP_VAL(&tp, NM_ATT_ARFCN_LIST) + i * 2);
	}

	/* 9.4.52 Starting Time */
	if (TLVP_PRESENT(&tp, NM_ATT_START_TIME)) {
		LOGPFOH(DOML, LOGL_NOTICE, foh, "SET CHAN ATTR: Starting time not supported\n");
		return oml_fom_ack_nack(msg, NM_NACK_SPEC_IMPL_NOTSUPP);
	}

	/* merge existing CHAN attributes with new attributes */
	tp_merged = osmo_tlvp_copy(ts->mo.nm_attr, ts->trx);
	talloc_set_name_const(tp_merged, "oml_chan_attr");
	osmo_tlvp_merge(tp_merged, &tp);

	/* Call into BTS driver to check attribute values */
	rc = bts_model_check_oml(bts, foh->msg_type, ts->mo.nm_attr, tp_merged, ts);
	if (rc < 0) {
		LOGPFOH(DOML, LOGL_ERROR, foh, "SET CHAN ATTR: invalid attribute value, rc=%d\n", rc);
		talloc_free(tp_merged);
		/* Send NACK */
		return oml_fom_ack_nack(msg, -rc);
	}

	/* Success: replace old CHAN attributes with new */
	talloc_free(ts->mo.nm_attr);
	ts->mo.nm_attr = tp_merged;

	/* 9.4.13 Channel Combination */
	if (TLVP_PRES_LEN(&tp, NM_ATT_CHAN_COMB, 1)) {
		const uint8_t comb = *TLVP_VAL(&tp, NM_ATT_CHAN_COMB);
		if ((rc = handle_chan_comb(ts, comb)) != 0) {
			LOGPFOH(DOML, LOGL_ERROR, foh, "SET CHAN ATTR: invalid Chan Comb 0x%x"
				" (pchan=%s, handle_chan_comb() returns %d)\n",
				comb, gsm_pchan_name(ts->pchan), rc);
			talloc_free(tp_merged);
			/* Send NACK */
			return oml_fom_ack_nack(msg, -rc);
		}
	}

	/* 9.4.60 TSC */
	if (TLVP_PRES_LEN(&tp, NM_ATT_TSC, 1)) {
		ts->tsc_oml = *TLVP_VAL(&tp, NM_ATT_TSC);
		ts->tsc_oml_configured = true;
	}

	if (ts->tsc_oml_configured) {
		if (bts->bsic_configured &&
		    ts->tsc_oml != BTS_TSC(bts) &&
		    !osmo_bts_has_feature(bts->features, BTS_FEAT_MULTI_TSC)) {
			LOGPFOH(DOML, LOGL_ERROR, foh, "SET CHAN ATTR: this BTS model does not "
				"support TSC %u != BSIC-BCC %u\n", ts->tsc_oml, BTS_TSC(bts));
			talloc_free(tp_merged);
			return oml_fom_ack_nack(msg, NM_NACK_PARAM_RANGE);
		}
		gsm_ts_apply_configured_tsc(ts);
	}

	LOGPFOH(DOML, LOGL_INFO, foh, "SET CHAN ATTR (TSC=%d pchan=%s",
		ts->tsc_oml_configured ? (int)ts->tsc_oml : -1,
		gsm_pchan_name(ts->pchan));
	if (ts->hopping.enabled)
		LOGPC(DOML, LOGL_INFO, " hsn=%u maio=%u chan_num=%u",
		      ts->hopping.hsn, ts->hopping.maio, ts->hopping.arfcn_num);
	LOGPC(DOML, LOGL_INFO, ")\n");

	ev_data = (struct nm_fsm_ev_setattr_data){
		.msg = msg,
	};

	rc = osmo_fsm_inst_dispatch(ts->mo.fi, NM_EV_RX_SETATTR, &ev_data);
	if (rc < 0)
		return oml_fom_ack_nack(msg, NM_NACK_CANT_PERFORM);
	return rc;
}

/* 8.9.2 Opstart has been received */
static int oml_rx_opstart(struct gsm_bts *bts, struct msgb *msg)
{
	struct abis_om_fom_hdr *foh = msgb_l3(msg);
	struct gsm_abis_mo *mo;
	void *obj;
	int rc;
	enum abis_nm_nack_cause c;

	DEBUGPFOH(DOML, foh, "Rx OPSTART\n");

	/* Step 1: Resolve MO by obj_class/obj_inst */
	if ((mo = gsm_objclass2mo(bts, foh->obj_class, &foh->obj_inst, &c)) == NULL)
		return oml_fom_ack_nack(msg, c);
	if ((obj = gsm_objclass2obj(bts, foh->obj_class, &foh->obj_inst, &c)) == NULL)
		return oml_fom_ack_nack(msg, c);

	/* Step 2: Do some global dependency/consistency checking */
	if (mo->nm_state.operational == NM_OPSTATE_ENABLED) {
		DEBUGPFOH(DOML, foh, "... automatic ACK, OP state already was Enabled\n");
		return oml_mo_opstart_ack(mo);
	}

	/* Make sure all NM objects already have an FSM implemented: */
	OSMO_ASSERT(mo->fi);

	rc = osmo_fsm_inst_dispatch(mo->fi, NM_EV_RX_OPSTART, NULL);
	if (rc < 0)
		return oml_fom_ack_nack(msg, NM_NACK_CANT_PERFORM);
	return rc;
}

static int oml_rx_chg_adm_state(struct gsm_bts *bts, struct msgb *msg)
{
	struct abis_om_fom_hdr *foh = msgb_l3(msg);
	struct tlv_parsed tp;
	struct gsm_abis_mo *mo;
	uint8_t adm_state;
	void *obj;
	int rc;
	enum abis_nm_nack_cause c;

	DEBUGPFOH(DOML, foh, "Rx CHG ADM STATE\n");

	rc = oml_tlv_parse(&tp, foh->data, msgb_l3len(msg) - sizeof(*foh));
	if (rc < 0) {
		LOGPFOH(DOML, LOGL_ERROR, foh, "Rx CHG ADM STATE: error during TLV parse\n");
		return oml_fom_ack_nack(msg, NM_NACK_INCORR_STRUCT);
	}

	if (!TLVP_PRESENT(&tp, NM_ATT_ADM_STATE)) {
		LOGPFOH(DOML, LOGL_ERROR, foh, "Rx CHG ADM STATE: no ADM state attribute\n");
		return oml_fom_ack_nack(msg, NM_NACK_INCORR_STRUCT);
	}

	adm_state = *TLVP_VAL(&tp, NM_ATT_ADM_STATE);

	/* Step 1: Resolve MO by obj_class/obj_inst */
	if ((mo = gsm_objclass2mo(bts, foh->obj_class, &foh->obj_inst, &c)) == NULL)
		return oml_fom_ack_nack(msg, c);
	if ((obj = gsm_objclass2obj(bts, foh->obj_class, &foh->obj_inst, &c)) == NULL)
		return oml_fom_ack_nack(msg, c);

	/* Step 2: Do some global dependency/consistency checking */
	if (mo->nm_state.administrative == adm_state) {
		LOGPFOH(DOML, LOGL_NOTICE, foh, "ADM state already was %s\n",
			get_value_string(abis_nm_adm_state_names, adm_state));
		return oml_mo_statechg_ack(mo);
	}
	LOGPFOH(DOML, LOGL_NOTICE, foh, "ADM STATE %s -> %s\n",
		get_value_string(abis_nm_adm_state_names, mo->nm_state.administrative),
		get_value_string(abis_nm_adm_state_names, adm_state));

	/* Step 3: Ask BTS driver to apply the state chg */
	return bts_model_chg_adm_state(bts, mo, obj, adm_state);
}

/* Check and report if the BTS number received via OML is incorrect:
   according to 3GPP TS 52.021 §9.3 BTS number is used to distinguish between different BTS of the same Site Manager.
   As we always have only single BTS per Site Manager (in case of Abis/IP with each BTS having dedicated OML connection
   to BSC), the only valid values are 0 and 0xFF (means all BTS' of a given Site Manager). */
static inline bool report_bts_number_incorrect(struct gsm_bts *bts, const struct abis_om_fom_hdr *foh, bool is_formatted)
{
	struct gsm_bts_trx *trx;
	const struct gsm_abis_mo *mo = &bts->mo;
	const char *form = is_formatted ?
		"Unexpected BTS %d in formatted O&M %s (exp. 0 or 0xFF)" :
		"Unexpected BTS %d in manufacturer O&M %s (exp. 0 or 0xFF)";

	if (foh->obj_inst.bts_nr != 0 && foh->obj_inst.bts_nr != 0xff) {
		trx = gsm_bts_trx_num(bts, foh->obj_inst.trx_nr);
		if (trx)
			mo = &trx->mo;
		oml_tx_failure_event_rep(mo, NM_SEVER_MAJOR, OSMO_EVT_MAJ_UKWN_MSG, form,
					 foh->obj_inst.bts_nr,
					 get_value_string(abis_nm_msgtype_names, foh->msg_type));

		return true;
	}

	return false;
}

static int down_fom(struct gsm_bts *bts, struct msgb *msg)
{
	struct abis_om_hdr *oh = msgb_l2(msg);
	struct abis_om_fom_hdr *foh = msgb_l3(msg);
	struct gsm_bts_trx *trx;
	const struct gsm_abis_mo *mo = &bts->mo;
	int ret;

	if (msgb_l2len(msg) < sizeof(*foh)) {
		trx = gsm_bts_trx_num(bts, foh->obj_inst.trx_nr);
		if (trx)
			mo = &trx->mo;
		oml_tx_failure_event_rep(mo, NM_SEVER_MAJOR, OSMO_EVT_MAJ_UKWN_MSG,
					 "Formatted O&M message too short");
		return -EIO;
	}

	if (msgb_l3len(msg) > oh->length) {
		if (oh->mdisc == ABIS_OM_MDISC_FOM && oh->data[0] == NM_MT_GET_ATTR &&
		    msgb_l3len(msg) == oh->length + 3) {
			/* work-around a bug present in OsmoBSC before February 2019 */
			DEBUGPFOH(DOML, foh, "GET ATTR with off-by-3 length: Fixing up for OS#3799\n");
			oh->length += 3;
		} else {
			LOGPFOH(DOML, LOGL_NOTICE, foh, "OML message with %u extraneous bytes at end: %s\n",
				msgb_l3len(msg) - oh->length, msgb_hexdump(msg));
			/* remove extra bytes at end */
			msgb_l3trim(msg, oh->length);
		}
	}

	if (report_bts_number_incorrect(bts, foh, true))
		return oml_fom_ack_nack(msg, NM_NACK_BTSNR_UNKN);

	switch (foh->msg_type) {
	case NM_MT_SET_BTS_ATTR:
		ret = oml_rx_set_bts_attr(bts, msg);
		break;
	case NM_MT_SET_RADIO_ATTR:
		trx = gsm_bts_trx_num(bts, foh->obj_inst.trx_nr);
		if (!trx)
			return oml_fom_ack_nack(msg, NM_NACK_TRXNR_UNKN);
		ret = oml_rx_set_radio_attr(trx, msg);
		break;
	case NM_MT_SET_CHAN_ATTR:
		trx = gsm_bts_trx_num(bts, foh->obj_inst.trx_nr);
		if (!trx)
			return oml_fom_ack_nack(msg, NM_NACK_TRXNR_UNKN);
		if (foh->obj_inst.ts_nr >= ARRAY_SIZE(trx->ts))
			return oml_fom_ack_nack(msg, NM_NACK_OBJINST_UNKN);
		ret = oml_rx_set_chan_attr(&trx->ts[foh->obj_inst.ts_nr], msg);
		break;
	case NM_MT_OPSTART:
		ret = oml_rx_opstart(bts, msg);
		break;
	case NM_MT_CHG_ADM_STATE:
		ret = oml_rx_chg_adm_state(bts, msg);
		break;
	case NM_MT_IPACC_SET_ATTR:
		ret = oml_ipa_set_attr(bts, msg);
		break;
	case NM_MT_GET_ATTR:
		ret = oml_rx_get_attr(bts, msg);
		break;
	default:
		trx = gsm_bts_trx_num(bts, foh->obj_inst.trx_nr);
		if (trx)
			mo = &trx->mo;
		oml_tx_failure_event_rep(mo, NM_SEVER_MINOR, OSMO_EVT_MAJ_UKWN_MSG,
					 "unknown Formatted O&M msg_type 0x%02x", foh->msg_type);
		ret = oml_fom_ack_nack(msg, NM_NACK_MSGTYPE_INVAL);
	}

	return ret;
}

/*
 * manufacturer related messages
 */

static int oml_ipa_mo_set_attr_nse(void *obj,
				   const struct msgb *msg,
				   const struct tlv_parsed *tp)
{
	struct gsm_gprs_nse *nse = obj;
	struct gsm_bts *bts = gsm_gprs_nse_get_bts(nse);
	struct nm_fsm_ev_setattr_data ev_data;
	int rc;

	if (TLVP_PRES_LEN(tp, NM_ATT_IPACC_NSEI, 2))
		nse->nsei =
			ntohs(tlvp_val16_unal(tp, NM_ATT_IPACC_NSEI));

	if (TLVP_PRES_LEN(tp, NM_ATT_IPACC_NS_CFG, 7)) {
		memcpy(&nse->timer,
		       TLVP_VAL(tp, NM_ATT_IPACC_NS_CFG), 7);
	}

	if (TLVP_PRES_LEN(tp, NM_ATT_IPACC_BSSGP_CFG, 11)) {
		memcpy(&bts->gprs.cell.timer,
		       TLVP_VAL(tp, NM_ATT_IPACC_BSSGP_CFG), 11);
	}

	ev_data = (struct nm_fsm_ev_setattr_data){
		.msg = msg,
	};
	rc = osmo_fsm_inst_dispatch(nse->mo.fi, NM_EV_RX_SETATTR, &ev_data);
	if (rc < 0)
		return NM_NACK_CANT_PERFORM;

	osmo_signal_dispatch(SS_GLOBAL, S_NEW_NSE_ATTR, bts);

	return 0;
}

static int oml_ipa_mo_set_attr_cell(void *obj,
				    const struct msgb *msg,
				    const struct tlv_parsed *tp)
{
	struct gsm_gprs_cell *gprs_cell = obj;
	struct gsm_bts *bts = gsm_gprs_cell_get_bts(gprs_cell);
	struct gprs_rlc_cfg *rlcc = &gprs_cell->rlc_cfg;
	struct nm_fsm_ev_setattr_data ev_data;
	int rc;
	const uint8_t *cur;
	uint16_t _cur_s;

	if (TLVP_PRES_LEN(tp, NM_ATT_IPACC_RAC, 1))
		bts->gprs.rac = *TLVP_VAL(tp, NM_ATT_IPACC_RAC);

	if (TLVP_PRES_LEN(tp, NM_ATT_IPACC_GPRS_PAGING_CFG, 2)) {
		cur = TLVP_VAL(tp, NM_ATT_IPACC_GPRS_PAGING_CFG);
		rlcc->paging.repeat_time = cur[0] * 50;
		rlcc->paging.repeat_count = cur[1];
	}

	if (TLVP_PRES_LEN(tp, NM_ATT_IPACC_BVCI, 2))
		gprs_cell->bvci = ntohs(tlvp_val16_unal(tp, NM_ATT_IPACC_BVCI));

	if (TLVP_PRES_LEN(tp, NM_ATT_IPACC_RLC_CFG, 9)) {
		cur = TLVP_VAL(tp, NM_ATT_IPACC_RLC_CFG);
		rlcc->parameter[RLC_T3142] = cur[0];
		rlcc->parameter[RLC_T3169] = cur[1];
		rlcc->parameter[RLC_T3191] = cur[2];
		rlcc->parameter[RLC_T3193] = cur[3];
		rlcc->parameter[RLC_T3195] = cur[4];
		rlcc->parameter[RLC_N3101] = cur[5];
		rlcc->parameter[RLC_N3103] = cur[6];
		rlcc->parameter[RLC_N3105] = cur[7];
		rlcc->parameter[CV_COUNTDOWN] = cur[8];
	}

	if (TLVP_PRES_LEN(tp, NM_ATT_IPACC_CODING_SCHEMES, 2)) {
		int i;
		rlcc->cs_mask = 0;
		cur = TLVP_VAL(tp, NM_ATT_IPACC_CODING_SCHEMES);

		for (i = 0; i < 4; i++) {
			if (cur[0] & (1 << i))
				rlcc->cs_mask |= (1 << (GPRS_CS1+i));
		}
		if (cur[0] & 0x80)
			rlcc->cs_mask |= (1 << GPRS_MCS9);
		for (i = 0; i < 8; i++) {
			if (cur[1] & (1 << i))
				rlcc->cs_mask |= (1 << (GPRS_MCS1+i));
		}
	}

	if (TLVP_PRES_LEN(tp, NM_ATT_IPACC_RLC_CFG_2, 5)) {
		cur = TLVP_VAL(tp, NM_ATT_IPACC_RLC_CFG_2);
		memcpy(&_cur_s, cur, 2);
		rlcc->parameter[T_DL_TBF_EXT] = ntohs(_cur_s) * 10;
		cur += 2;
		memcpy(&_cur_s, cur, 2);
		rlcc->parameter[T_UL_TBF_EXT] = ntohs(_cur_s) * 10;
		cur += 2;
		rlcc->initial_cs = *cur;
	}

	if (TLVP_PRES_LEN(tp, NM_ATT_IPACC_RLC_CFG_3, 1)) {
		rlcc->initial_mcs = *TLVP_VAL(tp, NM_ATT_IPACC_RLC_CFG_3);
	}

	ev_data = (struct nm_fsm_ev_setattr_data){
		.msg = msg,
	};
	rc = osmo_fsm_inst_dispatch(gprs_cell->mo.fi, NM_EV_RX_SETATTR, &ev_data);
	if (rc < 0)
		return NM_NACK_CANT_PERFORM;

	osmo_signal_dispatch(SS_GLOBAL, S_NEW_CELL_ATTR, bts);

	return 0;
}

static int oml_ipa_mo_set_attr_nsvc(struct gsm_gprs_nsvc *nsvc,
				    const struct msgb *msg,
				    const struct tlv_parsed *tp)
{
	struct nm_fsm_ev_setattr_data ev_data;
	int rc;

	if (TLVP_PRES_LEN(tp, NM_ATT_IPACC_NSVCI, 2))
		nsvc->nsvci = ntohs(tlvp_val16_unal(tp, NM_ATT_IPACC_NSVCI));

	if (TLVP_PRES_LEN(tp, NM_ATT_IPACC_NS_LINK_CFG, 8)) {
		const uint8_t *cur = TLVP_VAL(tp, NM_ATT_IPACC_NS_LINK_CFG);
		uint16_t _cur_s;
		uint32_t _cur_l;

		memset(&nsvc->local, 0, sizeof(nsvc->local));
		memset(&nsvc->remote, 0, sizeof(nsvc->remote));

		nsvc->local.u.sin.sin_family = AF_INET;
		nsvc->remote.u.sin.sin_family = AF_INET;

		memcpy(&_cur_s, cur, 2);
		nsvc->remote.u.sin.sin_port = _cur_s;
		cur += 2;
		memcpy(&_cur_l, cur, 4);
		nsvc->remote.u.sin.sin_addr.s_addr = _cur_l;
		cur += 4;
		memcpy(&_cur_s, cur, 2);
		nsvc->local.u.sin.sin_port = _cur_s;
	}

	if (TLVP_PRES_LEN(tp, NM_ATT_OSMO_NS_LINK_CFG, 10)) {
		const uint8_t *cur = TLVP_VAL(tp, NM_ATT_OSMO_NS_LINK_CFG);
		uint8_t address_family;

		memset(&nsvc->local, 0, sizeof(nsvc->local));
		memset(&nsvc->remote, 0, sizeof(nsvc->remote));

		address_family = *cur;
		/* 1byte padding */
		cur += 2;

		memcpy(&nsvc->local.u.sin.sin_port, cur, 2);
		cur += 2;

		memcpy(&nsvc->remote.u.sin.sin_port, cur, 2);
		cur += 2;

		switch (address_family) {
		case OSMO_NSVC_ADDR_IPV4:
			/* we already checked for 10 bytes */
			nsvc->remote.u.sas.ss_family = AF_INET;
			nsvc->local.u.sas.ss_family = AF_INET;
			memcpy(&nsvc->remote.u.sin.sin_addr.s_addr, cur, sizeof(in_addr_t));
			break;
		case OSMO_NSVC_ADDR_IPV6:
			if (TLVP_LEN(tp, NM_ATT_OSMO_NS_LINK_CFG) < 22) {
				return -1;
			}
			nsvc->remote.u.sas.ss_family = AF_INET6;
			nsvc->local.u.sas.ss_family = AF_INET6;
			memcpy(&nsvc->remote.u.sin6.sin6_addr, cur, sizeof(struct in6_addr));
			break;
		default:
			return -1;
		}
	}

	ev_data = (struct nm_fsm_ev_setattr_data){
		.msg = msg,
	};
	rc = osmo_fsm_inst_dispatch(nsvc->mo.fi, NM_EV_RX_SETATTR, &ev_data);
	if (rc < 0)
		return NM_NACK_CANT_PERFORM;

	osmo_signal_dispatch(SS_GLOBAL, S_NEW_NSVC_ATTR, nsvc);

	return 0;
}

static int oml_ipa_set_attr(struct gsm_bts *bts, struct msgb *msg)
{
	struct abis_om_fom_hdr *foh = msgb_l3(msg);
	struct gsm_abis_mo *mo;
	struct tlv_parsed tp, *tp_merged;
	void *obj;
	int rc;
	enum abis_nm_nack_cause c;

	DEBUGPFOH(DOML, foh, "Rx IPA SET ATTR\n");

	rc = oml_tlv_parse(&tp, foh->data, msgb_l3len(msg) - sizeof(*foh));
	if (rc < 0) {
		if ((mo = gsm_objclass2mo(bts, foh->obj_class, &foh->obj_inst, &c)) == NULL)
			return oml_fom_ack_nack(msg, c);
		oml_tx_failure_event_rep(mo, NM_SEVER_MAJOR, OSMO_EVT_MAJ_UNSUP_ATTR,
					 "New value for IPAC Set Attribute not supported\n");
		return oml_fom_ack_nack(msg, NM_NACK_INCORR_STRUCT);
	}

	/* Resolve MO by obj_class/obj_inst */
	if ((mo = gsm_objclass2mo(bts, foh->obj_class, &foh->obj_inst, &c)) == NULL)
		return oml_fom_ack_nack(msg, c);
	if ((obj = gsm_objclass2obj(bts, foh->obj_class, &foh->obj_inst, &c)) == NULL)
		return oml_fom_ack_nack(msg, c);


	switch (mo->obj_class) {
	case NM_OC_GPRS_NSE:
		rc =  oml_ipa_mo_set_attr_nse(obj, msg, &tp);
		break;
	case NM_OC_GPRS_CELL:
		rc = oml_ipa_mo_set_attr_cell(obj, msg, &tp);
		break;
	case NM_OC_GPRS_NSVC:
		rc = oml_ipa_mo_set_attr_nsvc(obj, msg, &tp);
		break;
	default:
		rc = NM_NACK_OBJINST_UNKN;
	}

	if (rc != 0)
		return oml_fom_ack_nack(msg, rc);

	/* Success: replace old MO attributes with new */
	/* merge existing MO attributes with new attributes */
	tp_merged = osmo_tlvp_copy(mo->nm_attr, bts);
	talloc_set_name_const(tp_merged, "oml_ipa_attr");
	osmo_tlvp_merge(tp_merged, &tp);
	talloc_free(mo->nm_attr);
	mo->nm_attr = tp_merged;

	return rc;
}

static int rx_oml_ipa_rsl_connect(struct gsm_bts *bts, struct msgb *msg,
				  const struct tlv_parsed *tp)
{
	struct e1inp_sign_link *oml_link = bts->oml_link;
	const struct abis_om_fom_hdr *foh = msgb_l3(msg);
	struct gsm_bts_trx *trx = gsm_bts_trx_num(bts, foh->obj_inst.trx_nr);
	const char *trx_name;
	struct in_addr in;
	uint16_t port = IPA_TCP_PORT_RSL;
	uint8_t stream_id = 0;
	int rc;

	if (TLVP_PRESENT(tp, NM_ATT_IPACC_DST_IP))
		in.s_addr = tlvp_val32_unal(tp, NM_ATT_IPACC_DST_IP);
	else
		in.s_addr = htonl(get_signlink_remote_ip(oml_link));

	if (TLVP_PRESENT(tp, NM_ATT_IPACC_DST_IP_PORT))
		port = ntohs(tlvp_val16_unal(tp, NM_ATT_IPACC_DST_IP_PORT));
	if (TLVP_PRESENT(tp, NM_ATT_IPACC_STREAM_ID))
		stream_id = *TLVP_VAL(tp, NM_ATT_IPACC_STREAM_ID);

	if (!trx) {
		LOGP(DOML, LOGL_ERROR, "Rx IPA RSL CONNECT IP=%s PORT=%u STREAM=0x%02x for unknown TRX_NR=%u\n",
		     inet_ntoa(in), port, stream_id, foh->obj_inst.trx_nr);
		rc = NM_NACK_TRXNR_UNKN;
		goto tx_ack_nack;
	}

	trx_name = gsm_trx_name(trx);

	LOGP(DOML, LOGL_INFO, "%s: Rx IPA RSL CONNECT IP=%s PORT=%u STREAM=0x%02x\n",
	     trx_name, inet_ntoa(in), port, stream_id);

	if (bts->variant == BTS_OSMO_OMLDUMMY) {
		rc = 0;
		LOGP(DOML, LOGL_NOTICE, "%s: Not connecting RSL in OML-DUMMY!\n", trx_name);
	} else {
		trx->rsl_tei = stream_id;
		rc = e1inp_ipa_bts_rsl_connect_n(oml_link->ts->line, inet_ntoa(in), port, trx->nr);
		if (rc < 0) {
			LOGP(DOML, LOGL_NOTICE, "%s: Error connecting IPA RSL: %d\n", trx_name, rc);
			rc = NM_NACK_CANT_PERFORM;
		}
	}

tx_ack_nack:
	/* The ACK/NACK is expected to contain all IEs */
	if (!TLVP_PRESENT(tp, NM_ATT_IPACC_DST_IP)) /* TV32 */
		msgb_tv_fixed_put(msg, NM_ATT_IPACC_DST_IP, sizeof(in),
				  (const uint8_t *) &in);
	if (!TLVP_PRESENT(tp, NM_ATT_IPACC_DST_IP_PORT)) /* TV16 */
		msgb_tv16_put(msg, NM_ATT_IPACC_DST_IP_PORT, port);
	if (!TLVP_PRESENT(tp, NM_ATT_IPACC_STREAM_ID)) /* TV */
		msgb_tv_put(msg, NM_ATT_IPACC_STREAM_ID, stream_id);

	return oml_fom_ack_nack(msg, rc);
}

static int down_mom(struct gsm_bts *bts, struct msgb *msg)
{
	struct abis_om_hdr *oh = msgb_l2(msg);
	const struct gsm_abis_mo *mo = &bts->mo;
	struct abis_om_fom_hdr *foh;
	struct gsm_bts_trx *trx;
	uint8_t idstrlen = oh->data[0];
	struct tlv_parsed tp;
	int ret;

	if (msgb_l2len(msg) < sizeof(*foh)) {
		oml_tx_failure_event_rep(&bts->mo, NM_SEVER_MAJOR, OSMO_EVT_MAJ_UKWN_MSG,
					 "Manufacturer O&M message too short\n");
		return -EIO;
	}

	if (strncmp((char *)&oh->data[1], abis_nm_ipa_magic, idstrlen)) {
		oml_tx_failure_event_rep(&bts->mo, NM_SEVER_MAJOR, OSMO_EVT_MAJ_UKWN_MSG,
					 "Manufacturer OML message != ipaccess not supported\n");
		return -EINVAL;
	}

	if (msgb_l3len(msg) > oh->length + 1 + oh->data[0]) {
		LOGP(DOML, LOGL_NOTICE, "OML message with %u extraneous bytes at end: %s\n",
			msgb_l3len(msg) - oh->length, msgb_hexdump(msg));
		/* remove extra bytes at end */
		msgb_l3trim(msg, oh->length);
	}

	msg->l3h = oh->data + 1 + idstrlen;
	foh = (struct abis_om_fom_hdr *) msg->l3h;

	if (report_bts_number_incorrect(bts, foh, false))
		return oml_fom_ack_nack(msg, NM_NACK_BTSNR_UNKN);

	ret = oml_tlv_parse(&tp, foh->data, oh->length - sizeof(*foh));
	if (ret < 0) {
		LOGPFOH(DOML, LOGL_ERROR, foh, "TLV parse error %d\n", ret);
		return oml_fom_ack_nack(msg, NM_NACK_BTSNR_UNKN);
	}

	DEBUGPFOH(DOML, foh, "Rx IPACCESS(0x%02x): %s\n", foh->msg_type,
		  osmo_hexdump(foh->data, oh->length - sizeof(*foh)));

	switch (foh->msg_type) {
	case NM_MT_IPACC_RSL_CONNECT:
		ret = rx_oml_ipa_rsl_connect(bts, msg, &tp);
		break;
	case NM_MT_IPACC_SET_ATTR:
		ret = oml_ipa_set_attr(bts, msg);
		break;
	default:
		trx = gsm_bts_trx_num(bts, foh->obj_inst.trx_nr);
		if (trx)
			mo = &trx->mo;
		oml_tx_failure_event_rep(mo, NM_SEVER_MINOR, OSMO_EVT_MAJ_UKWN_MSG,
					 "unknown Manufacturer O&M msg_type 0x%02x", foh->msg_type);
		ret = oml_fom_ack_nack(msg, NM_NACK_MSGTYPE_INVAL);
	}

	return ret;
}

/* incoming OML message from BSC */
int down_oml(struct gsm_bts *bts, struct msgb *msg)
{
	struct abis_om_hdr *oh = msgb_l2(msg);
	int ret = 0;

	if (msgb_l2len(msg) < sizeof(*oh)) {
		oml_tx_failure_event_rep(&bts->mo, NM_SEVER_MAJOR, OSMO_EVT_MAJ_UKWN_MSG,
					 "OML message too short\n");
		msgb_free(msg);
		return -EIO;
	}
	msg->l3h = (unsigned char *)oh + sizeof(*oh);

	/* We don't implement de-segmentation of segmented OML messages */
	if (oh->placement != ABIS_OM_PLACEMENT_ONLY || oh->sequence != 0) {
		oml_tx_failure_event_rep(&bts->mo, NM_SEVER_MAJOR, OSMO_EVT_MAJ_UKWN_MSG,
					 "Unsupported segmented O&M message\n");
		msgb_free(msg);
		return -EIO;
	}

	if (msgb_l3len(msg) < oh->length) {
		oml_tx_failure_event_rep(&bts->mo, NM_SEVER_MAJOR, OSMO_EVT_MAJ_UKWN_MSG,
					 "Short OML message: %u < %u\n",
					 msgb_l3len(msg), oh->length);
		msgb_free(msg);
		return -EIO;
	}

	switch (oh->mdisc) {
	case ABIS_OM_MDISC_FOM:
		if (msgb_l2len(msg) < sizeof(*oh)) {
			oml_tx_failure_event_rep(&bts->mo, NM_SEVER_MAJOR, OSMO_EVT_MAJ_UKWN_MSG,
						"Formatted O&M message too short\n");
			ret = -EIO;
			break;
		}
		ret = down_fom(bts, msg);
		break;
	case ABIS_OM_MDISC_MANUF:
		if (msgb_l2len(msg) < sizeof(*oh)) {
			oml_tx_failure_event_rep(&bts->mo, NM_SEVER_MAJOR, OSMO_EVT_MAJ_UKWN_MSG,
						"Manufacturer O&M message too short\n");
			ret = -EIO;
			break;
		}
		ret = down_mom(bts, msg);
		break;
	default:
		oml_tx_failure_event_rep(&bts->mo, NM_SEVER_MINOR, OSMO_EVT_MAJ_UKWN_MSG,
					 "unknown O&M msg_disc 0x%02x\n", oh->mdisc);
		ret = -EINVAL;
	}

	/* msgb was reused, do not free() */
	if (ret == 1)
		return 0;

	msgb_free(msg);

	return ret;
}

int oml_init()
{
	DEBUGP(DOML, "Initializing OML attribute definitions\n");
	tlv_def_patch(&abis_nm_att_tlvdef_ipa_local, &abis_nm_att_tlvdef_ipa);
	tlv_def_patch(&abis_nm_att_tlvdef_ipa_local, &abis_nm_att_tlvdef);
	tlv_def_patch(&abis_nm_att_tlvdef_ipa_local, &abis_nm_osmo_att_tlvdef);

	return 0;
}

void gsm_mo_init(struct gsm_abis_mo *mo, struct gsm_bts *bts,
		 uint8_t obj_class, uint8_t p1, uint8_t p2, uint8_t p3)
{
	mo->bts = bts;
	mo->obj_class = obj_class;
	mo->obj_inst.bts_nr = p1;
	mo->obj_inst.trx_nr = p2;
	mo->obj_inst.ts_nr = p3;
	mo->nm_state.operational = NM_OPSTATE_DISABLED;
	mo->nm_state.availability = NM_AVSTATE_POWER_OFF;
	mo->nm_state.administrative = NM_STATE_LOCKED;
}

/* Obtain the MO structure for a given object instance
 *  \param[out] c nack cause for reply in case of error. Ignored if NULL */
struct gsm_abis_mo *gsm_objclass2mo(struct gsm_bts *bts, uint8_t obj_class,
				    const struct abis_om_obj_inst *obj_inst,
				    enum abis_nm_nack_cause *c)
{
	struct gsm_bts_trx *trx;
	struct gsm_abis_mo *mo = NULL;

	switch (obj_class) {
	case NM_OC_BTS:
		mo = &bts->mo;
		break;
	case NM_OC_RADIO_CARRIER:
		if (!(trx = gsm_bts_trx_num(bts, obj_inst->trx_nr)))
			goto nm_nack_trxnr_unkn;
		mo = &trx->mo;
		break;
	case NM_OC_BASEB_TRANSC:
		if (!(trx = gsm_bts_trx_num(bts, obj_inst->trx_nr)))
			goto nm_nack_trxnr_unkn;
		mo = &trx->bb_transc.mo;
		break;
	case NM_OC_CHANNEL:
		if (!(trx = gsm_bts_trx_num(bts, obj_inst->trx_nr)))
			goto nm_nack_trxnr_unkn;
		if (obj_inst->ts_nr >= TRX_NR_TS)
			goto nm_nack_objinst_unkn;
		mo = &trx->ts[obj_inst->ts_nr].mo;
		break;
	case NM_OC_SITE_MANAGER:
		mo = &g_bts_sm->mo;
		break;
	case NM_OC_GPRS_NSE:
		if (obj_inst->bts_nr > 0)
			goto nm_nack_objinst_unkn;
		mo = &g_bts_sm->gprs.nse.mo;
		break;
	case NM_OC_GPRS_CELL:
		mo = &bts->gprs.cell.mo;
		break;
	case NM_OC_GPRS_NSVC:
		if (obj_inst->bts_nr > 0)
			goto nm_nack_objinst_unkn;
		if (obj_inst->trx_nr >= ARRAY_SIZE(g_bts_sm->gprs.nse.nsvc))
			goto nm_nack_objinst_unkn;
		mo = &g_bts_sm->gprs.nse.nsvc[obj_inst->trx_nr].mo;
		break;
	}
	return mo;

nm_nack_trxnr_unkn:
	if (c != NULL)
		*c = NM_NACK_TRXNR_UNKN;
	return NULL;
nm_nack_objinst_unkn:
	if (c != NULL)
		*c = NM_NACK_OBJINST_UNKN;
	return NULL;
}

/* Obtain the in-memory data structure of a given object instance
 *  \param[out] c nack cause for reply in case of error. Ignored if NULL */
void *gsm_objclass2obj(struct gsm_bts *bts, uint8_t obj_class,
		       const struct abis_om_obj_inst *obj_inst,
		       enum abis_nm_nack_cause *c)
{
	struct gsm_bts_trx *trx;
	void *obj = NULL;

	switch (obj_class) {
	case NM_OC_BTS:
		obj = bts;
		break;
	case NM_OC_RADIO_CARRIER:
		if (!(trx = gsm_bts_trx_num(bts, obj_inst->trx_nr)))
			goto nm_nack_trxnr_unkn;
		obj = trx;
		break;
	case NM_OC_BASEB_TRANSC:
		if (!(trx = gsm_bts_trx_num(bts, obj_inst->trx_nr)))
			goto nm_nack_trxnr_unkn;
		obj = &trx->bb_transc;
		break;
	case NM_OC_CHANNEL:
		if (!(trx = gsm_bts_trx_num(bts, obj_inst->trx_nr)))
			goto nm_nack_trxnr_unkn;
		if (obj_inst->ts_nr >= TRX_NR_TS)
			goto nm_nack_objinst_unkn;
		obj = &trx->ts[obj_inst->ts_nr];
		break;
	case NM_OC_SITE_MANAGER:
		obj = g_bts_sm;
		break;
	case NM_OC_GPRS_NSE:
		if (obj_inst->bts_nr > 0)
			goto nm_nack_objinst_unkn;
		obj = &g_bts_sm->gprs.nse;
		break;
	case NM_OC_GPRS_CELL:
		obj = &bts->gprs.cell;
		break;
	case NM_OC_GPRS_NSVC:
		if (obj_inst->bts_nr > 0)
			goto nm_nack_objinst_unkn;
		if (obj_inst->trx_nr >= ARRAY_SIZE(g_bts_sm->gprs.nse.nsvc))
			goto nm_nack_objinst_unkn;
		obj = &g_bts_sm->gprs.nse.nsvc[obj_inst->trx_nr];
		break;
	}
	return obj;

nm_nack_trxnr_unkn:
	if (c != NULL)
		*c = NM_NACK_TRXNR_UNKN;
	return NULL;
nm_nack_objinst_unkn:
	if (c != NULL)
		*c = NM_NACK_OBJINST_UNKN;
	return NULL;
}
