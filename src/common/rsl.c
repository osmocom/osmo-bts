/* GSM TS 08.58 RSL, BTS Side */

/* (C) 2011 by Andreas Eversberg <jolly@eversberg.eu>
 * (C) 2011-2019 by Harald Welte <laforge@gnumonks.org>
 * (C) 2020 by sysmocom - s.f.m.c. GmbH <info@sysmocom.de>
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
#include <stdbool.h>
#include <inttypes.h>
#include <sys/types.h>
#include <arpa/inet.h>

#include <osmocom/core/byteswap.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/talloc.h>
#include <osmocom/gsm/rsl.h>
#include <osmocom/gsm/lapdm.h>
#include <osmocom/gsm/gsm0808.h>
#include <osmocom/gsm/protocol/gsm_12_21.h>
#include <osmocom/gsm/protocol/gsm_08_58.h>
#include <osmocom/gsm/protocol/gsm_04_08.h>
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
#include <osmo-bts/measurement.h>
#include <osmo-bts/pcu_if.h>
#include <osmo-bts/handover.h>
#include <osmo-bts/cbch.h>
#include <osmo-bts/l1sap.h>
#include <osmo-bts/bts_model.h>
#include <osmo-bts/pcuif_proto.h>
#include <osmo-bts/notification.h>
#include <osmo-bts/asci.h>

//#define FAKE_CIPH_MODE_COMPL

/* Parse power attenuation (in dB) from BS Power IE (see 9.3.4) */
#define BS_POWER2DB(bs_power) \
	((bs_power & 0x0f) * 2)

bool rsl_chan_rt_is_asci(enum rsl_cmod_crt chan_rt)
{
	switch (chan_rt) {
	case RSL_CMOD_CRT_TCH_GROUP_Bm:
	case RSL_CMOD_CRT_TCH_GROUP_Lm:
	case RSL_CMOD_CRT_TCH_BCAST_Bm:
	case RSL_CMOD_CRT_TCH_BCAST_Lm:
		return true;
	default:
		return false;
	}
}

bool rsl_chan_rt_is_vgcs(enum rsl_cmod_crt chan_rt)
{
	switch (chan_rt) {
	case RSL_CMOD_CRT_TCH_GROUP_Bm:
	case RSL_CMOD_CRT_TCH_GROUP_Lm:
		return true;
	default:
		return false;
	}
}


static int rsl_tx_error_report(struct gsm_bts_trx *trx, uint8_t cause, const uint8_t *chan_nr,
				const uint8_t *link_id, const struct msgb *orig_msg);

/* list of RSL SI types that can occur on the SACCH */
static const unsigned int rsl_sacch_sitypes[] = {
	RSL_SYSTEM_INFO_5,
	RSL_SYSTEM_INFO_6,
	RSL_SYSTEM_INFO_5bis,
	RSL_SYSTEM_INFO_5ter,
	RSL_SYSTEM_INFO_10,
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

/* Handle RSL Channel Mode IE (see section 9.3.6) */
static int rsl_handle_chan_mod_ie(struct gsm_lchan *lchan,
				  const struct tlv_parsed *tp,
				  uint8_t *cause)
{
	const struct rsl_ie_chan_mode *cm;

	if (!TLVP_PRES_LEN(tp, RSL_IE_CHAN_MODE, sizeof(*cm))) {
		LOGPLCHAN(lchan, DRSL, LOGL_ERROR, "Channel Mode IE is not present\n");
		*cause = RSL_ERR_MAND_IE_ERROR;
		return -ENODEV;
	}

	cm = (const struct rsl_ie_chan_mode *) TLVP_VAL(tp, RSL_IE_CHAN_MODE);
	lchan->rsl_cmode = cm->spd_ind;
	lchan->rsl_chan_rt = cm->chan_rt;
	lchan->ts->trx->bts->dtxd = (cm->dtx_dtu & RSL_CMOD_DTXd) ? true : false;

	/* Octet 5: Channel rate and type */
	switch (cm->chan_rt) {
	case RSL_CMOD_CRT_SDCCH:
	case RSL_CMOD_CRT_TCH_Bm:
	case RSL_CMOD_CRT_TCH_Lm:
	case RSL_CMOD_CRT_TCH_GROUP_Bm:
	case RSL_CMOD_CRT_TCH_GROUP_Lm:
	case RSL_CMOD_CRT_TCH_BCAST_Bm:
	case RSL_CMOD_CRT_TCH_BCAST_Lm:
		break;
	case RSL_CMOD_CRT_OSMO_TCH_VAMOS_Bm:
	case RSL_CMOD_CRT_OSMO_TCH_VAMOS_Lm:
		/* Make sure that Osmocom specific TSC IE is present */
		if (!TLVP_PRES_LEN(tp, RSL_IE_OSMO_TRAINING_SEQUENCE, 2)) {
			LOGPLCHAN(lchan, DRSL, LOGL_ERROR,
				  "Training Sequence IE is not present\n");
			*cause = RSL_ERR_MAND_IE_ERROR;
			return -ENODEV;
		}
		break;
	default:
		LOGPLCHAN(lchan, DRSL, LOGL_ERROR, "Channel Mode IE contains "
			  "unknown 'Channel rate and type' value 0x%02x\n",
			  cm->chan_rt);
		*cause = RSL_ERR_IE_CONTENT;
		return -ENOTSUP;
	}

#define RSL_CMODE(spd_ind, chan_rate) \
	((spd_ind << 8) | chan_rate)

	/* Octet 6: Speech coding algorithm/data rate + transparency indicator.
	 * NOTE: coding of this octet depends on 'Speech or data indicator' */
	switch (RSL_CMODE(cm->spd_ind, cm->chan_rate)) {
	/* If octet 4 indicates signalling */
	case RSL_CMODE(RSL_CMOD_SPD_SIGN, 0x00):
		/* No resources required, all other values are reserved */
		lchan->tch_mode = GSM48_CMODE_SIGN;
		break;

	/* If octet 4 indicates speech */
	case RSL_CMODE(RSL_CMOD_SPD_SPEECH, RSL_CMOD_SP_GSM1):
		lchan->tch_mode = GSM48_CMODE_SPEECH_V1;
		break;
	case RSL_CMODE(RSL_CMOD_SPD_SPEECH, RSL_CMOD_SP_GSM2):
		lchan->tch_mode = GSM48_CMODE_SPEECH_EFR;
		break;
	case RSL_CMODE(RSL_CMOD_SPD_SPEECH, RSL_CMOD_SP_GSM3):
		lchan->tch_mode = GSM48_CMODE_SPEECH_AMR;
		break;
	case RSL_CMODE(RSL_CMOD_SPD_SPEECH, RSL_CMOD_SP_GSM4):
		lchan->tch_mode = GSM48_CMODE_SPEECH_V4;
		break;
	case RSL_CMODE(RSL_CMOD_SPD_SPEECH, RSL_CMOD_SP_GSM5):
		lchan->tch_mode = GSM48_CMODE_SPEECH_V5;
		break;
	case RSL_CMODE(RSL_CMOD_SPD_SPEECH, RSL_CMOD_SP_GSM6):
		lchan->tch_mode = GSM48_CMODE_SPEECH_V6;
		break;

	/* If octet 4 indicates non-transparent data */
	case RSL_CMODE(RSL_CMOD_SPD_DATA, RSL_CMOD_CSD_NT_14k5):
		lchan->tch_mode = GSM48_CMODE_DATA_14k5;
		break;
	case RSL_CMODE(RSL_CMOD_SPD_DATA, RSL_CMOD_CSD_NT_12k0):
		lchan->tch_mode = GSM48_CMODE_DATA_12k0;
		break;
	case RSL_CMODE(RSL_CMOD_SPD_DATA, RSL_CMOD_CSD_NT_6k0):
		lchan->tch_mode = GSM48_CMODE_DATA_6k0;
		break;
	case RSL_CMODE(RSL_CMOD_SPD_DATA, RSL_CMOD_CSD_NT_43k5):
		lchan->tch_mode = GSM48_CMODE_DATA_43k5;
		break;
	case RSL_CMODE(RSL_CMOD_SPD_DATA, RSL_CMOD_CSD_NT_28k8):
		/* 28.8 kbit/s services, 29.0 kbit/s radio interface rate */
		lchan->tch_mode = GSM48_CMODE_DATA_29k0;
		break;
	case RSL_CMODE(RSL_CMOD_SPD_DATA, RSL_CMOD_CSD_NTA_43k5_14k5):
		lchan->tch_mode = GSM48_CMODE_DATA_43k5_14k5;
		break;
	case RSL_CMODE(RSL_CMOD_SPD_DATA, RSL_CMOD_CSD_NTA_29k0_14k5):
		lchan->tch_mode = GSM48_CMODE_DATA_29k0_14k5;
		break;
	case RSL_CMODE(RSL_CMOD_SPD_DATA, RSL_CMOD_CSD_NTA_43k5_29k0):
		lchan->tch_mode = GSM48_CMODE_DATA_43k5_29k0;
		break;
	case RSL_CMODE(RSL_CMOD_SPD_DATA, RSL_CMOD_CSD_NTA_14k5_43k5):
		lchan->tch_mode = GSM48_CMODE_DATA_14k5_43k5;
		break;
	case RSL_CMODE(RSL_CMOD_SPD_DATA, RSL_CMOD_CSD_NTA_14k5_29k0):
		lchan->tch_mode = GSM48_CMODE_DATA_14k5_29k0;
		break;
	case RSL_CMODE(RSL_CMOD_SPD_DATA, RSL_CMOD_CSD_NTA_29k0_43k5):
		lchan->tch_mode = GSM48_CMODE_DATA_29k0_43k5;
		break;

	/* If octet 4 indicates transparent data */
	case RSL_CMODE(RSL_CMOD_SPD_DATA, RSL_CMOD_CSD_T_32000):
		/* 32.0 kbit/s services, 32.0 kbit/s radio interface rate */
		lchan->tch_mode = GSM48_CMODE_DATA_32k0;
		lchan->csd_mode = LCHAN_CSD_M_T_32000;
		break;
	case RSL_CMODE(RSL_CMOD_SPD_DATA, RSL_CMOD_CSD_T_29000):
		/* 29.0 kbit/s services, 29.0 kbit/s radio interface rate */
		lchan->tch_mode = GSM48_CMODE_DATA_29k0;
		lchan->csd_mode = LCHAN_CSD_M_T_29000;
		break;
	case RSL_CMODE(RSL_CMOD_SPD_DATA, RSL_CMOD_CSD_T_14400):
		/* 14.4 kbit/s services, 14.5 kbit/s radio interface rate */
		lchan->tch_mode = GSM48_CMODE_DATA_14k5;
		lchan->csd_mode = LCHAN_CSD_M_T_14400;
		break;
	case RSL_CMODE(RSL_CMOD_SPD_DATA, RSL_CMOD_CSD_T_9600):
		/* 9.6 kbit/s services, 12.0 kbit/s radio interface rate */
		lchan->tch_mode = GSM48_CMODE_DATA_12k0;
		lchan->csd_mode = LCHAN_CSD_M_T_9600;
		break;
	case RSL_CMODE(RSL_CMOD_SPD_DATA, RSL_CMOD_CSD_T_4800):
		/* 4.8 kbit/s services, 6.0 kbit/s radio interface rate */
		lchan->tch_mode = GSM48_CMODE_DATA_6k0;
		lchan->csd_mode = LCHAN_CSD_M_T_4800;
		break;
	case RSL_CMODE(RSL_CMOD_SPD_DATA, RSL_CMOD_CSD_T_2400):
		/* 2.4 kbit/s *and less* services, 3.6 kbit/s radio interface rate */
		lchan->tch_mode = GSM48_CMODE_DATA_3k6;
		lchan->csd_mode = LCHAN_CSD_M_T_2400;
		break;
	case RSL_CMODE(RSL_CMOD_SPD_DATA, RSL_CMOD_CSD_T_1200):
		/* 2.4 kbit/s *and less* services, 3.6 kbit/s radio interface rate */
		lchan->tch_mode = GSM48_CMODE_DATA_3k6;
		lchan->csd_mode = LCHAN_CSD_M_T_1200;
		break;
	case RSL_CMODE(RSL_CMOD_SPD_DATA, RSL_CMOD_CSD_T_600):
		/* 2.4 kbit/s *and less* services, 3.6 kbit/s radio interface rate */
		lchan->tch_mode = GSM48_CMODE_DATA_3k6;
		lchan->csd_mode = LCHAN_CSD_M_T_600;
		break;
	case RSL_CMODE(RSL_CMOD_SPD_DATA, RSL_CMOD_CSD_T_1200_75):
		/* 2.4 kbit/s *and less* services, 3.6 kbit/s radio interface rate */
		lchan->tch_mode = GSM48_CMODE_DATA_3k6;
		lchan->csd_mode = LCHAN_CSD_M_T_1200_75;
		break;

	default:
		LOGPLCHAN(lchan, DRSL, LOGL_ERROR, "Channel Mode IE contains "
			  "an unknown/unhandled combination of "
			  "'Speech or data indicator' 0x%02x and "
			  "'Speech coding algorithm/data rate' 0x%02x\n",
			  cm->spd_ind, cm->chan_rate);
		*cause = RSL_ERR_IE_CONTENT;
		return -ENOPROTOOPT;
	}

#undef RSL_CMODE

	if (!bts_supports_cm(lchan->ts->trx->bts, cm)) {
		LOGPLCHAN(lchan, DRSL, LOGL_ERROR, "Channel type=0x%02x/mode=%s "
			  "is not supported by the PHY\n", cm->chan_rt,
			  gsm48_chan_mode_name(lchan->tch_mode));
		*cause = RSL_ERR_SERV_OPT_UNAVAIL;
		return -ENOTSUP;
	}

	return 0;
}

/* Handle RSL Channel Identification IE (see section 9.3.5) */
static int rsl_handle_chan_ident_ie(struct gsm_lchan *lchan,
				    const struct tlv_parsed *tp,
				    uint8_t *cause)
{
	const struct gsm_bts_trx_ts *ts = lchan->ts;
	const struct gsm_bts *bts = ts->trx->bts;
	const struct gsm48_chan_desc *cd;

	if (TLVP_PRES_LEN(tp, RSL_IE_CHAN_IDENT, sizeof(*cd) + 1)) {
		/* Channel Description IE comes together with its IEI (see 9.3.5) */
		cd = (const struct gsm48_chan_desc *) (TLVP_VAL(tp, RSL_IE_CHAN_IDENT) + 1);

		/* The PHY may not support using different TSCs */
		if (!osmo_bts_has_feature(bts->features, BTS_FEAT_MULTI_TSC)
		    && cd->h0.tsc != BTS_TSC(bts)) {
			LOGPLCHAN(lchan, DL1C, LOGL_ERROR, "This PHY does not support "
				  "lchan TSC %u != BSIC-TSC %u, sending NACK\n",
				  cd->h0.tsc, BTS_TSC(bts));
			*cause = RSL_ERR_SERV_OPT_UNIMPL;
			return -ENOTSUP;
		}
	}

	return 0;
}

/* Handle Osmocom specific TSC IE */
static int rsl_handle_osmo_tsc_ie(struct gsm_lchan *lchan,
				  const struct tlv_parsed *tp,
				  uint8_t *cause)
{
	/* Osmocom specific IE indicating Training Sequence Code and Set */
	if (TLVP_PRES_LEN(tp, RSL_IE_OSMO_TRAINING_SEQUENCE, 2)) {
		const uint8_t *ie = TLVP_VAL(tp, RSL_IE_OSMO_TRAINING_SEQUENCE);
		lchan->ts->tsc_set = ie[0] & 0x03; /* Range: 0..3 */
		lchan->ts->tsc_rsl = ie[1] & 0x07; /* Range: 0..7 */
		lchan->ts->tsc_rsl_configured = true;
	} else {
		lchan->ts->tsc_rsl_configured = false;
		lchan->ts->tsc_rsl = 0xff;
		lchan->ts->tsc_set = 0;
	}
	gsm_ts_apply_configured_tsc(lchan->ts);

	return 0;
}


/*
 * support
 */

/* Is this channel number for a dedicated channel (true) or not (false) */
static bool chan_nr_is_dchan(uint8_t chan_nr)
{
	/* See TS 48.058 9.3.1 + Osmocom extension for RSL_CHAN_OSMO_PDCH */
	if ((chan_nr & 0xc0) == 0x80)
		return false;
	else
		return true;
}

static struct gsm_lchan *lchan_lookup(struct gsm_bts_trx *trx, uint8_t chan_nr,
				      const char *log_name)
{
	int rc;
	struct gsm_lchan *lchan = rsl_lchan_lookup(trx, chan_nr, &rc);

	if (!lchan) {
		LOGP(DRSL, LOGL_ERROR, "%sunknown chan_nr=0x%02x\n", log_name,
		     chan_nr);
		return NULL;
	}

	if (rc < 0) {
		LOGP(DRSL, LOGL_ERROR, "%s %smismatching chan_nr=0x%02x\n",
		     gsm_ts_and_pchan_name(lchan->ts), log_name, chan_nr);
		return NULL;
	}
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
static int rsl_tx_error_report(struct gsm_bts_trx *trx, uint8_t cause, const uint8_t *chan_nr,
				const uint8_t *link_id, const struct msgb *orig_msg)
{
	unsigned int len = sizeof(struct abis_rsl_common_hdr);
	struct msgb *nmsg;

	LOGP(DRSL, LOGL_NOTICE, "Tx RSL Error Report: cause = 0x%02x\n", cause);

	if (orig_msg)
		len += 2 + 3+msgb_l2len(orig_msg); /* chan_nr + TLV(orig_msg) */
	if (chan_nr)
		len += 2;
	if (link_id)
		len += 2;

	nmsg = rsl_msgb_alloc(len);
	if (!nmsg)
		return -ENOMEM;
	msgb_tlv_put(nmsg, RSL_IE_CAUSE, 1, &cause);
	if (orig_msg && msgb_l2len(orig_msg) >= sizeof(struct abis_rsl_common_hdr)) {
		struct abis_rsl_common_hdr *ch = (struct abis_rsl_common_hdr *) msgb_l2(orig_msg);
		msgb_tv_put(nmsg, RSL_IE_MSG_ID, ch->msg_type);
	}
	if (chan_nr)
		msgb_tv_put(nmsg, RSL_IE_CHAN_NR, *chan_nr);
	if (link_id)
		msgb_tv_put(nmsg, RSL_IE_LINK_IDENT, *link_id);
	if (orig_msg)
		msgb_tlv_put(nmsg, RSL_IE_ERR_MSG, msgb_l2len(orig_msg), msgb_l2(orig_msg));

	rsl_trx_push_hdr(nmsg, RSL_MT_ERROR_REPORT);
	nmsg->trx = trx;

	return abis_bts_rsl_sendmsg(nmsg);
}

/* 8.6.1 sending RF RESOURCE INDICATION */
int rsl_tx_rf_res(struct gsm_bts_trx *trx)
{
	unsigned int tn, ln;
	struct msgb *nmsg;

	LOGP(DRSL, LOGL_INFO, "Tx RSL RF RESource INDication\n");

	nmsg = rsl_msgb_alloc(sizeof(struct abis_rsl_common_hdr));
	if (!nmsg)
		return -ENOMEM;

	/* Add interference levels for each logical channel */
	uint8_t *len = msgb_tl_put(nmsg, RSL_IE_RESOURCE_INFO);

	for (tn = 0; tn < ARRAY_SIZE(trx->ts); tn++) {
		const struct gsm_bts_trx_ts *ts = &trx->ts[tn];

		if (ts->mo.nm_state.operational != NM_OPSTATE_ENABLED)
			continue;
		if (ts->mo.nm_state.availability != NM_AVSTATE_OK)
			continue;

		for (ln = 0; ln < ARRAY_SIZE(ts->lchan); ln++) {
			const struct gsm_lchan *lchan = &ts->lchan[ln];

			/* No average interference value => no band */
			if (lchan->meas.interf_meas_avg_dbm == 0)
				continue;

			/* Only for GSM_LCHAN_{SDCCH,TCH_F,TCH_H,PDTCH} */
			switch (lchan->type) {
			case GSM_LCHAN_SDCCH:
			case GSM_LCHAN_TCH_F:
			case GSM_LCHAN_TCH_H:
				/* We're not interested in active CS lchans */
				if (lchan->state == LCHAN_S_ACTIVE)
					continue;
				break;
			case GSM_LCHAN_PDTCH:
				break;
			default:
				continue;
			}

			msgb_v_put(nmsg, gsm_lchan2chan_nr_rsl(lchan));
			msgb_v_put(nmsg, (lchan->meas.interf_band & 0x07) << 5);
		}
	}

	/* Calculate length of the V part */
	*len = msgb_l3len(nmsg) - 2;

	rsl_trx_push_hdr(nmsg, RSL_MT_RF_RES_IND);
	nmsg->trx = trx;

	return abis_bts_rsl_sendmsg(nmsg);
}

/*
 * common channel related messages
 */

/* 8.5.1 BCCH INFOrmation is received */
static int rsl_rx_bcch_info(struct gsm_bts_trx *trx, struct msgb *msg)
{
	struct abis_rsl_cchan_hdr *cch = msgb_l2(msg);
	struct gsm_bts *bts = trx->bts;
	struct tlv_parsed tp;
	uint8_t rsl_si, count;
	enum osmo_sysinfo_type osmo_si;
	struct gsm48_system_information_type_2quater *si2q;
	struct bitvec bv;
	const uint8_t *si_buf;

	if (rsl_tlv_parse(&tp, msgb_l3(msg), msgb_l3len(msg)) < 0) {
		LOGPTRX(trx, DRSL, LOGL_ERROR, "%s(): rsl_tlv_parse() failed\n", __func__);
		return rsl_tx_error_report(trx, RSL_ERR_PROTO, &cch->chan_nr, NULL, msg);
	}

	/* 9.3.30 System Info Type */
	if (!TLVP_PRESENT(&tp, RSL_IE_SYSINFO_TYPE))
		return rsl_tx_error_report(trx, RSL_ERR_MAND_IE_ERROR, &cch->chan_nr, NULL, msg);

	rsl_si = *TLVP_VAL(&tp, RSL_IE_SYSINFO_TYPE);
	if (OSMO_IN_ARRAY(rsl_si, rsl_sacch_sitypes))
		return rsl_tx_error_report(trx, RSL_ERR_IE_CONTENT, &cch->chan_nr, NULL, msg);

	osmo_si = osmo_rsl2sitype(rsl_si);
	if (osmo_si == SYSINFO_TYPE_NONE) {
		LOGP(DRSL, LOGL_NOTICE, " Rx RSL SI 0x%02x not supported.\n", rsl_si);
		return rsl_tx_error_report(trx, RSL_ERR_IE_CONTENT, &cch->chan_nr, NULL, msg);
	}
	/* 9.3.39 Full BCCH Information */
	if (TLVP_PRESENT(&tp, RSL_IE_FULL_BCCH_INFO)) {
		uint8_t len = TLVP_LEN(&tp, RSL_IE_FULL_BCCH_INFO);
		if (len > sizeof(sysinfo_buf_t)) {
			LOGP(DRSL, LOGL_ERROR, "Truncating received Full BCCH Info (%u -> %zu) for SI%s\n",
			     len, sizeof(sysinfo_buf_t), get_value_string(osmo_sitype_strs, osmo_si));
			len = sizeof(sysinfo_buf_t);
		}

		LOGP(DRSL, LOGL_INFO, " Rx RSL BCCH INFO (SI%s, %u bytes)\n",
		     get_value_string(osmo_sitype_strs, osmo_si), len);

		if (SYSINFO_TYPE_2quater == osmo_si) {
			si2q = (struct gsm48_system_information_type_2quater *) TLVP_VAL(&tp, RSL_IE_FULL_BCCH_INFO);
			bv.data = si2q->rest_octets;
			bv.data_len = GSM_MACBLOCK_LEN;
			bv.cur_bit = 3;
			bts->si2q_index = (uint8_t) bitvec_get_uint(&bv, 4);

			count = (uint8_t) bitvec_get_uint(&bv, 4);
			if (bts->si2q_count && bts->si2q_count != count) {
				LOGP(DRSL, LOGL_NOTICE, " Rx RSL SI2quater count updated: %u -> %d\n",
				     bts->si2q_count, count);
			}

			bts->si2q_count = count;
			if (bts->si2q_index > bts->si2q_count) {
				LOGP(DRSL, LOGL_ERROR, " Rx RSL SI2quater with index %u > count %u\n",
				     bts->si2q_index, bts->si2q_count);
				return rsl_tx_error_report(trx, RSL_ERR_IE_CONTENT, &cch->chan_nr, NULL, msg);
			}

			if (bts->si2q_index > SI2Q_MAX_NUM || bts->si2q_count > SI2Q_MAX_NUM) {
				LOGP(DRSL, LOGL_ERROR, " Rx RSL SI2quater with impossible parameters: index %u, count %u"
				     "should be <= %u\n", bts->si2q_index, bts->si2q_count, SI2Q_MAX_NUM);
				return rsl_tx_error_report(trx, RSL_ERR_IE_CONTENT, &cch->chan_nr, NULL, msg);
			}

			memset(GSM_BTS_SI2Q(bts, bts->si2q_index), GSM_MACBLOCK_PADDING, sizeof(sysinfo_buf_t));
			memcpy(GSM_BTS_SI2Q(bts, bts->si2q_index), TLVP_VAL(&tp, RSL_IE_FULL_BCCH_INFO), len);
		} else {
			memset(bts->si_buf[osmo_si], GSM_MACBLOCK_PADDING, sizeof(sysinfo_buf_t));
			memcpy(bts->si_buf[osmo_si], TLVP_VAL(&tp, RSL_IE_FULL_BCCH_INFO), len);
		}

		bts->si_valid |= (1 << osmo_si);

		switch (osmo_si) {
		case SYSINFO_TYPE_3:
			if (trx->nr == 0 && num_agch(trx, "RSL") != 1) {
				lchan_deactivate(&trx->bts->c0->ts[0].lchan[CCCH_LCHAN]);
				/* will be reactivated by sapi_deactivate_cb() */
				trx->bts->c0->ts[0].lchan[CCCH_LCHAN].rel_act_kind =
					LCHAN_REL_ACT_REACT;
			}
			/* decode original SI3 Rest Octets as sent by BSC */
			si_buf = (const uint8_t *) GSM_BTS_SI(bts, osmo_si);
			si_buf += offsetof(struct gsm48_system_information_type_3, rest_octets);
			osmo_gsm48_rest_octets_si3_decode(&bts->si3_ro_decoded, si_buf);
			/* patch out GPRS indicator from binary if PCU is not connected; will be enabled
			 * after PCU connects */
			regenerate_si3_restoctets(bts);
			pcu_tx_si(trx->bts, SYSINFO_TYPE_3, true);
			break;
		case SYSINFO_TYPE_4:
			/* decode original SI4 Rest Octets as sent by BSC */
			si_buf = (const uint8_t *) GSM_BTS_SI(bts, osmo_si);
			int si4_ro_offset = get_si4_ro_offset(si_buf);
			if (si4_ro_offset > 0) {
				osmo_gsm48_rest_octets_si4_decode(&bts->si4_ro_decoded,
								  si_buf + si4_ro_offset,
								  GSM_MACBLOCK_LEN - si4_ro_offset);
				/* patch out GPRS indicator from binary if PCU is not connected; will be
				 * enabled after PCU connects */
				regenerate_si4_restoctets(bts);
			}
			break;
		case SYSINFO_TYPE_1:
			/* Get the position of the NCH, if enabled. */
			trx->bts->asci.pos_nch = pos_nch(trx, "BCCH INFO");
			pcu_tx_si(trx->bts, SYSINFO_TYPE_1, true);
			break;
		case SYSINFO_TYPE_2:
		case SYSINFO_TYPE_13:
			pcu_tx_si(trx->bts, osmo_si, true);
			break;
		default:
			break;
		}

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
		switch (osmo_si) {
		case SYSINFO_TYPE_13:
			pcu_tx_si(trx->bts, SYSINFO_TYPE_13, false);
			break;
		case SYSINFO_TYPE_3:
			memset(&bts->si3_ro_decoded, 0, sizeof(bts->si3_ro_decoded));
			pcu_tx_si(trx->bts, SYSINFO_TYPE_3, false);
			break;
		case SYSINFO_TYPE_1:
			pcu_tx_si(trx->bts, SYSINFO_TYPE_1, false);
			break;
		default:
			break;
		}
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

/* 8.5.4 DELETE INDICATION */
int rsl_tx_delete_ind(struct gsm_bts *bts, const uint8_t *ia, uint8_t ia_len)
{
	struct msgb *msg;

	msg = rsl_msgb_alloc(sizeof(struct abis_rsl_cchan_hdr));
	if (!msg)
		return -ENOMEM;
	rsl_cch_push_hdr(msg, RSL_MT_DELETE_IND, RSL_CHAN_PCH_AGCH);
	msgb_tlv_put(msg, RSL_IE_FULL_IMM_ASS_INFO, ia_len, ia);
	msg->trx = bts->c0;

	return abis_bts_rsl_sendmsg(msg);
}

/* 8.5.5 PAGING COMMAND */
static int rsl_rx_paging_cmd(struct gsm_bts_trx *trx, struct msgb *msg)
{
	struct abis_rsl_cchan_hdr *cch = msgb_l2(msg);
	struct tlv_parsed tp;
	struct gsm_bts *bts = trx->bts;
	uint8_t chan_needed = 0, paging_group;
	const uint8_t *identity_lv;
	int rc;

	if (rsl_tlv_parse(&tp, msgb_l3(msg), msgb_l3len(msg)) < 0) {
		LOGPTRX(trx, DRSL, LOGL_ERROR, "%s(): rsl_tlv_parse() failed\n", __func__);
		return rsl_tx_error_report(trx, RSL_ERR_PROTO, &cch->chan_nr, NULL, msg);
	}

	if (!TLVP_PRESENT(&tp, RSL_IE_PAGING_GROUP) ||
	    !TLVP_PRESENT(&tp, RSL_IE_MS_IDENTITY))
		return rsl_tx_error_report(trx, RSL_ERR_MAND_IE_ERROR, &cch->chan_nr, NULL, msg);

	paging_group = *TLVP_VAL(&tp, RSL_IE_PAGING_GROUP);
	identity_lv = TLVP_VAL(&tp, RSL_IE_MS_IDENTITY)-1;

	if (TLVP_PRES_LEN(&tp, RSL_IE_CHAN_NEEDED, 1))
		chan_needed = *TLVP_VAL(&tp, RSL_IE_CHAN_NEEDED);

	rc = paging_add_identity(bts->paging_state, paging_group, identity_lv, chan_needed);
	if (rc < 0) {
		/* FIXME: notfiy the BSC on other errors? */
		if (rc == -ENOSPC) {
			oml_tx_failure_event_rep(&trx->bts->mo, NM_SEVER_WARNING,
						 OSMO_EVT_MIN_PAG_TAB_FULL, "BTS paging table is full");
		}
	}

	pcu_tx_pag_req(identity_lv, chan_needed);

	return 0;
}

/* 8.5.8 SMS BROADCAST COMMAND */
static int rsl_rx_sms_bcast_cmd(struct gsm_bts_trx *trx, struct msgb *msg)
{
	struct abis_rsl_cchan_hdr *cch = msgb_l2(msg);
	struct tlv_parsed tp;
	struct rsl_ie_cb_cmd_type *cb_cmd_type;
	bool extended_cbch = false;
	int rc;

	if (rsl_tlv_parse(&tp, msgb_l3(msg), msgb_l3len(msg)) < 0) {
		LOGPTRX(trx, DRSL, LOGL_ERROR, "%s(): rsl_tlv_parse() failed\n", __func__);
		return rsl_tx_error_report(trx, RSL_ERR_PROTO, &cch->chan_nr, NULL, msg);
	}

	if (!TLVP_PRESENT(&tp, RSL_IE_CB_CMD_TYPE) ||
	    !TLVP_PRESENT(&tp, RSL_IE_SMSCB_MSG))
		return rsl_tx_error_report(trx, RSL_ERR_MAND_IE_ERROR, &cch->chan_nr, NULL, msg);

	if (TLVP_PRESENT(&tp, RSL_IE_SMSCB_CHAN_INDICATOR)) {
		if ((*TLVP_VAL(&tp, RSL_IE_SMSCB_CHAN_INDICATOR) & 0x0f) == 0x01)
			extended_cbch = true;
	}

	cb_cmd_type = (struct rsl_ie_cb_cmd_type *)
					TLVP_VAL(&tp, RSL_IE_CB_CMD_TYPE);

	rc = bts_process_smscb_cmd(trx->bts, *cb_cmd_type, extended_cbch,
				   TLVP_LEN(&tp, RSL_IE_SMSCB_MSG), TLVP_VAL(&tp, RSL_IE_SMSCB_MSG));
	if (rc < 0)
		return rsl_tx_error_report(trx, RSL_ERR_IE_CONTENT, &cch->chan_nr, NULL, msg);

	return 0;
}

/* Broadcast notification about new VGCS/VBS call on every dedicated channel.
 * This is required for MSs that are currently in dedicated mode that there is an ongoing call and on which channel
 * the call is active. Most MSs in dedicated mode may not be able to receive the NCH otherwise.
 * MSs that do not support ASCI will ignore it, as it is an unsupported message for them.
 */
static int asci_broadcast_facch(struct gsm_bts *bts, const uint8_t *group_call_ref, const uint8_t *chan_desc,
				uint8_t chan_desc_len, unsigned int count)
{
	uint8_t notif[23];
	struct msgb *msg, *cmsg;
	struct gsm_bts_trx *trx;
	struct gsm_lchan *lchan;
	unsigned int tn, ln, n;
	int rc;

	rc = bts_asci_notify_facch_gen_msg(bts, notif, group_call_ref, chan_desc, chan_desc_len);
	if (rc < 0)
		return rc;

	llist_for_each_entry(trx, &bts->trx_list, list) {
		for (tn = 0; tn < ARRAY_SIZE(trx->ts); tn++) {
			for (ln = 0; ln < ARRAY_SIZE(trx->ts[tn].lchan); ln++) {
				lchan = &trx->ts[tn].lchan[ln];
				if (!lchan_is_dcch(lchan))
					continue;
				if (lchan->state != LCHAN_S_ACTIVE)
					continue;
				msg = rsl_rll_simple(RSL_MT_UNIT_DATA_REQ, gsm_lchan2chan_nr(lchan), 0x00, 0);
				msg->l3h = msg->tail; /* emulate rsl_rx_rll() behaviour */
				msgb_tl16v_put(msg, RSL_IE_L3_INFO, sizeof(notif), (uint8_t *) &notif);
				for (n = 1; n < count; n++) {
					cmsg = msgb_copy(msg, "FACCH copy");
					lapdm_rslms_recvmsg(cmsg, &lchan->lapdm_ch);
				}
				lapdm_rslms_recvmsg(msg, &lchan->lapdm_ch);
			}
		}
	}

	return 0;
}

/* Number of times to broadcast ASCI call on every dedicated channel. */
#define ASCI_BROADCAST_NUM 3

/* 8.5.10 NOTIFICATION COMMAND */
static int rsl_rx_notification_cmd(struct gsm_bts_trx *trx, struct msgb *msg)
{
	struct abis_rsl_cchan_hdr *cch = msgb_l2(msg);
	struct tlv_parsed tp;
	uint8_t command_indicator;
	int rc;

	if (rsl_tlv_parse(&tp, msgb_l3(msg), msgb_l3len(msg)) < 0) {
		LOGPTRX(trx, DRSL, LOGL_ERROR, "%s(): rsl_tlv_parse() failed\n", __func__);
		return rsl_tx_error_report(trx, RSL_ERR_PROTO, &cch->chan_nr, NULL, msg);
	}

	if (cch->chan_nr != RSL_CHAN_PCH_AGCH) {
		LOGPTRX(trx, DRSL, LOGL_ERROR, "%s(): chan nr is not Downlink CCCH\n", __func__);
		return rsl_tx_error_report(trx, RSL_ERR_IE_CONTENT, &cch->chan_nr, NULL, msg);
	}

	if (!TLVP_PRES_LEN(&tp, RSL_IE_CMD_INDICATOR, 1))
		return rsl_tx_error_report(trx, RSL_ERR_MAND_IE_ERROR, &cch->chan_nr, NULL, msg);
	command_indicator = *TLVP_VAL(&tp, RSL_IE_CMD_INDICATOR);

	switch (command_indicator) {
	case RSL_CMD_INDICATOR_START:
		/* we need at least a Group Call Reference to start notification */
		if (!TLVP_PRES_LEN(&tp, RSL_IE_GROUP_CALL_REF, 5))
			return rsl_tx_error_report(trx, RSL_ERR_OPT_IE_ERROR, &cch->chan_nr, NULL, msg);
		rc = bts_asci_notification_add(trx->bts, TLVP_VAL(&tp, RSL_IE_GROUP_CALL_REF),
					       TLVP_VAL(&tp, RSL_IE_CHAN_DESC), TLVP_LEN(&tp, RSL_IE_CHAN_DESC),
					       (struct rsl_ie_nch_drx_info *) TLVP_VAL(&tp, RSL_IE_NCH_DRX_INFO));
		/* Broadcast to FACCH */
		asci_broadcast_facch(trx->bts, TLVP_VAL(&tp, RSL_IE_GROUP_CALL_REF), TLVP_VAL(&tp, RSL_IE_CHAN_DESC),
				     TLVP_LEN(&tp, RSL_IE_CHAN_DESC), ASCI_BROADCAST_NUM);
		break;
	case RSL_CMD_INDICATOR_STOP:
		if (!TLVP_PRES_LEN(&tp, RSL_IE_GROUP_CALL_REF, 5)) {
			/* interpret this as stopping of all notification */
			rc = bts_asci_notification_reset(trx->bts);
		} else {
			rc = bts_asci_notification_del(trx->bts, TLVP_VAL(&tp, RSL_IE_GROUP_CALL_REF));
		}
		break;
	default:
		return rsl_tx_error_report(trx, RSL_ERR_IE_CONTENT, &cch->chan_nr, NULL, msg);
	}

	return rc;
}

/* OSMO_ETWS_CMD - proprietary extension as TS 48.058 has no standardized way to do this :( */
static int rsl_rx_osmo_etws_cmd(struct gsm_bts_trx *trx, struct msgb *msg)
{
	struct abis_rsl_cchan_hdr *cch = msgb_l2(msg);
	struct gsm_bts *bts = trx->bts;
	struct tlv_parsed tp;

	if (rsl_tlv_parse(&tp, msgb_l3(msg), msgb_l3len(msg)) < 0) {
		LOGPTRX(trx, DRSL, LOGL_ERROR, "%s(): rsl_tlv_parse() failed\n", __func__);
		return rsl_tx_error_report(trx, RSL_ERR_PROTO, &cch->chan_nr, NULL, msg);
	}

	if (!TLVP_PRESENT(&tp, RSL_IE_SMSCB_MSG))
		return rsl_tx_error_report(trx, RSL_ERR_MAND_IE_ERROR, &cch->chan_nr, NULL, msg);

	bts->etws.prim_notif_len = TLVP_LEN(&tp, RSL_IE_SMSCB_MSG);
	if (bts->etws.prim_notif_len == 0) {
		LOGP(DRSL, LOGL_NOTICE, "ETWS Primary Notification OFF\n");
		talloc_free(bts->etws.prim_notif);
		bts->etws.prim_notif = NULL;
		bts->etws.prim_notif_len = 0;
		bts->etws.page_size = 0;
		bts->etws.num_pages = 0;
		bts->etws.next_page = 0;
	} else {
		LOGP(DRSL, LOGL_NOTICE, "ETWS Primary Notification: %s\n",
		     osmo_hexdump(TLVP_VAL(&tp, RSL_IE_SMSCB_MSG),
				  TLVP_LEN(&tp, RSL_IE_SMSCB_MSG)));
		talloc_free(bts->etws.prim_notif);
		bts->etws.prim_notif = talloc_memdup(bts, TLVP_VAL(&tp, RSL_IE_SMSCB_MSG),
						     bts->etws.prim_notif_len);

		bts->etws.page_size = 14; /* maximum possible in SI1 Rest Octets */
		bts->etws.num_pages = bts->etws.prim_notif_len / bts->etws.page_size;
		if (bts->etws.prim_notif_len % bts->etws.page_size)
			bts->etws.num_pages++;

		/* toggle the PNI to allow phones to distinguish new from old primary notification */
		bts->etws.pni = !bts->etws.pni;

		/* forward the request to the PCU, so the PCU can send it over any active TBF
		 * to phones which currently don't listen to the paging channel */
		pcu_tx_app_info_req(bts, 0, TLVP_LEN(&tp, RSL_IE_SMSCB_MSG),
				    TLVP_VAL(&tp, RSL_IE_SMSCB_MSG));
	}
	return 0;
}

/*! Prefix a given SACCH frame with a L2/LAPDm UI header and store it in given output buffer.
 *  \param[out] buf Output buffer, must be caller-allocated and hold at least len + 2 or sizeof(sysinfo_buf_t) bytes
 *  \param[out] valid pointer to bit-mask of 'valid' System information types
 *  \param[in] current input data (L3 without L2/L1 header)
 *  \param[in] osmo_si Sytstem Information Type (SYSINFO_TYPE_*)
 *  \param[in] len length of \a current in octets */
static inline void lapdm_ui_prefix(uint8_t *buf, uint32_t *valid, const uint8_t *current, uint8_t osmo_si, uint16_t len)
{
	/* We have to pre-fix with the two-byte LAPDM UI header */
	if (len > sizeof(sysinfo_buf_t) - 2) {
		LOGP(DRSL, LOGL_ERROR, "Truncating received SI%s (%u -> %zu) to prepend LAPDM UI header (2 bytes)\n",
		     get_value_string(osmo_sitype_strs, osmo_si), len, sizeof(sysinfo_buf_t) - 2);
		len = sizeof(sysinfo_buf_t) - 2;
	}

	(*valid) |= (1 << osmo_si);
	buf[0] = 0x03;	/* C/R + EA */
	buf[1] = 0x03;	/* UI frame */

	memset(buf + 2, GSM_MACBLOCK_PADDING, sizeof(sysinfo_buf_t) - 2);
	memcpy(buf + 2, current, len);
}

/*! Prefix a given SACCH frame with a L2/LAPDm UI header and store it in given BTS SACCH buffer
 *  \param[out] bts BTS in whose System Information State we shall store
 *  \param[in] current input data (L3 without L2/L1 header)
 *  \param[in] osmo_si Sytstem Information Type (SYSINFO_TYPE_*)
 *  \param[in] len length of \a current in octets */
static inline void lapdm_ui_prefix_bts(struct gsm_bts *bts, const uint8_t *current, uint8_t osmo_si, uint16_t len)
{
	lapdm_ui_prefix(GSM_BTS_SI(bts, osmo_si), &bts->si_valid, current, osmo_si, len);
}

/*! Prefix a given SACCH frame with a L2/LAPDm UI header and store it in given lchan SACCH buffer
 *  \param[out] lchan Logical Channel in whose System Information State we shall store
 *  \param[in] current input data (L3 without L2/L1 header)
 *  \param[in] osmo_si Sytstem Information Type (SYSINFO_TYPE_*)
 *  \param[in] len length of \a current in octets */
static inline void lapdm_ui_prefix_lchan(struct gsm_lchan *lchan, const uint8_t *current, uint8_t osmo_si, uint16_t len)
{
	lapdm_ui_prefix(GSM_LCHAN_SI(lchan, osmo_si), &lchan->si.valid, current, osmo_si, len);
}

/* 8.6.2 SACCH FILLING */
static int rsl_rx_sacch_fill(struct gsm_bts_trx *trx, struct msgb *msg)
{
	struct gsm_bts *bts = trx->bts;
	struct tlv_parsed tp;
	uint8_t rsl_si;
	enum osmo_sysinfo_type osmo_si;

	if (rsl_tlv_parse(&tp, msgb_l3(msg), msgb_l3len(msg)) < 0) {
		LOGPTRX(trx, DRSL, LOGL_ERROR, "%s(): rsl_tlv_parse() failed\n", __func__);
		return rsl_tx_error_report(trx, RSL_ERR_PROTO, NULL, NULL, msg);
	}

	/* 9.3.30 System Info Type */
	if (!TLVP_PRESENT(&tp, RSL_IE_SYSINFO_TYPE))
		return rsl_tx_error_report(trx, RSL_ERR_MAND_IE_ERROR, NULL, NULL, msg);

	rsl_si = *TLVP_VAL(&tp, RSL_IE_SYSINFO_TYPE);
	if (!OSMO_IN_ARRAY(rsl_si, rsl_sacch_sitypes))
		return rsl_tx_error_report(trx, RSL_ERR_IE_CONTENT, NULL, NULL, msg);

	osmo_si = osmo_rsl2sitype(rsl_si);
	if (osmo_si == SYSINFO_TYPE_NONE) {
		LOGP(DRSL, LOGL_NOTICE, " Rx SACCH SI 0x%02x not supported.\n", rsl_si);
		return rsl_tx_error_report(trx, RSL_ERR_IE_CONTENT, NULL, NULL, msg);
	}
	if (TLVP_PRESENT(&tp, RSL_IE_L3_INFO)) {
		uint16_t len = TLVP_LEN(&tp, RSL_IE_L3_INFO);
		struct gsm_bts_trx *t;

		lapdm_ui_prefix_bts(bts, TLVP_VAL(&tp, RSL_IE_L3_INFO), osmo_si, len);

		/* Propagate SI change to all lchans which adhere to BTS-global default. */
		llist_for_each_entry(t, &bts->trx_list, list) {
			int i, j;
			for (i = 0; i < ARRAY_SIZE(t->ts); i++) {
				struct gsm_bts_trx_ts *ts = &t->ts[i];
				for (j = 0; j < ARRAY_SIZE(ts->lchan); j++) {
					struct gsm_lchan *lchan = &ts->lchan[j];
					if (lchan->state == LCHAN_S_NONE || (lchan->si.overridden & (1 << osmo_si)))
						continue;
					lapdm_ui_prefix_lchan(lchan, TLVP_VAL(&tp, RSL_IE_L3_INFO), osmo_si, len);
				}
			}
		}

		LOGP(DRSL, LOGL_INFO, " Rx RSL SACCH FILLING (SI%s, %u bytes)\n",
		     get_value_string(osmo_sitype_strs, osmo_si), len);
	} else {
		struct gsm_bts_trx *t;

		bts->si_valid &= ~(1 << osmo_si);

		/* Propagate SI change to all lchans which adhere to BTS-global default. */
		llist_for_each_entry(t, &bts->trx_list, list) {
			int i, j;
			for (i = 0; i < ARRAY_SIZE(t->ts); i++) {
				struct gsm_bts_trx_ts *ts = &t->ts[i];
				for (j = 0; j < ARRAY_SIZE(ts->lchan); j++) {
					struct gsm_lchan *lchan = &ts->lchan[j];
					if (lchan->state == LCHAN_S_NONE || (lchan->si.overridden & (1 << osmo_si)))
						continue;
					lchan->si.valid &= ~(1 << osmo_si);
				}
			}
		}
		LOGP(DRSL, LOGL_INFO, " Rx RSL Disabling SACCH FILLING (SI%s)\n",
			get_value_string(osmo_sitype_strs, osmo_si));
	}
	osmo_signal_dispatch(SS_GLOBAL, S_NEW_SYSINFO, bts);

	return 0;

}

/* Parser for ip.access specific MS/BS Power parameters */
static int parse_power_ctrl_params(struct gsm_power_ctrl_params *params,
				   const uint8_t *data, size_t data_len)
{
	const struct tlv_p_entry *ie;
	struct tlv_parsed tp[3];
	unsigned int i;
	int rc;

	/* There can be multiple RSL_IPAC_EIE_MEAS_AVG_CFG, so we use tlv_parse2() */
	rc = tlv_parse2(&tp[0], ARRAY_SIZE(tp), &rsl_ipac_eie_tlvdef,
			data, data_len, 0, 0);
	if (rc < 0)
		return rc;

	/* Either of RSL_IPAC_EIE_{BS,MS}_PWR_CTL must be present */
	if (TLVP_PRESENT(&tp[0], RSL_IPAC_EIE_BS_PWR_CTL) &&
	    TLVP_PRESENT(&tp[0], RSL_IPAC_EIE_MS_PWR_CTL))
		return -EINVAL;

	/* (TV) Thresholds: {L,U}_RXLEV_XX_P and {L,U}_RXQUAL_XX_P */
	if ((ie = TLVP_GET(&tp[0], RSL_IPAC_EIE_BS_PWR_CTL)) != NULL ||
	    (ie = TLVP_GET(&tp[0], RSL_IPAC_EIE_MS_PWR_CTL)) != NULL) {
		const struct ipac_preproc_pc_thresh *thresh;

		thresh = (const struct ipac_preproc_pc_thresh *) ie->val;

		params->rxlev_meas.lower_thresh = thresh->l_rxlev;
		params->rxlev_meas.upper_thresh = thresh->u_rxlev;

		params->rxqual_meas.lower_thresh = thresh->l_rxqual;
		params->rxqual_meas.upper_thresh = thresh->u_rxqual;
	}

	/* Osmocom extension, C/I related thresholds: */
	if (TLVP_PRES_LEN(&tp[0], RSL_IPAC_EIE_OSMO_MS_PWR_CTL, sizeof(struct osmo_preproc_pc_thresh))) {
		const struct osmo_preproc_pc_thresh *osmo_thresh;
		ie = TLVP_GET(&tp[0], RSL_IPAC_EIE_OSMO_MS_PWR_CTL);
		osmo_thresh = (const struct osmo_preproc_pc_thresh *) ie->val;
		params->ci_fr_meas.lower_thresh = osmo_thresh->l_ci_fr;
		params->ci_fr_meas.upper_thresh = osmo_thresh->u_ci_fr;

		params->ci_hr_meas.lower_thresh = osmo_thresh->l_ci_hr;
		params->ci_hr_meas.upper_thresh = osmo_thresh->u_ci_hr;

		params->ci_amr_fr_meas.lower_thresh = osmo_thresh->l_ci_amr_fr;
		params->ci_amr_fr_meas.upper_thresh = osmo_thresh->u_ci_amr_fr;

		params->ci_amr_hr_meas.lower_thresh = osmo_thresh->l_ci_amr_hr;
		params->ci_amr_hr_meas.upper_thresh = osmo_thresh->u_ci_amr_hr;

		params->ci_sdcch_meas.lower_thresh = osmo_thresh->l_ci_sdcch;
		params->ci_sdcch_meas.upper_thresh = osmo_thresh->u_ci_sdcch;

		params->ci_gprs_meas.lower_thresh = osmo_thresh->l_ci_gprs;
		params->ci_gprs_meas.upper_thresh = osmo_thresh->u_ci_gprs;
	}

	/* (TV) PC Threshold Comparators */
	if ((ie = TLVP_GET(&tp[0], RSL_IPAC_EIE_PC_THRESH_COMP)) != NULL) {
		const struct ipac_preproc_pc_comp *thresh_comp;

		thresh_comp = (const struct ipac_preproc_pc_comp *) ie->val;

		/* RxLev: P1, N1, P2, N2 (see 3GPP TS 45.008, A.3.2.1, a & b) */
		params->rxlev_meas.lower_cmp_p = thresh_comp->p1;
		params->rxlev_meas.lower_cmp_n = thresh_comp->n1;
		params->rxlev_meas.upper_cmp_p = thresh_comp->p2;
		params->rxlev_meas.upper_cmp_n = thresh_comp->n2;

		/* RxQual: P3, N3, P4, N4 (see 3GPP TS 45.008, A.3.2.1, c & d) */
		params->rxqual_meas.lower_cmp_p = thresh_comp->p3;
		params->rxqual_meas.lower_cmp_n = thresh_comp->n3;
		params->rxqual_meas.upper_cmp_p = thresh_comp->p4;
		params->rxqual_meas.upper_cmp_n = thresh_comp->n4;

		/* Minimum interval between power level changes (P_Con_INTERVAL) */
		params->ctrl_interval = thresh_comp->pc_interval;

		/* Power increase / reduce step size: POWER_{INC,RED}_STEP_SIZE */
		params->inc_step_size_db = thresh_comp->inc_step_size;
		params->red_step_size_db = thresh_comp->red_step_size;
	}

	/* Osmocom extension, C/I related thresholds: */
	if (TLVP_PRES_LEN(&tp[0], RSL_IPAC_EIE_OSMO_PC_THRESH_COMP, sizeof(struct osmo_preproc_pc_thresh))) {
		const struct osmo_preproc_pc_comp *osmo_thresh_comp;
		ie = TLVP_GET(&tp[0], RSL_IPAC_EIE_OSMO_PC_THRESH_COMP);
		osmo_thresh_comp = (const struct osmo_preproc_pc_comp *) ie->val;
		#define SET_PREPROC_PC(PARAMS, FROM, TYPE) \
			(PARAMS)->TYPE##_meas.lower_cmp_p = (FROM)->TYPE.lower_p; \
			(PARAMS)->TYPE##_meas.lower_cmp_n = (FROM)->TYPE.lower_n; \
			(PARAMS)->TYPE##_meas.upper_cmp_p = (FROM)->TYPE.upper_p; \
			(PARAMS)->TYPE##_meas.upper_cmp_n = (FROM)->TYPE.upper_n
		SET_PREPROC_PC(params, osmo_thresh_comp, ci_fr);
		SET_PREPROC_PC(params, osmo_thresh_comp, ci_hr);
		SET_PREPROC_PC(params, osmo_thresh_comp, ci_amr_fr);
		SET_PREPROC_PC(params, osmo_thresh_comp, ci_amr_hr);
		SET_PREPROC_PC(params, osmo_thresh_comp, ci_sdcch);
		SET_PREPROC_PC(params, osmo_thresh_comp, ci_gprs);
		#undef SET_PREPROC_PC
	}

	/* (TLV) Measurement Averaging parameters for RxLev/RxQual */
	for (i = 0; i < ARRAY_SIZE(tp); i++) {
		const struct ipac_preproc_ave_cfg *ave_cfg;
		struct gsm_power_ctrl_meas_params *mp;

		ie = TLVP_GET(&tp[i], RSL_IPAC_EIE_MEAS_AVG_CFG);
		if (ie == NULL)
			break;

		if (ie->len < sizeof(*ave_cfg))
			return -EINVAL;

		ave_cfg = (const struct ipac_preproc_ave_cfg *) ie->val;

		switch (ave_cfg->param_id) {
		case IPAC_RXQUAL_AVE:
			mp = &params->rxqual_meas;
			break;
		case IPAC_RXLEV_AVE:
			mp = &params->rxlev_meas;
			break;
		default:
			/* Skip unknown parameters */
			continue;
		}

		mp->h_reqave = ave_cfg->h_reqave;
		mp->h_reqt = ave_cfg->h_reqt;

		mp->algo = ave_cfg->ave_method + 1;
		switch (ave_cfg->ave_method) {
		case IPAC_OSMO_EWMA_AVE:
			if (ie->len > sizeof(*ave_cfg))
				mp->ewma.alpha = ave_cfg->params[0];
			break;

		/* FIXME: not implemented */
		case IPAC_UNWEIGHTED_AVE:
		case IPAC_WEIGHTED_AVE:
		case IPAC_MEDIAN_AVE:
			break;
		}
	}

	/* (TLV) Measurement Averaging parameters for C/I (Osmocom extension)*/
	if (TLVP_PRES_LEN(&tp[0], RSL_IPAC_EIE_OSMO_MEAS_AVG_CFG, sizeof(struct osmo_preproc_ave_cfg))) {
		ie = TLVP_GET(&tp[0], RSL_IPAC_EIE_OSMO_MEAS_AVG_CFG);
		const struct osmo_preproc_ave_cfg *cfg = (const struct osmo_preproc_ave_cfg *) ie->val;
		unsigned params_offset = 0;
		#define SET_AVE_CFG(PARAMS, FROM, TYPE, PARAM_OFFSET) do {\
				if ((FROM)->TYPE.ave_enabled) { \
					(PARAMS)->TYPE##_meas.h_reqave = (FROM)->TYPE.h_reqave; \
					(PARAMS)->TYPE##_meas.h_reqt = (FROM)->TYPE.h_reqt; \
					(PARAMS)->TYPE##_meas.algo = (FROM)->TYPE.ave_method + 1; \
					switch ((FROM)->TYPE.ave_method) { \
					case IPAC_OSMO_EWMA_AVE: \
						if (ie->len > sizeof(*cfg) + (PARAM_OFFSET)) {  \
							(PARAMS)->TYPE##_meas.ewma.alpha = (FROM)->params[PARAM_OFFSET]; \
							(PARAM_OFFSET)++; \
						} \
						break; \
					/* FIXME: not implemented */ \
					case IPAC_UNWEIGHTED_AVE: \
					case IPAC_WEIGHTED_AVE: \
					case IPAC_MEDIAN_AVE: \
						break; \
					} \
				} else { \
					(PARAMS)->TYPE##_meas.algo = GSM_PWR_CTRL_MEAS_AVG_ALGO_NONE; \
				} \
			} while(0)
		SET_AVE_CFG(params, cfg, ci_fr, params_offset);
		SET_AVE_CFG(params, cfg, ci_hr, params_offset);
		SET_AVE_CFG(params, cfg, ci_amr_fr, params_offset);
		SET_AVE_CFG(params, cfg, ci_amr_hr, params_offset);
		SET_AVE_CFG(params, cfg, ci_sdcch, params_offset);
		SET_AVE_CFG(params, cfg, ci_gprs, params_offset);
		#undef SET_AVE_CFG
	}

	return 0;
}

/* ip.access specific Measurement Pre-processing Defaults for MS/BS Power control */
static int rsl_rx_meas_preproc_dft(struct gsm_bts_trx *trx, struct msgb *msg)
{
	const struct gsm_bts *bts = trx->bts;
	struct gsm_power_ctrl_params *params;
	const struct tlv_p_entry *ie;
	struct tlv_parsed tp;

	LOGPTRX(trx, DRSL, LOGL_INFO, "Rx Measurement Pre-processing Defaults\n");

	if (rsl_tlv_parse(&tp, msgb_l3(msg), msgb_l3len(msg)) < 0) {
		LOGPTRX(trx, DRSL, LOGL_ERROR, "%s(): rsl_tlv_parse() failed\n", __func__);
		return rsl_tx_error_report(trx, RSL_ERR_PROTO, NULL, NULL, msg);
	}

	/* TLV (O) BS Power Parameters IE */
	if ((ie = TLVP_GET(&tp, RSL_IE_BS_POWER_PARAM)) != NULL) {
		/* Allocate a new chunk and initialize with default values */
		params = talloc(trx, struct gsm_power_ctrl_params);
		power_ctrl_params_def_reset(params, true);

		if (ie->len && parse_power_ctrl_params(params, ie->val, ie->len) == 0) {
			/* Initially it points to the global defaults */
			if (trx->bs_dpc_params != &bts->bs_dpc_params)
				talloc_free(trx->bs_dpc_params);
			trx->bs_dpc_params = params;
		} else {
			LOGPTRX(trx, DRSL, LOGL_ERROR, "Failed to parse BS Power Parameters IE\n");
			rsl_tx_error_report(trx, RSL_ERR_IE_CONTENT, NULL, NULL, msg);
			talloc_free(params);
		}
	}

	/* TLV (O) MS Power Parameters IE */
	if ((ie = TLVP_GET(&tp, RSL_IE_MS_POWER_PARAM)) != NULL) {
		/* Allocate a new chunk and initialize with default values */
		params = talloc(trx, struct gsm_power_ctrl_params);
		power_ctrl_params_def_reset(params, false);

		if (ie->len && parse_power_ctrl_params(params, ie->val, ie->len) == 0) {
			/* Initially it points to the global defaults */
			if (trx->ms_dpc_params != &bts->ms_dpc_params)
				talloc_free(trx->ms_dpc_params);
			trx->ms_dpc_params = params;
		} else {
			LOGPTRX(trx, DRSL, LOGL_ERROR, "Failed to parse MS Power Parameters IE\n");
			rsl_tx_error_report(trx, RSL_ERR_IE_CONTENT, NULL, NULL, msg);
			talloc_free(params);
		}
	}

	return 0;
}

/* 8.5.6 IMMEDIATE ASSIGN COMMAND is received */
static int rsl_rx_imm_ass(struct gsm_bts_trx *trx, struct msgb *msg)
{
	struct abis_rsl_cchan_hdr *cch = msgb_l2(msg);
	struct tlv_parsed tp;

	if (rsl_tlv_parse(&tp, msgb_l3(msg), msgb_l3len(msg)) < 0) {
		LOGPTRX(trx, DRSL, LOGL_ERROR, "%s(): rsl_tlv_parse() failed\n", __func__);
		return rsl_tx_error_report(trx, RSL_ERR_PROTO, &cch->chan_nr, NULL, msg);
	}

	if (!TLVP_PRESENT(&tp, RSL_IE_FULL_IMM_ASS_INFO))
		return rsl_tx_error_report(trx, RSL_ERR_MAND_IE_ERROR, &cch->chan_nr, NULL, msg);

	rate_ctr_inc2(trx->bts->ctrs, BTS_CTR_AGCH_RCVD);

	/* cut down msg to the 04.08 RR part */
	msg->l3h = (uint8_t *) TLVP_VAL(&tp, RSL_IE_FULL_IMM_ASS_INFO);
	msg->data = msg->l3h;
	msg->l2h = NULL;
	msg->len = TLVP_LEN(&tp, RSL_IE_FULL_IMM_ASS_INFO);

	/* Early Immediate Assignment: when there is a lot of latency on Abis, the Abis roundtrip of Chan Activ -> Chan
	 * Activ ACK -> Immediate Assignment may take so long that each MS sends a second RACH for Chan Rqd, reserving
	 * two SDCCH for each request but using only one. To help with that, the Early IA feature in osmo-bsc sends the
	 * Immediate Assignment without waiting for the Channel Activation ACK. This may then be too early, and the MS
	 * may not be able to establish a channel. So to help with Early IA, look up whether the target lchan is already
	 * active. If not, then hold back the RR Immediate Assignment message, and send it once L1 has confirmed that
	 * the channel is active. Hence we still wait for the activation, but don't need the Abis roundtrip of Activ ACK
	 * -> Immediate Assignment via the BSC.
	 * If anything is wrong with the sizes or the lchan lookup, behave normally, i.e. do not do the RR IA caching,
	 * but just send the RR message to the MS as-is. */
	if (msg->len >= sizeof(struct gsm48_imm_ass)) {
		struct gsm48_imm_ass *rr_ia = (void*)msg->data;
		struct gsm_lchan *ia_target_lchan = lchan_lookup(trx, rr_ia->chan_desc.chan_nr, "Early IA check: ");
		if (ia_target_lchan && ia_target_lchan->state != LCHAN_S_ACTIVE) {
			/* Target lchan is not yet active. Cache the IA.
			 * If a previous IA is still lingering, free it. */
			msgb_free(ia_target_lchan->early_rr_ia);
			ia_target_lchan->early_rr_ia = msg;

			/* return 1 means: don't msgb_free() the msg */
			return 1;
		}
	}

	/* put into the AGCH queue of the BTS */
	if (bts_agch_enqueue(trx->bts, msg) < 0) {
		/* if there is no space in the queue: send DELETE IND */
		rsl_tx_delete_ind(trx->bts, TLVP_VAL(&tp, RSL_IE_FULL_IMM_ASS_INFO),
				  TLVP_LEN(&tp, RSL_IE_FULL_IMM_ASS_INFO));
		rate_ctr_inc2(trx->bts->ctrs, BTS_CTR_AGCH_DELETED);
		msgb_free(msg);
	}

	/* return 1 means: don't msgb_free() the msg */
	return 1;
}

/*
 * dedicated channel related messages
 */

/* Send an RF CHANnel RELease ACKnowledge with the given chan_nr. This chan_nr may mismatch the current
 * lchan state, if we received a CHANnel RELease for an already released channel, and we're just acking
 * what we got without taking any action. */
static int tx_rf_rel_ack(struct gsm_lchan *lchan, uint8_t chan_nr)
{
	struct msgb *msg;

	msg = rsl_msgb_alloc(sizeof(struct abis_rsl_dchan_hdr));
	if (!msg)
		return -ENOMEM;

	rsl_dch_push_hdr(msg, RSL_MT_RF_CHAN_REL_ACK, chan_nr);
	msg->trx = lchan->ts->trx;

	return abis_bts_rsl_sendmsg(msg);
}

/* 8.4.19 sending RF CHANnel RELease ACKnowledge */
int rsl_tx_rf_rel_ack(struct gsm_lchan *lchan)
{
	uint8_t chan_nr = gsm_lchan2chan_nr_rsl(lchan);
	bool send_rel_ack;

	switch (lchan->rel_act_kind) {
	case LCHAN_REL_ACT_RSL:
		send_rel_ack = true;
		break;

	case LCHAN_REL_ACT_PCU:
		switch (lchan->ts->pchan) {
		case GSM_PCHAN_OSMO_DYN:
			if (lchan->ts->dyn.pchan_is != GSM_PCHAN_PDCH) {
				LOGP(DRSL, LOGL_ERROR, "%s (ss=%d) PDCH release: not in PDCH mode\n",
				     gsm_ts_and_pchan_name(lchan->ts), lchan->nr);
				/* well, what to do about it ... carry on and hope it's fine. */
			}
			if (lchan->ts->dyn.pchan_want != GSM_PCHAN_PDCH) {
				/* Continue to ack the release below. (This is a non-standard rel ack invented
				 * specifically for GSM_PCHAN_OSMO_DYN). */
				/* remember the fact that the TS is now released */
				lchan->ts->dyn.pchan_is = GSM_PCHAN_NONE;
				send_rel_ack = true;
			} else {
				/* Administrteively locked TRX, no need to
				   inform BSC. Keep pchan_is for when we are
				   unlocked again, since lower layers are stil
				   lconfigured for PDCH but we simply annonced
				   non-availability to PCU */
				send_rel_ack = false;
			}
			break;
		case GSM_PCHAN_TCH_F_PDCH:
			/* GSM_PCHAN_TCH_F_PDCH, does not require a rel ack. The caller
			 * l1sap_info_rel_cnf() will continue with bts_model_ts_disconnect(). */
			send_rel_ack = false;
			break;
		case GSM_PCHAN_PDCH:
			/* Release was instructed by the BTS, for instance because the TRX is
			 * administrateively Locked */
			send_rel_ack = false;
			break;
		default:
			LOGPLCHAN(lchan, DRSL, LOGL_ERROR, "PCU rel ack for unexpected lchan kind %s\n",
				  gsm_pchan_name(lchan->ts->pchan));
			/* Release certainly was not requested by the BSC via RSL, so don't ack. */
			send_rel_ack = false;
			break;
		}
		break;

	default:
		/* A rel that was not requested by the BSC via RSL, hence not sending a rel ack to the
		 * BSC. */
		send_rel_ack = false;
		break;
	}

	if (!send_rel_ack) {
		LOGPLCHAN(lchan, DRSL, LOGL_INFO, "not sending REL ACK\n");
		return 0;
	}

	LOGP(DRSL, LOGL_INFO, "%s (ss=%d) %s Tx CHAN REL ACK\n",
	     gsm_ts_and_pchan_name(lchan->ts), lchan->nr,
	     gsm_lchant_name(lchan->type));

	return tx_rf_rel_ack(lchan, chan_nr);
}

/* 8.4.2 sending CHANnel ACTIVation ACKnowledge */
static int rsl_tx_chan_act_ack(struct gsm_lchan *lchan)
{
	struct gsm_time *gtime = get_time(lchan->ts->trx->bts);
	struct msgb *msg;
	uint8_t chan_nr = gsm_lchan2chan_nr_rsl(lchan);
	uint8_t ie[2];

	LOGP(DRSL, LOGL_INFO, "%s (ss=%d) %s Tx CHAN ACT ACK\n",
	     gsm_ts_and_pchan_name(lchan->ts), lchan->nr,
	     gsm_lchant_name(lchan->type));

	msg = rsl_msgb_alloc(sizeof(struct abis_rsl_dchan_hdr));
	if (!msg)
		return -ENOMEM;

	gsm48_gen_starting_time(ie, gtime);
	msgb_tv_fixed_put(msg, RSL_IE_FRAME_NUMBER, 2, ie);
	rsl_dch_push_hdr(msg, RSL_MT_CHAN_ACTIV_ACK, chan_nr);
	msg->trx = lchan->ts->trx;

	/* since activation was successful, do some lchan initialization */
	lchan_meas_reset(lchan);

	return abis_bts_rsl_sendmsg(msg);
}

/* common helper function for *_DETECT */
static int _rsl_tx_detect(struct gsm_lchan *lchan, uint8_t msg_type, uint8_t *acc_delay)
{
	struct msgb *msg;
	uint8_t chan_nr = gsm_lchan2chan_nr_rsl(lchan);

	msg = rsl_msgb_alloc(sizeof(struct abis_rsl_dchan_hdr));
	if (!msg)
		return -ENOMEM;

	/* 9.3.17 Access Delay */
	if (acc_delay)
		msgb_tv_put(msg, RSL_IE_ACCESS_DELAY, *acc_delay);

	rsl_dch_push_hdr(msg, msg_type, chan_nr);
	msg->trx = lchan->ts->trx;

	return abis_bts_rsl_sendmsg(msg);
}

/* 8.4.7 sending HANDOver DETection */
int rsl_tx_hando_det(struct gsm_lchan *lchan, uint8_t *ho_delay)
{
	LOGPLCHAN(lchan, DRSL, LOGL_INFO, "Sending HANDOver DETect\n");

	return _rsl_tx_detect(lchan, RSL_MT_HANDO_DET, ho_delay);
}

/* 8.4.22 sending LISTENER DETection */
int rsl_tx_listener_det(struct gsm_lchan *lchan, uint8_t *acc_delay)
{
	LOGPLCHAN(lchan, DRSL, LOGL_INFO, "Sending LISTENER DETect\n");

	return _rsl_tx_detect(lchan, RSL_MT_LISTENER_DET, acc_delay);
}

/* 8.4.21 sending TALKER DETection */
int rsl_tx_talker_det(struct gsm_lchan *lchan, uint8_t *acc_delay)
{
	LOGPLCHAN(lchan, DRSL, LOGL_INFO, "Sending TALKER DETect\n");

	return _rsl_tx_detect(lchan, RSL_MT_TALKER_DET, acc_delay);
}

/* 8.4.3 sending CHANnel ACTIVation Negative ACK */
static int _rsl_tx_chan_act_nack(struct gsm_bts_trx *trx, uint8_t chan_nr, uint8_t cause,
				 struct gsm_lchan *lchan)
{
	struct msgb *msg;

	if (lchan)
		LOGPLCHAN(lchan, DRSL, LOGL_NOTICE, "");
	else
		LOGP(DRSL, LOGL_NOTICE, "0x%02x: ", chan_nr);
	LOGPC(DRSL, LOGL_NOTICE, "Sending Channel Activated NACK: cause = 0x%02x\n", cause);

	msg = rsl_msgb_alloc(sizeof(struct abis_rsl_dchan_hdr));
	if (!msg)
		return -ENOMEM;

	/* 9.3.26 Cause */
	msgb_tlv_put(msg, RSL_IE_CAUSE, 1, &cause);
	rsl_dch_push_hdr(msg, RSL_MT_CHAN_ACTIV_NACK, chan_nr);
	msg->trx = trx;

	return abis_bts_rsl_sendmsg(msg);
}
static int rsl_tx_chan_act_nack(struct gsm_lchan *lchan, uint8_t cause) {
	return _rsl_tx_chan_act_nack(lchan->ts->trx, gsm_lchan2chan_nr_rsl(lchan), cause, lchan);
}

/* Send an RSL Channel Activation Ack if cause is zero, a Nack otherwise. */
int rsl_tx_chan_act_acknack(struct gsm_lchan *lchan, uint8_t cause)
{
	if (lchan->rel_act_kind != LCHAN_REL_ACT_RSL) {
		LOGPLCHAN(lchan, DRSL, LOGL_INFO, "not sending CHAN ACT %s\n",
			  cause ? "NACK" : "ACK");
		return 0;
	}

	if (cause)
		return rsl_tx_chan_act_nack(lchan, cause);
	return rsl_tx_chan_act_ack(lchan);
}

/* 8.4.4 sending CONNection FAILure */
int rsl_tx_conn_fail(const struct gsm_lchan *lchan, uint8_t cause)
{
	struct msgb *msg;
	uint8_t chan_nr = gsm_lchan2chan_nr_rsl(lchan);

	LOGPLCHAN(lchan, DRSL, LOGL_NOTICE, "Sending Connection Failure: cause = 0x%02x\n", cause);

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

	rsl_cch_push_hdr(nmsg, RSL_MT_CHAN_RQD, RSL_CHAN_RACH); // FIXME
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
		int osmo_si = osmo_rsl2sitype(rsl_si);
		uint32_t osmo_si_shifted = (1 << osmo_si);
		osmo_static_assert(_MAX_SYSINFO_TYPE <= sizeof(osmo_si_shifted) * 8,
				   si_enum_vals_fit_in_bit_mask);

		if (osmo_si == SYSINFO_TYPE_NONE)
			continue;
		if (!(bts->si_valid & osmo_si_shifted)) {
			lchan->si.valid &= ~osmo_si_shifted;
			continue;
		}
		lchan->si.valid |= osmo_si_shifted;
		memcpy(GSM_LCHAN_SI(lchan, osmo_si), GSM_BTS_SI(bts, osmo_si), sizeof(sysinfo_buf_t));
	}
}


static int encr_info2lchan(struct gsm_lchan *lchan,
			   const uint8_t *val, uint8_t len)
{
	int rc;
	struct gsm_bts *bts = lchan->ts->trx->bts;
	const char *ciph_name = get_value_string(gsm0808_chosen_enc_alg_names, *val);

	/* check if the encryption algorithm sent by BSC is supported! */
	rc = bts_supports_cipher(bts, *val);
	if (rc != 1) {
		LOGPLCHAN(lchan, DRSL, LOGL_ERROR, "BTS doesn't support cipher %s\n", ciph_name);
		return -EINVAL;
	}

	/* length can be '1' in case of no ciphering */
	if (len < 1) {
		LOGPLCHAN(lchan, DRSL, LOGL_ERROR, "Encryption Info cannot have len=%d\n", len);
		return -EINVAL;
	}

	lchan->encr.alg_id = *val++;
	lchan->encr.key_len = len -1;
	if (lchan->encr.key_len > sizeof(lchan->encr.key))
		lchan->encr.key_len = sizeof(lchan->encr.key);
	memcpy(lchan->encr.key, val, lchan->encr.key_len);
	LOGPLCHAN(lchan, DRSL, LOGL_DEBUG, "Setting lchan cipher algorithm %s\n", ciph_name);

	return 0;
}

/* Make sure no state from TCH use remains. */
static void clear_lchan_for_pdch_activ(struct gsm_lchan *lchan)
{
	/* These values don't apply to PDCH, just clear them. Particularly the encryption must be
	 * cleared, or we would enable encryption on PDCH with parameters remaining from the TCH. */
	lchan->rsl_cmode = 0;
	lchan->tch_mode = 0;
	memset(&lchan->encr, 0, sizeof(lchan->encr));
	memset(&lchan->ho, 0, sizeof(lchan->ho));
	memset(&lchan->ms_power_ctrl, 0, sizeof(lchan->ms_power_ctrl));
	memset(&lchan->bs_power_ctrl, 0, sizeof(lchan->bs_power_ctrl));
	lchan->ta_ctrl.current = 0;
	copy_sacch_si_to_lchan(lchan);
	memset(&lchan->tch, 0, sizeof(lchan->tch));
}

/*!
 * Store the CHAN_ACTIV msg, connect the L1 timeslot in the proper type and
 * then invoke rsl_rx_chan_activ() with msg.
 */
static int dyn_ts_l1_reconnect(struct gsm_bts_trx_ts *ts)
{
	DEBUGP(DRSL, "%s dyn_ts_l1_reconnect\n", gsm_ts_and_pchan_name(ts));

	switch (ts->dyn.pchan_want) {
	case GSM_PCHAN_TCH_F:
	case GSM_PCHAN_TCH_H:
	case GSM_PCHAN_SDCCH8_SACCH8C:
		break;
	case GSM_PCHAN_PDCH:
		/* Only the first lchan matters for PDCH */
		clear_lchan_for_pdch_activ(ts->lchan);
		break;
	default:
		LOGP(DRSL, LOGL_ERROR,
		     "%s Cannot reconnect as pchan %s\n",
		     gsm_ts_and_pchan_name(ts),
		     gsm_pchan_name(ts->dyn.pchan_want));
		return -EINVAL;
	}

	/* Disconnect, continue connecting from cb_ts_disconnected(). */
	DEBUGP(DRSL, "%s Disconnect\n", gsm_ts_and_pchan_name(ts));
	return bts_model_ts_disconnect(ts);
}

static enum gsm_phys_chan_config dyn_pchan_from_chan_nr(uint8_t chan_nr)
{
	uint8_t cbits = chan_nr >> 3;
	switch (cbits) {
	case ABIS_RSL_CHAN_NR_CBITS_Bm_ACCHs:
		return GSM_PCHAN_TCH_F;
	case ABIS_RSL_CHAN_NR_CBITS_Lm_ACCHs(0):
	case ABIS_RSL_CHAN_NR_CBITS_Lm_ACCHs(1):
		return GSM_PCHAN_TCH_H;
	case ABIS_RSL_CHAN_NR_CBITS_SDCCH8_ACCH(0):
	case ABIS_RSL_CHAN_NR_CBITS_SDCCH8_ACCH(1):
	case ABIS_RSL_CHAN_NR_CBITS_SDCCH8_ACCH(2):
	case ABIS_RSL_CHAN_NR_CBITS_SDCCH8_ACCH(3):
	case ABIS_RSL_CHAN_NR_CBITS_SDCCH8_ACCH(4):
	case ABIS_RSL_CHAN_NR_CBITS_SDCCH8_ACCH(5):
	case ABIS_RSL_CHAN_NR_CBITS_SDCCH8_ACCH(6):
	case ABIS_RSL_CHAN_NR_CBITS_SDCCH8_ACCH(7):
		return GSM_PCHAN_SDCCH8_SACCH8C;
	case ABIS_RSL_CHAN_NR_CBITS_OSMO_PDCH:
		return GSM_PCHAN_PDCH;
	default:
		LOGP(DRSL, LOGL_ERROR,
		     "chan nr 0x%x not covered by dyn_pchan_from_chan_nr()\n",
		     chan_nr);
		return GSM_PCHAN_UNKNOWN;
	}
}

/* Parse RSL_IE_OSMO_REP_ACCH_CAP */
static int parse_repeated_acch_capability(struct gsm_lchan *lchan, struct tlv_parsed *tp)
{
	/* 3GPP TS 24.008, section 10.5.1.7 defines a Repeated ACCH Capability
	 * bit that indicates if REPEATED FACCH/SACCH is supported or not.
	 * Unfortunately there is not 3gpp spec that describes how this bit
	 * should be communicated in the RSL CHANNEL ACTIVATION. For osmo-bts
	 * we will use a propritary IE. */

	memset(&lchan->rep_acch_cap, 0, sizeof(lchan->rep_acch_cap));

	if (!TLVP_PRES_LEN(tp, RSL_IE_OSMO_REP_ACCH_CAP, sizeof(lchan->rep_acch_cap)))
		return 0;

	if (!osmo_bts_has_feature(lchan->ts->trx->bts->features, BTS_FEAT_ACCH_REP))
		return -RSL_ERR_OPT_IE_ERROR;

	memcpy(&lchan->rep_acch_cap, TLVP_VAL(tp, RSL_IE_OSMO_REP_ACCH_CAP),
	       sizeof(lchan->rep_acch_cap));

	return 0;
}

/* Parse RSL_IE_OSMO_TOP_ACCH_CAP */
static int parse_temporary_overpower_acch_capability(struct gsm_lchan *lchan,
						     const struct tlv_parsed *tp)
{
	memset(&lchan->top_acch_cap, 0, sizeof(lchan->top_acch_cap));

	if (!TLVP_PRES_LEN(tp, RSL_IE_OSMO_TEMP_OVP_ACCH_CAP, sizeof(lchan->top_acch_cap)))
		return 0;

	if (!osmo_bts_has_feature(lchan->ts->trx->bts->features, BTS_FEAT_ACCH_TEMP_OVP))
		return -RSL_ERR_OPT_IE_ERROR;

	memcpy(&lchan->top_acch_cap,
	       TLVP_VAL(tp, RSL_IE_OSMO_TEMP_OVP_ACCH_CAP),
	       sizeof(lchan->top_acch_cap));

	/* Simplify checking whether the overpower is enabled at all: allow
	 * testing just one parameter (overpower_db > 0) instead of all three. */
	if (!lchan->top_acch_cap.sacch_enable && !lchan->top_acch_cap.facch_enable)
		lchan->top_acch_cap.overpower_db = 0;

	return 0;
}

/* Parse (O) MultiRate configuration IE (see 9.3.52) */
static int parse_multirate_config(struct gsm_lchan *lchan,
				  const struct tlv_parsed *tp)
{
	int rc;

	if (!TLVP_PRESENT(tp, RSL_IE_MR_CONFIG)) {
		/* Included if the Channel Mode indicates that a multi-rate codec is used */
		if (lchan->tch_mode == GSM48_CMODE_SPEECH_AMR) {
			LOGPLCHAN(lchan, DRSL, LOGL_NOTICE, "Missing MultiRate conf IE "
				  "(TCH mode is %s)\n", gsm48_chan_mode_name(lchan->tch_mode));
			/* Init lchan->tch.amr_mr with hard-coded default values */
			amr_init_mr_conf_def(lchan);
			goto parsed;
		}
		return 0;
	}

	/* Included if the Channel Mode indicates that a multi-rate codec is used */
	if (lchan->tch_mode != GSM48_CMODE_SPEECH_AMR) {
		LOGPLCHAN(lchan, DRSL, LOGL_ERROR, "Unexpected MultiRate conf IE "
			  "(TCH mode is %s)\n", gsm48_chan_mode_name(lchan->tch_mode));
		return -RSL_ERR_OPT_IE_ERROR;
	}

	rc = amr_parse_mr_conf(&lchan->tch.amr_mr,
			       TLVP_VAL(tp, RSL_IE_MR_CONFIG),
			       TLVP_LEN(tp, RSL_IE_MR_CONFIG));
	if (rc < 0) {
		LOGPLCHAN(lchan, DRSL, LOGL_ERROR, "Error parsing MultiRate conf IE\n");
		return -RSL_ERR_IE_CONTENT;
	}

parsed:
	amr_log_mr_conf(DRTP, LOGL_DEBUG, gsm_lchan_name(lchan), &lchan->tch.amr_mr);
	lchan->tch.last_cmr = AMR_CMR_NONE;
	return 0;
}

/* 8.4.1 CHANnel ACTIVation is received */
static int rsl_rx_chan_activ(struct msgb *msg)
{
	struct abis_rsl_dchan_hdr *dch = msgb_l2(msg);
	struct gsm_lchan *lchan = msg->lchan;
	struct gsm_bts_trx_ts *ts = lchan->ts;
	struct gsm_bts_trx_ts *primary_ts;
	struct tlv_parsed tp;
	const struct tlv_p_entry *ie;
	uint8_t type, cause;
	bool reactivation = false;
	int rc;

	if (rsl_tlv_parse(&tp, msgb_l3(msg), msgb_l3len(msg)) < 0) {
		LOGPLCHAN(lchan, DRSL, LOGL_ERROR, "%s(): rsl_tlv_parse() failed\n", __func__);
		return rsl_tx_chan_act_nack(lchan, RSL_ERR_PROTO);
	}

	/* 9.3.3 Activation Type */
	if (!TLVP_PRESENT(&tp, RSL_IE_ACT_TYPE)) {
		LOGPLCHAN(lchan, DRSL, LOGL_NOTICE, "missing Activation Type\n");
		return rsl_tx_chan_act_nack(lchan, RSL_ERR_MAND_IE_ERROR);
	}
	type = *TLVP_VAL(&tp, RSL_IE_ACT_TYPE);
	if ((type & RSL_ACT_TYPE_REACT)) {
		type -= RSL_ACT_TYPE_REACT;
		reactivation = true;
	}

	if (!reactivation && lchan->state != LCHAN_S_NONE) {
		LOGPLCHAN(lchan, DRSL, LOGL_ERROR, "error: lchan is not available, but in state: %s.\n",
			  gsm_lchans_name(lchan->state));
		return rsl_tx_chan_act_nack(lchan, RSL_ERR_EQUIPMENT_FAIL);
	}

	if (reactivation && lchan->state == LCHAN_S_NONE) {
		LOGPLCHAN(lchan, DRSL, LOGL_ERROR, "error: reactivation on inactive lchan.\n");
		return rsl_tx_chan_act_nack(lchan, RSL_ERR_EQUIPMENT_FAIL);
	}

	/* We need to pick the real TS here to check NM state: */
	primary_ts = ts->vamos.is_shadow ? ts->vamos.peer : ts;
	if (primary_ts->mo.nm_state.operational != NM_OPSTATE_ENABLED ||
	    primary_ts->mo.nm_state.availability != NM_AVSTATE_OK) {
		LOGP(DRSL, LOGL_ERROR, "%s rx chan activ but TS not in nm_state oper=ENABLED avail=OK, nack!\n",
		     gsm_ts_and_pchan_name(ts));
		return rsl_tx_chan_act_nack(lchan, RSL_ERR_RR_UNAVAIL);
	}

	if (ts->pchan == GSM_PCHAN_OSMO_DYN) {
		ts->dyn.pchan_want = dyn_pchan_from_chan_nr(dch->chan_nr);
		DEBUGP(DRSL, "%s rx chan activ\n", gsm_ts_and_pchan_name(ts));

		if (ts->dyn.pchan_is != ts->dyn.pchan_want) {
			/*
			 * The phy has the timeslot connected in a different
			 * mode than this activation needs it to be.
			 * Re-connect, then come back to rsl_rx_chan_activ().
			 */
			rc = dyn_ts_l1_reconnect(ts);
			if (rc)
				return rsl_tx_chan_act_nack(lchan, RSL_ERR_NORMAL_UNSPEC);
			/* will be fed back to rsl_rx_chan_activ() later */
			OSMO_ASSERT(lchan->pending_chan_activ == NULL);
			lchan->pending_chan_activ = msg;
			/* indicate that the msgb should not be freed. */
			return 1;
		}
	}

	/* Initialize MS Power Control defaults */
	lchan->ms_power_ctrl = (struct lchan_power_ctrl_state) {
		.max = ms_pwr_ctl_lvl(lchan->ts->trx->bts->band, 0),
		.current = lchan->ms_power_ctrl.max,
	};

	/* Initialize BS Power Control defaults */
	lchan->bs_power_ctrl = (struct lchan_power_ctrl_state) {
		.max = 2 * 15, /* maximum defined in 9.3.4 */
		.current = 0,
	};

	/* 9.3.6 Channel Mode */
	if (type != RSL_ACT_OSMO_PDCH) {
		if (rsl_handle_chan_mod_ie(lchan, &tp, &cause) != 0)
			return rsl_tx_chan_act_nack(lchan, cause);
		if (rsl_handle_chan_ident_ie(lchan, &tp, &cause) != 0)
			return rsl_tx_chan_act_nack(lchan, cause);
		if (rsl_handle_osmo_tsc_ie(lchan, &tp, &cause) != 0)
			return rsl_tx_chan_act_nack(lchan, cause);
	}

	/* 9.3.7 Encryption Information */
	if (TLVP_PRESENT(&tp, RSL_IE_ENCR_INFO)) {
		uint8_t len = TLVP_LEN(&tp, RSL_IE_ENCR_INFO);
		const uint8_t *val = TLVP_VAL(&tp, RSL_IE_ENCR_INFO);

		if (encr_info2lchan(lchan, val, len) < 0) {
			rsl_tx_error_report(msg->trx, RSL_ERR_IE_CONTENT, &dch->chan_nr, NULL, msg);
			return rsl_tx_chan_act_acknack(lchan, RSL_ERR_ENCR_UNIMPL);
		}
	} else
		memset(&lchan->encr, 0, sizeof(lchan->encr));

	/* 9.3.9 Handover Reference */
	if ((type == RSL_ACT_INTER_ASYNC || type == RSL_ACT_INTER_SYNC)) {
		/* According to 8.4.1, the Handover Reference element is included
		 * if activation type is handover. Assuming it's mandatory. */
		if (!TLVP_PRES_LEN(&tp, RSL_IE_HANDO_REF, 1)) {
			LOGPLCHAN(lchan, DRSL, LOGL_NOTICE, "Missing Handover Reference IE\n");
			return rsl_tx_chan_act_nack(lchan, RSL_ERR_MAND_IE_ERROR);
		}
		lchan->ho.active = HANDOVER_ENABLED;
		lchan->ho.ref = *TLVP_VAL(&tp, RSL_IE_HANDO_REF);
	}

	/* 9.3.4 BS Power */
	if (TLVP_PRES_LEN(&tp, RSL_IE_BS_POWER, 1)) {
		if (*TLVP_VAL(&tp, RSL_IE_BS_POWER) & (1 << 4)) {
			LOGPLCHAN(lchan, DRSL, LOGL_NOTICE,
				  "Fast Power Control is not supported\n");
			return rsl_tx_chan_act_nack(lchan, RSL_ERR_SERV_OPT_UNIMPL);
		}

		uint8_t red = BS_POWER2DB(*TLVP_VAL(&tp, RSL_IE_BS_POWER));

		/* BS power reduction is generally not allowed on BCCH/CCCH carrier.
		 * However, we allow it in the BCCH carrier power reduction operation.
		 * Constrain BS power value by the maximum reduction for this timeslot. */
		if (ts->trx->bts->c0 == ts->trx)
			red = OSMO_MIN(red, ts->c0_power_red_db);

		lchan->bs_power_ctrl.max = red;
		lchan->bs_power_ctrl.current = red;

		LOGPLCHAN(lchan, DRSL, LOGL_DEBUG, "BS Power attenuation %u dB\n",
			  lchan->bs_power_ctrl.current);
	}

	/* 9.3.13 MS Power */
	if (TLVP_PRES_LEN(&tp, RSL_IE_MS_POWER, 1)) {
		lchan->ms_power_ctrl.max = *TLVP_VAL(&tp, RSL_IE_MS_POWER) & 0x1F;
		lchan->ms_power_ctrl.current = lchan->ms_power_ctrl.max;
	}
	/* 9.3.24 Timing Advance */
	if (TLVP_PRES_LEN(&tp, RSL_IE_TIMING_ADVANCE, 1))
		lchan->ta_ctrl.current = *TLVP_VAL(&tp, RSL_IE_TIMING_ADVANCE);

	/* 9.3.31 (TLV) MS Power Parameters IE (vendor specific) */
	if ((ie = TLVP_GET(&tp, RSL_IE_MS_POWER_PARAM)) != NULL) {
		struct gsm_power_ctrl_params *params = &lchan->ms_dpc_params;

		/* Parsed parameters will override per-TRX defaults */
		memcpy(params, ts->trx->ms_dpc_params, sizeof(*params));

		if (ie->len && parse_power_ctrl_params(params, ie->val, ie->len) != 0) {
			LOGPLCHAN(lchan, DRSL, LOGL_ERROR, "Failed to parse MS Power Parameters IE\n");
			return rsl_tx_chan_act_nack(lchan, RSL_ERR_IE_CONTENT);
		}

		/* Spec explicitly states BTS should only perform
		* autonomous MS power control loop in BTS if 'MS Power
		* Parameters' IE is present! */
		lchan->ms_power_ctrl.dpc_params = params;
	}

	/* 9.3.32 (TLV) BS Power Parameters IE (vendor specific) */
	if ((ie = TLVP_GET(&tp, RSL_IE_BS_POWER_PARAM)) != NULL) {
		struct gsm_power_ctrl_params *params = &lchan->bs_dpc_params;

		/* Parsed parameters will override per-TRX defaults */
		memcpy(params, ts->trx->bs_dpc_params, sizeof(*params));

		/* Parsed parameters will override per-TRX defaults */
		if (ie->len && parse_power_ctrl_params(params, ie->val, ie->len) != 0) {
			LOGPLCHAN(lchan, DRSL, LOGL_ERROR, "Failed to parse BS Power Parameters IE\n");
			return rsl_tx_chan_act_nack(lchan, RSL_ERR_IE_CONTENT);
		}

		/* NOTE: it's safer to start from 0 */
		lchan->bs_power_ctrl.current = 0;
		lchan->bs_power_ctrl.dpc_params = params;
	}

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

			if (!OSMO_IN_ARRAY(rsl_si, rsl_sacch_sitypes)) {
				rsl_tx_error_report(msg->trx, RSL_ERR_IE_CONTENT,
						    &dch->chan_nr, NULL, msg);
				return rsl_tx_chan_act_acknack(lchan, RSL_ERR_IE_CONTENT);
			}

			osmo_si = osmo_rsl2sitype(rsl_si);
			if (osmo_si == SYSINFO_TYPE_NONE) {
				LOGPLCHAN(lchan, DRSL, LOGL_NOTICE, "Rx SACCH SI 0x%02x not supported.\n", rsl_si);
				rsl_tx_error_report(msg->trx, RSL_ERR_IE_CONTENT, &dch->chan_nr,
						    NULL, msg);
				return rsl_tx_chan_act_acknack(lchan, RSL_ERR_IE_CONTENT);
			}

			lapdm_ui_prefix_lchan(lchan, cur, osmo_si, si_len);

			cur += si_len;
			if (cur > val + tot_len) {
				LOGPLCHAN(lchan, DRSL, LOGL_ERROR, "Error parsing SACCH INFO IE\n");
				rsl_tx_error_report(msg->trx, RSL_ERR_IE_CONTENT, &dch->chan_nr,
						    NULL, msg);
				return rsl_tx_chan_act_acknack(lchan, RSL_ERR_IE_CONTENT);
			}
		}
	} else {
		/* use standard SACCH filling of the BTS */
		copy_sacch_si_to_lchan(lchan);
	}

	/* 9.3.52 MultiRate Configuration */
	rc = parse_multirate_config(lchan, &tp);
	if (rc < 0)
		return rsl_tx_chan_act_acknack(lchan, -rc);

	/* 9.3.53 MultiRate Control */
	/* 9.3.54 Supported Codec Types */

	LOGPLCHAN(lchan, DRSL, LOGL_INFO, "chan_nr=%s type=0x%02x=%s mode=%s\n",
		  rsl_chan_nr_str(dch->chan_nr),
		  type, get_value_string(rsl_act_type_names, type),
		  gsm48_chan_mode_name(lchan->tch_mode));

	/* Connecting PDCH on dyn TS goes via PCU instead. */
	if (ts->pchan == GSM_PCHAN_OSMO_DYN
	    && ts->dyn.pchan_want == GSM_PCHAN_PDCH) {
		/*
		 * We ack the activation to the BSC right away, regardless of
		 * the PCU succeeding or not; if a dynamic timeslot fails to go
		 * to PDCH mode for any reason, the BSC should still be able to
		 * switch it back to TCH modes and should not put the time slot
		 * in an error state. So for operating dynamic TS, the BSC
		 * would not take any action if the PDCH mode failed, e.g.
		 * because the PCU is not yet running. Even if alerting the
		 * core network of broken GPRS service is desired, this only
		 * makes sense when the PCU has not shown up for some time.
		 * It's easiest to not forward activation delays to the BSC: if
		 * the BSC tells us to do PDCH, we do our best, and keep the
		 * details on the BTS and PCU level. This is kind of analogous
		 * to how plain PDCH TS operate. Directly call
		 * rsl_tx_chan_act_ack() instead of rsl_tx_chan_act_acknack()
		 * because we don't want/need to decide whether to drop due to
		 * lchan->rel_act_kind.
		 */
		rc = rsl_tx_chan_act_ack(lchan);
		if (rc < 0)
			LOGP(DRSL, LOGL_ERROR, "%s Cannot send act ack: %d\n",
			     gsm_ts_and_pchan_name(ts), rc);

		/*
		 * pcu_tx_info_ind() will pick up the ts->dyn.pchan_want. If
		 * the PCU is not connected yet, ignore for now; the PCU will
		 * catch up (and send the RSL ack) once it connects.
		 */
		if (pcu_connected()) {
			DEBUGP(DRSL, "%s Activate via PCU\n", gsm_ts_and_pchan_name(ts));
			rc = pcu_tx_info_ind();
		}
		else {
			DEBUGP(DRSL, "%s Activate via PCU when PCU connects\n",
			       gsm_ts_and_pchan_name(ts));
			rc = 0;
		}
		if (rc) {
			rsl_tx_error_report(msg->trx, RSL_ERR_NORMAL_UNSPEC, &dch->chan_nr, NULL, msg);
			return rsl_tx_chan_act_acknack(lchan, RSL_ERR_NORMAL_UNSPEC);
		}
		return 0;
	}

	/* Indicate which SAPIs should be enabled before the first RACH is received, for handover. See 3GPP TS 48.058
	 * 4.1.3 and 4.1.4.
	 *
	 *          |          | Timing   ||  transmit         |  activate         |  This implementation
	 *          | MS Power | Advance  ||  on main channel  |  dl SACCH         |  activates DL SACCH
	 * -----------------------------------------------------------------------------------------
	 * async ho   no         *       -->  yes                 no                  no
	 * async ho   yes        *       -->  yes                 may be started      no
	 * async ho   yes        yes     -->  yes                 may be started      yes
	 * sync ho    no         no      -->  yes                 no                  no
	 * sync ho    yes        no      -->  yes                 may be started      no
	 * sync ho    yes        yes     -->  yes                 shall be started    yes
	 */
	switch (type) {
	case RSL_ACT_INTER_ASYNC:
	case RSL_ACT_INTER_SYNC:
		lchan->want_dl_sacch_active = (TLVP_PRES_LEN(&tp, RSL_IE_MS_POWER, 1)
					       && TLVP_PRES_LEN(&tp, RSL_IE_TIMING_ADVANCE, 1));
		break;
	default:
		lchan->want_dl_sacch_active = true;
		break;
	}

	/* Remember to send an RSL ACK once the lchan is active */
	lchan->rel_act_kind = LCHAN_REL_ACT_RSL;

	rc = parse_repeated_acch_capability(lchan, &tp);
	if (rc < 0)
		return rsl_tx_chan_act_acknack(lchan, -rc);
	rc = parse_temporary_overpower_acch_capability(lchan, &tp);
	if (rc < 0)
		return rsl_tx_chan_act_acknack(lchan, -rc);

	/* Take the first ACCH overpower decision (if allowed): it can be
	 * enabled immediately if the RxQual threshold is disabled (0). */
	if (lchan->top_acch_cap.overpower_db > 0)
		lchan->top_acch_active = !lchan->top_acch_cap.rxqual;
	else
		lchan->top_acch_active = false;

	/* set ASCI channel into right state */
	if (reactivation && rsl_chan_rt_is_asci(lchan->rsl_chan_rt))
		vgcs_lchan_react(lchan);

	/* on reactivation, the channel is already activated */
	if (reactivation) {
		rc = rsl_tx_chan_act_ack(lchan);
		if (rc < 0)
			LOGP(DRSL, LOGL_ERROR, "%s Cannot send act ack: %d\n",
			     gsm_ts_and_pchan_name(ts), rc);
		return 0;
	}

	/* actually activate the channel in the BTS */
	rc = l1sap_chan_act(lchan->ts->trx, dch->chan_nr);
	if (rc < 0)
		return rsl_tx_chan_act_acknack(lchan, -rc);

	return 0;
}

/* 8.4.14 RF CHANnel RELease is received */
static int rsl_rx_rf_chan_rel(struct gsm_lchan *lchan, uint8_t chan_nr)
{
	if (lchan->state == LCHAN_S_NONE) {
		LOGP(DRSL, LOGL_ERROR,
		     "%s ss=%d state=%s Rx RSL RF Channel Release, but is already inactive;"
		     " just ACKing the release\n",
		     gsm_ts_and_pchan_name(lchan->ts), lchan->nr,
		     gsm_lchans_name(lchan->state));
		/* Just ack the release and ignore. Make sure to reflect the same chan_nr we received,
		 * not necessarily reflecting the current lchan state. */
		return tx_rf_rel_ack(lchan, chan_nr);
	}
	gsm_lchan_release(lchan, LCHAN_REL_ACT_RSL);
	return 0;
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

	rsl_rll_push_l3(fake_msg, RSL_MT_DATA_IND, gsm_lchan2chan_nr_rsl(lchan),
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

	LOGPLCHAN(cmc->lchan, DRSL, LOGL_NOTICE, "Sending FAKE CIPHERING MODE COMPLETE to BSC (Alg %u)\n",
		  cmc->lchan->encr.alg_id);

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

	if (rsl_tlv_parse(&tp, msgb_l3(msg), msgb_l3len(msg)) < 0) {
		LOGPLCHAN(lchan, DRSL, LOGL_ERROR, "%s(): rsl_tlv_parse() failed\n", __func__);
		return rsl_tx_error_report(msg->trx, RSL_ERR_PROTO, &dch->chan_nr, NULL, msg);
	}

	if (!TLVP_PRESENT(&tp, RSL_IE_ENCR_INFO) ||
	    !TLVP_PRESENT(&tp, RSL_IE_L3_INFO) ||
	    !TLVP_PRESENT(&tp, RSL_IE_LINK_IDENT)) {
		return rsl_tx_error_report(msg->trx, RSL_ERR_MAND_IE_ERROR, &dch->chan_nr, NULL, msg);
	}

	/* 9.3.7 Encryption Information */
	if (TLVP_PRESENT(&tp, RSL_IE_ENCR_INFO)) {
		uint8_t len = TLVP_LEN(&tp, RSL_IE_ENCR_INFO);
		const uint8_t *val = TLVP_VAL(&tp, RSL_IE_ENCR_INFO);

		if (encr_info2lchan(lchan, val, len) < 0) {
			 return rsl_tx_error_report(msg->trx, RSL_ERR_IE_CONTENT, &dch->chan_nr,
					 	    NULL, msg);
		}
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
	LOGPLCHAN(lchan, DRSL, LOGL_INFO, "Fwd RSL ENCR CMD (Alg %u) to LAPDm\n", lchan->encr.alg_id);
	/* hand it into RSLms for transmission of L3_INFO to the MS */
	lapdm_rslms_recvmsg(msg, &lchan->lapdm_ch);
	/* return 1 to make sure the msgb is not free'd */
	return 1;
	}
}

/* 8.4.11 MODE MODIFY NEGATIVE ACKNOWLEDGE */
static int _rsl_tx_mode_modif_nack(struct gsm_bts_trx *trx, uint8_t chan_nr, uint8_t cause,
				  struct gsm_lchan *lchan)
{
	struct msgb *msg;

	if (lchan)
		LOGPLCHAN(lchan, DRSL, LOGL_NOTICE, "");
	else
		LOGP(DRSL, LOGL_NOTICE, "0x%02x: ", chan_nr);
	LOGPC(DRSL, LOGL_NOTICE, "Tx MODE MODIFY NACK (cause = 0x%02x)\n", cause);

	msg = rsl_msgb_alloc(sizeof(struct abis_rsl_dchan_hdr));
	if (!msg)
		return -ENOMEM;

	msg->len = 0;
	msg->data = msg->tail = msg->l3h;

	/* 9.3.26 Cause */
	msgb_tlv_put(msg, RSL_IE_CAUSE, 1, &cause);
	rsl_dch_push_hdr(msg, RSL_MT_MODE_MODIFY_NACK, chan_nr);
	msg->trx = trx;

	return abis_bts_rsl_sendmsg(msg);
}
static int rsl_tx_mode_modif_nack(struct gsm_lchan *lchan, uint8_t cause)
{
	return _rsl_tx_mode_modif_nack(lchan->ts->trx, gsm_lchan2chan_nr_rsl(lchan), cause, lchan);
}


/* 8.4.10 MODE MODIFY ACK */
static int rsl_tx_mode_modif_ack(struct gsm_lchan *lchan)
{
	struct msgb *msg;
	uint8_t chan_nr = gsm_lchan2chan_nr_rsl(lchan);

	LOGPLCHAN(lchan, DRSL, LOGL_INFO, "Tx MODE MODIF ACK\n");

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
	struct abis_rsl_dchan_hdr *dch = msgb_l2(msg);
	struct gsm_lchan *lchan = msg->lchan;
	struct tlv_parsed tp;
	uint8_t cause;
	int rc;

	if (rsl_tlv_parse(&tp, msgb_l3(msg), msgb_l3len(msg)) < 0) {
		LOGPLCHAN(lchan, DRSL, LOGL_ERROR, "%s(): rsl_tlv_parse() failed\n", __func__);
		return rsl_tx_mode_modif_nack(lchan, RSL_ERR_PROTO);
	}

	/* 9.3.6 Channel Mode */
	if (rsl_handle_chan_mod_ie(lchan, &tp, &cause) != 0)
		return rsl_tx_mode_modif_nack(lchan, cause);
	/* 9.3.5 Channel Identification */
	if (rsl_handle_chan_ident_ie(lchan, &tp, &cause) != 0)
		return rsl_tx_mode_modif_nack(lchan, cause);
	/* Osmocom specific TSC IE for VAMOS */
	if (rsl_handle_osmo_tsc_ie(lchan, &tp, &cause) != 0)
		return rsl_tx_mode_modif_nack(lchan, cause);

	/* 9.3.7 Encryption Information */
	if (TLVP_PRESENT(&tp, RSL_IE_ENCR_INFO)) {
		uint8_t len = TLVP_LEN(&tp, RSL_IE_ENCR_INFO);
		const uint8_t *val = TLVP_VAL(&tp, RSL_IE_ENCR_INFO);

		if (encr_info2lchan(lchan, val, len) < 0) {
			rsl_tx_error_report(msg->trx, RSL_ERR_IE_CONTENT, &dch->chan_nr, NULL, msg);
			return rsl_tx_mode_modif_nack(lchan, RSL_ERR_ENCR_UNIMPL);
		}
	}

	/* 9.3.45 Main channel reference */

	/* 9.3.52 MultiRate Configuration */
	rc = parse_multirate_config(lchan, &tp);
	if (rc < 0)
		return rsl_tx_mode_modif_nack(lchan, -rc);

	/* 9.3.53 MultiRate Control */
	/* 9.3.54 Supported Codec Types */

	rc = parse_repeated_acch_capability(lchan, &tp);
	if (rc < 0)
		return rsl_tx_mode_modif_nack(lchan, -rc);
	rc = parse_temporary_overpower_acch_capability(lchan, &tp);
	if (rc < 0)
		return rsl_tx_mode_modif_nack(lchan, -rc);

	/* Immediately disable ACCH overpower if the value is 0 dB,
	 * or enable if the RxQual threshold becomes disabled (0). */
	if (lchan->top_acch_cap.overpower_db == 0)
		lchan->top_acch_active = false;
	else if (lchan->top_acch_cap.rxqual == 0)
		lchan->top_acch_active = true;

	l1sap_chan_modify(lchan->ts->trx, dch->chan_nr);

	/* FIXME: delay this until L1 says OK? */
	rsl_tx_mode_modif_ack(lchan);

	return 0;
}

/* 8.4.15 MS POWER CONTROL */
static int rsl_rx_ms_pwr_ctrl(struct msgb *msg)
{
	struct abis_rsl_dchan_hdr *dch = msgb_l2(msg);
	struct gsm_lchan *lchan = msg->lchan;
	struct gsm_bts *bts = lchan->ts->trx->bts;
	const struct tlv_p_entry *ie;
	struct tlv_parsed tp;
	uint8_t pwr;
	int max_pwr, curr_pwr;

	if (rsl_tlv_parse(&tp, msgb_l3(msg), msgb_l3len(msg)) < 0) {
		LOGPLCHAN(lchan, DRSL, LOGL_ERROR, "%s(): rsl_tlv_parse() failed\n", __func__);
		return rsl_tx_error_report(msg->trx, RSL_ERR_PROTO, &dch->chan_nr, NULL, msg);
	}

	/* 9.3.13 MS Power (M) */
	if (!TLVP_PRES_LEN(&tp, RSL_IE_MS_POWER, 1))
		return rsl_tx_error_report(msg->trx, RSL_ERR_MAND_IE_ERROR, &dch->chan_nr, NULL, msg);

	pwr = *TLVP_VAL(&tp, RSL_IE_MS_POWER) & 0x1F;
	lchan->ms_power_ctrl.max = pwr;

	LOGPLCHAN(lchan, DRSL, LOGL_INFO, "Rx MS POWER CONTROL %" PRIu8 "\n", pwr);

	/* Spec explicitly states BTS should only perform autonomous MS Power
	 * control loop in BTS if 'MS Power Parameters' IE is present! */
	lchan->ms_power_ctrl.dpc_params = NULL;

	/* 9.3.31 (TLV) MS Power Parameters IE (vendor specific) */
	if ((ie = TLVP_GET(&tp, RSL_IE_MS_POWER_PARAM)) != NULL) {
		struct gsm_power_ctrl_params *params = &lchan->ms_dpc_params;

		/* Parsed parameters will override per-TRX defaults */
		memcpy(params, msg->trx->ms_dpc_params, sizeof(*params));

		/* Parsed parameters will override per-TRX defaults */
		if (ie->len && parse_power_ctrl_params(params, ie->val, ie->len) != 0) {
			LOGPLCHAN(lchan, DRSL, LOGL_ERROR, "Failed to parse MS Power Parameters IE\n");
			return rsl_tx_chan_act_nack(lchan, RSL_ERR_IE_CONTENT);
		}

		lchan->ms_power_ctrl.dpc_params = params;
	}

	/* Only set current to max if actual value of current
	   in dBm > value in dBm from max, or if fixed. */
	if (lchan->ms_power_ctrl.dpc_params == NULL) {
		lchan->ms_power_ctrl.current = lchan->ms_power_ctrl.max;
	} else {
		max_pwr = ms_pwr_dbm(bts->band, lchan->ms_power_ctrl.max);
		curr_pwr = ms_pwr_dbm(bts->band, lchan->ms_power_ctrl.current);
		if (max_pwr < 0 || curr_pwr < 0) {
			LOGPLCHAN(lchan, DRSL, LOGL_ERROR,
				  "Unable to calculate power levels to dBm: %" PRIu8 " -> %d, %" PRIu8 " -> %d\n",
				  lchan->ms_power_ctrl.max, max_pwr,
				  lchan->ms_power_ctrl.current, curr_pwr);
		} else if (curr_pwr > max_pwr) {
			lchan->ms_power_ctrl.current = lchan->ms_power_ctrl.max;
		}
	}

	bts_model_adjst_ms_pwr(lchan);

	return 0;
}

/* 8.4.16 BS POWER CONTROL */
static int rsl_rx_bs_pwr_ctrl(struct msgb *msg)
{
	struct abis_rsl_dchan_hdr *dch = msgb_l2(msg);
	struct gsm_lchan *lchan = msg->lchan;
	struct gsm_bts_trx *trx = msg->trx;
	const struct tlv_p_entry *ie;
	struct tlv_parsed tp;
	uint8_t old, new;

	if (rsl_tlv_parse(&tp, msgb_l3(msg), msgb_l3len(msg)) < 0) {
		LOGPLCHAN(lchan, DRSL, LOGL_ERROR, "%s(): rsl_tlv_parse() failed\n", __func__);
		return rsl_tx_error_report(trx, RSL_ERR_PROTO, &dch->chan_nr, NULL, msg);
	}

	/* 9.3.4 BS Power (M) */
	if (!TLVP_PRES_LEN(&tp, RSL_IE_BS_POWER, 1))
		return rsl_tx_error_report(trx, RSL_ERR_MAND_IE_ERROR, &dch->chan_nr, NULL, msg);

	if (*TLVP_VAL(&tp, RSL_IE_BS_POWER) & (1 << 4)) {
		LOGPLCHAN(lchan, DRSL, LOGL_NOTICE, "Fast Power Control is not supported\n");
		return rsl_tx_error_report(trx, RSL_ERR_SERV_OPT_UNIMPL, &dch->chan_nr, NULL, msg);
	}

	new = BS_POWER2DB(*TLVP_VAL(&tp, RSL_IE_BS_POWER));
	old = lchan->bs_power_ctrl.current;

	/* Osmocom specific extension for BCCH carrier power reduction */
	if (dch->chan_nr == RSL_CHAN_BCCH) {
		int rc = bts_set_c0_pwr_red(trx->bts, new);
		if (rc != 0) {
			const uint8_t cause = (rc == -ENOTSUP) ?
				RSL_ERR_SERV_OPT_UNIMPL : RSL_ERR_IE_CONTENT;
			return rsl_tx_error_report(trx, cause, &dch->chan_nr, NULL, msg);
		}

		return 0;
	}

	/* BS power reduction is generally not allowed on BCCH/CCCH carrier.
	 * However, we allow it in the BCCH carrier power reduction operation.
	 * Constrain BS power value by the maximum reduction for this timeslot. */
	if (trx->bts->c0 == trx)
		new = OSMO_MIN(new, lchan->ts->c0_power_red_db);

	/* 9.3.32 (TLV) BS Power Parameters IE (vendor specific) */
	if ((ie = TLVP_GET(&tp, RSL_IE_BS_POWER_PARAM)) != NULL) {
		struct gsm_power_ctrl_params *params = &lchan->bs_dpc_params;

		/* Parsed parameters will override per-TRX defaults */
		memcpy(params, trx->bs_dpc_params, sizeof(*params));

		/* Parsed parameters will override per-TRX defaults */
		if (ie->len && parse_power_ctrl_params(params, ie->val, ie->len) != 0) {
			LOGPLCHAN(lchan, DRSL, LOGL_ERROR, "Failed to parse BS Power Parameters IE\n");
			return rsl_tx_chan_act_nack(lchan, RSL_ERR_IE_CONTENT);
		}

		/* NOTE: it's safer to start from 0 */
		lchan->bs_power_ctrl.current = 0;
		lchan->bs_power_ctrl.max = new;
		lchan->bs_power_ctrl.dpc_params = params;
	} else {
		lchan->bs_power_ctrl.dpc_params = NULL;
		lchan->bs_power_ctrl.current = new;
	}

	if (lchan->bs_power_ctrl.current != old) {
		LOGPLCHAN(lchan, DRSL, LOGL_INFO, "BS POWER CONTROL: "
			  "attenuation change %u -> %u dB\n",
			  old, lchan->bs_power_ctrl.current);
	}

	return 0;
}


/* 8.4.20 SACCH INFO MODify */
static int rsl_rx_sacch_inf_mod(struct msgb *msg)
{
	struct abis_rsl_dchan_hdr *dch = msgb_l2(msg);
	struct gsm_lchan *lchan = msg->lchan;
	struct gsm_bts *bts = lchan->ts->trx->bts;
	struct tlv_parsed tp;
	uint8_t rsl_si, osmo_si;

	if (rsl_tlv_parse(&tp, msgb_l3(msg), msgb_l3len(msg)) < 0) {
		LOGPLCHAN(lchan, DRSL, LOGL_ERROR, "%s(): rsl_tlv_parse() failed\n", __func__);
		return rsl_tx_error_report(msg->trx, RSL_ERR_PROTO, &dch->chan_nr, NULL, msg);
	}

	if (TLVP_PRESENT(&tp, RSL_IE_STARTNG_TIME)) {
		LOGPLCHAN(lchan, DRSL, LOGL_NOTICE, "Starting time not supported\n");
		return rsl_tx_error_report(msg->trx, RSL_ERR_SERV_OPT_UNIMPL, &dch->chan_nr, NULL, msg);
	}

	/* 9.3.30 System Info Type */
	if (!TLVP_PRESENT(&tp, RSL_IE_SYSINFO_TYPE))
		return rsl_tx_error_report(msg->trx, RSL_ERR_MAND_IE_ERROR, &dch->chan_nr, NULL, msg);

	rsl_si = *TLVP_VAL(&tp, RSL_IE_SYSINFO_TYPE);
	if (!OSMO_IN_ARRAY(rsl_si, rsl_sacch_sitypes))
		return rsl_tx_error_report(msg->trx, RSL_ERR_IE_CONTENT, &dch->chan_nr, NULL, msg);

	osmo_si = osmo_rsl2sitype(rsl_si);
	if (osmo_si == SYSINFO_TYPE_NONE) {
		LOGPLCHAN(lchan, DRSL, LOGL_NOTICE, "Rx SACCH SI 0x%02x not supported.\n", rsl_si);
		return rsl_tx_error_report(msg->trx, RSL_ERR_IE_CONTENT, &dch->chan_nr, NULL, msg);
	}
	if (TLVP_PRESENT(&tp, RSL_IE_L3_INFO)) {
		uint16_t len = TLVP_LEN(&tp, RSL_IE_L3_INFO);

		lapdm_ui_prefix_lchan(lchan, TLVP_VAL(&tp, RSL_IE_L3_INFO), osmo_si, len);
		if (memcmp(GSM_BTS_SI(bts, osmo_si), TLVP_VAL(&tp, RSL_IE_L3_INFO), sizeof(sysinfo_buf_t)) != 0)
			lchan->si.overridden |= (1 << osmo_si);
		else
			lchan->si.overridden &= ~(1 << osmo_si);

		LOGPLCHAN(lchan, DRSL, LOGL_INFO, "Rx RSL SACCH FILLING (SI%s)\n",
			  get_value_string(osmo_sitype_strs, osmo_si));
	} else {
		lchan->si.valid &= ~(1 << osmo_si);
		LOGPLCHAN(lchan, DRSL, LOGL_INFO, "Rx RSL Disabling SACCH FILLING (SI%s)\n",
			  get_value_string(osmo_sitype_strs, osmo_si));
	}

	return 0;
}

/* 8.5.8 CBCH Load Information */
int rsl_tx_cbch_load_indication(struct gsm_bts *bts, bool ext_cbch, bool overflow, uint8_t amount)
{
	struct gsm_lchan *lchan;
	struct msgb *msg;
	uint8_t load_info;

	lchan = gsm_bts_get_cbch(bts);
	if (!lchan)
		return -ENODEV;

	msg = rsl_msgb_alloc(sizeof(struct abis_rsl_cchan_hdr));
	if (!msg)
		return -ENOMEM;

	/* 9.3.1 Channel Number */
	rsl_cch_push_hdr(msg, RSL_MT_CBCH_LOAD_IND, gsm_lchan2chan_nr_rsl(lchan));

	/* 9.3.43 CBCH Load Information */
	load_info = ((overflow & 1) << 7) | (amount & 0x0F);
	msgb_tv_put(msg, RSL_IE_CBCH_LOAD_INFO, load_info);
	/* 9.3.44 SMSCB Channel Indicator */
	if (ext_cbch)
		msgb_tv_put(msg, RSL_IE_SMSCB_CHAN_INDICATOR, 0x01);

	msg->trx = bts->c0;

	return abis_bts_rsl_sendmsg(msg);
}

/*
 * ip.access related messages
 */
static void rsl_add_rtp_stats(struct gsm_lchan *lchan, struct msgb *msg)
{
	uint32_t packets_sent, octets_sent;
	uint32_t packets_recv, octets_recv;
	uint32_t packets_lost;
	uint32_t arrival_jitter;

	msgb_tv_put(msg, RSL_IE_IPAC_CONN_STAT, sizeof(uint32_t) * 7);

	if (lchan->abis_ip.rtp_socket) {
		osmo_rtp_socket_stats(lchan->abis_ip.rtp_socket,
				      &packets_sent, &octets_sent,
				      &packets_recv, &octets_recv,
				      &packets_lost, &arrival_jitter);

		/* msgb_put_u32() uses osmo_store32be(),
		 * so we don't need to call htonl(). */
		msgb_put_u32(msg, packets_sent);
		msgb_put_u32(msg, octets_sent);
		msgb_put_u32(msg, packets_recv);
		msgb_put_u32(msg, octets_recv);
		msgb_put_u32(msg, packets_lost);
		msgb_put_u32(msg, arrival_jitter);
		/* FIXME: AVG Tx delay is always 0 */
		msgb_put_u32(msg, 0);
	} else {
		msgb_put(msg, sizeof(uint32_t) * 7);
		memset(msg->tail, 0x00, sizeof(uint32_t) * 7);
	}
}

int rsl_tx_ipac_dlcx_ind(struct gsm_lchan *lchan, uint8_t cause)
{
	struct msgb *nmsg;

	LOGPLCHAN(lchan, DRSL,
		  (cause == RSL_ERR_NORMAL_UNSPEC) ? LOGL_INFO : LOGL_NOTICE,
		  "Sending RTP delete indication: cause = %s\n",
		  rsl_err_name(cause));

	nmsg = rsl_msgb_alloc(sizeof(struct abis_rsl_dchan_hdr));
	if (!nmsg)
		return -ENOMEM;

	msgb_tv16_put(nmsg, RSL_IE_IPAC_CONN_ID, htons(lchan->abis_ip.conn_id));
	rsl_add_rtp_stats(lchan, nmsg);
	msgb_tlv_put(nmsg, RSL_IE_CAUSE, 1, &cause);
	rsl_ipa_push_hdr(nmsg, RSL_MT_IPAC_DLCX_IND, gsm_lchan2chan_nr_rsl(lchan));

	nmsg->trx = lchan->ts->trx;

	return abis_bts_rsl_sendmsg(nmsg);
}

/* transmit an CRCX ACK for the lchan */
static int rsl_tx_ipac_XXcx_ack(struct gsm_lchan *lchan, int inc_pt2,
				  uint8_t orig_msgt)
{
	struct msgb *msg;
	uint8_t chan_nr = gsm_lchan2chan_nr_rsl(lchan);
	const char *name;
	struct in_addr ia;

	if (orig_msgt == RSL_MT_IPAC_CRCX)
		name = "CRCX";
	else
		name = "MDCX";

	ia.s_addr = htonl(lchan->abis_ip.bound_ip);
	LOGPLCHAN(lchan, DRSL, LOGL_INFO, "RSL Tx IPAC_%s_ACK (local %s:%u, ",
		  name, inet_ntoa(ia), lchan->abis_ip.bound_port);
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

	/* Osmocom Extension: Osmux CID */
	if (lchan->abis_ip.osmux.use)
		msgb_tlv_put(msg, RSL_IE_OSMO_OSMUX_CID, 1,
			     &lchan->abis_ip.osmux.local_cid);

	/* push the header in front */
	rsl_ipa_push_hdr(msg, orig_msgt + 1, chan_nr);
	msg->trx = lchan->ts->trx;

	return abis_bts_rsl_sendmsg(msg);
}

static int rsl_tx_ipac_dlcx_ack(struct gsm_lchan *lchan, int inc_conn_id)
{
	struct msgb *msg;
	uint8_t chan_nr = gsm_lchan2chan_nr_rsl(lchan);

	LOGPLCHAN(lchan, DRSL, LOGL_INFO, "RSL Tx IPAC_DLCX_ACK\n");

	msg = rsl_msgb_alloc(sizeof(struct abis_rsl_dchan_hdr));
	if (!msg)
		return -ENOMEM;

	if (inc_conn_id) {
		msgb_tv16_put(msg, RSL_IE_IPAC_CONN_ID, lchan->abis_ip.conn_id);
		rsl_add_rtp_stats(lchan, msg);
	}

	rsl_ipa_push_hdr(msg, RSL_MT_IPAC_DLCX_ACK, chan_nr);
	msg->trx = lchan->ts->trx;

	return abis_bts_rsl_sendmsg(msg);
}

static int rsl_tx_ipac_dlcx_nack(struct gsm_lchan *lchan, int inc_conn_id,
				 uint8_t cause)
{
	struct msgb *msg;
	uint8_t chan_nr = gsm_lchan2chan_nr_rsl(lchan);

	LOGPLCHAN(lchan, DRSL, LOGL_INFO, "RSL Tx IPAC_DLCX_NACK\n");

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


/* Send an xxCX NACK for the given xxCX message type and lchan */
static int tx_ipac_XXcx_nack(struct gsm_lchan *lchan, uint8_t cause,
			     int inc_ipport, uint8_t orig_msgtype)
{
	struct msgb *msg;
	uint8_t chan_nr = gsm_lchan2chan_nr_rsl(lchan);
	uint8_t msg_type = orig_msgtype + 2;

	LOGPLCHAN(lchan, DRSL, LOGL_NOTICE, "RSL Tx %s\n", rsl_ipac_msg_name(msg_type));

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
	rsl_ipa_push_hdr(msg, msg_type, chan_nr);
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
	struct gsm_bts *bts = lchan->ts->trx->bts;
	const uint8_t *payload_type, *speech_mode, *payload_type2, *osmux_cid, *csd_fmt;
	uint32_t connect_ip = 0;
	uint16_t connect_port = 0;
	int rc, inc_ip_port = 0;
	char *name;
	struct in_addr ia;
	enum rsl_ipac_rtp_csd_format_d csd_fmt_d;
	enum rsl_ipac_rtp_csd_format_ir csd_fmt_ir;

	if (dch->c.msg_type == RSL_MT_IPAC_CRCX)
		name = "CRCX";
	else
		name = "MDCX";

	/* check the kind of channel and reject */
	if (lchan->type != GSM_LCHAN_TCH_F && lchan->type != GSM_LCHAN_TCH_H)
		return tx_ipac_XXcx_nack(lchan, 0x52,
					 0, dch->c.msg_type);

	if (rsl_tlv_parse(&tp, msgb_l3(msg), msgb_l3len(msg)) < 0) {
		LOGPLCHAN(lchan, DRSL, LOGL_ERROR, "%s(): rsl_tlv_parse() failed\n", __func__);
		return tx_ipac_XXcx_nack(lchan, RSL_ERR_PROTO, 0, dch->c.msg_type);
	}

	LOGPLCHAN(lchan, DRSL, LOGL_DEBUG, "IPAC_%s: ", name);
	if (TLVP_PRES_LEN(&tp, RSL_IE_IPAC_REMOTE_IP, 4)) {
		struct in_addr addr;
		connect_ip = tlvp_val32_unal(&tp, RSL_IE_IPAC_REMOTE_IP);
		addr.s_addr = connect_ip;
		LOGPC(DRSL, LOGL_DEBUG, "connect_ip=%s ", inet_ntoa(addr));
	}

	if (TLVP_PRES_LEN(&tp, RSL_IE_IPAC_REMOTE_PORT, 2)) {
		connect_port = tlvp_val16be(&tp, RSL_IE_IPAC_REMOTE_PORT);
		LOGPC(DRSL, LOGL_DEBUG, "connect_port=%u ", connect_port);
	}

	speech_mode = TLVP_VAL(&tp, RSL_IE_IPAC_SPEECH_MODE);
	if (speech_mode)
		LOGPC(DRSL, LOGL_DEBUG, "speech_mode=%u ", *speech_mode);

	payload_type = TLVP_VAL(&tp, RSL_IE_IPAC_RTP_PAYLOAD);
	if (payload_type)
		LOGPC(DRSL, LOGL_DEBUG, "payload_type=%u ", *payload_type);

	LOGPC(DRSL, LOGL_DEBUG, "\n");

	payload_type2 = TLVP_VAL(&tp, RSL_IE_IPAC_RTP_PAYLOAD2);
	if (payload_type2)
		LOGPC(DRSL, LOGL_DEBUG, "payload_type2=%u ", *payload_type2);

	osmux_cid = TLVP_VAL(&tp, RSL_IE_OSMO_OSMUX_CID);
	if (osmux_cid)
		LOGPC(DRSL, LOGL_DEBUG, "osmux_cid=%u ", *osmux_cid);

	if (dch->c.msg_type == RSL_MT_IPAC_CRCX && connect_ip && connect_port)
		inc_ip_port = 1;

	if (payload_type && payload_type2) {
		LOGPLCHAN(lchan, DRSL, LOGL_ERROR, "Rx RSL IPAC %s, "
			"RTP_PT and RTP_PT2 in same msg !?!\n", name);
		return tx_ipac_XXcx_nack(lchan, RSL_ERR_MAND_IE_ERROR,
					 inc_ip_port, dch->c.msg_type);
	}

	if ((csd_fmt = TLVP_VAL(&tp, RSL_IE_IPAC_RTP_CSD_FMT))) {
		csd_fmt_d = *csd_fmt & 0xf;
		csd_fmt_ir = *csd_fmt >> 4;
		LOGPC(DRSL, LOGL_DEBUG, "csd_fmt_d=%d csd_fmt_ir=%d ", csd_fmt_d, csd_fmt_ir);
		if (csd_fmt_d != RSL_IPAC_RTP_CSD_TRAU_BTS) {
			LOGPLCHAN(lchan, DRSL, LOGL_ERROR, "Rx RSL IPAC %s, csd_fmt_d=%d is not supported\n",
				  name, csd_fmt_d);
			return tx_ipac_XXcx_nack(lchan, RSL_ERR_SERV_OPT_UNIMPL, inc_ip_port, dch->c.msg_type);
		}
	}

	if (!osmux_cid) { /* Regular RTP */
		if (bts->osmux.use == OSMUX_USAGE_ONLY) {
			LOGPLCHAN(lchan, DRSL, LOGL_ERROR, "Rx RSL IPAC XXcx without Osmux CID"
				  "goes against configured Osmux policy 'only'\n");
			return tx_ipac_XXcx_nack(lchan, RSL_ERR_RES_UNAVAIL,
						 inc_ip_port, dch->c.msg_type);
		}

		if (dch->c.msg_type == RSL_MT_IPAC_CRCX) { /* CRCX */
			char *ipstr = NULL;
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
			rc = lchan_rtp_socket_create(lchan, ipstr);
			if (rc < 0)
				return tx_ipac_XXcx_nack(lchan, RSL_ERR_RES_UNAVAIL,
							 inc_ip_port, dch->c.msg_type);
		} else { /* MDCX */
			if (!lchan->abis_ip.rtp_socket) {
				LOGPLCHAN(lchan, DRSL, LOGL_ERROR, "Rx RSL IPAC MDCX, "
					  "but we have no RTP socket!\n");
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
		rc = lchan_rtp_socket_connect(lchan, &ia, connect_port);
		if (rc < 0) {
			lchan_rtp_socket_free(lchan);
			return tx_ipac_XXcx_nack(lchan, RSL_ERR_RES_UNAVAIL,
						 inc_ip_port, dch->c.msg_type);
		}

	} else { /* Osmux */
		if (bts->osmux.use == OSMUX_USAGE_OFF) {
			LOGPLCHAN(lchan, DRSL, LOGL_ERROR, "Rx RSL IPAC XXcx with Osmux CID"
				  "goes against configured Osmux policy 'off'\n");
			return tx_ipac_XXcx_nack(lchan, RSL_ERR_RES_UNAVAIL,
						 inc_ip_port, dch->c.msg_type);
		}

		if (dch->c.msg_type == RSL_MT_IPAC_CRCX) { /* CRCX */
			rc = lchan_osmux_init(lchan, payload_type ? *payload_type : 0);
			if (rc < 0)
				return tx_ipac_XXcx_nack(lchan, RSL_ERR_RES_UNAVAIL,
							 inc_ip_port, dch->c.msg_type);
		} else {  /* MDCX */
			if (!lchan->abis_ip.osmux.use) {
				LOGPLCHAN(lchan, DRSL, LOGL_ERROR, "Rx RSL IPAC MDCX with Osmux CID, "
					  "CRCX was configured as RTP!\n");
				return tx_ipac_XXcx_nack(lchan, RSL_ERR_RES_UNAVAIL,
							 inc_ip_port, dch->c.msg_type);
			}
		}

		if (connect_ip != 0)
			lchan->abis_ip.connect_ip = connect_ip;
		if (connect_port != 0)
			lchan->abis_ip.connect_port = connect_port;
		lchan->abis_ip.osmux.remote_cid = *osmux_cid;
		if (lchan->abis_ip.connect_ip && lchan->abis_ip.connect_port &&
		    !lchan_osmux_connected(lchan)) {
			rc = lchan_osmux_connect(lchan);
			if (rc < 0) {
				lchan_osmux_release(lchan);
				return tx_ipac_XXcx_nack(lchan, RSL_ERR_RES_UNAVAIL,
							 inc_ip_port, dch->c.msg_type);
			}
		}
	}

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

	if (rsl_tlv_parse(&tp, msgb_l3(msg), msgb_l3len(msg)) < 0) {
		LOGPLCHAN(lchan, DRSL, LOGL_ERROR, "%s(): rsl_tlv_parse() failed\n", __func__);
		return rsl_tx_ipac_dlcx_nack(lchan, 0, RSL_ERR_PROTO);
	}

	if (TLVP_PRESENT(&tp, RSL_IE_IPAC_CONN_ID))
		inc_conn_id = 1;

	rc = rsl_tx_ipac_dlcx_ack(lchan, inc_conn_id);
	if (lchan->abis_ip.rtp_socket) {
		osmo_rtp_socket_log_stats(lchan->abis_ip.rtp_socket, DRTP, LOGL_INFO,
					  "Closing RTP socket on DLCX ");
		lchan_rtp_socket_free(lchan);
	}
	return rc;
}

/*
 * dynamic TCH/F_PDCH related messages, originally ip.access specific but
 * reused for other BTS models (sysmo-bts, ...)
 */

/* PDCH ACT/DEACT ACKNOWLEDGE */
static int rsl_tx_dyn_pdch_ack(struct gsm_lchan *lchan, bool pdch_act)
{
	struct gsm_time *gtime = get_time(lchan->ts->trx->bts);
	uint8_t chan_nr = gsm_lchan2chan_nr_rsl(lchan);
	struct msgb *msg;
	uint8_t ie[2];

	LOGPLCHAN(lchan, DRSL, LOGL_NOTICE, "Tx PDCH %s ACK\n", pdch_act? "ACT" : "DEACT");

	msg = rsl_msgb_alloc(sizeof(struct abis_rsl_dchan_hdr));
	if (!msg)
		return -ENOMEM;

	if (pdch_act) {
		gsm48_gen_starting_time(ie, gtime);
		msgb_tv_fixed_put(msg, RSL_IE_FRAME_NUMBER, 2, ie);
	}
	rsl_dch_push_hdr(msg,
			 pdch_act? RSL_MT_IPAC_PDCH_ACT_ACK
				 : RSL_MT_IPAC_PDCH_DEACT_ACK,
			 chan_nr);
	msg->lchan = lchan;
	msg->trx = lchan->ts->trx;

	return abis_bts_rsl_sendmsg(msg);
}

/* PDCH ACT/DEACT NEGATIVE ACKNOWLEDGE */
static int rsl_tx_dyn_pdch_nack(struct gsm_lchan *lchan, bool pdch_act,
				uint8_t cause)
{
	struct msgb *msg;
	uint8_t chan_nr = gsm_lchan2chan_nr_rsl(lchan);

	LOGPLCHAN(lchan, DRSL, LOGL_NOTICE, "Tx PDCH %s NACK (cause = 0x%02x)\n",
		  pdch_act ? "ACT" : "DEACT", cause);

	msg = rsl_msgb_alloc(sizeof(struct abis_rsl_dchan_hdr));
	if (!msg)
		return -ENOMEM;

	msg->len = 0;
	msg->data = msg->tail = msg->l3h;

	/* 9.3.26 Cause */
	msgb_tlv_put(msg, RSL_IE_CAUSE, 1, &cause);
	rsl_dch_push_hdr(msg,
			 pdch_act? RSL_MT_IPAC_PDCH_ACT_NACK
				 : RSL_MT_IPAC_PDCH_DEACT_NACK,
			 chan_nr);
	msg->lchan = lchan;
	msg->trx = lchan->ts->trx;

	return abis_bts_rsl_sendmsg(msg);
}

/*
 * Starting point for dynamic PDCH switching. See osmo-gsm-manuals.git for a
 * diagram of what will happen here. The implementation is as follows:
 *
 * PDCH ACT == TCH/F -> PDCH:
 * 1. call bts_model_ts_disconnect() to disconnect TCH/F;
 * 2. cb_ts_disconnected() is called when done;
 * 3. call bts_model_ts_connect() to connect as PDTCH;
 * 4. cb_ts_connected(rc) is called when done;
 * 5. instruct the PCU to enable PDTCH;
 * 6. the PCU will call back with an activation request;
 * 7. l1sap_info_act_cnf() will call ipacc_dyn_pdch_complete() when SAPI
 *    activations are done;
 * 8. send a PDCH ACT ACK.
 *
 * PDCH DEACT == PDCH -> TCH/F:
 * 1. instruct the PCU to disable PDTCH;
 * 2. the PCU will call back with a deactivation request;
 * 3. l1sap_info_rel_cnf() will call bts_model_ts_disconnect() when SAPI
 *    deactivations are done;
 * 4. cb_ts_disconnected() is called when done;
 * 5. call bts_model_ts_connect() to connect as TCH/F;
 * 6. cb_ts_connected(rc) is called when done;
 * 7. directly call ipacc_dyn_pdch_complete(), since no further action required
 *    for TCH/F;
 * 8. send a PDCH DEACT ACK.
 *
 * When an error happens along the way, a PDCH DE/ACT NACK is sent.
 * TODO: may need to be made more waterproof in all stages, to send a NACK and
 * clear the PDCH pending flags from ts->flags.
 */
static void rsl_rx_dyn_pdch(struct msgb *msg, bool pdch_act)
{
	int rc;
	struct gsm_lchan *lchan = msg->lchan;
	struct gsm_bts_trx_ts *ts = lchan->ts;
	bool is_pdch_act = (ts->flags & TS_F_PDCH_ACTIVE);

	if (ts->flags & TS_F_PDCH_PENDING_MASK) {
		/* Only one of the pending flags should ever be set at the same
		 * time, but just log both in case both should be set. */
		LOGPLCHAN(lchan, DRSL, LOGL_ERROR,
			  "Request to PDCH %s, but PDCH%s%s is still pending\n",
			  pdch_act? "ACT" : "DEACT",
			  (ts->flags & TS_F_PDCH_ACT_PENDING)? " ACT" : "",
			  (ts->flags & TS_F_PDCH_DEACT_PENDING)? " DEACT" : "");
		rsl_tx_dyn_pdch_nack(lchan, pdch_act, RSL_ERR_NORMAL_UNSPEC);
		return;
	}

	if (lchan->state != LCHAN_S_NONE) {
		LOGP(DRSL, LOGL_NOTICE,
		     "%s Request to PDCH %s, but lchan is still in state %s\n",
		     gsm_ts_and_pchan_name(ts), pdch_act? "ACT" : "DEACT",
		     gsm_lchans_name(lchan->state));
		/* TCH takes preference over PDCH so allow forcing PDCH DEACT,
		 * but forbid forcing PDCH ACT if lchan still active */
		if (pdch_act) {
			rsl_tx_dyn_pdch_nack(lchan, pdch_act, RSL_ERR_NORMAL_UNSPEC);
			return;
		}
	}

	ts->flags |= pdch_act? TS_F_PDCH_ACT_PENDING
			     : TS_F_PDCH_DEACT_PENDING;

	/* ensure that this is indeed a dynamic-PDCH channel */
	if (ts->pchan != GSM_PCHAN_TCH_F_PDCH) {
		LOGPLCHAN(lchan, DRSL, LOGL_ERROR,
			  "Attempt to PDCH %s on TS that is not a TCH/F_PDCH (is %s)\n",
			  pdch_act? "ACT" : "DEACT", gsm_pchan_name(ts->pchan));
		ipacc_dyn_pdch_complete(ts, -EINVAL);
		return;
	}

	if (is_pdch_act == pdch_act) {
		LOGPLCHAN(lchan, DL1C, LOGL_NOTICE, "Request to PDCH %s, but is already so\n",
			  pdch_act? "ACT" : "DEACT");
		ipacc_dyn_pdch_complete(ts, 0);
		return;
	}

	if (pdch_act) {
		/* Clear TCH state. Only first lchan matters for PDCH */
		clear_lchan_for_pdch_activ(ts->lchan);
		/* First, disconnect the TCH channel, to connect PDTCH later */
		rc = bts_model_ts_disconnect(ts);
	} else {
		/* First, deactivate PDTCH through the PCU, to connect TCH
		 * later.
		 * pcu_tx_info_ind() will pick up TS_F_PDCH_DEACT_PENDING and
		 * trigger a deactivation.
		 * Except when the PCU is not connected yet, then trigger
		 * disconnect immediately from here. The PCU will catch up when
		 * it connects. */
		/* TODO: timeout on channel connect / disconnect request from PCU? */
		if (pcu_connected())
			rc = pcu_tx_info_ind();
		else
			rc = bts_model_ts_disconnect(ts);
	}

	/* Error? then NACK right now. */
	if (rc)
		ipacc_dyn_pdch_complete(ts, rc);
}

static void ipacc_dyn_pdch_ts_disconnected(struct gsm_bts_trx_ts *ts)
{
	int rc;
	enum gsm_phys_chan_config as_pchan;

	if (ts->flags & TS_F_PDCH_DEACT_PENDING) {
		LOGPLCHAN(ts->lchan, DRSL, LOGL_DEBUG,
			  "PDCH DEACT operation: channel disconnected, will reconnect as TCH\n");
		as_pchan = GSM_PCHAN_TCH_F;
	} else if (ts->flags & TS_F_PDCH_ACT_PENDING) {
		LOGPLCHAN(ts->lchan, DRSL, LOGL_DEBUG,
		     "PDCH ACT operation: channel disconnected, will reconnect as PDTCH\n");
		as_pchan = GSM_PCHAN_PDCH;
	} else
		/* No reconnect pending. */
		return;

	rc = conf_lchans_as_pchan(ts, as_pchan);
	if (rc)
		goto error_nack;

	bts_model_ts_connect(ts, as_pchan);
	return;

error_nack:
	/* Error? then NACK right now. */
	if (rc)
		ipacc_dyn_pdch_complete(ts, rc);
}

static void osmo_dyn_ts_disconnected(struct gsm_bts_trx_ts *ts)
{
	DEBUGP(DRSL, "%s Disconnected\n", gsm_ts_and_pchan_name(ts));
	ts->dyn.pchan_is = GSM_PCHAN_NONE;

	switch (ts->dyn.pchan_want) {
	case GSM_PCHAN_TCH_F:
	case GSM_PCHAN_TCH_H:
	case GSM_PCHAN_SDCCH8_SACCH8C:
	case GSM_PCHAN_PDCH:
		break;
	default:
		LOGP(DRSL, LOGL_ERROR,
		     "%s Dyn TS disconnected, but invalid desired pchan: %s\n",
		     gsm_ts_and_pchan_name(ts), gsm_pchan_name(ts->dyn.pchan_want));
		ts->dyn.pchan_want = GSM_PCHAN_NONE;
		/* TODO: how would this recover? */
		return;
	}

	conf_lchans_as_pchan(ts, ts->dyn.pchan_want);
	DEBUGP(DRSL, "%s Connect\n", gsm_ts_and_pchan_name(ts));
	bts_model_ts_connect(ts, ts->dyn.pchan_want);
}

void cb_ts_disconnected(struct gsm_bts_trx_ts *ts)
{
	OSMO_ASSERT(ts);

	switch (ts->pchan) {
	case GSM_PCHAN_TCH_F_PDCH:
		return ipacc_dyn_pdch_ts_disconnected(ts);
	case GSM_PCHAN_OSMO_DYN:
		return osmo_dyn_ts_disconnected(ts);
	default:
		return;
	}
}

static void ipacc_dyn_pdch_ts_connected(struct gsm_bts_trx_ts *ts, int rc)
{
	if (rc) {
		LOGPLCHAN(ts->lchan, DRSL, LOGL_NOTICE, "PDCH ACT IPA operation failed (%d) in bts model\n", rc);
		ipacc_dyn_pdch_complete(ts, rc);
		return;
	}

	if (ts->flags & TS_F_PDCH_DEACT_PENDING) {
		if (ts->lchan[0].type != GSM_LCHAN_TCH_F) {
			LOGPLCHAN(ts->lchan, DRSL, LOGL_ERROR, "PDCH DEACT error: timeslot connected, so "
				  "expecting lchan type TCH/F, but is %s\n", gsm_lchant_name(ts->lchan[0].type));
		}

		LOGPLCHAN(ts->lchan, DRSL, LOGL_DEBUG, "PDCH DEACT operation: timeslot connected as TCH/F\n");

		/* During PDCH DEACT, we're done right after the TCH/F came
		 * back up. */
		ipacc_dyn_pdch_complete(ts, 0);

	} else if (ts->flags & TS_F_PDCH_ACT_PENDING) {
		if (ts->lchan[0].type != GSM_LCHAN_PDTCH) {
			LOGPLCHAN(ts->lchan, DRSL, LOGL_ERROR, "PDCH ACT error: timeslot connected, "
				  "so expecting lchan type PDTCH, but is %s\n",
			     gsm_lchant_name(ts->lchan[0].type));
		}

		LOGPLCHAN(ts->lchan, DRSL, LOGL_DEBUG, "PDCH ACT operation: timeslot connected as PDTCH\n");

		/* The PDTCH is connected, now tell the PCU about it. Except
		 * when the PCU is not connected (yet), then there's nothing
		 * left to do now. The PCU will catch up when it connects. */
		if (!pcu_connected()) {
			ipacc_dyn_pdch_complete(ts, 0);
			return;
		}

		/* The PCU will request to activate the PDTCH SAPIs, which,
		 * when done, will call back to ipacc_dyn_pdch_complete(). */
		/* TODO: timeout on channel connect / disconnect request from PCU? */
		rc = pcu_tx_info_ind();

		/* Error? then NACK right now. */
		if (rc)
			ipacc_dyn_pdch_complete(ts, rc);
	}
}

static void osmo_dyn_ts_connected(struct gsm_bts_trx_ts *ts, int rc)
{
	unsigned int ln;

	if (rc) {
		LOGPLCHAN(ts->lchan, DRSL, LOGL_NOTICE, "PDCH ACT OSMO operation failed (%d) in bts model\n", rc);
		ipacc_dyn_pdch_complete(ts, rc);
		return;
	}

	ts->dyn.pchan_is = ts->dyn.pchan_want;
	DEBUGP(DRSL, "%s Connected\n", gsm_ts_and_pchan_name(ts));

	/* Handle postponed RSL CHANnel ACTIVation messages (if any) */
	for (ln = 0; ln < ARRAY_SIZE(ts->lchan); ln++) {
		struct gsm_lchan *lchan = &ts->lchan[ln];

		if (lchan->pending_chan_activ == NULL)
			continue;

		struct msgb *msg = lchan->pending_chan_activ;
		lchan->pending_chan_activ = NULL;

		/* Continue where we left off before re-connecting the TS */
		if (rsl_rx_chan_activ(msg) != 1)
			msgb_free(msg);
	}
}

void cb_ts_connected(struct gsm_bts_trx_ts *ts, int rc)
{
	OSMO_ASSERT(ts);

	switch (ts->pchan) {
	case GSM_PCHAN_TCH_F_PDCH:
		return ipacc_dyn_pdch_ts_connected(ts, rc);
	case GSM_PCHAN_OSMO_DYN:
		return osmo_dyn_ts_connected(ts, rc);
	default:
		return;
	}
}

void ipacc_dyn_pdch_complete(struct gsm_bts_trx_ts *ts, int rc)
{
	bool pdch_act;
	OSMO_ASSERT(ts);

	pdch_act = ts->flags & TS_F_PDCH_ACT_PENDING;

	if ((ts->flags & TS_F_PDCH_PENDING_MASK) == TS_F_PDCH_PENDING_MASK) {
		LOGPLCHAN(ts->lchan, DRSL, LOGL_ERROR,
			  "Internal Error: both PDCH ACT and PDCH DEACT pending\n");
	}

	ts->flags &= ~TS_F_PDCH_PENDING_MASK;

	if (rc != 0) {
		LOGP(DRSL, LOGL_ERROR,
		     "PDCH %s on dynamic TCH/F_PDCH returned error %d\n",
		     pdch_act? "ACT" : "DEACT", rc);
		rsl_tx_dyn_pdch_nack(ts->lchan, pdch_act, RSL_ERR_NORMAL_UNSPEC);
		return;
	}

	if (pdch_act)
		ts->flags |= TS_F_PDCH_ACTIVE;
	else
		ts->flags &= ~TS_F_PDCH_ACTIVE;
	LOGPLCHAN(ts->lchan, DRSL, LOGL_DEBUG, "%s switched to %s mode (ts->flags == %x)\n",
		  gsm_pchan_name(ts->pchan), pdch_act ? "PDCH" : "TCH/F", ts->flags);

	rc = rsl_tx_dyn_pdch_ack(ts->lchan, pdch_act);
	if (rc)
		LOGP(DRSL, LOGL_ERROR,
		     "Failed to transmit PDCH %s ACK, rc %d\n",
		     pdch_act? "ACT" : "DEACT", rc);
}

/* handle a message with an RSL CHAN_NR that is incompatible/unknown */
static int rsl_reject_unknown_lchan(struct msgb *msg)
{
	struct abis_rsl_common_hdr *rh = msgb_l2(msg);
	struct abis_rsl_dchan_hdr *dch;
	struct abis_rsl_cchan_hdr *cch;
	struct abis_rsl_rll_hdr *rllh;
	int rc;

	/* Handle GSM 08.58 7 Error Handling for the given input. This method will
	 * send either a CHANNEL ACTIVATION NACK, MODE MODIFY NACK or ERROR REPORT
	 * depending on the input of the method. */

	/* TS 48.058 Section 7 explains how to do error handling */
	switch (rh->msg_discr & 0xfe) {
	case ABIS_RSL_MDISC_IPACCESS:
		/* fall-through */
	case ABIS_RSL_MDISC_DED_CHAN:
		dch = msgb_l2(msg);
		switch (dch->c.msg_type) {
		case RSL_MT_CHAN_ACTIV:
			rc = _rsl_tx_chan_act_nack(msg->trx, dch->chan_nr,
						   RSL_ERR_MAND_IE_ERROR, NULL);
			break;
		case RSL_MT_MODE_MODIFY_REQ:
			rc = _rsl_tx_mode_modif_nack(msg->trx, dch->chan_nr,
						     RSL_ERR_MAND_IE_ERROR, NULL);
			break;
		default:
			rc = rsl_tx_error_report(msg->trx, RSL_ERR_MAND_IE_ERROR, &dch->chan_nr,
						 NULL, msg);
			break;
		}
		break;
	case ABIS_RSL_MDISC_RLL:
		rllh = msgb_l2(msg);
		/* ERROR REPORT */
		rc = rsl_tx_error_report(msg->trx, RSL_ERR_MAND_IE_ERROR,
					 &rllh->chan_nr, &rllh->link_id, msg);
		break;
	case ABIS_RSL_MDISC_COM_CHAN:
		cch = msgb_l2(msg);
		/* ERROR REPORT */
		rc = rsl_tx_error_report(msg->trx, RSL_ERR_MAND_IE_ERROR, &cch->chan_nr, NULL, msg);
		break;
	case ABIS_RSL_MDISC_TRX:
		/* fall-through */
	default:
		/* ERROR REPORT */
		rc = rsl_tx_error_report(msg->trx, RSL_ERR_MAND_IE_ERROR, NULL, NULL, msg);
	}

	msgb_free(msg);
	return rc;
}

/*
 * selecting message
 */

static int rsl_rx_rll(struct gsm_bts_trx *trx, struct msgb *msg)
{
	struct abis_rsl_rll_hdr *rh = msgb_l2(msg);
	struct abis_rsl_rll_hdr rh2;
	struct gsm_lchan *lchan;
	int rc;

	if (msgb_l2len(msg) < sizeof(*rh)) {
		LOGP(DRSL, LOGL_NOTICE, "RSL Radio Link Layer message too short\n");
		rsl_tx_error_report(trx, RSL_ERR_PROTO, &rh->chan_nr, &rh->link_id, msg);
		msgb_free(msg);
		return -EIO;
	}
	msg->l3h = (unsigned char *)rh + sizeof(*rh);

	if (!chan_nr_is_dchan(rh->chan_nr))
		return rsl_reject_unknown_lchan(msg);

	lchan = lchan_lookup(trx, rh->chan_nr, "RSL rx RLL: ");
	if (!lchan) {
		LOGP(DRLL, LOGL_NOTICE, "Rx RLL %s for unknown lchan\n",
		     rsl_msg_name(rh->c.msg_type));
		return rsl_reject_unknown_lchan(msg);
	}

	if (lchan->state != LCHAN_S_ACTIVE) {
		LOGPLCHAN(lchan, DRLL, LOGL_NOTICE, "Rx RLL %s for lchan which isn't active\n",
			rsl_msg_name(rh->c.msg_type));
		rsl_tx_error_report(trx, RSL_ERR_MSG_SEQ, &rh->chan_nr, &rh->link_id, msg);
		msgb_free(msg);
		return -1;
	}

	/* VGCS Uplink is released by MSC using REL-REQ. */
	if (rh->c.msg_type == RSL_MT_REL_REQ)
		vgcs_talker_reset(lchan);

	LOGPLCHAN(lchan, DRLL, LOGL_DEBUG, "Rx RLL %s Abis -> LAPDm\n", rsl_msg_name(rh->c.msg_type));

	/* make copy of RLL header, as the message will be free'd in case of erroneous return */
	rh2 = *rh;
	/* exception: RLL messages are _NOT_ freed as they are now
	 * owned by LAPDm which might have queued them */
	rc = lapdm_rslms_recvmsg(msg, &lchan->lapdm_ch);
	if (rc < 0)
		rsl_tx_error_report(trx, RSL_ERR_MSG_TYPE, &rh2.chan_nr, &rh2.link_id, NULL);
	return rc;
}

static inline int rsl_link_id_is_sacch(uint8_t link_id)
{
	if (link_id >> 6 == 1)
		return 1;
	else
		return 0;
}

static int rslms_get_rll_msg_type(struct msgb *msg)
{
	struct abis_rsl_common_hdr *rh = msgb_l2(msg);

	if ((rh->msg_discr & 0xfe) != ABIS_RSL_MDISC_RLL)
		return -1;

	return rh->msg_type;
}

static int rslms_get_rr_msg_type(struct msgb *msg, bool rllh_link_id_is_sacch)
{
	struct abis_rsl_common_hdr *rh = msgb_l2(msg);
	struct abis_rsl_rll_hdr *rllh;
	struct gsm48_hdr *gh;

	if ((rh->msg_discr & 0xfe) != ABIS_RSL_MDISC_RLL)
		return -1;


	rllh = msgb_l2(msg);
	if (rsl_link_id_is_sacch(rllh->link_id) != rllh_link_id_is_sacch)
		return -3;

	gh = msgb_l3(msg);
	if (gh->proto_discr != GSM48_PDISC_RR)
		return -4;

	return gh->msg_type;
}

static bool rslms_is_meas_rep(struct msgb *msg)
{
	int rll_msg_type = rslms_get_rll_msg_type(msg);
	int rr_msg_type;

	if (rll_msg_type != RSL_MT_UNIT_DATA_IND)
		return false;

	rr_msg_type = rslms_get_rr_msg_type(msg, 1);

	switch (rr_msg_type) {
	case GSM48_MT_RR_MEAS_REP:
	case GSM48_MT_RR_EXT_MEAS_REP:
		return true;
	}

	/* FIXME: this does not cover the Bter frame format and the associated
	 * short RR protocol descriptor for ENHANCED MEASUREMENT REPORT */

	return false;
}

static bool rslms_is_gprs_susp_req(struct msgb *msg)
{
	int rll_msg_type = rslms_get_rll_msg_type(msg);
	int rr_msg_type;

	if (rll_msg_type != RSL_MT_DATA_IND)
		return false;

	rr_msg_type = rslms_get_rr_msg_type(msg, false);

	return rr_msg_type == GSM48_MT_RR_GPRS_SUSP_REQ;
}

/* TS 44.018 9.1.13b GPRS suspension request */
static int handle_gprs_susp_req(struct msgb *msg)
{
	struct gsm48_hdr *gh = msgb_l3(msg);
	struct gsm48_gprs_susp_req *gsr;
	uint32_t tlli;
	int rc;

	if (!gh || msgb_l3len(msg) < sizeof(*gh)+sizeof(*gsr)) {
		LOGPLCHAN(msg->lchan, DRSL, LOGL_NOTICE, "Short GPRS SUSPEND REQ received, ignoring\n");
		msgb_free(msg);
		return -EINVAL;
	}

	gsr = (struct gsm48_gprs_susp_req *) gh->data;
	tlli = osmo_ntohl(gsr->tlli);

	LOGPLCHAN(msg->lchan, DRSL, LOGL_INFO, "Fwd GPRS SUSPEND REQ for TLLI=0x%08x to PCU\n", tlli);
	rc = pcu_tx_susp_req(msg->lchan, tlli, gsr->ra_id, gsr->cause);

	msgb_free(msg);

	return rc;
}

struct osmo_bts_supp_meas_info {
	int16_t toa256_mean;
	int16_t toa256_min;
	int16_t toa256_max;
	uint16_t toa256_std_dev;
} __attribute__((packed));

/* Compose and send 8.4.8 MEASUREMENT RESult via RSL. (timing_offset=-1 -> not present) */
int rsl_tx_meas_res(struct gsm_lchan *lchan, const uint8_t *l3, unsigned int l3_len, int timing_offset)
{
	struct msgb *msg;
	uint8_t meas_res[16];
	uint8_t chan_nr = gsm_lchan2chan_nr_rsl(lchan);
	int res_valid = lchan->meas.flags & LC_UL_M_F_RES_VALID;
	struct gsm_bts *bts = lchan->ts->trx->bts;

	LOGPLCHAN(lchan, DRSL, LOGL_DEBUG, "chan_num:%u Tx MEAS RES valid(%d), flags(%02x)\n",
		  chan_nr, res_valid, lchan->meas.flags);

	if (!res_valid)
		return -EINPROGRESS;

	msg = rsl_msgb_alloc(sizeof(struct abis_rsl_dchan_hdr));
	if (!msg)
		return -ENOMEM;

	LOGPLCHAN(lchan, DRSL, LOGL_DEBUG,
	     "Send Meas RES: NUM:%u, RXLEV_FULL:%u, RXLEV_SUB:%u, RXQUAL_FULL:%u, RXQUAL_SUB:%u, MS_PWR:%u, UL_TA:%u, L3_LEN:%u, TimingOff:%u\n",
	     lchan->meas.res_nr,
	     lchan->meas.ul_res.full.rx_lev,
	     lchan->meas.ul_res.sub.rx_lev,
	     lchan->meas.ul_res.full.rx_qual,
	     lchan->meas.ul_res.sub.rx_qual,
	     lchan->meas.l1_info.ms_pwr,
	     lchan->meas.l1_info.ta, l3_len, timing_offset - MEAS_MAX_TIMING_ADVANCE);

	msgb_tv_put(msg, RSL_IE_MEAS_RES_NR, lchan->meas.res_nr);
	size_t ie_len = gsm0858_rsl_ul_meas_enc(&lchan->meas.ul_res,
						lchan->tch.dtx.dl_active,
						meas_res);
	if (ie_len >= 3) {
		if (bts->supp_meas_toa256 && lchan->meas.flags & LC_UL_M_F_OSMO_EXT_VALID) {
			struct osmo_bts_supp_meas_info *smi;
			smi = (struct osmo_bts_supp_meas_info *) &meas_res[ie_len];
			ie_len += sizeof(struct osmo_bts_supp_meas_info);
			/* append signed 16bit value containing MS timing offset in 1/256th symbols
			 * in the vendor-specific "Supplementary Measurement Information" part of
			 * the uplink measurements IE.  The lchan->meas.ext members are the current
			 * offset *relative* to the TA which the MS has already applied.  As we want
			 * to know the total propagation time between MS and BTS, we need to add
			 * the actual TA value applied by the MS plus the respective toa256 value in
			 * 1/256 symbol periods. */
			int16_t ta256 = lchan->meas.l1_info.ta * 256;
			smi->toa256_mean = htons(ta256 + lchan->meas.ms_toa256);
			smi->toa256_min = htons(ta256 + lchan->meas.ext.toa256_min);
			smi->toa256_max = htons(ta256 + lchan->meas.ext.toa256_max);
			smi->toa256_std_dev = htons(lchan->meas.ext.toa256_std_dev);
		}
		msgb_tlv_put(msg, RSL_IE_UPLINK_MEAS, ie_len, meas_res);
	}
	msgb_tv_put(msg, RSL_IE_BS_POWER, lchan->bs_power_ctrl.current / 2);
	if (lchan->meas.flags & LC_UL_M_F_L1_VALID) {
		msgb_tv_fixed_put(msg, RSL_IE_L1_INFO, sizeof(lchan->meas.l1_info), (uint8_t*)&lchan->meas.l1_info);
	}

	if (l3 && l3_len > 0) {
		msgb_tl16v_put(msg, RSL_IE_L3_INFO, l3_len, l3);
		if (timing_offset != -1)
			msgb_tv_put(msg, RSL_IE_MS_TIMING_OFFSET, timing_offset);
	}

	rsl_dch_push_hdr(msg, RSL_MT_MEAS_RES, chan_nr);
	msg->trx = lchan->ts->trx;

	return abis_bts_rsl_sendmsg(msg);
}

/* call-back for LAPDm code, called when it wants to send msgs UP */
int lapdm_rll_tx_cb(struct msgb *msg, struct lapdm_entity *le, void *ctx)
{
	struct gsm_lchan *lchan = ctx;
	struct abis_rsl_common_hdr *rh;

	OSMO_ASSERT(msg);
	rh = msgb_l2(msg);

	if (lchan->state != LCHAN_S_ACTIVE) {
		LOGPLCHAN(lchan, DRSL, LOGL_ERROR, "(%s) is not active. Dropping message (len=%u): %s\n",
			  gsm_lchans_name(lchan->state), msgb_l2len(msg), msgb_hexdump_l2(msg));
		msgb_free(msg);
		return 0;
	}

	msg->trx = lchan->ts->trx;
	msg->lchan = lchan;

	/* If this is a Measurement Report, then we simply ignore it,
	 * because it has already been processed in l1sap_ph_data_ind(). */
	if (rslms_is_meas_rep(msg)) {
		msgb_free(msg);
		return 0;
	} else if (rslms_is_gprs_susp_req(msg)) {
		return handle_gprs_susp_req(msg);
	} else {
		LOGPLCHAN(lchan, DRSL, LOGL_INFO, "Fwd RLL msg %s from LAPDm to A-bis\n",
			  rsl_msg_name(rh->msg_type));

		/* REL_IND handling */
		if (rh->msg_type == RSL_MT_REL_IND && lchan_is_tch(lchan)) {
			vgcs_talker_reset(lchan);
			LOGPLCHAN(lchan, DRSL, LOGL_INFO,
				  "Scheduling %s to L3 in next associated TCH-RTS.ind\n",
				  rsl_msg_name(rh->msg_type));

			if (lchan->pending_rel_ind_msg) {
				LOGPLCHAN(lchan, DRSL, LOGL_INFO,
					  "Dropping pending release indication message\n");
				msgb_free(lchan->pending_rel_ind_msg);
			}

			lchan->pending_rel_ind_msg = msg;
			return 0;
		}

		return abis_bts_rsl_sendmsg(msg);
	}
}

static int rsl_rx_cchan(struct gsm_bts_trx *trx, struct msgb *msg)
{
	struct abis_rsl_cchan_hdr *cch = msgb_l2(msg);
	int ret = 0;

	if (msgb_l2len(msg) < sizeof(*cch)) {
		LOGP(DRSL, LOGL_NOTICE, "RSL Common Channel Management message too short\n");
		rsl_tx_error_report(trx, RSL_ERR_PROTO, NULL, NULL, msg);
		msgb_free(msg);
		return -EIO;
	}
	msg->l3h = (unsigned char *)cch + sizeof(*cch);

	/* normally we don't permit dedicated channels here ... */
	if (chan_nr_is_dchan(cch->chan_nr)) {
		/* ... however, CBCH is on a SDCCH, so we must permit it */
		if (cch->c.msg_type != RSL_MT_SMS_BC_CMD && cch->c.msg_type != RSL_MT_SMS_BC_REQ)
			return rsl_reject_unknown_lchan(msg);
	}

	msg->lchan = lchan_lookup(trx, cch->chan_nr, "RSL rx CCHAN: ");
	if (!msg->lchan) {
		LOGP(DRSL, LOGL_ERROR, "Rx RSL %s for unknown lchan\n",
			rsl_msg_name(cch->c.msg_type));
		return rsl_reject_unknown_lchan(msg);
	}

	LOGPLCHAN(msg->lchan, DRSL, LOGL_INFO, "Rx RSL %s\n", rsl_msg_name(cch->c.msg_type));

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
	case RSL_MT_SMS_BC_CMD:
		ret = rsl_rx_sms_bcast_cmd(trx, msg);
		break;
	case RSL_MT_NOT_CMD:
		ret = rsl_rx_notification_cmd(trx, msg);
		break;
	case RSL_MT_SMS_BC_REQ:
		LOGPLCHAN(msg->lchan, DRSL, LOGL_NOTICE, "unimplemented RSL cchan msg_type %s\n",
			  rsl_msg_name(cch->c.msg_type));
		rsl_tx_error_report(trx, RSL_ERR_MSG_TYPE, &cch->chan_nr, NULL, msg);
		break;
	case RSL_MT_OSMO_ETWS_CMD:
		ret = rsl_rx_osmo_etws_cmd(trx, msg);
		break;
	/* Osmocom specific extension for BCCH carrier power reduction */
	case RSL_MT_BS_POWER_CONTROL:
		ret = rsl_rx_bs_pwr_ctrl(msg);
		break;
	default:
		LOGPLCHAN(msg->lchan, DRSL, LOGL_NOTICE, "undefined RSL cchan msg_type 0x%02x\n",
			  cch->c.msg_type);
		rsl_tx_error_report(trx, RSL_ERR_MSG_TYPE, &cch->chan_nr, NULL, msg);
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

	if (!chan_nr_is_dchan(dch->chan_nr))
		return rsl_reject_unknown_lchan(msg);

	msg->lchan = lchan_lookup(trx, dch->chan_nr, "RSL rx DCHAN: ");
	if (!msg->lchan) {
		LOGP(DRSL, LOGL_ERROR, "Rx RSL %s for unknown lchan\n",
			rsl_or_ipac_msg_name(dch->c.msg_type));
		return rsl_reject_unknown_lchan(msg);
	}

	LOGP(DRSL, LOGL_INFO, "%s ss=%d Rx RSL %s\n",
	     gsm_ts_and_pchan_name(msg->lchan->ts), msg->lchan->nr,
	     rsl_or_ipac_msg_name(dch->c.msg_type));

	switch (dch->c.msg_type) {
	case RSL_MT_CHAN_ACTIV:
		ret = rsl_rx_chan_activ(msg);
		break;
	case RSL_MT_RF_CHAN_REL:
		ret = rsl_rx_rf_chan_rel(msg->lchan, dch->chan_nr);
		break;
	case RSL_MT_SACCH_INFO_MODIFY:
		ret = rsl_rx_sacch_inf_mod(msg);
		break;
	case RSL_MT_DEACTIVATE_SACCH:
		ret = l1sap_chan_deact_sacch(trx, dch->chan_nr);
		break;
	case RSL_MT_ENCR_CMD:
		ret = rsl_rx_encr_cmd(msg);
		break;
	case RSL_MT_MODE_MODIFY_REQ:
		ret = rsl_rx_mode_modif(msg);
		break;
	case RSL_MT_MS_POWER_CONTROL:
		ret = rsl_rx_ms_pwr_ctrl(msg);
		break;
	case RSL_MT_BS_POWER_CONTROL:
		ret = rsl_rx_bs_pwr_ctrl(msg);
		break;
	case RSL_MT_IPAC_PDCH_ACT:
	case RSL_MT_IPAC_PDCH_DEACT:
		rsl_rx_dyn_pdch(msg, dch->c.msg_type == RSL_MT_IPAC_PDCH_ACT);
		ret = 0;
		break;
	case RSL_MT_PHY_CONTEXT_REQ:
	case RSL_MT_PREPROC_CONFIG:
	case RSL_MT_RTD_REP:
	case RSL_MT_PRE_HANDO_NOTIF:
	case RSL_MT_MR_CODEC_MOD_REQ:
	case RSL_MT_TFO_MOD_REQ:
		LOGPLCHAN(msg->lchan, DRSL, LOGL_NOTICE, "unimplemented RSL dchan msg_type %s\n",
			  rsl_msg_name(dch->c.msg_type));
		rsl_tx_error_report(trx, RSL_ERR_MSG_TYPE, &dch->chan_nr, NULL, msg);
		break;
	default:
		LOGPLCHAN(msg->lchan, DRSL, LOGL_NOTICE, "undefined RSL dchan msg_type 0x%02x\n",
			  dch->c.msg_type);
		rsl_tx_error_report(trx, RSL_ERR_MSG_TYPE, &dch->chan_nr, NULL, msg);
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
		rsl_tx_error_report(trx, RSL_ERR_PROTO, NULL, NULL, msg);
		msgb_free(msg);
		return -EIO;
	}
	msg->l3h = (unsigned char *)th + sizeof(*th);

	switch (th->msg_type) {
	case RSL_MT_SACCH_FILL:
		ret = rsl_rx_sacch_fill(trx, msg);
		break;
	case RSL_MT_IPAC_MEAS_PREPROC_DFT:
		ret = rsl_rx_meas_preproc_dft(trx, msg);
		break;
	default:
		LOGP(DRSL, LOGL_NOTICE, "undefined RSL TRX msg_type 0x%02x\n",
			th->msg_type);
		rsl_tx_error_report(trx, RSL_ERR_MSG_TYPE, NULL, NULL, msg);
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
		rsl_tx_error_report(trx, RSL_ERR_PROTO, NULL, NULL, msg);
		msgb_free(msg);
		return -EIO;
	}
	msg->l3h = (unsigned char *)dch + sizeof(*dch);

	if (!chan_nr_is_dchan(dch->chan_nr))
		return rsl_reject_unknown_lchan(msg);

	msg->lchan = lchan_lookup(trx, dch->chan_nr, "RSL rx IPACC: ");
	if (!msg->lchan) {
		LOGP(DRSL, LOGL_ERROR, "Rx RSL %s for unknown lchan\n",
			rsl_msg_name(dch->c.msg_type));
		return rsl_reject_unknown_lchan(msg);
	}

	LOGPLCHAN(msg->lchan, DRSL, LOGL_INFO, "Rx RSL %s\n", rsl_ipac_msg_name(dch->c.msg_type));

	switch (dch->c.msg_type) {
	case RSL_MT_IPAC_CRCX:
	case RSL_MT_IPAC_MDCX:
		ret = rsl_rx_ipac_XXcx(msg);
		break;
	case RSL_MT_IPAC_DLCX:
		ret = rsl_rx_ipac_dlcx(msg);
		break;
	default:
		LOGPLCHAN(msg->lchan, DRSL, LOGL_NOTICE, "unsupported RSL ip.access msg_type 0x%02x\n",
			  dch->c.msg_type);
		rsl_tx_error_report(trx, RSL_ERR_MSG_TYPE, &dch->chan_nr, NULL, msg);
		ret = -EINVAL;
	}

	if (ret != 1)
		msgb_free(msg);
	return ret;
}

int down_rsl(struct gsm_bts_trx *trx, struct msgb *msg)
{
	struct abis_rsl_common_hdr *rslh;
	int ret = 0;

	OSMO_ASSERT(trx);
	OSMO_ASSERT(msg);

	rslh = msgb_l2(msg);

	if (msgb_l2len(msg) < sizeof(*rslh)) {
		LOGP(DRSL, LOGL_NOTICE, "RSL message too short\n");
		rsl_tx_error_report(trx, RSL_ERR_PROTO, NULL, NULL, msg);
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
		rsl_tx_error_report(trx, RSL_ERR_MSG_DISCR, NULL, NULL, msg);
		msgb_free(msg);
		ret = -EINVAL;
	}

	/* we don't free here, as rsl_rx{cchan,dchan,trx,ipaccess,rll} are
	 * responsible for owning the msg */

	return ret;
}
