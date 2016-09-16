/* Traffic channel support for NuRAN Wireless Litecell 1.5 BTS L1 */

/* Copyright (C) 2015 by Yves Godin <support@nuranwireless.com>
 * 
 * Based on sysmoBTS:
 *     (C) 2011-2012 by Harald Welte <laforge@gnumonks.org>
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

#include <osmocom/core/talloc.h>
#include <osmocom/core/utils.h>
#include <osmocom/core/select.h>
#include <osmocom/core/timer.h>
#include <osmocom/core/bits.h>
#include <osmocom/codec/codec.h>
#include <osmocom/gsm/gsm_utils.h>
#include <osmocom/trau/osmo_ortp.h>

#include <osmo-bts/logging.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/gsm_data.h>
#include <osmo-bts/msg_utils.h>
#include <osmo-bts/measurement.h>
#include <osmo-bts/amr.h>
#include <osmo-bts/l1sap.h>

#include <nrw/litecell15/litecell15.h>
#include <nrw/litecell15/gsml1prim.h>
#include <nrw/litecell15/gsml1const.h>
#include <nrw/litecell15/gsml1types.h>

#include "lc15bts.h"
#include "l1_if.h"

static struct msgb *l1_to_rtppayload_fr(uint8_t *l1_payload, uint8_t payload_len,
					struct gsm_lchan *lchan)
{
	struct msgb *msg;
	uint8_t *cur;

	msg = msgb_alloc_headroom(1024, 128, "L1C-to-RTP");
	if (!msg)
		return NULL;

	/* new L1 can deliver bits like we need them */
	cur = msgb_put(msg, GSM_FR_BYTES);
	memcpy(cur, l1_payload, GSM_FR_BYTES);

	lchan_set_marker(osmo_fr_check_sid(l1_payload, payload_len), lchan);

	return msg;
}

/*! \brief convert GSM-FR from RTP payload to L1 format
 *  \param[out] l1_payload payload part of L1 buffer
 *  \param[in] rtp_payload pointer to RTP payload data
 *  \param[in] payload_len length of \a rtp_payload
 *  \returns number of \a l1_payload bytes filled
 */
static int rtppayload_to_l1_fr(uint8_t *l1_payload, const uint8_t *rtp_payload,
				unsigned int payload_len)
{
	/* new L1 can deliver bits like we need them */
	memcpy(l1_payload, rtp_payload, GSM_FR_BYTES);
	return GSM_FR_BYTES;
}

static struct msgb *l1_to_rtppayload_efr(uint8_t *l1_payload,
					 uint8_t payload_len,
					 struct gsm_lchan *lchan)
{
	struct msgb *msg;
	uint8_t *cur;

	msg = msgb_alloc_headroom(1024, 128, "L1C-to-RTP");
	if (!msg)
		return NULL;

	/* new L1 can deliver bits like we need them */
	cur = msgb_put(msg, GSM_EFR_BYTES);
	memcpy(cur, l1_payload, GSM_EFR_BYTES);
	enum osmo_amr_type ft;
	enum osmo_amr_quality bfi;
	uint8_t cmr;
	int8_t sti, cmi;
	osmo_amr_rtp_dec(l1_payload, payload_len, &cmr, &cmi, &ft, &bfi, &sti);
	lchan_set_marker(ft == AMR_GSM_EFR_SID, lchan);

	return msg;
}

static int rtppayload_to_l1_efr(uint8_t *l1_payload, const uint8_t *rtp_payload,
				unsigned int payload_len)
{
	memcpy(l1_payload, rtp_payload, payload_len);

	return payload_len;
}

static struct msgb *l1_to_rtppayload_hr(uint8_t *l1_payload, uint8_t payload_len,
					struct gsm_lchan *lchan)
{
	struct msgb *msg;
	uint8_t *cur;

	msg = msgb_alloc_headroom(1024, 128, "L1C-to-RTP");
	if (!msg)
		return NULL;

	if (payload_len != GSM_HR_BYTES) {
		LOGP(DL1C, LOGL_ERROR, "L1 HR frame length %u != expected %u\n",
			payload_len, GSM_HR_BYTES);
		return NULL;
	}

	cur = msgb_put(msg, GSM_HR_BYTES);
	memcpy(cur, l1_payload, GSM_HR_BYTES);

	lchan_set_marker(osmo_hr_check_sid(l1_payload, payload_len), lchan);

	return msg;
}

/*! \brief convert GSM-FR from RTP payload to L1 format
 *  \param[out] l1_payload payload part of L1 buffer
 *  \param[in] rtp_payload pointer to RTP payload data
 *  \param[in] payload_len length of \a rtp_payload
 *  \returns number of \a l1_payload bytes filled
 */
static int rtppayload_to_l1_hr(uint8_t *l1_payload, const uint8_t *rtp_payload,
				unsigned int payload_len)
{

	if (payload_len != GSM_HR_BYTES) {
		LOGP(DL1C, LOGL_ERROR, "RTP HR frame length %u != expected %u\n",
			payload_len, GSM_HR_BYTES);
		return 0;
	}

	memcpy(l1_payload, rtp_payload, GSM_HR_BYTES);

	return GSM_HR_BYTES;
}

static struct msgb *l1_to_rtppayload_amr(uint8_t *l1_payload, uint8_t payload_len,
					 struct gsm_lchan *lchan)
{
	struct msgb *msg;
	uint8_t amr_if2_len = payload_len - 2;
	uint8_t *cur;

	msg = msgb_alloc_headroom(1024, 128, "L1C-to-RTP");
	if (!msg)
		return NULL;

	cur = msgb_put(msg, amr_if2_len);
	memcpy(cur, l1_payload+2, amr_if2_len);

	/*
	 * Audiocode's MGW doesn't like receiving CMRs that are not
	 * the same as the previous one. This means we need to patch
	 * the content here.
	 */
	if ((cur[0] & 0xF0) == 0xF0)
		cur[0]= lchan->tch.last_cmr << 4;
	else
		lchan->tch.last_cmr = cur[0] >> 4;

	return msg;
}

/*! \brief convert AMR from RTP payload to L1 format
 *  \param[out] l1_payload payload part of L1 buffer
 *  \param[in] rtp_payload pointer to RTP payload data
 *  \param[in] payload_len length of \a rtp_payload
 *  \returns number of \a l1_payload bytes filled
 */
static int rtppayload_to_l1_amr(uint8_t *l1_payload, const uint8_t *rtp_payload,
				uint8_t payload_len,
				struct gsm_lchan *lchan, uint32_t fn)
{
	struct amr_multirate_conf *amr_mrc = &lchan->tch.amr_mr;
	enum osmo_amr_type ft;
	enum osmo_amr_quality bfi;
	uint8_t cmr;
	int8_t sti, cmi;
	uint8_t *l1_cmi_idx = l1_payload;
	uint8_t *l1_cmr_idx = l1_payload+1;
	int rc;

	osmo_amr_rtp_dec(rtp_payload, payload_len, &cmr, &cmi, &ft, &bfi, &sti);
	memcpy(l1_payload+2, rtp_payload, payload_len);

	/* CMI in downlink tells the L1 encoder which encoding function
	 * it will use, so we have to use the frame type */
	switch (ft) {
	case 0: case 1: case 2: case 3:
	case 4: case 5: case 6: case 7:
		cmi = ft;
		LOGP(DRTP, LOGL_DEBUG, "SPEECH frame with CMI %u\n", cmi);
		break;
	case AMR_SID:
		LOGP(DRTP, LOGL_DEBUG, "SID %s frame with CMI %u\n",
		     sti ? "UPDATE" : "FIRST", cmi);
		break;
	default:
		LOGP(DRTP, LOGL_ERROR, "unsupported AMR FT 0x%02x\n", ft);
		return -EINVAL;
		break;
	}

	rc = get_amr_mode_idx(amr_mrc, cmi);
	if (rc < 0) {
		LOGP(DRTP, LOGL_ERROR, "AMR CMI %u not part of AMR MR set\n",
			cmi);
		*l1_cmi_idx = 0;
	} else
		*l1_cmi_idx = rc;

	/* Codec Mode Request is in upper 4 bits of RTP payload header,
	 * and we simply copy the CMR into the CMC */
	if (cmr == 0xF) {
		/* FIXME: we need some state about the last codec mode */
		*l1_cmr_idx = 0;
	} else {
		rc = get_amr_mode_idx(amr_mrc, cmr);
		if (rc < 0) {
			/* FIXME: we need some state about the last codec mode */
			LOGP(DRTP, LOGL_INFO, "RTP->L1: overriding CMR %u\n", cmr);
			*l1_cmr_idx = 0;
		} else
			*l1_cmr_idx = rc;
	}
#if 0
	/* check for bad quality indication */
	if (bfi == AMR_GOOD) {
		/* obtain frame type from AMR FT */
		l1_payload[2] = ft;
	} else {
		/* bad quality, we should indicate that... */
		if (ft == AMR_SID) {
			/* FIXME: Should we do GsmL1_TchPlType_Amr_SidBad? */
			l1_payload[2] = ft;
		} else {
			l1_payload[2] = ft;
		}
	}
#endif

	if (ft == AMR_SID)
		save_last_sid(lchan, l1_payload, payload_len, fn, sti);

	return payload_len+1;
}

#define RTP_MSGB_ALLOC_SIZE	512

/*! \brief function for incoming RTP via TCH.req
 *  \param rs RTP Socket
 *  \param[in] rtp_pl buffer containing RTP payload
 *  \param[in] rtp_pl_len length of \a rtp_pl
 *
 * This function prepares a msgb with a L1 PH-DATA.req primitive and
 * queues it into lchan->dl_tch_queue.
 *
 * Note that the actual L1 primitive header is not fully initialized
 * yet, as things like the frame number, etc. are unknown at the time we
 * pre-fill the primtive.
 */
void l1if_tch_encode(struct gsm_lchan *lchan, uint8_t *data, uint8_t *len,
	const uint8_t *rtp_pl, unsigned int rtp_pl_len, uint32_t fn)
{
	uint8_t *payload_type;
	uint8_t *l1_payload;
	int rc;

	DEBUGP(DRTP, "%s RTP IN: %s\n", gsm_lchan_name(lchan),
		osmo_hexdump(rtp_pl, rtp_pl_len));

	payload_type = &data[0];
	l1_payload = &data[1];

	switch (lchan->tch_mode) {
	case GSM48_CMODE_SPEECH_V1:
		if (lchan->type == GSM_LCHAN_TCH_F) {
			*payload_type = GsmL1_TchPlType_Fr;
			rc = rtppayload_to_l1_fr(l1_payload,
						 rtp_pl, rtp_pl_len);
		} else{
			*payload_type = GsmL1_TchPlType_Hr;
			rc = rtppayload_to_l1_hr(l1_payload,
						 rtp_pl, rtp_pl_len);
		}
		break;
	case GSM48_CMODE_SPEECH_EFR:
		*payload_type = GsmL1_TchPlType_Efr;
		rc = rtppayload_to_l1_efr(l1_payload, rtp_pl,
					  rtp_pl_len);
		break;
	case GSM48_CMODE_SPEECH_AMR:
		*payload_type = GsmL1_TchPlType_Amr;
		rc = rtppayload_to_l1_amr(l1_payload, rtp_pl,
					  rtp_pl_len, lchan, fn);
		break;
	default:
		/* we don't support CSD modes */
		rc = -1;
		break;
	}

	if (rc < 0) {
		LOGP(DRTP, LOGL_ERROR, "%s unable to parse RTP payload\n",
		     gsm_lchan_name(lchan));
		return;
	}

	*len = rc + 1;

	DEBUGP(DRTP, "%s RTP->L1: %s\n", gsm_lchan_name(lchan),
		osmo_hexdump(data, *len));
}

static int is_recv_only(uint8_t speech_mode)
{
	return (speech_mode & 0xF0) == (1 << 4);
}

/*! \brief receive a traffic L1 primitive for a given lchan */
int l1if_tch_rx(struct gsm_bts_trx *trx, uint8_t chan_nr, struct msgb *l1p_msg)
{
	GsmL1_Prim_t *l1p = msgb_l1prim(l1p_msg);
	GsmL1_PhDataInd_t *data_ind = &l1p->u.phDataInd;
	uint8_t payload_type = data_ind->msgUnitParam.u8Buffer[0];
	uint8_t *payload = data_ind->msgUnitParam.u8Buffer + 1;
	uint8_t payload_len, sid_first[7] = {0};
	struct msgb *rmsg = NULL;
	struct gsm_lchan *lchan = &trx->ts[L1SAP_CHAN2TS(chan_nr)].lchan[l1sap_chan2ss(chan_nr)];

	if (is_recv_only(lchan->abis_ip.speech_mode))
		return -EAGAIN;

	if (data_ind->msgUnitParam.u8Size < 1) {
		LOGP(DL1C, LOGL_ERROR, "chan_nr %d Rx Payload size 0\n",
			chan_nr);
		return -EINVAL;
	}
	payload_len = data_ind->msgUnitParam.u8Size - 1;

	switch (payload_type) {
	case GsmL1_TchPlType_Fr:
	case GsmL1_TchPlType_Efr:
		if (lchan->type != GSM_LCHAN_TCH_F)
			goto err_payload_match;
		break;
	case GsmL1_TchPlType_Hr:
		if (lchan->type != GSM_LCHAN_TCH_H)
			goto err_payload_match;
		break;
	case GsmL1_TchPlType_Amr:
		if (lchan->type != GSM_LCHAN_TCH_H &&
		    lchan->type != GSM_LCHAN_TCH_F)
			goto err_payload_match;
		break;
	case GsmL1_TchPlType_Amr_Onset:
		if (lchan->type != GSM_LCHAN_TCH_H &&
		    lchan->type != GSM_LCHAN_TCH_F)
			goto err_payload_match;
		/* according to 3GPP TS 26.093 ONSET frames precede the first
		   speech frame of a speech burst - set the marker for next RTP
		   frame and drop last SID */
		lchan->rtp_tx_marker = true;
		break;
	case GsmL1_TchPlType_Amr_SidFirstP1:
		if (lchan->type != GSM_LCHAN_TCH_H)
			goto err_payload_match;
		LOGP(DL1C, LOGL_DEBUG, "DTX: received SID_FIRST_P1 from L1 "
		     "(%d bytes)\n", payload_len);
		break;
	case GsmL1_TchPlType_Amr_SidFirstP2:
		if (lchan->type != GSM_LCHAN_TCH_H)
			goto err_payload_match;
		LOGP(DL1C, LOGL_DEBUG, "DTX: received SID_FIRST_P2 from L1 "
		     "(%d bytes)\n", payload_len);
		break;
	case GsmL1_TchPlType_Amr_SidFirstInH:
		if (lchan->type != GSM_LCHAN_TCH_H)
			goto err_payload_match;
		lchan->rtp_tx_marker = true;
		LOGP(DL1C, LOGL_DEBUG, "DTX: received SID_FIRST_INH from L1 "
		     "(%d bytes)\n", payload_len);
		break;
	case GsmL1_TchPlType_Amr_SidUpdateInH:
		if (lchan->type != GSM_LCHAN_TCH_H)
			goto err_payload_match;
		lchan->rtp_tx_marker = true;
		LOGP(DL1C, LOGL_DEBUG, "DTX: received SID_UPDATE_INH from L1 "
		     "(%d bytes)\n", payload_len);
		break;
	default:
		LOGP(DL1C, LOGL_NOTICE, "%s Rx Payload Type %s is unsupported\n",
			gsm_lchan_name(lchan),
			get_value_string(lc15bts_tch_pl_names, payload_type));
		break;
	}


	switch (payload_type) {
	case GsmL1_TchPlType_Fr:
		rmsg = l1_to_rtppayload_fr(payload, payload_len, lchan);
		break;
	case GsmL1_TchPlType_Hr:
		rmsg = l1_to_rtppayload_hr(payload, payload_len, lchan);
		break;
	case GsmL1_TchPlType_Efr:
		rmsg = l1_to_rtppayload_efr(payload, payload_len, lchan);
		break;
	case GsmL1_TchPlType_Amr:
		rmsg = l1_to_rtppayload_amr(payload, payload_len, lchan);
		break;
	case GsmL1_TchPlType_Amr_SidFirstP2:
		/* L1 do not give us SID_FIRST data, just indication */
		memcpy(sid_first, payload, payload_len);
		int len = osmo_amr_rtp_enc(sid_first, 0, AMR_SID, AMR_GOOD);
		if (len < 0)
			return 0;
		rmsg = l1_to_rtppayload_amr(sid_first, len, lchan);
		break;
	}

	if (rmsg)
		return add_l1sap_header(trx, rmsg, lchan, chan_nr,
					data_ind->u32Fn);

	return 0;

err_payload_match:
	LOGP(DL1C, LOGL_ERROR, "%s Rx Payload Type %s incompatible with lchan\n",
		gsm_lchan_name(lchan),
		get_value_string(lc15bts_tch_pl_names, payload_type));
	return -EINVAL;
}

struct msgb *gen_empty_tch_msg(struct gsm_lchan *lchan, uint32_t fn)
{
	struct msgb *msg;
	GsmL1_Prim_t *l1p;
	GsmL1_PhDataReq_t *data_req;
	GsmL1_MsgUnitParam_t *msu_param;
	uint8_t *payload_type;
	uint8_t *l1_payload;

	msg = l1p_msgb_alloc();
	if (!msg)
		return NULL;

	l1p = msgb_l1prim(msg);
	data_req = &l1p->u.phDataReq;
	msu_param = &data_req->msgUnitParam;
	payload_type = &msu_param->u8Buffer[0];
	l1_payload = &msu_param->u8Buffer[1];

	switch (lchan->tch_mode) {
	case GSM48_CMODE_SPEECH_AMR:
		*payload_type = GsmL1_TchPlType_Amr;
		if (dtx_amr_sid_optional(lchan, fn)) {
			msgb_free(msg);
			return NULL;
		}
		msu_param->u8Size = repeat_last_sid(lchan, l1_payload, fn);
		if (!msu_param->u8Size)
			osmo_amr_rtp_enc(l1_payload, 0, AMR_NO_DATA, AMR_GOOD);
		break;
	case GSM48_CMODE_SPEECH_V1:
		if (lchan->type == GSM_LCHAN_TCH_F)
			*payload_type = GsmL1_TchPlType_Fr;
		else
			*payload_type = GsmL1_TchPlType_Hr;
		/* unlike AMR, FR & HR schedued based on absolute FN value */
		if (dtx_sched_optional(lchan, fn)) {
			msgb_free(msg);
			return NULL;
		}
		msu_param->u8Size = repeat_last_sid(lchan, l1_payload, fn);
		if (!msu_param->u8Size)
			return NULL;
		break;
	case GSM48_CMODE_SPEECH_EFR:
		*payload_type = GsmL1_TchPlType_Efr;
		if (dtx_sched_optional(lchan, fn)) {
			msgb_free(msg);
			return NULL;
		}
		msu_param->u8Size = repeat_last_sid(lchan, l1_payload, fn);
		if (!msu_param->u8Size)
			return NULL;
		break;
	default:
		msgb_free(msg);
		msg = NULL;
		break;
	}

	return msg;
}
