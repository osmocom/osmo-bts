/* Traffic channel support for Sysmocom BTS L1 */

/* (C) 2011-2012 by Harald Welte <laforge@gnumonks.org>
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
#include <osmocom/gsm/gsm_utils.h>
#include <osmocom/trau/osmo_ortp.h>

#include <osmo-bts/logging.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/gsm_data.h>
#include <osmo-bts/measurement.h>
#include <osmo-bts/amr.h>

#include <sysmocom/femtobts/superfemto.h>
#include <sysmocom/femtobts/gsml1prim.h>
#include <sysmocom/femtobts/gsml1const.h>
#include <sysmocom/femtobts/gsml1types.h>

#include "femtobts.h"
#include "l1_if.h"

/* input octet-aligned, output not octet-aligned */
void osmo_nibble_shift_right(uint8_t *out, const uint8_t *in,
			     unsigned int num_nibbles)
{
	unsigned int i;
	unsigned int num_whole_bytes = num_nibbles / 2;

	/* first byte: upper nibble empty, lower nibble from src */
	out[0] = (in[0] >> 4);

	/* bytes 1.. */
	for (i = 1; i < num_whole_bytes; i++)
		out[i] = ((in[i-1] & 0xF) << 4) | (in[i] >> 4);

	/* shift the last nibble, in case there's an odd count */
	i = num_whole_bytes;
	if (num_nibbles & 1)
		out[i] = ((in[i-1] & 0xF) << 4) | (in[i] >> 4);
	else
		out[i] = (in[i-1] & 0xF) << 4;
}


/* input unaligned, output octet-aligned */
void osmo_nibble_shift_left_unal(uint8_t *out, const uint8_t *in,
				unsigned int num_nibbles)
{
	unsigned int i;
	unsigned int num_whole_bytes = num_nibbles / 2;

	for (i = 0; i < num_whole_bytes; i++)
		out[i] = ((in[i] & 0xF) << 4) | (in[i+1] >> 4);

	/* shift the last nibble, in case there's an odd count */
	i = num_whole_bytes;
	if (num_nibbles & 1)
		out[i] = (in[i] & 0xF) << 4;
}


#define GSM_FR_BITS	260
#define GSM_EFR_BITS	244

#define GSM_FR_BYTES	33	/* TS 101318 Chapter 5.1: 260 bits + 4bit sig */
#define GSM_HR_BYTES	14	/* TS 101318 Chapter 5.2: 112 bits, no sig */
#define GSM_EFR_BYTES	31	/* TS 101318 Chapter 5.3: 244 bits + 4bit sig */

static struct msgb *l1_to_rtppayload_fr(uint8_t *l1_payload, uint8_t payload_len)
{
	struct msgb *msg;
	uint8_t *cur;

	msg = msgb_alloc_headroom(1024, 128, "L1C-to-RTP");
	if (!msg)
		return NULL;

#ifdef USE_L1_RTP_MODE
	/* new L1 can deliver bits like we need them */
	cur = msgb_put(msg, GSM_FR_BYTES);
	memcpy(cur, l1_payload, GSM_FR_BYTES);
#else
	/* step1: reverse the bit-order of each payload byte */
	osmo_revbytebits_buf(l1_payload, payload_len);

	cur = msgb_put(msg, GSM_FR_BYTES);

	/* step2: we need to shift the entire L1 payload by 4 bits right */
	osmo_nibble_shift_right(cur, l1_payload, GSM_FR_BITS/4);

	cur[0] |= 0xD0;
#endif /* USE_L1_RTP_MODE */

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
#ifdef USE_L1_RTP_MODE
	/* new L1 can deliver bits like we need them */
	memcpy(l1_payload, rtp_payload, GSM_FR_BYTES);
#else
	/* step2: we need to shift the RTP payload left by one nibble*/
	osmo_nibble_shift_left_unal(l1_payload, rtp_payload, GSM_FR_BITS/4);

	/* step1: reverse the bit-order of each payload byte */
	osmo_revbytebits_buf(l1_payload, payload_len);
#endif /* USE_L1_RTP_MODE */
	return GSM_FR_BYTES;
}

#if defined(L1_HAS_EFR) && defined(USE_L1_RTP_MODE)
static struct msgb *l1_to_rtppayload_efr(uint8_t *l1_payload, uint8_t payload_len)
{
	struct msgb *msg;
	uint8_t *cur;

	msg = msgb_alloc_headroom(1024, 128, "L1C-to-RTP");
	if (!msg)
		return NULL;

#ifdef USE_L1_RTP_MODE
	/* new L1 can deliver bits like we need them */
	cur = msgb_put(msg, GSM_EFR_BYTES);
	memcpy(cur, l1_payload, GSM_EFR_BYTES);
#else
	/* step1: reverse the bit-order of each payload byte */
	osmo_revbytebits_buf(l1_payload, payload_len);

	cur = msgb_put(msg, GSM_EFR_BYTES);

	/* step 2: we need to shift the entire L1 payload by 4 bits right */
	osmo_nibble_shift_right(cur, l1_payload, GSM_EFR_BITS/4);

	cur[0] |= 0xC0;
#endif /* USE_L1_RTP_MODE */
	return msg;
}

static int rtppayload_to_l1_efr(uint8_t *l1_payload, const uint8_t *rtp_payload,
				unsigned int payload_len)
{
#ifndef USE_L1_RTP_MODE
#error We don't support EFR with L1 that doesn't support RTP mode!
#else
	memcpy(l1_payload, rtp_payload, payload_len);

	return payload_len;
#endif
}
#else
#warning No EFR support in L1
#endif /* L1_HAS_EFR */

#ifdef USE_L1_RTP_MODE
/* change the bit-order of each unaligned field inside the HR codec
 * payload from little-endian bit-ordering to bit-endian and vice-versa.
 * This is required on all sysmoBTS DSP versions < 5.3.3 in order to
 * be compliant with ETSI TS 101 318 Chapter 5.2 */
static void hr_jumble(uint8_t *dst, const uint8_t *src)
{
	/* Table 2 / Section 5.2.1 of ETSI TS 101 381 */
	const int p_unvoiced[] =
		{ 5, 11,  9,  8,  1,  2,  7,  7,  5,  7,  7,  5,  7,  7,  5,  7,  7,  5 };
	/* Table 3 / Section 5.2.1 of ETSI TS 101 381 */
	const int p_voiced[] =
		{ 5, 11,  9,  8,  1,  2,  8,  9,  5,  4,  9,  5,  4,  9,  5,  4,  9,  5 };

	int base, i, j, l, si, di;
	const int *p;

	memset(dst, 0x00, GSM_HR_BYTES);

	p = (src[4] & 0x30) ? p_voiced : p_unvoiced;

	base = 0;
	for (i = 0; i < 18; i++) {
		l = p[i];
		for (j = 0; j < l; j++) {
			si = base + j;
			di = base + l - j - 1;

			if (src[si >> 3] & (1 << (7 - (si & 7))))
				dst[di >> 3] |= (1 << (7 - (di & 7)));
		}

		base += l;
	}
}
#endif /* USE_L1_RTP_MODE */

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
#ifdef USE_L1_RTP_MODE
	struct femtol1_hdl *fl1h = trx_femtol1_hdl(lchan->ts->trx);
	if (fl1h->rtp_hr_jumble_needed)
		hr_jumble(cur, l1_payload);
	else
		memcpy(cur, l1_payload, GSM_HR_BYTES);
#else /* USE_L1_RTP_MODE */
	memcpy(cur, l1_payload, GSM_HR_BYTES);
	/* reverse the bit-order of each payload byte */
	osmo_revbytebits_buf(cur, GSM_HR_BYTES);
#endif /* USE_L1_RTP_MODE */

	return msg;
}

/*! \brief convert GSM-FR from RTP payload to L1 format
 *  \param[out] l1_payload payload part of L1 buffer
 *  \param[in] rtp_payload pointer to RTP payload data
 *  \param[in] payload_len length of \a rtp_payload
 *  \returns number of \a l1_payload bytes filled
 */
static int rtppayload_to_l1_hr(uint8_t *l1_payload, const uint8_t *rtp_payload,
				unsigned int payload_len, struct gsm_lchan *lchan)
{

	if (payload_len != GSM_HR_BYTES) {
		LOGP(DL1C, LOGL_ERROR, "RTP HR frame length %u != expected %u\n",
			payload_len, GSM_HR_BYTES);
		return 0;
	}

#ifdef USE_L1_RTP_MODE
	struct femtol1_hdl *fl1h = trx_femtol1_hdl(lchan->ts->trx);
	if (fl1h->rtp_hr_jumble_needed)
		hr_jumble(l1_payload, rtp_payload);
	else
		memcpy(l1_payload, rtp_payload, GSM_HR_BYTES);
#else /* USE_L1_RTP_MODE */
	memcpy(l1_payload, rtp_payload, GSM_HR_BYTES);
	/* reverse the bit-order of each payload byte */
	osmo_revbytebits_buf(l1_payload, GSM_HR_BYTES);
#endif /* USE_L1_RTP_MODE */

	return GSM_HR_BYTES;
}

static struct msgb *l1_to_rtppayload_amr(uint8_t *l1_payload, uint8_t payload_len,
					 struct gsm_lchan *lchan)
{
#ifndef USE_L1_RTP_MODE
	struct amr_multirate_conf *amr_mrc = &lchan->tch.amr_mr;
#endif
	struct msgb *msg;
	uint8_t amr_if2_len = payload_len - 2;
	uint8_t *cur;

	msg = msgb_alloc_headroom(1024, 128, "L1C-to-RTP");
	if (!msg)
		return NULL;

#ifdef USE_L1_RTP_MODE
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
#else
	u_int8_t cmr;
	uint8_t ft = l1_payload[2] & 0xF;
	uint8_t cmr_idx = l1_payload[1];
	/* CMR == Unset means CMR was not transmitted at this TDMA */
	if (cmr_idx == GsmL1_AmrCodecMode_Unset)
		cmr = lchan->tch.last_cmr;
	else if (cmr_idx >= amr_mrc->num_modes ||
		 cmr_idx > GsmL1_AmrCodecMode_Unset) {
		/* Make sure the CMR of the phone is in the active codec set */
		LOGP(DL1C, LOGL_NOTICE, "L1->RTP: overriding CMR IDX %u\n", cmr_idx);
		cmr = AMR_CMR_NONE;
	} else {
		cmr = amr_mrc->mode[cmr_idx].mode;
		lchan->tch.last_cmr = cmr;
	}

	/* RFC 3267  4.4.1 Payload Header */
	msgb_put_u8(msg, (cmr << 4));

	/* RFC 3267  AMR TOC */
	msgb_put_u8(msg, AMR_TOC_QBIT | (ft << 3));

	cur = msgb_put(msg, amr_if2_len-1);

	/* step1: reverse the bit-order within every byte */
	osmo_revbytebits_buf(l1_payload+2, amr_if2_len);

	/* step2: shift everything left by one nibble */
	osmo_nibble_shift_left_unal(cur, l1_payload+2, amr_if2_len*2 -1);

#endif /* USE_L1_RTP_MODE */

	return msg;
}

enum amr_frame_type {
	AMR_FT_SID_AMR	= 8,
};

int get_amr_mode_idx(const struct amr_multirate_conf *amr_mrc, uint8_t cmi)
{
	unsigned int i;
	for (i = 0; i < amr_mrc->num_modes; i++) {
		if (amr_mrc->mode[i].mode == cmi)
			return i;
	}
	return -EINVAL;
}

/*! \brief convert AMR from RTP payload to L1 format
 *  \param[out] l1_payload payload part of L1 buffer
 *  \param[in] rtp_payload pointer to RTP payload data
 *  \param[in] payload_len length of \a rtp_payload
 *  \returns number of \a l1_payload bytes filled
 */
static int rtppayload_to_l1_amr(uint8_t *l1_payload, const uint8_t *rtp_payload,
				uint8_t payload_len,
				struct gsm_lchan *lchan)
{
	struct amr_multirate_conf *amr_mrc = &lchan->tch.amr_mr;
	uint8_t ft = (rtp_payload[1] >> 3) & 0xf;
	uint8_t cmr = rtp_payload[0] >> 4;
	uint8_t cmi, sti;
	uint8_t *l1_cmi_idx = l1_payload;
	uint8_t *l1_cmr_idx = l1_payload+1;
	int rc;

#ifdef USE_L1_RTP_MODE
	memcpy(l1_payload+2, rtp_payload, payload_len);
#else
	uint8_t amr_if2_core_len = payload_len - 2;

	/* step1: shift everything right one nibble; make space for FT */
	osmo_nibble_shift_right(l1_payload+2, rtp_payload+2, amr_if2_core_len*2);
	/* step2: reverse the bit-order within every byte of the IF2
	 * core frame contained in the RTP payload */
	osmo_revbytebits_buf(l1_payload+2, amr_if2_core_len+1);

	/* lower 4 bit of first FR2 byte contains FT */
	l1_payload[2] |= ft;
#endif /* USE_L1_RTP_MODE */

	/* CMI in downlink tells the L1 encoder which encoding function
	 * it will use, so we have to use the frame type */
	switch (ft) {
	case 0: case 1: case 2: case 3:
	case 4: case 5: case 6: case 7:
		cmi = ft;
		LOGP(DRTP, LOGL_DEBUG, "SPEECH frame with CMI %u\n", cmi);
		break;
	case AMR_FT_SID_AMR:
		/* extract the mode indiciation from last bits of
		 * 39 bit SID frame (Table 6 / 26.101) */
		cmi = (rtp_payload[2+4] >> 1) & 0x7;
		sti = rtp_payload[2+4] & 0x10;
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
	if (rtp_payload[1] & AMR_TOC_QBIT) {
		/* obtain frame type from AMR FT */
		l1_payload[2] = ft;
	} else {
		/* bad quality, we should indicate that... */
		if (ft == AMR_FT_SID_AMR) {
			/* FIXME: Should we do GsmL1_TchPlType_Amr_SidBad? */
			l1_payload[2] = ft;
		} else {
			l1_payload[2] = ft;
		}
	}
#endif

	if (ft == AMR_FT_SID_AMR) {
		/* store the last SID frame in lchan context */
		unsigned int copy_len;
		copy_len = OSMO_MIN(payload_len+1,
				    ARRAY_SIZE(lchan->tch.last_sid.buf));
		lchan->tch.last_sid.len = copy_len;
		memcpy(lchan->tch.last_sid.buf, l1_payload, copy_len);
	}

	return payload_len+1;
}

#define RTP_MSGB_ALLOC_SIZE	512

/*! \brief call-back function for incoming RTP 
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
void bts_model_rtp_rx_cb(struct osmo_rtp_socket *rs, const uint8_t *rtp_pl,
			 unsigned int rtp_pl_len)
{
	struct gsm_lchan *lchan = rs->priv;
	struct msgb *msg;
	GsmL1_Prim_t *l1p;
	GsmL1_PhDataReq_t *data_req;
	GsmL1_MsgUnitParam_t *msu_param;
	uint8_t *payload_type;
	uint8_t *l1_payload;
	int rc;

	/* skip processing of incoming RTP frames if we are in loopback mode */
	if (lchan->loopback)
		return;

	msg = l1p_msgb_alloc();
	if (!msg) {
		LOGP(DRTP, LOGL_ERROR, "%s: Failed to allocate Rx payload.\n",
			gsm_lchan_name(lchan));
		return;
	}

	l1p = msgb_l1prim(msg);
	data_req = &l1p->u.phDataReq;
	msu_param = &data_req->msgUnitParam;
	payload_type = &msu_param->u8Buffer[0];
	l1_payload = &msu_param->u8Buffer[1];

	switch (lchan->tch_mode) {
	case GSM48_CMODE_SPEECH_V1:
		if (lchan->type == GSM_LCHAN_TCH_F) {
			*payload_type = GsmL1_TchPlType_Fr;
			rc = rtppayload_to_l1_fr(l1_payload,
						 rtp_pl, rtp_pl_len);
		} else{
			*payload_type = GsmL1_TchPlType_Hr;
			rc = rtppayload_to_l1_hr(l1_payload,
						 rtp_pl, rtp_pl_len, lchan);
		}
		break;
#if defined(L1_HAS_EFR) && defined(USE_L1_RTP_MODE)
	case GSM48_CMODE_SPEECH_EFR:
		*payload_type = GsmL1_TchPlType_Efr;
		rc = rtppayload_to_l1_efr(l1_payload, rtp_pl,
					  rtp_pl_len);
		break;
#endif
	case GSM48_CMODE_SPEECH_AMR:
		*payload_type = GsmL1_TchPlType_Amr;
		rc = rtppayload_to_l1_amr(l1_payload, rtp_pl,
					  rtp_pl_len, lchan);
		break;
	default:
		/* we don't support CSD modes */
		rc = -1;
		break;
	}

	if (rc < 0) {
		LOGP(DRTP, LOGL_ERROR, "%s unable to parse RTP payload\n",
		     gsm_lchan_name(lchan));
		msgb_free(msg);
		return;
	}

	msu_param->u8Size = rc + 1;


	/* make sure the number of entries in the dl_tch_queue is never
	 * more than 3 */
	{
		struct msgb *tmp;
		int count = 0;

		llist_for_each_entry(tmp, &lchan->dl_tch_queue, list)
			count++;

		DEBUGP(DL1C, "%s DL TCH queue length = %u\n",
			gsm_lchan_name(lchan), count);

		while (count >= 2) {
			tmp = msgb_dequeue(&lchan->dl_tch_queue);
			msgb_free(tmp);
			count--;
		}
	}

	/* enqueue msgb to be transmitted to L1 */
	msgb_enqueue(&lchan->dl_tch_queue, msg);
}

static int is_recv_only(uint8_t speech_mode)
{
	return (speech_mode & 0xF0) == (1 << 4);
}

/*! \brief receive a traffic L1 primitive for a given lchan */
int l1if_tch_rx(struct gsm_lchan *lchan, struct msgb *l1p_msg)
{
	GsmL1_Prim_t *l1p = msgb_l1prim(l1p_msg);
	GsmL1_PhDataInd_t *data_ind = &l1p->u.phDataInd;
	uint8_t payload_type = data_ind->msgUnitParam.u8Buffer[0];
	uint8_t *payload = data_ind->msgUnitParam.u8Buffer + 1;
	uint8_t payload_len;
	struct msgb *rmsg = NULL;

	if (is_recv_only(lchan->abis_ip.speech_mode))
		return -EAGAIN;

	if (data_ind->msgUnitParam.u8Size < 1) {
		LOGP(DL1C, LOGL_ERROR, "%s Rx Payload size 0\n",
			gsm_lchan_name(lchan));
		return -EINVAL;
	}
	payload_len = data_ind->msgUnitParam.u8Size - 1;

	if (lchan->loopback) {
		GsmL1_Prim_t *rl1p;
		GsmL1_PhDataReq_t *data_req;
		GsmL1_MsgUnitParam_t *msu_param;

		struct msgb *tmp;
		int count = 0;

		/* generate a new msgb from the paylaod */
		rmsg = l1p_msgb_alloc();
		if (!rmsg)
			return -ENOMEM;

		rl1p = msgb_l1prim(rmsg);
		data_req = &rl1p->u.phDataReq;
		msu_param = &data_req->msgUnitParam;

		memcpy(msu_param->u8Buffer,
			data_ind->msgUnitParam.u8Buffer,
			data_ind->msgUnitParam.u8Size);
		msu_param->u8Size = data_ind->msgUnitParam.u8Size;

		/* make sure the queue doesn't get too long */
		llist_for_each_entry(tmp, &lchan->dl_tch_queue, list)
			count++;
		while (count >= 1) {
			tmp = msgb_dequeue(&lchan->dl_tch_queue);
			msgb_free(tmp);
			count--;
		}

		msgb_enqueue(&lchan->dl_tch_queue, rmsg);

		return 0;
	}

	switch (payload_type) {
	case GsmL1_TchPlType_Fr:
#if defined(L1_HAS_EFR) && defined(USE_L1_RTP_MODE)
	case GsmL1_TchPlType_Efr:
#endif
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
	default:
		LOGP(DL1C, LOGL_NOTICE, "%s Rx Payload Type %s is unsupported\n",
			gsm_lchan_name(lchan),
			get_value_string(femtobts_tch_pl_names, payload_type));
		break;
	}


	switch (payload_type) {
	case GsmL1_TchPlType_Fr:
		rmsg = l1_to_rtppayload_fr(payload, payload_len);
		break;
	case GsmL1_TchPlType_Hr:
		rmsg = l1_to_rtppayload_hr(payload, payload_len, lchan);
		break;
#if defined(L1_HAS_EFR) && defined(USE_L1_RTP_MODE)
	case GsmL1_TchPlType_Efr:
		rmsg = l1_to_rtppayload_efr(payload, payload_len);
		break;
#endif
	case GsmL1_TchPlType_Amr:
		rmsg = l1_to_rtppayload_amr(payload, payload_len, lchan);
		break;
	}

	if (rmsg) {
		/* hand rmsg to RTP code for transmission */
		if (lchan->abis_ip.rtp_socket)
			osmo_rtp_send_frame(lchan->abis_ip.rtp_socket,
					    rmsg->data, rmsg->len, 160);
		msgb_free(rmsg);
	}

	return 0;

err_payload_match:
	LOGP(DL1C, LOGL_ERROR, "%s Rx Payload Type %s incompatible with lchan\n",
		gsm_lchan_name(lchan),
		get_value_string(femtobts_tch_pl_names, payload_type));
	return -EINVAL;
}

struct msgb *gen_empty_tch_msg(struct gsm_lchan *lchan)
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
		if (lchan->tch.last_sid.len) {
			memcpy(l1_payload, lchan->tch.last_sid.buf,
				lchan->tch.last_sid.len);
			msu_param->u8Size = lchan->tch.last_sid.len+1;
		} else {
			/* FIXME: decide if we should send SPEECH_BAD or
			 * SID_BAD */
#if 0
			*payload_type = GsmL1_TchPlType_Amr_SidBad;
			memset(l1_payload, 0xFF, 5);
			msu_param->u8Size = 5 + 3;
#else
			/* send an all-zero SID */
			msu_param->u8Size = 8;
#endif
		}
		break;
	default:
		msgb_free(msg);
		msg = NULL;
		break;
	}

	return msg;
}
