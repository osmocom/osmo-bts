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
#include <stdbool.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <inttypes.h>

#include <osmocom/core/msgb.h>
#include <osmocom/gsm/l1sap.h>
#include <osmocom/gsm/gsm_utils.h>
#include <osmocom/gsm/rsl.h>
#include <osmocom/core/gsmtap.h>
#include <osmocom/core/gsmtap_util.h>
#include <osmocom/core/utils.h>

#include <osmocom/codec/codec.h>

#include <osmocom/trau/osmo_ortp.h>

#include <osmo-bts/logging.h>
#include <osmo-bts/gsm_data.h>
#include <osmo-bts/l1sap.h>
#include <osmo-bts/dtx_dl_amr_fsm.h>
#include <osmo-bts/pcu_if.h>
#include <osmo-bts/measurement.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/rsl.h>
#include <osmo-bts/oml.h>
#include <osmo-bts/abis.h>
#include <osmo-bts/bts_model.h>
#include <osmo-bts/handover.h>
#include <osmo-bts/msg_utils.h>
#include <osmo-bts/rtp_input_preen.h>
#include <osmo-bts/pcuif_proto.h>
#include <osmo-bts/cbch.h>
#include <osmo-bts/asci.h>
#include <osmo-bts/csd_v110.h>

/* determine the CCCH block number based on the frame number */
unsigned int l1sap_fn2ccch_block(uint32_t fn)
{
	int rc = gsm0502_fn2ccch_block(fn);
	/* if FN is negative, we were called for something that's not CCCH! */
	OSMO_ASSERT(rc >= 0);
	return rc;
}

struct gsm_lchan *get_lchan_by_chan_nr(struct gsm_bts_trx *trx,
				       unsigned int chan_nr)
{
	struct gsm_bts_trx_ts *ts;
	unsigned int tn, ss;

	tn = L1SAP_CHAN2TS(chan_nr);
	ts = &trx->ts[tn];

	if (L1SAP_IS_CHAN_VAMOS(chan_nr)) {
		if (ts->vamos.peer == NULL)
			return NULL;
		ts = ts->vamos.peer;
	}

	if (L1SAP_IS_CHAN_CBCH(chan_nr))
		ss = 2; /* CBCH is always on sub-slot 2 */
	else
		ss = l1sap_chan2ss(chan_nr);
	OSMO_ASSERT(ss < ARRAY_SIZE(ts->lchan));

	return &ts->lchan[ss];
}

static struct gsm_lchan *
get_active_lchan_by_chan_nr(struct gsm_bts_trx *trx, unsigned int chan_nr)
{
	struct gsm_lchan *lchan = get_lchan_by_chan_nr(trx, chan_nr);

	if (lchan && lchan->state != LCHAN_S_ACTIVE) {
		LOGPLCHAN(lchan, DL1P, LOGL_NOTICE, "assuming active lchan, but state is %s\n",
			  gsm_lchans_name(lchan->state));
		return NULL;
	}
	return lchan;
}

static int l1sap_down(struct gsm_bts_trx *trx, struct osmo_phsap_prim *l1sap);

static uint32_t fn_ms_adj(uint32_t fn, const struct gsm_lchan *lchan)
{
	uint32_t samples_passed, r;

	if (lchan->tch.last_fn != LCHAN_FN_DUMMY) {
		/* 12/13 frames usable for audio in TCH,
		   160 samples per RTP packet,
		   1 RTP packet per 4 frames */
		const uint32_t num_fn = GSM_TDMA_FN_SUB(fn, lchan->tch.last_fn);
		samples_passed = num_fn * 12 * 160 / (13 * 4);
		/* round number of samples to the nearest multiple of
		   GSM_RTP_DURATION */
		r = samples_passed + GSM_RTP_DURATION / 2;
		r -= r % GSM_RTP_DURATION;

		if (r != GSM_RTP_DURATION)
			LOGPLCHAN(lchan, DRTP, LOGL_ERROR, "RTP clock out of sync with lower layer:"
				  " %"PRIu32" vs %d (%"PRIu32"->%"PRIu32")\n",
				  r, GSM_RTP_DURATION, lchan->tch.last_fn, fn);
	}
	return GSM_RTP_DURATION;
}

/* allocate a msgb containing a osmo_phsap_prim + optional l2 data
 * in order to wrap femtobts header around l2 data, there must be enough space
 * in front and behind data pointer */
struct msgb *l1sap_msgb_alloc(unsigned int l2_len)
{
	const int headroom = L1SAP_MSGB_HEADROOM;
	const int size = headroom + sizeof(struct osmo_phsap_prim) + l2_len;
	struct msgb *msg = msgb_alloc_headroom(size, headroom, "l1sap_prim");

	if (!msg)
		return NULL;

	msg->l1h = msgb_put(msg, sizeof(struct osmo_phsap_prim));

	return msg;
}

/* Enclose rmsg into an osmo_phsap primitive and hand it over to the higher
 * layers. The phsap primitive also contains measurement information. The
 * parameters rssi, ta_offs and is_sub are only needed when the measurement
 * information is passed along with the TCH data. When separate measurement
 * indications are used, those last three parameters may be set to zero. */
int add_l1sap_header(struct gsm_bts_trx *trx, struct msgb *rmsg,
		     struct gsm_lchan *lchan, uint8_t chan_nr, uint32_t fn,
		     uint16_t ber10k, int16_t lqual_cb, int8_t rssi,
		     int16_t ta_offs, uint8_t is_sub)
{
	struct osmo_phsap_prim *l1sap;

	LOGPLCHAN(lchan, DL1P, LOGL_DEBUG, "Rx -> RTP: %s\n", osmo_hexdump(rmsg->data, rmsg->len));

	rmsg->l2h = rmsg->data;
	rmsg->l1h = msgb_push(rmsg, sizeof(*l1sap));
	l1sap = msgb_l1sap_prim(rmsg);
	osmo_prim_init(&l1sap->oph, SAP_GSM_PH, PRIM_TCH, PRIM_OP_INDICATION,
		       rmsg);
	l1sap->u.tch.chan_nr = chan_nr;
	l1sap->u.tch.fn = fn;
	l1sap->u.tch.ber10k = ber10k;
	l1sap->u.tch.lqual_cb = lqual_cb;

	l1sap->u.tch.rssi = rssi;
	l1sap->u.tch.ta_offs_256bits = ta_offs;
	l1sap->u.tch.is_sub = is_sub;

	return l1sap_up(trx, l1sap);
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
	uint8_t n_s;

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

	/* Remember N(S) + 1 to find the first ciphered frame */
	n_s = (msg->data[1] >> 1) & 0x7;
	lchan->ciph_ns = (n_s + 1) % 8;

	l1sap_tx_ciph_req(lchan->ts->trx, chan_nr, 0, 1);

	return 1;
}

/* public helpers for the test */
int bts_check_for_ciph_cmd(struct msgb *msg, struct gsm_lchan *lchan,
			   uint8_t chan_nr)
{
	return check_for_ciph_cmd(msg, lchan, chan_nr);
}

uint16_t l1sap_log_ctx_sapi;

const struct value_string l1sap_common_sapi_names[] = {
	{ L1SAP_COMMON_SAPI_UNKNOWN,	"UNKNOWN" },
	/* alphabetic order */
	{ L1SAP_COMMON_SAPI_AGCH,	"AGCH" },
	{ L1SAP_COMMON_SAPI_BCCH,	"BCCH" },
	{ L1SAP_COMMON_SAPI_CBCH,	"CBCH" },
	{ L1SAP_COMMON_SAPI_FACCH_F,	"FACCH/F" },
	{ L1SAP_COMMON_SAPI_FACCH_H,	"FACCH/H" },
	{ L1SAP_COMMON_SAPI_FCCH,	"FCCH" },
	{ L1SAP_COMMON_SAPI_IDLE,	"IDLE" },
	{ L1SAP_COMMON_SAPI_NCH,	"NCH" },
	{ L1SAP_COMMON_SAPI_PACCH,	"PACCH" },
	{ L1SAP_COMMON_SAPI_PAGCH,	"PAGCH" },
	{ L1SAP_COMMON_SAPI_PBCCH,	"PBCCH" },
	{ L1SAP_COMMON_SAPI_PCH,	"PCH" },
	{ L1SAP_COMMON_SAPI_PDTCH,	"PDTCH" },
	{ L1SAP_COMMON_SAPI_PNCH,	"PNCH" },
	{ L1SAP_COMMON_SAPI_PPCH,	"PPCH" },
	{ L1SAP_COMMON_SAPI_PRACH,	"PRACH" },
	{ L1SAP_COMMON_SAPI_PTCCH,	"PTCCH" },
	{ L1SAP_COMMON_SAPI_RACH,	"RACH" },
	{ L1SAP_COMMON_SAPI_SACCH,	"SACCH" },
	{ L1SAP_COMMON_SAPI_SCH,	"SCH" },
	{ L1SAP_COMMON_SAPI_SDCCH,	"SDCCH" },
	{ L1SAP_COMMON_SAPI_TCH_F,	"TCH/F" },
	{ L1SAP_COMMON_SAPI_TCH_H,	"TCH/H" },
	{ 0, NULL }
};

static enum l1sap_common_sapi get_common_sapi_ph_data(struct gsm_bts_trx *trx, struct osmo_phsap_prim *l1sap)
{
	uint8_t link_id = l1sap->u.data.link_id;
	uint8_t chan_nr = l1sap->u.data.chan_nr;
	uint32_t u32Fn = l1sap->u.data.fn;

	if (L1SAP_IS_CHAN_TCHF(chan_nr))
		return L1SAP_COMMON_SAPI_TCH_F;

	if (L1SAP_IS_CHAN_TCHH(chan_nr))
		return L1SAP_COMMON_SAPI_TCH_H;

	if (L1SAP_IS_CHAN_SDCCH4(chan_nr) || L1SAP_IS_CHAN_SDCCH8(chan_nr))
		return L1SAP_COMMON_SAPI_SDCCH;

	if (L1SAP_IS_CHAN_BCCH(chan_nr))
		return L1SAP_COMMON_SAPI_BCCH;

	if (L1SAP_IS_CHAN_AGCH_PCH(chan_nr))
		/* The sapi depends on DSP configuration, not on the actual SYSTEM INFORMATION 3. */
		return ((l1sap_fn2ccch_block(u32Fn) >= num_agch(trx, "PH-DATA-REQ"))
			? L1SAP_COMMON_SAPI_PCH
			: L1SAP_COMMON_SAPI_AGCH);

	if (L1SAP_IS_CHAN_CBCH(chan_nr))
		return L1SAP_COMMON_SAPI_CBCH;

	if (L1SAP_IS_LINK_SACCH(link_id))
		return L1SAP_COMMON_SAPI_SACCH;

	return L1SAP_COMMON_SAPI_UNKNOWN;
}

static enum l1sap_common_sapi get_common_sapi_by_trx_prim(struct gsm_bts_trx *trx, struct osmo_phsap_prim *l1sap)
{
	/* Only downlink prims are relevant */
	switch (OSMO_PRIM_HDR(&l1sap->oph)) {
	case OSMO_PRIM(PRIM_PH_DATA, PRIM_OP_REQUEST):
		if (ts_is_pdch(&trx->ts[L1SAP_CHAN2TS(l1sap->u.data.chan_nr)]))
			return ((L1SAP_IS_PTCCH(l1sap->u.data.fn))
				? L1SAP_COMMON_SAPI_PTCCH
				: L1SAP_COMMON_SAPI_PDTCH);
		return get_common_sapi_ph_data(trx, l1sap);
	default:
		return L1SAP_COMMON_SAPI_UNKNOWN;
	}
}

/* send primitive as gsmtap */
static int gsmtap_ph_data(const struct osmo_phsap_prim *l1sap,
			  uint8_t *chan_type, uint8_t *ss, uint32_t fn,
			  uint8_t **data, unsigned int *len,
			  uint8_t num_agch)
{
	struct msgb *msg = l1sap->oph.msg;
	uint8_t chan_nr, link_id;

	*data = msgb_l2(msg);
	*len = msgb_l2(msg) ? msgb_l2len(msg) : 0;
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
		/* The sapi depends on DSP configuration, not
		 * on the actual SYSTEM INFORMATION 3. */
		if (l1sap_fn2ccch_block(fn) >= num_agch)
			*chan_type = GSMTAP_CHANNEL_PCH;
		else
			*chan_type = GSMTAP_CHANNEL_AGCH;
	} else if (L1SAP_IS_CHAN_CBCH(chan_nr)) {
		*chan_type = GSMTAP_CHANNEL_CBCH51;
	} else if (L1SAP_IS_CHAN_PDCH(chan_nr)) {
		*chan_type = GSMTAP_CHANNEL_PDTCH;
	}
	if (L1SAP_IS_LINK_SACCH(link_id))
		*chan_type |= GSMTAP_CHANNEL_ACCH;

	return 0;
}

static int gsmtap_pdch(const struct osmo_phsap_prim *l1sap,
		       uint8_t *chan_type, uint8_t *ss, uint32_t fn,
		       uint8_t **data, unsigned int *len)
{
	struct msgb *msg = l1sap->oph.msg;

	*data = msgb_l2(msg);
	*len = msgb_l2(msg) ? msgb_l2len(msg) : 0;

	if (L1SAP_IS_PTCCH(fn)) {
		*chan_type = GSMTAP_CHANNEL_PTCCH;
		*ss = L1SAP_FN2PTCCHBLOCK(fn);
	} else {
		/* TODO: distinguish PACCH */
		*chan_type = GSMTAP_CHANNEL_PDTCH;
	}

	return 0;
}

static int gsmtap_ph_rach(const struct osmo_phsap_prim *l1sap, uint8_t *chan_type,
			  uint8_t *tn, uint8_t *ss, uint32_t *fn,
			  uint8_t **data, unsigned int *len)
{
	uint8_t chan_nr = l1sap->u.rach_ind.chan_nr;
	static uint8_t ra_buf[2];

	*chan_type = GSMTAP_CHANNEL_RACH;
	*fn = l1sap->u.rach_ind.fn;
	*tn = L1SAP_CHAN2TS(chan_nr);

	if (L1SAP_IS_CHAN_TCHH(chan_nr))
		*ss = L1SAP_CHAN2SS_TCHH(chan_nr);
	else if (L1SAP_IS_CHAN_SDCCH4(chan_nr))
		*ss = L1SAP_CHAN2SS_SDCCH4(chan_nr);
	else if (L1SAP_IS_CHAN_SDCCH8(chan_nr))
		*ss = L1SAP_CHAN2SS_SDCCH8(chan_nr);
	else if (L1SAP_IS_CHAN_PDCH(chan_nr)) {
		if (L1SAP_IS_PTCCH(*fn)) {
			/* TODO: calculate sub-slot from frame-number */
			*chan_type = GSMTAP_CHANNEL_PTCCH;
		} else {
			*chan_type = GSMTAP_CHANNEL_PDTCH;
		}
	}

	if (l1sap->u.rach_ind.is_11bit) {
		/* Pack as described in 3GPP TS 44.004, figure 7.4a.b */
		ra_buf[0] = (uint8_t) (l1sap->u.rach_ind.ra >> 3);
		ra_buf[1] = (uint8_t) (l1sap->u.rach_ind.ra & 0x07);
		*len  = sizeof(ra_buf);
		*data = ra_buf;
	} else {
		ra_buf[0] = (uint8_t) (l1sap->u.rach_ind.ra & 0xff);
		*len  = sizeof(ra_buf[0]);
		*data = ra_buf;
	}

	return 0;
}

/* Paging Request 1 with "no identity" content, i.e. empty/dummy paging */
static const uint8_t paging_fill[GSM_MACBLOCK_LEN] = {
	0x15, 0x06, 0x21, 0x00, 0x01, 0xf0, 0x2b, 0x2b, 0x2b, 0x2b,
	0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b,
	0x2b, 0x2b, 0x2b };

static bool is_fill_frame(uint8_t chan_type, const uint8_t *data, unsigned int len)
{
	if (len != GSM_MACBLOCK_LEN)
		return false;

	switch (chan_type) {
	case GSMTAP_CHANNEL_AGCH:
	case GSMTAP_CHANNEL_SDCCH:
	case GSMTAP_CHANNEL_TCH_F:
	case GSMTAP_CHANNEL_TCH_H:
		if (!memcmp(data, fill_frame, GSM_MACBLOCK_LEN))
			return true;
		break;
	case GSMTAP_CHANNEL_PCH:
		if (!memcmp(data, paging_fill, GSM_MACBLOCK_LEN))
			return true;
		break;
	/* FIXME: implement the same for GSMTAP_CHANNEL_PDTCH from/to PCU */
	/* don't use 'default' case here as the above only conditionally return true */
	}
	return false;
}

static int to_gsmtap(struct gsm_bts_trx *trx, struct osmo_phsap_prim *l1sap)
{
	uint8_t *data;
	unsigned int len;
	uint8_t chan_type = 0, tn = 0, ss = 0;
	uint32_t fn;
	uint16_t uplink = GSMTAP_ARFCN_F_UPLINK;
	int8_t signal_dbm;
	int rc;

	struct gsmtap_inst *inst = trx->bts->gsmtap.inst;
	if (!inst)
		return 0;

	switch (OSMO_PRIM_HDR(&l1sap->oph)) {
	case OSMO_PRIM(PRIM_PH_DATA, PRIM_OP_REQUEST):
		uplink = 0;
		/* fall through */
	case OSMO_PRIM(PRIM_PH_DATA, PRIM_OP_INDICATION):
		fn = l1sap->u.data.fn;
		tn = L1SAP_CHAN2TS(l1sap->u.data.chan_nr);
		if (ts_is_pdch(&trx->ts[tn]))
			rc = gsmtap_pdch(l1sap, &chan_type, &ss, fn, &data,
					 &len);
		else
			rc = gsmtap_ph_data(l1sap, &chan_type, &ss, fn, &data,
					    &len, num_agch(trx, "GSMTAP"));
		signal_dbm = l1sap->u.data.rssi;
		break;
	case OSMO_PRIM(PRIM_PH_RACH, PRIM_OP_INDICATION):
		rc = gsmtap_ph_rach(l1sap, &chan_type, &tn, &ss, &fn, &data,
			&len);
		signal_dbm = l1sap->u.rach_ind.rssi;
		break;
	default:
		rc = -ENOTSUP;
	}

	if (rc)
		return rc;

	if (len == 0)
		return 0;
	if ((chan_type & GSMTAP_CHANNEL_ACCH)) {
		if (!trx->bts->gsmtap.sapi_acch)
			return 0;
	} else {
		if (!((1 << (chan_type & 31)) & trx->bts->gsmtap.sapi_mask))
			return 0;
	}

	/* don't log fill frames via GSMTAP; they serve no purpose other than
	 * to clog up your logs */
	if (is_fill_frame(chan_type, data, len))
		return 0;

	gsmtap_send(inst, trx->arfcn | uplink, tn, chan_type, ss, fn,
		    signal_dbm, 0 /* TODO: SNR */, data, len);

	return 0;
}

/* Calculate the number of RACH slots that expire in a certain GSM frame
 * See also 3GPP TS 05.02 Clause 7 Table 5 of 9 */
static unsigned int calc_exprd_rach_frames(struct gsm_bts *bts, uint32_t fn)
{
	int rach_frames_expired = 0;
	uint8_t ccch_conf;
	struct gsm48_system_information_type_3 *si3;
	unsigned int blockno;

	si3 = GSM_BTS_SI(bts, SYSINFO_TYPE_3);
	ccch_conf = si3->control_channel_desc.ccch_conf;

	if (ccch_conf == RSL_BCCH_CCCH_CONF_1_C) {
		/* It is possible to combine a CCCH with an SDCCH4, in this
		 * case the CCCH will have to share the available frames with
		 * the other channel, this results in a limited number of
		 * available rach slots */
		blockno = fn % 51;
		if (blockno == 4 || blockno == 5
		    || (blockno >= 15 && blockno <= 36) || blockno == 45
		    || blockno == 46)
			rach_frames_expired = 1;
	} else {
		/* It is possible to have multiple CCCH channels on
		 * different physical channels (large cells), this
		 * also multiplies the available/expired RACH channels.
		 * See also TS 04.08, Chapter 10.5.2.11, table 10.29 */
		if (ccch_conf == RSL_BCCH_CCCH_CONF_2_NC)
			rach_frames_expired = 2;
		else if (ccch_conf == RSL_BCCH_CCCH_CONF_3_NC)
			rach_frames_expired = 3;
		else if (ccch_conf == RSL_BCCH_CCCH_CONF_4_NC)
			rach_frames_expired = 4;
		else
			rach_frames_expired = 1;
	}

	return rach_frames_expired;
}

static void l1sap_interf_meas_calc_avg(struct gsm_bts_trx *trx)
{
	unsigned int tn, ln;

	for (tn = 0; tn < ARRAY_SIZE(trx->ts); tn++) {
		struct gsm_bts_trx_ts *ts = &trx->ts[tn];

		if (ts->mo.nm_state.operational != NM_OPSTATE_ENABLED)
			continue;
		if (ts->mo.nm_state.availability != NM_AVSTATE_OK)
			continue;

		for (ln = 0; ln < ARRAY_SIZE(ts->lchan); ln++) {
			struct gsm_lchan *lchan = &ts->lchan[ln];

			lchan->meas.interf_meas_avg_dbm = 0;
			lchan->meas.interf_band = 0;

			/* There must be at least one sample */
			if (lchan->meas.interf_meas_num == 0)
				continue;

			/* Average all collected samples */
			gsm_lchan_interf_meas_calc_avg(lchan);
		}
	}
}

static void l1sap_interf_meas_report(struct gsm_bts *bts)
{
	const uint32_t period = bts->interference.intave * 104;
	struct gsm_bts_trx *trx;

	if (bts->interference.intave == 0)
		return;
	if (bts->gsm_time.fn % period != 0)
		return;

	llist_for_each_entry(trx, &bts->trx_list, list) {
		if (trx->mo.nm_state.operational != NM_OPSTATE_ENABLED ||
		    trx->bb_transc.mo.nm_state.operational != NM_OPSTATE_ENABLED)
			continue;
		/* Calculate the average of all received samples */
		l1sap_interf_meas_calc_avg(trx);
		/* Report to the BSC over the A-bis/RSL */
		rsl_tx_rf_res(trx);
		/* Report to the PCU over the PCUIF */
		pcu_tx_interf_ind(trx, bts->gsm_time.fn);
	}
}

/* time information received from bts model */
static int l1sap_info_time_ind(struct gsm_bts *bts,
			       struct osmo_phsap_prim *l1sap,
			       struct info_time_ind_param *info_time_ind)
{
	unsigned int frames_expired;
	unsigned int i;

	DEBUGPFN(DL1P, info_time_ind->fn, "Rx MPH_INFO time ind\n");

	/* Calculate and check frame difference */
	frames_expired = GSM_TDMA_FN_SUB(info_time_ind->fn, bts->gsm_time.fn);
	if (frames_expired > 1) {
		if (bts->gsm_time.fn)
			LOGPFN(DL1P, LOGL_ERROR, info_time_ind->fn,
			     "Invalid condition detected: Frame difference is %"PRIu32"-%"PRIu32"=%u > 1!\n",
			     info_time_ind->fn, bts->gsm_time.fn, frames_expired);
	}

	/* Update our data structures with the current GSM time */
	gsm_fn2gsmtime(&bts->gsm_time, info_time_ind->fn);

	/* Update time on PCU interface */
	pcu_tx_time_ind(info_time_ind->fn);

	/* increment number of RACH slots that have passed by since the
	 * last time indication */
	for (i = 0; i < frames_expired; i++) {
		uint32_t fn = GSM_TDMA_FN_SUB(info_time_ind->fn, i);
		bts->load.rach.total += calc_exprd_rach_frames(bts, fn);
	}

	/* Report interference levels to the BSC */
	if (bts_internal_flag_get(bts, BTS_INTERNAL_FLAG_INTERF_MEAS))
		l1sap_interf_meas_report(bts);

	return 0;
}

static inline void set_ms_to_data(struct gsm_lchan *lchan, int16_t data, bool set_ms_to)
{
	if (!lchan)
		return;

	if (data + 63 > 255) { /* According to 3GPP TS 48.058 §9.3.37 Timing Offset field cannot exceed 255 */
		LOGPLCHAN(lchan, DL1P, LOGL_ERROR, "Attempting to set invalid Timing Offset value "
			  "%d (MS TO = %u)!\n", data, set_ms_to);
		return;
	}

	if (set_ms_to) {
		lchan->ms_t_offs = data + 63;
		lchan->p_offs = -1;
	} else {
		lchan->p_offs = data + 63;
		lchan->ms_t_offs = -1;
	}
}

bool trx_sched_is_sacch_fn(const struct gsm_bts_trx_ts *ts, uint32_t fn, bool uplink);

/* measurement information received from bts model */
static void process_l1sap_meas_data(struct gsm_lchan *lchan,
				    const struct osmo_phsap_prim *l1sap,
				    enum osmo_ph_prim ind_type)
{
	struct bts_ul_meas ulm;
	const struct info_meas_ind_param *info_meas_ind;
	const struct ph_data_param *ph_data_ind;
	const struct ph_tch_param *ph_tch_ind;
	uint32_t fn;
	const char *ind_name;

	/* Do not process measurement reports from non-active VGCS calls. */
	if (rsl_chan_rt_is_asci(lchan->rsl_chan_rt) && lchan->asci.talker_active != VGCS_TALKER_ACTIVE)
		return;

	switch (ind_type) {
	case PRIM_MPH_INFO:
		/* (legacy way, see also OS#2977) */
	        info_meas_ind = &l1sap->u.info.u.meas_ind;
		fn = info_meas_ind->fn;
		ind_name = "MPH INFO";
		ulm = (struct bts_ul_meas) {
			.ta_offs_256bits = info_meas_ind->ta_offs_256bits,
			.inv_rssi = info_meas_ind->inv_rssi,
			.ber10k = info_meas_ind->ber10k,
			.ci_cb = info_meas_ind->c_i_cb,
		};
		/* additionally treat SACCH frames (match by TDMA FN) as SUB frames */
		if (info_meas_ind->is_sub || trx_sched_is_sacch_fn(lchan->ts, fn, true))
			ulm.is_sub = 1;
		break;
	case PRIM_TCH:
		ph_tch_ind = &l1sap->u.tch;
		if (ph_tch_ind->rssi == 0)
			return;
		fn = ph_tch_ind->fn;
		ind_name = "TCH";
		ulm = (struct bts_ul_meas) {
			.ta_offs_256bits = ph_tch_ind->ta_offs_256bits,
			.inv_rssi = abs(ph_tch_ind->rssi),
			.ber10k = ph_tch_ind->ber10k,
			.ci_cb = ph_tch_ind->lqual_cb,
			.is_sub = ph_tch_ind->is_sub,
		};
		/* PRIM_TCH always carries DCCH, not SACCH */
		break;
	case PRIM_PH_DATA:
		ph_data_ind = &l1sap->u.data;
		if (ph_data_ind->rssi == 0)
			return;
		fn = ph_data_ind->fn;
		ind_name = "DATA";
		ulm = (struct bts_ul_meas) {
			.ta_offs_256bits = ph_data_ind->ta_offs_256bits,
			.inv_rssi = abs(ph_data_ind->rssi),
			.ber10k = ph_data_ind->ber10k,
			.ci_cb = ph_data_ind->lqual_cb,
		};
		/* additionally treat SACCH frames (match by RSL link ID) as SUB frames */
		if (ph_data_ind->is_sub || L1SAP_IS_LINK_SACCH(ph_data_ind->link_id))
			ulm.is_sub = 1;
		break;
	default:
		OSMO_ASSERT(false);
	}

	LOGPLCFN(lchan, fn, DL1P, LOGL_DEBUG,
		 "%s meas ind, ta_offs_256bits=%d, ber10k=%d, inv_rssi=%u, C/I=%d cB\n", ind_name,
		 ulm.ta_offs_256bits, ulm.ber10k, ulm.inv_rssi, ulm.ci_cb);

	/* we assume that symbol period is 1 bit: */
	set_ms_to_data(lchan, ulm.ta_offs_256bits / 256, true);

	lchan_meas_process_measurement(lchan, &ulm, fn);

	return;
}

/* any L1 MPH_INFO indication prim received from bts model */
static int l1sap_mph_info_ind(struct gsm_bts_trx *trx,
	 struct osmo_phsap_prim *l1sap, struct mph_info_param *info)
{
	const struct info_meas_ind_param *meas_ind;
	struct gsm_lchan *lchan;
	int rc = 0;

	switch (info->type) {
	case PRIM_INFO_TIME:
		if (trx != trx->bts->c0) {
			LOGPFN(DL1P, LOGL_NOTICE, info->u.time_ind.fn,
				"BTS model is sending us PRIM_INFO_TIME for TRX %u, please fix it\n",
				trx->nr);
			rc = -1;
		} else
			rc = l1sap_info_time_ind(trx->bts, l1sap,
						 &info->u.time_ind);
		break;
	case PRIM_INFO_MEAS:
		/* We should never get an INFO_IND with PRIM_INFO_MEAS
		 * when BTS_INTERNAL_FLAG_MEAS_PAYLOAD_COMB is set */
		if (bts_internal_flag_get(trx->bts, BTS_INTERNAL_FLAG_MEAS_PAYLOAD_COMB))
			OSMO_ASSERT(false);

		meas_ind = &l1sap->u.info.u.meas_ind;

		lchan = get_active_lchan_by_chan_nr(trx, meas_ind->chan_nr);
		if (!lchan) {
			LOGPFN(DL1P, LOGL_ERROR, meas_ind->fn,
			       "No lchan for chan_nr=%s\n",
			       rsl_chan_nr_str(meas_ind->chan_nr));
			return 0;
		}

		process_l1sap_meas_data(lchan, l1sap, PRIM_MPH_INFO);
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
	struct gsm_lchan *lchan = get_lchan_by_chan_nr(trx, info_act_cnf->chan_nr);
	if (lchan == NULL) {
		LOGPTRX(trx, DL1C, LOGL_ERROR, "get_lchan_by_chan_nr(chan_nr=%s) "
			"yields NULL for PRIM_INFO_ACTIVATE.conf\n",
			rsl_chan_nr_str(info_act_cnf->chan_nr));
		return -ENODEV;
	}

	LOGPLCHAN(lchan, DL1C, LOGL_INFO, "activate confirm chan_nr=%s trx=%d\n",
		  rsl_chan_nr_str(info_act_cnf->chan_nr), trx->nr);

	rsl_tx_chan_act_acknack(lchan, info_act_cnf->cause);

	/* During PDCH ACT, this is where we know that the PCU is done
	 * activating a PDCH, and PDCH switchover is complete.  See
	 * rsl_rx_dyn_pdch() */
	if (lchan->ts->pchan == GSM_PCHAN_TCH_F_PDCH
	    && (lchan->ts->flags & TS_F_PDCH_ACT_PENDING))
		ipacc_dyn_pdch_complete(lchan->ts,
					info_act_cnf->cause? -EIO : 0);

	return 0;
}

/* activation confirm received from bts model */
static int l1sap_info_rel_cnf(struct gsm_bts_trx *trx,
	struct osmo_phsap_prim *l1sap,
	struct info_act_cnf_param *info_act_cnf)
{
	struct gsm_lchan *lchan = get_lchan_by_chan_nr(trx, info_act_cnf->chan_nr);
	if (lchan == NULL) {
		LOGPTRX(trx, DL1C, LOGL_ERROR, "get_lchan_by_chan_nr(chan_nr=%s) "
			"yields NULL for PRIM_INFO_ACTIVATE.conf\n",
			rsl_chan_nr_str(info_act_cnf->chan_nr));
		return -ENODEV;
	}

	LOGPLCHAN(lchan, DL1C, LOGL_INFO, "deactivate confirm chan_nr=%s trx=%d\n",
		  rsl_chan_nr_str(info_act_cnf->chan_nr), trx->nr);

	rsl_tx_rf_rel_ack(lchan);

	/* During PDCH DEACT, this marks the deactivation of the PDTCH as
	 * requested by the PCU. Next up, we disconnect the TS completely and
	 * call back to cb_ts_disconnected(). See rsl_rx_dyn_pdch(). */
	if (lchan->ts->pchan == GSM_PCHAN_TCH_F_PDCH
	    && (lchan->ts->flags & TS_F_PDCH_DEACT_PENDING))
		bts_model_ts_disconnect(lchan->ts);

	return 0;
}

/* any L1 MPH_INFO confirm prim received from bts model */
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
		LOGP(DL1C, LOGL_NOTICE, "unknown MPH_INFO cnf type %d\n",
			info->type);
		break;
	}

	return rc;
}

/*! handling for PDTCH loopback mode, used for BER testing
 *  \param[in] lchan logical channel on which we operate
 *  \param[in] rts_ind PH-RTS.ind from PHY which we process
 *  \param[out] msg Message buffer to which we write data
 *
 *  The function will fill \a msg, from which the caller can then
 *  subsequently build a PH-DATA.req */
static int lchan_pdtch_ph_rts_ind_loop(struct gsm_lchan *lchan,
					const struct ph_data_param *rts_ind,
					struct msgb *msg, const struct gsm_time *tm)
{
	struct msgb *loop_msg;
	uint8_t *p;

	/* de-queue response message (loopback) */
	loop_msg = msgb_dequeue_count(&lchan->dl_tch_queue, &lchan->dl_tch_queue_len);
	if (!loop_msg) {
		LOGPLCGT(lchan, tm, DL1P, LOGL_NOTICE, "no looped PDTCH message, sending empty\n");
		/* empty downlink message */
		p = msgb_put(msg, GSM_MACBLOCK_LEN);
		memset(p, 0, GSM_MACBLOCK_LEN);
	} else {
		LOGPLCGT(lchan, tm, DL1P, LOGL_NOTICE, "looped PDTCH message of %u bytes\n",
			 msgb_l2len(loop_msg));
		/* copy over data from queued response message */
		p = msgb_put(msg, msgb_l2len(loop_msg));
		memcpy(p, msgb_l2(loop_msg), msgb_l2len(loop_msg));
		msgb_free(loop_msg);
	}
	return 0;
}

/* Check if given CCCH frame number is for a NCH, PCH or for an AGCH (this function is
 * only used internally, it is public to call it from unit-tests) */
enum ccch_msgt get_ccch_msgt(struct gsm_bts_trx *trx, uint32_t fn)
{
	uint8_t block, first_block, num_blocks;
	int rc;

	block = l1sap_fn2ccch_block(fn);

	/* If there is an NCH, check if the block number matches. It has priority over PCH/AGCH. */
	if (trx->bts->asci.pos_nch >= 0) {
		rc = osmo_gsm48_si1ro_nch_pos_decode(trx->bts->asci.pos_nch, &num_blocks, &first_block);
		if (rc >= 0 && block >= first_block && block < first_block + num_blocks)
			return CCCH_MSGT_NCH;
	}

	/* Note: The number of available access grant channels is set by the
	 * parameter BS_AG_BLKS_RES via system information type 3. This SI is
	 * transferred to osmo-bts via RSL */
	if (l1sap_fn2ccch_block(fn) < num_agch(trx, "PH-RTS-IND"))
		return CCCH_MSGT_AGCH;

	return CCCH_MSGT_PCH;
}


/* return the measured average of frame numbers that the RTS clock is running in advance */
int32_t bts_get_avg_fn_advance(const struct gsm_bts *bts)
{
	if (bts->fn_stats.avg_count == 0)
		return 0;
	return bts->fn_stats.avg256 / bts->fn_stats.avg_count;
}

static void l1sap_update_fnstats(struct gsm_bts *bts, uint32_t rts_fn)
{
	int32_t delta = GSM_TDMA_FN_SUB(rts_fn, bts->gsm_time.fn);

	if (delta < bts->fn_stats.min)
		bts->fn_stats.min = delta;
	if (delta > bts->fn_stats.max)
		bts->fn_stats.max = delta;

	if (bts->fn_stats.avg_count > bts->fn_stats.avg_window) {
		/* reset and start old average and new sample */
		bts->fn_stats.avg256 = (bts->fn_stats.avg256 / bts->fn_stats.avg_count) + delta;
		bts->fn_stats.avg_count = 2;
	} else {
		bts->fn_stats.avg256 += delta;
		bts->fn_stats.avg_count++;
	}
}

/* Common dequeueing function */
static inline struct msgb *lapdm_phsap_dequeue_msg(struct lapdm_entity *le)
{
	struct osmo_phsap_prim pp;
	if (lapdm_phsap_dequeue_prim(le, &pp) < 0)
		return NULL;
	return pp.oph.msg;
}

/* Special dequeueing function with FACCH repetition (3GPP TS 44.006, section 10) */
static inline struct msgb *lapdm_phsap_dequeue_msg_facch(struct gsm_lchan *lchan, struct lapdm_entity *le, uint32_t fn)
{
	struct osmo_phsap_prim pp;
	struct msgb *msg;

	/* Note: The repeated version of the FACCH block must be scheduled 8 or 9 bursts after the original
	 * transmission. see 3GPP TS 44.006, section 10.2 for a more detailed explaination. */
	if (lchan->rep_acch.dl_facch[0].msg && GSM_TDMA_FN_SUB(fn, lchan->rep_acch.dl_facch[0].fn) >= 8) {
		/* Re-use stored FACCH message buffer from SLOT #0 for repetition. */
		msg = lchan->rep_acch.dl_facch[0].msg;
		lchan->rep_acch.dl_facch[0].msg = NULL;
	} else if (lchan->rep_acch.dl_facch[1].msg && GSM_TDMA_FN_SUB(fn, lchan->rep_acch.dl_facch[1].fn) >= 8) {
		/* Re-use stored FACCH message buffer from SLOT #1 for repetition. */
		msg = lchan->rep_acch.dl_facch[1].msg;
		lchan->rep_acch.dl_facch[1].msg = NULL;
	} else {
		/* Fetch new FACCH from queue ... */
		if (lapdm_phsap_dequeue_prim(le, &pp) < 0)
			return NULL;
		msg = pp.oph.msg;

		/* Check if the LAPDm frame is a command frame,
		 * see also: 3GPP TS 04.06 section 3.2 and 3.3.2.
		 * If the MS explicitly indicated that repeated ACCH is
		 * supported, than all FACCH frames may be repeated
		 * see also: 3GPP TS 44.006, section 10.3). */
		if (!(lchan->rep_acch_cap.dl_facch_all || msg->data[0] & 0x02))
			return msg;

		/* ... and store the message buffer for repetition. */
		if (lchan->rep_acch.dl_facch[0].msg == NULL) {
			lchan->rep_acch.dl_facch[0].msg = msgb_copy(msg, "rep_facch_0");
			lchan->rep_acch.dl_facch[0].fn = fn;
		} else if (lchan->rep_acch.dl_facch[1].msg == NULL) {
			lchan->rep_acch.dl_facch[1].msg = msgb_copy(msg, "rep_facch_1");
			lchan->rep_acch.dl_facch[1].fn = fn;
		} else {
			/* By definition 3GPP TS 05.02 does not allow more than two (for TCH/H only one) FACCH blocks
			 * to be transmitted simultaniously. */
			OSMO_ASSERT(false);
		}
	}

	return msg;
}

/* Special dequeueing function with SACCH repetition (3GPP TS 44.006, section 11) */
static inline struct msgb *lapdm_phsap_dequeue_msg_sacch(struct gsm_lchan *lchan, struct lapdm_entity *le)
{
	struct osmo_phsap_prim pp;
	struct msgb *msg;
	uint8_t sapi;

	/* Note: When the MS disables SACCH repetition, we still must collect
	 * possible candidates in order to have one ready in case the MS enables
	 * SACCH repetition. */

	if (lchan->rep_acch.dl_sacch_msg) {
		if (lchan->meas.l1_info.srr_sro == 0) {
			/* Toss previous repetition candidate */
			msgb_free(lchan->rep_acch.dl_sacch_msg);
			lchan->rep_acch.dl_sacch_msg = NULL;
		} else {
			/* Use previous repetition candidate */
			msg = lchan->rep_acch.dl_sacch_msg;
			lchan->rep_acch.dl_sacch_msg = NULL;
			return msg;
		}
	}

	/* Fetch new repetition candidate from queue */
	if (lapdm_phsap_dequeue_prim(le, &pp) < 0)
		return NULL;
	msg = pp.oph.msg;
	sapi = (msg->data[0] >> 2) & 0x07;

	/* Only LAPDm frames for SAPI 0 may become a repetition
	 * candidate. */
	if (sapi == 0)
		lchan->rep_acch.dl_sacch_msg = msgb_copy(msg, "rep_sacch");

	return msg;
}

/* PH-RTS-IND prim received from bts model */
static int l1sap_ph_rts_ind(struct gsm_bts_trx *trx,
	struct osmo_phsap_prim *l1sap, struct ph_data_param *rts_ind)
{
	struct msgb *msg = l1sap->oph.msg;
	struct gsm_time g_time;
	struct gsm_lchan *lchan;
	uint8_t chan_nr, link_id;
	uint8_t tn;
	uint32_t fn;
	uint8_t *p = NULL;
	uint8_t *si;
	struct lapdm_entity *le;
	struct msgb *pp_msg;
	bool dtxd_facch = false;
	int rc;

	chan_nr = rts_ind->chan_nr;
	link_id = rts_ind->link_id;
	fn = rts_ind->fn;
	tn = L1SAP_CHAN2TS(chan_nr);

	gsm_fn2gsmtime(&g_time, fn);

	DEBUGPGT(DL1P, &g_time, "Rx PH-RTS.ind chan_nr=%s link_id=0x%02xd\n", rsl_chan_nr_str(chan_nr), link_id);

	l1sap_update_fnstats(trx->bts, fn);

	/* reuse PH-RTS.ind for PH-DATA.req */
	if (!msg) {
		LOGPGT(DL1P, LOGL_FATAL, &g_time, "RTS without msg to be reused. Please fix!\n");
		abort();
	}
	msgb_trim(msg, sizeof(*l1sap));
	osmo_prim_init(&l1sap->oph, SAP_GSM_PH, PRIM_PH_DATA, PRIM_OP_REQUEST,
		msg);
	msg->l2h = msg->l1h + sizeof(*l1sap);

	if (ts_is_pdch(&trx->ts[tn])) {
		lchan = get_active_lchan_by_chan_nr(trx, chan_nr);
		if (lchan && lchan->loopback) {
			if (!L1SAP_IS_PTCCH(rts_ind->fn))
				lchan_pdtch_ph_rts_ind_loop(lchan, rts_ind, msg, &g_time);
			/* continue below like for SACCH/FACCH/... */
		} else {
			/* forward RTS.ind to PCU */
			if (L1SAP_IS_PTCCH(rts_ind->fn)) {
				pcu_tx_rts_req(&trx->ts[tn], 1, fn, trx->arfcn,
						L1SAP_FN2PTCCHBLOCK(fn));
			} else {
				pcu_tx_rts_req(&trx->ts[tn], 0, fn, trx->arfcn,
						L1SAP_FN2MACBLOCK(fn));
			}
			/* return early, PCU takes care of rest */
			return 0;
		}
	} else if (L1SAP_IS_CHAN_BCCH(chan_nr)) {
		p = msgb_put(msg, GSM_MACBLOCK_LEN);
		/* get them from bts->si_buf[] */
		si = bts_sysinfo_get(trx->bts, &g_time);
		if (si)
			memcpy(p, si, GSM_MACBLOCK_LEN);
		else
			memcpy(p, fill_frame, GSM_MACBLOCK_LEN);
	} else if (L1SAP_IS_CHAN_CBCH(chan_nr)) {
		p = msgb_put(msg, GSM_MACBLOCK_LEN);
		bts_cbch_get(trx->bts, p, &g_time);
	} else if (!(chan_nr & 0x80)) { /* only TCH/F, TCH/H, SDCCH/4 and SDCCH/8 have C5 bit cleared */
		lchan = get_active_lchan_by_chan_nr(trx, chan_nr);
		if (!lchan) {
			LOGPGT(DL1P, LOGL_ERROR, &g_time, "No lchan for PH-RTS.ind (chan_nr=%s)\n",
			       rsl_chan_nr_str(chan_nr));
			return 0;
		}
		if (lchan->pending_rel_ind_msg) {
			LOGPLCGT(lchan, &g_time, DRSL, LOGL_INFO, "Forward RLL RELease INDication to the BSC\n");
			abis_bts_rsl_sendmsg(lchan->pending_rel_ind_msg);
			lchan->pending_rel_ind_msg = NULL;
		}
		if (L1SAP_IS_LINK_SACCH(link_id)) {
			p = msgb_put(msg, GSM_MACBLOCK_LEN);
			/* L1-header, if not set/modified by layer 1 */
			p[0] = lchan->ms_power_ctrl.current;
			if (lchan->rep_acch.ul_sacch_active)
				p[0] |= 0x40; /* See also: 3GPP TS 44.004, section 7.1 */
			p[1] = lchan->ta_ctrl.current;
			le = &lchan->lapdm_ch.lapdm_acch;
			if (lchan->rep_acch_cap.dl_sacch) {
				/* Check if MS requests SACCH repetition and update state accordingly */
				if (lchan->meas.l1_info.srr_sro) {
					if (lchan->rep_acch.dl_sacch_active == false)
						LOGPLCHAN(lchan, DL1P, LOGL_DEBUG, "DL-SACCH repetition: inactive => active\n");
					lchan->rep_acch.dl_sacch_active = true;
				} else {
					if (lchan->rep_acch.dl_sacch_active == true)
						LOGPLCHAN(lchan, DL1P, LOGL_DEBUG, "DL-SACCH repetition: active => inactive\n");
					lchan->rep_acch.dl_sacch_active = false;
				}
				pp_msg = lapdm_phsap_dequeue_msg_sacch(lchan, le);
			} else {
				pp_msg = lapdm_phsap_dequeue_msg(le);
			}
		} else {
			if (lchan->ts->trx->bts->dtxd)
				dtxd_facch = true;
			le = &lchan->lapdm_ch.lapdm_dcch;
			if (lchan->rep_acch.dl_facch_active && lchan->rsl_cmode != RSL_CMOD_SPD_SIGN)
				pp_msg = lapdm_phsap_dequeue_msg_facch(lchan, le, fn);
			else
				pp_msg = lapdm_phsap_dequeue_msg(le);
			lchan->tch.dtx_fr_hr_efr.dl_facch_stealing = (pp_msg != NULL);
		}
		if (!pp_msg) {
			if (L1SAP_IS_LINK_SACCH(link_id)) {
				/* No SACCH data from LAPDM pending, send SACCH filling */
				uint8_t *si = lchan_sacch_get(lchan);
				if (si) {
					/* The +2 is empty space where the DSP inserts the L1 hdr */
					memcpy(p + 2, si, GSM_MACBLOCK_LEN - 2);
				} else
					memcpy(p + 2, fill_frame, GSM_MACBLOCK_LEN - 2);
			} else if (vgcs_is_uplink_free(lchan)) {
				/* If UPLINK FREE message is stored, send it with every DCCH frame. */
				p = msgb_put(msg, GSM_MACBLOCK_LEN);
				vgcs_uplink_free_get(lchan, p);
			} else if (L1SAP_IS_CHAN_SDCCH4(chan_nr) || L1SAP_IS_CHAN_SDCCH8(chan_nr) ||
				   (lchan->rsl_cmode == RSL_CMOD_SPD_SIGN && !lchan->ts->trx->bts->dtxd)) {
				/*
				 * SDCCH or TCH in signalling mode without DTX.
				 *
				 * Send fill frame according to GSM 05.08, section 8.3: "On the SDCCH and on the
				 * half rate speech traffic channel in signalling only mode DTX is not allowed.
				 * In these cases and during signalling on the TCH when DTX is not used, the same
				 * L2 fill frame shall be transmitted in case there is nothing else to transmit."
				 */
				p = msgb_put(msg, GSM_MACBLOCK_LEN);
				memcpy(p, fill_frame, GSM_MACBLOCK_LEN);
			} /* else the message remains empty, so TCH frames are sent */
		} else {
			/* The +2 is empty space where the DSP inserts the L1 hdr */
			if (L1SAP_IS_LINK_SACCH(link_id))
				memcpy(p + 2, pp_msg->data + 2, GSM_MACBLOCK_LEN - 2);
			else {
				p = msgb_put(msg, GSM_MACBLOCK_LEN);
				memcpy(p, pp_msg->data, GSM_MACBLOCK_LEN);
				/* check if it is a RR CIPH MODE CMD. if yes, enable RX ciphering */
				check_for_ciph_cmd(pp_msg, lchan, chan_nr);
				if (dtxd_facch)
					dtx_dispatch(lchan, E_FACCH);
				if (rsl_chan_rt_is_vgcs(lchan->rsl_chan_rt)) {
					/* Check for UPLINK FREE message and store. */
					if (pp_msg->data[0] == GSM48_MT_RR_SH_UL_FREE << 2)
						vgcs_uplink_free_set(lchan, pp_msg->data);
					/* Keep UPLINK FREE message when sending short header messages. */
					else if ((pp_msg->data[0] & 0x03) != 0x00)
						vgcs_uplink_free_reset(lchan);
				}
			}
			msgb_free(pp_msg);
		}
	} else if (L1SAP_IS_CHAN_AGCH_PCH(chan_nr)) {
		p = msgb_put(msg, GSM_MACBLOCK_LEN);
		rc = bts_ccch_copy_msg(trx->bts, p, &g_time, get_ccch_msgt(trx, fn));
		if (rc <= 0)
			memcpy(p, fill_frame, GSM_MACBLOCK_LEN);
	}

	DEBUGPGT(DL1P, &g_time, "Tx PH-DATA.req chan_nr=%s link_id=0x%02x\n", rsl_chan_nr_str(chan_nr), link_id);

	l1sap_down(trx, l1sap);

	/* don't free, because we forwarded data */
	return 1;
}

/* The following static functions are helpers for l1sap_tch_rts_ind(),
 * used only for FR/HR/EFR speech modes.  For these speech TCH modes,
 * if our incoming RTP stream includes SID frames, we need to apply
 * the following transformations to the downlink frame stream we actually
 * transmit:
 *
 * - We need to cache the last received SID and retransmit it in
 *   SACCH-aligned frame positions, or if the SACCH-aligned frame
 *   position was stolen by FACCH, then right after that FACCH.
 *
 * - That cached SID needs to be aged and expired, in accord with
 *   TS 28.062 section C.3.2.1.1 paragraph 5 - the paragraph concerning
 *   expiration of cached downlink SID.
 *
 * - In all other frame positions, extraneous SID frames need to be
 *   dropped - we send an empty payload to the BTS model, causing it
 *   to transmit an induced BFI condition on the air (fake DTXd),
 *   or perhaps real DTXd (actually turning off Tx) in the future.
 */

/*! \brief Check if the given FN of TCH-RTS-IND corresponds to a mandatory
 *  SID position for non-AMR codecs that follow SACCH alignment.
 *  \param[in] lchan Logical channel on which we check scheduling
 *  \param[in] fn Frame Number for which we check scheduling
 *  \returns true if this FN is a mandatory SID position, false otherwise
 */
static inline bool fr_hr_efr_sid_position(struct gsm_lchan *lchan, uint32_t fn)
{
	/* See GSM 05.08 section 8.3 for frame numbers - but we are
	 * specifically looking for FNs corresponding to the beginning
	 * of the complete SID frame to be transmitted, rather than all FNs
	 * where we have to put out a non-suppressed burst. */
	switch (lchan->type) {
	case GSM_LCHAN_TCH_F:
		return fn % 104 == 52;
	case GSM_LCHAN_TCH_H:
		switch (lchan->nr) {
		case 0:
			return fn % 104 == 0 || fn % 104 == 52;
		case 1:
			return fn % 104 == 14 || fn % 104 == 66;
		default:
			return false;
		}
	default:
		return false;
	}
}

/*! \brief This helper function implements DTXd input processing for FR/HR/EFR:
 *  we got an RTP input frame, now we need to check if it is SID or not,
 *  and update our DL SID reshaper state accordingly.
 *  \param[in] lchan Logical channel structure of the TCH we work with
 *  \param[in] resp_msg The input frame from RTP
 *  \param[out] sid_result Output flag indicating if the received frame is SID
 *  \returns true if the frame in resp_msg is good, false otherwise
 */
static bool fr_hr_efr_dtxd_input(struct gsm_lchan *lchan, struct msgb *resp_msg,
				 bool *sid_result)
{
	enum osmo_gsm631_sid_class sidc;
	bool is_sid;

	switch (lchan->tch_mode) {
	case GSM48_CMODE_SPEECH_V1:
		if (lchan->type == GSM_LCHAN_TCH_F) {
			sidc = osmo_fr_sid_classify(msgb_l2(resp_msg));
			switch (sidc) {
			case OSMO_GSM631_SID_CLASS_SPEECH:
				is_sid = false;
				break;
			case OSMO_GSM631_SID_CLASS_INVALID:
				/* TS 28.062 section C.3.2.1.1: invalid SIDs
				 * from call leg A UL are treated like BFIs.
				 * Drop this frame and act as if we got nothing
				 * at all from RTP for this FN. */
				return false;
			case OSMO_GSM631_SID_CLASS_VALID:
				is_sid = true;
				/* The SID code word may have a one bit error -
				 * rejuvenate it. */
				osmo_fr_sid_reset(msgb_l2(resp_msg));
				break;
			default:
				/* SID classification per GSM 06.31 section
				 * 6.1.1 has only 3 possible outcomes. */
				OSMO_ASSERT(0);
			}
		} else {
			/* The same kind of classification as we do in
			 * osmo_{fr,efr}_sid_classify() is impossible for HR:
			 * the equivalent ternary SID classification per
			 * GSM 06.41 can only be done in the UL-handling BTS,
			 * directly coupled to the GSM 05.03 channel decoder,
			 * and cannot be reconstructed downstream from frame
			 * payload bits.  The only kind of SID we can detect
			 * here is the perfect, error-free kind.  However,
			 * if we received RFC 5993 payload and the sender
			 * told us it is valid SID, honor that indication
			 * and rejuvenate the SID codeword. */
			if (rtpmsg_is_rfc5993_sid(resp_msg)) {
				is_sid = true;
				osmo_hr_sid_reset(msgb_l2(resp_msg));
			} else {
				is_sid = osmo_hr_check_sid(msgb_l2(resp_msg),
							   msgb_l2len(resp_msg));
			}
		}
		break;
	case GSM48_CMODE_SPEECH_EFR:
		/* same logic as for FRv1 */
		sidc = osmo_efr_sid_classify(msgb_l2(resp_msg));
		switch (sidc) {
		case OSMO_GSM631_SID_CLASS_SPEECH:
			is_sid = false;
			break;
		case OSMO_GSM631_SID_CLASS_INVALID:
			return false;
		case OSMO_GSM631_SID_CLASS_VALID:
			is_sid = true;
			osmo_efr_sid_reset(msgb_l2(resp_msg));
			break;
		default:
			OSMO_ASSERT(0);
		}
		break;
	default:
		/* This static function should never be called except for
		 * V1 and EFR speech modes. */
		OSMO_ASSERT(0);
	}
	*sid_result = is_sid;
	lchan->tch.dtx_fr_hr_efr.last_rtp_input_was_sid = is_sid;
	if (is_sid) {
		uint8_t copy_len;

		copy_len = OSMO_MIN(msgb_l2len(resp_msg),
				ARRAY_SIZE(lchan->tch.dtx_fr_hr_efr.last_sid));
		memcpy(lchan->tch.dtx_fr_hr_efr.last_sid,
			msgb_l2(resp_msg), copy_len);
		lchan->tch.dtx_fr_hr_efr.last_sid_len = copy_len;
		lchan->tch.dtx_fr_hr_efr.last_sid_age = 0;
	} else {
		/* We got a speech frame, not SID - therefore, the state flag
		 * of "we already transmitted a SID" needs to be cleared,
		 * so that the very first SID that follows this talkspurt
		 * will get transmitted right away, without waiting for
		 * the next mandatory SID position. */
		lchan->tch.dtx_fr_hr_efr.dl_sid_transmitted = false;
	}
	return true;
}

/*! \brief This helper function implements DTXd output processing for FR/HR/EFR:
 *  here we update our state to deal with cached SID aging, mandatory-Tx
 *  frame positions and FACCH stealing, and we make the desired output
 *  transformations of either regurgitating a cached SID or vice-versa,
 *  dropping a SID we received from RTP.
 *  \param[in] lchan Logical channel structure of the TCH we work with
 *  \param[in] fn Frame Number for which we are preparing DL
 *  \param[in] current_input_is_sid Self-explanatory
 *  \param[in] resp_msg_p Pointer to l1sap_tch_rts_ind() internal variable
 *  \param[in] resp_l1sap_p ditto
 *  \param[in] empty_l1sap_p ditto
 *
 *  This function gets called with pointers to l1sap_tch_rts_ind() internal
 *  variables and cannot be properly understood on its own, without
 *  understanding the parent function first.  This situation is unfortunate,
 *  but it was the only way to factor the present logic out of
 *  l1sap_tch_rts_ind() main body.
 */
static void fr_hr_efr_dtxd_output(struct gsm_lchan *lchan, uint32_t fn,
				  bool current_input_is_sid,
				  struct msgb **resp_msg_p,
				  struct osmo_phsap_prim **resp_l1sap_p,
				  struct osmo_phsap_prim *empty_l1sap_p)
{
	struct msgb *resp_msg = *resp_msg_p;
	bool sid_in_hand = current_input_is_sid;

	/* Are we at a mandatory-Tx frame position? If so, clear the state flag
	 * of "we already transmitted a SID in this window" - as of right now,
	 * a SID has _not_ been transmitted in the present window yet, and if
	 * we are in a DTX pause, we do need to transmit a SID update as soon
	 * as we are able to, FACCH permitting. */
	if (fr_hr_efr_sid_position(lchan, fn))
		lchan->tch.dtx_fr_hr_efr.dl_sid_transmitted = false;

	/* The next stanza implements logic that was originally meant to reside
	 * in the TFO block in TRAUs: if the source feeding us RTP is in a DTXu
	 * pause (resp_msg == NULL, meaning we got nothing from RTP) *and* the
	 * most recent RTP input we did get was a SID, and that SID hasn't
	 * expired, then we need to replicate that SID. */
	if (!resp_msg && lchan->tch.dtx_fr_hr_efr.last_rtp_input_was_sid &&
	    lchan->tch.dtx_fr_hr_efr.last_sid_age < 47) {
		/* Whatever we do further below, any time another 20 ms window
		 * has passed since the last SID was received in RTP, we have
		 * to age that cached SID. */
		lchan->tch.dtx_fr_hr_efr.last_sid_age++;

		/* The following conditional checking dl_sid_transmitted flag
		 * is peculiar to our sans-E1 architecture.  In the original
		 * T1/E1 Abis architecture the TFO-enabled TRAU would repeat
		 * the cached SID from call leg A into *every* 20 ms frame in
		 * its Abis output, and then the BTS would select which ones
		 * it should transmit per the rules of GSM 06.31/06.81 section
		 * 5.1.2.  But in our architecture it would be silly and
		 * wasteful to allocate and fill an msgb for this cached SID
		 * only to toss it later in the same function - hence the
		 * logic of section 5.1.2 is absorbed into the decision right
		 * here to proceed with cached SID regurgitation or not,
		 * in the form of the following conditional. */
		if (!lchan->tch.dtx_fr_hr_efr.dl_sid_transmitted)
			resp_msg = l1sap_msgb_alloc(lchan->tch.dtx_fr_hr_efr.last_sid_len);
		if (resp_msg) {
			resp_msg->l2h = msgb_put(resp_msg,
						lchan->tch.dtx_fr_hr_efr.last_sid_len);
			memcpy(resp_msg->l2h, lchan->tch.dtx_fr_hr_efr.last_sid,
				lchan->tch.dtx_fr_hr_efr.last_sid_len);
			*resp_msg_p = resp_msg;
			*resp_l1sap_p = msgb_l1sap_prim(resp_msg);
			sid_in_hand = true;
		}
	} else if (resp_msg && sid_in_hand) {
		/* This "else if" leg implements the logic of section 5.1.2
		 * for cases when a SID is already present in the RTP input,
		 * as indicated by (resp_msg != NULL) and sid_in_hand being
		 * true.  Because we are in the "else" clause of the big "if"
		 * above, this path executes only when the SID has come from
		 * RTP in _this_ frame, rather than regurgitated from cache.
		 * But be it fresh or cached, the rules of section 5.1.2 still
		 * apply, and if we've already transmitted a SID in the current
		 * window, then we need to suppress further SIDs and send
		 * an empty payload to the BTS model, causing the latter
		 * to transmit an induced BFI condition on the air.  This
		 * strange-seeming behavior is needed so that the spec-compliant
		 * Rx DTX handler in the MS will produce the expected output,
		 * same as if the GSM network were the old-fashioned kind with
		 * Abis TRAUs and TFO. */
		if (lchan->tch.dtx_fr_hr_efr.dl_sid_transmitted) {
			msgb_free(resp_msg);
			resp_msg = NULL;
			*resp_msg_p = NULL;
			*resp_l1sap_p = empty_l1sap_p;
		}
	}

	/* The following conditional answers this question: are we actually
	 * transmitting a SID frame on the air right now, at this frame number?
	 * If we are certain the BTS model is going to transmit this SID,
	 * we set the state flag so we won't be transmitting any more SIDs
	 * until we either hit the next mandatory-Tx position or get another
	 * little talkspurt followed by new SID.  The check for FACCH stealing
	 * is included because if the BTS model is going to transmit FACCH in
	 * the current FN, then we are not actually transmitting SID right now,
	 * and we still need to transmit a SID ASAP, as soon as the TCH becomes
	 * becomes free from FACCH activity.  GSM 06.31/06.81 section 5.1.2
	 * does mention that if the mandatory-Tx frame position is taken up
	 * by FACCH, then we need to send SID in the following frame. */
	if (resp_msg && sid_in_hand && !lchan->tch.dtx_fr_hr_efr.dl_facch_stealing)
		lchan->tch.dtx_fr_hr_efr.dl_sid_transmitted = true;
}

/* TCH-RTS-IND prim received from bts model */
static int l1sap_tch_rts_ind(struct gsm_bts_trx *trx,
	struct osmo_phsap_prim *l1sap, struct ph_tch_param *rts_ind)
{
	struct msgb *resp_msg;
	struct osmo_phsap_prim *resp_l1sap, empty_l1sap;
	struct gsm_time g_time;
	struct gsm_lchan *lchan;
	uint8_t chan_nr, marker = 0;
	uint32_t fn;
	bool is_fr_hr_efr_sid = false;

	chan_nr = rts_ind->chan_nr;
	fn = rts_ind->fn;

	gsm_fn2gsmtime(&g_time, fn);

	lchan = get_active_lchan_by_chan_nr(trx, chan_nr);
	if (!lchan) {
		LOGPGT(DL1P, LOGL_ERROR, &g_time, "No lchan for PH-RTS.ind (chan_nr=%s)\n", rsl_chan_nr_str(chan_nr));
		return 0;
	} else {
		LOGPLCGT(lchan, &g_time, DL1P, LOGL_DEBUG, "Rx TCH-RTS.ind\n");
	}

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
	resp_msg = msgb_dequeue_count(&lchan->dl_tch_queue, &lchan->dl_tch_queue_len);
	if (!resp_msg) {
		LOGPLCGT(lchan, &g_time, DL1P, LOGL_DEBUG, "DL TCH Tx queue underrun\n");
		resp_l1sap = &empty_l1sap;
	} else {
		/* Obtain RTP header Marker bit from control buffer */
		marker = rtpmsg_marker_bit(resp_msg);

		resp_msg->l2h = resp_msg->data;
		msgb_push(resp_msg, sizeof(*resp_l1sap));
		resp_msg->l1h = resp_msg->data;
		resp_l1sap = msgb_l1sap_prim(resp_msg);

		/* FR/HR/EFR SID or non-SID input handling */
		if (lchan->tch_mode == GSM48_CMODE_SPEECH_V1 ||
		    lchan->tch_mode == GSM48_CMODE_SPEECH_EFR) {
			bool drop;

			drop = !fr_hr_efr_dtxd_input(lchan, resp_msg,
						     &is_fr_hr_efr_sid);
			if (OSMO_UNLIKELY(drop)) {
				msgb_free(resp_msg);
				resp_msg = NULL;
				resp_l1sap = &empty_l1sap;
			}
		}
	}

	/* FR/HR/EFR DTXd output stage */
	if (lchan->tch_mode == GSM48_CMODE_SPEECH_V1 ||
	    lchan->tch_mode == GSM48_CMODE_SPEECH_EFR) {
		fr_hr_efr_dtxd_output(lchan, fn, is_fr_hr_efr_sid, &resp_msg,
					&resp_l1sap, &empty_l1sap);
	}

	memset(resp_l1sap, 0, sizeof(*resp_l1sap));
	osmo_prim_init(&resp_l1sap->oph, SAP_GSM_PH, PRIM_TCH, PRIM_OP_REQUEST,
		resp_msg);
	resp_l1sap->u.tch.chan_nr = chan_nr;
	resp_l1sap->u.tch.fn = fn;
	resp_l1sap->u.tch.marker = marker;

	LOGPLCGT(lchan, &g_time, DL1P, LOGL_DEBUG, "Tx TCH.req\n");

	l1sap_down(trx, resp_l1sap);

	return 0;
}

/* Reset link timeout to current value. */
void radio_link_timeout_reset(struct gsm_lchan *lchan)
{
	struct gsm_bts *bts = lchan->ts->trx->bts;

	lchan->s = bts->radio_link_timeout.current;
}

/* process radio link timeout counter S. Follows TS 05.08 Section 5.2
 * "MS Procedure" as the "BSS Procedure [...] shall be determined by the
 * network operator." */
static void radio_link_timeout(struct gsm_lchan *lchan, bool bad_frame)
{
	struct gsm_bts *bts = lchan->ts->trx->bts;

	/* Bypass radio link timeout on VGCS/VBS channels: There is no
	 * uplink SACCH on these when talker is not active. */
	if (rsl_chan_rt_is_asci(lchan->rsl_chan_rt) && lchan->asci.talker_active != VGCS_TALKER_ACTIVE)
		return;

	/* Bypass radio link timeout if set to -1 */
	if (bts->radio_link_timeout.current < 0)
		return;

	/* if link loss criterion already reached */
	if (lchan->s == 0) {
		LOGPLCHAN(lchan, DMEAS, LOGL_DEBUG,
			  "radio link timeout counter S is already 0\n");
		return;
	}

	if (bad_frame) {
		LOGPLCHAN(lchan, DMEAS, LOGL_DEBUG,
			  "decreasing radio link timeout counter S=%d -> %d\n",
			  lchan->s, lchan->s - 1);
		lchan->s--; /* count down radio link counter S */
		if (lchan->s == 0) {
			LOGPLCHAN(lchan, DMEAS, LOGL_NOTICE,
				  "radio link timeout counter S reached zero, "
				  "dropping connection\n");
			rsl_tx_conn_fail(lchan, RSL_ERR_RADIO_LINK_FAIL);
		}
		return;
	}

	if (lchan->s < bts->radio_link_timeout.current) {
		/* count up radio link counter S */
		int s = lchan->s + 2;
		if (s > bts->radio_link_timeout.current)
			s = bts->radio_link_timeout.current;
		LOGPLCHAN(lchan, DMEAS, LOGL_DEBUG,
			  "increasing radio link timeout counter S=%d -> %d\n",
			  lchan->s, s);
		lchan->s = s;
	}
}

static inline int check_for_first_ciphrd(struct gsm_lchan *lchan,
					  uint8_t *data, int len)
{
	uint8_t n_r;

	/* if this is the first valid message after enabling Rx
	 * decryption, we have to enable Tx encryption */
	if (lchan->ciph_state != LCHAN_CIPH_RX_CONF)
		return 0;

	/* HACK: check if it's an I frame, in order to
	 * ignore some still buffered/queued UI frames received
	 * before decryption was enabled */
	if (data[0] != 0x01)
		return 0;

	if ((data[1] & 0x01) != 0)
		return 0;

	n_r = data[1] >> 5;
	if (lchan->ciph_ns != n_r)
		return 0;

	return 1;
}

/* public helper for the test */
int bts_check_for_first_ciphrd(struct gsm_lchan *lchan,
				uint8_t *data, int len)
{
	return check_for_first_ciphrd(lchan, data, len);
}

/* Decide if repeated UL-SACCH should be applied or not. If the BER level, of
 * the received SACCH blocks rises above a certain threshold UL-SACCH
 * repetition is enabled */
static void repeated_ul_sacch_active_decision(struct gsm_lchan *lchan,
					      uint16_t ber10k)
{
	uint16_t upper = 0;
	uint16_t lower = 0;
	bool prev_repeated_ul_sacch_active = lchan->rep_acch.ul_sacch_active;

	/* This is an optimization so that we exit as quickly as possible if
	 * there are no uplink SACCH repetition capabilities present.
	 * However If the repeated UL-SACCH capabilities vanish for whatever
	 * reason, we must be sure that UL-SACCH repetition is disabled. */
	if (!lchan->rep_acch_cap.ul_sacch) {
		lchan->rep_acch.ul_sacch_active = false;
		goto out;
	}

	/* Threshold disabled (repetition is always on) */
	if (lchan->rep_acch_cap.rxqual == 0) {
		lchan->rep_acch.ul_sacch_active = true;
		goto out;
	}

	/* convert from RXQUAL value to ber10k value.
	 * see also GSM 05.08, section 8.2.4 (first table, without frame */
	static const uint16_t ber10k_by_rxqual_upper[] =
	    { 0, 20, 40, 80, 160, 320, 640, 1280 };
	static const uint16_t ber10k_by_rxqual_lower[] =
	    { 0, 0, 0, 20, 40, 80, 160, 320 };
	/* Note: The values in the upper vector are taken from the left side
	 * of the table in GSM 05.08, section 8.2.4. The lower vector is just
	 * the upper vector shifted by 2. */

	upper = ber10k_by_rxqual_upper[lchan->rep_acch_cap.rxqual];
	lower = ber10k_by_rxqual_lower[lchan->rep_acch_cap.rxqual];

	/* If upper/rxqual == 0, then repeated UL-SACCH is always on */
	if (ber10k >= upper)
		lchan->rep_acch.ul_sacch_active = true;
	else if (ber10k <= lower)
		lchan->rep_acch.ul_sacch_active = false;

out:
	if (lchan->rep_acch.ul_sacch_active == prev_repeated_ul_sacch_active)
		return;
	if (lchan->rep_acch.ul_sacch_active)
		LOGPLCHAN(lchan, DL1P, LOGL_DEBUG, "UL-SACCH repetition: inactive => active\n");
	else
		LOGPLCHAN(lchan, DL1P, LOGL_DEBUG, "UL-SACCH repetition: active => inactive\n");
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
	uint8_t tn;
	uint32_t fn;
	enum osmo_ph_pres_info_type pr_info = data_ind->pdch_presence_info;

	chan_nr = data_ind->chan_nr;
	link_id = data_ind->link_id;
	fn = data_ind->fn;
	tn = L1SAP_CHAN2TS(chan_nr);

	gsm_fn2gsmtime(&g_time, fn);

	DEBUGPGT(DL1P, &g_time, "Rx PH-DATA.ind chan_nr=%s link_id=0x%02x len=%d\n",
		 rsl_chan_nr_str(chan_nr), link_id, len);

	if (ts_is_pdch(&trx->ts[tn])) {
		lchan = get_lchan_by_chan_nr(trx, chan_nr);
		if (!lchan)
			LOGPGT(DL1P, LOGL_ERROR, &g_time, "No lchan for chan_nr=%s\n", rsl_chan_nr_str(chan_nr));
		if (lchan && lchan->loopback) {
			/* we are in loopback mode (for BER testing)
			 * mode and need to enqeue the frame to be
			 * returned in downlink */
			lchan_dl_tch_queue_enqueue(lchan, msg, 1);

			/* Return 1 to signal that we're still using msg
			 * and it should not be freed */
			return 1;
		}

		/* There can be no DATA.ind on PTCCH/U (rather RACH.ind instead), but some
		 * BTS models with buggy implementation may still be sending them to us. */
		if (L1SAP_IS_PTCCH(fn)) {
			LOGPGT(DL1P, LOGL_NOTICE, &g_time, "There can be no DATA.ind on PTCCH/U. "
			       "This is probably a bug of the BTS model you're using, please fix!\n");
			return -EINVAL;
		}

		/* Drop all data from incomplete UL block */
		if (pr_info != PRES_INFO_BOTH)
			len = 0;

		/* PDTCH / PACCH frame handling */
		pcu_tx_data_ind(&trx->ts[tn], PCU_IF_SAPI_PDTCH, fn, trx->arfcn,
				L1SAP_FN2MACBLOCK(fn), data, len, data_ind->rssi, data_ind->ber10k,
				data_ind->ta_offs_256bits/64, data_ind->lqual_cb);
		return 0;
	}

	lchan = get_active_lchan_by_chan_nr(trx, chan_nr);
	if (!lchan) {
		LOGPGT(DL1P, LOGL_ERROR, &g_time, "No lchan for chan_nr=%s\n", rsl_chan_nr_str(chan_nr));
		return 0;
	}

	/* The ph_data_param contained in the l1sap primitive may contain
	 * measurement data. If this data is present, forward it for
	 * processing */
	if (bts_internal_flag_get(trx->bts, BTS_INTERNAL_FLAG_MEAS_PAYLOAD_COMB))
		process_l1sap_meas_data(lchan, l1sap, PRIM_PH_DATA);

	if (L1SAP_IS_LINK_SACCH(link_id)) {
		repeated_ul_sacch_active_decision(lchan, data_ind->ber10k);

		/* Radio Link Timeout counter */
		if (len == 0) {
			LOGPLCGT(lchan, &g_time, DL1P, LOGL_INFO, "Lost SACCH block\n");
			radio_link_timeout(lchan, true);
		} else {
			radio_link_timeout(lchan, false);
		}

		/* Trigger the measurement reporting/processing logic */
		lchan_meas_handle_sacch(lchan, msg);
	}

	/* bad frame */
	if (len == 0)
		return -EINVAL;

	/* report first valid received frame to handover process */
	if (lchan->ho.active == HANDOVER_WAIT_FRAME)
		handover_frame(lchan);

	/* report first valid received frame to VGCS talker process */
	if (rsl_chan_rt_is_asci(lchan->rsl_chan_rt) && lchan->asci.talker_active == VGCS_TALKER_WAIT_FRAME)
		vgcs_talker_frame(lchan);

	if (L1SAP_IS_LINK_SACCH(link_id))
		le = &lchan->lapdm_ch.lapdm_acch;
	else
		le = &lchan->lapdm_ch.lapdm_dcch;

	if (check_for_first_ciphrd(lchan, data, len))
		l1sap_tx_ciph_req(lchan->ts->trx, chan_nr, 1, 0);

	/* SDCCH, SACCH and FACCH all go to LAPDm */
	msgb_pull_to_l2(msg);
	lapdm_phsap_up(&l1sap->oph, le);

	/* don't free, because we forwarded data */
	return 1;
}

static void send_ul_rtp_packet_data(struct gsm_lchan *lchan, uint32_t fn,
				    const uint8_t *data, uint16_t data_len)
{
	uint8_t rtp_pl[RFC4040_RTP_PLEN];
	int rc;

	rc = csd_v110_rtp_encode(lchan, &rtp_pl[0], data, data_len);
	if (rc < 0)
		return;

	osmo_rtp_send_frame_ext(lchan->abis_ip.rtp_socket,
				&rtp_pl[0], sizeof(rtp_pl),
				fn_ms_adj(fn, lchan),
				lchan->rtp_tx_marker);
	/* Only clear the marker bit once we have sent a RTP packet with it */
	lchan->rtp_tx_marker = false;
}

/* a helper function for the logic in l1sap_tch_ind() */
static void send_ul_rtp_packet_speech(struct gsm_lchan *lchan, uint32_t fn,
				      const uint8_t *rtp_pl, uint16_t rtp_pl_len)
{
	if (lchan->abis_ip.osmux.use) {
		lchan_osmux_send_frame(lchan, rtp_pl, rtp_pl_len,
				       fn_ms_adj(fn, lchan), lchan->rtp_tx_marker);
	} else if (lchan->abis_ip.rtp_socket) {
		osmo_rtp_send_frame_ext(lchan->abis_ip.rtp_socket,
			rtp_pl, rtp_pl_len, fn_ms_adj(fn, lchan), lchan->rtp_tx_marker);
	}
	/* Only clear the marker bit once we have sent a RTP packet with it */
	lchan->rtp_tx_marker = false;
}

/* a helper function for emitting HR1 UL in RFC 5993 format */
static void send_rtp_rfc5993(struct gsm_lchan *lchan, uint32_t fn,
			     struct msgb *msg)
{
	uint8_t toc;

	OSMO_ASSERT(msg->len == GSM_HR_BYTES);
	/* FIXME: implement proper SID classification per GSM 06.41 section
	 * 6.1.1; see OS#6036.  Until then, detect error-free SID frames
	 * using our existing osmo_hr_check_sid() function. */
	if (osmo_hr_check_sid(msg->data, msg->len))
		toc = 0x20;
	else
		toc = 0x00;
	msgb_push_u8(msg, toc);
	send_ul_rtp_packet_speech(lchan, fn, msg->data, msg->len);
}

/* A helper function for l1sap_tch_ind(): handling BFI
 *
 * Please note that we pass the msgb to this function, even though it is
 * currently not used.  This msgb passing is a provision for adding
 * support for TRAU-UL-like RTP payload formats like TW-TS-001 that allow
 * indicating BFI along with deemed-bad frame data bits, just like
 * GSM 08.60 and 08.61 TRAU-UL frames.
 */
static void tch_ul_bfi_handler(struct gsm_lchan *lchan,
			       const struct gsm_time *g_time, struct msgb *msg)
{
	uint32_t fn = g_time->fn;
	uint8_t ecu_out[GSM_FR_BYTES];
	int rc;

	/* Are we applying an ECU to this uplink, and are we in a state
	 * (not DTX pause) where we emit ECU output? */
	if (lchan->ecu_state && !osmo_ecu_is_dtx_pause(lchan->ecu_state)) {
		rc = osmo_ecu_frame_out(lchan->ecu_state, ecu_out);
		/* did it actually give us some output? */
		if (rc > 0) {
			/* yes, send it out in RTP */
			send_ul_rtp_packet_speech(lchan, fn, ecu_out, rc);
			return;
		}
	}

	/* Are we in rtp continuous-streaming special mode? If so, send out
	 * a BFI packet as zero-length RTP payload. */
	if (lchan->ts->trx->bts->rtp_nogaps_mode) {
		send_ul_rtp_packet_speech(lchan, fn, NULL, 0);
		return;
	}

	/* Most classic form of BFI handling: generate an intentional gap
	 * in the outgoing RTP stream. */
	LOGPLCGT(lchan, g_time, DRTP, LOGL_DEBUG,
		 "Skipping RTP frame with lost payload\n");
	if (lchan->abis_ip.osmux.use)
		lchan_osmux_skipped_frame(lchan, fn_ms_adj(fn, lchan));
	else if (lchan->abis_ip.rtp_socket)
		osmo_rtp_skipped_frame(lchan->abis_ip.rtp_socket, fn_ms_adj(fn, lchan));
	lchan->rtp_tx_marker = true;
}

/* TCH received from bts model */
static int l1sap_tch_ind(struct gsm_bts_trx *trx, struct osmo_phsap_prim *l1sap,
	struct ph_tch_param *tch_ind)
{
	struct gsm_bts *bts = trx->bts;
	struct msgb *msg = l1sap->oph.msg;
	struct gsm_time g_time;
	struct gsm_lchan *lchan;
	uint8_t  chan_nr;
	uint32_t fn;

	chan_nr = tch_ind->chan_nr;
	fn = tch_ind->fn;

	gsm_fn2gsmtime(&g_time, fn);

	lchan = get_active_lchan_by_chan_nr(trx, chan_nr);
	if (!lchan) {
		LOGPGT(DL1P, LOGL_ERROR, &g_time, "No lchan for TCH.ind (chan_nr=%s)\n", rsl_chan_nr_str(chan_nr));
		return 0;
	} else {
		LOGPLCGT(lchan, &g_time, DL1P, LOGL_DEBUG, "Rx TCH.ind\n");
	}

	/* The ph_tch_param contained in the l1sap primitive may contain
	 * measurement data. If this data is present, forward it for
	 * processing */
	if (bts_internal_flag_get(trx->bts, BTS_INTERNAL_FLAG_MEAS_PAYLOAD_COMB))
		process_l1sap_meas_data(lchan, l1sap, PRIM_TCH);

	msgb_pull_to_l2(msg);

	/* Low level layers always call us when TCH content is expected, even if
	 * the content is not available due to decoding issues. Content not
	 * available is expected as empty payload. We also check if quality is
	 * good enough. */
	if (msg->len && tch_ind->lqual_cb >= bts->min_qual_norm) {
		/* feed the good frame to the ECU, if we are applying one */
		if (lchan->ecu_state)
			osmo_ecu_frame_in(lchan->ecu_state, false, msg->data, msg->len);
		/* hand msg to RTP code for transmission */
		switch (lchan->rsl_cmode) {
		case RSL_CMOD_SPD_SPEECH:
			if (bts->emit_hr_rfc5993 && lchan->type == GSM_LCHAN_TCH_H &&
			    lchan->tch_mode == GSM48_CMODE_SPEECH_V1)
				send_rtp_rfc5993(lchan, fn, msg);
			else
				send_ul_rtp_packet_speech(lchan, fn, msg->data, msg->len);
			break;
		case RSL_CMOD_SPD_DATA:
			send_ul_rtp_packet_data(lchan, fn, msg->data, msg->len);
			break;
		case RSL_CMOD_SPD_SIGN:
		default: /* shall not happen */
			OSMO_ASSERT(0);
		}
		/* if loopback is enabled, also queue received RTP data */
		if (lchan->loopback) {
			/* add new frame to queue, make sure the queue doesn't get too long */
			lchan_dl_tch_queue_enqueue(lchan, msg, 1);
			/* Return 1 to signal that we're still using msg and it should not be freed */
			return 1;
		}
	} else {
		tch_ul_bfi_handler(lchan, &g_time, msg);
	}

	lchan->tch.last_fn = fn;
	return 0;
}

#define RACH_MIN_TOA256 -2 * 256

static bool rach_pass_filter(struct ph_rach_ind_param *rach_ind, struct gsm_bts *bts,
			     const char *chan_name)
{
	int16_t toa256 = rach_ind->acc_delay_256bits;

	/* Check for RACH exceeding BER threshold (ghost RACH) */
	if (rach_ind->ber10k > bts->max_ber10k_rach) {
		LOGPFN(DL1C, LOGL_DEBUG, rach_ind->fn, "Ignoring an Access Burst on %s: "
			"BER10k(%u) > BER10k_MAX(%u)\n", chan_name,
			rach_ind->ber10k, bts->max_ber10k_rach);
		return false;
	}

	/**
	 * Make sure that ToA (Timing of Arrival) is acceptable.
	 * We allow early arrival up to 2 symbols, and delay
	 * according to maximal allowed Timing Advance value.
	 */
	if (toa256 < RACH_MIN_TOA256 || toa256 > bts->max_ta * 256) {
		LOGPFN(DL1C, LOGL_DEBUG, rach_ind->fn, "Ignoring an Access Burst on %s: "
			"ToA(%d) exceeds the allowed range (%d..%d)\n", chan_name,
			toa256, RACH_MIN_TOA256, bts->max_ta * 256);
		return false;
	}

	/* Link quality defined by C/I (Carrier-to-Interference ratio) */
	if (rach_ind->lqual_cb < bts->min_qual_rach) {
		LOGPFN(DL1C, LOGL_DEBUG, rach_ind->fn, "Ignoring an Access Burst on %s: "
			"link quality (%d) below the minimum (%d)\n", chan_name,
			rach_ind->lqual_cb, bts->min_qual_rach);
		return false;
	}

	return true;
}

/* Special case where RACH on DCCH uplink is detected */
static int l1sap_dcch_rach(struct gsm_bts_trx *trx, struct ph_rach_ind_param *rach_ind)
{
	struct gsm_lchan *lchan;

	/* Filter out noise / interference / ghosts */
	if (!rach_pass_filter(rach_ind, trx->bts, "DCCH")) {
		rate_ctr_inc2(trx->bts->ctrs, BTS_CTR_RACH_DROP);
		return 0;
	}

	lchan = get_lchan_by_chan_nr(trx, rach_ind->chan_nr);
	/* Differentiate + dispatch hand-over and VGCS RACH */
	if (rsl_chan_rt_is_asci(lchan->rsl_chan_rt)) {
		rate_ctr_inc2(trx->bts->ctrs, BTS_CTR_RACH_VGCS);
		vgcs_rach(lchan, rach_ind->ra, rach_ind->acc_delay, rach_ind->fn);
	} else {
		rate_ctr_inc2(trx->bts->ctrs, BTS_CTR_RACH_HO);
		handover_rach(lchan, rach_ind->ra, rach_ind->acc_delay);
	}

	/* must return 0, so in case of msg at l1sap, it will be freed */
	return 0;
}

/* Special case for Access Bursts on PDTCH/U or PTCCH/U */
static int l1sap_pdch_rach(struct gsm_bts_trx *trx, struct ph_rach_ind_param *rach_ind)
{
	/* Filter out noise / interference / ghosts */
	if (!rach_pass_filter(rach_ind, trx->bts, "PDCH"))
		return -EAGAIN;

	/* PTCCH/U (Packet Timing Advance Control Channel) */
	if (L1SAP_IS_PTCCH(rach_ind->fn)) {
		LOGPFN(DL1P, LOGL_DEBUG, rach_ind->fn,
		       /* TODO: calculate and print Timing Advance Index */
		       "Access Burst for continuous Timing Advance control (toa256=%d)\n",
		       rach_ind->acc_delay_256bits);

		/* QTA: Timing Advance in units of 1/4 of a symbol */
		pcu_tx_rach_ind(trx->bts->nr, trx->nr, rach_ind->chan_nr & 0x07,
				rach_ind->acc_delay_256bits >> 6,
				rach_ind->ra, rach_ind->fn, rach_ind->is_11bit,
				rach_ind->burst_type, PCU_IF_SAPI_PTCCH);
		return 0;
	} else { /* The MS may acknowledge DL data by 4 consequent Access Bursts */
		LOGPFN(DL1P, LOGL_NOTICE, rach_ind->fn,
		       "Access Bursts on PDTCH/U are not (yet) supported\n");
		return -ENOTSUP;
	}
}

/* RACH received from bts model */
static int l1sap_ph_rach_ind(struct gsm_bts_trx *trx,
	 struct osmo_phsap_prim *l1sap, struct ph_rach_ind_param *rach_ind)
{
	struct gsm_bts *bts = trx->bts;
	struct lapdm_channel *lc;

	DEBUGPFN(DL1P, rach_ind->fn, "Rx PH-RA.ind\n");

	/* Check the origin of an Access Burst */
	switch (rach_ind->chan_nr & 0xf8) {
	case RSL_CHAN_RACH:
		/* CS or PS RACH, to be handled in this function */
		break;
	case RSL_CHAN_OSMO_PDCH:
		/* TODO: do we need to count Access Bursts on PDCH? */
		return l1sap_pdch_rach(trx, rach_ind);
	default:
		return l1sap_dcch_rach(trx, rach_ind);
	}

	rate_ctr_inc2(trx->bts->ctrs, BTS_CTR_RACH_RCVD);

	/* increment number of busy RACH slots, if required */
	if (rach_ind->rssi >= bts->load.rach.busy_thresh)
		bts->load.rach.busy++;

	/* Filter out noise / interference / ghosts */
	if (!rach_pass_filter(rach_ind, bts, "CCCH")) {
		rate_ctr_inc2(trx->bts->ctrs, BTS_CTR_RACH_DROP);
		return 0;
	}

	/* increment number of RACH slots with valid non-handover RACH burst */
	bts->load.rach.access++;

	lc = &trx->ts[0].lchan[CCCH_LCHAN].lapdm_ch;

	/* According to 3GPP TS 48.058 § 9.3.17 Access Delay is expressed same way as TA (number of symbols) */
	set_ms_to_data(get_lchan_by_chan_nr(trx, rach_ind->chan_nr),
		rach_ind->acc_delay, false);

	/* check for packet access */
	if ((trx == bts->c0 && L1SAP_IS_PACKET_RACH(rach_ind->ra)) ||
		(trx == bts->c0 && rach_ind->is_11bit)) {
		rate_ctr_inc2(trx->bts->ctrs, BTS_CTR_RACH_PS);

		LOGPFN(DL1P, LOGL_INFO, rach_ind->fn, "RACH for packet access (toa=%d, ra=%d)\n",
			rach_ind->acc_delay, rach_ind->ra);

		/* QTA: Timing Advance in units of 1/4 of a symbol */
		pcu_tx_rach_ind(bts->nr, trx->nr, rach_ind->chan_nr & 0x07,
			rach_ind->acc_delay_256bits >> 6,
			rach_ind->ra, rach_ind->fn, rach_ind->is_11bit,
			rach_ind->burst_type, PCU_IF_SAPI_RACH);
		return 0;
	}

	LOGPFN(DL1P, LOGL_INFO, rach_ind->fn, "RACH for RR access (toa=%d, ra=%d)\n",
		rach_ind->acc_delay, rach_ind->ra);
	rate_ctr_inc2(trx->bts->ctrs, BTS_CTR_RACH_CS);
	lapdm_phsap_up(&l1sap->oph, &lc->lapdm_dcch);

	return 0;
}

/* Process any L1 prim received from bts model.
 *
 * This function takes ownership of the msgb.
 * If l1sap contains a msgb, it assumes that msgb->l2h was set by lower layer.
 */
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
		oml_tx_failure_event_rep(&trx->mo, NM_SEVER_MAJOR, OSMO_EVT_MAJ_UKWN_MSG,
					 "unknown prim %d op %d",
					 l1sap->oph.primitive,
					 l1sap->oph.operation);
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
	l1sap_log_ctx_sapi = get_common_sapi_by_trx_prim(trx, l1sap);
	log_set_context(LOG_CTX_L1_SAPI, &l1sap_log_ctx_sapi);

	if (OSMO_PRIM_HDR(&l1sap->oph) ==
				 OSMO_PRIM(PRIM_PH_DATA, PRIM_OP_REQUEST))
		to_gsmtap(trx, l1sap);

	return bts_model_l1sap_down(trx, l1sap);
}

/* pcu (socket interface) sends us a data request primitive */
int l1sap_pdch_req(struct gsm_bts_trx_ts *ts, int is_ptcch, uint32_t fn,
	uint16_t arfcn, uint8_t block_nr, const uint8_t *data, uint8_t len)
{
	struct msgb *msg;
	struct osmo_phsap_prim *l1sap;
	struct gsm_time g_time;

	gsm_fn2gsmtime(&g_time, fn);

	DEBUGP(DL1P, "TX packet data %s is_ptcch=%d trx=%d ts=%d "
		"block_nr=%d, arfcn=%d, len=%d\n", osmo_dump_gsmtime(&g_time),
		is_ptcch, ts->trx->nr, ts->nr, block_nr, arfcn, len);

	msg = l1sap_msgb_alloc(len);
	l1sap = msgb_l1sap_prim(msg);
	osmo_prim_init(&l1sap->oph, SAP_GSM_PH, PRIM_PH_DATA, PRIM_OP_REQUEST,
		msg);
	l1sap->u.data.chan_nr = RSL_CHAN_OSMO_PDCH | ts->nr;
	l1sap->u.data.link_id = 0x00;
	l1sap->u.data.fn = fn;
	if (len) {
		msg->l2h = msgb_put(msg, len);
		memcpy(msg->l2h, data, len);
	} else {
		msg->l2h = NULL; /* Idle block */
	}

	return l1sap_down(ts->trx, l1sap);
}

/*! \brief call-back function for incoming RTP */
void l1sap_rtp_rx_cb(struct osmo_rtp_socket *rs, const uint8_t *rtp_pl,
                     unsigned int rtp_pl_len, uint16_t seq_number,
		     uint32_t timestamp, bool marker)
{
	struct gsm_lchan *lchan = rs->priv;
	struct msgb *msg;
	bool rfc5993_sid = false;

	/* if we're in loopback mode, we don't accept frames from the
	 * RTP socket anymore */
	if (lchan->loopback)
		return;

	/* initial preen */
	switch (rtp_payload_input_preen(lchan, rtp_pl, rtp_pl_len, &rfc5993_sid)) {
	case PL_DECISION_DROP:
		return;
	case PL_DECISION_ACCEPT:
		break;
	case PL_DECISION_STRIP_HDR_OCTET:
		rtp_pl++;
		rtp_pl_len--;
		break;
	default:
		OSMO_ASSERT(0);
	}

	msg = l1sap_msgb_alloc(512);
	if (!msg)
		return;

	if (lchan->rsl_cmode == RSL_CMOD_SPD_DATA) {
		int rc = csd_v110_rtp_decode(lchan, msg->tail,
					     rtp_pl, rtp_pl_len);
		if (rc > 0) {
			msgb_put(msg, rc);
		} else {
			msgb_free(msg);
			return;
		}
	} else {
		memcpy(msgb_put(msg, rtp_pl_len), rtp_pl, rtp_pl_len);
	}

	msgb_pull(msg, sizeof(struct osmo_phsap_prim));

	/* Store RTP header Marker bit in control buffer */
	rtpmsg_marker_bit(msg) = marker;
	/* Store RTP header Sequence Number in control buffer */
	rtpmsg_seq(msg) = seq_number;
	/* Store RTP header Timestamp in control buffer */
	rtpmsg_ts(msg) = timestamp;
	/* Store RFC 5993 SID flag likewise */
	rtpmsg_is_rfc5993_sid(msg) = rfc5993_sid;

	/* make sure the queue doesn't get too long */
	lchan_dl_tch_queue_enqueue(lchan, msg, 1);
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

int l1sap_chan_act(struct gsm_bts_trx *trx, uint8_t chan_nr)
{
	struct gsm_lchan *lchan = get_lchan_by_chan_nr(trx, chan_nr);
	int rc;

	if (lchan->state == LCHAN_S_ACTIVE) {
		LOGPLCHAN(lchan, DL1C, LOGL_ERROR, "Trying to activate already active channel %s\n",
			  rsl_chan_nr_str(chan_nr));
		return -1;
	}

	LOGPLCHAN(lchan, DL1C, LOGL_INFO, "Activating channel %s\n", rsl_chan_nr_str(chan_nr));

	radio_link_timeout_reset(lchan);

	rc = l1sap_chan_act_dact_modify(trx, chan_nr, PRIM_INFO_ACTIVATE, 0);
	if (rc)
		return -RSL_ERR_EQUIPMENT_FAIL;

	/* Is it TCH?  If it is, attempt to allocate an Error Concealment Unit
	 * instance, if available, unless it is disabled by vty config. */
	if (lchan_is_tch(lchan) && trx->bts->use_ul_ecu)
		lchan->ecu_state = osmo_ecu_init(trx, lchan2ecu_codec(lchan));
	else
		lchan->ecu_state = NULL;

	/* Init DTX DL FSM if necessary */
	if (trx->bts->dtxd && lchan_is_tch(lchan)) {
		lchan->tch.dtx.dl_amr_fsm = osmo_fsm_inst_alloc(&dtx_dl_amr_fsm,
								tall_bts_ctx,
								lchan,
								LOGL_DEBUG,
								NULL);
		if (!lchan->tch.dtx.dl_amr_fsm) {
			l1sap_chan_act_dact_modify(trx, chan_nr, PRIM_INFO_DEACTIVATE, 0);
			return -RSL_ERR_EQUIPMENT_FAIL;
		}

		rc = osmo_fsm_inst_update_id_f(lchan->tch.dtx.dl_amr_fsm,
					       "bts%u-trx%u-ts%u-ss%u%s",
					       trx->bts->nr, trx->nr,
					       lchan->ts->nr, lchan->nr,
					       lchan->ts->vamos.is_shadow ? "-shadow" : "");
		OSMO_ASSERT(rc == 0);
	}
	return 0;
}

int l1sap_chan_rel(struct gsm_bts_trx *trx, uint8_t chan_nr)
{
	struct gsm_lchan *lchan = get_lchan_by_chan_nr(trx, chan_nr);

	if (lchan->state == LCHAN_S_NONE) {
		LOGPLCHAN(lchan, DL1C, LOGL_ERROR, "Trying to deactivate already deactivated channel %s\n",
			  rsl_chan_nr_str(chan_nr));
		return -1;
	}

	LOGPLCHAN(lchan, DL1C, LOGL_INFO, "Deactivating channel %s\n",
		  rsl_chan_nr_str(chan_nr));

	if (lchan->tch.dtx.dl_amr_fsm) {
		osmo_fsm_inst_free(lchan->tch.dtx.dl_amr_fsm);
		lchan->tch.dtx.dl_amr_fsm = NULL;
	}

	/* clear ECU state (if any) */
	if (lchan->ecu_state) {
		osmo_ecu_destroy(lchan->ecu_state);
		lchan->ecu_state = NULL;
	}

	return l1sap_chan_act_dact_modify(trx, chan_nr, PRIM_INFO_DEACTIVATE,
		0);
}

int l1sap_chan_deact_sacch(struct gsm_bts_trx *trx, uint8_t chan_nr)
{
	struct gsm_lchan *lchan = get_lchan_by_chan_nr(trx, chan_nr);

	LOGPLCHAN(lchan, DL1C, LOGL_INFO, "Deactivating SACCH on channel %s\n",
		  rsl_chan_nr_str(chan_nr));

	return l1sap_chan_act_dact_modify(trx, chan_nr, PRIM_INFO_DEACTIVATE,
		1);
}

int l1sap_chan_modify(struct gsm_bts_trx *trx, uint8_t chan_nr)
{
	struct gsm_lchan *lchan = get_lchan_by_chan_nr(trx, chan_nr);

	LOGPLCHAN(lchan, DL1C, LOGL_INFO, "Modifying channel %s\n",
		rsl_chan_nr_str(chan_nr));

	/* Is it TCH?  If it is and we are applying internal uplink ECUs,
	 * the new channel mode calls for a different ECU.  Any changes
	 * in vty config (enabling or disabling this ECU application)
	 * will also take effect upon channel modification. */
	if (lchan_is_tch(lchan)) {
		if (lchan->ecu_state)
			osmo_ecu_destroy(lchan->ecu_state);
		if (trx->bts->use_ul_ecu)
			lchan->ecu_state = osmo_ecu_init(trx, lchan2ecu_codec(lchan));
		else
			lchan->ecu_state = NULL;
	}

	return l1sap_chan_act_dact_modify(trx, chan_nr, PRIM_INFO_MODIFY, 0);
}
