/* (C) 2014 by sysmocom s.f.m.c. GmbH
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

#include <osmo-bts/dtx_dl_amr_fsm.h>
#include <osmo-bts/msg_utils.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/oml.h>
#include <osmo-bts/amr.h>

#include <osmocom/gsm/protocol/ipaccess.h>
#include <osmocom/gsm/protocol/gsm_12_21.h>
#include <osmocom/gsm/abis_nm.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/fsm.h>
#include <osmocom/trau/osmo_ortp.h>

#include <arpa/inet.h>
#include <errno.h>

static int check_fom(struct abis_om_hdr *omh, size_t len)
{
	if (omh->length != len) {
		LOGP(DL1C, LOGL_ERROR, "Incorrect OM hdr length value %d %zu\n",
		     omh->length, len);
		return -1;
	}

	if (len < sizeof(struct abis_om_fom_hdr)) {
		LOGP(DL1C, LOGL_ERROR, "FOM header insufficient space %zu %zu\n",
		     len, sizeof(struct abis_om_fom_hdr));
		return -1;
	}

	return 0;
}

static int check_manuf(struct msgb *msg, struct abis_om_hdr *omh, size_t msg_size)
{
	int type;
	size_t size;

	if (msg_size < 1) {
		LOGP(DL1C, LOGL_ERROR, "No ManId Length Indicator %zu\n",
		     msg_size);
		return -1;
	}

	if (omh->data[0] >= msg_size - 1) {
		LOGP(DL1C, LOGL_ERROR,
		     "Insufficient message space for this ManId Length %d %zu\n",
		     omh->data[0], msg_size - 1);
		return -1;
	}

	if (omh->data[0] == sizeof(abis_nm_ipa_magic) &&
		strncmp(abis_nm_ipa_magic, (const char *)omh->data + 1,
			sizeof(abis_nm_ipa_magic)) == 0) {
		type = OML_MSG_TYPE_IPA;
		size = sizeof(abis_nm_ipa_magic) + 1;
	} else if (omh->data[0] == sizeof(abis_nm_osmo_magic) &&
		strncmp(abis_nm_osmo_magic, (const char *) omh->data + 1,
			sizeof(abis_nm_osmo_magic)) == 0) {
		type = OML_MSG_TYPE_OSMO;
		size = sizeof(abis_nm_osmo_magic) + 1;
	} else {
		LOGP(DL1C, LOGL_ERROR, "Manuf Label Unknown\n");
		return -1;
	}

	/* we have verified that the vendor string fits */
	msg->l3h = omh->data + size;
	if (check_fom(omh, msgb_l3len(msg)) != 0)
		return -1;
	return type;
}

/* update lchan SID status */
void lchan_set_marker(bool t, struct gsm_lchan *lchan)
{
	if (t)
		lchan->tch.dtx.ul_sid = true;
	else if (lchan->tch.dtx.ul_sid) {
		lchan->tch.dtx.ul_sid = false;
		lchan->rtp_tx_marker = true;
	}
}

/*! \brief Store the last SID frame in lchan context
 *  \param[in] lchan Logical channel on which we check scheduling
 *  \param[in] l1_payload buffer with SID data
 *  \param[in] length length of l1_payload
 *  \param[in] fn Frame Number for which we check scheduling
 *  \param[in] update 0 if SID_FIRST, 1 if SID_UPDATE, -1 if not AMR SID
 */
void dtx_cache_payload(struct gsm_lchan *lchan, const uint8_t *l1_payload,
		       size_t length, uint32_t fn, int update)
{
	size_t amr = (update < 0) ? 0 : 2,
		copy_len = OSMO_MIN(length + 1,
				ARRAY_SIZE(lchan->tch.dtx.cache) - amr);

	lchan->tch.dtx.len = copy_len + amr;
	lchan->tch.dtx.fn = fn;
	lchan->tch.dtx.is_update = update;

	memcpy(lchan->tch.dtx.cache + amr, l1_payload, copy_len);
}

/*! \brief Check current state of DTX DL AMR FSM and dispatch necessary events
 *  \param[in] lchan Logical channel on which we check scheduling
 *  \param[in] rtp_pl buffer with RTP data
 *  \param[in] rtp_pl_len length of rtp_pl
 *  \param[in] fn Frame Number for which we check scheduling
 *  \param[in] l1_payload buffer where CMR and CMI prefix should be added
 *  \param[out] len Length of expected L1 payload
 *  \param[out] ft_out Frame Type to be populated after decoding
 *  \returns 0 in case of success; negative on error
 */
int dtx_dl_amr_fsm_step(struct gsm_lchan *lchan, const uint8_t *rtp_pl,
			size_t rtp_pl_len, uint32_t fn, uint8_t *l1_payload,
			bool marker, uint8_t *len, uint8_t *ft_out)
{
	uint8_t cmr;
	enum osmo_amr_type ft;
	enum osmo_amr_quality bfi;
	int8_t sti, cmi;
	int rc;

	if (rtp_pl == NULL) { /* SID-FIRST P1 -> P2 */
		*len = 3;
		memcpy(l1_payload, lchan->tch.dtx.cache, 2);
		osmo_fsm_inst_dispatch(lchan->tch.dtx.dl_amr_fsm, E_COMPL,
				       (void *)lchan);
		return 0;
	}

	rc = osmo_amr_rtp_dec(rtp_pl, rtp_pl_len, &cmr, &cmi, &ft, &bfi, &sti);
	if (rc < 0) {
		LOGP(DRTP, LOGL_ERROR, "failed to decode AMR RTP (length %zu)\n",
		     rtp_pl_len);
		return rc;
	}

	/* only needed for old sysmo firmware: */
	*ft_out = ft;

	/* CMI in downlink tells the L1 encoder which encoding function
	 * it will use, so we have to use the frame type */
	if (osmo_amr_is_speech(ft))
		cmi = ft;

	/* populate L1 payload with CMR/CMI - might be ignored by caller: */
	amr_set_mode_pref(l1_payload, &lchan->tch.amr_mr, cmi, cmr);

	/* populate DTX cache with CMR/CMI - overwrite cache which will be
	   either updated or invalidated by caller anyway: */
	amr_set_mode_pref(lchan->tch.dtx.cache, &lchan->tch.amr_mr, cmi, cmr);
	*len = 3 + rtp_pl_len;

	/* DTX DL is not enabled, move along */
	if (!lchan->ts->trx->bts->dtxd)
		return 0;

	if (osmo_amr_is_speech(ft)) {
		if (lchan->tch.dtx.dl_amr_fsm->state == ST_SID_F1 ||
		    lchan->tch.dtx.dl_amr_fsm->state == ST_SID_U) /* AMR HR */
			if (lchan->type == GSM_LCHAN_TCH_H && marker)
				return osmo_fsm_inst_dispatch(lchan->tch.dtx.dl_amr_fsm,
							      E_INHIB,
							      (void *)lchan);
		/* AMR FR */
		if (marker && lchan->tch.dtx.dl_amr_fsm->state == ST_SID_U)
			return osmo_fsm_inst_dispatch(lchan->tch.dtx.dl_amr_fsm,
						      E_ONSET, (void *)lchan);
		return osmo_fsm_inst_dispatch(lchan->tch.dtx.dl_amr_fsm, E_VOICE,
					      (void *)lchan);
	}

	if (ft == AMR_SID) {
		dtx_cache_payload(lchan, rtp_pl, rtp_pl_len, fn, sti);
		return osmo_fsm_inst_dispatch(lchan->tch.dtx.dl_amr_fsm,
					      sti ? E_SID_U : E_SID_F,
					      (void *)lchan);
	}

	if (ft != AMR_NO_DATA) {
		LOGP(DRTP, LOGL_ERROR, "unsupported AMR FT 0x%02x\n", ft);
		return -ENOTSUP;
	}

	if (marker)
		osmo_fsm_inst_dispatch(lchan->tch.dtx.dl_amr_fsm, E_VOICE,
				       (void *)lchan);
	*len = 0;
	return 0;
}

/*! \brief Check if enough time has passed since last SID (if any) to repeat it
 *  \param[in] lchan Logical channel on which we check scheduling
 *  \param[in] fn Frame Number for which we check scheduling
 *  \returns true if transmission can be omitted, false otherwise
 */
static inline bool dtx_amr_sid_optional(const struct gsm_lchan *lchan,
					uint32_t fn)
{
	/* Compute approx. time delta based on Fn duration */
	uint32_t delta = GSM_FN_TO_MS(fn - lchan->tch.dtx.fn);

	/* according to 3GPP TS 26.093 A.5.1.1: */
	if (lchan->tch.dtx.is_update) {
		/* SID UPDATE should be repeated every 8th RTP frame */
		if (delta < GSM_RTP_FRAME_DURATION_MS * 8)
			return true;
		return false;
	}
	/* 3rd frame after SID FIRST should be SID UPDATE */
	if (delta < GSM_RTP_FRAME_DURATION_MS * 3)
		return true;
	return false;
}

static inline bool fn_chk(const uint8_t *t, uint32_t fn, uint8_t len)
{
	uint8_t i;
	for (i = 0; i < len; i++)
		if (fn % 104 == t[i])
			return false;
	return true;
}

/*! \brief Check if TX scheduling is optional for a given FN in case of DTX
 *  \param[in] lchan Logical channel on which we check scheduling
 *  \param[in] fn Frame Number for which we check scheduling
 *  \returns true if transmission can be omitted, false otherwise
 */
static inline bool dtx_sched_optional(struct gsm_lchan *lchan, uint32_t fn)
{
	/* According to 3GPP TS 45.008 § 8.3: */
	static const uint8_t f[] = { 52, 53, 54, 55, 56, 57, 58, 59 },
				h0[] = { 0, 2, 4, 6, 52, 54, 56, 58 },
				h1[] = { 14, 16, 18, 20, 66, 68, 70, 72 };
	if (lchan->tch_mode == GSM48_CMODE_SPEECH_V1) {
		if (lchan->type == GSM_LCHAN_TCH_F)
			return fn_chk(f, fn, ARRAY_SIZE(f));
		else
			return fn_chk(lchan->nr ? h1 : h0, fn,
				      lchan->nr ? ARRAY_SIZE(h1) :
				      ARRAY_SIZE(h0));
	}
	return false;
}

/* repeat last SID if possible, returns SID length + 1 or 0 */
/*! \brief Repeat last SID if possible in case of DTX
 *  \param[in] lchan Logical channel on which we check scheduling
 *  \param[in] dst Buffer to copy last SID into
 *  \returns Number of bytes copied + 1 (to accommodate for extra byte with
 *           payload type), 0 if there's nothing to copy
 */
uint8_t repeat_last_sid(struct gsm_lchan *lchan, uint8_t *dst, uint32_t fn)
{
	/* FIXME: add EFR support */
	if (lchan->tch_mode == GSM48_CMODE_SPEECH_EFR)
		return 0;

	if (lchan->tch_mode != GSM48_CMODE_SPEECH_AMR) {
		if (dtx_sched_optional(lchan, fn))
			return 0;
	} else
		if (dtx_amr_sid_optional(lchan, fn))
			return 0;

	if (lchan->tch.dtx.len) {
		memcpy(dst, lchan->tch.dtx.cache, lchan->tch.dtx.len);
		lchan->tch.dtx.fn = fn;
		return lchan->tch.dtx.len + 1;
	}

	LOGP(DL1C, LOGL_DEBUG, "Have to send %s frame on TCH but SID buffer "
	     "is empty - sent nothing\n",
	     get_value_string(gsm48_chan_mode_names, lchan->tch_mode));

	return 0;
}

/**
 * Return 0 in case the IPA structure is okay and in this
 * case the l2h will be set to the beginning of the data.
 */
int msg_verify_ipa_structure(struct msgb *msg)
{
	struct ipaccess_head *hh;

	if (msgb_l1len(msg) < sizeof(struct ipaccess_head)) {
		LOGP(DL1C, LOGL_ERROR,
			"Ipa header insufficient space %d %zu\n",
			msgb_l1len(msg), sizeof(struct ipaccess_head));
		return -1;
	}

	hh = (struct ipaccess_head *) msg->l1h;

	if (ntohs(hh->len) != msgb_l1len(msg) - sizeof(struct ipaccess_head)) {
		LOGP(DL1C, LOGL_ERROR,
			"Incorrect ipa header msg size %d %zu\n",
			ntohs(hh->len), msgb_l1len(msg) - sizeof(struct ipaccess_head));
		return -1;
	}

	if (hh->proto == IPAC_PROTO_OSMO) {
		struct ipaccess_head_ext *hh_ext = (struct ipaccess_head_ext *) hh->data;
		if (ntohs(hh->len) < sizeof(*hh_ext)) {
			LOGP(DL1C, LOGL_ERROR, "IPA length shorter than OSMO header\n");
			return -1;
		}
		msg->l2h = hh_ext->data;
	} else
		msg->l2h = hh->data;

	return 0;
}

/**
 * \brief Verify the structure of the OML message and set l3h
 *
 * This function verifies that the data in \param in msg is a proper
 * OML message. This code assumes that msg->l2h points to the
 * beginning of the OML message. In the successful case the msg->l3h
 * will be set and will point to the FOM header. The value is undefined
 * in all other cases.
 *
 * \param msg The message to analyze starting from msg->l2h.
 * \return In case the structure is correct a positive number will be
 * returned and msg->l3h will point to the FOM. The number is a
 * classification of the vendor type of the message.
 */
int msg_verify_oml_structure(struct msgb *msg)
{
	struct abis_om_hdr *omh;

	if (msgb_l2len(msg) < sizeof(*omh)) {
		LOGP(DL1C, LOGL_ERROR, "Om header insufficient space %d %zu\n",
		     msgb_l2len(msg), sizeof(*omh));
		return -1;
	}

	omh = (struct abis_om_hdr *) msg->l2h;
	if (omh->mdisc != ABIS_OM_MDISC_FOM &&
	    omh->mdisc != ABIS_OM_MDISC_MANUF) {
		LOGP(DL1C, LOGL_ERROR, "Incorrect om mdisc value %x\n",
		     omh->mdisc);
		return -1;
	}

	if (omh->placement != ABIS_OM_PLACEMENT_ONLY) {
		LOGP(DL1C, LOGL_ERROR, "Incorrect om placement value %x %x\n",
		     omh->placement, ABIS_OM_PLACEMENT_ONLY);
		return -1;
	}

	if (omh->sequence != 0) {
		LOGP(DL1C, LOGL_ERROR, "Incorrect om sequence value %d\n",
		     omh->sequence);
		return -1;
	}

	if (omh->mdisc == ABIS_OM_MDISC_MANUF)
		return check_manuf(msg, omh, msgb_l2len(msg) - sizeof(*omh));

	msg->l3h = omh->data;
	if (check_fom(omh, msgb_l3len(msg)) != 0)
		return -1;
	return OML_MSG_TYPE_ETSI;
}
