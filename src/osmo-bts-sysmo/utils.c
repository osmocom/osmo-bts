/*
 * Helper utilities that are used in OML
 *
 * (C) 2011-2013 by Harald Welte <laforge@gnumonks.org>
 * (C) 2013 by Holger Hans Peter Freyther
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

#include "utils.h"

#include <osmo-bts/bts.h>
#include <osmo-bts/gsm_data.h>
#include <osmo-bts/logging.h>

#include <osmocom/core/msgb.h>
#include <osmocom/gsm/protocol/ipaccess.h>

#include "femtobts.h"
#include "l1_if.h"

int band_femto2osmo(GsmL1_FreqBand_t band)
{
	switch (band) {
	case GsmL1_FreqBand_850:
		return GSM_BAND_850;
	case GsmL1_FreqBand_900:
		return GSM_BAND_900;
	case GsmL1_FreqBand_1800:
		return GSM_BAND_1800;
	case GsmL1_FreqBand_1900:
		return GSM_BAND_1900;
	default:
		return -1;
	}
}

static int band_osmo2femto(struct gsm_bts_trx *trx, enum gsm_band osmo_band)
{
	struct femtol1_hdl *fl1h = trx_femtol1_hdl(trx);

	/* check if the TRX hardware actually supports the given band */
	if (!(fl1h->hw_info.band_support & osmo_band))
		return -1;

	/* if yes, convert from osmcoom style band definition to L1 band */
	switch (osmo_band) {
	case GSM_BAND_850:
		return GsmL1_FreqBand_850;
	case GSM_BAND_900:
		return GsmL1_FreqBand_900;
	case GSM_BAND_1800:
		return GsmL1_FreqBand_1800;
	case GSM_BAND_1900:
		return GsmL1_FreqBand_1900;
	default:
		return -1;
	}
}

/**
 * Select the band that matches the ARFCN. In general the ARFCNs
 * for GSM1800 and GSM1900 overlap and one needs to specify the
 * rightband. When moving between GSM900/GSM1800 and GSM850/1900
 * modifying the BTS configuration is a bit annoying. The auto-band
 * configuration allows to ease with this transition.
 */
int sysmobts_select_femto_band(struct gsm_bts_trx *trx, uint16_t arfcn)
{
	enum gsm_band band;
	struct gsm_bts *bts = trx->bts;
	struct gsm_bts_role_bts *btsb = bts_role_bts(bts);

	if (!btsb->auto_band)
		return band_osmo2femto(trx, bts->band);

	/*
	 * We need to check what will happen now.
	 */
	band = gsm_arfcn2band(arfcn);

	/* if we are already on the right band return */
	if (band == bts->band)
		return band_osmo2femto(trx, bts->band);

	/* Check if it is GSM1800/GSM1900 */
	if (band == GSM_BAND_1800 && bts->band == GSM_BAND_1900)
		return band_osmo2femto(trx, bts->band);

	/*
	 * Now to the actual autobauding. We just want DCS/DCS and
	 * PCS/PCS for PCS we check for 850/1800 though
	 */
	if ((band == GSM_BAND_900 && bts->band == GSM_BAND_1800)
		|| (band == GSM_BAND_1800 && bts->band == GSM_BAND_900)
		|| (band == GSM_BAND_850 && bts->band == GSM_BAND_1900))
		return band_osmo2femto(trx, band);
	if (band == GSM_BAND_1800 && bts->band == GSM_BAND_850)
		return band_osmo2femto(trx, GSM_BAND_1900);

	/* give up */
	return -1;
}

int sysmobts_get_nominal_power(struct gsm_bts_trx *trx)
{
	struct femtol1_hdl *fl1h = trx_femtol1_hdl(trx);

	switch (fl1h->hw_info.model_nr) {
	case 0:
	case 0xffff:
		/* old units have empty flash where the model number is
		 * stored in later units */
	case 1002:
		/* 200mW (23 dBm) nominal power */
		return 23;
	case 2050:
		/* 5W(39dBm) per TRX. This could be raiesd to 10W(40dBm)
		 * if the second TRX is not used. */
		return 37;
	default:
		LOGP(DL1C, LOGL_ERROR, "Model number %u/0x%x not known.\n",
			fl1h->hw_info.model_nr, fl1h->hw_info.model_nr);
		break;
	}
	return -1;
}

int sysmobts_get_power_trx(struct gsm_bts_trx *trx)
{
	int power_transmitter = trx->nominal_power - trx->max_power_red;
	power_transmitter -= trx->power_reduce;

	if (power_transmitter < 0)
		power_transmitter = 0;

	return power_transmitter;
}

void prepend_oml_ipa_header(struct msgb *msg)
{
	struct ipaccess_head *hh;

	hh = (struct ipaccess_head *) msgb_push(msg, sizeof(*hh));
	hh->proto = IPAC_PROTO_OML;
	hh->len = htons(msg->len - sizeof(struct ipaccess_head));
}

/**
 * \brief Check that the data in \param msg is a proper OML message
 *
 * This function verifies that the data in \param in msg is a proper
 * OML message and can be handled by later functions. In the successful
 * case the msg->l2h will now point to the OML header and the msg->l3h
 * will point to the FOM header. The value of l2h/l3h is undefined in
 * case the verification of the \param msg is failing.
 *
 * \param msg The message to analyze. msg->len starting from msg->data
 * will be analyzed.
 * \return This function returns the msg with the l2h/l3h pointers in the right
 * direction on success and on failure, in the case that the msg doesn't contain
 * the OML header or the OML header values aren't the expect, the function
 * doesn't set the l2h and l3h. In the case that the msg don't contains the FOM
 * header or the FOM header values aren't the expect, the function set the l2h
 * but doesn't set the l3h.
 */

int check_oml_msg(struct msgb *msg)
{
	struct abis_om_hdr *omh;
	int abis_oml_hdr_len;
	char label_id[255];

	abis_oml_hdr_len = sizeof(struct abis_om_hdr);

	if (msg->len < abis_oml_hdr_len) {
		LOGP(DL1C, LOGL_ERROR, "Om header insufficient space %d %d\n",
		     msg->len, abis_oml_hdr_len);
		return -1;
	}

	msg->l2h = msg->data;

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

	if (omh->length != sizeof(struct abis_om_fom_hdr)) {
		LOGP(DL1C, LOGL_ERROR, "Incorrect om length value %d %d\n",
		     omh->length, sizeof(struct abis_om_fom_hdr));
		return -1;
	}

	if (omh->mdisc == ABIS_OM_MDISC_MANUF) {
		abis_oml_hdr_len += sizeof(ipaccess_magic);

		if (msg->len < abis_oml_hdr_len) {
			LOGP(DL1C, LOGL_ERROR,
			     "ID manuf label insufficient space %d %d\n",
			     msg->len, abis_oml_hdr_len);
			return -1;
		}
	}

	abis_oml_hdr_len += sizeof(struct abis_om_fom_hdr);

	if (msg->len < abis_oml_hdr_len) {
		LOGP(DL1C, LOGL_ERROR, "Fom header insufficient space %d %d\n",
		     msg->len, abis_oml_hdr_len);
		return -1;
	}

	msg->l3h = msg->data + sizeof(struct abis_om_hdr);

	if (omh->mdisc == ABIS_OM_MDISC_MANUF) {
		strncpy(label_id, (const char *) msg->l3h + 1,
			sizeof(ipaccess_magic));

		if (strncmp(ipaccess_magic, label_id,
			    sizeof(ipaccess_magic)) == 0)
			msg->l3h = msg->l3h + sizeof(ipaccess_magic) + 1;
		else if (strncmp(osmocom_magic, label_id,
				 sizeof(osmocom_magic)) == 0)
			msg->l3h = msg->l3h + sizeof(osmocom_magic) + 1;
		else {
			msg->l3h = NULL;
			LOGP(DL1C, LOGL_ERROR,
			     "Manuf Label Unknown %s\n", label_id);
			return -1;
		}
	}

	return 0;
}

int check_ipa_header(struct msgb *msg)
{
	struct ipaccess_head *hh;

	if (msg->len < sizeof(struct ipaccess_head)) {
		LOGP(DL1C, LOGL_ERROR, "Ipa header insufficient space %d %d\n",
					msg->len, sizeof(struct ipaccess_head));
		return -1;
	}

	hh = (struct ipaccess_head *)msg->data;

	if (hh->proto != IPAC_PROTO_OML) {
		LOGP(DL1C, LOGL_ERROR, "Incorrect ipa header protocol %x %x\n",
		     hh->proto, IPAC_PROTO_OML);
		return -1;
	}

	if (ntohs(hh->len) != msg->len - sizeof(struct ipaccess_head)) {
		LOGP(DL1C, LOGL_ERROR, "Incorrect ipa header msg size %d %d\n",
		     ntohs(hh->len), msg->len - sizeof(struct ipaccess_head));
		return -1;
	}

	return 0;
}

int add_manufacturer_id_label(struct msgb *msg, int manuf_type_id)
{
	uint8_t *manuf;

	switch (manuf_type_id) {
	case IPACCESS_MANUF_ID:
		manuf = msgb_push(msg, 1 + sizeof(ipaccess_magic));
		manuf[0] = sizeof(ipaccess_magic);
		memcpy(manuf+1, ipaccess_magic, sizeof(ipaccess_magic));
		break;
	case OSMOCOM_MANUF_ID:
		manuf = msgb_push(msg, 1 + sizeof(osmocom_magic));
		manuf[0] = sizeof(osmocom_magic);
		memcpy(manuf+1, osmocom_magic, sizeof(osmocom_magic));
		break;
	default:
		return -1;
	}
	return 0;
}
