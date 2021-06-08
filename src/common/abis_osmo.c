/* OSMO extenion link associated to same line as oml_link: */

/* (C) 2021 by sysmocom - s.m.f.c. GmbH <info@sysmocom.de>
 * Author: Pau Espin Pedrol <pespin@sysmocom.de>
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

#include <errno.h>

#include <osmocom/core/msgb.h>
#include <osmocom/gsm/ipa.h>
#include <osmocom/gsm/protocol/ipaccess.h>

#include <osmo-bts/bts.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/pcu_if.h>
#include <osmo-bts/pcuif_proto.h>

extern struct gsm_network bts_gsmnet;

#define OM_HEADROOM_SIZE	128

////////////////////////////////////////
// OSMO ABIS extensions (PCU)
///////////////////////////////////////

static struct msgb *abis_osmo_pcu_msgb_alloc(uint8_t msg_type, uint8_t bts_nr, size_t extra_size)
{
	struct msgb *msg;
	struct gsm_pcu_if *pcu_prim;
	msg = msgb_alloc_headroom(OM_HEADROOM_SIZE + sizeof(struct gsm_pcu_if) + extra_size,
				  OM_HEADROOM_SIZE, "IPA/ABIS/OSMO");
	/* Only header is filled, caller is responible for reserving + filling
	 * message type specific contents: */
	msgb_put(msg, PCUIF_HDR_SIZE);
	pcu_prim = (struct gsm_pcu_if *) msgb_data(msg);
	pcu_prim->msg_type = msg_type;
	pcu_prim->bts_nr = bts_nr;
	return msg;
}

/* Send a OML NM Message from BSC to BTS */
int abis_osmo_sendmsg(struct gsm_bts *bts, struct msgb *msg)
{
	msg->dst = bts->osmo_link;
	msg->l2h = msg->data;
	return abis_sendmsg(msg);
}


/* Send IPA/OSMO/PCU extension Abis message from PCU to BSC */
static int abis_osmo_pcu_sendmsg(struct gsm_bts *bts, struct msgb *msg)
{
	ipa_prepend_header_ext(msg, IPAC_PROTO_EXT_PCU);
	return abis_osmo_sendmsg(bts, msg);
}

int abis_osmo_pcu_tx_container(struct gsm_bts *bts, const struct gsm_pcu_if_container *container)
{
	uint16_t data_length = osmo_load16be(&container->length);
	struct msgb *msg = abis_osmo_pcu_msgb_alloc(PCU_IF_MSG_CONTAINER, bts->nr, data_length);
	struct gsm_pcu_if *pcu_prim = (struct gsm_pcu_if *) msgb_data(msg);
	struct gsm_pcu_if_container *tx_cont = &pcu_prim->u.container;

	msgb_put(msg, sizeof(*tx_cont) + data_length);
	tx_cont->msg_type = container->msg_type;
	tx_cont->length = container->length;
	if (data_length)
		memcpy(tx_cont->data, container->data, data_length);

	return abis_osmo_pcu_sendmsg(bts, msg);
}


/* incoming IPA/OSMOEXT/PCU Abis message from BSC */
static int rx_down_osmo_pcu(struct gsm_bts *bts, struct msgb *msg)
{
	struct gsm_pcu_if *pcu_prim;
	if (msgb_l2len(msg) < PCUIF_HDR_SIZE) {
		LOGP(DPCU, LOGL_ERROR, "ABIS_OSMO_PCU message too short\n");
		oml_tx_failure_event_rep(&bts->mo, NM_SEVER_MAJOR, OSMO_EVT_MAJ_UKWN_MSG,
					 "ABIS_OSMO_PCU message too short\n");
		msgb_free(msg);
		return -EIO;
	}
	pcu_prim = msgb_l2(msg);
	LOGP(DPCU, LOGL_INFO, "Rx BSC->BTS%d ABIS_OSMO_PCU msg type %u\n",
	     pcu_prim->bts_nr, pcu_prim->msg_type);
	/* we patch the bts_nr received from BTS with the bts_nr we used to set up in the local PCU */
	pcu_prim->bts_nr = bts->nr;
	/* Trim Abis lower layers: */
	msgb_pull_to_l2(msg);
	/* we simply forward it to PCUIF: */
	return pcu_sock_send(&bts_gsmnet, msg);
}

/* incoming IPA/OSMO extension Abis message from BSC */
int down_osmo(struct gsm_bts *bts, struct msgb *msg)
{
	uint8_t *type;

	if (msgb_l2len(msg) < 1) {
		oml_tx_failure_event_rep(&bts->mo, NM_SEVER_MAJOR, OSMO_EVT_MAJ_UKWN_MSG,
					 "OSMO message too short\n");
		msgb_free(msg);
		return -EIO;
	}

	type = msgb_l2(msg);
	msg->l2h = type + 1;

	switch (*type) {
	case IPAC_PROTO_EXT_PCU:
		return rx_down_osmo_pcu(bts, msg);
	default:
		oml_tx_failure_event_rep(&bts->mo, NM_SEVER_MAJOR, OSMO_EVT_MAJ_UKWN_MSG,
					 "OSMO message unknown extension %u\n", *type);
		msgb_free(msg);
		return -EIO;
	}
}
