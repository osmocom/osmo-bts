/* OCXO/TCXO calibration control for SysmoBTS management daemon */

/*
 * (C) 2014 by Holger Hans Peter Freyther
 * (C) 2014 by Harald Welte for the IPA code from the oml router
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

#include "misc/sysmobts_mgr.h"
#include "osmo-bts/msg_utils.h"

#include <osmocom/core/logging.h>
#include <osmocom/core/select.h>

#include <osmocom/gsm/ipa.h>
#include <osmocom/gsm/protocol/ipaccess.h>

#include <osmocom/abis/abis.h>
#include <osmocom/abis/e1_input.h>
#include <osmocom/abis/ipa.h>

static void bts_updown_cb(struct ipa_client_conn *link, int up);

/* Schedule a connect towards the BTS */
static void schedule_bts_connect(struct sysmobts_mgr_instance *mgr)
{
	DEBUGP(DLCTRL, "Scheduling BTS connect\n");
	osmo_timer_schedule(&mgr->calib.recon_timer, 1, 0);
}

/* BTS re-connect timer call-back */
static void bts_recon_timer_cb(void *data)
{
	int rc;
	struct sysmobts_mgr_instance *mgr = data;

	/* The connection failures are to be expected during boot */
	mgr->calib.bts_conn->ofd->when |= BSC_FD_WRITE;
	rc = ipa_client_conn_open(mgr->calib.bts_conn);
	if (rc < 0) {
		LOGP(DLCTRL, LOGL_NOTICE, "Failed to connect to BTS.\n");
		schedule_bts_connect(mgr);
	}
}

static int bts_read_cb(struct ipa_client_conn *link, struct msgb *msg)
{
	int rc;
	struct ipaccess_head *hh = (struct ipaccess_head *) msgb_l1(msg);

	DEBUGP(DLCTRL, "Received data from BTS: %s\n",
		osmo_hexdump(msgb_data(msg), msgb_length(msg)));

	/* regular message handling */
	rc = msg_verify_ipa_structure(msg);
	if (rc < 0) {
		LOGP(DLCTRL, LOGL_ERROR,
			"Invalid IPA message from BTS (rc=%d)\n", rc);
		goto err;
	}

	switch (hh->proto) {
	case IPAC_PROTO_IPACCESS:
		/* handle the core IPA CCM messages in libosmoabis */
		ipa_ccm_rcvmsg_bts_base(msg, link->ofd);
		msgb_free(msg);
		break;
	default:
		LOGP(DLCTRL, LOGL_NOTICE,
		     "Unhandled stream ID %u from BTS\n", hh->proto);
		msgb_free(msg);
		break;
	}

	return 0;
err:
	msgb_free(msg);
	return -1;
}

/* link to BSC has gone up or down */
static void bts_updown_cb(struct ipa_client_conn *link, int up)
{
	struct sysmobts_mgr_instance *mgr = link->data;

	LOGP(DLCTRL, LOGL_INFO, "BTS connection %s\n", up ? "up" : "down");

	if (up) {
		mgr->calib.is_up = 1;
	} else {
		mgr->calib.is_up = 0;
		schedule_bts_connect(mgr);
	}
}

int sysmobts_mgr_calib_init(struct sysmobts_mgr_instance *mgr)
{
	mgr->calib.bts_conn = ipa_client_conn_create(tall_mgr_ctx, NULL, 0,
					"localhost", 4238,
					bts_updown_cb, bts_read_cb,
					NULL, mgr);
	if (!mgr) {
		LOGP(DCALIB, LOGL_ERROR,
			"Failed to create IPA connection\n");
		return -1;
	}

	mgr->calib.recon_timer.cb = bts_recon_timer_cb;
	mgr->calib.recon_timer.data = mgr;
	schedule_bts_connect(mgr);

	return 0;
}
