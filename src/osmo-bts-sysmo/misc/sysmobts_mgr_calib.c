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

#include <osmocom/ctrl/control_cmd.h>

#include <osmocom/gsm/ipa.h>
#include <osmocom/gsm/protocol/ipaccess.h>

#include <osmocom/abis/abis.h>
#include <osmocom/abis/e1_input.h>
#include <osmocom/abis/ipa.h>

static void bts_updown_cb(struct ipa_client_conn *link, int up);

enum calib_state {
	CALIB_INITIAL,
	CALIB_CTR_RESET,
	CALIB_CTR_WAIT,
	CALIB_COR_SET,
};

static void send_ctrl_cmd(struct sysmobts_mgr_instance *mgr,
			struct msgb *msg)
{
	ipa_prepend_header_ext(msg, IPAC_PROTO_EXT_CTRL);
	ipa_prepend_header(msg, IPAC_PROTO_OSMO);
	ipa_client_conn_send(mgr->calib.bts_conn, msg);
}

static void send_set_ctrl_cmd(struct sysmobts_mgr_instance *mgr,
				const char *key, const char *val)
{
	struct msgb *msg;
	int ret;

	msg = msgb_alloc_headroom(1024, 128, "CTRL SET");
	ret = snprintf((char *) msg->data, 4096, "SET %u %s %s",
			mgr->calib.last_seqno++, key, val);
	msg->l2h = msgb_put(msg, ret);
	return send_ctrl_cmd(mgr, msg);
}

static void send_get_ctrl_cmd(struct sysmobts_mgr_instance *mgr,
				const char *key)
{
	struct msgb *msg;
	int ret;

	msg = msgb_alloc_headroom(1024, 128, "CTRL GET");
	ret = snprintf((char *) msg->data, 4096, "GET %u %s",
			mgr->calib.last_seqno++, key);
	msg->l2h = msgb_put(msg, ret);
	return send_ctrl_cmd(mgr, msg);
}

int sysmobts_mgr_calib_run(struct sysmobts_mgr_instance *mgr)
{
	if (!mgr->calib.is_up) {
		LOGP(DCALIB, LOGL_ERROR, "Control interface not connected.\n");
		return -1;
	}

	if (mgr->calib.state != CALIB_INITIAL) {
		LOGP(DCALIB, LOGL_ERROR, "Calib is already in progress.\n");
		return -2;
	}

	send_set_ctrl_cmd(mgr, "trx.0.clock-info", "1");
	mgr->calib.state = CALIB_CTR_RESET;
	return 0;
}

static void calib_state_reset(struct sysmobts_mgr_instance *mgr)
{
	mgr->calib.state = CALIB_INITIAL;
	osmo_timer_del(&mgr->calib.timer);
}

static void calib_get_clock_err_cb(void *_data)
{
	struct sysmobts_mgr_instance *mgr = _data;

	send_get_ctrl_cmd(mgr, "trx.0.clock-info");
}

static void handle_ctrl_reset_resp(
				struct sysmobts_mgr_instance *mgr,
				struct ctrl_cmd *cmd)
{
	if (strcmp(cmd->variable, "trx.0.clock-info") != 0) {
		LOGP(DCALIB, LOGL_ERROR,
			"Unexpected variable: %s\n", cmd->variable);
		calib_state_reset(mgr);
		return;
	}

	if (strcmp(cmd->reply, "success") != 0) {
		LOGP(DCALIB, LOGL_ERROR,
			"Unexpected reply: %s\n", cmd->variable);
		calib_state_reset(mgr);
		return;
	}

	mgr->calib.state = CALIB_CTR_WAIT;
	mgr->calib.timer.cb = calib_get_clock_err_cb;
	mgr->calib.timer.data = mgr;
	osmo_timer_schedule(&mgr->calib.timer, 60, 0);
}

static void handle_ctrl_get_resp(
				struct sysmobts_mgr_instance *mgr,
				struct ctrl_cmd *cmd)
{
	char *saveptr = NULL;
	char *clk_cur;
	char *clk_src;
	char *cal_err;
	char *cal_res;
	char *cal_src;

	if (strcmp(cmd->variable, "trx.0.clock-info") != 0) {
		LOGP(DCALIB, LOGL_ERROR,
			"Unexpected variable: %s\n", cmd->variable);
		calib_state_reset(mgr);
		return;
	}

	clk_cur = strtok_r(cmd->reply, ",", &saveptr);
	clk_src = strtok_r(NULL, ",", &saveptr);
	cal_err = strtok_r(NULL, ",", &saveptr);
	cal_res = strtok_r(NULL, ",", &saveptr);
	cal_src = strtok_r(NULL, ",", &saveptr);

	if (!clk_cur || !clk_src || !cal_err || !cal_res || !cal_src) {
		LOGP(DCALIB, LOGL_ERROR, "Parse error on clock-info reply\n");
		calib_state_reset(mgr);
		return;

	}
	LOGP(DCALIB, LOGL_NOTICE,
		"Calibration CUR(%s) SRC(%s) ERR(%s) RES(%s) SRC(%s)\n",
		clk_cur, clk_src, cal_err, cal_res, cal_src);

	if (strcmp(cal_res, "0") == 0) {
		LOGP(DCALIB, LOGL_ERROR, "Invalid clock resolution. Giving up\n");
		calib_state_reset(mgr);
		return;
	}

	/* Now we can finally set the new value */
	send_set_ctrl_cmd(mgr, "trx.0.clock-correction", cal_err);
	mgr->calib.state = CALIB_COR_SET;
}

static void handle_ctrl_set_cor(
			struct sysmobts_mgr_instance *mgr,
			struct ctrl_cmd *cmd)
{
	if (strcmp(cmd->variable, "trx.0.clock-correction") != 0) {
		LOGP(DCALIB, LOGL_ERROR,
			"Unexpected variable: %s\n", cmd->variable);
		calib_state_reset(mgr);
		return;
	}

	if (strcmp(cmd->reply, "success") != 0) {
		LOGP(DCALIB, LOGL_ERROR,
			"Unexpected reply: %s\n", cmd->variable);
		calib_state_reset(mgr);
		return;
	}

	LOGP(DCALIB, LOGL_NOTICE,
		"Calibration process completed\n");
	mgr->calib.state = CALIB_INITIAL;
}

static void handle_ctrl(struct sysmobts_mgr_instance *mgr, struct msgb *msg)
{
	struct ctrl_cmd *cmd = ctrl_cmd_parse(tall_mgr_ctx, msg);
	if (!cmd) {
		LOGP(DCALIB, LOGL_ERROR, "Failed to parse command/response\n");
		return;
	}

	switch (cmd->type) {
	case CTRL_TYPE_GET_REPLY:
		switch (mgr->calib.state) {
		case CALIB_CTR_WAIT:
			handle_ctrl_get_resp(mgr, cmd);
			break;
		default:
			LOGP(DCALIB, LOGL_ERROR,
				"Unhandled response in state: %d %s/%s\n",
				mgr->calib.state, cmd->variable, cmd->reply);
			calib_state_reset(mgr);
			break;
		};
		break;
	case CTRL_TYPE_SET_REPLY:
		switch (mgr->calib.state) {
		case CALIB_CTR_RESET:
			handle_ctrl_reset_resp(mgr, cmd);
			break;
		case CALIB_COR_SET:
			handle_ctrl_set_cor(mgr, cmd);
			break;
		default:
			LOGP(DCALIB, LOGL_ERROR,
				"Unhandled response in state: %d %s/%s\n",
				mgr->calib.state, cmd->variable, cmd->reply);
			calib_state_reset(mgr);
			break;
		};
		break;
	case CTRL_TYPE_TRAP:
		/* ignore any form of trap */
		break;
	default:
		LOGP(DCALIB, LOGL_ERROR,
			"Unhandled CTRL response: %d. Resetting state\n",
			cmd->type);
		calib_state_reset(mgr);
		break;
	}

	talloc_free(cmd);
}

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
	struct ipaccess_head_ext *hh_ext;

	DEBUGP(DCALIB, "Received data from BTS: %s\n",
		osmo_hexdump(msgb_data(msg), msgb_length(msg)));

	/* regular message handling */
	rc = msg_verify_ipa_structure(msg);
	if (rc < 0) {
		LOGP(DCALIB, LOGL_ERROR,
			"Invalid IPA message from BTS (rc=%d)\n", rc);
		goto err;
	}

	switch (hh->proto) {
	case IPAC_PROTO_IPACCESS:
		/* handle the core IPA CCM messages in libosmoabis */
		ipa_ccm_rcvmsg_bts_base(msg, link->ofd);
		msgb_free(msg);
		break;
	case IPAC_PROTO_OSMO:
		hh_ext = (struct ipaccess_head_ext *) hh->data;
		switch (hh_ext->proto) {
		case IPAC_PROTO_EXT_CTRL:
			handle_ctrl(link->data, msg);
			break;
		default:
			LOGP(DCALIB, LOGL_NOTICE,
				"Unhandled osmo ID %u from BTS\n", hh_ext->proto);
		};
		msgb_free(msg);
		break;
	default:
		LOGP(DCALIB, LOGL_NOTICE,
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
		mgr->calib.last_seqno = 0;
		calib_state_reset(mgr);
	} else {
		mgr->calib.is_up = 0;
		schedule_bts_connect(mgr);
		calib_state_reset(mgr);
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
