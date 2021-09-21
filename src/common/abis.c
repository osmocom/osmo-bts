/* Abis/IP interface routines utilizing libosmo-abis (Pablo) */

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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "btsconfig.h"

#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#include <osmocom/core/select.h>
#include <osmocom/core/timer.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/signal.h>
#include <osmocom/core/macaddr.h>
#include <osmocom/core/fsm.h>
#include <osmocom/abis/abis.h>
#include <osmocom/abis/e1_input.h>
#include <osmocom/abis/ipaccess.h>
#include <osmocom/gsm/ipa.h>

#include <osmo-bts/abis.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/gsm_data.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/rsl.h>
#include <osmo-bts/oml.h>
#include <osmo-bts/abis_osmo.h>
#include <osmo-bts/bts_model.h>
#include <osmo-bts/bts_trx.h>

static struct gsm_bts *g_bts;

static struct e1inp_line_ops line_ops;

static struct ipaccess_unit bts_dev_info;

#define S(x) (1 << (x))
#define	OML_RETRY_TIMER	5

enum abis_link_fsm_state {
	ABIS_LINK_ST_WAIT_RECONNECT, /* OML link has not yet been established */
	ABIS_LINK_ST_CONNECTING, /* OML link in process of been established */
	ABIS_LINK_ST_CONNECTED, /* OML link is established, RSL links may be established or not */
	ABIS_LINK_ST_FAILED, /* There used to be an active OML connection but it became broken */
};

static const struct value_string abis_link_fsm_event_names[] = {
	{ ABIS_LINK_EV_SIGN_LINK_OML_UP, "SIGN_LINK_OML_UP", },
	{ ABIS_LINK_EV_SIGN_LINK_DOWN,   "SIGN_LINK_DOWN" },
	{ ABIS_LINK_EV_VTY_RM_ADDR,      "VTY_RM_ADDR" },
	{}
};

struct abis_link_fsm_priv {
	struct bsc_oml_host *current_bsc;
	struct gsm_bts *bts;
	char *model_name;
	int line_ctr;
};

static void reset_oml_link(struct gsm_bts *bts)
{
	if (bts->oml_link) {
		struct timespec now;

		e1inp_sign_link_destroy(bts->oml_link);

		/* Log a special notice if the OML connection was dropped relatively quickly. */
		if (bts->oml_conn_established_timestamp.tv_sec != 0 && clock_gettime(CLOCK_MONOTONIC, &now) == 0 &&
		    bts->oml_conn_established_timestamp.tv_sec + OSMO_BTS_OML_CONN_EARLY_DISCONNECT >= now.tv_sec) {
			LOGP(DABIS, LOGL_FATAL, "OML link was closed early within %" PRIu64 " seconds. "
			     "If this situation persists, please check your BTS and BSC configuration files for errors. "
			     "A common error is a mismatch between unit_id configuration parameters of BTS and BSC.\n",
			     (uint64_t) (now.tv_sec - bts->oml_conn_established_timestamp.tv_sec));
		}
		bts->oml_link = NULL;
	}
	memset(&bts->oml_conn_established_timestamp, 0, sizeof(bts->oml_conn_established_timestamp));
}

static int pick_next_bsc(struct osmo_fsm_inst *fi)
{
	struct abis_link_fsm_priv *priv = fi->priv;
	struct gsm_bts *bts = priv->bts;
	struct bsc_oml_host *last;

	if (llist_empty(&bts->bsc_oml_hosts)) {
		LOGPFSML(fi, LOGL_ERROR, "List of BSCs to connect to is empty!\n");
		return -1;
	}

	last = (struct bsc_oml_host *)llist_last_entry(&bts->bsc_oml_hosts, struct bsc_oml_host, list);

	if (!priv->current_bsc || priv->current_bsc == last) /* Pick first one (wrap around): */
		priv->current_bsc = (struct bsc_oml_host *)llist_first_entry(&bts->bsc_oml_hosts, struct bsc_oml_host, list);
	else if (priv->current_bsc != last)
		priv->current_bsc = (struct bsc_oml_host *)llist_entry(priv->current_bsc->list.next, struct bsc_oml_host, list);

	return 0;
}

static void abis_link_connecting_onenter(struct osmo_fsm_inst *fi, uint32_t prev_state)
{
	struct e1inp_line *line;
	struct abis_link_fsm_priv *priv = fi->priv;
	struct gsm_bts *bts = priv->bts;

	if (pick_next_bsc(fi) < 0) {
		LOGPFSML(fi, LOGL_FATAL, "No BSC available, A-bis connection establishment failed\n");
		osmo_fsm_inst_state_chg(fi, ABIS_LINK_ST_FAILED, 0, 0);
		return;
	}

	LOGP(DABIS, LOGL_NOTICE, "A-bis connection establishment to BSC (%s) in progress...\n", priv->current_bsc->addr);

	/* patch in various data from VTY and other sources */
	line_ops.cfg.ipa.addr = priv->current_bsc->addr;
	osmo_get_macaddr(bts_dev_info.mac_addr, "eth0");
	bts_dev_info.site_id = bts->ip_access.site_id;
	bts_dev_info.bts_id = bts->ip_access.bts_id;
	bts_dev_info.unit_name = priv->model_name;
	if (bts->description)
		bts_dev_info.unit_name = bts->description;
	bts_dev_info.location2 = priv->model_name;

	line = e1inp_line_find(priv->line_ctr);
	if (line) {
		e1inp_line_get2(line, __FILE__);	/* We want a new reference for returned line */
	} else
		line = e1inp_line_create(priv->line_ctr, "ipa");	/* already comes with a reference */

	/* The abis connection may fail and we may have to try again with a different BSC (if configured). The next
	 * attempt must happen on a different line. */
	priv->line_ctr++;

	if (!line) {
		osmo_fsm_inst_state_chg(fi, ABIS_LINK_ST_FAILED, 0, 0);
		return;
	}
	e1inp_line_bind_ops(line, &line_ops);

	/* This will open the OML connection now */
	if (e1inp_line_update(line) < 0) {
		osmo_fsm_inst_state_chg(fi, ABIS_LINK_ST_FAILED, 0, 0);
		return;
	}

	/* The TCP connection to the BSC is now in progress.
	 * Wait for OML Link UP to transition to CONNECTED. */
}

static void abis_link_connecting(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct abis_link_fsm_priv *priv = fi->priv;
	struct gsm_bts *bts = priv->bts;

	switch (event) {
	case ABIS_LINK_EV_SIGN_LINK_OML_UP:
		osmo_fsm_inst_state_chg(fi, ABIS_LINK_ST_CONNECTED, 0, 0);
		break;
	case ABIS_LINK_EV_SIGN_LINK_DOWN:
		reset_oml_link(bts);
		osmo_fsm_inst_state_chg(fi, ABIS_LINK_ST_WAIT_RECONNECT, OML_RETRY_TIMER, 0);
		break;
	default:
		OSMO_ASSERT(0);
	}
}

static void abis_link_connected_onenter(struct osmo_fsm_inst *fi, uint32_t prev_state)
{
	bts_link_estab(g_bts);
}

static void abis_link_connected(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct abis_link_fsm_priv *priv = fi->priv;
	struct gsm_bts *bts = priv->bts;
	struct gsm_bts_trx *trx;
	OSMO_ASSERT(event == ABIS_LINK_EV_SIGN_LINK_DOWN);

	/* First remove the OML signalling link */
	reset_oml_link(bts);

	/* Then iterate over the RSL signalling links */
	llist_for_each_entry(trx, &bts->trx_list, list) {
		if (trx->rsl_link) {
			e1inp_sign_link_destroy(trx->rsl_link);
			trx->rsl_link = NULL;
		}
	}
	bts_model_abis_close(bts);
	osmo_fsm_inst_state_chg(fi, ABIS_LINK_ST_WAIT_RECONNECT, OML_RETRY_TIMER, 0);
}

static void abis_link_failed_onenter(struct osmo_fsm_inst *fi, uint32_t prev_state)
{
	struct abis_link_fsm_priv *priv = fi->priv;
	struct gsm_bts *bts = priv->bts;

	/* None of the configured BSCs was reachable or there was an existing
	 * OML/RSL connection that broke. Initiate BTS process shut down now. */
	bts_model_abis_close(bts);
}

static void abis_link_allstate(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct abis_link_fsm_priv *priv = fi->priv;
	struct gsm_bts *bts = priv->bts;

	OSMO_ASSERT(event == ABIS_LINK_EV_VTY_RM_ADDR);

	if (priv->current_bsc == data) {
		if (llist_count(&bts->bsc_oml_hosts) <= 1)
			priv->current_bsc = NULL;
		else
			pick_next_bsc(fi);
	}
}

int abis_link_fsm_timer_cb(struct osmo_fsm_inst *fi)
{
	switch (fi->state) {
	case ABIS_LINK_ST_WAIT_RECONNECT:
		osmo_fsm_inst_state_chg(fi, ABIS_LINK_ST_CONNECTING, 0, 0);
		break;
	default:
		OSMO_ASSERT(0);
	}
	return 0;
}


static struct osmo_fsm_state abis_link_fsm_states[] = {
	[ABIS_LINK_ST_WAIT_RECONNECT] = {
		.name = "WAIT_RECONNECT",
		.out_state_mask =
			S(ABIS_LINK_ST_CONNECTING),
	},
	[ABIS_LINK_ST_CONNECTING] = {
		.name = "CONNECTING",
		.in_event_mask =
			S(ABIS_LINK_EV_SIGN_LINK_OML_UP) |
			S(ABIS_LINK_EV_SIGN_LINK_DOWN),
		.out_state_mask =
			S(ABIS_LINK_ST_WAIT_RECONNECT) |
			S(ABIS_LINK_ST_CONNECTED) |
			S(ABIS_LINK_ST_FAILED),
		.onenter = abis_link_connecting_onenter,
		.action = abis_link_connecting,
	},
	[ABIS_LINK_ST_CONNECTED] = {
		.name = "CONNECTED",
		.in_event_mask =
			S(ABIS_LINK_EV_SIGN_LINK_DOWN),
		.out_state_mask =
			S(ABIS_LINK_ST_WAIT_RECONNECT),
		.onenter = abis_link_connected_onenter,
		.action = abis_link_connected,
	},
	[ABIS_LINK_ST_FAILED] = {
		.name = "FAILED",
		.onenter = abis_link_failed_onenter,
	},
};

static struct osmo_fsm abis_link_fsm = {
	.name = "abis_link",
	.states = abis_link_fsm_states,
	.num_states = ARRAY_SIZE(abis_link_fsm_states),
	.log_subsys = DABIS,
	.event_names = abis_link_fsm_event_names,
	.allstate_action = abis_link_allstate,
	.allstate_event_mask = S(ABIS_LINK_EV_VTY_RM_ADDR),
	.timer_cb = abis_link_fsm_timer_cb,
};

int abis_oml_sendmsg(struct msgb *msg)
{
	struct gsm_bts *bts = msg->trx->bts;

	if (!bts->oml_link) {
		LOGP(DABIS, LOGL_INFO, "Drop Tx OML msg, OML link is down\n");
		return 0;
	}

	/* osmo-bts uses msg->trx internally, but libosmo-abis uses
	 * the signalling link at msg->dst */
	msg->dst = bts->oml_link;
	return abis_sendmsg(msg);
}

int abis_bts_rsl_sendmsg(struct msgb *msg)
{
	OSMO_ASSERT(msg->trx);

	if (msg->trx->bts->variant == BTS_OSMO_OMLDUMMY) {
		msgb_free(msg);
		return 0;
	}

	/* osmo-bts uses msg->trx internally, but libosmo-abis uses
	 * the signalling link at msg->dst */
	msg->dst = msg->trx->rsl_link;
	return abis_sendmsg(msg);
}

static struct e1inp_sign_link *sign_link_up(void *unit, struct e1inp_line *line,
					    enum e1inp_sign_type type)
{
	struct e1inp_ts *sign_ts;
	struct gsm_bts_trx *trx;
	int trx_nr;

	switch (type) {
	case E1INP_SIGN_OML:
		sign_ts = e1inp_line_ipa_oml_ts(line);
		LOGP(DABIS, LOGL_INFO, "OML Signalling link up\n");
		e1inp_ts_config_sign(sign_ts, line);
		g_bts->oml_link = e1inp_sign_link_create(sign_ts, E1INP_SIGN_OML,
							g_bts->c0, IPAC_PROTO_OML, 0);
		if (clock_gettime(CLOCK_MONOTONIC, &g_bts->oml_conn_established_timestamp) != 0)
			memset(&g_bts->oml_conn_established_timestamp, 0,
			       sizeof(g_bts->oml_conn_established_timestamp));
		g_bts->osmo_link = e1inp_sign_link_create(sign_ts, E1INP_SIGN_OSMO,
							  g_bts->c0, IPAC_PROTO_OSMO, 0);
		osmo_fsm_inst_dispatch(g_bts->abis_link_fi, ABIS_LINK_EV_SIGN_LINK_OML_UP, NULL);
		return g_bts->oml_link;

	case E1INP_SIGN_RSL:
		/* fall through to default to catch TRXn having type = E1INP_SIGN_RSL + n  */
	default:
		trx_nr = type - E1INP_SIGN_RSL;
		sign_ts = e1inp_line_ipa_rsl_ts(line, trx_nr);
		LOGP(DABIS, LOGL_INFO, "RSL Signalling link for TRX%d up\n",
			trx_nr);
		trx = gsm_bts_trx_num(g_bts, trx_nr);
		if (!trx) {
			LOGP(DABIS, LOGL_ERROR, "TRX%d does not exist!\n",
				trx_nr);
			break;
		}
		e1inp_ts_config_sign(sign_ts, line);
		trx->rsl_link = e1inp_sign_link_create(sign_ts, E1INP_SIGN_RSL,
						       trx, trx->rsl_tei, 0);
		trx_link_estab(trx);
		return trx->rsl_link;
	}
	return NULL;
}

static void sign_link_down(struct e1inp_line *line)
{
	LOGPIL(line, DABIS, LOGL_ERROR, "Signalling link down\n");
	osmo_fsm_inst_dispatch(g_bts->abis_link_fi, ABIS_LINK_EV_SIGN_LINK_DOWN, NULL);
}


/* callback for incoming messages from A-bis/IP */
static int sign_link_cb(struct msgb *msg)
{
	struct e1inp_sign_link *link = msg->dst;

	/* osmo-bts code assumes msg->trx is set, but libosmo-abis works
	 * with the sign_link stored in msg->dst, so we have to convert
	 * here */
	msg->trx = link->trx;

	switch (link->type) {
	case E1INP_SIGN_OML:
		down_oml(link->trx->bts, msg);
		break;
	case E1INP_SIGN_RSL:
		down_rsl(link->trx, msg);
		break;
	case E1INP_SIGN_OSMO:
		down_osmo(link->trx->bts, msg);
		break;
	default:
		msgb_free(msg);
		break;
	}

	return 0;
}

uint32_t get_signlink_remote_ip(struct e1inp_sign_link *link)
{
	int fd = link->ts->driver.ipaccess.fd.fd;
	struct sockaddr_in sin;
	socklen_t slen = sizeof(sin);
	int rc;

	rc = getpeername(fd, (struct sockaddr *)&sin, &slen);
	if (rc < 0) {
		LOGP(DOML, LOGL_ERROR, "Cannot determine remote IP Addr: %s\n",
			strerror(errno));
		return 0;
	}

	/* we assume that the socket is AF_INET.  As Abis/IP contains
	 * lots of hard-coded IPv4 addresses, this safe */
	OSMO_ASSERT(sin.sin_family == AF_INET);

	return ntohl(sin.sin_addr.s_addr);
}


static int inp_s_cbfn(unsigned int subsys, unsigned int signal,
		      void *hdlr_data, void *signal_data)
{
	if (subsys != SS_L_INPUT)
		return 0;

	struct input_signal_data *isd = signal_data;
	DEBUGP(DABIS, "Input Signal %s received for link_type=%s\n",
	       get_value_string(e1inp_signal_names, signal), e1inp_signtype_name(isd->link_type));

	return 0;
}


static struct ipaccess_unit bts_dev_info = {
	.unit_name	= "osmo-bts",
	.equipvers	= "",	/* FIXME: read this from hw */
	.swversion	= PACKAGE_VERSION,
	.location1	= "",
	.location2	= "",
	.serno		= "",
};

static struct e1inp_line_ops line_ops = {
	.cfg = {
		.ipa = {
			.role	= E1INP_LINE_R_BTS,
			.dev	= &bts_dev_info,
		},
	},
	.sign_link_up	= sign_link_up,
	.sign_link_down	= sign_link_down,
	.sign_link	= sign_link_cb,
};

void abis_init(struct gsm_bts *bts)
{
	g_bts = bts;

	oml_init();
	libosmo_abis_init(tall_bts_ctx);

	osmo_signal_register_handler(SS_L_INPUT, &inp_s_cbfn, bts);
}

int abis_open(struct gsm_bts *bts, char *model_name)
{
	struct abis_link_fsm_priv *abis_link_fsm_priv;

	if (llist_empty(&bts->bsc_oml_hosts)) {
		LOGP(DABIS, LOGL_FATAL, "No BSC configured, cannot start BTS without knowing BSC OML IP\n");
		return -EINVAL;
	}

	bts->abis_link_fi = osmo_fsm_inst_alloc(&abis_link_fsm, bts, NULL, LOGL_DEBUG, "abis_link");
	OSMO_ASSERT(bts->abis_link_fi);

	abis_link_fsm_priv = talloc_zero(bts->abis_link_fi, struct abis_link_fsm_priv);
	OSMO_ASSERT(abis_link_fsm_priv);
	abis_link_fsm_priv->bts = bts;
	abis_link_fsm_priv->model_name = model_name;
	bts->abis_link_fi->priv = abis_link_fsm_priv;

	osmo_fsm_inst_state_chg(bts->abis_link_fi, ABIS_LINK_ST_CONNECTING, 0, 0);

	return 0;
}

static __attribute__((constructor)) void abis_link_fsm_init(void)
{
	OSMO_ASSERT(osmo_fsm_register(&abis_link_fsm) == 0);
}
