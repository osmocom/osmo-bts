/* OML Message Router (server side) */

/* (C) 2014 by Harald Welte <laforge@gnumonks.org>
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

#include <sys/socket.h>
#include <sys/signal.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <osmocom/core/talloc.h>
#include <osmocom/core/logging.h>
#include <osmocom/core/application.h>
#include <osmocom/core/select.h>
#include <osmocom/core/macaddr.h>
#include <osmocom/gsm/tlv.h>
#include <osmocom/gsm/protocol/gsm_12_21.h>

#include <osmocom/core/msgb.h>
#include <osmocom/abis/abis.h>
#include <osmocom/abis/e1_input.h>
#include <osmocom/abis/ipa.h>
#include <osmocom/gsm/protocol/ipaccess.h>
#include <osmocom/gsm/ipa.h>

#include <osmocom/vty/vty.h>
#include <osmocom/vty/command.h>
#include <osmocom/vty/telnet_interface.h>

#include <osmo-bts/logging.h>
#include <osmo-bts/msg_utils.h>
#include <osmo-bts/oml_routing.h>
#include <osmo-bts/oml_router_ctrl.h>
#include <osmo-bts/vty.h>

#include "../../btsconfig.h"

static const char *config_file = "osmobts-omlrouter.cfg";
static int daemonize = 0;

enum bsc_link_state {
	BSC_LS_WAIT_RECONNECT,
	BSC_LS_CONNECTING,
	BSC_LS_CONNECTED,
};

static const struct value_string bsc_ls_names[] = {
	{ BSC_LS_WAIT_RECONNECT, "waiting for reconnect" },
	{ BSC_LS_CONNECTING, "attempting to connect" },
	{ BSC_LS_CONNECTED, "connected" },
	{ 0, NULL }
};

struct oml_router {
	/* config */
	char *bsc_oml_host;
	uint16_t bsc_oml_port;
	char *listen_host;
	uint16_t listen_port;
	unsigned int bsc_reconnect_secs;

	/* state */

	struct oml_routing_inst *routing;
	/* state of the BSC connection */
	enum bsc_link_state bsc_link_state;
	/* BSC connection */
	struct ipa_client_conn *bsc_conn;
	/* BSC re-connection timer */
	struct osmo_timer_list bsc_recon_timer;
	/* server listening for clients */
	struct ipa_server_link *server_link;
	/* list of clients connected to us */
	struct llist_head clients;
};

struct oml_client {
	struct llist_head list;
	struct ipa_server_conn *conn;
	struct oml_router *router;
	struct ipaccess_unit *unit_data;
	char *name;
};

char *oml_route_client_name(struct oml_client *cl)
{
	struct ipaccess_unit *ud = cl->unit_data;
	static char outbuf[256];

	snprintf(outbuf, sizeof(outbuf)-1, "%d/%d/%d(%s)",
		 ud->site_id, ud->bts_id, ud->trx_id,
		 cl->name ? cl->name : "NULL");
	outbuf[sizeof(outbuf)-1] = '\0';

	return outbuf;
}

char *oml_route_client_addr(struct oml_client *cl)
{
	static char outbuf[256];
	struct sockaddr_in sin;
	socklen_t slen = sizeof(sin);

	getpeername(cl->conn->ofd.fd, (struct sockaddr *) &sin, &slen);
	snprintf(outbuf, sizeof(outbuf)-1, "%s:%u",
		 inet_ntoa(sin.sin_addr), ntohs(sin.sin_port));
	outbuf[sizeof(outbuf)-1] = '\0';

	return outbuf;
}

static struct ipaccess_unit bts_dev_info = {
	.unit_name	= "sysmoBTS",
	.equipvers	= "",	/* FIXME: read this from hw */
	.swversion	= PACKAGE_VERSION,
	.location1	= "",
	.location2	= "",
	.serno		= "",	/* FIXME: read this from hw */
};

/* We shouldn't need this, but there's no 'private data' inside an e1inp_line */
static struct oml_router *g_inst;

#define ORC_HEADROOM 20

static struct msgb *orc_msgb_alloc(void)
{
	unsigned int headroom;

	headroom = ORC_HEADROOM + sizeof(struct ipaccess_head) +
		   sizeof(struct ipaccess_head_ext);

	return msgb_alloc_headroom(1200 + headroom, headroom, "OML Router Ctrl");
}

/* push ipa_ext and ipa header */
static void orc_push_header(struct msgb *msg)
{
	struct ipaccess_head_ext *he;

	he = (struct ipaccess_head_ext *) msgb_push(msg, sizeof(*he));
	he->proto = IPAC_PROTO_EXT_ORC;

	ipa_msg_push_header(msg, IPAC_PROTO_OSMO);
}

/* push ipa_ext and ipa header + send to client */
static void orc_client_send(struct oml_client *client, struct msgb *msg)
{
	orc_push_header(msg);
	ipa_server_conn_send(client->conn, msg);
}

/* Send ORC ACK to client */
static void client_send_ack(struct oml_client *client,
			    enum osmo_omlrctrl_msgtype type)
{
	struct msgb *msg = orc_msgb_alloc();
	struct osmo_omlrctrl_hdr *oh;

	oh = (struct osmo_omlrctrl_hdr *) msgb_put(msg, sizeof(*oh));
	oh->version = 0;
	oh->msg_type = type;
	oh->data_len = 0;

	orc_client_send(client, msg);
}

/* Send ORC NACK to client */
static void client_send_nack(struct oml_client *client,
			     enum osmo_omlrctrl_msgtype type,
			     uint8_t cause, char *diag)
{
	struct msgb *msg = orc_msgb_alloc();
	struct osmo_omlrctrl_hdr *oh;
	unsigned int d_len = 0, payload_len = 1;

	/* optional diagnostics LV at end of message */
	if (diag) {
		d_len = strlen(diag);
		payload_len += d_len + 2; /* NULL + Length */
	}

	oh = (struct osmo_omlrctrl_hdr *)
			msgb_put(msg, sizeof(*oh)+payload_len);
	oh->version = 0;
	oh->msg_type = type;
	oh->data_len = payload_len;
	/* first byte is cause */
	oh->data[0] = cause;
	/* followed by optional diagnostics string as LV */
	if (d_len) {
		oh->data[1] = d_len;
		memcpy(oh->data+2, diag, d_len);
	}

	orc_client_send(client, msg);
}

/* process incoming ORC REGISTER recquest from client */
static int client_rx_orc_register(struct oml_client *client,
				  struct msgb *msg)
{
	struct osmo_omlrctrl_register_req *rr =
		(struct osmo_omlrctrl_register_req *) msgb_l3(msg);

	if (msgb_l3len(msg) < sizeof(*rr)) {
		client_send_nack(client, OSMO_ORC_MSGT_REGISTER_NACK,
				 0xff, "Short header");
		return -1;
	}

	if (rr->name_len && rr->name[rr->name_len-1] != '\0') {
		client_send_nack(client, OSMO_ORC_MSGT_REGISTER_NACK,
				 0xff, "Name without NULL termination");
		return -1;
	}

	/* store client identity parameters */
	if (rr->name_len) {
		talloc_free(client->name);
		client->name = talloc_strndup(client, rr->name, rr->name_len);
	}

	LOGP(DOML, LOGL_INFO, "Client %s registered as %s\n",
	     oml_route_client_addr(client),
	     oml_route_client_name(client));

	client_send_ack(client, OSMO_ORC_MSGT_REGISTER_ACK);

	return 0;
}

/* process incoming ORC message from client */
static int ipa_omlrouter_rcvmsg(struct oml_client *client, struct msgb *msg)
{
	struct osmo_omlrctrl_hdr *oh =
		(struct osmo_omlrctrl_hdr *) msgb_l2(msg);
	struct oml_route *rt;
	int rc;

	/* FIXME: size / consistency checks */
	msg->l3h = oh->data;

	switch (oh->msg_type) {
	case OSMO_ORC_MSGT_REGISTER_REQ:
		rc = client_rx_orc_register(client, msg);
		break;
	case OSMO_ORC_MSGT_ROUTE_ADD_REQ:
		DEBUGP(DOML, "route add request\n");
		if (oh->data_len < sizeof(*rt)) {
			LOGP(DOML, LOGL_ERROR, "route too small (%u < %lu)\n",
				oh->data_len, sizeof(*rt));
			client_send_nack(client, OSMO_ORC_MSGT_ROUTE_ADD_NACK,
					 0, "Route size mismatch");
			goto err;
		}
		rt = (struct oml_route *) &oh->data;
		rc = oml_route_add(client->router->routing, rt, client);
		if (rc < 0)
			client_send_nack(client, OSMO_ORC_MSGT_ROUTE_ADD_NACK,
					 -rc, "Could not add route");
		else
			client_send_ack(client, OSMO_ORC_MSGT_ROUTE_ADD_ACK);
		break;
	case OSMO_ORC_MSGT_ROUTE_DEL_REQ:
		if (oh->data_len <= sizeof(*rt))
			goto err;
		rt = (struct oml_route *) &oh->data;
		rc = oml_route_del(client->router->routing, rt);
		if (rc < 0)
			client_send_nack(client, OSMO_ORC_MSGT_ROUTE_DEL_NACK,
					 -rc, "Could not delete route");
		else
			client_send_ack(client, OSMO_ORC_MSGT_ROUTE_DEL_ACK);
		break;
	}

err:
	msgb_free(msg);
	return rc;
}

static int client_rx_ipa_ccm(struct oml_client *client, struct msgb *msg)
{
	struct tlv_parsed tp;
	uint8_t msg_type = *(msg->l2h);
	int rc;

	/* deal with PING/PONG/ACK */
	rc = ipa_ccm_rcvmsg_base(msg, &client->conn->ofd);
	if (rc == 1) {
		/* message was handled by rcvmsg_base */
		return 0;
	}

	/* messages that we have to handle locally */
	switch (msg_type) {
	case IPAC_MSGT_ID_RESP:
		rc = ipa_ccm_idtag_parse(&tp, msgb_l2(msg)+2,
					  msgb_l2len(msg)-2);
		if (rc < 0)
			return rc;

		/* store result in client structure */
		rc = ipa_ccm_tlv_to_unitdata(client->unit_data, &tp);
		if (rc < 0)
			return rc;

		if (client->unit_data->site_id != bts_dev_info.site_id) {
			LOGP(DOML, LOGL_ERROR, "Client %s Wrong Site ID "
			     "(client %u != our %u); Closing\n",
			     oml_route_client_addr(client),
			     client->unit_data->site_id, bts_dev_info.site_id);
			ipa_server_conn_destroy(client->conn);
			return -1;
		}
		break;
	default:
		LOGP(DOML, LOGL_NOTICE, "Not handled IPA CCM from client\n");
		return -1;
	}

	return rc;
}

/* one of the clients has sent data to us */
static int client_data_cb(struct ipa_server_conn *conn, struct msgb *msg)
{
	struct oml_client *client = conn->data;
	struct ipaccess_head *hh = (struct ipaccess_head *) msgb_l1(msg);
	struct ipaccess_head_ext *hh_ext;
	int rc;

	DEBUGP(DOML, "Received data from client: %s\n",
		osmo_hexdump(msgb_data(msg), msgb_length(msg)));

	/* regular message handling */
	rc = msg_verify_ipa_structure(msg);
	if (rc < 0) {
		LOGP(DOML, LOGL_ERROR,
			"Invalid IPA message from client (rc=%d)\n", rc);
		goto err;
	}

	switch (hh->proto) {
	case IPAC_PROTO_IPACCESS:
		rc = client_rx_ipa_ccm(client, msg);
		msgb_free(msg);
		break;
	case IPAC_PROTO_OSMO:
		hh_ext = (struct ipaccess_head_ext *) hh->data;
		switch (hh_ext->proto) {
		case IPAC_PROTO_EXT_ORC:
			rc = ipa_omlrouter_rcvmsg(client, msg);
			break;
		default:
			LOGP(DOML, LOGL_NOTICE,
			     "Unknown IPA-EXT protocol %u\n", hh_ext->proto);
			msgb_free(msg);
			break;
		}
		break;
	case IPAC_PROTO_OML:
		rc = msg_verify_oml_structure(msg);
		if (rc < 0) {
			LOGP(DOML, LOGL_ERROR,
				"Invalid OML message from Client (rc=%d)\n", rc);
			goto err;
		}
		/* forward message to BSC */
		ipa_client_conn_send(client->router->bsc_conn, msg);
		break;
	default:
		LOGP(DOML, LOGL_NOTICE, "Unhandled stream ID %u from client\n", hh->proto);
		msgb_free(msg);
		break;
	}

	return 0;
err:
	msgb_free(msg);
	return -1;
}

/* a client connection was lost */
static int client_closed_cb(struct ipa_server_conn *conn)
{
	struct oml_client *client = conn->data;

	oml_route_del_client(client->router->routing, client);

	llist_del(&client->list);
	talloc_free(client);

	return 0;
}

/* a new client has connected to our server */
static int server_accept_cb(struct ipa_server_link *link, int fd)
{
	struct oml_router *inst = link->data;
	struct oml_client *client = talloc_zero(link->data, struct oml_client);

	client->unit_data = talloc_zero(client, struct ipaccess_unit);
	client->router = inst;

	client->conn = ipa_server_conn_create(link, link, fd, client_data_cb,
					      client_closed_cb, client);
	if (!client->conn) {
		talloc_free(client);
		return -1;
	}

	llist_add_tail(&client->list, &inst->clients);

	/* send IPA ID REQ */
	ipa_ccm_send_id_req(fd);

	return 0;
}

/* schedule a re-connect towards the BSC */
static void reschedule_bsc_connect(struct oml_router *inst)
{
	DEBUGP(DOML, "Re-scheduling BSC connect\n");
	osmo_timer_schedule(&inst->bsc_recon_timer, inst->bsc_reconnect_secs, 0);
	inst->bsc_link_state = BSC_LS_CONNECTING;
}

/* BSC re-connect timer call-back */
static void bsc_recon_timer_cb(void *data)
{
	struct oml_router *inst = data;

	if (ipa_client_conn_open(inst->bsc_conn) < 0) {
		LOGP(DOML, LOGL_NOTICE, "failed to connect to BSC\n");
		reschedule_bsc_connect(inst);
	}
}


/* link to BSC has gone up or down */
static void bsc_updown_cb(struct ipa_client_conn *link, int up)
{
	struct oml_router *inst = link->data;

	LOGP(DOML, LOGL_INFO, "BSC connection %s\n", up ? "up" : "down");

	if (up) {
		inst->bsc_link_state = BSC_LS_CONNECTED;
		/* FIXME */
	} else {
		reschedule_bsc_connect(link->data);
		inst->bsc_link_state = BSC_LS_WAIT_RECONNECT;
	}
}

/* derive routing key from OML message */
static void key_from_omlmsg(struct oml_routing_key *key, const struct msgb *msg)
{
	struct abis_om_hdr *oh = (struct abis_om_hdr *) msgb_l2(msg);
	struct abis_om_fom_hdr *foh;

	memset(key, 0, sizeof(*key));

	key->mdisc = oh->mdisc;
	switch (oh->mdisc) {
	case ABIS_OM_MDISC_FOM:
		foh = (struct abis_om_fom_hdr *) oh->data;
		key->obj_class = foh->obj_class;
		memcpy(&key->obj_inst, &foh->obj_inst, sizeof(key->obj_inst));
		break;
	case ABIS_OM_MDISC_MANUF:
		break;
	}
}

/* incoming data from the BSC */
static int bsc_read_cb(struct ipa_client_conn *link, struct msgb *msg)
{
	int rc;
	struct ipaccess_head *hh = (struct ipaccess_head *) msgb_l1(msg);
	struct oml_router *inst = link->data;
	struct oml_client *client;
	struct oml_routing_key key;

	DEBUGP(DOML, "Received data from BSC: %s\n",
		osmo_hexdump(msgb_data(msg), msgb_length(msg)));

	/* regular message handling */
	rc = msg_verify_ipa_structure(msg);
	if (rc < 0) {
		LOGP(DOML, LOGL_ERROR,
			"Invalid IPA message from BSC (rc=%d)\n", rc);
		goto err;
	}

	switch (hh->proto) {
	case IPAC_PROTO_IPACCESS:
		/* handle the core IPA CCM messages in libosmoabis */
		rc = ipaccess_bts_handle_ccm(link, &bts_dev_info, msg);
		if (rc == 0)
			LOGP(DOML, LOGL_NOTICE, "Not handled IPA CCM from BSC\n");
		else if (rc < 0)
			LOGP(DOML, LOGL_ERROR, "Error handling IPA CCM from BSC\n");
		msgb_free(msg);
		break;
	case IPAC_PROTO_OML:
		rc = msg_verify_oml_structure(msg);
		if (rc < 0) {
			LOGP(DOML, LOGL_ERROR,
				"Invalid OML message from BSC (rc=%d)\n", rc);
			goto err;
		}

		/* find route */
		key_from_omlmsg(&key, msg);
		client = oml_route(inst->routing, &key);
		if (!client) {
			LOGP(DOML, LOGL_ERROR, "Cannot find route!\n");
			goto err;
		}
		/* send OML message to respective client */
		DEBUGP(DOML, "Routed to client %s\n",
			oml_route_client_name(client));
		ipa_server_conn_send(client->conn, msg);
		break;
	default:
		LOGP(DOML, LOGL_NOTICE,
		     "Unhandled stream ID %u from BSC\n", hh->proto);
		msgb_free(msg);
		break;
	}

	return 0;
err:
	msgb_free(msg);
	return -1;
}

DEFUN(show_bsc_link, show_bsc_link_cmd,
	"show bsc-link",
	SHOW_STR "Display information about our link to the BSC\n")
{
	vty_out(vty, "BSC OML destination at %s:%u, State: %s%s",
		g_inst->bsc_oml_host, g_inst->bsc_oml_port,
		get_value_string(bsc_ls_names, g_inst->bsc_link_state),
		VTY_NEWLINE);

	return CMD_SUCCESS;
}

DEFUN(show_clients, show_clients_cmd,
	"show oml-clients",
	SHOW_STR "Display information about OML clients\n")
{
	struct oml_client *cl;

	llist_for_each_entry(cl, &g_inst->clients, list) {
		struct ipaccess_unit *ud = cl->unit_data;
		vty_out(vty, "Client '%s' from %s%s",
			oml_route_client_name(cl),
			oml_route_client_addr(cl), VTY_NEWLINE);
		vty_out(vty, " Name '%s', Serno '%s'%s",
			ud->unit_name, ud->serno, VTY_NEWLINE);
		vty_out(vty, " CCM Equipment Version: '%s', SW Version: '%s'%s",
			ud->equipvers, ud->swversion, VTY_NEWLINE);
		vty_out(vty, " CCM Location: '%s' / '%s'%s",
			ud->location1, ud->location2, VTY_NEWLINE);

	}

	return CMD_SUCCESS;
}

static struct cmd_node omlr_node = {
	OMLR_NODE,
	"%s(oml-router)# ",
	1,
};

DEFUN(cfg_omlr, cfg_omlr_cmd,
	"oml-router",
	"Configure the OML Router\n")
{
	vty->index = g_inst;
	vty->node = OMLR_NODE;

	return CMD_SUCCESS;
}

#define REMOTE_STR "Configuration of outbound OML connection\n"
#define LOCAL_STR "Configuration of OML server for inbound connections\n"

DEFUN(cfg_oml_remote_ip, cfg_oml_remote_ip_cmd,
	"remote connect-ip A.B.C.D", REMOTE_STR
	"Configure remote (BSC) IP address for OML\n"
	"IP address of BSC\n")
{
	struct oml_router *inst = vty->index;

	if (inst->bsc_oml_host)
		talloc_free(inst->bsc_oml_host);
	inst->bsc_oml_host = talloc_strdup(inst, argv[0]);

	return CMD_SUCCESS;
}

DEFUN(cfg_oml_remote_port, cfg_oml_remote_port_cmd,
	"remote connect-port <0-65535>", REMOTE_STR
	"Configure remote (BSC) TCP port for OML\n")
{
	struct oml_router *inst = vty->index;

	inst->bsc_oml_port = atoi(argv[0]);

	return CMD_SUCCESS;
}

DEFUN(cfg_oml_reconnect_secs, cfg_oml_reconnect_secs_cmd,
	"remote reconnect-timer <0-65535>", REMOTE_STR
	"Reconnect interval for OML connection to BSC\n")
{
	struct oml_router *inst = vty->index;

	inst->bsc_reconnect_secs = atoi(argv[0]);

	return CMD_SUCCESS;
}


DEFUN(cfg_oml_local_ip, cfg_oml_local_ip_cmd,
	"local listen-ip A.B.C.D", LOCAL_STR
	"Configure local listen IP address for incoming OML connections\n"
	"local IP address for incoming OML connections\n")
{
	struct oml_router *inst = vty->index;

	if (inst->listen_host)
		talloc_free(inst->listen_host);
	inst->listen_host = talloc_strdup(inst, argv[0]);

	return CMD_SUCCESS;
}

DEFUN(cfg_oml_local_port, cfg_oml_local_port_cmd,
	"local listen-port <0-65535>", LOCAL_STR
	"Configure local listen TCP port for incoming OML connections\n"
	"local TCP port for incoming OML connections\n")
{
	struct oml_router *inst = vty->index;

	inst->listen_port = atoi(argv[0]);

	return CMD_SUCCESS;
}

DEFUN(cfg_ipa_unit_id, cfg_ipa_unit_id_cmd,
	"ipa unit-id <0-65535> <0-255>",
	"Configure IPA parameters\n"
	"Site ID for this OML router\n"
	"BTS ID for this OML router\n")
{
	bts_dev_info.site_id = atoi(argv[0]);
	bts_dev_info.bts_id = atoi(argv[1]);

	return CMD_SUCCESS;
}

static int config_write_omlr(struct vty *vty)
{
	struct oml_router *inst = g_inst;

	vty_out(vty, "oml-router%s", VTY_NEWLINE);
	vty_out(vty, " ipa unit-id %u %u%s", bts_dev_info.site_id,
		bts_dev_info.bts_id, VTY_NEWLINE);
	vty_out(vty, " local listen-ip %s%s",
		inst->listen_host, VTY_NEWLINE);
	vty_out(vty, " local listen-port %u%s",
		inst->listen_port, VTY_NEWLINE);
	vty_out(vty, " remote connect-ip %s%s",
		inst->bsc_oml_host, VTY_NEWLINE);
	vty_out(vty, " remote connect-port %u%s",
		inst->bsc_oml_port, VTY_NEWLINE);
	vty_out(vty, " remote reconnect-timer %u%s",
		inst->bsc_reconnect_secs, VTY_NEWLINE);

	return CMD_SUCCESS;
}

static void *router_ctx;

struct log_info_cat router_cat[] = {
	[DOML] = {
		.name = "OML",
		.description = "A-bis OML (TS 12.21)",
		.enabled = 1, .loglevel = LOGL_DEBUG,
	},
};

static struct log_info router_log_info = {
	.filter_fn = NULL,
	.cat = router_cat,
	.num_cat = ARRAY_SIZE(router_cat),
};

static enum node_type omlr_vty_go_parent(struct vty *vty)
{
	switch (vty->node) {
	case OMLR_NODE:
		vty->node = CONFIG_NODE;
		break;
	default:
		vty->node = CONFIG_NODE;
	}
	return vty->node;
}

static int omlr_vty_is_config_node(struct vty *vty, int node)
{
	switch (node) {
	case OMLR_NODE:
		return 1;
	default:
		return 0;
	}
}

static const char copyright[] =
	"Copyright (C) 2014 by Harald Welte\r\n"
	"License AGPLv3+: GNU AGPL version 3 or later <http://gnu.org/licenses/agpl-3.0.html>\r\n"
	"This is free software: you are free to change and redistribute it.\r\n"
	 "There is NO WARRANTY, to the extent permitted by law.\r\n";


static struct vty_app_info vty_info = {
	.name		= "OsmoBTS-OMLrouter",
	.version	= PACKAGE_VERSION,
	.copyright	= copyright,
	.go_parent_cb	= omlr_vty_go_parent,
	.is_config_node = omlr_vty_is_config_node,
};

static void signal_handler(int signal)
{
	fprintf(stderr, "signal %u received\n", signal);

	switch (signal) {
	case SIGUSR1:
	case SIGUSR2:
		talloc_report_full(router_ctx, stderr);
		break;
	default:
		break;
	}
}

int main(int argc, char **argv)
{
	struct oml_router *inst;
	int rc;

	router_ctx = talloc_named_const(NULL, 1, "OML router");

	g_inst = inst = talloc_zero(router_ctx, struct oml_router);

	osmo_get_macaddr(bts_dev_info.mac_addr, "eth0");

	inst->bsc_oml_host = talloc_strdup(inst, "");
	inst->bsc_oml_port = 3002;
	inst->listen_host = talloc_strdup(inst, "0.0.0.0");
	inst->listen_port = 3002;
	inst->bsc_reconnect_secs = 10;

	INIT_LLIST_HEAD(&inst->clients);
	inst->bsc_recon_timer.cb = bsc_recon_timer_cb;
	inst->bsc_recon_timer.data = inst;

	libosmo_abis_init(router_ctx);

	osmo_init_logging(&router_log_info);

	//vty_info.tall_ctx = router_ctx;
	vty_init(&vty_info);

	install_node(&omlr_node, config_write_omlr);
	install_element(CONFIG_NODE, &cfg_omlr_cmd);
	install_element(OMLR_NODE, &cfg_oml_remote_ip_cmd);
	install_element(OMLR_NODE, &cfg_oml_remote_port_cmd);
	install_element(OMLR_NODE, &cfg_oml_reconnect_secs_cmd);
	install_element(OMLR_NODE, &cfg_oml_local_ip_cmd);
	install_element(OMLR_NODE, &cfg_oml_local_port_cmd);
	install_element(OMLR_NODE, &cfg_ipa_unit_id_cmd);

	install_element_ve(&show_bsc_link_cmd);
	install_element_ve(&show_clients_cmd);
	telnet_init(router_ctx, NULL, 4230);

	rc = vty_read_config_file(config_file, NULL);
	if (rc < 0) {
		fprintf(stderr, "Failed to parse the config file: '%s'\n",
			config_file);
		exit(1);
	}

	signal(SIGUSR1, &signal_handler);
	signal(SIGUSR2, &signal_handler);
	osmo_init_ignore_signals();

	inst->routing = oml_route_init(g_inst, g_inst);

	inst->server_link = ipa_server_link_create(router_ctx, NULL,
					inst->listen_host, inst->listen_port,
					server_accept_cb, inst);
	if (!inst->server_link) {
		LOGP(DOML, LOGL_ERROR, "Cannot create server instance\n");
		return -1;
	}


	if (ipa_server_link_open(inst->server_link) < 0) {
		LOGP(DOML, LOGL_ERROR, "Cannot bind server to TCP port %u\n",
		     inst->listen_port);
		return -1;
	}

	inst->bsc_conn = ipa_client_conn_create(router_ctx, NULL, 0,
					     inst->bsc_oml_host,
					     inst->bsc_oml_port,
					     bsc_updown_cb, bsc_read_cb,
					     NULL, inst);
	if (!inst->bsc_conn) {
		LOGP(DOML, LOGL_ERROR, "Cannot create client instance\n");
		return -1;
	}

	/* attempt tp connect and re-schedule connect if needed */
	bsc_recon_timer_cb(inst);

	if (daemonize) {
		rc = osmo_daemonize();
		if (rc < 0) {
			perror("Error during daemonize");
			exit(1);
		}
	}

	LOGP(DOML, LOGL_NOTICE, "entering main loop\n");

	while (1) {
		log_reset_context();
		osmo_select_main(0);
	}

	return 0;
}
