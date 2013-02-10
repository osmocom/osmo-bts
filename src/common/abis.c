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

#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>

#include <osmocom/core/select.h>
#include <osmocom/core/timer.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/signal.h>
#include <osmocom/abis/abis.h>
#include <osmocom/abis/e1_input.h>
#include <osmocom/abis/ipaccess.h>

#include <osmo-bts/logging.h>
#include <osmo-bts/gsm_data.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/rsl.h>
#include <osmo-bts/oml.h>

static struct gsm_bts *g_bts;

int abis_oml_sendmsg(struct msgb *msg)
{
	struct gsm_bts *bts = msg->trx->bts;

	/* osmo-bts uses msg->trx internally, but libosmo-abis uses
	 * the signalling link at msg->dst */
	msg->dst = bts->oml_link;
	return abis_sendmsg(msg);
}

int abis_rsl_sendmsg(struct msgb *msg)
{
	/* osmo-bts uses msg->trx internally, but libosmo-abis uses
	 * the signalling link at msg->dst */
	msg->dst = msg->trx->rsl_link;
	return abis_sendmsg(msg);
}

static struct e1inp_sign_link *sign_link_up(void *unit, struct e1inp_line *line,
					    enum e1inp_sign_type type)
{
	struct e1inp_sign_link *sign_link = NULL;

	switch (type) {
	case E1INP_SIGN_OML:
		LOGP(DABIS, LOGL_INFO, "OML Signalling link up\n");
		e1inp_ts_config_sign(&line->ts[E1INP_SIGN_OML-1], line);
		sign_link = g_bts->oml_link =
			e1inp_sign_link_create(&line->ts[E1INP_SIGN_OML-1],
						E1INP_SIGN_OML, NULL, 255, 0);
		sign_link->trx = g_bts->c0;
		bts_link_estab(g_bts);
		break;
	case E1INP_SIGN_RSL:
		LOGP(DABIS, LOGL_INFO, "RSL Signalling link up\n");
		e1inp_ts_config_sign(&line->ts[E1INP_SIGN_RSL-1], line);
		sign_link = g_bts->c0->rsl_link =
			e1inp_sign_link_create(&line->ts[E1INP_SIGN_RSL-1],
						E1INP_SIGN_RSL, NULL, 0, 0);
		/* FIXME: This assumes there is only one TRX! */
		sign_link->trx = g_bts->c0;
		trx_link_estab(sign_link->trx);
		break;
	default:
		break;
	}

	return sign_link;
}

static void sign_link_down(struct e1inp_line *line)
{
	LOGP(DABIS, LOGL_ERROR, "Signalling link down\n");

	if (g_bts->c0->rsl_link) {
		e1inp_sign_link_destroy(g_bts->c0->rsl_link);
		g_bts->c0->rsl_link = NULL;
		trx_link_estab(g_bts->c0);
	}

	e1inp_sign_link_destroy(g_bts->oml_link);
	g_bts->oml_link = NULL;

	bts_shutdown(g_bts, "Abis close");
}


/* callback for incoming mesages from A-bis/IP */
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
	default:
		msgb_free(msg);
		break;
	}

	return 0;
}


#include <sys/ioctl.h>
#include <net/if.h>
#include <netinet/ip.h>

static int get_mac_addr(const char *dev_name, uint8_t *mac_out)
{
	int fd, rc;
	struct ifreq ifr;

	fd = socket(PF_INET, SOCK_DGRAM, IPPROTO_IP);
	if (fd < 0)
		return fd;

	memset(&ifr, 0, sizeof(ifr));
	memcpy(&ifr.ifr_name, dev_name, sizeof(ifr.ifr_name));
	rc = ioctl(fd, SIOCGIFHWADDR, &ifr);
	if (rc < 0)
		return rc;

	memcpy(mac_out, ifr.ifr_hwaddr.sa_data, 6);

	close(fd);

	return 0;
}

static int inp_s_cbfn(unsigned int subsys, unsigned int signal,
		      void *hdlr_data, void *signal_data)
{
	if (subsys != SS_L_INPUT)
		return 0;

	DEBUGP(DABIS, "Input Signal %u received\n", signal);

	return 0;
}


static struct ipaccess_unit bts_dev_info = {
	.unit_name	= "sysmoBTS",
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

/* UGLY: we assume this function is only called once as it does some
 * global initialization as well as the actual opening of the A-bis link
 * */
struct e1inp_line *abis_open(struct gsm_bts *bts, const char *dst_host,
			     const char *model_name)
{
	struct e1inp_line *line;

	g_bts = bts;

	oml_init();
	libosmo_abis_init(NULL);

	osmo_signal_register_handler(SS_L_INPUT, &inp_s_cbfn, bts);

	/* patch in various data from VTY and othe sources */
	line_ops.cfg.ipa.addr = dst_host;
	get_mac_addr("eth0", bts_dev_info.mac_addr);
	bts_dev_info.site_id = bts->ip_access.site_id;
	bts_dev_info.bts_id = bts->ip_access.bts_id;
	bts_dev_info.unit_name = model_name;
	if (bts->description)
		bts_dev_info.unit_name = bts->description;
	bts_dev_info.location2 = model_name;

	line = e1inp_line_create(0, "ipa");
	if (!line)
		return NULL;
	e1inp_line_bind_ops(line, &line_ops);

	/* This is what currently starts both the outbound OML and RSL
	 * connections, which is wrong.
	 * FIXME: It should only start OML and wait for the RLS IP
	 * address to be set as part of the TRX attributes */
	if (e1inp_line_update(line) < 0)
		return NULL;

	return line;
}
