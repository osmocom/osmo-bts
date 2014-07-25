/* Main program for SysmoBTS management daemon */

/* (C) 2012 by Harald Welte <laforge@gnumonks.org>
 * (C) 2014 by Holger Hans Peter Freyther
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
#include <stdlib.h>
#include <errno.h>
#include <getopt.h>
#include <limits.h>
#include <arpa/inet.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <osmocom/core/talloc.h>
#include <osmocom/core/application.h>
#include <osmocom/core/timer.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/serial.h>
#include <osmocom/vty/telnet_interface.h>
#include <osmocom/vty/logging.h>

#include "btsconfig.h"
#include "misc/sysmobts_misc.h"
#include "misc/sysmobts_mgr.h"
#include "misc/sysmobts_nl.h"
#include "misc/sysmobts_par.h"

static int no_eeprom_write = 0;
static int daemonize = 0;
void *tall_mgr_ctx;

/* every 6 hours means 365*4 = 1460 EEprom writes per year (max) */
#define TEMP_TIMER_SECS		(6 * 3600)

/* every 1 hours means 365*24 = 8760 EEprom writes per year (max) */
#define HOURS_TIMER_SECS	(1 * 3600)

#ifdef BUILD_SBTS2050
static struct osmo_timer_list temp_uc_timer;
static void check_uctemp_timer_cb(void *data)
{
	int temp_pa = 0, temp_board = 0;
	struct uc *ucontrol0 = data;

	sbts2050_uc_check_temp(ucontrol0, &temp_pa, &temp_board);

	osmo_timer_schedule(&temp_uc_timer, TEMP_TIMER_SECS, 0);
}
#endif

static void initialize_sbts2050(void)
{
#ifdef BUILD_SBTS2050
	static struct uc ucontrol0 = {
		.id = 0,
		.path = "/dev/ttyS0"
	};
	int val;

	if (sysmobts_par_get_int(SYSMOBTS_PAR_MODEL_NR, &val) < 0) {
		LOGP(DFIND, LOGL_ERROR,
		     "Failed to get Model number\n");
		return;
	}

	if (val != 2050)
		return;

	if (sysmobts_par_get_int(SYSMOBTS_PAR_TRX_NR, &val) < 0) {
		LOGP(DFIND, LOGL_ERROR, "Failed to get the TRX number\n");
		return;
	}

	if (val != 0)
		return;

	ucontrol0.fd = osmo_serial_init(ucontrol0.path, 115200);
	if (ucontrol0.fd < 0) {
		LOGP(DFIND, LOGL_ERROR,
		     "Failed to open the serial interface\n");
		return;
	}

	temp_uc_timer.cb = check_uctemp_timer_cb;
	temp_uc_timer.data = &ucontrol0;
	check_uctemp_timer_cb(&ucontrol0);
#endif
}

static struct osmo_timer_list temp_timer;
static void check_temp_timer_cb(void *unused)
{
	sysmobts_check_temp(no_eeprom_write);

	osmo_timer_schedule(&temp_timer, TEMP_TIMER_SECS, 0);
}

static struct osmo_timer_list hours_timer;
static void hours_timer_cb(void *unused)
{
	sysmobts_update_hours(no_eeprom_write);

	osmo_timer_schedule(&hours_timer, HOURS_TIMER_SECS, 0);
}

static void print_help(void)
{
	printf("sysmobts-mgr [-nsD] [-d cat]\n");
	printf(" -n Do not write to EEPROM\n");
	printf(" -s Disable color\n");
	printf(" -d CAT enable debugging\n");
	printf(" -D daemonize\n");
}

static int parse_options(int argc, char **argv)
{
	int opt;

	while ((opt = getopt(argc, argv, "nhsd:")) != -1) {
		switch (opt) {
		case 'n':
			no_eeprom_write = 1;
			break;
		case 'h':
			print_help();
			return -1;
		case 's':
			log_set_use_color(osmo_stderr_target, 0);
			break;
		case 'd':
			log_parse_category_mask(osmo_stderr_target, optarg);
			break;
		case 'D':
			daemonize = 1;
			break;
		default:
			return -1;
		}
	}

	return 0;
}

static void signal_handler(int signal)
{
	fprintf(stderr, "signal %u received\n", signal);

	switch (signal) {
	case SIGINT:
		sysmobts_check_temp(no_eeprom_write);
		sysmobts_update_hours(no_eeprom_write);
		exit(0);
		break;
	case SIGABRT:
	case SIGUSR1:
	case SIGUSR2:
		talloc_report_full(tall_mgr_ctx, stderr);
		break;
	default:
		break;
	}
}

#include <osmocom/core/logging.h>
#include <osmocom/core/application.h>
#include <osmocom/core/utils.h>
#include <osmocom/core/socket.h>

#include <osmocom/gsm/protocol/ipaccess.h>
#include <osmocom/gsm/tlv.h>

#include <osmo-bts/bts.h>
#include <osmo-bts/logging.h>

static struct log_info_cat mgr_log_info_cat[] = {
	[DTEMP] = {
		.name = "DTEMP",
		.description = "Temperature monitoring",
		.color = "\033[1;35m",
		.enabled = 1, .loglevel = LOGL_INFO,
	},
	[DFW] =	{
		.name = "DFW",
		.description = "DSP/FPGA firmware management",
		.color = "\033[1;36m",
		.enabled = 1, .loglevel = LOGL_INFO,
	},
	[DFIND] = {
		.name = "DFIND",
		.description = "ipaccess-find handling",
		.color = "\033[1;37m",
		.enabled = 1, .loglevel = LOGL_INFO,
	},
};

static const struct log_info mgr_log_info = {
	.cat = mgr_log_info_cat,
	.num_cat = ARRAY_SIZE(mgr_log_info_cat),
};

static int mgr_log_init(void)
{
	osmo_init_logging(&mgr_log_info);
	return 0;
}

/*
 * The TLV structure in IPA messages in UDP packages is a bit
 * weird. First the header appears to have an extra NULL byte
 * and second the L16 of the L16TV needs to include +1 for the
 * tag. The default msgb/tlv and libosmo-abis routines do not
 * provide this.
 */

static void ipaccess_prepend_header_quirk(struct msgb *msg, int proto)
{
	struct ipaccess_head *hh;

	/* prepend the ip.access header */
	hh = (struct ipaccess_head *) msgb_push(msg, sizeof(*hh) + 1);
	hh->len = htons(msg->len - sizeof(*hh) - 1);
	hh->proto = proto;
}

static void quirk_l16tv_put(struct msgb *msg, uint16_t len, uint8_t tag,
			const uint8_t *val)
{
	uint8_t *buf = msgb_put(msg, len + 2 + 1);

	*buf++ = (len + 1) >> 8;
	*buf++ = (len + 1) & 0xff;
	*buf++ = tag;
	memcpy(buf, val, len);
}

/*
 * We don't look at the content of the request yet and lie
 * about most of the responses.
 */
static void respond_to(struct sockaddr_in *src, struct osmo_fd *fd,
			uint8_t *data, size_t len)
{
	static int fetched_info = 0;
	static char mac_str[20] = { };
	static char *model_name;

	struct sockaddr_in loc_addr;
	int rc;
	char loc_ip[INET_ADDRSTRLEN];
	struct msgb *msg = msgb_alloc_headroom(512, 128, "ipa get response");
	if (!msg) {
		LOGP(DFIND, LOGL_ERROR, "Failed to allocate msgb\n");
		return;
	}

	if (!fetched_info) {
		uint8_t mac[6];
		int val;

		/* fetch the MAC */
		sysmobts_par_get_buf(SYSMOBTS_PAR_MAC, mac, sizeof(mac));
		snprintf(mac_str, sizeof(mac_str), "%.2x:%.2x:%.2x:%.2x:%.2x:%.2x",
				mac[0], mac[1], mac[2],
				mac[3], mac[4], mac[5]);

		/* fetch the model and trx number */
		sysmobts_par_get_int(SYSMOBTS_PAR_MODEL_NR, &val);
		switch(val) {
		case 0:
		case 0xffff:
		case 1002:
			model_name = "sysmoBTS 1002";
			break;
		case 2050:
			sysmobts_par_get_int(SYSMOBTS_PAR_TRX_NR, &val);
			if (val == 0)
				model_name = "sysmoBTS 2050 (master)";
			else if (val == 1)
				model_name = "sysmoBTS 2050 (slave)";
			else
				model_name = "sysmoBTS 2050 (unknown)";
			break;
		default:
			model_name = "Unknown";
			break;
		}


		fetched_info = 1;
	}

	if (source_for_dest(&src->sin_addr, &loc_addr.sin_addr) != 0) {
		LOGP(DFIND, LOGL_ERROR, "Failed to determine local source\n");
		return;
	}

	msgb_put_u8(msg, IPAC_MSGT_ID_RESP);

	/* append MAC addr */
	quirk_l16tv_put(msg, strlen(mac_str) + 1, IPAC_IDTAG_MACADDR, (uint8_t *) mac_str);

	/* append ip address */
	inet_ntop(AF_INET, &loc_addr.sin_addr, loc_ip, sizeof(loc_ip));
	quirk_l16tv_put(msg, strlen(loc_ip) + 1, IPAC_IDTAG_IPADDR, (uint8_t *) loc_ip);

	/* abuse some flags */
	quirk_l16tv_put(msg, strlen(model_name) + 1, IPAC_IDTAG_UNIT, (uint8_t *) model_name);

	/* ip.access nanoBTS would reply to port==3006 */
	ipaccess_prepend_header_quirk(msg, IPAC_PROTO_IPACCESS);
	rc = sendto(fd->fd, msg->data, msg->len, 0, (struct sockaddr *)src, sizeof(*src));
	if (rc != msg->len)
		LOGP(DFIND, LOGL_ERROR,
			"Failed to send with rc(%d) errno(%d)\n", rc, errno);
}

static int ipaccess_bcast(struct osmo_fd *fd, unsigned int what)
{
	uint8_t data[2048];
	char src[INET_ADDRSTRLEN];
	struct sockaddr_in addr = {};
	socklen_t len = sizeof(addr);
	int rc;

	rc = recvfrom(fd->fd, data, sizeof(data), 0,
			(struct sockaddr *) &addr, &len);
	if (rc <= 0) {
		LOGP(DFIND, LOGL_ERROR,
			"Failed to read from socket errno(%d)\n", errno);
		return -1;
	}

	LOGP(DFIND, LOGL_DEBUG,
		"Received request from: %s size %d\n",
		inet_ntop(AF_INET, &addr.sin_addr, src, sizeof(src)), rc);

	if (rc < 6)
		return 0;

	if (data[2] != IPAC_PROTO_IPACCESS || data[4] != IPAC_MSGT_ID_GET)
		return 0;

	respond_to(&addr, fd, data + 6, rc - 6);
	return 0;
}

int main(int argc, char **argv)
{
	struct osmo_fd fd;
	void *tall_msgb_ctx;
	int rc;


	tall_mgr_ctx = talloc_named_const(NULL, 1, "bts manager");
	tall_msgb_ctx = talloc_named_const(tall_mgr_ctx, 1, "msgb");
	msgb_set_talloc_ctx(tall_msgb_ctx);

	mgr_log_init();

	osmo_init_ignore_signals();
	signal(SIGINT, &signal_handler);
	signal(SIGUSR1, &signal_handler);
	signal(SIGUSR2, &signal_handler);

	rc = parse_options(argc, argv);
	if (rc < 0)
		exit(2);

	/* start temperature check timer */
	temp_timer.cb = check_temp_timer_cb;
	check_temp_timer_cb(NULL);

	/* start operational hours timer */
	hours_timer.cb = hours_timer_cb;
	hours_timer_cb(NULL);

	/* start uc temperature check timer */
	initialize_sbts2050();

	/* handle broadcast messages for ipaccess-find */
	fd.cb = ipaccess_bcast;
	rc = osmo_sock_init_ofd(&fd, AF_INET, SOCK_DGRAM, IPPROTO_UDP,
				"0.0.0.0", 3006, OSMO_SOCK_F_BIND);
	if (rc < 0) {
		perror("Socket creation");
		exit(3);
	}

	if (daemonize) {
		rc = osmo_daemonize();
		if (rc < 0) {
			perror("Error during daemonize");
			exit(1);
		}
	}


	while (1) {
		log_reset_context();
		osmo_select_main(0);
	}
}
