/* (C) 2011 by Andreas Eversberg <jolly@eversberg.eu>
 * (C) 2011 by Harald Welte <laforge@gnumonks.org>
 *
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#include <arpa/inet.h>
#include <netinet/in.h>

#include <osmocom/core/talloc.h>
#include <osmocom/core/application.h>
#include <osmocom/vty/telnet_interface.h>
#include <osmocom/vty/logging.h>
#include <osmo-bts/gsm_data.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/abis.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/vty.h>

#include "l1_if.h"

#include <net/if.h>

#define _GNU_SOURCE
#include <getopt.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <netdb.h>
#include <string.h>
#include <time.h>

char *debugs = "DL1C:DLLAPDM:DABIS:DOML:DRSL:DSUM";
int debug_set = 0;
int level_set = 0;
int ref_set = 0;
static int daemonize = 0;
void *l23_ctx = NULL;
static struct gsm_bts *bts;
char *software_version = "0.0";
uint8_t abis_mac[6] = { 0, 1, 2, 3, 4, 5 };
char *bsc_host = "localhost";
uint16_t site_id = 1801, bts_id = 0;

// FIXME this is a hack
static void get_mac(void)
{
	struct if_nameindex *ifn = if_nameindex();
	struct ifreq ifr;
	int sock;
	int ret;

	sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sock < 0)
		return;

	memset(&ifr, 0, sizeof(ifr));
	if (!ifn)
		return;
	while (ifn->if_name) {
		strncpy(ifr.ifr_name, ifn->if_name, sizeof(ifr.ifr_name)-1);
		ret = ioctl(sock, SIOCGIFHWADDR, &ifr);
		if (ret == 0 && !!memcmp(ifr.ifr_hwaddr.sa_data,
					"\0\0\0\0\0\0", 6)) {
			memcpy(abis_mac, ifr.ifr_hwaddr.sa_data, 6);
			printf("Using MAC address of %s: "
				"'%02x:%02x:%02x:%02x:%02x:%02x'\n",
				ifn->if_name,
				abis_mac[0], abis_mac[1], abis_mac[2],
				abis_mac[3], abis_mac[4], abis_mac[5]);
			break;
		}
		ifn++;
	}
//	if_freenameindex(ifn);
}

struct ipabis_link *link_init(struct gsm_bts *bts, uint32_t bsc_ip)
{
	struct ipabis_link *link = talloc_zero(bts, struct ipabis_link);
	int rc;

	link->bts = bts;
	bts->oml_link = link;

	rc = abis_open(link, bsc_ip);
	if (rc < 0)
		return NULL;

	return link;
}

static void print_usage(const char *app)
{
	printf("Usage: %s -r <arfcn> [option]\n", app);
	printf("  -h --help             this text\n");
	printf("  -r --ref-arfcn        Set channel number of reference BTS for clocking\n");
	printf("  -d --debug            Change debug flags. (default %s)\n", debugs);
	printf("  -s --disable-color    Don't use colors in stderr log output\n");
	printf("  -T --timestamp        Prefix every log line with a timestamp\n");
	printf("  -e --log-level        Set a global log-level\n");
	printf("  -i --bsc-ip           Hostname or IP address of the BSC\n");
	printf("  -D --daemonize        For the process into a background daemon\n");
}

static void bts_logo(void)
{
	printf("\n   ((*))\n");
	printf("     |\n");
	printf("    / \\ OsmoBTS\n");
	printf("-------------------\n");
}
static void handle_options(int argc, char **argv)
{
	while (1) {
		int option_index = 0, c;
		static struct option long_options[] = {
			{ "help", 0, 0, 'h' },
			{ "ref-arfcn", 1, 0, 'r' },
			{ "debug", 1, 0, 'd' },
			{ "disable-color", 0, 0, 's' },
			{ "timestamp", 0, 0, 'T' },
			{ "log-level", 1, 0, 'e' },
			{ "bsc-ip", 1, 0, 'i' },
			{ "daemonize", 0, 0, 'D' },
			{0, 0, 0, 0},
		};

		c = getopt_long(argc, argv, "hr:d:sTe:i:D",
				long_options, &option_index);
		if (c == -1)
			break;

		switch (c) {
		case 'h':
			print_usage(argv[0]);
			exit(0);
		case 'r':
			ref_arfcn = atoi(optarg);
			ref_set = 1;
			break;
		case 'd':
			log_parse_category_mask(osmo_stderr_target, optarg);
			debug_set = 1;
			break;
		case 's':
			log_set_use_color(osmo_stderr_target, 0);
			break;
		case 'T':
			log_set_print_timestamp(osmo_stderr_target, 1);
			break;
		case 'e':
			log_set_log_level(osmo_stderr_target, atoi(optarg));
			level_set = 1;
			break;
		case 'i':
			bsc_host = strdup(optarg);
			break;
		case 'D':
			daemonize = 1;
			break;
		default:
			break;
		}
	}
}

void sighandler(int sigset)
{
	if (sigset == SIGHUP || sigset == SIGPIPE)
		return;

	fprintf(stderr, "Signal %d recevied. Press ^C again to terminate "
		"instantly.\n", sigset);

	/* in case there is a lockup during exit */
	signal(SIGINT, SIG_DFL);
	signal(SIGHUP, SIG_DFL);
	signal(SIGTERM, SIG_DFL);
	signal(SIGPIPE, SIG_DFL);

	bts_shutdown(bts, "SIGINT", 1);
//	dispatch_signal(SS_GLOBAL, S_GLOBAL_SHUTDOWN, NULL);
}

int main(int argc, char **argv)
{
	struct ipabis_link *link;
	uint32_t bsc_ip;
	struct hostent *hostent;
	struct in_addr *ina;
	void *tall_msgb_ctx;
	int rc;
#if 0
	int ret;
	uint8_t maskv_tx[2], maskv_rx[2];
#endif

	bts_logo();

	get_mac();

	tall_bts_ctx = talloc_named_const(NULL, 1, "OsmoBTS context");
	tall_msgb_ctx = talloc_named_const(tall_bts_ctx, 1, "msgb");
	msgb_set_talloc_ctx(tall_msgb_ctx);

	bts_log_init(NULL);

	bts = gsm_bts_alloc(tall_bts_ctx);

	vty_init(&bts_vty_info);
	logging_vty_add_cmds(&bts_log_info);

	rc = telnet_init(tall_bts_ctx, NULL, 4241);
	if (rc < 0) {
		fprintf(stderr, "Error initializing telnet\n");
		exit(1);
	}

	handle_options(argc, argv);

	if (!ref_set) {
		fprintf(stderr, "Error: ARFCN for reference clock not specified. Use '-h' for help.\n");
		exit(1);
	}
	printf("Using ARFCN '%d' for clock reference.\n", ref_arfcn);

	if (!debug_set)
		log_parse_category_mask(osmo_stderr_target, debugs);
	if (!level_set)
		log_set_log_level(osmo_stderr_target, LOGL_INFO);

	if (bts_init(bts) < 0) {
		fprintf(stderr, "unable to to open bts\n");
		exit(1);
	}

	hostent = gethostbyname(bsc_host);
	if (!hostent) {
		fprintf(stderr, "Failed to resolve BSC hostname '%s'.\n",
			bsc_host);
	}
	ina = (struct in_addr *) hostent->h_addr;
	bsc_ip = ntohl(ina->s_addr);
	printf("Using BSC at IP: '%d.%d.%d.%d'\n", bsc_ip >> 24,
		(bsc_ip >> 16) & 0xff, (bsc_ip >> 8) & 0xff, bsc_ip & 0xff);
	link = link_init(bts, bsc_ip);
	if (!link) {
		fprintf(stderr, "unable to connect to BSC\n");
		exit(1);
	}

	printf("Using BTS ID: '%d/%d'\n", site_id, bts_id);
	bts->ip_access.site_id = site_id;
	bts->ip_access.bts_id = bts_id;

	if (daemonize) {
		rc = osmo_daemonize();
		if (rc < 0) {
			perror("Error during daemonize");
			exit(1);
		}
	}

	signal(SIGINT, sighandler);
	signal(SIGHUP, sighandler);
	signal(SIGTERM, sighandler);
	signal(SIGPIPE, sighandler);
	osmo_init_ignore_signals();

	while (1) {
		log_reset_context();
		osmo_select_main(0);
	}

#if 0

	maskv_tx[0] = 0x55;
	maskv_rx[0] = 0x55;
	ret = create_ms(bts->trx[0], 1, maskv_tx, maskv_rx);
	if (ret < 0)
		goto fail;

fail:
	destroy_bts(bts);

	return 0;

#endif
}

int bts_model_init(struct gsm_bts *bts)
{
	l1if_open();

	/* send reset to baseband */
	l1if_reset(bts->c0);
 
	return 0;
}

