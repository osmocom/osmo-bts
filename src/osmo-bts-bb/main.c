/* (C) 2011 by Andreas Eversberg <jolly@eversberg.eu>
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

#include <netinet/in.h>
#include <osmocore/talloc.h>
#include <osmocore/signal.h>
#include <osmocore/timer.h>
#include <osmocore/select.h>
#include <osmocore/signal.h>
#include <osmo-bts/logging.h>
//#include <osmocom/bb/common/osmocom_data.h>
#include <osmo-bts/support.h>
#include <osmo-bts/abis.h>
#include <osmo-bts/rtp.h>
#include <osmo-bts/bts.h>

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

struct log_target *stderr_target;
char *debugs = "DL1C:DLAPDM:DABIS:DOML:DRSL:DSUM";

void *l23_ctx = NULL;
static struct osmocom_bts *bts;
int debug_set = 0;
char *software_version = "0.0";
uint8_t abis_mac[6] = { 0, 1, 2, 3, 4, 5 };
char *bsc_host = "localhost";
char *bts_id = "1801/0";
int quit = 0;

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

static void print_usage(const char *app)
{
	printf("Usage: %s [option]\n", app);
	printf("  -h --help		this text\n");
	printf("  -d --debug		Change debug flags. (-d %s)\n", debugs);
	printf("  -i --bsc-ip           IP address of the BSC\n");
}

static void handle_options(int argc, char **argv)
{
	while (1) {
		int option_index = 0, c;
		static struct option long_options[] = {
			{"help", 0, 0, 'h'},
			{"debug", 1, 0, 'd'},
			{"bsc-ip", 1, 0, 'i'},
			{0, 0, 0, 0},
		};

		c = getopt_long(argc, argv, "hd:i:",
				long_options, &option_index);
		if (c == -1)
			break;

		switch (c) {
		case 'h':
			print_usage(argv[0]);
			exit(0);
		case 'd':
			log_parse_category_mask(stderr_target, optarg);
			debug_set = 1;
			break;
		case 'i':
			bsc_host = strdup(optarg);
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

	fprintf(stderr, "Signal %d recevied.\n", sigset);

	/* in case there is a lockup during exit */
	signal(SIGINT, SIG_DFL);
	signal(SIGHUP, SIG_DFL);
	signal(SIGTERM, SIG_DFL);
	signal(SIGPIPE, SIG_DFL);

	quit = 1;
//	dispatch_signal(SS_GLOBAL, S_GLOBAL_SHUTDOWN, NULL);
}

int main(int argc, char **argv)
{
	int ret;
	struct hostent *hostent;
	struct in_addr *ina;
	uint32_t bsc_ip;
	uint8_t maskv_tx[2], maskv_rx[2];

	printf("((*))\n");
	printf("  |\n");
	printf(" / \\ OsmoBTS\n");

	get_mac();
	bts_support_init();

	srand(time(NULL));

	log_init(&log_info);
	stderr_target = log_target_create_stderr();
	log_add_target(stderr_target);
	log_set_all_filter(stderr_target, 1);

	l23_ctx = talloc_named_const(NULL, 1, "layer2 context");

	handle_options(argc, argv);

	if (!debug_set)
		log_parse_category_mask(stderr_target, debugs);
	log_set_log_level(stderr_target, LOGL_INFO);

	hostent = gethostbyname(bsc_host);
	if (!hostent) {
		fprintf(stderr, "Failed to resolve BSC hostname '%s'.\n",
			bsc_host);
	}
	ina = (struct in_addr *) hostent->h_addr;
	bsc_ip = ntohl(ina->s_addr);
	printf("Using BSC at IP: '%d.%d.%d.%d'\n", bsc_ip >> 24,
		(bsc_ip >> 16) & 0xff, (bsc_ip >> 8) & 0xff, bsc_ip & 0xff);

	printf("Using BTS ID: '%s'\n", bts_id);
	bts = create_bts(1, bts_id);
	if (!bts)
		exit(-1);
	maskv_tx[0] = 0x55;
	maskv_rx[0] = 0x55;
	ret = create_ms(bts->trx[0], 1, maskv_tx, maskv_rx);
	if (ret < 0)
		goto fail;
	ret = abis_open(&bts->link, bsc_ip);
	if (ret < 0)
		goto fail;

	signal(SIGINT, sighandler);
	signal(SIGHUP, sighandler);
	signal(SIGTERM, sighandler);
	signal(SIGPIPE, sighandler);

	while (!quit) {
		work_bts(bts);
		bsc_select_main(0);
	}

fail:
	destroy_bts(bts);

	return 0;
}
