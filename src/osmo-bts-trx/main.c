/* Main program for OsmoBTS-TRX */

/* (C) 2011 by Harald Welte <laforge@gnumonks.org>
 * (C) 2013 by Andreas Eversberg <jolly@eversberg.eu>
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
#include <sched.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

#include <netinet/in.h>
#include <arpa/inet.h>
#include <net/if.h>

#include <osmocom/core/talloc.h>
#include <osmocom/core/application.h>
#include <osmocom/vty/telnet_interface.h>
#include <osmocom/vty/logging.h>
#include <osmocom/core/gsmtap.h>
#include <osmocom/core/gsmtap_util.h>
#include <osmocom/core/bits.h>

#include <osmo-bts/gsm_data.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/abis.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/vty.h>
#include <osmo-bts/bts_model.h>
#include <osmo-bts/pcu_if.h>
#include <osmo-bts/l1sap.h>

#include "l1_if.h"
#include "trx_if.h"
#include "scheduler.h"

const int pcu_direct = 0;

int quit = 0;
static const char *config_file = "osmo-bts.cfg";
static int daemonize = 0;
static char *gsmtap_ip = 0;
static int rt_prio = -1;
static int trx_num = 1;
char *software_version = "0.0";
uint8_t abis_mac[6] = { 0, 1, 2, 3, 4, 5 };
char *bsc_host = "localhost";
char *bts_id = "1801/0";

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

int bts_model_init(struct gsm_bts *bts)
{
	void *l1h;
	struct gsm_bts_trx *trx;

	llist_for_each_entry(trx, &bts->trx_list, list) {
		l1h = l1if_open(trx);
		if (!l1h) {
			LOGP(DL1C, LOGL_FATAL, "Cannot open L1 Interface\n");
			goto error;
		}

		trx->role_bts.l1h = l1h;
		trx->nominal_power = 23;

		l1if_reset(l1h);
	}

	bts_model_vty_init(bts);

	return 0;

error:
	llist_for_each_entry(trx, &bts->trx_list, list) {
		l1h = trx->role_bts.l1h;
		if (l1h)
			l1if_close(l1h);
	}

	return -EIO;
}

/* dummy, since no direct dsp support */
uint32_t trx_get_hlayer1(struct gsm_bts_trx *trx)
{
	return 0;
}

static void print_help()
{
	printf( "Some useful options:\n"
		"  -h	--help		this text\n"
		"  -d	--debug MASK	Enable debugging (e.g. -d DRSL:DOML:DLAPDM)\n"
		"  -D	--daemonize	For the process into a background daemon\n"
		"  -c	--config-file 	Specify the filename of the config file\n"
		"  -s	--disable-color	Don't use colors in stderr log output\n"
		"  -T	--timestamp	Prefix every log line with a timestamp\n"
		"  -V	--version	Print version information and exit\n"
		"  -e	--log-level	Set a global log-level\n"
		"  -t	--trx-num	Set number of TRX (default=%d)\n"
		"  -i	--gsmtap-ip	The destination IP used for GSMTAP.\n"
		"  -r	--realtime PRIO	Set realtime scheduler with given prio\n"
		"  -I	--local-trx-ip	Local IP for transceiver to connect (default=%s)\n"
		,trx_num, transceiver_ip);
}

/* FIXME: finally get some option parsing code into libosmocore */
static void handle_options(int argc, char **argv)
{
	while (1) {
		int option_idx = 0, c;
		static const struct option long_options[] = {
			/* FIXME: all those are generic Osmocom app options */
			{ "help", 0, 0, 'h' },
			{ "debug", 1, 0, 'd' },
			{ "daemonize", 0, 0, 'D' },
			{ "config-file", 1, 0, 'c' },
			{ "disable-color", 0, 0, 's' },
			{ "timestamp", 0, 0, 'T' },
			{ "version", 0, 0, 'V' },
			{ "log-level", 1, 0, 'e' },
			{ "trx-num", 1, 0, 't' },
			{ "gsmtap-ip", 1, 0, 'i' },
			{ "realtime", 1, 0, 'r' },
			{ "local-trx-ip", 1, 0, 'I' },
			{ 0, 0, 0, 0 }
		};

		c = getopt_long(argc, argv, "hc:d:Dc:sTVe:t:i:r:I:",
				long_options, &option_idx);
		if (c == -1)
			break;

		switch (c) {
		case 'h':
			print_help();
			exit(0);
			break;
		case 's':
			log_set_use_color(osmo_stderr_target, 0);
			break;
		case 'd':
			log_parse_category_mask(osmo_stderr_target, optarg);
			break;
		case 'D':
			daemonize = 1;
			break;
		case 'c':
			config_file = strdup(optarg);
			break;
		case 'T':
			log_set_print_timestamp(osmo_stderr_target, 1);
			break;
		case 'V':
			print_version(1);
			exit(0);
			break;
		case 'e':
			log_set_log_level(osmo_stderr_target, atoi(optarg));
			break;
		case 't':
			trx_num = atoi(optarg);
			if (trx_num < 1)
				trx_num = 1;
			break;
		case 'i':
			gsmtap_ip = optarg;
			break;
		case 'r':
			rt_prio = atoi(optarg);
			break;
		case 'I':
			transceiver_ip = strdup(optarg);
			break;
		default:
			break;
		}
	}
}

static struct gsm_bts *bts;

static void signal_handler(int signal)
{
	fprintf(stderr, "signal %u received\n", signal);

	switch (signal) {
	case SIGINT:
		//osmo_signal_dispatch(SS_GLOBAL, S_GLOBAL_SHUTDOWN, NULL);
		if (!quit)
			bts_shutdown(bts, "SIGINT");
		quit++;
		break;
	case SIGABRT:
	case SIGUSR1:
	case SIGUSR2:
		talloc_report_full(tall_bts_ctx, stderr);
		break;
	default:
		break;
	}
}

static int write_pid_file(char *procname)
{
	FILE *outf;
	char tmp[PATH_MAX+1];

	snprintf(tmp, sizeof(tmp)-1, "/var/run/%s.pid", procname);
	tmp[PATH_MAX-1] = '\0';

	outf = fopen(tmp, "w");
	if (!outf)
		return -1;

	fprintf(outf, "%d\n", getpid());

	fclose(outf);

	return 0;
}

int main(int argc, char **argv)
{
	struct gsm_bts_role_bts *btsb;
	struct gsm_bts_trx *trx;
	struct e1inp_line *line;
	void *tall_msgb_ctx;
	int rc, i;

	printf("((*))\n  |\n / \\ OsmoBTS\n");

	get_mac();

	tall_bts_ctx = talloc_named_const(NULL, 1, "OsmoBTS context");
	tall_msgb_ctx = talloc_named_const(tall_bts_ctx, 1, "msgb");
	msgb_set_talloc_ctx(tall_msgb_ctx);

	bts_log_init(NULL);

	handle_options(argc, argv);

	bts = gsm_bts_alloc(tall_bts_ctx);
	if (!bts) {
		fprintf(stderr, "Failed to create BTS structure\n");
		exit(1);
	}
	for (i = 1; i < trx_num; i++) {
		trx = gsm_bts_trx_alloc(bts);
		if (!trx) {
			fprintf(stderr, "Failed to TRX structure\n");
			exit(1);
		}
	}

	vty_init(&bts_vty_info);
	bts_vty_init(bts, trx_num, &bts_log_info);

	if (bts_init(bts) < 0) {
		fprintf(stderr, "unable to to open bts\n");
		exit(1);
	}
	btsb = bts_role_bts(bts);
	btsb->support.ciphers = CIPHER_A5(1) | CIPHER_A5(2);

        if (gsmtap_ip) {
		gsmtap = gsmtap_source_init(gsmtap_ip, GSMTAP_UDP_PORT, 1);
		if (!gsmtap) {
			fprintf(stderr, "Failed during gsmtap_init()\n");
			exit(1);
		}
		gsmtap_source_add_sink(gsmtap);
	}

	rc = vty_read_config_file(config_file, NULL);
	if (rc < 0) {
		fprintf(stderr, "Failed to parse the config file: '%s'\n",
			config_file);
		exit(1);
	}
	if (!settsc_enabled && !setbsic_enabled)
		settsc_enabled = setbsic_enabled = 1;

	write_pid_file("osmo-bts");

	rc = telnet_init(tall_bts_ctx, NULL, 4241);
	if (rc < 0) {
		fprintf(stderr, "Error initializing telnet\n");
		exit(1);
	}

	if (pcu_sock_init()) {
		fprintf(stderr, "PCU L1 socket failed\n");
		exit(-1);
	}

	signal(SIGINT, &signal_handler);
	//signal(SIGABRT, &signal_handler);
	signal(SIGUSR1, &signal_handler);
	signal(SIGUSR2, &signal_handler);
	osmo_init_ignore_signals();

	if (!btsb->bsc_oml_host) {
		fprintf(stderr, "Cannot start BTS without knowing BSC OML IP\n");
		exit(1);
	}

	line = abis_open(bts, btsb->bsc_oml_host, "sysmoBTS");
	if (!line) {
		fprintf(stderr, "unable to connect to BSC\n");
		exit(1);
	}

	if (daemonize) {
		rc = osmo_daemonize();
		if (rc < 0) {
			perror("Error during daemonize");
			exit(1);
		}
	}

	if (rt_prio != -1) {
		struct sched_param schedp;

		/* high priority scheduling required for handling bursts */
		memset(&schedp, 0, sizeof(schedp));
		schedp.sched_priority = rt_prio;
		rc = sched_setscheduler(0, SCHED_RR, &schedp);
		if (rc) {
			fprintf(stderr, "Error setting SCHED_RR with prio %d\n",
				rt_prio);
		}
	}

	while (quit < 2) {
		log_reset_context();
		osmo_select_main(0);
	}

#if 0
	telnet_exit();

	talloc_report_full(tall_bts_ctx, stderr);
#endif

	return 0;
}

