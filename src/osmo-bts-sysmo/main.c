/* Main program for Sysmocom BTS */

/* (C) 2011-2013 by Harald Welte <laforge@gnumonks.org>
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
#include <sys/signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sched.h>

#include <netinet/in.h>
#include <arpa/inet.h>

#include <osmocom/core/talloc.h>
#include <osmocom/core/application.h>
#include <osmocom/vty/telnet_interface.h>
#include <osmocom/vty/logging.h>

#include <osmo-bts/gsm_data.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/abis.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/vty.h>
#include <osmo-bts/bts_model.h>
#include <osmo-bts/pcu_if.h>

#define SYSMOBTS_RF_LOCK_PATH	"/var/lock/bts_rf_lock"

#include "eeprom.h"
#include "l1_if.h"

/* FIXME: read from real hardware */
const uint8_t abis_mac[6] = { 0,1,2,3,4,5 };
int pcu_direct = 0;

static const char *config_file = "osmo-bts.cfg";
static int daemonize = 0;
static unsigned int dsp_trace = 0x71c00020;
static int rt_prio = -1;

int bts_model_init(struct gsm_bts *bts)
{
	struct femtol1_hdl *fl1h;

	fl1h = l1if_open(bts->c0);
	if (!fl1h) {
		LOGP(DL1C, LOGL_FATAL, "Cannot open L1 Interface\n");
		return -EIO;
	}
	fl1h->dsp_trace_f = dsp_trace;

	bts->c0->role_bts.l1h = fl1h;
	bts->c0->nominal_power = 23;

	bts_model_vty_init(bts);

	return 0;
}

int bts_model_oml_estab(struct gsm_bts *bts)
{
	struct femtol1_hdl *fl1h = bts->c0->role_bts.l1h;

	l1if_reset(fl1h);

	return 0;
}

/* Set the clock calibration to the value
 * read from the eeprom.
 */
void clk_cal_use_eeprom(struct gsm_bts *bts)
{
	int rc;
	struct femtol1_hdl *hdl;
	eeprom_RfClockCal_t rf_clk;

	hdl = bts->c0->role_bts.l1h;

	if (!hdl || !hdl->clk_use_eeprom)
		return;

	rc = eeprom_ReadRfClockCal(&rf_clk);
	if (rc != EEPROM_SUCCESS) {
		LOGP(DL1C, LOGL_ERROR, "Failed to read from EEPROM.\n");
		return;
	}

	hdl->clk_cal = rf_clk.iClkCor;
	LOGP(DL1C, LOGL_NOTICE,
		"Read clock calibration(%d) from EEPROM.\n", hdl->clk_cal);
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
		"  -e 	--log-level	Set a global log-level\n"
		"  -p	--dsp-trace	Set DSP trace flags\n"
		"  -r	--realtime PRIO	Use SCHED_RR with the specified priority\n"
		"  -w	--hw-version	Print the targeted HW Version\n"
		"  -M	--pcu-direct	Force PCU to access message queue for "
			"PDCH dchannel directly\n"
		);
}

static void print_hwversion()
{
#ifdef HW_SYSMOBTS_V1
	printf("sysmobts was compiled for hw version 1.\n");
#else
	printf("sysmobts was compiled for hw version 2.\n");
#endif
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
			{ "dsp-trace", 1, 0, 'p' },
			{ "hw-version", 0, 0, 'w' },
			{ "pcu-direct", 0, 0, 'M' },
			{ "realtime", 1, 0, 'r' },
			{ 0, 0, 0, 0 }
		};

		c = getopt_long(argc, argv, "hc:d:Dc:sTVe:p:w:Mr:",
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
		case 'M':
			pcu_direct = 1;
			break;
		case 'V':
			print_version(1);
			exit(0);
			break;
		case 'e':
			log_set_log_level(osmo_stderr_target, atoi(optarg));
			break;
		case 'p':
			dsp_trace = strtoul(optarg, NULL, 16);
			break;
		case 'w':
			print_hwversion();
			exit(0);
			break;
		case 'r':
			rt_prio = atoi(optarg);
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
		bts_shutdown(bts, "SIGINT");
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
	struct stat st;
	struct sched_param param;
	struct gsm_bts_role_bts *btsb;
	struct e1inp_line *line;
	void *tall_msgb_ctx;
	int rc;

	tall_bts_ctx = talloc_named_const(NULL, 1, "OsmoBTS context");
	tall_msgb_ctx = talloc_named_const(tall_bts_ctx, 1, "msgb");
	msgb_set_talloc_ctx(tall_msgb_ctx);

	bts_log_init(NULL);

	vty_init(&bts_vty_info);
	bts_vty_init(&bts_log_info);

	handle_options(argc, argv);

	/* enable realtime priority for us */
	if (rt_prio != -1) {
		memset(&param, 0, sizeof(param));
		param.sched_priority = rt_prio;
		rc = sched_setscheduler(getpid(), SCHED_RR, &param);
		if (rc != 0) {
			fprintf(stderr, "Setting SCHED_RR priority(%d) failed: %s\n",
				param.sched_priority, strerror(errno));
			exit(1);
		}
	}

	bts = gsm_bts_alloc(tall_bts_ctx);
	if (bts_init(bts) < 0) {
		fprintf(stderr, "unable to to open bts\n");
		exit(1);
	}
	btsb = bts_role_bts(bts);
	btsb->support.ciphers = CIPHER_A5(1) | CIPHER_A5(2) | CIPHER_A5(3);

	rc = vty_read_config_file(config_file, NULL);
	if (rc < 0) {
		fprintf(stderr, "Failed to parse the config file: '%s'\n",
			config_file);
		exit(1);
	}

	clk_cal_use_eeprom(bts);

	if (stat(SYSMOBTS_RF_LOCK_PATH, &st) == 0) {
		LOGP(DL1C, LOGL_NOTICE, "Not starting BTS due to RF_LOCK file present\n");
		exit(23);
	}
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

	while (1) {
		log_reset_context();
		osmo_select_main(0);
	}
}
