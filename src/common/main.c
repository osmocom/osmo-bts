/* Main program for Osmocom BTS */

/* (C) 2011-2016 by Harald Welte <laforge@gnumonks.org>
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
 * GNU Affero General Public License for more details.
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

#include <osmocom/core/stats.h>
#include <osmocom/core/talloc.h>
#include <osmocom/core/application.h>
#include <osmocom/vty/telnet_interface.h>
#include <osmocom/vty/logging.h>
#include <osmocom/vty/stats.h>
#include <osmocom/vty/misc.h>
#include <osmocom/vty/cpu_sched_vty.h>
#include <osmocom/core/gsmtap_util.h>
#include <osmocom/core/gsmtap.h>

#include <osmo-bts/gsm_data.h>
#include <osmo-bts/phy_link.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/abis.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/bts_sm.h>
#include <osmo-bts/vty.h>
#include <osmo-bts/l1sap.h>
#include <osmo-bts/bts_model.h>
#include <osmo-bts/pcu_if.h>
#include <osmo-bts/control_if.h>
#include <osmocom/ctrl/control_if.h>
#include <osmocom/ctrl/ports.h>
#include <osmocom/ctrl/control_vty.h>
#include <osmo-bts/oml.h>

static int quit = 0;
static const char *config_file = "osmo-bts.cfg";
static int daemonize = 0;
static int rt_prio = -1;
static char *gsmtap_ip = 0;
extern int g_vty_port_num;
static bool vty_test_mode = false;

static void print_help()
{
	printf( "Some useful options:\n"
		"  -h	--help			this text\n"
		"  -d	--debug MASK		Enable debugging (e.g. -d DRSL:DOML:DLAPDM)\n"
		"  -D	--daemonize		For the process into a background daemon\n"
		"  -c	--config-file 		Specify the filename of the config file\n"
		"  -s	--disable-color		Don't use colors in stderr log output\n"
		"  -T	--timestamp		Prefix every log line with a timestamp\n"
		"  -V	--version		Print version information and exit\n"
		"  -e 	--log-level		Set a global log-level\n"
		"\nVTY reference generation:\n"
		"	--vty-ref-mode MODE	VTY reference generation mode (e.g. 'expert').\n"
		"	--vty-ref-xml		Generate the VTY reference XML output and exit.\n"
		"\nRegression testing:\n"
		"       --vty-test		VTY test mode. Do not connect to BSC, do not exit.\n"
		);
	bts_model_print_help();
}

static void handle_long_options(const char *prog_name, const int long_option)
{
	static int vty_ref_mode = VTY_REF_GEN_MODE_DEFAULT;

	switch (long_option) {
	case 1:
		vty_ref_mode = get_string_value(vty_ref_gen_mode_names, optarg);
		if (vty_ref_mode < 0) {
			fprintf(stderr, "%s: Unknown VTY reference generation "
				"mode '%s'\n", prog_name, optarg);
			exit(2);
		}
		break;
	case 2:
		fprintf(stderr, "Generating the VTY reference in mode '%s' (%s)\n",
			get_value_string(vty_ref_gen_mode_names, vty_ref_mode),
			get_value_string(vty_ref_gen_mode_desc, vty_ref_mode));
		vty_dump_xml_ref_mode(stdout, (enum vty_ref_gen_mode) vty_ref_mode);
		exit(0);
	case 3:
		vty_test_mode = true;
		break;
	default:
		fprintf(stderr, "%s: error parsing cmdline options\n", prog_name);
		exit(2);
	}
}

/* FIXME: finally get some option parsing code into libosmocore */
static void handle_options(int argc, char **argv)
{
	char *argv_out[argc];
	int argc_out = 0;

	argv_out[argc_out++] = argv[0];

	/* disable generation of error messages on encountering unknown
	 * options */
	opterr = 0;

	while (1) {
		int option_idx = 0, c;
		static int long_option = 0;
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
			/* FIXME: generic BTS app options */
			{ "gsmtap-ip", 1, 0, 'i' },
			{ "trx-num", 1, 0, 't' },
			{ "realtime", 1, 0, 'r' },
			{ "vty-ref-mode", 1, &long_option, 1 },
			{ "vty-ref-xml", 0, &long_option, 2 },
			{ "vty-test", 0, &long_option, 3 },
			{ 0, 0, 0, 0 }
		};

		c = getopt_long(argc, argv, "-hc:d:Dc:sTVe:i:t:r:",
				long_options, &option_idx);
		if (c == -1)
			break;

		switch (c) {
		case 'h':
			print_help();
			exit(0);
			break;
		case 0:
			handle_long_options(argv[0], long_option);
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
			config_file = optarg;
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
		case 'r':
			rt_prio = atoi(optarg);
			fprintf(stderr, "Command line argument '-r' is deprecated, use VTY "
				"cpu-sched node setting 'policy rr %d' instead.\n", rt_prio);
			break;
		case 'i':
			gsmtap_ip = optarg;
			fprintf(stderr, "Command line argument '-i' is deprecated, use VTY "
				"parameter 'gsmtap-remote-host %s' instead.\n", gsmtap_ip);
			break;
		case 't':
			fprintf(stderr, "Command line argument '-t' is deprecated and does nothing, "
				"TRX number is calculated from the VTY automatically.\n");
			break;
		case '?':
		case 1:
			/* prepare argv[] for bts_model */
			argv_out[argc_out++] = argv[optind-1];
			break;
		default:
			break;
		}
	}

	/* re-set opt-ind for new parsig round */
	optind = 1;
	/* enable error-checking for the following getopt call */
	opterr = 1;
	if (bts_model_handle_options(argc_out, argv_out)) {
		print_help();
		exit(1);
	}
}

/* FIXME: remove this once we add multi-BTS support */
struct gsm_bts *g_bts = NULL;

static void signal_handler(int signum)
{
	fprintf(stderr, "signal %u received\n", signum);

	switch (signum) {
	case SIGINT:
	case SIGTERM:
		if (!quit) {
			oml_tx_failure_event_rep(&g_bts->mo,
						 NM_SEVER_CRITICAL, OSMO_EVT_CRIT_PROC_STOP,
						 "BTS: SIGINT received -> shutdown");
			bts_shutdown_ext(g_bts, "SIGINT", true, false);
		}
		quit++;
		break;
	case SIGABRT:
		/* in case of abort, we want to obtain a talloc report and
		 * then run default SIGABRT handler, who will generate coredump
		 * and abort the process. abort() should do this for us after we
		 * return, but program wouldn't exit if an external SIGABRT is
		 * received.
		 */
		talloc_report_full(tall_bts_ctx, stderr);
		signal(SIGABRT, SIG_DFL);
		raise(SIGABRT);
		break;
	case SIGUSR1:
	case SIGUSR2:
		talloc_report_full(tall_bts_ctx, stderr);
		break;
	default:
		break;
	}
}

int bts_main(int argc, char **argv)
{
	struct gsm_bts_trx *trx;
	int rc;

	/* Track the use of talloc NULL memory contexts */
	talloc_enable_null_tracking();

	tall_bts_ctx = talloc_named_const(NULL, 1, "OsmoBTS context");
	msgb_talloc_ctx_init(tall_bts_ctx, 100*1024);
	bts_vty_info.tall_ctx = tall_bts_ctx;

	osmo_init_logging2(tall_bts_ctx, &bts_log_info);
	osmo_stats_init(tall_bts_ctx);
	vty_init(&bts_vty_info);
	ctrl_vty_init(tall_bts_ctx);
	osmo_cpu_sched_vty_init(tall_bts_ctx);
	rate_ctr_init(tall_bts_ctx);

	logging_vty_add_cmds();
	osmo_talloc_vty_add_cmds();
	osmo_stats_vty_add_cmds();
	osmo_fsm_vty_add_cmds();

	bts_vty_init(tall_bts_ctx);
	e1inp_vty_init();

	logging_vty_add_deprecated_subsys(tall_bts_ctx, "sum");

	handle_options(argc, argv);

	fprintf(stderr, "((*))\n  |\n / \\ OsmoBTS\n");
	if (vty_test_mode)
		fprintf(stderr, "--- VTY test mode: not connecting to BSC, not exiting ---\n");

	g_bts_sm = gsm_bts_sm_alloc(tall_bts_ctx);
	if (!g_bts_sm) {
		fprintf(stderr, "Failed to create BTS Site Manager structure\n");
		exit(1);
	}

	g_bts = gsm_bts_alloc(g_bts_sm, 0);
	if (!g_bts) {
		fprintf(stderr, "Failed to create BTS structure\n");
		exit(1);
	}

	/* enable realtime priority for us */
	if (rt_prio != -1) {
		struct sched_param param;

		memset(&param, 0, sizeof(param));
		param.sched_priority = rt_prio;
		rc = sched_setscheduler(getpid(), SCHED_RR, &param);
		if (rc != 0) {
			fprintf(stderr, "Setting SCHED_RR priority(%d) failed: %s\n",
				param.sched_priority, strerror(errno));
			exit(1);
		}
	}

	if (bts_init(g_bts) < 0) {
		fprintf(stderr, "unable to open bts\n");
		exit(1);
	}

	abis_init(g_bts);

	rc = vty_read_config_file(config_file, NULL);
	if (rc < 0) {
		fprintf(stderr, "Failed to parse the config file: '%s'\n",
			config_file);
		exit(1);
	}

	if (!phy_link_by_num(0)) {
		fprintf(stderr, "You need to configure at least phy0\n");
		exit(1);
	}

	llist_for_each_entry(trx, &g_bts->trx_list, list) {
		if (!trx->pinst) {
			fprintf(stderr, "TRX %u has no associated PHY instance\n",
				trx->nr);
			exit(1);
		}
	}

	/* Accept a GSMTAP host from VTY config, but a commandline option overrides that. */
	if (gsmtap_ip != NULL) {
		if (g_bts->gsmtap.remote_host != NULL) {
			LOGP(DLGLOBAL, LOGL_NOTICE,
			     "Command line argument '-i %s' overrides "
			     "'gsmtap-remote-host %s' from the config file\n",
			     gsmtap_ip, g_bts->gsmtap.remote_host);
			talloc_free(g_bts->gsmtap.remote_host);
		}
		g_bts->gsmtap.remote_host = talloc_strdup(g_bts, gsmtap_ip);
	}

	/* TODO: move this to gsm_bts_alloc() */
	if (g_bts->gsmtap.remote_host != NULL) {
		LOGP(DLGLOBAL, LOGL_NOTICE,
		     "Setting up GSMTAP Um forwarding '%s'->'%s:%u'\n",
		     g_bts->gsmtap.local_host, g_bts->gsmtap.remote_host, GSMTAP_UDP_PORT);
		g_bts->gsmtap.inst = gsmtap_source_init2(g_bts->gsmtap.local_host, 0,
							 g_bts->gsmtap.remote_host, GSMTAP_UDP_PORT, 1);
		if (g_bts->gsmtap.inst == NULL) {
			fprintf(stderr, "Failed during gsmtap_source_init2()\n");
			exit(1);
		}
		gsmtap_source_add_sink(g_bts->gsmtap.inst);
	}

	bts_controlif_setup(g_bts, OSMO_CTRL_PORT_BTS);

	rc = telnet_init_default(tall_bts_ctx, NULL, g_vty_port_num);
	if (rc < 0) {
		fprintf(stderr, "Error initializing telnet\n");
		exit(1);
	}

	if (pcu_sock_init(g_bts->pcu.sock_path, g_bts->pcu.sock_wqueue_len_max)) {
		fprintf(stderr, "PCU L1 socket failed\n");
		exit(1);
	}

	signal(SIGINT, &signal_handler);
	signal(SIGTERM, &signal_handler);
	signal(SIGABRT, &signal_handler);
	signal(SIGUSR1, &signal_handler);
	signal(SIGUSR2, &signal_handler);
	osmo_init_ignore_signals();

	if (bts_osmux_open(g_bts) < 0) {
		fprintf(stderr, "Osmux setup failed\n");
		exit(1);
	}

	if (vty_test_mode) {
		/* Just select-loop without connecting to the BSC, don't exit. This allows running tests on the VTY
		 * telnet port. */
		while (!quit) {
			log_reset_context();
			osmo_select_main(0);
		}
		return EXIT_SUCCESS;
	}

	if (abis_open(g_bts, "osmo-bts") != 0)
		exit(1);

	rc = phy_links_open();
	if (rc < 0) {
		fprintf(stderr, "unable to open PHY link(s)\n");
		exit(2);
	}

	if (daemonize) {
		rc = osmo_daemonize();
		if (rc < 0) {
			perror("Error during daemonize");
			exit(1);
		}
	}

	while (quit < 2) {
		log_reset_context();
		osmo_select_main(0);
	}

	return EXIT_SUCCESS;
}
