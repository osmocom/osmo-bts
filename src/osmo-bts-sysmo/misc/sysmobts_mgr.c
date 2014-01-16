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
#include <sys/signal.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <osmocom/core/talloc.h>
#include <osmocom/core/application.h>
#include <osmocom/core/timer.h>
#include <osmocom/core/msgb.h>
#include <osmocom/vty/telnet_interface.h>
#include <osmocom/vty/logging.h>

#include "misc/sysmobts_misc.h"
#include "misc/sysmobts_mgr.h"

static int no_eeprom_write = 0;
static int daemonize = 0;
void *tall_mgr_ctx;

/* every 6 hours means 365*4 = 1460 EEprom writes per year (max) */
#define TEMP_TIMER_SECS		(6 * 3600)

/* every 1 hours means 365*24 = 8760 EEprom writes per year (max) */
#define HOURS_TIMER_SECS	(1 * 3600)

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

int main(int argc, char **argv)
{
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

	if (daemonize) {
		rc = osmo_daemonize();
		if (rc < 0) {
			perror("Error during daemonize");
			exit(1);
		}
	}

	/* start temperature check timer */
	temp_timer.cb = check_temp_timer_cb;
	check_temp_timer_cb(NULL);

	/* start operational hours timer */
	hours_timer.cb = hours_timer_cb;
	hours_timer_cb(NULL);

	while (1) {
		log_reset_context();
		osmo_select_main(0);
	}
}
