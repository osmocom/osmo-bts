/* Main program for NuRAN Wireless Litecell 1.5 BTS */

/* Copyright (C) 2015 by Yves Godin <support@nuranwireless.com>
 * Copyright (C) 2016 by Harald Welte <laforge@gnumonks.org>
 *
 * Based on sysmoBTS:
 *     (C) 2011-2013 by Harald Welte <laforge@gnumonks.org>
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
#include <osmocom/vty/ports.h>

#include <osmo-bts/gsm_data.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/vty.h>
#include <osmo-bts/bts_model.h>
#include <osmo-bts/pcu_if.h>
#include <osmo-bts/l1sap.h>

static int write_status_file(char *status_file, char *status_str)
{
	FILE *outf;
	char tmp[PATH_MAX+1];

	snprintf(tmp, sizeof(tmp)-1, "/var/run/osmo-bts/%s", status_file);
	tmp[PATH_MAX-1] = '\0';

	outf = fopen(tmp, "w");
	if (!outf)
		return -1;

	fprintf(outf, "%s\n", status_str);

	fclose(outf);

	return 0;
}

/*NTQD: Change how rx_nr is handle in multi-trx*/
#define LC15BTS_RF_LOCK_PATH	"/var/lock/bts_rf_lock"

#include "utils.h"
#include "l1_if.h"
#include "hw_misc.h"
#include "misc/lc15bts_bid.h"

unsigned int dsp_trace = 0x00000000;

int bts_model_init(struct gsm_bts *bts)
{
	struct stat st;

	struct bts_lc15_priv *bts_lc15 = talloc(bts, struct bts_lc15_priv);

	bts->model_priv = bts_lc15;
	bts->variant = BTS_OSMO_LITECELL15;
	bts->support.ciphers = CIPHER_A5(1) | CIPHER_A5(2) | CIPHER_A5(3);

	/* specific default values for LC15 platform */
	bts_lc15->led_ctrl_mode = LC15_BTS_LED_CTRL_MODE_DEFAULT;
	/* RTP drift threshold default */
	bts_lc15->rtp_drift_thres_ms = LC15_BTS_RTP_DRIFT_THRES_DEFAULT;

	if (stat(LC15BTS_RF_LOCK_PATH, &st) == 0) {
		LOGP(DL1C, LOGL_NOTICE, "Not starting BTS due to RF_LOCK file present\n");
		exit(23);
	}

	/* order alphabetically */
	osmo_bts_set_feature(bts->features, BTS_FEAT_AGCH_PCH_PROP);
	osmo_bts_set_feature(bts->features, BTS_FEAT_EGPRS);
	osmo_bts_set_feature(bts->features, BTS_FEAT_GPRS);
	osmo_bts_set_feature(bts->features, BTS_FEAT_OML_ALERTS);
	osmo_bts_set_feature(bts->features, BTS_FEAT_PAGING_COORDINATION);
	osmo_bts_set_feature(bts->features, BTS_FEAT_SPEECH_F_AMR);
	osmo_bts_set_feature(bts->features, BTS_FEAT_SPEECH_F_EFR);
	osmo_bts_set_feature(bts->features, BTS_FEAT_SPEECH_F_V1);
	osmo_bts_set_feature(bts->features, BTS_FEAT_SPEECH_H_AMR);
	osmo_bts_set_feature(bts->features, BTS_FEAT_SPEECH_H_V1);

	bts_internal_flag_set(bts, BTS_INTERNAL_FLAG_MS_PWR_CTRL_DSP);
	bts_internal_flag_set(bts, BTS_INTERNAL_FLAG_NM_RCHANNEL_DEPENDS_RCARRIER);

	return 0;
}

int bts_model_trx_init(struct gsm_bts_trx *trx)
{
	trx->nominal_power = 40;
	trx->power_params.trx_p_max_out_mdBm = to_mdB(trx->bts->c0->nominal_power);
	return 0;
}

void bts_model_phy_link_set_defaults(struct phy_link *plink)
{
}

void bts_model_phy_instance_set_defaults(struct phy_instance *pinst)
{
}

int bts_model_oml_estab(struct gsm_bts *bts)
{
	/* update status file */
	write_status_file("state", "");

	return 0;
}

void bts_update_status(enum bts_global_status which, int on)
{
	static uint64_t states = 0;
	uint64_t old_states = states;

	if (on)
		states |= (1ULL << which);
	else
		states &= ~(1ULL << which);

	LOGP(DL1C, LOGL_INFO,
	     "Set global status #%d to %d (%04llx -> %04llx)",
	     which, on,
	     (long long)old_states, (long long)states);
}

void bts_model_print_help()
{
	printf( "\nModel specific options:\n"
		"  -w	--hw-version		Print the targeted HW Version\n"
		"  -M	--pcu-direct		Force PCU to access message queue for "
						"PDCH dchannel directly\n"
		"  -p	--dsp-trace		Set DSP trace flags\n"
		);
}

static void print_hwversion()
{
	printf(get_hwversion_desc());
	printf("\n");
}

int bts_model_handle_options(int argc, char **argv)
{
	int num_errors = 0;

	while (1) {
		int option_idx = 0, c;
		static const struct option long_options[] = {
			{ "dsp-trace", 1, 0, 'p' },
			{ "hw-version", 0, 0, 'w' },
			{ "pcu-direct", 0, 0, 'M' },
			{ 0, 0, 0, 0 }
		};

		c = getopt_long(argc, argv, "p:wM",
				long_options, &option_idx);
		if (c == -1)
			break;

		switch (c) {
		case 'p':
			dsp_trace = strtoul(optarg, NULL, 16);
			break;
		case 'M':
			pcu_direct = 1;
			break;
		case 'w':
			print_hwversion();
			exit(0);
			break;
		default:
			num_errors++;
			break;
		}
	}

	return num_errors;
}

void bts_model_abis_close(struct gsm_bts *bts)
{
	/* write to status file */
	write_status_file("state", "ABIS DOWN");

	/* for now, we simply terminate the program and re-spawn */
	bts_shutdown(bts, "Abis close");
}

int main(int argc, char **argv)
{
	/* create status file with initial state */
	write_status_file("state", "ABIS DOWN");

	return bts_main(argc, argv);
}
