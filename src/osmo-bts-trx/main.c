/* Main program for OsmoBTS-TRX */

/* (C) 2011-2015 by Harald Welte <laforge@gnumonks.org>
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

#include <osmocom/core/talloc.h>
#include <osmocom/core/application.h>
#include <osmocom/vty/telnet_interface.h>
#include <osmocom/vty/logging.h>
#include <osmocom/vty/ports.h>
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
#include <osmo-bts/control_if.h>
#include <osmo-bts/scheduler.h>

#include "l1_if.h"
#include "trx_if.h"

int bts_model_init(struct gsm_bts *bts)
{
	void *l1h;
	struct gsm_bts_trx *trx;
	struct gsm_bts_role_bts *btsb = bts_role_bts(bts);

	btsb->support.ciphers = CIPHER_A5(1) | CIPHER_A5(2);
	if (!settsc_enabled && !setbsic_enabled)
		settsc_enabled = setbsic_enabled = 1;

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

void bts_model_print_help()
{
	printf(
		"  -I	--local-trx-ip	Local IP for transceiver to connect (default=%s)\n"
		, transceiver_ip
		);
}

int bts_model_handle_options(int argc, char **argv)
{
	int num_errors = 0;

	while (1) {
		int option_idx = 0, c;
		static const struct option long_options[] = {
			/* specific to this hardware */
			{ "local-trx-ip", 1, 0, 'I' },
			{ 0, 0, 0, 0 }
		};

		c = getopt_long(argc, argv, "I:",
				long_options, &option_idx);

		if (c == -1)
			break;

		switch (c) {
		case 'I':
			transceiver_ip = strdup(optarg);
			break;
		default:
			num_errors++;
			break;
		}
	}

	return num_errors;
}

int main(int argc, char **argv)
{
	return bts_main(argc, argv);
}
