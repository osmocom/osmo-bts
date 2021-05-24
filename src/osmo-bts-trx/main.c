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
#include <osmocom/core/rate_ctr.h>
#include <osmocom/core/stats.h>

#include <osmo-bts/gsm_data.h>
#include <osmo-bts/phy_link.h>
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

static const struct rate_ctr_desc btstrx_ctr_desc[] = {
	[BTSTRX_CTR_SCHED_DL_MISS_FN] = {
		"trx_clk:sched_dl_miss_fn",
		"Downlink frames scheduled later than expected due to missed timerfd event (due to high system load)"
	},
	[BTSTRX_CTR_SCHED_DL_FH_NO_CARRIER] = {
		"trx_sched:dl_fh_no_carrier",
		"Frequency hopping: no carrier found for a Downlink burst (check hopping parameters)"
	},
	[BTSTRX_CTR_SCHED_UL_FH_NO_CARRIER] = {
		"trx_sched:ul_fh_no_carrier",
		"Frequency hopping: no carrier found for an Uplink burst (check hopping parameters)"
	},
};
static const struct rate_ctr_group_desc btstrx_ctrg_desc = {
	"bts-trx",
	"osmo-bts-trx specific counters",
	OSMO_STATS_CLASS_GLOBAL,
	ARRAY_SIZE(btstrx_ctr_desc),
	btstrx_ctr_desc
};

/* dummy, since no direct dsp support */
uint32_t trx_get_hlayer1(const struct gsm_bts_trx *trx)
{
	return 0;
}

void bts_model_print_help()
{
}

int bts_model_handle_options(int argc, char **argv)
{
	int num_errors = 0;

	while (1) {
		int option_idx = 0, c;
		static const struct option long_options[] = {
			{ 0, 0, 0, 0 }
		};

		c = getopt_long(argc, argv, "",
				long_options, &option_idx);

		if (c == -1)
			break;

		switch (c) {
		default:
			num_errors++;
			break;
		}
	}

	return num_errors;
}

int bts_model_init(struct gsm_bts *bts)
{
	struct bts_trx_priv *bts_trx = talloc_zero(bts, struct bts_trx_priv);
	bts_trx->clk_s.fn_timer_ofd.fd = -1;
	bts_trx->ctrs = rate_ctr_group_alloc(bts_trx, &btstrx_ctrg_desc, 0);

	bts->model_priv = bts_trx;
	bts->variant = BTS_OSMO_TRX;
	bts->support.ciphers = CIPHER_A5(1) | CIPHER_A5(2) | CIPHER_A5(3);

	/* The nominal value for each TRX is later overwritten through VTY cmd
	 * 'nominal-tx-power' if present, otherwise through TRXC cmd NOMTXPOWER.
	 */
	bts->c0->nominal_power = 23;

	osmo_bts_set_feature(bts->features, BTS_FEAT_GPRS);
	osmo_bts_set_feature(bts->features, BTS_FEAT_EGPRS);
	osmo_bts_set_feature(bts->features, BTS_FEAT_OML_ALERTS);
	osmo_bts_set_feature(bts->features, BTS_FEAT_SPEECH_F_V1);
	osmo_bts_set_feature(bts->features, BTS_FEAT_SPEECH_H_V1);
	osmo_bts_set_feature(bts->features, BTS_FEAT_SPEECH_F_EFR);
	osmo_bts_set_feature(bts->features, BTS_FEAT_SPEECH_F_AMR);
	osmo_bts_set_feature(bts->features, BTS_FEAT_SPEECH_H_AMR);
	osmo_bts_set_feature(bts->features, BTS_FEAT_CBCH);
	osmo_bts_set_feature(bts->features, BTS_FEAT_HOPPING);
	osmo_bts_set_feature(bts->features, BTS_FEAT_ACCH_REP);
	osmo_bts_set_feature(bts->features, BTS_FEAT_MULTI_TSC);

	bts_internal_flag_set(bts, BTS_INTERNAL_FLAG_MEAS_PAYLOAD_COMB);

	return 0;
}

int bts_model_trx_init(struct gsm_bts_trx *trx)
{
	/* The nominal value for each TRX is later overwritten through VTY cmd
	 * 'nominal-tx-power' if present, otherwise through TRXC cmd NOMTXPOWER.
	 */
	l1if_trx_set_nominal_power(trx, trx->bts->c0->nominal_power);
	return 0;
}

void bts_model_phy_link_set_defaults(struct phy_link *plink)
{
	plink->u.osmotrx.local_ip = talloc_strdup(plink, "127.0.0.1");
	plink->u.osmotrx.remote_ip = talloc_strdup(plink, "127.0.0.1");
	plink->u.osmotrx.base_port_local = 5800;
	plink->u.osmotrx.base_port_remote = 5700;
	plink->u.osmotrx.clock_advance = 2;
	plink->u.osmotrx.rts_advance = 3;
	/* attempt use newest TRXD version by default: */
	plink->u.osmotrx.trxd_pdu_ver_max = TRX_DATA_PDU_VER;
}

void bts_model_phy_instance_set_defaults(struct phy_instance *pinst)
{
	struct trx_l1h *l1h;
	l1h = trx_l1h_alloc(tall_bts_ctx, pinst);
	pinst->u.osmotrx.hdl = l1h;

	l1h->config.forced_max_power_red = -1;
}

int main(int argc, char **argv)
{
	return bts_main(argc, argv);
}
