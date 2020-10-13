/*
 * (C) 2013,2014 by Holger Hans Peter Freyther
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
 */

#include <osmocom/core/talloc.h>
#include <osmocom/core/application.h>

#include <osmo-bts/bts.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/l1sap.h>
#include <osmo-bts/power_control.h>

#include <stdio.h>

static inline void apply_power_test(struct gsm_lchan *lchan, int rxlev, int exp_ret, uint8_t exp_current)
{
	int ret = lchan_ms_pwr_ctrl(lchan, lchan->ms_power_ctrl.current, rxlev);

	printf("power control [%d]: MS current power %u\n", ret, lchan->ms_power_ctrl.current);
	OSMO_ASSERT(ret == exp_ret);
	OSMO_ASSERT(lchan->ms_power_ctrl.current == exp_current);
}

static void test_power_loop(void)
{
	struct gsm_bts bts;
	struct gsm_bts_trx trx;
	struct gsm_bts_trx_ts ts;
	struct gsm_lchan *lchan;

	memset(&bts, 0, sizeof(bts));
	memset(&trx, 0, sizeof(trx));
	memset(&ts, 0, sizeof(ts));

	lchan = &ts.lchan[0];
	lchan->ts = &ts;
	ts.trx = &trx;
	trx.bts = &bts;
	bts.band = GSM_BAND_1800;
	trx.ms_pwr_ctl_soft = true;
	bts.ul_power_target = -75;

	lchan->state = LCHAN_S_NONE;
	lchan->ms_power_ctrl.current = ms_pwr_ctl_lvl(GSM_BAND_1800, 0);
	OSMO_ASSERT(lchan->ms_power_ctrl.current == 15);
	lchan->ms_power_ctrl.max = ms_pwr_ctl_lvl(GSM_BAND_1800, 26);
	OSMO_ASSERT(lchan->ms_power_ctrl.max == 2);

	/* Simply clamping */
	apply_power_test(lchan, -60, 0, 15);

	/*
	 * Now 15 dB too little and we should power it up. Could be a
	 * power level of 7 or 8 for 15 dBm. However, since we limit peace at
	 * which we change values, expect several steps of MS_RAISE_MAX_DB/2 levels:
	 */
	apply_power_test(lchan, -90, 1, 13);
	apply_power_test(lchan, -90, 1, 11);
	apply_power_test(lchan, -90, 1, 9);
	apply_power_test(lchan, -90, 1, 7);
	apply_power_test(lchan, -90, 1, 5);

	/* Check good RSSI value keeps it at same power level: */
	apply_power_test(lchan, bts.ul_power_target, 0, 5);

	apply_power_test(lchan, -90, 1, 3);
	apply_power_test(lchan, -90, 1, 2); /* .max is pwr lvl 2 */
	apply_power_test(lchan, -90, 0, 2); /* .max is pwr lvl 2 */

	lchan->ms_power_ctrl.max = ms_pwr_ctl_lvl(GSM_BAND_1800, 30);
	OSMO_ASSERT(lchan->ms_power_ctrl.max == 0);
	apply_power_test(lchan, -90, 1, 0); /* .max is pwr lvl 0 */
	apply_power_test(lchan, -90, 0, 0); /* .max is pwr lvl 0 */

	lchan->ms_power_ctrl.max = ms_pwr_ctl_lvl(GSM_BAND_1800, 36);
	OSMO_ASSERT(lchan->ms_power_ctrl.max == 29);
	apply_power_test(lchan, -90, 1, 30);
	apply_power_test(lchan, -90, 1, 29);
	apply_power_test(lchan, -90, 0, 29);

	/* Check good RSSI value keeps it at same power level: */
	apply_power_test(lchan, bts.ul_power_target, 0, 29);

	/* Now go down, steps are double size in this direction: */
	apply_power_test(lchan, -45, 1, 1);
	apply_power_test(lchan, -45, 1, 5);
	apply_power_test(lchan, -45, 1, 9);

	/* Go down only one level down and up: */
	apply_power_test(lchan, bts.ul_power_target + 2, 1, 10);
	apply_power_test(lchan, bts.ul_power_target - 2, 1, 9);

	/* Check if BSC requesting a low max power is applied after loop calculation: */
	lchan->ms_power_ctrl.max = ms_pwr_ctl_lvl(GSM_BAND_1800, 2);
	OSMO_ASSERT(lchan->ms_power_ctrl.max == 14);
	apply_power_test(lchan, bts.ul_power_target + 2, 1, 14);
	/* Set back a more normal max: */
	lchan->ms_power_ctrl.max = ms_pwr_ctl_lvl(GSM_BAND_1800, 30);
	OSMO_ASSERT(lchan->ms_power_ctrl.max == 0);

	/* Fix it and jump down */
	lchan->ms_power_ctrl.fixed = true;
	apply_power_test(lchan, -60, 0, 14);

	/* And leave it again */
	lchan->ms_power_ctrl.fixed = false;
	apply_power_test(lchan, -40, 1, 15);
}

int main(int argc, char **argv)
{
	printf("Testing power loop...\n");

	tall_bts_ctx = talloc_named_const(NULL, 1, "OsmoBTS context");
	msgb_talloc_ctx_init(tall_bts_ctx, 0);

	osmo_init_logging2(tall_bts_ctx, &bts_log_info);
	osmo_stderr_target->categories[DLOOP].loglevel = LOGL_DEBUG;
	osmo_stderr_target->categories[DL1C].loglevel = LOGL_DEBUG;
	log_set_print_filename(osmo_stderr_target, 0);
	log_set_use_color(osmo_stderr_target, 0);

	test_power_loop();

	printf("Power loop test OK\n");

	return 0;
}
