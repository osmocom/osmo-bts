/*
 * (C) 2013,2014 by Holger Hans Peter Freyther
 * Contributions by sysmocom - s.f.m.c. GmbH <info@sysmocom.de>
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
 */

#include <osmocom/core/talloc.h>
#include <osmocom/core/application.h>

#include <osmo-bts/bts.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/l1sap.h>
#include <osmo-bts/power_control.h>

#include <stdio.h>

#define PWR_TEST_RXLEV_TARGET_DBM	-75
#define PWR_TEST_RXLEV_TARGET \
	dbm2rxlev(PWR_TEST_RXLEV_TARGET_DBM)

static struct gsm_bts *g_bts = NULL;
static struct gsm_bts_trx *g_trx = NULL;

static void init_test(const char *name)
{
	if (g_trx != NULL)
		talloc_free(g_trx);
	if (g_bts != NULL)
		talloc_free(g_bts);

	g_bts = talloc_zero(tall_bts_ctx, struct gsm_bts);
	OSMO_ASSERT(g_bts != NULL);

	INIT_LLIST_HEAD(&g_bts->trx_list);
	g_trx = gsm_bts_trx_alloc(g_bts);
	OSMO_ASSERT(g_trx != NULL);

	g_trx->ms_pwr_ctl_soft = true;

	g_bts->band = GSM_BAND_1800;
	g_bts->c0 = g_trx;

	/* Init default MS power control parameters, enable dynamic power control */
	struct gsm_power_ctrl_params *params = &g_trx->ts[0].lchan[0].ms_power_ctrl.dpc_params;
	*params = power_ctrl_params_def;
	g_trx->ts[0].lchan[0].ms_power_ctrl.dpc_enabled = true;

	/* Disable loop SACCH block skip by default: */
	params->ctrl_interval = 0;

	/* Disable RxLev pre-processing and hysteresis by default */
	struct gsm_power_ctrl_meas_params *mp = &params->rxlev_meas;
	mp->lower_thresh = mp->upper_thresh = PWR_TEST_RXLEV_TARGET;
	mp->algo = GSM_PWR_CTRL_MEAS_AVG_ALGO_NONE;

	printf("\nStarting test case '%s'\n", name);
}

static void apply_power_test_ext(struct gsm_lchan *lchan, uint8_t ms_pwr, int rxlev, int lqual_cb, int exp_ret, uint8_t exp_current)
{
	int ret;

	ret = lchan_ms_pwr_ctrl(lchan, ms_pwr, rxlev, lqual_cb);

	/* Keep the measurement counter updated */
	lchan->meas.res_nr++;

	printf("lchan_ms_pwr_ctrl(RxLvl=%d dBm) returns %d (expected %d)\n",
	       rxlev, ret, exp_ret);
	printf("\tMS current power %u -> %u (expected %u)\n",
	       ms_pwr, lchan->ms_power_ctrl.current, exp_current);
}

static inline void apply_power_test(struct gsm_lchan *lchan, int rxlev, int lqual_cb, int exp_ret, uint8_t exp_current)
{
	apply_power_test_ext(lchan, lchan->ms_power_ctrl.current, rxlev, lqual_cb, exp_ret, exp_current);
}

static void test_power_loop(void)
{
	struct gsm_lchan *lchan;
	const struct gsm_power_ctrl_params *params;
	int16_t good_lqual;

	init_test(__func__);
	lchan = &g_trx->ts[0].lchan[0];
	params = &lchan->ms_power_ctrl.dpc_params;
	lchan->type = GSM_LCHAN_SDCCH;
	good_lqual = (params->ci_sdcch_meas.lower_thresh + 2) * 10;

	lchan->ms_power_ctrl.current = ms_pwr_ctl_lvl(GSM_BAND_1800, 0);
	OSMO_ASSERT(lchan->ms_power_ctrl.current == 15);
	lchan->ms_power_ctrl.max = ms_pwr_ctl_lvl(GSM_BAND_1800, 26);
	OSMO_ASSERT(lchan->ms_power_ctrl.max == 2);

	/* Simply clamping */
	apply_power_test(lchan, -60, good_lqual, 0, 15);

	/*
	 * Now 15 dB too little and we should power it up. Could be a
	 * power level of 7 or 8 for 15 dBm. However, since we limit peace at
	 * which we change values, expect several steps of MS_RAISE_MAX_DB/2 levels:
	 */
	apply_power_test(lchan, -90, good_lqual, 1, 13);
	apply_power_test(lchan, -90, good_lqual, 1, 11);
	apply_power_test(lchan, -90, good_lqual, 1, 9);
	apply_power_test(lchan, -90, good_lqual, 1, 7);
	apply_power_test(lchan, -90, good_lqual, 1, 5);

	/* Check good RSSI value keeps it at same power level: */
	apply_power_test(lchan, PWR_TEST_RXLEV_TARGET_DBM, good_lqual, 0, 5);

	apply_power_test(lchan, -90, good_lqual, 1, 3);
	apply_power_test(lchan, -90, good_lqual, 1, 2); /* .max is pwr lvl 2 */
	apply_power_test(lchan, -90, good_lqual, 0, 2); /* .max is pwr lvl 2 */

	lchan->ms_power_ctrl.max = ms_pwr_ctl_lvl(GSM_BAND_1800, 30);
	OSMO_ASSERT(lchan->ms_power_ctrl.max == 0);
	apply_power_test(lchan, -90, good_lqual, 1, 0); /* .max is pwr lvl 0 */
	apply_power_test(lchan, -90, good_lqual, 0, 0); /* .max is pwr lvl 0 */

	lchan->ms_power_ctrl.max = ms_pwr_ctl_lvl(GSM_BAND_1800, 36);
	OSMO_ASSERT(lchan->ms_power_ctrl.max == 29);
	apply_power_test(lchan, -90, good_lqual, 1, 30);
	apply_power_test(lchan, -90, good_lqual, 1, 29);
	apply_power_test(lchan, -90, good_lqual, 0, 29);

	/* Check good RSSI value keeps it at same power level: */
	apply_power_test(lchan, PWR_TEST_RXLEV_TARGET_DBM, good_lqual, 0, 29);

	/* Now go down, steps are double size in this direction: */
	apply_power_test(lchan, -45, good_lqual, 1, 1);
	apply_power_test(lchan, -45, good_lqual, 1, 5);
	apply_power_test(lchan, -45, good_lqual, 1, 9);

	/* Go down only one level down and up: */
	apply_power_test(lchan, PWR_TEST_RXLEV_TARGET_DBM + 2, good_lqual, 1, 10);
	apply_power_test(lchan, PWR_TEST_RXLEV_TARGET_DBM - 2, good_lqual, 1, 9);

	/* Check if BSC requesting a low max power is applied after loop calculation: */
	lchan->ms_power_ctrl.max = ms_pwr_ctl_lvl(GSM_BAND_1800, 2);
	OSMO_ASSERT(lchan->ms_power_ctrl.max == 14);
	apply_power_test(lchan, PWR_TEST_RXLEV_TARGET_DBM + 2, good_lqual, 1, 14);
	/* Set back a more normal max: */
	lchan->ms_power_ctrl.max = ms_pwr_ctl_lvl(GSM_BAND_1800, 30);
	OSMO_ASSERT(lchan->ms_power_ctrl.max == 0);

	/* Disable dynamic power control and jump down */
	lchan->ms_power_ctrl.dpc_enabled = false;
	apply_power_test(lchan, -60, good_lqual, 0, 14);

	/* Enable and leave it again */
	lchan->ms_power_ctrl.dpc_enabled = true;
	apply_power_test(lchan, -40, good_lqual, 1, 15);
}

static void test_pf_algo_ewma(void)
{
	struct gsm_lchan *lchan;
	const struct gsm_power_ctrl_params *params;
	int16_t good_lqual;
	const int *avg100;

	init_test(__func__);
	lchan = &g_trx->ts[0].lchan[0];
	lchan->type = GSM_LCHAN_SDCCH;
	params = &lchan->ms_power_ctrl.dpc_params;
	good_lqual = (params->ci_sdcch_meas.lower_thresh + 2) * 10;
	avg100 = &lchan->ms_power_ctrl.rxlev_meas_proc.ewma.Avg100;

	struct gsm_power_ctrl_meas_params *mp = &lchan->ms_power_ctrl.dpc_params.rxlev_meas;
	mp->algo = GSM_PWR_CTRL_MEAS_AVG_ALGO_OSMO_EWMA;
	mp->ewma.alpha = 20; /* 80% smoothing */

	lchan->ms_power_ctrl.current = ms_pwr_ctl_lvl(GSM_BAND_1800, 0);
	OSMO_ASSERT(lchan->ms_power_ctrl.current == 15);
	lchan->ms_power_ctrl.max = ms_pwr_ctl_lvl(GSM_BAND_1800, 26);
	OSMO_ASSERT(lchan->ms_power_ctrl.max == 2);

#define CHECK_RXLEV_AVG100(exp) \
	printf("\tAvg[t] is RxLev %2.2f (expected %2.2f)\n", \
	       ((float) *avg100) / 100, exp);

	/* UL RSSI remains constant => no UL power change */
	apply_power_test(lchan, -75, good_lqual, 0, 15);
	CHECK_RXLEV_AVG100((float)dbm2rxlev(-75)); /* RXLEV 35 */

	/* Avg[t] = (0.2 * 20) + (0.8 * 35) = RXLEV 32, (-78 dBm) */
	apply_power_test(lchan, -90, good_lqual, 1, 13); /* -90 dBm = RXLEV 20 */
	CHECK_RXLEV_AVG100(32.00);

	/* Avg[t] = (0.2 * 20) + (0.8 * 32) = RXLEV 29.6 (-80.4 dBm) */
	apply_power_test(lchan, -90, good_lqual, 1, 11);  /* -90 dBm = RXLEV 20 */
	CHECK_RXLEV_AVG100(29.60);

	/* Avg[t] = (0.2 * 40) + (0.8 * 29.60) = RXLEV 31.68 (-78.32 dBm),
	 * but due to up-/down-scaling artefacts we get the following:
	 *   Avg100[t] = Avg100[t - 1] + A * (Pwr - Avg[t] / 100)
	 *   Avg100[t] = 2960 + 20 * (40 - ((2960+50) / 100)) <- HERE we lose 0.1: (2960+50) / 100) = 30.1
	 *   Avg100[t] = 2960 + 20 * (40 - 30) <- HERE we lose 20*0.1 = 2.0! (upscaled, hence we lose finally 2.0/100=0.2)
	 *   Avg[t] = (3160) / 100 = 31.60*/
	apply_power_test(lchan, -70, good_lqual, 1, 9); /* RXLEV 40 */
	CHECK_RXLEV_AVG100(31.60);

	mp->ewma.alpha = 70; /* 30% smoothing */
	lchan->ms_power_ctrl.current = 15;
	lchan->ms_power_ctrl.rxlev_meas_proc = \
		(struct gsm_power_ctrl_meas_proc_state) { 0 };

	/* This is the first sample, the filter outputs it as-is */
	apply_power_test(lchan, -50, good_lqual, 0, 15); /* RXLEV 60 */
	CHECK_RXLEV_AVG100((float)dbm2rxlev(-50));

	/* Avg[t] = (0.7 * 60) + (0.3 * 60) = RXLEV 60 (-50.0 dBm) */
	apply_power_test(lchan, -50, good_lqual, 0, 15);
	CHECK_RXLEV_AVG100((float)dbm2rxlev(-50));

	/* Simulate SACCH block loss (-110 dBm):
	 * Avg[t] = (0.7 * 0) + (0.3 * 60) = RXLEV 18.0 (-92.0 dBm) */
	apply_power_test(lchan, -110, good_lqual, 1, 13); /* RXLEV 0 */
	CHECK_RXLEV_AVG100(18.0);
}

static void test_power_hysteresis(void)
{
	struct gsm_lchan *lchan;
	struct gsm_power_ctrl_params *params;
	int16_t good_lqual;

	init_test(__func__);
	lchan = &g_trx->ts[0].lchan[0];
	lchan->type = GSM_LCHAN_SDCCH;
	params = &lchan->ms_power_ctrl.dpc_params;
	good_lqual = (params->ci_sdcch_meas.lower_thresh + 2) * 10;

	/* Tolerate power deviations in range -80 .. -70 */
	params->rxlev_meas.lower_thresh = 30;
	params->rxlev_meas.upper_thresh = 40;

	lchan->ms_power_ctrl.current = ms_pwr_ctl_lvl(GSM_BAND_1800, 0);
	OSMO_ASSERT(lchan->ms_power_ctrl.current == 15);
	lchan->ms_power_ctrl.max = ms_pwr_ctl_lvl(GSM_BAND_1800, 26);
	OSMO_ASSERT(lchan->ms_power_ctrl.max == 2);

	apply_power_test(lchan, PWR_TEST_RXLEV_TARGET_DBM, good_lqual, 0, 15);
	apply_power_test(lchan, PWR_TEST_RXLEV_TARGET_DBM + 3, good_lqual, 0, 15);
	apply_power_test(lchan, PWR_TEST_RXLEV_TARGET_DBM - 3, good_lqual, 0, 15);

	apply_power_test(lchan, PWR_TEST_RXLEV_TARGET_DBM, good_lqual, 0, 15);
	apply_power_test(lchan, PWR_TEST_RXLEV_TARGET_DBM + 5, good_lqual, 0, 15);
	apply_power_test(lchan, PWR_TEST_RXLEV_TARGET_DBM - 5, good_lqual, 0, 15);

	apply_power_test(lchan, PWR_TEST_RXLEV_TARGET_DBM - 10, good_lqual, 1, 13);
}

static void test_power_ctrl_interval(void)
{
	struct gsm_lchan *lchan;
	const struct gsm_power_ctrl_params *params;
	int16_t good_lqual;
	unsigned int i, j;

	init_test(__func__);
	lchan = &g_trx->ts[0].lchan[0];
	lchan->type = GSM_LCHAN_SDCCH;
	params = &lchan->ms_power_ctrl.dpc_params;
	good_lqual = (params->ci_sdcch_meas.lower_thresh + 2) * 10;

	lchan->ms_power_ctrl.max = ms_pwr_ctl_lvl(GSM_BAND_1800, 26);
	OSMO_ASSERT(lchan->ms_power_ctrl.max == 2);

	const int script[][8][4] = {
		{ /* P_Con_INTERVAL=0 (480 ms) */
			/* { UL RxLev, expected rc, expected Tx power level } */
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	good_lqual,	1,	13 },
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	good_lqual,	1,	11 },
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	good_lqual,	1,	 9 },
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	good_lqual,	1,	 7 },
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	good_lqual,	1,	 5 },
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	good_lqual,	1,	 3 },
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	good_lqual,	1,	 2 },
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	good_lqual,	1,	 2 },
		},
		{ /* P_Con_INTERVAL=1 (960 ms) */
			/* { UL RxLev, expected rc, expected Tx power level } */
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	good_lqual,	1,	13 },
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	good_lqual,	0,	13 }, /* skipped */
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	good_lqual,	1,	11 },
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	good_lqual,	0,	11 }, /* skipped */
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	good_lqual,	1,	 9 },
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	good_lqual,	0,	 9 }, /* skipped */
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	good_lqual,	1,	 7 },
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	good_lqual,	0,	 7 }, /* skipped */
		},
		{ /* P_Con_INTERVAL=2 (1920 ms) */
			/* { UL RxLev, expected rc, expected Tx power level } */
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	good_lqual,	1,	13 },
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	good_lqual,	0,	13 }, /* skipped */
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	good_lqual,	0,	13 }, /* skipped */
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	good_lqual,	0,	13 }, /* skipped */
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	good_lqual,	1,	11 },
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	good_lqual,	0,	11 }, /* skipped */
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	good_lqual,	0,	11 }, /* skipped */
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	good_lqual,	0,	11 }, /* skipped */
		},
	};

	for (i = 0; i < ARRAY_SIZE(script); i++) {
		lchan->ms_power_ctrl.current = ms_pwr_ctl_lvl(GSM_BAND_1800, 0);
		OSMO_ASSERT(lchan->ms_power_ctrl.current == 15);

		/* Set the corresponding power control interval */
		printf("%s(): power control interval is now %u\n", __func__, i);
		lchan->ms_power_ctrl.dpc_params.ctrl_interval = i;

		for (j = 0; j < ARRAY_SIZE(script[i]); j++) {
			apply_power_test(lchan, script[i][j][0],  /* UL RxLev */
						script[i][j][1],  /* UL C/I */
						script[i][j][2],  /* expected rc */
						script[i][j][3]); /* expected Tx power level */
		}

		printf("\n");
	}
}

static void test_power_loop_ci(void)
{
	struct gsm_lchan *lchan;
	const struct gsm_power_ctrl_params *params;
	int16_t good_lqual, too_low_lqual, too_high_lqual;

	init_test(__func__);
	lchan = &g_trx->ts[0].lchan[0];
	params = &lchan->ms_power_ctrl.dpc_params;
	lchan->type = GSM_LCHAN_SDCCH;
	good_lqual = (params->ci_sdcch_meas.lower_thresh + 2) * 10;
	too_low_lqual = (params->ci_sdcch_meas.lower_thresh - 1) * 10;
	too_high_lqual = (params->ci_sdcch_meas.upper_thresh + 1) * 10;

	lchan->ms_power_ctrl.current = ms_pwr_ctl_lvl(GSM_BAND_1800, 0);
	OSMO_ASSERT(lchan->ms_power_ctrl.current == 15);
	lchan->ms_power_ctrl.max = ms_pwr_ctl_lvl(GSM_BAND_1800, 26);
	OSMO_ASSERT(lchan->ms_power_ctrl.max == 2);

	/* Simply clamping */
	apply_power_test(lchan, -60, good_lqual, 0, 15);

	/* Now UL C/I is too bad as well as RSSI: */
	apply_power_test(lchan, -100, too_low_lqual, 1, 13);
	apply_power_test(lchan, -100, too_low_lqual, 1, 11);

	/* Now UL C/I is good again while RSSI is good: */
	apply_power_test(lchan, -60, good_lqual, 1, 12);
	apply_power_test(lchan, -60, too_high_lqual, 1, 13);

	/* Now UL C/I is good while RSSI is bad, C/I mandates: */
	apply_power_test(lchan, -100, good_lqual, 1, 11);
	apply_power_test(lchan, -100, too_high_lqual, 1, 12);

	/* Now UL C/I is bad again while RSSI is good, C/I mandates: */
	apply_power_test(lchan, -60, good_lqual, 1, 13);
	apply_power_test(lchan, -60, too_high_lqual, 1, 14);
}

/* Test whether ping pong between requested MS Power Level and announced MS
 * Power level occurs, oscillating between considered good levels all the time:
 * FIXME: Current code shows there's an issue with oscillating values. */
static void test_good_threshold_convergence(void)
{
	struct gsm_lchan *lchan;
	struct gsm_power_ctrl_params *params;
	int16_t good_lqual, good_rxlev;

	init_test(__func__);
	lchan = &g_trx->ts[0].lchan[0];
	params = &lchan->ms_power_ctrl.dpc_params;
	params->rxlev_meas.upper_thresh = 37;
	params->rxlev_meas.lower_thresh = 30;
	lchan->type = GSM_LCHAN_SDCCH;
	good_lqual = (params->ci_sdcch_meas.lower_thresh + 2) * 10;
	good_rxlev = rxlev2dbm(params->rxlev_meas.lower_thresh + 2);

	lchan->ms_power_ctrl.current = 10;
	lchan->ms_power_ctrl.max = 2;

	apply_power_test_ext(lchan, 9, good_rxlev, good_lqual, 0, 10);
	apply_power_test_ext(lchan, 10, good_rxlev, good_lqual, 0, 10);
	apply_power_test_ext(lchan, 9, good_rxlev, good_lqual, 0, 10);
	apply_power_test_ext(lchan, 10, good_rxlev, good_lqual, 0, 10);
	apply_power_test_ext(lchan, 9, good_rxlev, good_lqual, 0, 10);
}

int main(int argc, char **argv)
{
	printf("Testing power loop...\n");

	tall_bts_ctx = talloc_named_const(NULL, 1, "OsmoBTS context");
	msgb_talloc_ctx_init(tall_bts_ctx, 0);

	osmo_init_logging2(tall_bts_ctx, &bts_log_info);
	log_set_category_filter(osmo_stderr_target, DLOOP, 1, LOGL_DEBUG);
	log_set_category_filter(osmo_stderr_target, DL1C, 1, LOGL_DEBUG);
	log_set_print_filename2(osmo_stderr_target, LOG_FILENAME_NONE);
	log_set_use_color(osmo_stderr_target, 0);
	log_set_print_category(osmo_stderr_target, 0);
	log_set_print_category_hex(osmo_stderr_target, 0);

	test_power_loop();
	test_pf_algo_ewma();
	test_power_hysteresis();
	test_power_ctrl_interval();
	test_power_loop_ci();
	test_good_threshold_convergence();

	printf("Power loop test OK\n");

	return 0;
}
