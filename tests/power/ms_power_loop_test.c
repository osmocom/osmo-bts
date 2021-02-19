/*
 * (C) 2013,2014 by Holger Hans Peter Freyther
 * Contributions by sysmocom - s.m.f.c. GmbH <info@sysmocom.de>
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
	struct gsm_power_ctrl_params *params = &g_trx->ts[0].lchan[0].ms_dpc_params;
	g_trx->ts[0].lchan[0].ms_power_ctrl.dpc_params = params;
	*params = power_ctrl_params_def;

	/* Disable RxLev pre-processing and hysteresis by default */
	struct gsm_power_ctrl_meas_params *mp = &params->rxlev_meas;
	mp->lower_thresh = mp->upper_thresh = PWR_TEST_RXLEV_TARGET;
	mp->algo = GSM_PWR_CTRL_MEAS_AVG_ALGO_NONE;

	printf("\nStarting test case '%s'\n", name);
}

static inline void apply_power_test(struct gsm_lchan *lchan, int rxlev, int exp_ret, uint8_t exp_current)
{
	uint8_t old;
	int ret;

	old = lchan->ms_power_ctrl.current;
	ret = lchan_ms_pwr_ctrl(lchan, lchan->ms_power_ctrl.current, rxlev);

	/* Keep the measurement counter updated */
	lchan->meas.res_nr++;

	printf("lchan_ms_pwr_ctrl(RxLvl=%d dBm) returns %d (expected %d)\n",
	       rxlev, ret, exp_ret);
	printf("\tMS current power %u -> %u (expected %u)\n",
	       old, lchan->ms_power_ctrl.current, exp_current);
}

static void test_power_loop(void)
{
	struct gsm_lchan *lchan;

	init_test(__func__);
	lchan = &g_trx->ts[0].lchan[0];

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
	apply_power_test(lchan, PWR_TEST_RXLEV_TARGET_DBM, 0, 5);

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
	apply_power_test(lchan, PWR_TEST_RXLEV_TARGET_DBM, 0, 29);

	/* Now go down, steps are double size in this direction: */
	apply_power_test(lchan, -45, 1, 1);
	apply_power_test(lchan, -45, 1, 5);
	apply_power_test(lchan, -45, 1, 9);

	/* Go down only one level down and up: */
	apply_power_test(lchan, PWR_TEST_RXLEV_TARGET_DBM + 2, 1, 10);
	apply_power_test(lchan, PWR_TEST_RXLEV_TARGET_DBM - 2, 1, 9);

	/* Check if BSC requesting a low max power is applied after loop calculation: */
	lchan->ms_power_ctrl.max = ms_pwr_ctl_lvl(GSM_BAND_1800, 2);
	OSMO_ASSERT(lchan->ms_power_ctrl.max == 14);
	apply_power_test(lchan, PWR_TEST_RXLEV_TARGET_DBM + 2, 1, 14);
	/* Set back a more normal max: */
	lchan->ms_power_ctrl.max = ms_pwr_ctl_lvl(GSM_BAND_1800, 30);
	OSMO_ASSERT(lchan->ms_power_ctrl.max == 0);

	/* Disable dynamic power control and jump down */
	lchan->ms_power_ctrl.dpc_params = NULL;
	apply_power_test(lchan, -60, 0, 14);

	/* Enable and leave it again */
	lchan->ms_power_ctrl.dpc_params = &lchan->ms_dpc_params;
	apply_power_test(lchan, -40, 1, 15);
}

static void test_pf_algo_ewma(void)
{
	struct gsm_lchan *lchan;
	const int *avg100;

	init_test(__func__);
	lchan = &g_trx->ts[0].lchan[0];
	avg100 = &lchan->ms_power_ctrl.rxlev_meas_proc.ewma.Avg100;

	struct gsm_power_ctrl_meas_params *mp = &lchan->ms_dpc_params.rxlev_meas;
	mp->algo = GSM_PWR_CTRL_MEAS_AVG_ALGO_OSMO_EWMA;
	mp->ewma.alpha = 20; /* 80% smoothing */

	lchan->ms_power_ctrl.current = ms_pwr_ctl_lvl(GSM_BAND_1800, 0);
	OSMO_ASSERT(lchan->ms_power_ctrl.current == 15);
	lchan->ms_power_ctrl.max = ms_pwr_ctl_lvl(GSM_BAND_1800, 26);
	OSMO_ASSERT(lchan->ms_power_ctrl.max == 2);

#define CHECK_UL_RSSI_AVG100(exp) \
	printf("\tAvg[t] is %2.2f dBm (expected %2.2f dBm)\n", \
	       ((float) *avg100) / 100, exp);

	/* UL RSSI remains constant => no UL power change */
	apply_power_test(lchan, -75, 0, 15);
	CHECK_UL_RSSI_AVG100(-75.00);

	/* Avg[t] = (0.2 * -90) + (0.8 * -75) = -78.0 dBm */
	apply_power_test(lchan, -90, 1, 13);
	CHECK_UL_RSSI_AVG100(-78.00);

	/* Avg[t] = (0.2 * -90) + (0.8 * -78) = -80.4 dBm */
	apply_power_test(lchan, -90, 1, 11);
	CHECK_UL_RSSI_AVG100(-80.40);

	/* Avg[t] = (0.2 * -70) + (0.8 * -80.4) = -78.32 dBm,
	 * but due to up-/down-scaling artefacts we get the following:
	 *   Avg100[t] = Avg100[t - 1] + A * (Pwr - Avg[t] / 100)
	 *   Avg100[t] = -8040 + 20 * (-70 - (-8040 / 100))
	 *   Avg100[t] = -8040 + 20 * (-70 - (-8040 / 100))
	 *   Avg100[t] = -8040 + 20 * (-70 + 80)
	 *   Avg100[t] = -8040 + 200 = -7840
	 *   Avg[t] = -7840 / 100 = -78.4 */
	apply_power_test(lchan, -70, 1, 9);
	CHECK_UL_RSSI_AVG100(-78.40);

	mp->ewma.alpha = 70; /* 30% smoothing */
	lchan->ms_power_ctrl.current = 15;
	lchan->ms_power_ctrl.rxlev_meas_proc = \
		(struct gsm_power_ctrl_meas_proc_state) { 0 };

	/* This is the first sample, the filter outputs it as-is */
	apply_power_test(lchan, -50, 0, 15);
	CHECK_UL_RSSI_AVG100(-50.00);

	/* Avg[t] = (0.7 * -50) + (0.3 * -50) = -50.0 dBm */
	apply_power_test(lchan, -50, 0, 15);
	CHECK_UL_RSSI_AVG100(-50.0);

	/* Simulate SACCH block loss (-110 dBm):
	 * Avg[t] = (0.7 * -110) + (0.3 * -50) = -92.0 dBm */
	apply_power_test(lchan, -110, 1, 13);
	CHECK_UL_RSSI_AVG100(-92.0);
}

static void test_power_hysteresis(void)
{
	struct gsm_lchan *lchan;

	init_test(__func__);
	lchan = &g_trx->ts[0].lchan[0];

	/* Tolerate power deviations in range -80 .. -70 */
	lchan->ms_dpc_params.rxlev_meas.lower_thresh = 30;
	lchan->ms_dpc_params.rxlev_meas.upper_thresh = 40;

	lchan->ms_power_ctrl.current = ms_pwr_ctl_lvl(GSM_BAND_1800, 0);
	OSMO_ASSERT(lchan->ms_power_ctrl.current == 15);
	lchan->ms_power_ctrl.max = ms_pwr_ctl_lvl(GSM_BAND_1800, 26);
	OSMO_ASSERT(lchan->ms_power_ctrl.max == 2);

	apply_power_test(lchan, PWR_TEST_RXLEV_TARGET_DBM, 0, 15);
	apply_power_test(lchan, PWR_TEST_RXLEV_TARGET_DBM + 3, 0, 15);
	apply_power_test(lchan, PWR_TEST_RXLEV_TARGET_DBM - 3, 0, 15);

	apply_power_test(lchan, PWR_TEST_RXLEV_TARGET_DBM, 0, 15);
	apply_power_test(lchan, PWR_TEST_RXLEV_TARGET_DBM + 5, 0, 15);
	apply_power_test(lchan, PWR_TEST_RXLEV_TARGET_DBM - 5, 0, 15);

	apply_power_test(lchan, PWR_TEST_RXLEV_TARGET_DBM - 10, 1, 13);
}

static void test_power_ctrl_interval(void)
{
	struct gsm_lchan *lchan;
	unsigned int i, j;

	init_test(__func__);
	lchan = &g_trx->ts[0].lchan[0];

	lchan->ms_power_ctrl.max = ms_pwr_ctl_lvl(GSM_BAND_1800, 26);
	OSMO_ASSERT(lchan->ms_power_ctrl.max == 2);

	static const int script[][8][3] = {
		{ /* P_Con_INTERVAL=0 (480 ms) */
			/* { UL RxLev, expected rc, expected Tx power level } */
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	1,	13 },
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	1,	11 },
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	1,	 9 },
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	1,	 7 },
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	1,	 5 },
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	1,	 3 },
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	1,	 2 },
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	1,	 2 },
		},
		{ /* P_Con_INTERVAL=1 (960 ms) */
			/* { UL RxLev, expected rc, expected Tx power level } */
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	1,	13 },
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	0,	13 }, /* skipped */
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	1,	11 },
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	0,	11 }, /* skipped */
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	1,	 9 },
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	0,	 9 }, /* skipped */
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	1,	 7 },
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	0,	 7 }, /* skipped */
		},
		{ /* P_Con_INTERVAL=2 (1920 ms) */
			/* { UL RxLev, expected rc, expected Tx power level } */
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	1,	13 },
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	0,	13 }, /* skipped */
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	0,	13 }, /* skipped */
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	0,	13 }, /* skipped */
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	1,	11 },
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	0,	11 }, /* skipped */
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	0,	11 }, /* skipped */
			{ PWR_TEST_RXLEV_TARGET_DBM - 15,	0,	11 }, /* skipped */
		},
	};

	for (i = 0; i < ARRAY_SIZE(script); i++) {
		lchan->ms_power_ctrl.current = ms_pwr_ctl_lvl(GSM_BAND_1800, 0);
		OSMO_ASSERT(lchan->ms_power_ctrl.current == 15);

		/* Set the corresponding power control interval */
		printf("%s(): power control interval is now %u\n", __func__, i);
		lchan->ms_dpc_params.ctrl_interval = i;

		for (j = 0; j < ARRAY_SIZE(script[i]); j++) {
			apply_power_test(lchan, script[i][j][0],  /* UL RxLev */
						script[i][j][1],  /* expected rc */
						script[i][j][2]); /* expected Tx power level */
		}

		printf("\n");
	}
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
	log_set_print_category(osmo_stderr_target, 0);
	log_set_print_category_hex(osmo_stderr_target, 0);

	test_power_loop();
	test_pf_algo_ewma();
	test_power_hysteresis();
	test_power_ctrl_interval();

	printf("Power loop test OK\n");

	return 0;
}
