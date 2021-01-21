/*
 * (C) 2020 by sysmocom - s.m.f.c. GmbH <info@sysmocom.de>
 * Author: Vadim Yanitskiy <vyanitskiy@sysmocom.de>
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

#include <stdio.h>

#include <osmocom/core/utils.h>
#include <osmocom/core/talloc.h>
#include <osmocom/core/application.h>

#include <osmo-bts/bts.h>
#include <osmo-bts/l1sap.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/power_control.h>

#define PWR_TEST_RXLEV_TARGET	30

#define PWR_TEST_CFG_RXLEV_THRESH(hyst) \
	.lower_thresh = PWR_TEST_RXLEV_TARGET - hyst, \
	.upper_thresh = PWR_TEST_RXLEV_TARGET + hyst

#define DL_MEAS_FULL(rxqual, rxlev) \
	.rxqual_full = rxqual, \
	.rxlev_full = rxlev

#define DL_MEAS_SUB(rxqual, rxlev) \
	.rxqual_sub = rxqual, \
	.rxlev_sub = rxlev

#define DL_MEAS_FULL_SUB(rxqual, rxlev) \
	{ DL_MEAS_FULL(rxqual, rxlev), \
	  DL_MEAS_SUB(rxqual, rxlev) }

#define DL_MEAS_FULL_SUB_INV(rxqual, rxlev) \
	{ DL_MEAS_FULL(rxqual, rxlev), \
	  DL_MEAS_SUB(rxqual, rxlev), \
	  .invalid = true }

enum power_test_step_type {
	PWR_TEST_ST_IND_MEAS = 0,
	PWR_TEST_ST_IND_DUMMY,
	PWR_TEST_ST_SET_STATE,
	PWR_TEST_ST_SET_STEP_SIZE,
	PWR_TEST_ST_SET_RXLEV_PARAMS,
	PWR_TEST_ST_ENABLE_DTXD,
	PWR_TEST_ST_DISABLE_DPC,
};

struct power_test_step {
	/* Instruction to be performed */
	enum power_test_step_type type;
	/* Instruction parameters */
	union {
		/* Power Control state */
		struct lchan_power_ctrl_state state;
		/* Measurement pre-processing parameters */
		struct gsm_power_ctrl_meas_params mp;
		/* Indicated DL measurements */
		struct {
			uint8_t rxqual_full;
			uint8_t rxqual_sub;
			uint8_t rxlev_full;
			uint8_t rxlev_sub;
			bool invalid;
		} meas;
		/* Increase / reduce step size */
		struct {
			uint8_t inc;
			uint8_t red;
		} step_size;
	};
	/* Expected Tx power reduction */
	uint8_t exp_txred;
};

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

	g_bts->band = GSM_BAND_900;
	g_bts->c0 = g_trx;

	printf("\nStarting test case '%s'\n", name);
}

static void enc_meas_rep(struct gsm48_hdr *gh,
			 const unsigned int n,
			 const struct power_test_step *step)
{
	struct gsm48_meas_res *mr = (struct gsm48_meas_res *) gh->data;

	gh->proto_discr = GSM48_PDISC_RR;
	gh->msg_type = GSM48_MT_RR_MEAS_REP;

	*mr = (struct gsm48_meas_res) {
		.rxlev_full = step->meas.rxlev_full,
		.rxlev_sub = step->meas.rxlev_sub,
		.rxqual_full = step->meas.rxqual_full,
		.rxqual_sub = step->meas.rxqual_sub,
		/* NOTE: inversed logic (1 means invalid) */
		.meas_valid = step->meas.invalid,
	};

	printf("#%02u %s() -> Measurement Results (%svalid): "
	       "RXLEV-FULL(%02u), RXQUAL-FULL(%u), "
	       "RXLEV-SUB(%02u), RXQUAL-SUB(%u)\n",
	       n, __func__, step->meas.invalid ? "in" : "",
	       mr->rxlev_full, mr->rxqual_full,
	       mr->rxlev_sub, mr->rxqual_sub);
}

static int exec_power_step(struct gsm_lchan *lchan,
			   const unsigned int n,
			   const struct power_test_step *step)
{
	struct gsm48_hdr *gh;
	uint8_t old, new;
	uint8_t buf[18];

	gh = (struct gsm48_hdr *) buf;

	switch (step->type) {
	case PWR_TEST_ST_SET_STATE:
		printf("#%02u %s() <- State (re)set (current %u dB, max %u dB)\n",
		       n, __func__, step->state.current, step->state.max);
		lchan->bs_power_ctrl = step->state;
		lchan->bs_power_ctrl.dpc_params = &lchan->bs_dpc_params;
		return 0; /* we're done */
	case PWR_TEST_ST_DISABLE_DPC:
		printf("#%02u %s() <- Dynamic power control is disabled\n", n, __func__);
		lchan->bs_power_ctrl.dpc_params = NULL;
		return 0; /* we're done */
	case PWR_TEST_ST_SET_STEP_SIZE:
		printf("#%02u %s() <- Set step size: inc %u dB, red %u dB\n",
		       n, __func__, step->step_size.inc, step->step_size.red);
		lchan->bs_dpc_params.inc_step_size_db = step->step_size.inc;
		lchan->bs_dpc_params.red_step_size_db = step->step_size.red;
		return 0; /* we're done */
	case PWR_TEST_ST_SET_RXLEV_PARAMS:
		printf("#%02u %s() <- (Re)set RxLev params (thresh %u .. %u, "
							   "averaging is %sabled)\n",
		       n, __func__, step->mp.lower_thresh, step->mp.upper_thresh,
		       step->mp.algo != GSM_PWR_CTRL_MEAS_AVG_ALGO_NONE ? "en" : "dis");
		lchan->bs_dpc_params.rxlev_meas = step->mp;
		return 0; /* we're done */
	case PWR_TEST_ST_ENABLE_DTXD:
		printf("#%02u %s() <- Enable DTXd\n", n, __func__);
		lchan->tch.dtx.dl_active = true;
		return 0; /* we're done */
	case PWR_TEST_ST_IND_DUMMY:
		printf("#%02u %s() <- Dummy block\n", n, __func__);
		memset(buf, 0x2b, sizeof(buf));
		break;
	case PWR_TEST_ST_IND_MEAS:
		enc_meas_rep(gh, n, step);
		break;
	}

	printf("#%02u lchan_bs_pwr_ctrl() <- UL SACCH: %s\n",
	       n, osmo_hexdump(buf, sizeof(buf)));

	old = lchan->bs_power_ctrl.current;
	lchan_bs_pwr_ctrl(lchan, gh);
	new = lchan->bs_power_ctrl.current;

	printf("#%02u lchan_bs_pwr_ctrl() -> BS power reduction: "
	       "%u -> %u (expected %u)\n",
	       n, old, new, step->exp_txred);

	return new != step->exp_txred;
}

static void exec_power_test(const struct power_test_step *steps,
			    unsigned int num_steps,
			    const char *name)
{
	unsigned int n;
	int rc = 0;

	init_test(name);

	struct gsm_lchan *lchan = &g_trx->ts[0].lchan[0];
	struct gsm_power_ctrl_params *params = &lchan->bs_dpc_params;

	/* Default BS power control parameters */
	memcpy(params, &power_ctrl_params_def, sizeof(*params));

	/* No RxLev hysteresis: lower == upper */
	params->rxlev_meas.lower_thresh = PWR_TEST_RXLEV_TARGET;
	params->rxlev_meas.upper_thresh = PWR_TEST_RXLEV_TARGET;

	/* No RxLev pre-processing by default */
	params->rxlev_meas.algo = GSM_PWR_CTRL_MEAS_AVG_ALGO_NONE;

	for (n = 0; n < num_steps; n++)
		rc |= exec_power_step(lchan, n, &steps[n]);

	printf("Test case verdict: %s\n", rc ? "FAIL" : "SUCCESS");
}

/* Verify that the power remains constant in fixed mode. */
static const struct power_test_step TC_fixed_mode[] = {
	/* Initial state: 10 dB, up to 20 dB */
	{ .type = PWR_TEST_ST_SET_STATE,
	  .state = { .current = 10, .max = 2 * 10 } },
	{ .type = PWR_TEST_ST_DISABLE_DPC },

	/* MS indicates random RxQual/RxLev values, which must be ignored */
	{ .meas = DL_MEAS_FULL_SUB(0, 63),	.exp_txred = 10 },
	{ .meas = DL_MEAS_FULL_SUB(7, 0),	.exp_txred = 10 },
	{ .meas = DL_MEAS_FULL_SUB(0, 30),	.exp_txred = 10 },
	{ .meas = DL_MEAS_FULL_SUB(1, 30),	.exp_txred = 10 },
	{ .meas = DL_MEAS_FULL_SUB(1, 50),	.exp_txred = 10 },
};

/* Verify that the power remains constant if RxLev equals the target level. */
static const struct power_test_step TC_rxlev_target[] = {
	/* Initial state: 0 dB, up to 20 dB */
	{ .type = PWR_TEST_ST_SET_STATE,
	  .state = { .current = 0, .max = 2 * 10 } },

	/* MS indicates RxLev values that match the target level */
	{ .meas = DL_MEAS_FULL_SUB(0, PWR_TEST_RXLEV_TARGET) },
	{ .meas = DL_MEAS_FULL_SUB(0, PWR_TEST_RXLEV_TARGET) },
	{ .meas = DL_MEAS_FULL_SUB(0, PWR_TEST_RXLEV_TARGET) },
	{ .meas = DL_MEAS_FULL_SUB(0, PWR_TEST_RXLEV_TARGET) },
};

/* Verify that the power is gradually reduced/increased to the
 * minimum/maximum if the MS reports high/low RxLev values. */
static const struct power_test_step TC_rxlev_max_min[] = {
	/* Initial state: 0 dB, up to 20 dB */
	{ .type = PWR_TEST_ST_SET_STATE,
	  .state = { .current = 0, .max = 2 * 10 } },

	/* MS indicates high RxLev values (-50 dBm), inc step is 2 dB */
	{ .meas = DL_MEAS_FULL_SUB(0, 60),	.exp_txred =  2 },
	{ .meas = DL_MEAS_FULL_SUB(0, 60),	.exp_txred =  4 },
	{ .meas = DL_MEAS_FULL_SUB(0, 60),	.exp_txred =  6 },
	{ .meas = DL_MEAS_FULL_SUB(0, 60),	.exp_txred =  8 },
	{ .meas = DL_MEAS_FULL_SUB(0, 60),	.exp_txred = 10 },
	{ .meas = DL_MEAS_FULL_SUB(0, 60),	.exp_txred = 12 },
	{ .meas = DL_MEAS_FULL_SUB(0, 60),	.exp_txred = 14 },
	{ .meas = DL_MEAS_FULL_SUB(0, 60),	.exp_txred = 16 },
	{ .meas = DL_MEAS_FULL_SUB(0, 60),	.exp_txred = 18 },
	{ .meas = DL_MEAS_FULL_SUB(0, 60),	.exp_txred = 20 }, /* max */
	{ .meas = DL_MEAS_FULL_SUB(0, 60),	.exp_txred = 20 }, /* max */
	{ .meas = DL_MEAS_FULL_SUB(0, 60),	.exp_txred = 20 }, /* max */

	/* MS indicates low RxLev values (-100 dBm), red step is 4 dB */
	{ .meas = DL_MEAS_FULL_SUB(0, 10),	.exp_txred = 16 },
	{ .meas = DL_MEAS_FULL_SUB(0, 10),	.exp_txred = 12 },
	{ .meas = DL_MEAS_FULL_SUB(0, 10),	.exp_txred =  8 },
	{ .meas = DL_MEAS_FULL_SUB(0, 10),	.exp_txred =  4 },
	{ .meas = DL_MEAS_FULL_SUB(0, 10),	.exp_txred =  0 }, /* min */
	{ .meas = DL_MEAS_FULL_SUB(0, 10),	.exp_txred =  0 }, /* min */
	{ .meas = DL_MEAS_FULL_SUB(0, 10),	.exp_txred =  0 }, /* min */
};

/* Verify that delta values never exceed the corresponding step size,
 * but still can be smaller than the step size if the target is close. */
static const struct power_test_step TC_inc_red_step_size[] = {
	/* Initial state: 0 dB, up to 20 dB */
	{ .type = PWR_TEST_ST_SET_STATE,
	  .state = { .current = 0, .max = 2 * 10 } },

	{ .type = PWR_TEST_ST_SET_STEP_SIZE,
	  .step_size = { .inc = 6, .red = 4 } },

	/* MS indicates high RxLev values (-50 dBm), red step is 4 dB */
	{ .meas = DL_MEAS_FULL_SUB(0, 60),	.exp_txred =  4 },
	{ .meas = DL_MEAS_FULL_SUB(0, 60),	.exp_txred =  8 },
	{ .meas = DL_MEAS_FULL_SUB(0, 60),	.exp_txred = 12 },
	{ .meas = DL_MEAS_FULL_SUB(0, 60),	.exp_txred = 16 },
	{ .meas = DL_MEAS_FULL_SUB(0, 60),	.exp_txred = 20 }, /* max */
	{ .meas = DL_MEAS_FULL_SUB(0, 60),	.exp_txred = 20 }, /* max */

	/* MS indicates low RxLev values (-100 dBm), inc step is 6 dB */
	{ .meas = DL_MEAS_FULL_SUB(0, 10),	.exp_txred = 14 },
	{ .meas = DL_MEAS_FULL_SUB(0, 10),	.exp_txred =  8 },
	{ .meas = DL_MEAS_FULL_SUB(0, 10),	.exp_txred =  2 },
	{ .meas = DL_MEAS_FULL_SUB(0, 10),	.exp_txred =  0 }, /* min */
	{ .meas = DL_MEAS_FULL_SUB(0, 10),	.exp_txred =  0 }, /* min */

	/* Reset state: current 10 dB, up to 20 dB */
	{ .type = PWR_TEST_ST_SET_STATE,
	  .state = { .current = 10, .max = 2 * 10 } },

	/* Let's say the current value is now 1 dB greater than the target (current red 10 dB) */
	{ .meas = DL_MEAS_FULL_SUB(0, PWR_TEST_RXLEV_TARGET + 1),	.exp_txred = 10 + 1 },
	{ .meas = DL_MEAS_FULL_SUB(0, PWR_TEST_RXLEV_TARGET + 0),	.exp_txred = 10 + 1 },
	/* Let's say the current value is now 2 dB greater than the target (current red 11 dB) */
	{ .meas = DL_MEAS_FULL_SUB(0, PWR_TEST_RXLEV_TARGET + 2),	.exp_txred = 11 + 2 },
	{ .meas = DL_MEAS_FULL_SUB(0, PWR_TEST_RXLEV_TARGET + 0),	.exp_txred = 11 + 2 },
	/* Let's say the current value is now 3 dB greater than the target (current red 13 dB) */
	{ .meas = DL_MEAS_FULL_SUB(0, PWR_TEST_RXLEV_TARGET + 3),	.exp_txred = 13 + 3 },
	{ .meas = DL_MEAS_FULL_SUB(0, PWR_TEST_RXLEV_TARGET + 0),	.exp_txred = 13 + 3 },

	/* Reset state: current 10 dB, up to 20 dB */
	{ .type = PWR_TEST_ST_SET_STATE,
	  .state = { .current = 10, .max = 2 * 10 } },

	/* Let's say the current value is now 1 dB lower than the target (current red 10 dB) */
	{ .meas = DL_MEAS_FULL_SUB(0, PWR_TEST_RXLEV_TARGET - 1),	.exp_txred = 10 - 1 },
	{ .meas = DL_MEAS_FULL_SUB(0, PWR_TEST_RXLEV_TARGET - 0),	.exp_txred = 10 - 1 },
	/* Let's say the current value is now 3 dB lower than the target (current red 9 dB) */
	{ .meas = DL_MEAS_FULL_SUB(0, PWR_TEST_RXLEV_TARGET - 3),	.exp_txred = 9 - 3 },
	{ .meas = DL_MEAS_FULL_SUB(0, PWR_TEST_RXLEV_TARGET - 0),	.exp_txred = 9 - 3 },
	/* Let's say the current value is now 5 dB lower than the target (current red 6 dB) */
	{ .meas = DL_MEAS_FULL_SUB(0, PWR_TEST_RXLEV_TARGET - 5),	.exp_txred = 6 - 5 },
	{ .meas = DL_MEAS_FULL_SUB(0, PWR_TEST_RXLEV_TARGET - 0),	.exp_txred = 6 - 5 },
};

/* Verify that the logic picks the 'SUB' values in DTXd mode. */
static const struct power_test_step TC_dtxd_mode[] = {
	/* Initial state: 0 dB, up to 20 dB */
	{ .type = PWR_TEST_ST_SET_STATE,
	  .state = { .current = 0, .max = 2 * 10 } },

	{ .meas = DL_MEAS_FULL_SUB(0, PWR_TEST_RXLEV_TARGET) },
	{ .meas = DL_MEAS_FULL_SUB(0, PWR_TEST_RXLEV_TARGET) },

	{ .type = PWR_TEST_ST_ENABLE_DTXD }, /* DTXd mode */

	/* MS indicates target RxLev values as 'SUB', and random as 'FULL' */
	{ .meas = { DL_MEAS_FULL(7,  0), DL_MEAS_SUB(0, PWR_TEST_RXLEV_TARGET) } },
	{ .meas = { DL_MEAS_FULL(3, 30), DL_MEAS_SUB(0, PWR_TEST_RXLEV_TARGET) } },
	{ .meas = { DL_MEAS_FULL(0, 63), DL_MEAS_SUB(0, PWR_TEST_RXLEV_TARGET) } },
};

/* Verify that high RxQual reduces the current attenuation value. */
static const struct power_test_step TC_rxqual_ber[] = {
	/* Initial state: 16 dB, up to 20 dB */
	{ .type = PWR_TEST_ST_SET_STATE,
	  .state = { .current = 16, .max = 2 * 10 } },

	/* MS indicates target RxLev, and no bit errors */
	{ .meas = DL_MEAS_FULL_SUB(0, PWR_TEST_RXLEV_TARGET),	.exp_txred = 16 },
	{ .meas = DL_MEAS_FULL_SUB(0, PWR_TEST_RXLEV_TARGET),	.exp_txred = 16 },

	/* MS indicates target RxLev, but RxQual values better than L_RXQUAL_XX_P=3 */
	{ .meas = DL_MEAS_FULL_SUB(1, PWR_TEST_RXLEV_TARGET),	.exp_txred = 16 },
	{ .meas = DL_MEAS_FULL_SUB(2, PWR_TEST_RXLEV_TARGET),	.exp_txred = 16 },
	{ .meas = DL_MEAS_FULL_SUB(3, PWR_TEST_RXLEV_TARGET),	.exp_txred = 16 },

	/* MS indicates target RxLev, but RxQual values worse than L_RXQUAL_XX_P=3 */
	{ .meas = DL_MEAS_FULL_SUB(4, PWR_TEST_RXLEV_TARGET +  0),  .exp_txred = 16 -  4 },
	{ .meas = DL_MEAS_FULL_SUB(5, PWR_TEST_RXLEV_TARGET +  4),  .exp_txred = 16 -  8 },
	{ .meas = DL_MEAS_FULL_SUB(6, PWR_TEST_RXLEV_TARGET +  8),  .exp_txred = 16 - 12 },
	{ .meas = DL_MEAS_FULL_SUB(7, PWR_TEST_RXLEV_TARGET + 12),  .exp_txred = 16 - 16 }, /* max */
	{ .meas = DL_MEAS_FULL_SUB(7, PWR_TEST_RXLEV_TARGET + 16),  .exp_txred = 16 - 16 }, /* max */

	/* MS indicates target RxLev, but no bit errors anymore => reducing Tx power */
	{ .meas = DL_MEAS_FULL_SUB(0, PWR_TEST_RXLEV_TARGET + 16),  .exp_txred = 2 },
	{ .meas = DL_MEAS_FULL_SUB(0, PWR_TEST_RXLEV_TARGET + 14),  .exp_txred = 4 },
	{ .meas = DL_MEAS_FULL_SUB(0, PWR_TEST_RXLEV_TARGET + 12),  .exp_txred = 6 },

	/* Reset state: 0 dB, up to 20 dB */
	{ .type = PWR_TEST_ST_SET_STATE,
	  .state = { .current = 0, .max = 2 * 10 } },

	/* MS indicates target RxLev, but RxQual values worse than L_RXQUAL_XX_P=3 */
	{ .meas = DL_MEAS_FULL_SUB(7, PWR_TEST_RXLEV_TARGET) }, /* max */
	{ .meas = DL_MEAS_FULL_SUB(7, PWR_TEST_RXLEV_TARGET) }, /* max */
};

/* Verify that invalid and dummy SACCH blocks are ignored. */
static const struct power_test_step TC_inval_dummy[] = {
	/* Initial state: 16 dB, up to 20 dB */
	{ .type = PWR_TEST_ST_SET_STATE,
	  .state = { .current = 16, .max = 2 * 10 } },

	/* MS sends invalid measurement results which must be ignored */
	{ .meas = DL_MEAS_FULL_SUB_INV(7, 63),			.exp_txred = 16 },
	{ .meas = DL_MEAS_FULL_SUB_INV(0, 0),			.exp_txred = 16 },

	/* Let's say SMS (SAPI=3) blocks substitute some of the reports */
	{ .meas = DL_MEAS_FULL_SUB(0, PWR_TEST_RXLEV_TARGET),	.exp_txred = 16 },
	{ .type = PWR_TEST_ST_IND_DUMMY, /* not a report */	.exp_txred = 16 },
	{ .meas = DL_MEAS_FULL_SUB(0, PWR_TEST_RXLEV_TARGET),	.exp_txred = 16 },
	{ .type = PWR_TEST_ST_IND_DUMMY, /* not a report */	.exp_txred = 16 },
	{ .meas = DL_MEAS_FULL_SUB(0, PWR_TEST_RXLEV_TARGET),	.exp_txred = 16 },
};

/* Verify that small deviations from the target do not trigger any changes. */
static const struct power_test_step TC_rxlev_hyst[] = {
	/* Initial state: 16 dB, up to 20 dB */
	{ .type = PWR_TEST_ST_SET_STATE,
	  .state = { .current = 12, .max = 2 * 8 } },

	/* Hysteresis is not enabled, so small deviations trigger oscillations */
	{ .meas = DL_MEAS_FULL_SUB(0, PWR_TEST_RXLEV_TARGET + 1),	.exp_txred = 13 },
	{ .meas = DL_MEAS_FULL_SUB(0, PWR_TEST_RXLEV_TARGET - 2),	.exp_txred = 11 },
	{ .meas = DL_MEAS_FULL_SUB(0, PWR_TEST_RXLEV_TARGET + 3),	.exp_txred = 13 },
	{ .meas = DL_MEAS_FULL_SUB(0, PWR_TEST_RXLEV_TARGET - 2),	.exp_txred = 11 },

	/* Enable hysteresis */
	{ .type = PWR_TEST_ST_SET_RXLEV_PARAMS,
	  .mp = { PWR_TEST_CFG_RXLEV_THRESH(3) }
	},

	/* Hysteresis is enabled, so small deviations do not trigger any changes */
	{ .meas = DL_MEAS_FULL_SUB(0, PWR_TEST_RXLEV_TARGET + 1),	.exp_txred = 11 },
	{ .meas = DL_MEAS_FULL_SUB(0, PWR_TEST_RXLEV_TARGET - 2),	.exp_txred = 11 },
	{ .meas = DL_MEAS_FULL_SUB(0, PWR_TEST_RXLEV_TARGET + 3),	.exp_txred = 11 },
	{ .meas = DL_MEAS_FULL_SUB(0, PWR_TEST_RXLEV_TARGET - 2),	.exp_txred = 11 },
};

/* Verify EWMA based power filtering. */
static const struct power_test_step TC_rxlev_pf_ewma[] = {
	/* Initial state: 20 dB, up to 30 dB */
	{ .type = PWR_TEST_ST_SET_STATE,
	  .state = { .current = 16, .max = 2 * 15 } },

	/* Enable EWMA based pre-processing for RxLev */
	{ .type = PWR_TEST_ST_SET_RXLEV_PARAMS,
	  .mp = {
		PWR_TEST_CFG_RXLEV_THRESH(0),
		.algo = GSM_PWR_CTRL_MEAS_AVG_ALGO_OSMO_EWMA,
		.ewma.alpha = 50,
	  }
	},

	/* MS indicates target RxLev, power level remains constant */
	{ .meas = DL_MEAS_FULL_SUB(0, PWR_TEST_RXLEV_TARGET),		.exp_txred = 16 },
	{ .meas = DL_MEAS_FULL_SUB(0, PWR_TEST_RXLEV_TARGET),		.exp_txred = 16 },

	/* Avg[t] = (0.5 * 26) + (0.5 * 30) = 28, so delta is 2 */
	{ .meas = DL_MEAS_FULL_SUB(0, PWR_TEST_RXLEV_TARGET - 4),	.exp_txred = 14 },
	/* Avg[t] = (0.5 * 26) + (0.5 * 28) = 27, so delta is 3 */
	{ .meas = DL_MEAS_FULL_SUB(0, PWR_TEST_RXLEV_TARGET - 4),	.exp_txred = 11 },
	/* Avg[t] = (0.5 * 35) + (0.5 * 27) = 31, so delta is 1 */
	{ .meas = DL_MEAS_FULL_SUB(0, PWR_TEST_RXLEV_TARGET + 5),	.exp_txred = 12 },
	/* Avg[t] = (0.5 * 35) + (0.5 * 31) = 33, so delta is 3, but red step size is 2 dB */
	{ .meas = DL_MEAS_FULL_SUB(0, PWR_TEST_RXLEV_TARGET + 5),	.exp_txred = 14 },
};

int main(int argc, char **argv)
{
	printf("Testing BS Power loop...\n");

	tall_bts_ctx = talloc_named_const(NULL, 1, "OsmoBTS context");
	msgb_talloc_ctx_init(tall_bts_ctx, 0);

	osmo_init_logging2(tall_bts_ctx, &bts_log_info);
	osmo_stderr_target->categories[DLOOP].loglevel = LOGL_DEBUG;
	osmo_stderr_target->categories[DL1C].loglevel = LOGL_DEBUG;
	log_set_print_filename(osmo_stderr_target, 0);
	log_set_use_color(osmo_stderr_target, 0);

#define exec_test(test) \
	exec_power_test(test, ARRAY_SIZE(test), #test)

	exec_test(TC_fixed_mode);
	exec_test(TC_rxlev_target);
	exec_test(TC_rxlev_max_min); /* FIXME */
	exec_test(TC_inc_red_step_size);

	exec_test(TC_dtxd_mode);
	exec_test(TC_rxqual_ber);
	exec_test(TC_inval_dummy);

	exec_test(TC_rxlev_hyst);
	exec_test(TC_rxlev_pf_ewma);

	return 0;
}
