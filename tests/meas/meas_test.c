#include <stdio.h>
#include <stdint.h>

#include <osmocom/core/talloc.h>
#include <osmocom/core/application.h>
#include <osmocom/gsm/gsm_utils.h>

#include <osmo-bts/gsm_data.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/measurement.h>
#include <osmo-bts/rsl.h>

static struct gsm_bts *bts;
struct gsm_bts_trx *trx;

struct fn_sample {
	uint32_t fn;
	uint8_t ts;
	uint8_t ss;
	int rc;
};

#include "sysmobts_fr_samples.h"
#include "meas_testcases.h"

void test_fn_sample(struct fn_sample *s, unsigned int len, uint8_t pchan, uint8_t tsmap)
{
	int rc;
	struct gsm_lchan *lchan;
	unsigned int i;
	unsigned int delta = 0;
	uint8_t tsmap_result = 0;
	uint32_t fn_prev = 0;
	struct gsm_time gsm_time;


	printf("\n\n");
	printf("===========================================================\n");

	for (i = 0; i < len; i++) {

		lchan = &trx->ts[s[i].ts].lchan[s[i].ss];
		trx->ts[s[i].ts].pchan = pchan;
		lchan->meas.num_ul_meas = 1;

		rc = lchan_meas_check_compute(lchan, s[i].fn);
		if (rc) {
			gsm_fn2gsmtime(&gsm_time, s[i].fn);
			fprintf(stdout, "Testing: ts[%i]->lchan[%i], fn=%u=>%s, fn%%104=%u, rc=%i, delta=%i\n", s[i].ts,
				s[i].ss, s[i].fn, osmo_dump_gsmtime(&gsm_time), s[i].fn % 104, rc, s[i].fn - fn_prev);
			fn_prev = s[i].fn;
			tsmap_result |= (1 << s[i].ts);
		} else
			delta++;

		/* If the test data set provides a return
		 * code, we check that as well */
		if (s[i].rc != -1)
			OSMO_ASSERT(s[i].rc == rc);
	}

	/* Make sure that we exactly trigger on the right frames
	 * timeslots must match exactlty to what we expect */
	OSMO_ASSERT(tsmap_result == tsmap);
}

static void reset_lchan_meas(struct gsm_lchan *lchan)
{
	lchan->state = LCHAN_S_ACTIVE;
	memset(&lchan->meas, 0, sizeof(lchan->meas));
}

static void test_meas_compute(const struct meas_testcase *mtc)
{
	struct gsm_lchan *lchan = &trx->ts[1].lchan[0];
	unsigned int i;
	unsigned int fn = 0;

	printf("\nMeasurement Compute Test %s\n", mtc->name);

	lchan->ts->pchan = GSM_PCHAN_TCH_F;
	reset_lchan_meas(lchan);

	/* feed uplink measurements into the code */
	for (i = 0; i < mtc->ulm_count; i++) {
		lchan_new_ul_meas(lchan, (struct bts_ul_meas *) &mtc->ulm[i], fn);
		fn += 1;
	}

	/* compute the results */
	OSMO_ASSERT(lchan_meas_check_compute(lchan, mtc->final_fn) == mtc->res.success);
	if (!mtc->res.success) {
		OSMO_ASSERT(!(lchan->meas.flags & LC_UL_M_F_RES_VALID));
	} else {
		OSMO_ASSERT(lchan->meas.flags & (LC_UL_M_F_RES_VALID|LC_UL_M_F_OSMO_EXT_VALID));
		printf("meas.ext.toa256_min      | %6d | %6d\n",
			lchan->meas.ext.toa256_min, mtc->res.toa256_min);
		printf("meas.ext.toa256_max      | %6d | %6d\n",
			lchan->meas.ext.toa256_max, mtc->res.toa256_max);
		printf("meas.ms_toa256           | %6d | %6d\n",
			lchan->meas.ms_toa256, mtc->res.toa256_mean);
		printf("meas.ext.toa256_std_dev  | %6u | %6u\n",
			lchan->meas.ext.toa256_std_dev, mtc->res.toa256_std_dev);
		printf("meas.ul_res.full.rx_lev  | %6u | %6u\n",
			lchan->meas.ul_res.full.rx_lev, mtc->res.rx_lev_full);
		printf("meas.ul_res.full.rx_qual | %6u | %6u\n",
			lchan->meas.ul_res.full.rx_qual, mtc->res.rx_qual_full);

		if ((lchan->meas.ext.toa256_min != mtc->res.toa256_min) ||
		    (lchan->meas.ext.toa256_max != mtc->res.toa256_max) ||
		    (lchan->meas.ms_toa256 != mtc->res.toa256_mean) ||
		    (lchan->meas.ext.toa256_std_dev != mtc->res.toa256_std_dev) ||
		    (lchan->meas.ul_res.full.rx_lev != mtc->res.rx_lev_full)) {
			fprintf(stderr, "%s: Unexpected measurement result!\n", mtc->name);
		}
	}

}

/* This tests the function is_meas_overdue() and since is_meas_overdue()
 * internally makes use of is_meas_complete(), this also gives
 * is_meas_complete() a detailed check. */
static void test_is_meas_overdue(void)
{
	struct gsm_lchan *lchan;
	bool rc;
	uint32_t fn_missed_end;
	unsigned int i;

	printf("\n\n");
	printf("===========================================================\n");
	printf("Testing is_meas_overdue() and is_meas_complete()\n");

	/* Missing period-end-trigger at fn=12, TCH/F, TS0 */
	fn_missed_end = LCHAN_FN_DUMMY;
	lchan = &trx->ts[0].lchan[0];
	lchan->ts->pchan = GSM_PCHAN_TCH_F;
	lchan->meas.last_fn = 95;
	rc = is_meas_overdue(lchan, &fn_missed_end, 17 + 104);
	OSMO_ASSERT(rc);
	OSMO_ASSERT(fn_missed_end == 12 + 104);

	/* Missing period-end-trigger at fn=12, TCH/H, TS0 */
	fn_missed_end = LCHAN_FN_DUMMY;
	lchan = &trx->ts[0].lchan[0];
	lchan->ts->pchan = GSM_PCHAN_TCH_H;
	lchan->meas.last_fn = 95;
	rc = is_meas_overdue(lchan, &fn_missed_end, 17 + 104);
	OSMO_ASSERT(rc);
	OSMO_ASSERT(fn_missed_end == 12 + 104);

	/* Missing period-end-trigger at fn=12, TCH/H, TS1 */
	fn_missed_end = LCHAN_FN_DUMMY;
	lchan = &trx->ts[1].lchan[0];
	lchan->ts->pchan = GSM_PCHAN_TCH_H;
	lchan->meas.last_fn = 95;
	rc = is_meas_overdue(lchan, &fn_missed_end, 17 + 104);
	OSMO_ASSERT(rc);
	OSMO_ASSERT(fn_missed_end == 12 + 104);

	/* Missing period-end-trigger at fn=25, TCH/F, TS1 */
	fn_missed_end = LCHAN_FN_DUMMY;
	lchan = &trx->ts[1].lchan[0];
	lchan->ts->pchan = GSM_PCHAN_TCH_F;
	lchan->meas.last_fn = 21;
	rc = is_meas_overdue(lchan, &fn_missed_end, 30);
	OSMO_ASSERT(rc);
	OSMO_ASSERT(fn_missed_end == 25);

	/* Missing period-end-trigger at fn=25, TCH/H, TS0 */
	fn_missed_end = LCHAN_FN_DUMMY;
	lchan = &trx->ts[0].lchan[1];
	lchan->ts->pchan = GSM_PCHAN_TCH_H;
	lchan->meas.last_fn = 21;
	rc = is_meas_overdue(lchan, &fn_missed_end, 30);
	OSMO_ASSERT(rc);
	OSMO_ASSERT(fn_missed_end == 25);

	/* Missing period-end-trigger at fn=25, TCH/H, TS1 */
	fn_missed_end = LCHAN_FN_DUMMY;
	lchan = &trx->ts[1].lchan[1];
	lchan->ts->pchan = GSM_PCHAN_TCH_H;
	lchan->meas.last_fn = 21;
	rc = is_meas_overdue(lchan, &fn_missed_end, 30);
	OSMO_ASSERT(rc);
	OSMO_ASSERT(fn_missed_end == 25);

	/* Missing period-end-trigger at fn=38, TCH/F, TS2 */
	fn_missed_end = LCHAN_FN_DUMMY;
	lchan = &trx->ts[2].lchan[0];
	lchan->ts->pchan = GSM_PCHAN_TCH_F;
	lchan->meas.last_fn = 34;
	rc = is_meas_overdue(lchan, &fn_missed_end, 43);
	OSMO_ASSERT(rc);
	OSMO_ASSERT(fn_missed_end == 38);

	/* Missing period-end-trigger at fn=38, TCH/H, TS2 */
	fn_missed_end = LCHAN_FN_DUMMY;
	lchan = &trx->ts[2].lchan[0];
	lchan->ts->pchan = GSM_PCHAN_TCH_H;
	lchan->meas.last_fn = 34;
	rc = is_meas_overdue(lchan, &fn_missed_end, 43);
	OSMO_ASSERT(rc);
	OSMO_ASSERT(fn_missed_end == 38);

	/* Missing period-end-trigger at fn=38, TCH/H, TS3 */
	fn_missed_end = LCHAN_FN_DUMMY;
	lchan = &trx->ts[3].lchan[0];
	lchan->ts->pchan = GSM_PCHAN_TCH_H;
	lchan->meas.last_fn = 34;
	rc = is_meas_overdue(lchan, &fn_missed_end, 43);
	OSMO_ASSERT(rc);
	OSMO_ASSERT(fn_missed_end == 38);

	/* Missing period-end-trigger at fn=51, TCH/F, TS3 */
	fn_missed_end = LCHAN_FN_DUMMY;
	lchan = &trx->ts[3].lchan[0];
	lchan->ts->pchan = GSM_PCHAN_TCH_F;
	lchan->meas.last_fn = 47;
	rc = is_meas_overdue(lchan, &fn_missed_end, 52);
	OSMO_ASSERT(rc);
	OSMO_ASSERT(fn_missed_end == 51);

	/* Missing period-end-trigger at fn=51, TCH/H, TS2 */
	fn_missed_end = LCHAN_FN_DUMMY;
	lchan = &trx->ts[2].lchan[1];
	lchan->ts->pchan = GSM_PCHAN_TCH_H;
	lchan->meas.last_fn = 47;
	rc = is_meas_overdue(lchan, &fn_missed_end, 52);
	OSMO_ASSERT(rc);
	OSMO_ASSERT(fn_missed_end == 51);

	/* Missing period-end-trigger at fn=51, TCH/H, TS3 */
	fn_missed_end = LCHAN_FN_DUMMY;
	lchan = &trx->ts[3].lchan[1];
	lchan->ts->pchan = GSM_PCHAN_TCH_H;
	lchan->meas.last_fn = 47;
	rc = is_meas_overdue(lchan, &fn_missed_end, 52);
	OSMO_ASSERT(rc);
	OSMO_ASSERT(fn_missed_end == 51);

	/* Missing period-end-trigger at fn=64, TCH/F, TS4 */
	fn_missed_end = LCHAN_FN_DUMMY;
	lchan = &trx->ts[4].lchan[0];
	lchan->ts->pchan = GSM_PCHAN_TCH_F;
	lchan->meas.last_fn = 60;
	rc = is_meas_overdue(lchan, &fn_missed_end, 69);
	OSMO_ASSERT(rc);
	OSMO_ASSERT(fn_missed_end == 64);

	/* Missing period-end-trigger at fn=64, TCH/H, TS4 */
	fn_missed_end = LCHAN_FN_DUMMY;
	lchan = &trx->ts[4].lchan[0];
	lchan->ts->pchan = GSM_PCHAN_TCH_H;
	lchan->meas.last_fn = 60;
	rc = is_meas_overdue(lchan, &fn_missed_end, 69);
	OSMO_ASSERT(rc);
	OSMO_ASSERT(fn_missed_end == 64);

	/* Missing period-end-trigger at fn=64, TCH/H, TS4 */
	fn_missed_end = LCHAN_FN_DUMMY;
	lchan = &trx->ts[5].lchan[0];
	lchan->ts->pchan = GSM_PCHAN_TCH_H;
	lchan->meas.last_fn = 60;
	rc = is_meas_overdue(lchan, &fn_missed_end, 69);
	OSMO_ASSERT(rc);
	OSMO_ASSERT(fn_missed_end == 64);

	/* Missing period-end-trigger at fn=77, TCH/F, TS5 */
	fn_missed_end = LCHAN_FN_DUMMY;
	lchan = &trx->ts[5].lchan[0];
	lchan->ts->pchan = GSM_PCHAN_TCH_F;
	lchan->meas.last_fn = 73;
	rc = is_meas_overdue(lchan, &fn_missed_end, 78);
	OSMO_ASSERT(rc);
	OSMO_ASSERT(fn_missed_end == 77);

	/* Missing period-end-trigger at fn=77, TCH/H, TS4 */
	fn_missed_end = LCHAN_FN_DUMMY;
	lchan = &trx->ts[4].lchan[1];
	lchan->ts->pchan = GSM_PCHAN_TCH_H;
	lchan->meas.last_fn = 73;
	rc = is_meas_overdue(lchan, &fn_missed_end, 78);
	OSMO_ASSERT(rc);
	OSMO_ASSERT(fn_missed_end == 77);

	/* Missing period-end-trigger at fn=77, TCH/H, TS5 */
	fn_missed_end = LCHAN_FN_DUMMY;
	lchan = &trx->ts[5].lchan[1];
	lchan->ts->pchan = GSM_PCHAN_TCH_H;
	lchan->meas.last_fn = 73;
	rc = is_meas_overdue(lchan, &fn_missed_end, 78);
	OSMO_ASSERT(rc);
	OSMO_ASSERT(fn_missed_end == 77);

	/* Missing period-end-trigger at fn=90, TCH/F, TS6 */
	fn_missed_end = LCHAN_FN_DUMMY;
	lchan = &trx->ts[6].lchan[0];
	lchan->ts->pchan = GSM_PCHAN_TCH_F;
	lchan->meas.last_fn = 86;
	rc = is_meas_overdue(lchan, &fn_missed_end, 91);
	OSMO_ASSERT(rc);
	OSMO_ASSERT(fn_missed_end == 90);

	/* Missing period-end-trigger at fn=90, TCH/H, TS6 */
	fn_missed_end = LCHAN_FN_DUMMY;
	lchan = &trx->ts[6].lchan[0];
	lchan->ts->pchan = GSM_PCHAN_TCH_H;
	lchan->meas.last_fn = 86;
	rc = is_meas_overdue(lchan, &fn_missed_end, 91);
	OSMO_ASSERT(rc);
	OSMO_ASSERT(fn_missed_end == 90);

	/* Missing period-end-trigger at fn=90, TCH/H, TS7 */
	fn_missed_end = LCHAN_FN_DUMMY;
	lchan = &trx->ts[7].lchan[0];
	lchan->ts->pchan = GSM_PCHAN_TCH_H;
	lchan->meas.last_fn = 86;
	rc = is_meas_overdue(lchan, &fn_missed_end, 91);
	OSMO_ASSERT(rc);
	OSMO_ASSERT(fn_missed_end == 90);

	/* Missing period-end-trigger at fn=103, TCH/F, TS7 */
	fn_missed_end = LCHAN_FN_DUMMY;
	lchan = &trx->ts[7].lchan[0];
	lchan->ts->pchan = GSM_PCHAN_TCH_F;
	lchan->meas.last_fn = 99;
	rc = is_meas_overdue(lchan, &fn_missed_end, 0+104);
	OSMO_ASSERT(rc);
	OSMO_ASSERT(fn_missed_end == 103);

	/* Missing period-end-trigger at fn=103, TCH/H, TS6 */
	fn_missed_end = LCHAN_FN_DUMMY;
	lchan = &trx->ts[6].lchan[1];
	lchan->ts->pchan = GSM_PCHAN_TCH_H;
	lchan->meas.last_fn = 99;
	rc = is_meas_overdue(lchan, &fn_missed_end, 0+104);
	OSMO_ASSERT(rc);
	OSMO_ASSERT(fn_missed_end == 103);

	/* Missing period-end-trigger at fn=103, TCH/H, TS7 */
	fn_missed_end = LCHAN_FN_DUMMY;
	lchan = &trx->ts[7].lchan[1];
	lchan->ts->pchan = GSM_PCHAN_TCH_H;
	lchan->meas.last_fn = 99;
	rc = is_meas_overdue(lchan, &fn_missed_end, 0+104);
	OSMO_ASSERT(rc);
	OSMO_ASSERT(fn_missed_end == 103);

	/* Dropout inside the interval, no period-end-trigger missed */
	fn_missed_end = LCHAN_FN_DUMMY;
	lchan = &trx->ts[2].lchan[0];
	lchan->ts->pchan = GSM_PCHAN_TCH_H;
	lchan->meas.last_fn = 56;
	rc = is_meas_overdue(lchan, &fn_missed_end, 69);
	OSMO_ASSERT(!rc);
	OSMO_ASSERT(fn_missed_end == LCHAN_FN_DUMMY);

	/* No dropout, but right after period-end-trigger */
	fn_missed_end = LCHAN_FN_DUMMY;
	lchan = &trx->ts[2].lchan[0];
	lchan->ts->pchan = GSM_PCHAN_TCH_H;
	lchan->meas.last_fn = 38;
	rc = is_meas_overdue(lchan, &fn_missed_end, 39);
	OSMO_ASSERT(!rc);
	OSMO_ASSERT(fn_missed_end == LCHAN_FN_DUMMY);

	/* No dropout, two neigbouring frames at random position
	 * (should not happen in the real world) */
	fn_missed_end = LCHAN_FN_DUMMY;
	lchan = &trx->ts[2].lchan[0];
	lchan->ts->pchan = GSM_PCHAN_TCH_H;
	lchan->meas.last_fn = 43;
	rc = is_meas_overdue(lchan, &fn_missed_end, 44);
	OSMO_ASSERT(!rc);
	OSMO_ASSERT(fn_missed_end == LCHAN_FN_DUMMY);

	/* No dropout, Two neigbouring frames (period end, right side) */
	fn_missed_end = LCHAN_FN_DUMMY;
	lchan = &trx->ts[2].lchan[0];
	lchan->ts->pchan = GSM_PCHAN_TCH_H;
	lchan->meas.last_fn = 38;
	rc = is_meas_overdue(lchan, &fn_missed_end, 39);
	OSMO_ASSERT(!rc);
	OSMO_ASSERT(fn_missed_end == LCHAN_FN_DUMMY);

	/* No dropout, Two neigbouring frames (period end, left side,
	 * should not happen in the real world) */
	fn_missed_end = LCHAN_FN_DUMMY;
	lchan = &trx->ts[2].lchan[0];
	lchan->ts->pchan = GSM_PCHAN_TCH_H;
	lchan->meas.last_fn = 37;
	rc = is_meas_overdue(lchan, &fn_missed_end, 38);
	OSMO_ASSERT(!rc);
	OSMO_ASSERT(fn_missed_end == LCHAN_FN_DUMMY);

	/* No dropout, test directly on a the trigger frame */
	fn_missed_end = LCHAN_FN_DUMMY;
	lchan = &trx->ts[2].lchan[0];
	lchan->ts->pchan = GSM_PCHAN_TCH_H;
	lchan->meas.last_fn = 34;
	rc = is_meas_overdue(lchan, &fn_missed_end, 38);
	OSMO_ASSERT(!rc);
	OSMO_ASSERT(fn_missed_end == LCHAN_FN_DUMMY);

	/* No dropout, previous frame is trigger frame
	 * (should not happen in the real world) */
	fn_missed_end = LCHAN_FN_DUMMY;
	lchan = &trx->ts[2].lchan[0];
	lchan->ts->pchan = GSM_PCHAN_TCH_H;
	lchan->meas.last_fn = 38;
	rc = is_meas_overdue(lchan, &fn_missed_end, 38);
	OSMO_ASSERT(!rc);
	OSMO_ASSERT(fn_missed_end == LCHAN_FN_DUMMY);

	/* Missing period-end-trigger at fn=38+i*104, TCH/F, TS2 to
	 * see the modulus is correct. */
	for (i = 0; i < 100; i++) {
		fn_missed_end = LCHAN_FN_DUMMY;
		lchan = &trx->ts[2].lchan[0];
		lchan->ts->pchan = GSM_PCHAN_TCH_F;
		lchan->meas.last_fn = 34 + 104 * 1;
		rc = is_meas_overdue(lchan, &fn_missed_end, 43 + 104 * 1);
		OSMO_ASSERT(rc);
		OSMO_ASSERT(fn_missed_end == 38 + 104 * 1);
	}

	/* See whats happening if we miss a period-end-triggerend at the
	 * hyperframe beginning. */
	fn_missed_end = LCHAN_FN_DUMMY;
	lchan = &trx->ts[0].lchan[0];
	lchan->ts->pchan = GSM_PCHAN_TCH_F;
	lchan->meas.last_fn = GSM_MAX_FN-104+95;
	rc = is_meas_overdue(lchan, &fn_missed_end, 17);
	OSMO_ASSERT(rc);
	OSMO_ASSERT(fn_missed_end == 12);

	/* See whats happening if we miss a period-end-triggerend at the
	 * hyperframe ending. */
	fn_missed_end = LCHAN_FN_DUMMY;
	lchan = &trx->ts[6].lchan[0];
	lchan->ts->pchan = GSM_PCHAN_TCH_F;
	lchan->meas.last_fn = GSM_MAX_FN-104+86;
	rc = is_meas_overdue(lchan, &fn_missed_end, 8);
	OSMO_ASSERT(rc);
	OSMO_ASSERT(fn_missed_end == GSM_MAX_FN-104+90);

	/* See whats happening if we miss a period-end-triggerend exactly at the
	 * hyperframe ending. */
	fn_missed_end = LCHAN_FN_DUMMY;
	lchan = &trx->ts[7].lchan[0];
	lchan->ts->pchan = GSM_PCHAN_TCH_F;
	lchan->meas.last_fn = GSM_MAX_FN-104+99;
	rc = is_meas_overdue(lchan, &fn_missed_end, 0);
	OSMO_ASSERT(rc);
	OSMO_ASSERT(fn_missed_end == GSM_MAX_FN-1);

	/* Test a wrap around at the hyperframe ending, while no measurements
	 * are lost */
	fn_missed_end = LCHAN_FN_DUMMY;
	lchan = &trx->ts[0].lchan[0];
	lchan->ts->pchan = GSM_PCHAN_TCH_F;
	lchan->meas.last_fn = GSM_MAX_FN-104+99;
	rc = is_meas_overdue(lchan, &fn_missed_end, 0);
	OSMO_ASSERT(!rc);
	OSMO_ASSERT(fn_missed_end == LCHAN_FN_DUMMY);

	/* Test a wrap around at the hyperframe ending, measurements are lost,
	 * but not the one that triggers the period end */
	fn_missed_end = LCHAN_FN_DUMMY;
	lchan = &trx->ts[0].lchan[0];
	lchan->ts->pchan = GSM_PCHAN_TCH_F;
	lchan->meas.last_fn = GSM_MAX_FN-104+95;
	rc = is_meas_overdue(lchan, &fn_missed_end, 4);
	OSMO_ASSERT(!rc);
	OSMO_ASSERT(fn_missed_end == LCHAN_FN_DUMMY);

	/* Test a wrap around right before the hyperframe ending, while no
	 * measurements are lost. */
	fn_missed_end = LCHAN_FN_DUMMY;
	lchan = &trx->ts[7].lchan[0];
	lchan->ts->pchan = GSM_PCHAN_TCH_F;
	lchan->meas.last_fn = GSM_MAX_FN-104+99;
	rc = is_meas_overdue(lchan, &fn_missed_end, GSM_MAX_FN-1);
	OSMO_ASSERT(!rc);
	OSMO_ASSERT(fn_missed_end == LCHAN_FN_DUMMY);
}

/* This tests the robustness of lchan_meas_process_measurement(). This is the
 * function that is called from l1_sap.c each time a measurement indication is
 * received. The process must still go on when measurement indications (blocks)
 * are lost or otherwise spaced out. Even the complete absence of the
 * measurement indications from the SACCH which are used to detect the interval
 * end must not keep the interval from beeing processed. */
void test_lchan_meas_process_measurement(bool no_sacch, bool dropouts)
{
	struct gsm_lchan *lchan = &trx->ts[2].lchan[0];
	unsigned int i;
	unsigned int k = 0;
	unsigned int fn = 0;
	struct bts_ul_meas ulm;

	printf("\n\n");
	printf("===========================================================\n");
	printf("Testing lchan_meas_process_measurement()\n");
	if (no_sacch)
		printf(" * SACCH blocks not generated.\n");
	if (dropouts)
		printf
		    (" * Simulate dropouts by leaving out every 4th measurement\n");

	ulm.ber10k = 0;
	ulm.ta_offs_256bits = 256;
	ulm.c_i = 0;
	ulm.is_sub = 0;
	ulm.inv_rssi = 90;

	lchan->ts->pchan = GSM_PCHAN_TCH_F;
	reset_lchan_meas(lchan);

	/* feed uplink measurements into the code */
	for (i = 0; i < 100; i++) {

		if (dropouts == false || i % 4)
			lchan_meas_process_measurement(lchan, &ulm, fn);
		else
			printf
			    ("(leaving out measurement sample for frame number %u)\n",
			     fn);

		fn += 4;
		if (k == 2) {
			fn++;
			k = 0;
		} else
			k++;

		if (fn % 104 == 39 && no_sacch == false) {
			printf
			    ("(now adding measurement sample for SACCH block)\n");
			lchan_meas_process_measurement(lchan, &ulm, fn - 1);
		} else
			printf
			    ("(leaving out measurement sample for SACCH block)\n");
	}
}

int main(int argc, char **argv)
{
	void *tall_bts_ctx;

	tall_bts_ctx = talloc_named_const(NULL, 1, "OsmoBTS context");
	msgb_talloc_ctx_init(tall_bts_ctx, 0);

	osmo_init_logging2(tall_bts_ctx, &bts_log_info);
	osmo_stderr_target->categories[DMEAS].loglevel = LOGL_DEBUG;

	bts = gsm_bts_alloc(tall_bts_ctx, 0);
	if (!bts) {
		fprintf(stderr, "Failed to create BTS structure\n");
		exit(1);
	}
	trx = gsm_bts_trx_alloc(bts);
	if (!trx) {
		fprintf(stderr, "Failed to TRX structure\n");
		exit(1);
	}

	if (bts_init(bts) < 0) {
		fprintf(stderr, "unable to to open bts\n");
		exit(1);
	}

	printf("\n");
	printf("***********************\n");
	printf("*** FULL RATE TESTS ***\n");
	printf("***********************\n");

	/* Test full rate */
	test_fn_sample(test_fn_tch_f_ts_2_3, ARRAY_SIZE(test_fn_tch_f_ts_2_3), GSM_PCHAN_TCH_F, (1 << 2) | (1 << 3));
	test_fn_sample(test_fn_tch_f_ts_4_5, ARRAY_SIZE(test_fn_tch_f_ts_4_5), GSM_PCHAN_TCH_F, (1 << 4) | (1 << 5));
	test_fn_sample(test_fn_tch_f_ts_6_7, ARRAY_SIZE(test_fn_tch_f_ts_6_7), GSM_PCHAN_TCH_F, (1 << 6) | (1 << 7));

	printf("\n");
	printf("***********************\n");
	printf("*** HALF RATE TESTS ***\n");
	printf("***********************\n");

	/* Test half rate */
	test_fn_sample(test_fn_tch_h_ts_2_ss0_ss1, ARRAY_SIZE(test_fn_tch_h_ts_2_ss0_ss1), GSM_PCHAN_TCH_H, (1 << 2));
	test_fn_sample(test_fn_tch_h_ts_3_ss0_ss1, ARRAY_SIZE(test_fn_tch_h_ts_3_ss0_ss1), GSM_PCHAN_TCH_H, (1 << 3));
	test_fn_sample(test_fn_tch_h_ts_4_ss0_ss1, ARRAY_SIZE(test_fn_tch_h_ts_4_ss0_ss1), GSM_PCHAN_TCH_H, (1 << 4));
	test_fn_sample(test_fn_tch_h_ts_5_ss0_ss1, ARRAY_SIZE(test_fn_tch_h_ts_5_ss0_ss1), GSM_PCHAN_TCH_H, (1 << 5));
	test_fn_sample(test_fn_tch_h_ts_6_ss0_ss1, ARRAY_SIZE(test_fn_tch_h_ts_6_ss0_ss1), GSM_PCHAN_TCH_H, (1 << 6));
	test_fn_sample(test_fn_tch_h_ts_7_ss0_ss1, ARRAY_SIZE(test_fn_tch_h_ts_7_ss0_ss1), GSM_PCHAN_TCH_H, (1 << 7));

	test_meas_compute(&mtc1);
	test_meas_compute(&mtc2);
	test_meas_compute(&mtc3);
	test_meas_compute(&mtc4);
	test_meas_compute(&mtc5);

	printf("\n");
	printf("***************************************************\n");
	printf("*** MEASUREMENT INTERVAL ENDING DETECTION TESTS ***\n");
	printf("***************************************************\n");

	test_is_meas_overdue();
	test_lchan_meas_process_measurement(false, false);
	test_lchan_meas_process_measurement(true, false);
	test_lchan_meas_process_measurement(false, true);
	test_lchan_meas_process_measurement(true, true);

	printf("Success\n");

	return 0;
}

/* Stubs */
void bts_model_abis_close(struct gsm_bts *bts)
{
}

int bts_model_oml_estab(struct gsm_bts *bts)
{
	return 0;
}

int bts_model_l1sap_down(struct gsm_bts_trx *trx, struct osmo_phsap_prim *l1sap)
{
	return 0;
}

int bts_model_check_oml(struct gsm_bts *bts, uint8_t msg_type, struct tlv_parsed *old_attr, struct tlv_parsed *new_attr,
			void *obj)
{
	return 0;
}

int bts_model_apply_oml(struct gsm_bts *bts, struct msgb *msg, struct tlv_parsed *new_attr, int obj_kind, void *obj)
{
	return 0;
}

int bts_model_opstart(struct gsm_bts *bts, struct gsm_abis_mo *mo, void *obj)
{
	return 0;
}

int bts_model_chg_adm_state(struct gsm_bts *bts, struct gsm_abis_mo *mo, void *obj, uint8_t adm_state)
{
	return 0;
}

int bts_model_init(struct gsm_bts *bts)
{
	return 0;
}

int bts_model_trx_deact_rf(struct gsm_bts_trx *trx)
{
	return 0;
}

int bts_model_trx_close(struct gsm_bts_trx *trx)
{
	return 0;
}

void trx_get_hlayer1(void)
{
}

int bts_model_adjst_ms_pwr(struct gsm_lchan *lchan)
{
	return 0;
}

int bts_model_ts_disconnect(struct gsm_bts_trx_ts *ts)
{
	return 0;
}

int bts_model_ts_connect(struct gsm_bts_trx_ts *ts, enum gsm_phys_chan_config as_pchan)
{
	return 0;
}

int bts_model_lchan_deactivate(struct gsm_lchan *lchan)
{
	return 0;
}

int bts_model_lchan_deactivate_sacch(struct gsm_lchan *lchan)
{
	return 0;
}
