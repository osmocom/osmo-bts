/* (C) 2021 by sysmocom s.f.m.c. GmbH
 * All Rights Reserved
 *
 * Author: Philipp Maier
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

#include <osmo-bts/logging.h>
#include <osmocom/core/utils.h>
#include <stdlib.h>
#include <stdio.h>
#include "../../src/osmo-bts-trx/sched_utils.h"

struct amr_cmi_test_data {
	/* Frame number that marks the beginning of the voice block */
	uint32_t gsm_fn;
	/* In uplink: True, when the voice block is a CMI block, false otherwise. */
	/* In downlink: False, when the voice block is a CMI block, true otherwise. */
	bool is_cmi;
};

/* The behavior of AHS in subslot 0 and AFS is the same */
static const struct amr_cmi_test_data testvec_ahs_h0_and_afs[] = {
	{ 0, true },
	{ 4, false },
	{ 8, true },
	{ 13, false },
	{ 17, true },
	{ 21, false },
	{ 26, true },
	{ 30, false },
	{ 34, true },
	{ 39, false },
	{ 43, true },
	{ 47, false },
	{ 52, true },
	{ 56, false },
	{ 60, true },
	{ 65, false },
	{ 69, true },
	{ 73, false },
	{ 78, true },
	{ 82, false },
	{ 86, true },
	{ 91, false },
	{ 95, true },
	{ 99, false },
};

static const struct amr_cmi_test_data testvec_ahs_h1[] = {
	{ 1, true },
	{ 5, false },
	{ 9, true },
	{ 14, false },
	{ 18, true },
	{ 22, false },
	{ 27, true },
	{ 31, false },
	{ 35, true },
	{ 40, false },
	{ 44, true },
	{ 48, false },
	{ 53, true },
	{ 57, false },
	{ 61, true },
	{ 66, false },
	{ 70, true },
	{ 74, false },
	{ 79, true },
	{ 83, false },
	{ 87, true },
	{ 92, false },
	{ 96, true },
	{ 100, false },
};

static void test_amr_cmi_sched(void)
{
	unsigned int i;
	bool res;

	printf("AMR transmission phase (CMI) in relation to GSM FN:\n");

	for (i = 0; i < ARRAY_SIZE(testvec_ahs_h0_and_afs); i++) {
		res = ul_amr_fn_is_cmi(testvec_ahs_h0_and_afs[i].gsm_fn);
		printf("Uplink, AMR AHS on HR subslot 0: fn_begin=%u, CMI=%u\n", testvec_ahs_h0_and_afs[i].gsm_fn, res);
		OSMO_ASSERT(res == testvec_ahs_h0_and_afs[i].is_cmi);
	}

	printf("\n");

	for (i = 0; i < ARRAY_SIZE(testvec_ahs_h0_and_afs); i++) {
		res = dl_amr_fn_is_cmi(testvec_ahs_h0_and_afs[i].gsm_fn);
		printf("Downlink, AMR AHS on HR subslot 0: fn_begin=%u, CMI=%u\n", testvec_ahs_h0_and_afs[i].gsm_fn, res);
		OSMO_ASSERT(res == !testvec_ahs_h0_and_afs[i].is_cmi);
	}

	printf("\n");
	printf("\n");

	for (i = 0; i < ARRAY_SIZE(testvec_ahs_h1); i++) {
		res = ul_amr_fn_is_cmi(testvec_ahs_h1[i].gsm_fn);
		printf("Uplink, AMR AHS on HR subslot 1: fn_begin=%u, CMI=%u\n", testvec_ahs_h1[i].gsm_fn, res);
		OSMO_ASSERT(res == testvec_ahs_h1[i].is_cmi);
	}

	printf("\n");

	for (i = 0; i < ARRAY_SIZE(testvec_ahs_h1); i++) {
		res = dl_amr_fn_is_cmi(testvec_ahs_h1[i].gsm_fn);
		printf("Downlink, AMR AHS on HR subslot 1: fn_begin=%u, CMI=%u\n", testvec_ahs_h1[i].gsm_fn, res);
		OSMO_ASSERT(res == !testvec_ahs_h1[i].is_cmi);
	}

	printf("\n");
	printf("\n");

	for (i = 0; i < ARRAY_SIZE(testvec_ahs_h0_and_afs); i++) {
		res = ul_amr_fn_is_cmi(testvec_ahs_h0_and_afs[i].gsm_fn);
		printf("Uplink, AMR AFS: fn_begin=%u, CMI=%u\n", testvec_ahs_h0_and_afs[i].gsm_fn, res);
		OSMO_ASSERT(res == testvec_ahs_h0_and_afs[i].is_cmi);
	}

	printf("\n");

	for (i = 0; i < ARRAY_SIZE(testvec_ahs_h0_and_afs); i++) {
		res = dl_amr_fn_is_cmi(testvec_ahs_h0_and_afs[i].gsm_fn);
		printf("Downlink, AMR AFS: fn_begin=%u, CMI=%u\n", testvec_ahs_h0_and_afs[i].gsm_fn, res);
		OSMO_ASSERT(res == !testvec_ahs_h0_and_afs[i].is_cmi);
	}
}

int main(int argc, char **argv)
{

	test_amr_cmi_sched();
	return EXIT_SUCCESS;
}
