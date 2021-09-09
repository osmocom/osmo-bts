/* Test cases for tx_control.c Timing Advance Computation */

/* (C) 2016 by sysmocom s.f.m.c. GmbH <info@sysmocom.de>
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
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>

#include <osmocom/core/talloc.h>
#include <osmocom/core/application.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/gsm_data.h>
#include <osmo-bts/ta_control.h>

void lchan_ms_ta_ctrl_test(int16_t toa256_start, unsigned int steps)
{
	struct gsm_lchan lchan = { };
	unsigned int i;
	uint8_t rqd_ta_after;
	uint8_t rqd_ta_before;
	int16_t toa256 = toa256_start;

	printf("toa256_start = %u / 256 = %u, steps = %u\n", toa256_start,
	       toa256_start / 256, steps);

	for (i = 0; i < steps; i++) {
		printf("Step #%u\n", i);
		printf("  lchan.ta_ctrl.current (before) = %u\n", lchan.ta_ctrl.current);
		printf("  toa256 (before) = %u / 256 = %u\n", toa256,
		       toa256 / 256);

		rqd_ta_before = lchan.ta_ctrl.current;

		lchan_ms_ta_ctrl(&lchan, rqd_ta_before, toa256);

		rqd_ta_after = lchan.ta_ctrl.current;
		toa256 -= (rqd_ta_after - rqd_ta_before) * 256;

		printf("  lchan.ta_ctrl.current (after) = %u\n", lchan.ta_ctrl.current);
		printf("  toa256 (after) = %u / 256 = %u\n", toa256,
		       toa256 / 256);
	}

	printf("Done.\n");
	printf("\n");
}

int main(int argc, char **argv)
{
	void *tall_bts_ctx;

	tall_bts_ctx = talloc_named_const(NULL, 1, "OsmoBTS context");
	osmo_init_logging2(tall_bts_ctx, &bts_log_info);

	lchan_ms_ta_ctrl_test(16 * 256, 20);
	lchan_ms_ta_ctrl_test(4000, 50);
	lchan_ms_ta_ctrl_test(12345, 50);
}
