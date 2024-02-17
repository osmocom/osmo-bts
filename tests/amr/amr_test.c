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
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <osmo-bts/logging.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/amr.h>

#include <osmocom/core/application.h>
#include <osmocom/core/utils.h>

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>

static void test_amr_parse_mr_conf(void)
{
	uint8_t mrc_enc[] = { 0x20, 0xa5, 0x0d, 0x46, 0x52, 0x54 };
	struct amr_multirate_conf mrc = { 0 };
	unsigned int i;

	printf("amr_parse_mr_conf() <- %s\n", osmo_hexdump(&mrc_enc[0], sizeof(mrc_enc)));
	OSMO_ASSERT(amr_parse_mr_conf(&mrc, &mrc_enc[0], sizeof(mrc_enc)) > 0);
	printf("amr_parse_mr_conf() -> num_modes=%u\n", mrc.num_modes);
	for (i = 0; i < mrc.num_modes; i++) {
		printf("  Mode[%u] = %u/%u/%u\n",
		       i, mrc.mode[i].mode,
		       mrc.mode[i].threshold,
		       mrc.mode[i].hysteresis);
	}

	mrc_enc[1] = 0xff; /* all codec modes active */
	printf("amr_parse_mr_conf() <- %s\n", osmo_hexdump(&mrc_enc[0], sizeof(mrc_enc)));
	OSMO_ASSERT(amr_parse_mr_conf(&mrc, &mrc_enc[0], sizeof(mrc_enc)) == -EINVAL);

	mrc_enc[0] = 0xff; /* unknown version */
	printf("amr_parse_mr_conf() <- %s\n", osmo_hexdump(&mrc_enc[0], sizeof(mrc_enc)));
	OSMO_ASSERT(amr_parse_mr_conf(&mrc, &mrc_enc[0], sizeof(mrc_enc)) == -EINVAL);

	printf("amr_parse_mr_conf() <- %s\n", osmo_hexdump(&mrc_enc[0], 1)); /* short read */
	OSMO_ASSERT(amr_parse_mr_conf(&mrc, &mrc_enc[0], 1) == -EINVAL);
}

int main(int argc, char **argv)
{
	osmo_init_logging2(NULL, &bts_log_info);
	log_set_print_filename2(osmo_stderr_target, LOG_FILENAME_NONE);
	log_set_print_category_hex(osmo_stderr_target, 0);
	log_set_print_category(osmo_stderr_target, 1);
	log_set_print_level(osmo_stderr_target, 1);
	log_set_use_color(osmo_stderr_target, 0);

	test_amr_parse_mr_conf();
	return EXIT_SUCCESS;
}
