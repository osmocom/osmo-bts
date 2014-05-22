/* testing misc code */

/* (C) 2011 by Holger Hans Peter Freyther
 * (C) 2014 by sysmocom s.f.m.c. GmbH
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

#include <osmo-bts/bts.h>

#include <stdlib.h>
#include <stdio.h>

static void test_sacch_get(void)
{
	struct gsm_lchan lchan;
	int i, off;

	printf("Testing lchan_sacch_get\n");
	memset(&lchan, 0, sizeof(lchan));

	/* initialize the input. */
	for (i = 1; i < _MAX_SYSINFO_TYPE; ++i) {
		lchan.si.valid |= (1 << i);
		memset(&lchan.si.buf[i], i, sizeof(lchan.si.buf[i]));
	}

	/* It will start with '1' */
	for (i = 1, off = 0; i <= 32; ++i) {
		uint8_t *data = lchan_sacch_get(&lchan);
		off = (off + 1) % _MAX_SYSINFO_TYPE;
		if (off == 0)
			off += 1;

		//printf("i=%d (%%=%d) -> data[0]=%d\n", i, off, data[0]);
		OSMO_ASSERT(data[0] == off);
	}
}

int main(int argc, char **argv)
{
	test_sacch_get();
	return EXIT_SUCCESS;
}
