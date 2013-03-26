/* AMR support for OsmoBTS-TRX */

/* (C) 2013 by Andreas Eversberg <jolly@eversberg.eu>
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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <stdint.h>

static int amr_len_by_ft[16] = {
	12, 13, 15, 17, 19, 20, 26, 31,
	0,  0,  0,  0,  0,  0,  0,  0
};

int amr_decompose_payload(uint8_t *payload, int payload_len, uint8_t *_cmr,
	uint8_t *_ft, uint8_t *_bfi)
{
	uint8_t cmr, f, ft, q;

	if (payload_len < 2)
		return -EINVAL;

	cmr = payload[0] >> 4;
	if (_cmr)
		*_cmr = cmr;

	f = payload[1] >> 7;

	ft = (payload[1] >> 3) & 0xf;
	if (_ft)
		*_ft = ft;

	q = (payload[1] >> 2) & 0x1;
	if (_bfi)
		*_bfi = !q;
	
	if (f) {
		fprintf(stderr, "%s: multiple payloads not supported\n",
			__func__);
		return -ENOTSUP;
	}

	if (payload_len - 2 < amr_len_by_ft[ft])
		return -EINVAL;

	return 2 + amr_len_by_ft[ft];
}

int amr_compose_payload(uint8_t *payload, uint8_t cmr, uint8_t ft, uint8_t bfi)
{
	if (cmr >= 16)
		return -EINVAL;

	if (ft >= 16)
		return -EINVAL;

	payload[0] = cmr << 4;

	payload[1] = (ft << 3) | ((!bfi) << 2); /* F = 0 */

	return 2 + amr_len_by_ft[ft];
}

