/* Auxiliary scheduler utilities.
 *
 * (C) 2017 by Harald Welte <laforge@gnumonks.org>
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

#pragma once

#include <stdint.h>
#include <errno.h>

extern void *tall_bts_ctx;

/* Compute the bit error rate in 1/10000 units */
static inline uint16_t compute_ber10k(int n_bits_total, int n_errors)
{
	if (n_bits_total == 0)
		return 10000;
	else
		return 10000 * n_errors / n_bits_total;
}

/* determine if the FN is transmitting a CMR (1) or not (0) */
static inline int fn_is_codec_mode_request(uint32_t fn)
{
	return (((fn + 4) % 26) >> 2) & 1;
}
