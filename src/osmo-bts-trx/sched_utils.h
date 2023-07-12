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

/* Burst Payload LENgth (short alias) */
#define BPLEN GSM_NBITS_NB_GMSK_PAYLOAD

/* Burst BUFfer capacity (in BPLEN units) */
#define BUFMAX 24

/* Burst BUFfer position macros */
#define BUFPOS(buf, n) &buf[(n) * BPLEN]
#define BUFTAIL8(buf) BUFPOS(buf, (BUFMAX - 8))

extern void *tall_bts_ctx;

#define BAD_DATA_MSG_FMT "Received bad data (rc=%d, BER %d/%d) ending at fn=%u/%u"
#define BAD_DATA_MSG_ARGS \
	rc, n_errors, n_bits_total, bi->fn % l1ts->mf_period, l1ts->mf_period

/* Compute the bit error rate in 1/10000 units */
static inline uint16_t compute_ber10k(int n_bits_total, int n_errors)
{
	if (n_bits_total == 0)
		return 10000;
	else
		return 10000 * n_errors / n_bits_total;
}
