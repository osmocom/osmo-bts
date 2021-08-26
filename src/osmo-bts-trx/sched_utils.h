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
#include <stdbool.h>
#include <osmo-bts/scheduler.h>

extern void *tall_bts_ctx;

/* Compute the bit error rate in 1/10000 units */
static inline uint16_t compute_ber10k(int n_bits_total, int n_errors)
{
	if (n_bits_total == 0)
		return 10000;
	else
		return 10000 * n_errors / n_bits_total;
}

/*! determine whether an uplink AMR block is CMI according to 3GPP TS 45.009.
 *  \param[in] fn_begin frame number of the beginning of the block.
 *  \returns true in case of CMI; false otherwise. */
static inline bool ul_amr_fn_is_cmi(uint32_t fn_begin)
{
	switch (fn_begin % 26) {
		/*! See also: 3GPP TS 45.009, section 3.2.1.3 Transmitter/Receiver Synchronisation */
		/* valid for AHS subslot 0 and AFS: */
	case 0:
	case 8:
	case 17:
		/* valid for AHS subslot 1: */
	case 1:
	case 9:
	case 18:
		return true;
		break;
		/* Complementary values for sanity check */
		/* valid for AHS subslot 0 and AFS: */
	case 4:
	case 13:
	case 21:
		/* valid for AHS subslot 1: */
	case 5:
	case 14:
	case 22:
		return false;
		break;
	default:
		LOGP(DL1P, LOGL_DEBUG,
		     "uplink frame number fn_begin=%u does not mark the beginning of a voice block!\n", fn_begin);
		OSMO_ASSERT(false);
		return false;
		break;
	}
}

/*! determine the whether a downlink AMR block is CMI according to 3GPP TS 45.009.
 *  \param[in] fn_begin frame number of the beginning of the block.
 *  \returns true in case of CMI; false otherwise. */
static inline bool dl_amr_fn_is_cmi(uint32_t fn_begin)
{
	switch (fn_begin % 26) {
		/*! See also: 3GPP TS 45.009, section 3.2.1.3 Transmitter/Receiver Synchronisation */
		/* valid for AHS subslot 0 and AFS: */
	case 4:
	case 13:
	case 21:
		/* valid for AHS subslot 1: */
	case 5:
	case 14:
	case 22:
		return true;
		break;
		/* Complementary values for sanity check */
		/* valid for AHS subslot 0 and AFS: */
	case 0:
	case 8:
	case 17:
		/* valid for AHS subslot 1: */
	case 1:
	case 9:
	case 18:
		return false;
		break;
	default:
		LOGP(DL1P, LOGL_DEBUG,
		     "downlink frame number fn_begin=%u does not mark the beginning of a voice block!\n", fn_begin);
		OSMO_ASSERT(false);
		return false;
		break;
	}
}
