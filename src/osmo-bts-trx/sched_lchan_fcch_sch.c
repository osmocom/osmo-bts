/*
 * (C) 2013 by Andreas Eversberg <jolly@eversberg.eu>
 * (C) 2015-2017 by Harald Welte <laforge@gnumonks.org>
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

#include <stdint.h>
#include <errno.h>

#include <osmocom/core/bits.h>
#include <osmocom/core/utils.h>
#include <osmocom/coding/gsm0503_coding.h>

#include <osmo-bts/bts.h>
#include <osmo-bts/l1sap.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/scheduler.h>
#include <osmo-bts/scheduler_backend.h>

#include <sched_utils.h>

/* obtain a to-be-transmitted FCCH (frequency correction channel) burst */
ubit_t *tx_fcch_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
		   enum trx_chan_type chan, uint8_t bid, uint16_t *nbits)
{
	LOGL1S(DL1P, LOGL_DEBUG, l1t, tn, chan, fn, "Transmitting FCCH\n");

	if (nbits)
		*nbits = GSM_BURST_LEN;

	/* BURST BYPASS */

	return (ubit_t *) _sched_fcch_burst;
}

/* obtain a to-be-transmitted SCH (synchronization channel) burst */
ubit_t *tx_sch_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
		  enum trx_chan_type chan, uint8_t bid, uint16_t *nbits)
{
	static ubit_t bits[GSM_BURST_LEN], burst[78];
	uint8_t sb_info[4];
	struct	gsm_time t;
	uint8_t t3p, bsic;

	LOGL1S(DL1P, LOGL_DEBUG, l1t, tn, chan, fn, "Transmitting SCH\n");

	/* BURST BYPASS */

	/* create SB info from GSM time and BSIC */
	gsm_fn2gsmtime(&t, fn);
	t3p = t.t3 / 10;
	bsic = l1t->trx->bts->bsic;
	sb_info[0] =
		((bsic &  0x3f) << 2) |
		((t.t1 & 0x600) >> 9);
	sb_info[1] =
		((t.t1 & 0x1fe) >> 1);
	sb_info[2] =
		((t.t1 & 0x001) << 7) |
		((t.t2 &  0x1f) << 2) |
		((t3p  &   0x6) >> 1);
	sb_info[3] =
		 (t3p  &   0x1);

	/* encode bursts */
	gsm0503_sch_encode(burst, sb_info);

	/* compose burst */
	memset(bits, 0, 3);
	memcpy(bits + 3, burst, 39);
	memcpy(bits + 42, _sched_sch_train, 64);
	memcpy(bits + 106, burst + 39, 39);
	memset(bits + 145, 0, 3);

	if (nbits)
		*nbits = GSM_BURST_LEN;

	return bits;
}
