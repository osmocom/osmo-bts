/*
 * (C) 2013 by Andreas Eversberg <jolly@eversberg.eu>
 * (C) 2015-2017 by Harald Welte <laforge@gnumonks.org>
 * Contributions by sysmocom - s.f.m.c. GmbH
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
int tx_fcch_fn(struct l1sched_ts *l1ts, struct trx_dl_burst_req *br)
{
	LOGL1SB(DL1P, LOGL_DEBUG, l1ts, br, "Transmitting FCCH\n");

	/* A frequency correction burst is basically a sequence of zeros */
	memset(br->burst, 0x00, GSM_BURST_LEN);
	br->burst_len = GSM_BURST_LEN;

	return 0;
}

/* obtain a to-be-transmitted SCH (synchronization channel) burst */
int tx_sch_fn(struct l1sched_ts *l1ts, struct trx_dl_burst_req *br)
{
	ubit_t burst[78];
	uint8_t sb_info[4];
	struct	gsm_time t;
	uint8_t t3p, bsic;

	LOGL1SB(DL1P, LOGL_DEBUG, l1ts, br, "Transmitting SCH\n");

	/* BURST BYPASS */

	/* create SB info from GSM time and BSIC */
	gsm_fn2gsmtime(&t, br->fn);
	t3p = t.t3 / 10;
	bsic = l1ts->ts->trx->bts->bsic;
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
	memcpy(br->burst + 3, burst, 39);
	memcpy(br->burst + 42, _sched_train_seq_gmsk_sb, 64);
	memcpy(br->burst + 106, burst + 39, 39);

	br->burst_len = GSM_BURST_LEN;

	return 0;
}
