/* Support for generating RSL Load Indication */

/* (C) 2011 by Harald Welte <laforge@gnumonks.org>
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

#include <osmocom/core/timer.h>
#include <osmocom/core/msgb.h>

#include <osmo-bts/gsm_data.h>
#include <osmo-bts/rsl.h>
#include <osmo-bts/paging.h>

static void reset_load_counters(struct gsm_bts *bts)
{
	struct gsm_bts_role_bts *btsb = bts_role_bts(bts);

	/* re-set the counters */
	btsb->load.ccch.pch_used = btsb->load.ccch.pch_total = 0;
}

static void load_timer_cb(void *data)
{
	struct gsm_bts *bts = data;
	struct gsm_bts_role_bts *btsb = bts_role_bts(bts);
	unsigned int pch_percent, rach_percent;

	/* compute percentages */
	pch_percent = (btsb->load.ccch.pch_used * 100) / btsb->load.ccch.pch_total;

	if (pch_percent >= btsb->load.ccch.load_ind_thresh) {
		/* send RSL load indication message to BSC */
		uint16_t buffer_space = paging_buffer_space(btsb->paging_state);
		rsl_tx_ccch_load_ind_pch(bts, buffer_space);
	}

	rach_percent = (btsb->load.rach.busy * 100) / btsb->load.rach.total;
	if (rach_percent >= btsb->load.ccch.load_ind_thresh) {
		/* send RSL load indication message to BSC */
		rsl_tx_ccch_load_ind_rach(bts, btsb->load.rach.total,
					  btsb->load.rach.busy,
					  btsb->load.rach.access);
	}

	reset_load_counters(bts);

	/* re-schedule the timer */
	osmo_timer_schedule(&btsb->load.ccch.timer,
			    btsb->load.ccch.load_ind_period, 0);
}

void load_timer_start(struct gsm_bts *bts)
{
	struct gsm_bts_role_bts *btsb = bts_role_bts(bts);

	btsb->load.ccch.timer.data = bts;
	btsb->load.ccch.timer.cb = load_timer_cb;
	reset_load_counters(bts);
	osmo_timer_schedule(&btsb->load.ccch.timer,
			    btsb->load.ccch.load_ind_period, 0);
}

void load_timer_stop(struct gsm_bts *bts)
{
	struct gsm_bts_role_bts *btsb = bts_role_bts(bts);

	osmo_timer_del(&btsb->load.ccch.timer);
}
