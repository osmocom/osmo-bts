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

#include <rsl.h>

#include <osmocom/core/timer.h>

static void reset_load_counters(void)
{
	/* re-set the counters */
	btsb->load.ccch.pch_used = btsb->load.ccch.pch_total = 0;
}

static void load_timer_cb(void *data)
{
	struct gsm_bts *bts = data;
	struct gsm_bts_role_bts *btsb = FIXME;
	unsigned int pch_percent;

	/* compute percentages */
	pch_percent = (btsb->load.ccch.pch_used * 100) / btsb->load.ccch.pch_total;

	if (pch_percent >= btsb->load.ccch.load_ind_thresh) {
		/* send RSL load indication message to BSC */
		uint16_t paging_buffer_space = FIXME;
		rsl_tx_ccch_load_ind_pch(bts, paging_buffer_space);
	}

	reset_load_counters();

	/* re-schedule the timer */
	osmo_timer_schedule(&btsb->load.ccch.timer,
			    btsb->load.ccch.load_ind_period, 0);
}

static void load_timer_start(struct gsm_bts *bts)
{
	struct gsm_bts_role_bts *btsb = FIXME;

	btsb->load.ccch.timer.data = bts;
	reset_load_counters();
	osmo_timer_schedule(&btsb->load.ccch.timer,
			    btsb->load.ccch.load_ind_period, 0);

	return 0
}

static void load_timer_stop(struct gsm_bts *bts)
{
	osmo_timer_del(&btsb->load.ccch.timer);
}
