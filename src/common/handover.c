/* Paging message encoding + queue management */

/* (C) 2012-2013 by Harald Welte <laforge@gnumonks.org>
 *                  Andreas Eversberg <jolly@eversberg.eu>
 * (C) 2014 by Holger Hans Peter Freyther
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

#include <stdlib.h>
#include <stdint.h>
#include <errno.h>

#include <osmocom/gsm/protocol/gsm_04_08.h>
#include <osmocom/gsm/rsl.h>

#include <osmo-bts/bts.h>
#include <osmo-bts/bts_model.h>
#include <osmo-bts/rsl.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/handover.h>

/* release handover state */
void handover_reset(struct gsm_lchan *lchan)
{
	/* Stop T3105 */
	osmo_timer_del(&lchan->ho.t3105);

	/* Handover process is done */
	lchan->ho.active = HANDOVER_NONE;
}
