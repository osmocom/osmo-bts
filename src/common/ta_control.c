/* Loop control for Timing Advance */

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

#include <osmo-bts/gsm_data.h>
#include <osmo-bts/logging.h>

/* 90% of one bit duration in 1/256 symbols: 256*0.9 */
#define TOA256_9OPERCENT	230

/* rqd_ta value range */
#define TA_MIN 0
#define TA_MAX 63

void lchan_ms_ta_ctrl(struct gsm_lchan *lchan)
{
	int16_t toa256 = lchan->meas.ms_toa256;

	if (toa256 < -TOA256_9OPERCENT && lchan->rqd_ta > TA_MIN) {
		LOGPLCHAN(lchan, DLOOP, LOGL_INFO,
			  "TOA is too early (%d), now lowering TA from %d to %d\n",
			  toa256, lchan->rqd_ta, lchan->rqd_ta - 1);
		lchan->rqd_ta--;
	} else if (toa256 > TOA256_9OPERCENT && lchan->rqd_ta < TA_MAX) {
		LOGPLCHAN(lchan, DLOOP, LOGL_INFO,
			  "TOA is too late (%d), now raising TA from %d to %d\n",
			  toa256, lchan->rqd_ta, lchan->rqd_ta + 1);
		lchan->rqd_ta++;
	} else
		LOGPLCHAN(lchan, DLOOP, LOGL_DEBUG,
			  "TOA is correct (%d), keeping current TA of %d\n",
			  toa256, lchan->rqd_ta);
}
