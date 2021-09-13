/* Loop control for Timing Advance */

/* (C) 2013 by Andreas Eversberg <jolly@eversberg.eu>
 * (C) 2021 by sysmocom - s.m.f.c. GmbH <info@sysmocom.de>
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

/* Related specs: 3GPP TS 45.010 sections 5.5, 5.6 */

#include <osmo-bts/gsm_data.h>
#include <osmo-bts/logging.h>

/* 3GPP TS 45.010 sec 5.6.3 Delay assessment error:
 * 75% of one bit duration in 1/256 symbols: 256*0.75 */
#define TOA256_THRESH	192

/* rqd_ta value range */
#define TA_MIN 0
#define TA_MAX 63

/* TODO: make configurable over osmo-bts VTY? Pass it BSC->BTS? */
#define TA_MAX_INC_STEP 2
#define TA_MAX_DEC_STEP 2


/*! compute the new "Ordered Timing Advance" communicated to the MS and store it in lchan.
 * \param lchan logical channel for which to compute (and in which to store) new power value.
 * \param[in] ms_tx_ta The TA used by the MS and reported in L1SACCH, see struct gsm_sacch_l1_hdr field "ta".
 * \param[in] toa256 Time of Arrival (in 1/256th bits) computed at Rx side
 */
void lchan_ms_ta_ctrl(struct gsm_lchan *lchan, uint8_t ms_tx_ta, int16_t toa256)
{
	int16_t new_ta;
	/* Shall we skip current block based on configured interval? */

	/*TODO: implement P_CON_INTERVAL for TA loop */

	int16_t delta_ta = toa256/256;
	if (toa256 >= 0) {
		if ((toa256 - (256 * delta_ta)) > TOA256_THRESH)
			delta_ta++;
		if (delta_ta > TA_MAX_INC_STEP)
			delta_ta = TA_MAX_INC_STEP;
	} else {
		if ((toa256 - (256 * delta_ta)) < -TOA256_THRESH)
			delta_ta--;
		if (delta_ta < -TA_MAX_DEC_STEP)
			delta_ta = -TA_MAX_DEC_STEP;
	}

	new_ta = ms_tx_ta + delta_ta;

	/* Make sure new_ta is never negative: */
	if (new_ta < TA_MIN)
		new_ta = TA_MIN;

	/* Don't ask for out of range TA: */
	if (new_ta > TA_MAX)
		new_ta = TA_MAX;

	if (lchan->ta_ctrl.current == (uint8_t)new_ta) {
		LOGPLCHAN(lchan, DLOOP, LOGL_DEBUG,
			  "Keeping current TA at %u: TOA was %d\n",
			  lchan->ta_ctrl.current, toa256);
		return;
	}

	LOGPLCHAN(lchan, DLOOP, LOGL_INFO,
		  "%s TA %u => %u: TOA was too %s (%d)\n",
		  (uint8_t)new_ta > lchan->ta_ctrl.current ? "Raising" : "Lowering",
		  lchan->ta_ctrl.current, (uint8_t)new_ta,
		  (uint8_t)new_ta > lchan->ta_ctrl.current ? "late" : "early",
		  toa256);

	/* store the resulting new TA in the lchan */
	lchan->ta_ctrl.current = (uint8_t)new_ta;
}
