/* MS Power Control Loop L1 */

/* (C) 2014 by Holger Hans Peter Freyther
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
#include <unistd.h>
#include <errno.h>
#include <inttypes.h>

#include <osmo-bts/logging.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/gsm_data.h>
#include <osmo-bts/measurement.h>
#include <osmo-bts/bts_model.h>
#include <osmo-bts/l1sap.h>
#include <osmo-bts/power_control.h>

/* how many dB do we raise/lower as maximum (1 ms power level = 2 dB) */
#define MS_RAISE_MAX_DB 4
#define MS_LOWER_MAX_DB 8

 /*! compute the new MS POWER LEVEL communicated to the MS and store it in lchan.
  *  \param lchan logical channel for which to compute (and in which to store) new power value.
  *  \param[in] ms_power MS Power Level received from Uplink L1 SACCH Header in SACCH block.
  *  \param[in] rxLevel Signal level of the received SACCH block, in dBm.
  */
int lchan_ms_pwr_ctrl(struct gsm_lchan *lchan,
		      const uint8_t ms_power, const int rxLevel)
{
	int diff;
	struct gsm_bts_trx *trx = lchan->ts->trx;
	struct gsm_bts *bts = trx->bts;
	enum gsm_band band = bts->band;
	int8_t new_power; /* TS 05.05 power level */
	int8_t new_dbm, current_dbm, bsc_max_dbm;

	if (!trx_ms_pwr_ctrl_is_osmo(lchan->ts->trx))
		return 0;
	if (lchan->ms_power_ctrl.fixed)
		return 0;

	current_dbm = ms_pwr_dbm(band, ms_power);
	if (current_dbm < 0) {
		LOGPLCHAN(lchan, DLOOP, LOGL_NOTICE,
			  "Failed to calculate dBm for power ctl level %" PRIu8 " on band %s\n",
			  ms_power, gsm_band_name(band));
		return 0;
	}
	bsc_max_dbm = ms_pwr_dbm(band, lchan->ms_power_ctrl.max);
	if (bsc_max_dbm < 0) {
		LOGPLCHAN(lchan, DLOOP, LOGL_NOTICE,
			  "Failed to calculate dBm for power ctl level %" PRIu8 " on band %s\n",
			  lchan->ms_power_ctrl.max, gsm_band_name(band));
		return 0;
	}

	/* How many dBs measured power should be increased (+) or decreased (-)
	   to reach expected power. */
	diff = bts->ul_power_target - rxLevel;

	/* don't ever change more than MS_{LOWER,RAISE}_MAX_DBM during one loop
	   iteration, i.e. reduce the speed at which the MS transmit power can
	   change. A higher value means a lower level (and vice versa) */
	if (diff > MS_RAISE_MAX_DB)
		diff = MS_RAISE_MAX_DB;
	else if (diff < -MS_LOWER_MAX_DB)
		diff = -MS_LOWER_MAX_DB;

	new_dbm = current_dbm + diff;

	/* Make sure new_dbm is never negative. ms_pwr_ctl_lvl() can later on
	   cope with any unsigned dbm value, regardless of band minimal value. */
	if (new_dbm < 0)
		new_dbm = 0;

	/* Don't ask for smaller ms power level than the one set by BSC upon RSL CHAN ACT */
	if (new_dbm > bsc_max_dbm)
		new_dbm = bsc_max_dbm;

	new_power = ms_pwr_ctl_lvl(band, new_dbm);
	if (new_power < 0) {
		LOGPLCHAN(lchan, DLOOP, LOGL_NOTICE,
			  "Failed to retrieve power level for %" PRId8 " dBm on band %d\n",
			  new_dbm, band);
		return 0;
	}

	if (lchan->ms_power_ctrl.current == new_power) {
		LOGPLCHAN(lchan, DLOOP, LOGL_INFO, "Keeping MS power at control level %d, %d dBm "
			  "(rx-ms-pwr-lvl %" PRIu8 ", rx-current %d dBm, rx-target %d dBm)\n",
			new_power, ms_pwr_dbm(band, new_power), ms_power, rxLevel, bts->ul_power_target);
		return 0;
	}

	LOGPLCHAN(lchan, DLOOP, LOGL_INFO, "%s MS power from control level %d (%d dBm) to %d, %d dBm "
		  "(rx-ms-pwr-lvl %" PRIu8 ", rx-current %d dBm, rx-target %d dBm)\n",
		(diff > 0) ? "Raising" : "Lowering",
		lchan->ms_power_ctrl.current, ms_pwr_dbm(band, lchan->ms_power_ctrl.current),
		new_power, ms_pwr_dbm(band, new_power), ms_power, rxLevel, bts->ul_power_target);

	/* store the resulting new MS power level in the lchan */
	lchan->ms_power_ctrl.current = new_power;
	bts_model_adjst_ms_pwr(lchan);

	return 1;
}
