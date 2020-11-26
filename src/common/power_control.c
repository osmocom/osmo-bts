/* MS Power Control Loop L1 */

/* (C) 2014 by Holger Hans Peter Freyther
 * Contributions by sysmocom - s.m.f.c. GmbH <info@sysmocom.de>
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

/* We don't want to deal with floating point, so we scale up */
#define EWMA_SCALE_FACTOR 100

/* Base Low-Pass Single-Pole IIR Filter (EWMA) formula:
 *
 *   Avg[n] = a * Pwr[n] + (1 - a) * Avg[n - 1]
 *
 * where parameter 'a' determines how much weight of the latest UL RSSI measurement
 * result 'Pwr[n]' carries vs the weight of the average 'Avg[n - 1]'.  The value of
 * 'a' is usually a float in range 0 .. 1, so:
 *
 *  - value 0.5 gives equal weight to both 'Pwr[n]' and 'Avg[n - 1]';
 *  - value 1.0 means no filtering at all (pass through);
 *  - value 0.0 makes no sense.
 *
 * Further optimization:
 *
 *   Avg[n] = a * Pwr[n] + Avg[n - 1] - a * Avg[n - 1]
 *   ^^^^^^                ^^^^^^^^^^
 *
 * a) this can be implemented in C using '+=' operator:
 *
 *   Avg += a * Pwr - a * Avg
 *   Avg += a * (Pwr - Avg)
 *
 * b) everything is scaled up by 100 to avoid floating point stuff:
 *
 *   Avg100 += A * (Pwr - Avg)
 *
 * where 'Avg100' is 'Avg * 100' and 'A' is 'a * 100'.
 *
 * For more details, see:
 *
 *   https://en.wikipedia.org/wiki/Moving_average
 *   https://en.wikipedia.org/wiki/Low-pass_filter#Simple_infinite_impulse_response_filter
 *   https://tomroelandts.com/articles/low-pass-single-pole-iir-filter
 */
static int8_t lchan_ul_pf_ewma(const struct gsm_bts *bts,
			       struct gsm_lchan *lchan,
			       const int8_t Pwr)
{
	const uint8_t A = bts->ul_power_ctrl.pf.ewma.alpha;
	int *Avg100 = &lchan->ms_power_ctrl.avg100_ul_rssi;

	/* We don't have 'Avg[n - 1]' if this is the first run */
	if (lchan->meas.res_nr == 0) {
		*Avg100 = Pwr * EWMA_SCALE_FACTOR;
		return Pwr;
	}

	*Avg100 += A * (Pwr - *Avg100 / EWMA_SCALE_FACTOR);
	return *Avg100 / EWMA_SCALE_FACTOR;
}

 /*! compute the new MS POWER LEVEL communicated to the MS and store it in lchan.
  *  \param lchan logical channel for which to compute (and in which to store) new power value.
  *  \param[in] ms_power_lvl MS Power Level received from Uplink L1 SACCH Header in SACCH block.
  *  \param[in] ul_rssi_dbm Signal level of the received SACCH block, in dBm.
  */
int lchan_ms_pwr_ctrl(struct gsm_lchan *lchan,
		      const uint8_t ms_power_lvl,
		      const int8_t ul_rssi_dbm)
{
	int diff;
	struct gsm_bts_trx *trx = lchan->ts->trx;
	struct gsm_bts *bts = trx->bts;
	enum gsm_band band = bts->band;
	int8_t new_power_lvl; /* TS 05.05 power level */
	int8_t ms_dbm, new_dbm, current_dbm, bsc_max_dbm;
	int8_t avg_ul_rssi_dbm;

	if (!trx_ms_pwr_ctrl_is_osmo(lchan->ts->trx))
		return 0;
	if (lchan->ms_power_ctrl.fixed)
		return 0;

	ms_dbm = ms_pwr_dbm(band, ms_power_lvl);
	if (ms_dbm < 0) {
		LOGPLCHAN(lchan, DLOOP, LOGL_NOTICE,
			  "Failed to calculate dBm for power ctl level %" PRIu8 " on band %s\n",
			  ms_power_lvl, gsm_band_name(band));
		return 0;
	}
	bsc_max_dbm = ms_pwr_dbm(band, lchan->ms_power_ctrl.max);
	if (bsc_max_dbm < 0) {
		LOGPLCHAN(lchan, DLOOP, LOGL_NOTICE,
			  "Failed to calculate dBm for power ctl level %" PRIu8 " on band %s\n",
			  lchan->ms_power_ctrl.max, gsm_band_name(band));
		return 0;
	}

	/* Filter UL RSSI to reduce unnecessary Tx power oscillations */
	switch (bts->ul_power_ctrl.pf_algo) {
	case MS_UL_PF_ALGO_EWMA:
		avg_ul_rssi_dbm = lchan_ul_pf_ewma(bts, lchan, ul_rssi_dbm);
		break;
	case MS_UL_PF_ALGO_NONE:
	default:
		/* No filtering (pass through) */
		avg_ul_rssi_dbm = ul_rssi_dbm;
	}

	/* How many dBs measured power should be increased (+) or decreased (-)
	   to reach expected power. */
	diff = bts->ul_power_ctrl.target - avg_ul_rssi_dbm;


	/* Tolerate small deviations from 'rx-target' */
	if (abs(diff) <= bts->ul_power_ctrl.hysteresis) {
		LOGPLCHAN(lchan, DLOOP, LOGL_INFO,
			  "Keeping MS power at control level %d (%d dBm) because diff %d dBm "
			  "from 'rx-target' %d dBm is not significant (hysteresis %d dBm)\n",
			  ms_power_lvl, ms_dbm, diff, bts->ul_power_ctrl.target, bts->ul_power_ctrl.hysteresis);
		/* Keep the current power level in sync (just to be sure) */
		lchan->ms_power_ctrl.current = ms_power_lvl;
		bts_model_adjst_ms_pwr(lchan);
		return 0;
	}

	/* don't ever change more than MS_{LOWER,RAISE}_MAX_DBM during one loop
	   iteration, i.e. reduce the speed at which the MS transmit power can
	   change. A higher value means a lower level (and vice versa) */
	if (diff > MS_RAISE_MAX_DB)
		diff = MS_RAISE_MAX_DB;
	else if (diff < -MS_LOWER_MAX_DB)
		diff = -MS_LOWER_MAX_DB;

	new_dbm = ms_dbm + diff;

	/* Make sure new_dbm is never negative. ms_pwr_ctl_lvl() can later on
	   cope with any unsigned dbm value, regardless of band minimal value. */
	if (new_dbm < 0)
		new_dbm = 0;

	/* Don't ask for smaller ms power level than the one set by BSC upon RSL CHAN ACT */
	if (new_dbm > bsc_max_dbm)
		new_dbm = bsc_max_dbm;

	new_power_lvl = ms_pwr_ctl_lvl(band, new_dbm);
	if (new_power_lvl < 0) {
		LOGPLCHAN(lchan, DLOOP, LOGL_NOTICE,
			  "Failed to retrieve power level for %" PRId8 " dBm on band %d\n",
			  new_dbm, band);
		return 0;
	}

	if (lchan->ms_power_ctrl.current == new_power_lvl) {
		LOGPLCHAN(lchan, DLOOP, LOGL_INFO, "Keeping MS power at control level %d, %d dBm "
			  "(rx-ms-pwr-lvl %" PRIu8 ", max-ms-pwr-lvl %" PRIu8 ", rx-current %d dBm, rx-target %d dBm)\n",
			  new_power_lvl, new_dbm,
			  ms_power_lvl, lchan->ms_power_ctrl.max,
			  avg_ul_rssi_dbm, bts->ul_power_ctrl.target);
		return 0;
	}

	current_dbm = ms_pwr_dbm(band, lchan->ms_power_ctrl.current);
	LOGPLCHAN(lchan, DLOOP, LOGL_INFO, "%s MS power from control level %d (%d dBm) to %d, %d dBm "
		  "(rx-ms-pwr-lvl %" PRIu8 ", max-ms-pwr-lvl %" PRIu8 ", rx-current %d dBm, rx-target %d dBm)\n",
		  (new_dbm > current_dbm) ? "Raising" : "Lowering",
		  lchan->ms_power_ctrl.current, current_dbm, new_power_lvl, new_dbm,
		  ms_power_lvl, lchan->ms_power_ctrl.max,
		  avg_ul_rssi_dbm, bts->ul_power_ctrl.target);

	/* store the resulting new MS power level in the lchan */
	lchan->ms_power_ctrl.current = new_power_lvl;
	bts_model_adjst_ms_pwr(lchan);

	return 1;
}
