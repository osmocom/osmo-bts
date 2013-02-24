/* Loop control for OsmoBTS-TRX */

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

#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>

#include <osmo-bts/gsm_data.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/l1sap.h>
#include <osmocom/core/bits.h>

#include "trx_if.h"
#include "l1_if.h"
#include "loops.h"

#define MS_PWR_DBM(lvl) ms_pwr_dbm(gsm_arfcn2band(l1h->config.arfcn), lvl)

/*
 * MS Power loop
 */

int trx_ms_power_loop = 0;
int8_t trx_target_rssi = -10;

/* how much power levels do we raise/lower as maximum (1 level = 2 dB) */
#define MS_RAISE_MAX 4
#define MS_LOWER_MAX 4

static int ms_power_diff(struct trx_l1h *l1h, struct gsm_lchan *lchan,
	uint8_t chan_nr, struct trx_chan_state *chan_state, int8_t diff)
{
	int8_t new_power;
	
	new_power = lchan->ms_power - (diff >> 1);

	if (diff == 0)
		return 0;

	if (new_power < 0)
		new_power = 0;

	// FIXME: to go above 1W, we need to know classmark of MS
	if (l1h->config.arfcn >= 512 && l1h->config.arfcn <= 885) {
		if (new_power > 15)
			new_power = 15;
	} else {
		if (new_power > 19)
			new_power = 19;
	}

	/* a higher value means a lower level (and vice versa) */
	if (new_power > lchan->ms_power + MS_LOWER_MAX)
		new_power = lchan->ms_power + MS_LOWER_MAX;
	else if (new_power < lchan->ms_power - MS_RAISE_MAX)
		new_power = lchan->ms_power - MS_RAISE_MAX;

	if (lchan->ms_power == new_power) {
		LOGP(DLOOP, LOGL_INFO, "Keeping MS new_power of trx=%u "
			"chan_nr=0x%02x at control level %d (%d dBm)\n",
			l1h->trx->nr, chan_nr, new_power,
			MS_PWR_DBM(new_power));

		return 0;
	}

	LOGP(DLOOP, LOGL_INFO, "%s MS new_power of trx=%u chan_nr=0x%02x from "
		"control level %d (%d dBm) to %d (%d dBm)\n",
		(diff > 0) ? "Raising" : "Lowering",
		l1h->trx->nr, chan_nr, lchan->ms_power,
		MS_PWR_DBM(lchan->ms_power), new_power, MS_PWR_DBM(new_power));

	lchan->ms_power = new_power;

	return 0;
}

static int ms_power_val(struct trx_l1h *l1h, struct gsm_lchan *lchan,
	uint8_t chan_nr, struct trx_chan_state *chan_state, int8_t rssi)
{
	/* ignore inserted dummy frames, treat as lost frames */
	if (rssi < -127)
		return 0;

	LOGP(DLOOP, LOGL_DEBUG, "Got RSSI value of %d\n", rssi);

	chan_state->meas.rssi_count++;

	/* check if the current L1 header compares to the current ordered TA */
//	if ((lchan->meas.l1_info[0] >> 3) != lchan->ms_power)
//		return 0;

	chan_state->meas.rssi_got_burst = 1;

	/* store and process RSSI */
	if (chan_state->meas.rssi_valid_count
					== ARRAY_SIZE(chan_state->meas.rssi))
		return 0;
	chan_state->meas.rssi[chan_state->meas.rssi_valid_count++] = rssi;
	chan_state->meas.rssi_valid_count++;

	return 0;
}

static int ms_power_clock(struct trx_l1h *l1h, struct gsm_lchan *lchan,
	uint8_t chan_nr, struct trx_chan_state *chan_state)
{
	int rssi;
	int i;

	/* skip every second clock, to prevent oscillating due to roundtrip
	 * delay */
	if (!(chan_state->meas.clock & 1))
		return 0;

	LOGP(DLOOP, LOGL_DEBUG, "Got SACCH master clock at RSSI count %d\n",
		chan_state->meas.rssi_count);

	/* wait for initial burst */
	if (!chan_state->meas.rssi_got_burst)
		return 0;

	/* if no burst was received from MS at clock */
	if (chan_state->meas.rssi_count == 0) {
		LOGP(DLOOP, LOGL_NOTICE, "LOST SACCH frame of trx=%u "
			"chan_nr=0x%02x, so we raise MS power\n",
			l1h->trx->nr, chan_nr);
		return ms_power_diff(l1h, lchan, chan_nr, chan_state,
			MS_RAISE_MAX);
	}

	/* reset total counter */
	chan_state->meas.rssi_count = 0;

	/* check the minimum level received after MS acknowledged the ordered
	 * power level */
	if (chan_state->meas.rssi_valid_count == 0)
		return 0;
	for (rssi = 999, i = 0; i < chan_state->meas.rssi_valid_count; i++) {
		if (rssi > chan_state->meas.rssi[i])
			rssi = chan_state->meas.rssi[i];
	}

	/* reset valid counter */
	chan_state->meas.rssi_valid_count = 0;

	/* change RSSI */
	LOGP(DLOOP, LOGL_DEBUG, "Lowest RSSI: %d Target RSSI: %d Current "
		"MS power: %d (%d dBm) of trx=%u chan_nr=0x%02x\n", rssi,
		trx_target_rssi, lchan->ms_power, MS_PWR_DBM(lchan->ms_power),
		l1h->trx->nr, chan_nr);
	ms_power_diff(l1h, lchan, chan_nr, chan_state, trx_target_rssi - rssi);

	return 0;
}


/*
 * Timing Advance loop
 */

int trx_ta_loop = 1;

int ta_val(struct trx_l1h *l1h, struct gsm_lchan *lchan, uint8_t chan_nr,
	struct trx_chan_state *chan_state, float toa)
{
	/* check if the current L1 header acks to the current ordered TA */
	if (lchan->meas.l1_info[1] != lchan->rqd_ta)
		return 0;

	/* sum measurement */
	chan_state->meas.toa_sum += toa;
	if (++(chan_state->meas.toa_num) < 16)
		return 0;

	/* complete set */
	toa = chan_state->meas.toa_sum / chan_state->meas.toa_num;

	/* check for change of TOA */
	if (toa < -0.9F && lchan->rqd_ta > 0) {
		LOGP(DLOOP, LOGL_INFO, "TOA of trx=%u chan_nr=0x%02x is too "
			"early (%.2f), now lowering TA from %d to %d\n",
			l1h->trx->nr, chan_nr, toa, lchan->rqd_ta,
			lchan->rqd_ta - 1);
		lchan->rqd_ta--;
	} else if (toa > 0.9F && lchan->rqd_ta < 63) {
		LOGP(DLOOP, LOGL_INFO, "TOA of trx=%u chan_nr=0x%02x is too "
			"late (%.2f), now raising TA from %d to %d\n",
			l1h->trx->nr, chan_nr, toa, lchan->rqd_ta,
			lchan->rqd_ta + 1);
		lchan->rqd_ta++;
	} else
		LOGP(DLOOP, LOGL_INFO, "TOA of trx=%u chan_nr=0x%02x is "
			"correct (%.2f), keeping current TA of %d\n",
			l1h->trx->nr, chan_nr, toa, lchan->rqd_ta);

	chan_state->meas.toa_num = 0;
	chan_state->meas.toa_sum = 0;

	return 0;
}

int trx_loop_input(struct trx_l1h *l1h, uint8_t chan_nr,
	struct trx_chan_state *chan_state, int8_t rssi, float toa)
{
	struct gsm_lchan *lchan = &l1h->trx->ts[L1SAP_CHAN2TS(chan_nr)]
					.lchan[l1sap_chan2ss(chan_nr)];

	if (trx_ms_power_loop)
		ms_power_val(l1h, lchan, chan_nr, chan_state, rssi);

	if (trx_ta_loop)
		ta_val(l1h, lchan, chan_nr, chan_state, toa);

	return 0;
}

int trx_loop_sacch_clock(struct trx_l1h *l1h, uint8_t chan_nr,
	struct trx_chan_state *chan_state)
{
	struct gsm_lchan *lchan = &l1h->trx->ts[L1SAP_CHAN2TS(chan_nr)]
					.lchan[l1sap_chan2ss(chan_nr)];

	if (trx_ms_power_loop)
		ms_power_clock(l1h, lchan, chan_nr, chan_state);

	/* count the number of SACCH clocks */
	chan_state->meas.clock++;

	return 0;
}

