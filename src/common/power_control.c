/* MS Power Control Loop L1 */

/* (C) 2014 by Holger Hans Peter Freyther
 * (C) 2020-2021 by sysmocom - s.m.f.c. GmbH <info@sysmocom.de>
 * Author: Vadim Yanitskiy <vyanitskiy@sysmocom.de>
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

/* We don't want to deal with floating point, so we scale up */
#define EWMA_SCALE_FACTOR 100
/* EWMA_SCALE_FACTOR/2 = +50: Round to nearest value when downscaling, otherwise floor() is applied. */
#define EWMA_ROUND_FACTOR (EWMA_SCALE_FACTOR / 2)

/* Base Low-Pass Single-Pole IIR Filter (EWMA) formula:
 *
 *   Avg[n] = a * Val[n] + (1 - a) * Avg[n - 1]
 *
 * where parameter 'a' determines how much weight of the latest measurement value
 * 'Val[n]' carries vs the weight of the accumulated average 'Avg[n - 1]'.  The
 * value of 'a' is usually a float in range 0 .. 1, so:
 *
 *  - value 0.5 gives equal weight to both 'Val[n]' and 'Avg[n - 1]';
 *  - value 1.0 means no filtering at all (pass through);
 *  - value 0.0 makes no sense.
 *
 * Further optimization:
 *
 *   Avg[n] = a * Val[n] + Avg[n - 1] - a * Avg[n - 1]
 *   ^^^^^^                ^^^^^^^^^^
 *
 * a) this can be implemented in C using '+=' operator:
 *
 *   Avg += a * Val - a * Avg
 *   Avg += a * (Val - Avg)
 *
 * b) everything is scaled up by 100 to avoid floating point stuff:
 *
 *   Avg100 += A * (Val - Avg)
 *
 * where 'Avg100' is 'Avg * 100' and 'A' is 'a * 100'.
 *
 * For more details, see:
 *
 *   https://en.wikipedia.org/wiki/Moving_average
 *   https://en.wikipedia.org/wiki/Low-pass_filter#Simple_infinite_impulse_response_filter
 *   https://tomroelandts.com/articles/low-pass-single-pole-iir-filter
 */
static int do_pf_ewma(const struct gsm_power_ctrl_meas_params *mp,
		      struct gsm_power_ctrl_meas_proc_state *mps,
		      const int Val)
{
	const uint8_t A = mp->ewma.alpha;
	int *Avg100 = &mps->ewma.Avg100;

	/* We don't have 'Avg[n - 1]' if this is the first run */
	if (mps->meas_num++ == 0) {
		*Avg100 = Val * EWMA_SCALE_FACTOR;
		return Val;
	}

	*Avg100 += A * (Val - (*Avg100 + EWMA_ROUND_FACTOR) / EWMA_SCALE_FACTOR);
	return (*Avg100 + EWMA_ROUND_FACTOR) / EWMA_SCALE_FACTOR;
}

/* Calculate target RxLev value from lower/upper thresholds */
#define CALC_TARGET(mp) \
	((mp).lower_thresh + (mp).upper_thresh) / 2

static int do_avg_algo(const struct gsm_power_ctrl_meas_params *mp,
		       struct gsm_power_ctrl_meas_proc_state *mps,
		       const int val)
{
	int val_avg;
	switch (mp->algo) {
	case GSM_PWR_CTRL_MEAS_AVG_ALGO_OSMO_EWMA:
		val_avg = do_pf_ewma(mp, mps, val);
		break;
	/* TODO: implement other pre-processing methods */
	case GSM_PWR_CTRL_MEAS_AVG_ALGO_NONE:
	default:
		/* No filtering (pass through) */
		val_avg = val;
	}
	return val_avg;
}
/* Calculate a 'delta' value (for the given MS/BS power control parameters)
 * to be applied to the current Tx power level to approach the target level. */
static int calc_delta_rxlev(const struct gsm_power_ctrl_params *params, const uint8_t rxlev)
{
	int delta;

	/* Check if RxLev is within the threshold window */
	if (rxlev >= params->rxlev_meas.lower_thresh &&
	    rxlev <= params->rxlev_meas.upper_thresh)
		return 0;

	/* How many dBs measured power should be increased (+) or decreased (-)
	 * to reach expected power. */
	delta = CALC_TARGET(params->rxlev_meas) - rxlev;

	/* Don't ever change more than PWR_{LOWER,RAISE}_MAX_DBM during one loop
	 * iteration, i.e. reduce the speed at which the MS transmit power can
	 * change. A higher value means a lower level (and vice versa) */
	if (delta > params->inc_step_size_db)
		delta = params->inc_step_size_db;
	else if (delta < -params->red_step_size_db)
		delta = -params->red_step_size_db;

	return delta;
}

/* Shall we skip current block based on configured interval? */
static bool ctrl_interval_skip_block(const struct gsm_power_ctrl_params *params,
				     struct lchan_power_ctrl_state *state)
{
	/* Power control interval: how many blocks do we skip? */
	if (state->skip_block_num-- > 0)
		return true;

	/* Reset the number of SACCH blocks to be skipped:
	 *   ctrl_interval=0 => 0 blocks to skip,
	 *   ctrl_interval=1 => 1 blocks to skip,
	 *   ctrl_interval=2 => 3 blocks to skip,
	 *     so basically ctrl_interval * 2 - 1. */
	state->skip_block_num = params->ctrl_interval * 2 - 1;
	return false;
}

static const struct gsm_power_ctrl_meas_params *lchan_get_ci_thresholds(const struct gsm_lchan *lchan)
{
	const struct gsm_power_ctrl_params *params = lchan->ms_power_ctrl.dpc_params;

	switch (lchan->type) {
	case GSM_LCHAN_SDCCH:
		return &params->ci_sdcch_meas;
	case GSM_LCHAN_PDTCH:
		return &params->ci_gprs_meas;
	case GSM_LCHAN_TCH_F:
		if (lchan->tch_mode == GSM48_CMODE_SPEECH_AMR)
			return &params->ci_amr_fr_meas;
		else
			return &params->ci_fr_meas;
	case GSM_LCHAN_TCH_H:
		if (lchan->tch_mode == GSM48_CMODE_SPEECH_AMR)
			return &params->ci_amr_hr_meas;
		else
			return &params->ci_hr_meas;
	default:
		OSMO_ASSERT(0);
	}
}

/*! compute the new MS POWER LEVEL communicated to the MS and store it in lchan.
 *  \param lchan logical channel for which to compute (and in which to store) new power value.
 *  \param[in] ms_power_lvl MS Power Level received from Uplink L1 SACCH Header in SACCH block.
 *  \param[in] ul_rssi_dbm Signal level of the received SACCH block, in dBm.
 *  \param[in] ul_lqual_cb C/I of the received SACCH block, in dB.
 */
int lchan_ms_pwr_ctrl(struct gsm_lchan *lchan,
		      const uint8_t ms_power_lvl,
		      const int8_t ul_rssi_dbm,
		      const int16_t ul_lqual_cb)
{
	struct lchan_power_ctrl_state *state = &lchan->ms_power_ctrl;
	const struct gsm_power_ctrl_params *params = state->dpc_params;
	struct gsm_bts_trx *trx = lchan->ts->trx;
	struct gsm_bts *bts = trx->bts;
	enum gsm_band band = bts->band;
	int8_t new_power_lvl; /* TS 05.05 power level */
	int8_t ms_dbm, new_dbm, current_dbm, bsc_max_dbm;
	uint8_t rxlev_avg;
	int16_t ul_lqual_cb_avg;
	const struct gsm_power_ctrl_meas_params *ci_meas;
	bool ignore;

	if (!trx_ms_pwr_ctrl_is_osmo(trx))
		return 0;
	if (params == NULL)
		return 0;

	/* Shall we skip current block based on configured interval? */
	if (ctrl_interval_skip_block(params, state))
		return 0;

	ms_dbm = ms_pwr_dbm(band, ms_power_lvl);
	if (ms_dbm < 0) {
		LOGPLCHAN(lchan, DLOOP, LOGL_NOTICE,
			  "Failed to calculate dBm for power ctl level %" PRIu8 " on band %s\n",
			  ms_power_lvl, gsm_band_name(band));
		return 0;
	}
	bsc_max_dbm = ms_pwr_dbm(band, state->max);
	if (bsc_max_dbm < 0) {
		LOGPLCHAN(lchan, DLOOP, LOGL_NOTICE,
			  "Failed to calculate dBm for power ctl level %" PRIu8 " on band %s\n",
			  state->max, gsm_band_name(band));
		return 0;
	}

	/* If computed C/I is out of acceptable thresholds: */
	ci_meas = lchan_get_ci_thresholds(lchan);
	ul_lqual_cb_avg = do_avg_algo(ci_meas, &state->ci_meas_proc, ul_lqual_cb);
	rxlev_avg = do_avg_algo(&params->rxlev_meas, &state->rxlev_meas_proc, dbm2rxlev(ul_rssi_dbm));
	if (ul_lqual_cb_avg < ci_meas->lower_thresh * 10) {
		new_dbm = ms_dbm + params->inc_step_size_db;
	} else if (ul_lqual_cb_avg > ci_meas->upper_thresh * 10) {
		new_dbm = ms_dbm - params->red_step_size_db;
	} else {
		/* Calculate the new Tx power value (in dBm) */
		new_dbm = ms_dbm + calc_delta_rxlev(params, rxlev_avg);
	}

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

	current_dbm = ms_pwr_dbm(band, state->current);

	/* In this Power Control Loop, we infer a new good MS Power Level based
	 * on the previous MS Power Level announced by the MS (not the previous
	 * one we requested!) together with the related computed measurements.
	 * Hence, and since we allow for several good MS Power Levels falling into our
	 * thresholds, we could finally converge into an oscillation loop where
	 * the MS bounces between 2 different correct MS Power levels all the
	 * time, due to the fact that we "accept" and "request back" whatever
	 * good MS Power Level we received from the MS, but at that time the MS
	 * will be transmitting using the previous MS Power Level we
	 * requested, which we will later "accept" and "request back" on next loop
	 * iteration. As a result MS effectively bounces between those 2 MS
	 * Power Levels.
	 * In order to fix this permanent oscillation, if current MS_PWR used/announced
	 * by MS is good ("ms_dbm == new_dbm", hence within thresholds and no change
	 * required) but has higher Tx power than the one we last requested, we ignore
	 * it and keep requesting for one with lower Tx power. This way we converge to
	 * the lowest good Tx power avoiding oscillating over values within thresholds.
	 */
	ignore = (ms_dbm == new_dbm && ms_dbm > current_dbm);

	if (state->current == new_power_lvl || ignore) {
		LOGPLCHAN(lchan, DLOOP, LOGL_INFO, "Keeping MS power at control level %d (%d dBm): "
			  "ms-pwr-lvl[curr %" PRIu8 ", max %" PRIu8 "], RSSI[curr %d, avg %d, thresh %d..%d] dBm,"
			  " C/I[curr %d, avg %d, thresh %d..%d] dB\n",
			  new_power_lvl, new_dbm, ms_power_lvl, state->max, ul_rssi_dbm, rxlev2dbm(rxlev_avg),
			  rxlev2dbm(params->rxlev_meas.lower_thresh), rxlev2dbm(params->rxlev_meas.upper_thresh),
			  ul_lqual_cb/10, ul_lqual_cb_avg/10, ci_meas->lower_thresh, ci_meas->upper_thresh);
		return 0;
	}

	LOGPLCHAN(lchan, DLOOP, LOGL_INFO, "%s MS power control level %d (%d dBm) => %d (%d dBm): "
		  "ms-pwr-lvl[curr %" PRIu8 ", max %" PRIu8 "], RSSI[curr %d, avg %d, thresh %d..%d] dBm,"
		  " C/I[curr %d, avg %d, thresh %d..%d] dB\n",
		  (new_dbm > current_dbm) ? "Raising" : "Lowering",
		  state->current, current_dbm, new_power_lvl, new_dbm, ms_power_lvl,
		  state->max, ul_rssi_dbm, rxlev2dbm(rxlev_avg),
		  rxlev2dbm(params->rxlev_meas.lower_thresh), rxlev2dbm(params->rxlev_meas.upper_thresh),
		  ul_lqual_cb/10, ul_lqual_cb_avg/10, ci_meas->lower_thresh, ci_meas->upper_thresh);

	/* store the resulting new MS power level in the lchan */
	state->current = new_power_lvl;
	bts_model_adjst_ms_pwr(lchan);

	return 1;
}

/*! compute the new Downlink attenuation value for the given logical channel.
 *  \param lchan logical channel for which to compute (and in which to store) new power value.
 *  \param[in] gh pointer to the beginning of (presumably) a Measurement Report.
 */
int lchan_bs_pwr_ctrl(struct gsm_lchan *lchan,
		      const struct gsm48_hdr *gh)
{
	struct lchan_power_ctrl_state *state = &lchan->bs_power_ctrl;
	const struct gsm_power_ctrl_params *params = state->dpc_params;
	uint8_t rxqual_full, rxqual_sub;
	uint8_t rxlev_full, rxlev_sub;
	uint8_t rxqual, rxqual_avg, rxlev, rxlev_avg;
	int new_att;

	/* Check if dynamic BS Power Control is enabled */
	if (params == NULL)
		return 0;
	/* Check if this is a Measurement Report */
	if (gh->proto_discr != GSM48_PDISC_RR)
		return 0;
	if (gh->msg_type != GSM48_MT_RR_MEAS_REP)
		return 0;

	/* Check if the measurement results are valid */
	if ((gh->data[1] & 0x40) == 0x40) {
		LOGPLCHAN(lchan, DLOOP, LOGL_DEBUG,
			  "The measurement results are not valid\n");
		return 0;
	}

	/* See 3GPP TS 44.018, section 10.5.2.20 */
	rxqual_full = (gh->data[2] >> 4) & 0x7;
	rxqual_sub = (gh->data[2] >> 1) & 0x7;

	rxlev_full = gh->data[0] & 0x3f;
	rxlev_sub = gh->data[1] & 0x3f;

	LOGPLCHAN(lchan, DLOOP, LOGL_DEBUG, "Rx DL Measurement Report: "
		  "RXLEV-FULL(%02u), RXQUAL-FULL(%u), "
		  "RXLEV-SUB(%02u), RXQUAL-SUB(%u), "
		  "DTx is %s => using %s\n",
		  rxlev_full, rxqual_full, rxlev_sub, rxqual_sub,
		  lchan->tch.dtx.dl_active ? "enabled" : "disabled",
		  lchan->tch.dtx.dl_active ? "SUB" : "FULL");

	/* Shall we skip current block based on configured interval? */
	if (ctrl_interval_skip_block(params, state))
		return 0;

	/* If DTx is active on Downlink, use the '-SUB' */
	if (lchan->tch.dtx.dl_active) {
		rxqual = rxqual_sub;
		rxlev = rxlev_sub;
	} else { /* ... otherwise use the '-FULL' */
		rxqual = rxqual_full;
		rxlev = rxlev_full;
	}

	rxlev_avg = do_avg_algo(&params->rxlev_meas, &state->rxlev_meas_proc, rxlev);
	rxqual_avg = do_avg_algo(&params->rxqual_meas, &state->rxqual_meas_proc, rxqual);
	/* If RxQual > L_RXQUAL_XX_P, try to increase Tx power */
	if (rxqual_avg > params->rxqual_meas.lower_thresh) {
		/* Increase Tx power by reducing Tx attenuation */
		new_att = state->current - params->inc_step_size_db;
	} else if (rxqual_avg < params->rxqual_meas.upper_thresh) {
		/* Increase Tx power by Increasing Tx attenuation */
		new_att = state->current + params->red_step_size_db;
	} else {
		/* Basic signal transmission / reception formula:
		 *
		 *   RxLev = TxPwr - (PathLoss + TxAtt)
		 *
		 * Here we want to change RxLev at the MS side, so:
		 *
		 *   RxLev + Delta = TxPwr - (PathLoss + TxAtt) + Delta
		 *
		 * The only parameter we can change here is TxAtt, so:
		 *
		 *   RxLev + Delta = TxPwr - PathLoss -  TxAtt + Delta
		 *   RxLev + Delta = TxPwr - PathLoss - (TxAtt - Delta)
		 */
		new_att = state->current - calc_delta_rxlev(params, rxlev_avg);
	}

	/* Make sure new TxAtt is never negative: */
	if (new_att < 0)
		new_att = 0;

	/* Don't ask for higher TxAtt than permitted:  */
	if (new_att > state->max)
		new_att = state->max;

	if (state->current == new_att) {
		LOGPLCHAN(lchan, DLOOP, LOGL_INFO, "Keeping DL attenuation at %u dB: "
			  "max %u dB, RSSI[curr %d, avg %d, thresh %d..%d] dBm, "
			  "RxQual[curr %d, avg %d, thresh %d..%d]\n",
			  state->current,  state->max, rxlev2dbm(rxlev), rxlev2dbm(rxlev_avg),
			  rxlev2dbm(params->rxlev_meas.lower_thresh), rxlev2dbm(params->rxlev_meas.upper_thresh),
			  rxqual, rxqual_avg, params->rxqual_meas.lower_thresh, params->rxqual_meas.upper_thresh);
		return 0;
	}

	LOGPLCHAN(lchan, DLOOP, LOGL_INFO, "%s DL attenuation %u dB => %u dB:"
		  "max %u dB, RSSI[curr %d, avg %d, thresh %d..%d] dBm, "
		   "RxQual[curr %d, avg %d, thresh %d..%d]\n",
		  (new_att > state->current) ? "Raising" : "Lowering",
		  state->current, new_att, state->max, rxlev2dbm(rxlev), rxlev2dbm(rxlev_avg),
		  rxlev2dbm(params->rxlev_meas.lower_thresh), rxlev2dbm(params->rxlev_meas.upper_thresh),
		  rxqual, rxqual_avg, params->rxqual_meas.lower_thresh, params->rxqual_meas.upper_thresh);
	state->current = new_att;
	return 1;
}
