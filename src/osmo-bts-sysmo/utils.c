/*
 * Helper utilities that are used in OML
 *
 * (C) 2011-2013 by Harald Welte <laforge@gnumonks.org>
 * (C) 2013 by Holger Hans Peter Freyther
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

#include "utils.h"

#include <osmo-bts/bts.h>
#include <osmo-bts/gsm_data.h>
#include <osmo-bts/logging.h>

#include "femtobts.h"
#include "l1_if.h"

int band_femto2osmo(GsmL1_FreqBand_t band)
{
	switch (band) {
	case GsmL1_FreqBand_850:
		return GSM_BAND_850;
	case GsmL1_FreqBand_900:
		return GSM_BAND_900;
	case GsmL1_FreqBand_1800:
		return GSM_BAND_1800;
	case GsmL1_FreqBand_1900:
		return GSM_BAND_1900;
	default:
		return -1;
	}
}

static int band_osmo2femto(struct gsm_bts_trx *trx, enum gsm_band osmo_band)
{
	struct femtol1_hdl *fl1h = trx_femtol1_hdl(trx);

	/* check if the TRX hardware actually supports the given band */
	if (!(fl1h->hw_info.band_support & osmo_band))
		return -1;

	/* if yes, convert from osmcoom style band definition to L1 band */
	switch (osmo_band) {
	case GSM_BAND_850:
		return GsmL1_FreqBand_850;
	case GSM_BAND_900:
		return GsmL1_FreqBand_900;
	case GSM_BAND_1800:
		return GsmL1_FreqBand_1800;
	case GSM_BAND_1900:
		return GsmL1_FreqBand_1900;
	default:
		return -1;
	}
}

/**
 * Select the band that matches the ARFCN. In general the ARFCNs
 * for GSM1800 and GSM1900 overlap and one needs to specify the
 * rightband. When moving between GSM900/GSM1800 and GSM850/1900
 * modifying the BTS configuration is a bit annoying. The auto-band
 * configuration allows to ease with this transition.
 */
int sysmobts_select_femto_band(struct gsm_bts_trx *trx, uint16_t arfcn)
{
	enum gsm_band band;
	struct gsm_bts *bts = trx->bts;
	struct gsm_bts_role_bts *btsb = bts_role_bts(bts);

	if (!btsb->auto_band)
		return band_osmo2femto(trx, bts->band);

	/*
	 * We need to check what will happen now.
	 */
	band = gsm_arfcn2band(arfcn);

	/* if we are already on the right band return */
	if (band == bts->band)
		return band_osmo2femto(trx, bts->band);

	/* Check if it is GSM1800/GSM1900 */
	if (band == GSM_BAND_1800 && bts->band == GSM_BAND_1900)
		return band_osmo2femto(trx, bts->band);

	/*
	 * Now to the actual autobauding. We just want DCS/DCS and
	 * PCS/PCS for PCS we check for 850/1800 though
	 */
	if ((band == GSM_BAND_900 && bts->band == GSM_BAND_1800)
		|| (band == GSM_BAND_1800 && bts->band == GSM_BAND_900)
		|| (band == GSM_BAND_850 && bts->band == GSM_BAND_1900))
		return band_osmo2femto(trx, band);
	if (band == GSM_BAND_1800 && bts->band == GSM_BAND_850)
		return band_osmo2femto(trx, GSM_BAND_1900);

	/* give up */
	return -1;
}

int sysmobts_get_nominal_power(struct gsm_bts_trx *trx)
{
	struct femtol1_hdl *fl1h = trx_femtol1_hdl(trx);

	switch (fl1h->hw_info.model_nr) {
	case 0:
	case 0xffff:
		/* old units have empty flash where the model number is
		 * stored in later units */
	case 1002:
		/* 200mW (23 dBm) nominal power */
		return 23;
	case 2050:
		/* 5W(37dBm) per TRX. This could be raiesd to 10W(40dBm)
		 * if the second TRX is not used. */
		return 37;
	default:
		LOGP(DL1C, LOGL_ERROR, "Model number %u/0x%x not known.\n",
			fl1h->hw_info.model_nr, fl1h->hw_info.model_nr);
		break;
	}
	return -1;
}

int sysmobts_get_target_power(struct gsm_bts_trx *trx)
{
	int target_power = trx->nominal_power - trx->max_power_red;
	target_power -= trx->power_reduce;

	if (target_power < 0)
		target_power = 0;

	return target_power;
}

void sysmobts_pa_pwr_init(struct gsm_bts_trx *trx)
{
	int target_power = sysmobts_get_target_power(trx);

	/* Cancel any pending request */
	osmo_timer_del(&trx->pa.step_timer);

	/* is this below our initial target */
	if (target_power <= trx->pa.max_initial_power) {
		LOGP(DL1C, LOGL_NOTICE,
			"PA target_power(%d) is below initial power.\n",
			target_power);
		trx->pa.current_power = target_power;
		return;
	}

	/* is this below our current value? */
	if (target_power <= trx->pa.current_power) {
		LOGP(DL1C, LOGL_NOTICE,
			"PA target_power(%d) is below current_power.\n",
			target_power);
		trx->pa.current_power = target_power;
		return;
	}

	if (trx->pa.current_power > trx->pa.max_initial_power) {
		LOGP(DL1C, LOGL_NOTICE,
			"PA target_power(%d) starting from current_power.\n",
			target_power);
		return;
	}

	/* We need to step it up. Start from the initial value */
	trx->pa.current_power = trx->pa.max_initial_power;
	LOGP(DL1C, LOGL_NOTICE,
		"PA target_power(%d) starting with %d dBm.\n",
		target_power, trx->pa.current_power);
}

static void pa_trx_cb(void *_trx)
{
	struct gsm_bts_trx *trx = _trx;

	LOGP(DL1C, LOGL_NOTICE,
		"PA raising power to %d dBm.\n", trx->pa.current_power);
	l1if_set_txpower(trx_femtol1_hdl(trx), (float) trx->pa.current_power);
}

void sysmobts_pa_maybe_step(struct gsm_bts_trx *trx)
{
	/* it can not have changed */
	int target_power = sysmobts_get_target_power(trx);

	/* We are done */
	if (trx->pa.current_power >= target_power) {
		LOGP(DL1C, LOGL_NOTICE,
			"PA have reached target power: %d dBm.\n",
			target_power);
		return;
	}

	/* Step up the current power but clamp it */
	trx->pa.current_power += trx->pa.step_size;
	if (trx->pa.current_power > target_power)
		trx->pa.current_power = target_power;

	LOGP(DL1C, LOGL_NOTICE,
		"PA scheduling to step to %d dBm.\n",
		trx->pa.current_power);

	trx->pa.step_timer.data = trx;
	trx->pa.step_timer.cb = pa_trx_cb;
	osmo_timer_schedule(&trx->pa.step_timer, trx->pa.step_interval, 0);
}
