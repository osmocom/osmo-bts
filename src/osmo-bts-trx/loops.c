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
#include <inttypes.h>

#include <osmo-bts/gsm_data.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/l1sap.h>
#include <osmocom/core/bits.h>
#include <osmocom/gsm/gsm_utils.h>

#include "trx_if.h"
#include "l1_if.h"
#include "loops.h"

/*
 * Timing Advance loop
 */

/* 90% of one bit duration in 1/256 symbols: 256*0.9 */
#define TOA256_9OPERCENT	230

void ta_val(struct gsm_lchan *lchan, struct l1sched_chan_state *chan_state, int16_t toa256)
{
	/* check if the current L1 header acks to the current ordered TA */
	if (lchan->meas.l1_info[1] != lchan->rqd_ta)
		return;

	/* sum measurement */
	chan_state->meas.toa256_sum += toa256;
	if (++(chan_state->meas.toa_num) < 16)
		return;

	/* complete set */
	toa256 = chan_state->meas.toa256_sum / chan_state->meas.toa_num;

	/* check for change of TOA */
	if (toa256 < -TOA256_9OPERCENT && lchan->rqd_ta > 0) {
		LOGPLCHAN(lchan, DLOOP, LOGL_INFO, "TOA is too early (%d), now lowering TA from %d to %d\n",
			toa256, lchan->rqd_ta, lchan->rqd_ta - 1);
		lchan->rqd_ta--;
	} else if (toa256 > TOA256_9OPERCENT && lchan->rqd_ta < 63) {
		LOGPLCHAN(lchan, DLOOP, LOGL_INFO, "TOA is too late (%d), now raising TA from %d to %d\n",
			toa256, lchan->rqd_ta, lchan->rqd_ta + 1);
		lchan->rqd_ta++;
	} else
		LOGPLCHAN(lchan, DLOOP, LOGL_INFO, "TOA is correct (%d), keeping current TA of %d\n",
			toa256, lchan->rqd_ta);

	chan_state->meas.toa_num = 0;
	chan_state->meas.toa256_sum = 0;
}

/*! Process a SACCH event as input to the MS power control and TA loop.  Function
 *  is called once every uplink SACCH block is received.
 * \param l1t L1 TRX instance on which we operate
 * \param chan_nr RSL channel number on which we operate
 * \param chan_state L1 scheduler channel state of the channel on which we operate
 * \param[in] rssi Receive Signal Strength Indication
 * \param[in] toa256 Time of Arrival in 1/256 symbol periods */
void trx_loop_sacch_input(struct l1sched_trx *l1t, uint8_t chan_nr,
	struct l1sched_chan_state *chan_state, int16_t toa256)
{
	struct gsm_lchan *lchan = &l1t->trx->ts[L1SAP_CHAN2TS(chan_nr)]
					.lchan[l1sap_chan2ss(chan_nr)];
	struct phy_instance *pinst = trx_phy_instance(l1t->trx);

	/* if TA loop is enabled, handle it */
	if (pinst->phy_link->u.osmotrx.trx_ta_loop)
		ta_val(lchan, chan_state, toa256);
}

void trx_loop_amr_input(struct l1sched_trx *l1t, uint8_t chan_nr,
	struct l1sched_chan_state *chan_state,
	int n_errors, int n_bits_total)
{
	struct gsm_bts_trx *trx = l1t->trx;
	struct gsm_lchan *lchan = &trx->ts[L1SAP_CHAN2TS(chan_nr)]
					.lchan[l1sap_chan2ss(chan_nr)];
	float ber;

	/* calculate BER (Bit Error Ratio) */
	if (n_bits_total == 0)
		ber = 1.0; /* 100% BER */
	else
		ber = (float) n_errors / (float) n_bits_total;

	/* check if loop is enabled */
	if (!chan_state->amr_loop)
		return;

	/* wait for MS to use the requested codec */
	if (chan_state->ul_ft != chan_state->dl_cmr)
		return;

	/* count bit errors */
	if (L1SAP_IS_CHAN_TCHH(chan_nr)) {
		chan_state->ber_num += 2;
		chan_state->ber_sum += (ber + ber);
	} else {
		chan_state->ber_num++;
		chan_state->ber_sum += ber;
	}

	/* count frames */
	if (chan_state->ber_num < 48)
		return;

	/* calculate average (reuse ber variable) */
	ber = chan_state->ber_sum / chan_state->ber_num;

	/* reset bit errors */
	chan_state->ber_num = 0;
	chan_state->ber_sum = 0;

	LOGPLCHAN(lchan, DLOOP, LOGL_DEBUG, "Current bit error rate (BER) %.6f "
		"codec id %d\n", ber, chan_state->ul_ft);

	/* degrade */
	if (chan_state->dl_cmr > 0) {
		/* degrade, if ber is above threshold FIXME: C/I */
		if (ber >
		   lchan->tch.amr_mr.bts_mode[chan_state->dl_cmr-1].threshold) {
			LOGPLCHAN(lchan, DLOOP, LOGL_DEBUG, "Degrading due to BER %.6f "
				"from codec id %d to %d\n", ber, chan_state->dl_cmr,
				chan_state->dl_cmr - 1);
			chan_state->dl_cmr--;
		}
	} else if (chan_state->dl_cmr < chan_state->codecs - 1) {
		/* degrade, if ber is above threshold  FIXME: C/I*/
		if (ber <
		    lchan->tch.amr_mr.bts_mode[chan_state->dl_cmr].threshold
		  - lchan->tch.amr_mr.bts_mode[chan_state->dl_cmr].hysteresis) {
			LOGPLCHAN(lchan, DLOOP, LOGL_DEBUG, "Upgrading due to BER %.6f "
				"from codec id %d to %d\n", ber, chan_state->dl_cmr,
				chan_state->dl_cmr + 1);
			chan_state->dl_cmr++;
		}
	}
}

void trx_loop_amr_set(struct l1sched_chan_state *chan_state, int loop)
{
	if (chan_state->amr_loop && !loop) {
		chan_state->amr_loop = 0;
		return;
	}

	if (!chan_state->amr_loop && loop) {
		chan_state->amr_loop = 1;

		/* reset bit errors */
		chan_state->ber_num = 0;
		chan_state->ber_sum = 0;

		return;
	}
}
