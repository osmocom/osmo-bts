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

void trx_loop_amr_input(struct l1sched_chan_state *chan_state,
			int n_errors, int n_bits_total)
{
	struct gsm_lchan *lchan = chan_state->lchan;
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
	if (lchan->type == GSM_LCHAN_TCH_H) {
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
