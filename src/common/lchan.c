/* OsmoBTS lchan interface */

/* (C) 2012 by Holger Hans Peter Freyther
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

#include <osmocom/core/logging.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/gsm_data.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/rsl.h>

void lchan_set_state(struct gsm_lchan *lchan, enum gsm_lchan_state state)
{
	DEBUGP(DL1C, "%s state %s -> %s\n",
	       gsm_lchan_name(lchan),
	       gsm_lchans_name(lchan->state),
	       gsm_lchans_name(state));
	lchan->state = state;

	/* Early Immediate Assignment: if we have a cached early IA pending, send it upon becoming active, or discard it
	 * when releasing. */
	if (lchan->early_rr_ia) {
		struct gsm_bts *bts = lchan->ts->trx->bts;
		switch (lchan->state) {
		case LCHAN_S_ACT_REQ:
			/* Activation is requested, keep the early IA until active. This allows the BSC to send the IA
			 * even before a dynamic timeslot is done switching to a different pchan kind (experimental). */
			break;
		case LCHAN_S_ACTIVE:
			/* Activation is done, send the RR IA now. Put RR IA msg into the AGCH queue of the BTS. */
			if (bts_agch_enqueue(bts, lchan->early_rr_ia) < 0) {
				/* if there is no space in the queue: send DELETE IND */
				rsl_tx_delete_ind(bts, lchan->early_rr_ia->data, lchan->early_rr_ia->len);
				rate_ctr_inc2(bts->ctrs, BTS_CTR_AGCH_DELETED);
				msgb_free(lchan->early_rr_ia);
			}
			lchan->early_rr_ia = NULL;
			break;
		default:
			/* Transition to any other state means whatever IA the BSC has sent shall now not be relevant
			 * anymore. */
			msgb_free(lchan->early_rr_ia);
			lchan->early_rr_ia = NULL;
			break;
		}
	}
}

bool ts_is_pdch(const struct gsm_bts_trx_ts *ts)
{
	switch (ts->pchan) {
	case GSM_PCHAN_PDCH:
		return true;
	case GSM_PCHAN_TCH_F_PDCH:
		return (ts->flags & TS_F_PDCH_ACTIVE)
		       && !(ts->flags & TS_F_PDCH_PENDING_MASK);
	case GSM_PCHAN_OSMO_DYN:
		return ts->dyn.pchan_is == GSM_PCHAN_PDCH
		       && ts->dyn.pchan_want == ts->dyn.pchan_is;
	default:
		return false;
	}
}
