
/* (C) 2011 by Harald Welte <laforge@gnumonks.org>
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
#include <osmo-bts/rsl.h>
#include <osmo-bts/oml.h>
#include <osmo-bts/bts_model.h>

#include "l1_if.h"

int bts_model_rsl_chan_act(struct gsm_lchan *lchan, struct tlv_parsed *tp)
{
	//uint8_t mode = *TLVP_VAL(tp, RSL_IE_CHAN_MODE);
	//uint8_t type = *TLVP_VAL(tp, RSL_IE_ACT_TYPE);

	lchan_activate(lchan);
	/* FIXME: only do this in case of success */

	return rsl_tx_chan_act_ack(lchan, bts_model_get_time(lchan->ts->trx->bts));
}

int bts_model_rsl_chan_rel(struct gsm_lchan *lchan)
{
	lchan_deactivate(lchan);
	return rsl_tx_rf_rel_ack(lchan);
}

int bts_model_rsl_deact_sacch(struct gsm_lchan *lchan)
{
	return lchan_deactivate_sacch(lchan);
}

int bts_model_trx_deact_rf(struct gsm_bts_trx *trx)
{
	struct femtol1_hdl *fl1 = trx_femtol1_hdl(trx);

	return l1if_activate_rf(fl1, 0);
}
