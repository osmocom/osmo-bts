/* Paging message encoding + queue management */

/* (C) 2012-2013 by Harald Welte <laforge@gnumonks.org>
 *                  Andreas Eversberg <jolly@eversberg.eu>
 * (C) 2014 by Holger Hans Peter Freyther
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

#include <stdlib.h>
#include <stdint.h>
#include <errno.h>

#include <osmocom/gsm/protocol/gsm_04_08.h>
#include <osmocom/gsm/rsl.h>

#include <osmo-bts/bts.h>
#include <osmo-bts/bts_model.h>
#include <osmo-bts/rsl.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/handover.h>
#include <osmo-bts/l1sap.h>

/* Transmit a handover related PHYS INFO on given lchan */
static int ho_tx_phys_info(struct gsm_lchan *lchan)
{
	struct msgb *msg = msgb_alloc_headroom(1024, 128, "PHYS INFO");
	struct gsm48_hdr *gh;

	if (!msg)
		return -ENOMEM;

	LOGPLCHAN(lchan, DHO, LOGL_INFO, "Sending PHYSICAL INFORMATION to MS.\n");

	/* Build RSL UNITDATA REQUEST message with 04.08 PHYS INFO */
	msg->l3h = msg->data;
	gh = (struct gsm48_hdr *) msgb_put(msg, sizeof(*gh));
	gh->proto_discr = GSM48_PDISC_RR;
	gh->msg_type = GSM48_MT_RR_HANDO_INFO;
	msgb_put_u8(msg, lchan->rqd_ta);

	rsl_rll_push_l3(msg, RSL_MT_UNIT_DATA_REQ, gsm_lchan2chan_nr(lchan),
		0x00, 0);

	lapdm_rslms_recvmsg(msg, &lchan->lapdm_ch);
	return 0;
}

/* timer call-back for T3105 (handover PHYS INFO re-transmit) */
static void ho_t3105_cb(void *data)
{
	struct gsm_lchan *lchan = data;
	struct gsm_bts *bts = lchan->ts->trx->bts;

	LOGPLCHAN(lchan, DHO, LOGL_INFO, "T3105 timeout (%d resends left)\n",
		  bts->ny1 - lchan->ho.phys_info_count);

	if (lchan->state != LCHAN_S_ACTIVE) {
		LOGPLCHAN(lchan, DHO, LOGL_NOTICE, "is in not active. It is in state %s. Ignoring\n",
			  gsm_lchans_name(lchan->state));
		return;
	}

	if (lchan->ho.phys_info_count >= bts->ny1) {
		/* HO Abort */
		LOGPLCHAN(lchan, DHO, LOGL_NOTICE, "NY1 reached, sending CONNection FAILure to BSC.\n");
		rsl_tx_conn_fail(lchan, RSL_ERR_HANDOVER_ACC_FAIL);
		return;
	}

	ho_tx_phys_info(lchan);
	lchan->ho.phys_info_count++;
	osmo_timer_schedule(&lchan->ho.t3105, 0, bts->t3105_ms * 1000);
}

/* received random access on dedicated channel */
void handover_rach(struct gsm_lchan *lchan, uint8_t ra, uint8_t acc_delay)
{
	struct gsm_bts *bts = lchan->ts->trx->bts;

	/* Ignore invalid handover ref */
	if (lchan->ho.ref != ra) {
		LOGPLCHAN(lchan, DHO, LOGL_INFO, "RACH on dedicated channel received, but "
			  "ra=0x%02x != expected ref=0x%02x. (This is no bug)\n", ra, lchan->ho.ref);
		return;
	}

	/* Ignore handover on channels other than DCCH and SACCH */
	if (lchan->type != GSM_LCHAN_SDCCH && lchan->type != GSM_LCHAN_TCH_H &&
		lchan->type != GSM_LCHAN_TCH_F) {
		LOGPLCHAN(lchan, DHO, LOGL_ERROR, "handover RACH received on %s?!\n",
			  gsm_lchant_name(lchan->type));
		return;
	}

	LOGPLCHAN(lchan, DHO, LOGL_NOTICE, "RACH on dedicated channel type %s received with "
		  "TA=%u, ref=%u\n", gsm_lchant_name(lchan->type), acc_delay, ra);

	/* Set timing advance */
	lchan->rqd_ta = acc_delay;

	/* Stop handover detection, wait for valid frame */
	lchan->ho.active = HANDOVER_WAIT_FRAME;
	if (l1sap_chan_modify(lchan->ts->trx, gsm_lchan2chan_nr(lchan)) != 0) {
		LOGPLCHAN(lchan, DHO, LOGL_ERROR, "failed to modify channel after handover\n");
		rsl_tx_conn_fail(lchan, RSL_ERR_HANDOVER_ACC_FAIL);
		return;
	}

	/* Send HANDover DETect to BSC */
	rsl_tx_hando_det(lchan, &lchan->rqd_ta);

	/* Send PHYS INFO */
	lchan->ho.phys_info_count = 1;
	ho_tx_phys_info(lchan);

	/* Start T3105 */
	LOGPLCHAN(lchan, DHO, LOGL_DEBUG, "Starting T3105 with %u ms\n", bts->t3105_ms);
	lchan->ho.t3105.cb = ho_t3105_cb;
	lchan->ho.t3105.data = lchan;
	osmo_timer_schedule(&lchan->ho.t3105, 0, bts->t3105_ms * 1000);
}

/* received first valid data frame on dedicated channel */
void handover_frame(struct gsm_lchan *lchan)
{
	LOGPLCHAN(lchan, DHO, LOGL_INFO, "First valid frame detected\n");
	handover_reset(lchan);
}

/* release handover state */
void handover_reset(struct gsm_lchan *lchan)
{
	/* Stop T3105 */
	osmo_timer_del(&lchan->ho.t3105);

	/* Handover process is done */
	lchan->ho.active = HANDOVER_NONE;
}
