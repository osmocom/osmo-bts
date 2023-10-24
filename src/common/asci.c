/* ASCI (VGCS/VBS) related common code */

/* (C) 2023 by Harald Welte <laforge@osmocom.org>
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
#include <osmo-bts/l1sap.h>
#include <osmo-bts/asci.h>

static int tx_vgcs_ul_grant(struct gsm_lchan *lchan)
{
	struct gsm0408_vgcs_ul_grant ul_grant;
	struct gsm_time gt;
	struct msgb *msg;

	gsm_fn2gsmtime(&gt, lchan->asci.fn);

	/* build the RR VGCS UPLINK GRANT message as per TS 44.018 Section 9.1.49 */
	ul_grant = (struct gsm0408_vgcs_ul_grant) {
		.hdr = {
			.proto_discr = GSM48_PDISC_RR,
			.msg_type = GSM48_MT_RR_VGCS_UPL_GRANT,
		},
		.req_ref = {
			.ra = lchan->asci.ref,
			.t1 = gt.t1,
			.t2 = gt.t2,
			.t3_low = gt.t3 & 7,
			.t3_high = gt.t3 >> 3,
		},
		.ta = lchan->ta_ctrl.current,
	};

	/* Wrap it in a RSL UNITDATA REQUEST */
	msg = rsl_rll_simple(RSL_MT_UNIT_DATA_REQ, gsm_lchan2chan_nr(lchan), 0x00, 0);
	msg->l3h = msg->tail; /* emulate rsl_rx_rll() behaviour */
	msgb_tl16v_put(msg, RSL_IE_L3_INFO, sizeof(ul_grant), (uint8_t *) &ul_grant);

	/* send it towards MS, just like a RSL message from the BSC */
	return lapdm_rslms_recvmsg(msg, &lchan->lapdm_ch);
}

/* timer call-back for T3115 (VGCS UPLINK GRANT re-transmit) */
static void vgcs_t3115_cb(void *data)
{
	struct gsm_lchan *lchan = data;
	struct gsm_bts *bts = lchan->ts->trx->bts;

	LOGPLCHAN(lchan, DASCI, LOGL_INFO, "T3115 timeout (%d resends left)\n",
		  bts->ny2 - lchan->asci.vgcs_ul_grant_count);

	if (lchan->state != LCHAN_S_ACTIVE) {
		LOGPLCHAN(lchan, DASCI, LOGL_NOTICE, "is not active. It is in state %s. Ignoring\n",
			  gsm_lchans_name(lchan->state));
		return;
	}

	if (lchan->asci.vgcs_ul_grant_count >= bts->ny2) {
		lchan->asci.vgcs_ul_grant_count = 0;
		LOGPLCHAN(lchan, DASCI, LOGL_NOTICE, "NY2 reached, sending CONNection FAILure to BSC.\n");
		rsl_tx_conn_fail(lchan, RSL_ERR_TALKER_ACC_FAIL);
		lchan->asci.talker_active = VGCS_TALKER_NONE;
		return;
	}

	tx_vgcs_ul_grant(lchan);
	lchan->asci.vgcs_ul_grant_count++;
	osmo_timer_schedule(&lchan->asci.t3115, 0, bts->t3115_ms * 1000);
}

/* Received random access on dedicated channel. */
void vgcs_rach(struct gsm_lchan *lchan, uint8_t ra, uint8_t acc_delay, uint32_t fn)
{
	LOGPLCHAN(lchan, DASCI, LOGL_NOTICE, "VGCS RACH on dedicated channel type %s received with "
		  "TA=%u, ref=%u\n", gsm_lchant_name(lchan->type), acc_delay, ra);

	if (ra == 0x25) { /* See TS 44.018 Table 9.1.45.1 */
		/* Listener Detection (TS 48.058 Section 4.14) */
		if (!lchan->asci.listener_detected) {
			rsl_tx_listener_det(lchan, &acc_delay);
			lchan->asci.listener_detected = true;
		}
	} else {
		/* Talker Detection (TS 48.058 Section 4.13) */
		struct gsm_bts *bts = lchan->ts->trx->bts;

		/* Talker detection on group channels only */
		if (!rsl_chan_rt_is_vgcs(lchan->rsl_chan_rt))
			return;

		if (lchan->asci.talker_active != VGCS_TALKER_NONE) {
			LOGPLCHAN(lchan, DASCI, LOGL_DEBUG, "Ignoring RACH, there is an active talker already.\n");
			return;
		}

		/* Set timing advance, power level and activate SACCH */
		lchan->ta_ctrl.current = acc_delay;
		lchan->ms_power_ctrl.current = lchan->ms_power_ctrl.max;
		lchan->want_dl_sacch_active = true;

		/* Stop RACH detection, wait for valid frame */
		lchan->asci.talker_active = VGCS_TALKER_WAIT_FRAME;
		if (l1sap_uplink_access(lchan, false) != 0) {
			LOGPLCHAN(lchan, DASCI, LOGL_ERROR, "Failed to deactivate uplink access after TALKER DET.\n");
			rsl_tx_conn_fail(lchan, RSL_ERR_EQUIPMENT_FAIL);
			lchan->asci.talker_active = VGCS_TALKER_NONE;
			return;
		}

		lchan->asci.ref = ra;
		lchan->asci.fn = fn;

		/* Send TALKER DETECT via RSL to BSC */
		rsl_tx_talker_det(lchan, &acc_delay);

		/* Send VGCS UPLINK GRANT */
		lchan->asci.vgcs_ul_grant_count = 1;
		tx_vgcs_ul_grant(lchan);

		/* Start T3115 */
		LOGPLCHAN(lchan, DASCI, LOGL_DEBUG, "Starting T3115 with %u ms\n", bts->t3115_ms);
		lchan->asci.t3115.cb = vgcs_t3115_cb;
		lchan->asci.t3115.data = lchan;
		osmo_timer_schedule(&lchan->asci.t3115, 0, bts->t3115_ms * 1000);
	}
}

/* Received channel activation. */
void vgcs_lchan_activate(struct gsm_lchan *lchan)
{
	LOGPLCHAN(lchan, DASCI, LOGL_INFO, "Channel is activated.\n");
	if (l1sap_uplink_access(lchan, true) != 0) {
		LOGPLCHAN(lchan, DASCI, LOGL_ERROR, "Failed to activate uplink access after channel activation.\n");
		rsl_tx_conn_fail(lchan, RSL_ERR_EQUIPMENT_FAIL);
	}
}

/* Received channel reactivation. (for assignment) */
void vgcs_lchan_react(struct gsm_lchan *lchan)
{
	LOGPLCHAN(lchan, DASCI, LOGL_INFO, "Channel is activated for assignment.\n");
	lchan->asci.talker_active = VGCS_TALKER_WAIT_FRAME;
	if (l1sap_uplink_access(lchan, false) != 0) {
		LOGPLCHAN(lchan, DASCI, LOGL_ERROR, "Failed to deactivate uplink access for assignment.\n");
		rsl_tx_conn_fail(lchan, RSL_ERR_EQUIPMENT_FAIL);
	}
	radio_link_timeout_reset(lchan);
}

/* Received first valid data frame on dedicated channel. */
void vgcs_talker_frame(struct gsm_lchan *lchan)
{
	LOGPLCHAN(lchan, DASCI, LOGL_INFO, "First valid frame detected, talker now active.\n");
	osmo_timer_del(&lchan->asci.t3115);
	lchan->asci.talker_active = VGCS_TALKER_ACTIVE;
	radio_link_timeout_reset(lchan);
}

/* Release VGCS Talker state. */
void vgcs_talker_reset(struct gsm_lchan *lchan, bool ul_access)
{
	if (lchan->asci.talker_active == VGCS_TALKER_NONE)
		return;

	LOGPLCHAN(lchan, DASCI, LOGL_INFO, "Uplink released, no talker.\n");

	/* Stop T3115 */
	osmo_timer_del(&lchan->asci.t3115);

	/* Talker released. */
	lchan->asci.talker_active = VGCS_TALKER_NONE;
	if (ul_access) {
		if (l1sap_uplink_access(lchan, true) != 0) {
			LOGPLCHAN(lchan, DASCI, LOGL_ERROR,
				  "Failed to activate uplink access after uplink became free.\n");
			rsl_tx_conn_fail(lchan, RSL_ERR_EQUIPMENT_FAIL);
		}
	}
}

/* Release VGCS Listener state. */
void vgcs_listener_reset(struct gsm_lchan *lchan)
{
	lchan->asci.listener_detected = false;
}
