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

#include <osmocom/trau/osmo_ortp.h>

#include <osmo-bts/logging.h>
#include <osmo-bts/lchan.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/rsl.h>
#include <osmo-bts/pcu_if.h>
#include <osmo-bts/handover.h>
#include <osmo-bts/l1sap.h>
#include <errno.h>

static const struct value_string lchan_s_names[] = {
	{ LCHAN_S_NONE,		"NONE" },
	{ LCHAN_S_ACT_REQ,	"ACTIVATION REQUESTED" },
	{ LCHAN_S_ACTIVE,	"ACTIVE" },
	{ LCHAN_S_REL_REQ,	"RELEASE REQUESTED" },
	{ LCHAN_S_REL_ERR,	"RELEASE DUE ERROR" },
	{ LCHAN_S_BROKEN,	"BROKEN UNUSABLE" },
	{ 0,			NULL }
};

const struct value_string lchan_ciph_state_names[] = {
	{ LCHAN_CIPH_NONE,	"NONE" },
	{ LCHAN_CIPH_RX_REQ,	"RX_REQ" },
	{ LCHAN_CIPH_RX_CONF,	"RX_CONF" },
	{ LCHAN_CIPH_RXTX_REQ,	"RXTX_REQ" },
	{ LCHAN_CIPH_RX_CONF_TX_REQ,	"RX_CONF_TX_REQ" },
	{ LCHAN_CIPH_RXTX_CONF,	"RXTX_CONF" },
	{ 0, NULL }
};

void gsm_lchan_init(struct gsm_lchan *lchan, struct gsm_bts_trx_ts *ts, unsigned int lchan_nr)
{
	lchan->ts = ts;
	lchan->nr = lchan_nr;
	lchan->type = GSM_LCHAN_NONE;
	gsm_lchan_name_update(lchan);

	INIT_LLIST_HEAD(&lchan->sapi_cmds);
	INIT_LLIST_HEAD(&lchan->dl_tch_queue);
}

void gsm_lchan_name_update(struct gsm_lchan *lchan)
{
	const struct gsm_bts_trx_ts *ts = lchan->ts;
	const struct gsm_bts_trx *trx = ts->trx;
	char *name;

	name = talloc_asprintf(trx, "(" GSM_TS_NAME_FMT ",ss=%u)",
			       GSM_TS_NAME_ARGS(ts), lchan->nr);
	if (lchan->name != NULL)
		talloc_free(lchan->name);
	lchan->name = name;
}

static int dyn_ts_pdch_release(struct gsm_lchan *lchan)
{
	struct gsm_bts_trx_ts *ts = lchan->ts;

	if (ts->dyn.pchan_is != ts->dyn.pchan_want) {
		LOGP(DRSL, LOGL_ERROR, "%s: PDCH release requested but already"
		     " in switchover\n", gsm_ts_and_pchan_name(ts));
		return -EINVAL;
	}

	/*
	 * Indicate PDCH Disconnect in dyn_pdch.want, let pcu_tx_info_ind()
	 * pick it up and wait for PCU to disable the channel.
	 */
	ts->dyn.pchan_want = GSM_PCHAN_NONE;

	if (!pcu_connected()) {
		/* PCU not connected yet. Just record the new type and done,
		 * the PCU will pick it up once connected. */
		ts->dyn.pchan_is = GSM_PCHAN_NONE;
		return 1;
	}

	return pcu_tx_info_ind();
}

void gsm_lchan_release(struct gsm_lchan *lchan, enum lchan_rel_act_kind rel_kind)
{
	int rc;

	if (lchan->abis_ip.rtp_socket) {
		rsl_tx_ipac_dlcx_ind(lchan, RSL_ERR_NORMAL_UNSPEC);
		osmo_rtp_socket_log_stats(lchan->abis_ip.rtp_socket, DRTP, LOGL_INFO,
			"Closing RTP socket on Channel Release ");
		osmo_rtp_socket_free(lchan->abis_ip.rtp_socket);
		lchan->abis_ip.rtp_socket = NULL;
		msgb_queue_flush(&lchan->dl_tch_queue);
	}

	/* FIXME: right now we allow creating the rtp_socket even if chan is not
	 * activated... Once we check for that, we can move this check at the
	 * start of the function */
	if (lchan->state == LCHAN_S_NONE)
		return;

	/* release handover state */
	handover_reset(lchan);

	lchan->rel_act_kind = rel_kind;

	/* Dynamic channel in PDCH mode is released via PCU */
	if (lchan->ts->pchan == GSM_PCHAN_OSMO_DYN
	    && lchan->ts->dyn.pchan_is == GSM_PCHAN_PDCH) {
		rc = dyn_ts_pdch_release(lchan);
		if (rc == 1) {
			/* If the PCU is not connected, continue to rel ack right away. */
			lchan->rel_act_kind = LCHAN_REL_ACT_PCU;
			rsl_tx_rf_rel_ack(lchan);
			return;
		}
		/* Waiting for PDCH release */
		return;
	}

	l1sap_chan_rel(lchan->ts->trx, gsm_lchan2chan_nr(lchan));

	lapdm_channel_exit(&lchan->lapdm_ch);
}

const char *gsm_lchans_name(enum gsm_lchan_state s)
{
	return get_value_string(lchan_s_names, s);
}

void early_rr_ia_delay_cb(void *data)
{
	struct gsm_lchan *lchan = data;
	struct gsm_bts *bts = lchan->ts->trx->bts;

	if (!lchan->early_rr_ia) {
		/* The IA message has disappeared since the timer was started. */
		return;
	}

	if (lchan->state != LCHAN_S_ACTIVE) {
		/* Release has happened since the timer was started. */
		msgb_free(lchan->early_rr_ia);
		lchan->early_rr_ia = NULL;
		return;
	}

	/* Activation is done, send the RR IA now. Put RR IA msg into the AGCH queue of the BTS. */
	if (bts_agch_enqueue(bts, lchan->early_rr_ia) < 0) {
		/* if there is no space in the queue: send DELETE IND */
		rsl_tx_delete_ind(bts, lchan->early_rr_ia->data, lchan->early_rr_ia->len);
		rate_ctr_inc2(bts->ctrs, BTS_CTR_AGCH_DELETED);
		msgb_free(lchan->early_rr_ia);
	}
	lchan->early_rr_ia = NULL;
}

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
		switch (lchan->state) {
		case LCHAN_S_ACT_REQ:
			/* Activation is requested, keep the early IA until active. This allows the BSC to send the IA
			 * even before a dynamic timeslot is done switching to a different pchan kind (experimental). */
			break;
		case LCHAN_S_ACTIVE:
			/* Activation is done, send the RR IA now. Delay a bit more to give Um time to let the lchan
			 * light up for the MS */
			osmo_timer_del(&lchan->early_rr_ia_delay);
			osmo_timer_setup(&lchan->early_rr_ia_delay, early_rr_ia_delay_cb, lchan);
			osmo_timer_schedule(&lchan->early_rr_ia_delay, 0,
					    osmo_tdef_get(abis_T_defs, -15, OSMO_TDEF_US, -1));
			break;
		default:
			/* Transition to any other state means whatever IA the BSC has sent shall now not be relevant
			 * anymore. */
			osmo_timer_del(&lchan->early_rr_ia_delay);
			msgb_free(lchan->early_rr_ia);
			lchan->early_rr_ia = NULL;
			break;
		}
	}
}

/* See Table 10.5.25 of GSM04.08 */
static uint8_t gsm_pchan2chan_nr(enum gsm_phys_chan_config pchan,
			  uint8_t ts_nr, uint8_t lchan_nr)
{
	uint8_t cbits, chan_nr;

	OSMO_ASSERT(pchan != GSM_PCHAN_OSMO_DYN);
	OSMO_ASSERT(pchan != GSM_PCHAN_TCH_F_PDCH);

	switch (pchan) {
	case GSM_PCHAN_TCH_F:
		OSMO_ASSERT(lchan_nr == 0);
		cbits = ABIS_RSL_CHAN_NR_CBITS_Bm_ACCHs;
		break;
	case GSM_PCHAN_PDCH:
		OSMO_ASSERT(lchan_nr == 0);
		cbits = ABIS_RSL_CHAN_NR_CBITS_OSMO_PDCH;
		break;
	case GSM_PCHAN_TCH_H:
		OSMO_ASSERT(lchan_nr < 2);
		cbits = ABIS_RSL_CHAN_NR_CBITS_Lm_ACCHs(lchan_nr);
		break;
	case GSM_PCHAN_CCCH_SDCCH4:
	case GSM_PCHAN_CCCH_SDCCH4_CBCH:
		/*
		 * As a special hack for BCCH, lchan_nr == 4 may be passed
		 * here. This should never be sent in an RSL message.
		 * See osmo-bts-xxx/oml.c:opstart_compl().
		 */
		if (lchan_nr == CCCH_LCHAN)
			cbits = ABIS_RSL_CHAN_NR_CBITS_BCCH;
		else {
			OSMO_ASSERT(lchan_nr < 4);
			cbits = ABIS_RSL_CHAN_NR_CBITS_SDCCH4_ACCH(lchan_nr);
		}
		break;
	case GSM_PCHAN_SDCCH8_SACCH8C:
	case GSM_PCHAN_SDCCH8_SACCH8C_CBCH:
		OSMO_ASSERT(lchan_nr < 8);
		cbits = ABIS_RSL_CHAN_NR_CBITS_SDCCH8_ACCH(lchan_nr);
		break;
	case GSM_PCHAN_CCCH:
		cbits = ABIS_RSL_CHAN_NR_CBITS_BCCH;
		break;
	case GSM_PCHAN_NONE:
		LOGP(DRSL, LOGL_ERROR, "Physical channel %s not expected!\n",
		     gsm_pchan_name(pchan));
		cbits = 0x00;
		break;
	default:
		LOGP(DRSL, LOGL_ERROR, "Physical channel %s (0x%02x) not expected!\n",
		     gsm_pchan_name(pchan), (int)pchan);
		/* OSMO_ASSERT(lchan_nr == 0);
		 * FIXME: On octphy and litecell, we hit above assertion (see
		 * Max's comment at https://gerrit.osmocom.org/589 ); disabled
		 * for BTS until this is clarified; remove the #ifdef when it
		 * is fixed. Tracked in OS#2906.
		 */
#pragma message "fix caller that passes lchan_nr != 0"
		cbits = 0x10;
		break;
	}

	chan_nr = (cbits << 3) | (ts_nr & 0x7);

	return chan_nr;
}

static uint8_t _gsm_lchan2chan_nr(const struct gsm_lchan *lchan, bool rsl)
{
	uint8_t chan_nr;

	switch (lchan->ts->pchan) {
	case GSM_PCHAN_OSMO_DYN:
		/* Return chan_nr reflecting the current TS pchan, either a standard TCH kind, or the
		 * nonstandard value reflecting PDCH for Osmocom style dyn TS. */
		chan_nr = gsm_lchan_as_pchan2chan_nr(lchan, lchan->ts->dyn.pchan_is);
		break;
	case GSM_PCHAN_TCH_F_PDCH:
		/* For ip.access style dyn TS, on RSL we want to use the chan_nr as if it was TCH/F.
		 * We're using custom PDCH ACT and DEACT messages that use the usual chan_nr values. */
		if (rsl)
			chan_nr = gsm_lchan_as_pchan2chan_nr(lchan, GSM_PCHAN_TCH_F);
		else if (~lchan->ts->flags & TS_F_PDCH_ACTIVE)
			chan_nr = gsm_lchan_as_pchan2chan_nr(lchan, GSM_PCHAN_TCH_F);
		else
			chan_nr = gsm_lchan_as_pchan2chan_nr(lchan, GSM_PCHAN_PDCH);
		break;
	default:
		chan_nr = gsm_pchan2chan_nr(lchan->ts->pchan, lchan->ts->nr, lchan->nr);
		break;
	}

	/* VAMOS: if this lchan belongs to a shadow timeslot, we must reflect
	 * this in the channel number.  Convert it to Osmocom specific value. */
	if (lchan->ts->vamos.is_shadow)
		chan_nr |= RSL_CHAN_OSMO_VAMOS_MASK;

	return chan_nr;
}

uint8_t gsm_lchan2chan_nr(const struct gsm_lchan *lchan)
{
	return _gsm_lchan2chan_nr(lchan, false);
}

uint8_t gsm_lchan2chan_nr_rsl(const struct gsm_lchan *lchan)
{
	return _gsm_lchan2chan_nr(lchan, true);
}

uint8_t gsm_lchan_as_pchan2chan_nr(const struct gsm_lchan *lchan,
				   enum gsm_phys_chan_config as_pchan)
{
	if (lchan->ts->pchan == GSM_PCHAN_OSMO_DYN
	    && as_pchan == GSM_PCHAN_PDCH)
		return RSL_CHAN_OSMO_PDCH | (lchan->ts->nr & ~RSL_CHAN_NR_MASK);
	return gsm_pchan2chan_nr(as_pchan, lchan->ts->nr, lchan->nr);
}

/* Called by the model specific code every 104 TDMA frames (SACCH period) */
void gsm_lchan_interf_meas_push(struct gsm_lchan *lchan, int dbm)
{
	const uint8_t meas_num = lchan->meas.interf_meas_num;

	if (meas_num >= ARRAY_SIZE(lchan->meas.interf_meas_dbm)) {
		LOGPLCHAN(lchan, DL1C, LOGL_ERROR, "Not enough room "
			  "to store interference report (%ddBm)\n", dbm);
		return;
	}

	lchan->meas.interf_meas_dbm[meas_num] = dbm;
	lchan->meas.interf_meas_num++;
}

/* Called by the higher layers every Intave * 104 TDMA frames */
int gsm_lchan_interf_meas_calc_band(struct gsm_lchan *lchan)
{
	const uint8_t meas_num = lchan->meas.interf_meas_num;
	const struct gsm_bts *bts = lchan->ts->trx->bts;
	int b, meas_avg, meas_sum = 0;

	/* There must be at least one sample */
	if (meas_num == 0)
		return -EAGAIN;

	/* Calculate the sum of all collected samples (in -x dBm) */
	while (lchan->meas.interf_meas_num) {
		uint8_t i = --lchan->meas.interf_meas_num;
		meas_sum += lchan->meas.interf_meas_dbm[i];
	}

	/* Calculate the average of all collected samples */
	meas_avg = meas_sum / (int) meas_num;

	/* Determine the band using interference boundaries from BSC */
	for (b = 0; b < ARRAY_SIZE(bts->interference.boundary); b++) {
		if (meas_avg >= bts->interference.boundary[b])
			break; /* Current 'b' is the band value */
	}

	LOGPLCHAN(lchan, DL1C, LOGL_DEBUG,
		  "Interference AVG: %ddBm (band %d, samples %u)\n",
		  meas_avg, b, meas_num);

	return b;
}

/* determine the ECU codec constant for the codec used by given lchan */
int lchan2ecu_codec(const struct gsm_lchan *lchan)
{
	const struct gsm_bts_trx_ts *ts = lchan->ts;

	switch (lchan->tch_mode) {
	case GSM48_CMODE_SPEECH_V1:
		if (ts_pchan(ts) == GSM_PCHAN_TCH_H)
			return OSMO_ECU_CODEC_HR;
		else
			return OSMO_ECU_CODEC_FR;
		break;
	case GSM48_CMODE_SPEECH_EFR:
		return OSMO_ECU_CODEC_EFR;
	case GSM48_CMODE_SPEECH_AMR:
		return OSMO_ECU_CODEC_AMR;
	default:
		return -1;
	}
}
