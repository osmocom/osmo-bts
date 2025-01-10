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
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "btsconfig.h"	/* for PACKAGE_VERSION */

#include <osmocom/core/logging.h>

#include <osmo-bts/logging.h>
#include <osmo-bts/lchan.h>
#include <osmo-bts/rtp_abstract.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/rsl.h>
#include <osmo-bts/pcu_if.h>
#include <osmo-bts/handover.h>
#include <osmo-bts/l1sap.h>
#include <osmo-bts/bts_model.h>
#include <osmo-bts/asci.h>
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

const struct value_string lchan_csd_mode_descs[] = {
	{ LCHAN_CSD_M_NT,		"non-transparent" },
	{ LCHAN_CSD_M_T_1200_75,	"transparent @ 1200/75 bps" },
	{ LCHAN_CSD_M_T_600,		"transparent @ 600 bps" },
	{ LCHAN_CSD_M_T_1200,		"transparent @ 1200 bps" },
	{ LCHAN_CSD_M_T_2400,		"transparent @ 2400 bps" },
	{ LCHAN_CSD_M_T_4800,		"transparent @ 4800 bps" },
	{ LCHAN_CSD_M_T_9600,		"transparent @ 9600 bps" },
	{ LCHAN_CSD_M_T_14400,		"transparent @ 14400 bps" },
	{ LCHAN_CSD_M_T_29000,		"transparent @ 29000 bps" },
	{ LCHAN_CSD_M_T_32000,		"transparent @ 32000 bps" },
	{ 0, NULL }
};

/* prepare the per-SAPI T200 arrays for a given lchan */
static int t200_by_lchan(uint32_t *t200_fn_dcch, uint32_t *t200_fn_acch, struct gsm_lchan *lchan)
{
	struct gsm_bts *bts = lchan->ts->trx->bts;

	switch (lchan->type) {
	case GSM_LCHAN_SDCCH:
		t200_fn_dcch[DL_SAPI0] = bts->t200_fn[T200_SDCCH];
		t200_fn_dcch[DL_SAPI3] = bts->t200_fn[T200_SDCCH_SAPI3];
		t200_fn_acch[DL_SAPI0] = bts->t200_fn[T200_SACCH_SDCCH];
		t200_fn_acch[DL_SAPI3] = bts->t200_fn[T200_SACCH_SDCCH];
		break;
	case GSM_LCHAN_TCH_F:
		t200_fn_dcch[DL_SAPI0] = bts->t200_fn[T200_FACCH_F];
		t200_fn_dcch[DL_SAPI3] = bts->t200_fn[T200_FACCH_F];
		t200_fn_acch[DL_SAPI0] = bts->t200_fn[T200_SACCH_TCH_SAPI0];
		t200_fn_acch[DL_SAPI3] = bts->t200_fn[T200_SACCH_TCH_SAPI3];
		break;
	case GSM_LCHAN_TCH_H:
		t200_fn_dcch[DL_SAPI0] = bts->t200_fn[T200_FACCH_H];
		t200_fn_dcch[DL_SAPI3] = bts->t200_fn[T200_FACCH_H];
		t200_fn_acch[DL_SAPI0] = bts->t200_fn[T200_SACCH_TCH_SAPI0];
		t200_fn_acch[DL_SAPI3] = bts->t200_fn[T200_SACCH_TCH_SAPI3];
		break;
	default:
		/* Channels such as CCCH don't use lapdm DL, and hence no T200 is needed */
		return -1;
	}

	/* Add time of two extra messages frames. */
	if (lchan->rep_acch_cap.dl_facch_all && lchan_is_tch(lchan)) {
		t200_fn_acch[DL_SAPI0] += 104 * 2;
		t200_fn_acch[DL_SAPI3] += 104 * 2;
	}

	return 0;
}

static void early_rr_ia_delay_cb(void *data)
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

void gsm_lchan_init(struct gsm_lchan *lchan, struct gsm_bts_trx_ts *ts, unsigned int lchan_nr)
{
	lchan->ts = ts;
	lchan->nr = lchan_nr;
	lchan->type = GSM_LCHAN_NONE;
	gsm_lchan_name_update(lchan);

	osmo_timer_setup(&lchan->early_rr_ia_delay, early_rr_ia_delay_cb, lchan);

	INIT_LLIST_HEAD(&lchan->sapi_cmds);
	INIT_LLIST_HEAD(&lchan->dl_tch_queue);
	lchan->dl_tch_queue_len = 0;
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

int lchan_init_lapdm(struct gsm_lchan *lchan)
{
	struct lapdm_channel *lc = &lchan->lapdm_ch;
	uint32_t t200_fn_dcch[_NR_DL_SAPI], t200_fn_acch[_NR_DL_SAPI];

	if (t200_by_lchan(t200_fn_dcch, t200_fn_acch, lchan) == 0) {
		LOGPLCHAN(lchan, DLLAPD, LOGL_DEBUG,
			  "Setting T200 D0=%u, D3=%u, S0=%u, S3=%u (all in frames)\n",
			  t200_fn_dcch[DL_SAPI0], t200_fn_dcch[DL_SAPI3],
			  t200_fn_acch[DL_SAPI0], t200_fn_acch[DL_SAPI3]);
		lapdm_channel_init3(lc, LAPDM_MODE_BTS, NULL, NULL, lchan->type, gsm_lchan_name(lchan));
		lapdm_channel_set_flags(lc, LAPDM_ENT_F_POLLING_ONLY | LAPDM_ENT_F_RTS);
		lapdm_channel_set_l1(lc, NULL, lchan);
		lapdm_channel_set_t200_fn(lc, t200_fn_dcch, t200_fn_acch);
	}
	/* We still need to set Rx callback to receive RACH requests: */
	lapdm_channel_set_l3(lc, lapdm_rll_tx_cb, lchan);

	return 0;
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
		rtp_abst_socket_log_stats(lchan->abis_ip.rtp_socket, DRTP, LOGL_INFO,
			"Closing RTP socket on Channel Release ");
		lchan_rtp_socket_free(lchan);
	} else if (lchan->abis_ip.osmux.use) {
		lchan_osmux_release(lchan);
	}
	/* reset all Abis related config: */
	memset(&lchan->abis_ip, 0, sizeof(lchan->abis_ip));

	/* FIXME: right now we allow creating the rtp_socket even if chan is not
	 * activated... Once we check for that, we can move this check at the
	 * start of the function */
	if (lchan->state == LCHAN_S_NONE)
		return;

	/* release handover, listener and talker states */
	handover_reset(lchan);
	vgcs_talker_reset(lchan, false);
	vgcs_listener_reset(lchan);
	vgcs_uplink_free_reset(lchan);

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
}

int lchan_deactivate(struct gsm_lchan *lchan)
{
	OSMO_ASSERT(lchan);

	lchan->ciph_state = 0;
	return bts_model_lchan_deactivate(lchan);
}

const char *gsm_lchans_name(enum gsm_lchan_state s)
{
	return get_value_string(lchan_s_names, s);
}

/* obtain the next to-be transmitted dowlink SACCH frame (L2 hdr + L3); returns pointer to lchan->si buffer */
uint8_t *lchan_sacch_get(struct gsm_lchan *lchan)
{
	uint32_t tmp, i;

	for (i = 0; i < _MAX_SYSINFO_TYPE; i++) {
		tmp = (lchan->si.last + 1 + i) % _MAX_SYSINFO_TYPE;
		if (!(lchan->si.valid & (1 << tmp)))
			continue;
		lchan->si.last = tmp;
		return GSM_LCHAN_SI(lchan, tmp);
	}
	LOGPLCHAN(lchan, DL1P, LOGL_NOTICE, "SACCH no SI available\n");
	return NULL;
}

void lchan_set_state(struct gsm_lchan *lchan, enum gsm_lchan_state state)
{
	if (lchan->state == state)
		return;
	LOGPLCHAN(lchan, DL1C, LOGL_INFO, "state %s -> %s\n",
		  gsm_lchans_name(lchan->state), gsm_lchans_name(state));
	lchan->state = state;

	switch (lchan->state) {
	case LCHAN_S_ACT_REQ:
		/* Early Immediate Assignment: Activation is requested, keep the
		 * early IA until active. This allows the BSC to send the IA
		 * even before a dynamic timeslot is done switching to a
		 * different pchan kind (experimental). */
		break;
	case LCHAN_S_ACTIVE:
		lchan_init_lapdm(lchan);
		if (lchan->early_rr_ia) {
			/* Early Immediate Assignment: Activation is done, send
			 * the RR IA now. Delay a bit more to give Um time to
			 * let the lchan light up for the MS */
			osmo_timer_del(&lchan->early_rr_ia_delay);
			osmo_timer_schedule(&lchan->early_rr_ia_delay, 0,
					    osmo_tdef_get(abis_T_defs, -15, OSMO_TDEF_US, -1));
		}
		break;
	case LCHAN_S_NONE:
		lapdm_channel_exit(&lchan->lapdm_ch);
		/* Also ensure that there are no leftovers from repeated FACCH or
		 * repeated SACCH that might cause memory leakage. */
		msgb_free(lchan->rep_acch.dl_facch[0].msg);
		msgb_free(lchan->rep_acch.dl_facch[1].msg);
		lchan->rep_acch.dl_facch[0].msg = NULL;
		lchan->rep_acch.dl_facch[1].msg = NULL;
		msgb_free(lchan->rep_acch.dl_sacch_msg);
		lchan->rep_acch.dl_sacch_msg = NULL;
		/* free() pending messages */
		msgb_free(lchan->pending_rel_ind_msg);
		lchan->pending_rel_ind_msg = NULL;
		msgb_free(lchan->pending_chan_activ);
		lchan->pending_chan_activ = NULL;
		/* fall through */
	default:
		if (lchan->early_rr_ia) {
			/* Early Immediate Assignment: Transition to any other
			 * state means whatever IA the BSC has sent shall now
			 * not be relevant anymore. */
			osmo_timer_del(&lchan->early_rr_ia_delay);
			msgb_free(lchan->early_rr_ia);
			lchan->early_rr_ia = NULL;
		}
		break;
	}
}

/* See 3GPP TS 44.018 Table 10.5.2.5.1 "Channel Description information element" */
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
		LOGP(DRSL, LOGL_ERROR, "ts=%u,ss=%u Physical channel %s not expected!\n",
		     ts_nr, lchan_nr, gsm_pchan_name(pchan));
		cbits = 0x00; /* Reserved */
		break;
	default:
		LOGP(DRSL, LOGL_ERROR, "ts=%u,ss=%u Physical channel %s (0x%02x) not expected!\n",
		     ts_nr, lchan_nr, gsm_pchan_name(pchan), (int)pchan);
		OSMO_ASSERT(0);
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
void gsm_lchan_interf_meas_calc_avg(struct gsm_lchan *lchan)
{
	const uint8_t meas_num = lchan->meas.interf_meas_num;
	const struct gsm_bts *bts = lchan->ts->trx->bts;
	int b, meas_avg, meas_sum = 0;

	/* There must be at least one sample */
	OSMO_ASSERT(meas_num > 0);

	/* Calculate the sum of all collected samples (in -x dBm) */
	while (lchan->meas.interf_meas_num) {
		uint8_t i = --lchan->meas.interf_meas_num;
		meas_sum += lchan->meas.interf_meas_dbm[i];
	}

	/* Calculate the average of all collected samples */
	meas_avg = meas_sum / (int) meas_num;

	/* 3GPP TS 48.008 defines 5 interference bands, and 6 interference level
	 * boundaries (0, X1, ... X5).  It's not clear how to handle values
	 * exceeding the outer boundaries (0 or X5), because bands 0 and 6 do
	 * not exist (sigh).  Let's map such values to closest bands 1 and 5. */
	if (bts->interference.boundary[0] < bts->interference.boundary[5]) {
		/* Ascending order (band=1 indicates lowest interference) */
		for (b = 1; b < ARRAY_SIZE(bts->interference.boundary) - 1; b++) {
			if (meas_avg < bts->interference.boundary[b])
				break; /* Current 'b' is the band value */
		}
	} else {
		/* Descending order (band=1 indicates highest interference) */
		for (b = 1; b < ARRAY_SIZE(bts->interference.boundary) - 1; b++) {
			if (meas_avg >= bts->interference.boundary[b])
				break; /* Current 'b' is the band value */
		}
	}

	LOGPLCHAN(lchan, DL1C, LOGL_DEBUG,
		  "Interference AVG: %ddBm (band %d, samples %u)\n",
		  meas_avg, b, meas_num);

	lchan->meas.interf_meas_avg_dbm = meas_avg;
	lchan->meas.interf_band = b;
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

static int bind_rtp(struct gsm_bts *bts, struct rtp_abst_socket *rs, const char *ip)
{
	int rc;
	unsigned int i;
	unsigned int tries;

	tries = (bts->rtp_port_range_end - bts->rtp_port_range_start) / 2;
	for (i = 0; i < tries; i++) {

		if (bts->rtp_port_range_next >= bts->rtp_port_range_end)
			bts->rtp_port_range_next = bts->rtp_port_range_start;

		rc = rtp_abst_socket_bind(rs, ip, bts->rtp_port_range_next);

		bts->rtp_port_range_next += 2;

		if (rc != 0)
			continue;

		if (bts->rtp_ip_dscp != -1) {
			if (rtp_abst_socket_set_dscp(rs, bts->rtp_ip_dscp))
				LOGP(DRSL, LOGL_ERROR, "failed to set DSCP=%d: %s\n",
					bts->rtp_ip_dscp, strerror(errno));
		}
		if (bts->rtp_priority != -1) {
			if (rtp_abst_socket_set_priority(rs, bts->rtp_priority))
				LOGP(DRSL, LOGL_ERROR, "failed to set socket priority %d: %s\n",
					bts->rtp_priority, strerror(errno));
		}
		return 0;
	}

	return -1;
}

int lchan_rtp_socket_create(struct gsm_lchan *lchan, const char *bind_ip)
{
	struct gsm_bts *bts = lchan->ts->trx->bts;
	char cname[256+4];
	int rc;

	if (lchan->abis_ip.rtp_socket) {
		LOGPLCHAN(lchan, DRSL, LOGL_ERROR, "Rx RSL IPAC CRCX, "
			  "but we already have socket!\n");
		return -EALREADY;
	}

	/* FIXME: select default value depending on speech_mode */
	//if (!payload_type)
	lchan->tch.last_fn = LCHAN_FN_DUMMY;
	lchan->abis_ip.rtp_socket = rtp_abst_socket_create(lchan->ts->trx,
					bts->use_twrtp, &bts->twjit_cfg);

	if (!lchan->abis_ip.rtp_socket) {
		LOGPLCHAN(lchan, DRTP, LOGL_ERROR, "IPAC Failed to create RTP/RTCP sockets\n");
		oml_tx_failure_event_rep(&lchan->ts->trx->mo,
					 NM_SEVER_MINOR, OSMO_EVT_CRIT_RTP_TOUT,
					 "%s IPAC Failed to create RTP/RTCP sockets",
					 gsm_lchan_name(lchan));
		return -ENOTCONN;
	}

	rc = rtp_abst_socket_set_param(lchan->abis_ip.rtp_socket,
				       bts->rtp_jitter_adaptive,
				       bts->rtp_jitter_buf_ms);
	if (rc < 0)
		LOGPLCHAN(lchan, DRTP, LOGL_ERROR,
			  "IPAC Failed to set RTP socket parameters: %s\n", strerror(-rc));
	else
		LOGPLCHAN(lchan, DRTP, LOGL_INFO, "IPAC set RTP socket parameters: %d\n", rc);

	lchan->abis_ip.rtp_socket->priv = lchan;
	lchan->abis_ip.rtp_socket->rx_cb = &l1sap_rtp_rx_cb;

	rc = bind_rtp(bts, lchan->abis_ip.rtp_socket, bind_ip);
	if (rc < 0) {
		LOGPLCHAN(lchan, DRTP, LOGL_ERROR, "IPAC Failed to bind RTP/RTCP sockets\n");
		oml_tx_failure_event_rep(&lchan->ts->trx->mo,
					 NM_SEVER_MINOR, OSMO_EVT_CRIT_RTP_TOUT,
					 "%s IPAC Failed to bind RTP/RTCP sockets",
					 gsm_lchan_name(lchan));
		lchan_rtp_socket_free(lchan);
		return -EBADFD;
	}

	/* Ensure RTCP SDES contains some useful information */
	snprintf(cname, sizeof(cname), "bts@%s", bind_ip);
	rtp_abst_set_source_desc(lchan->abis_ip.rtp_socket, cname,
				 gsm_lchan_name(lchan), NULL, NULL,
				 gsm_trx_unit_id(lchan->ts->trx),
				 "OsmoBTS-" PACKAGE_VERSION, NULL);
	/* FIXME: multiplex connection, BSC proxy */
	return 0;
}


int lchan_rtp_socket_connect(struct gsm_lchan *lchan, const struct in_addr *ia, uint16_t connect_port)
{
	int bound_port = 0;
	int rc;

	rc = rtp_abst_socket_connect(lchan->abis_ip.rtp_socket,
				     ia, connect_port);
	if (rc < 0) {
		LOGPLCHAN(lchan, DRTP, LOGL_ERROR, "Failed to connect RTP/RTCP sockets\n");
		return -ECONNREFUSED;
	}
	/* save IP address and port number */
	lchan->abis_ip.connect_ip = ntohl(ia->s_addr);
	lchan->abis_ip.connect_port = connect_port;

	rc = rtp_abst_get_bound_ip_port(lchan->abis_ip.rtp_socket,
					&lchan->abis_ip.bound_ip,
					&bound_port);
	if (rc < 0)
		LOGPLCHAN(lchan, DRTP, LOGL_ERROR, "IPAC cannot obtain locally bound IP/port: %d\n", rc);
	lchan->abis_ip.bound_port = bound_port;
	return 0;
}

void lchan_rtp_socket_free(struct gsm_lchan *lchan)
{
	rtp_abst_socket_free(lchan->abis_ip.rtp_socket);
	lchan->abis_ip.rtp_socket = NULL;
	msgb_queue_free(&lchan->dl_tch_queue);
	lchan->dl_tch_queue_len = 0;
}

/*! limit number of queue entries to %u; drops any surplus messages */
void lchan_dl_tch_queue_enqueue(struct gsm_lchan *lchan, struct msgb *msg, unsigned int limit)
{
	if (lchan->dl_tch_queue_len > limit) {
		unsigned int excess = lchan->dl_tch_queue_len - limit;
		LOGPLCHAN(lchan, DL1P, LOGL_NOTICE, "freeing %d queued frames\n", excess);
		rate_ctr_add2(lchan->ts->trx->bts->ctrs, BTS_CTR_RTP_RX_DROP_OVERFLOW, excess);
	}
	while (lchan->dl_tch_queue_len > limit) {
		struct msgb *tmp = msgb_dequeue_count(&lchan->dl_tch_queue, &lchan->dl_tch_queue_len);
		msgb_free(tmp);
	}
	msgb_enqueue_count(&lchan->dl_tch_queue, msg, &lchan->dl_tch_queue_len);
}
