/* (C) 2008-2010 by Harald Welte <laforge@gnumonks.org>
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


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <ctype.h>

#include <netinet/in.h>

#include <osmocom/core/linuxlist.h>
#include <osmocom/core/talloc.h>
#include <osmocom/core/statistics.h>
#include <osmocom/core/fsm.h>

#include <osmocom/gsm/gsm_utils.h>
#include <osmocom/gsm/abis_nm.h>
#include <osmocom/codec/ecu.h>

#include <osmo-bts/gsm_data.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/bts_trx.h>
#include <osmo-bts/logging.h>

struct osmo_tdef_group bts_tdef_groups[] = {
	{ .name = "bts", .tdefs = bts_T_defs, .desc = "BTS process timers" },
	{ .name = "abis", .tdefs = abis_T_defs, .desc = "Abis (RSL) related timers" },
	{}
};

const struct value_string gsm_pchant_names[13] = {
	{ GSM_PCHAN_NONE,	"NONE" },
	{ GSM_PCHAN_CCCH,	"CCCH" },
	{ GSM_PCHAN_CCCH_SDCCH4,"CCCH+SDCCH4" },
	{ GSM_PCHAN_TCH_F,	"TCH/F" },
	{ GSM_PCHAN_TCH_H,	"TCH/H" },
	{ GSM_PCHAN_SDCCH8_SACCH8C, "SDCCH8" },
	{ GSM_PCHAN_PDCH,	"PDCH" },
	{ GSM_PCHAN_TCH_F_PDCH,	"TCH/F_PDCH" },
	{ GSM_PCHAN_UNKNOWN,	"UNKNOWN" },
	{ GSM_PCHAN_CCCH_SDCCH4_CBCH, "CCCH+SDCCH4+CBCH" },
	{ GSM_PCHAN_SDCCH8_SACCH8C_CBCH, "SDCCH8+CBCH" },
	{ GSM_PCHAN_OSMO_DYN, "TCH/F_TCH/H_SDCCH8_PDCH" },
	{ 0,			NULL }
};

const struct value_string gsm_pchant_descs[13] = {
	{ GSM_PCHAN_NONE,	"Physical Channel not configured" },
	{ GSM_PCHAN_CCCH,	"FCCH + SCH + BCCH + CCCH (Comb. IV)" },
	{ GSM_PCHAN_CCCH_SDCCH4,
		"FCCH + SCH + BCCH + CCCH + 4 SDCCH + 2 SACCH (Comb. V)" },
	{ GSM_PCHAN_TCH_F,	"TCH/F + FACCH/F + SACCH (Comb. I)" },
	{ GSM_PCHAN_TCH_H,	"2 TCH/H + 2 FACCH/H + 2 SACCH (Comb. II)" },
	{ GSM_PCHAN_SDCCH8_SACCH8C, "8 SDCCH + 4 SACCH (Comb. VII)" },
	{ GSM_PCHAN_PDCH,	"Packet Data Channel for GPRS/EDGE" },
	{ GSM_PCHAN_TCH_F_PDCH,	"Dynamic TCH/F or GPRS PDCH" },
	{ GSM_PCHAN_UNKNOWN,	"Unknown / Unsupported channel combination" },
	{ GSM_PCHAN_CCCH_SDCCH4_CBCH, "FCCH + SCH + BCCH + CCCH + CBCH + 3 SDCCH + 2 SACCH (Comb. V)" },
	{ GSM_PCHAN_SDCCH8_SACCH8C_CBCH, "7 SDCCH + 4 SACCH + CBCH (Comb. VII)" },
	{ GSM_PCHAN_OSMO_DYN, "Dynamic TCH/F or TCH/H or SDCCH/8 or GPRS PDCH" },
	{ 0,			NULL }
};

const char *gsm_pchan_name(enum gsm_phys_chan_config c)
{
	return get_value_string(gsm_pchant_names, c);
}

/* TODO: move to libosmocore, next to gsm_chan_t_names? */
const char *gsm_lchant_name(enum gsm_chan_t c)
{
	return get_value_string(gsm_chan_t_names, c);
}

static const struct value_string lchan_s_names[] = {
	{ LCHAN_S_NONE,		"NONE" },
	{ LCHAN_S_ACT_REQ,	"ACTIVATION REQUESTED" },
	{ LCHAN_S_ACTIVE,	"ACTIVE" },
	{ LCHAN_S_REL_REQ,	"RELEASE REQUESTED" },
	{ LCHAN_S_REL_ERR,	"RELEASE DUE ERROR" },
	{ LCHAN_S_BROKEN,	"BROKEN UNUSABLE" },
	{ 0,			NULL }
};

const char *gsm_lchans_name(enum gsm_lchan_state s)
{
	return get_value_string(lchan_s_names, s);
}

static char ts2str[255];

char *gsm_ts_name(const struct gsm_bts_trx_ts *ts)
{
	snprintf(ts2str, sizeof(ts2str),
		 "(" GSM_TS_NAME_FMT ")",
		 GSM_TS_NAME_ARGS(ts));

	return ts2str;
}

/*! Log timeslot number with full pchan information */
char *gsm_ts_and_pchan_name(const struct gsm_bts_trx_ts *ts)
{
	switch (ts->pchan) {
	case GSM_PCHAN_OSMO_DYN:
		if (ts->dyn.pchan_is == ts->dyn.pchan_want)
			snprintf(ts2str, sizeof(ts2str),
				 "(" GSM_TS_NAME_FMT ",pchan=%s as %s)",
				 GSM_TS_NAME_ARGS(ts),
				 gsm_pchan_name(ts->pchan),
				 gsm_pchan_name(ts->dyn.pchan_is));
		else
			snprintf(ts2str, sizeof(ts2str),
				 "(" GSM_TS_NAME_FMT ",pchan=%s switching %s -> %s)",
				 GSM_TS_NAME_ARGS(ts),
				 gsm_pchan_name(ts->pchan),
				 gsm_pchan_name(ts->dyn.pchan_is),
				 gsm_pchan_name(ts->dyn.pchan_want));
		break;
	case GSM_PCHAN_TCH_F_PDCH:
		if ((ts->flags & TS_F_PDCH_PENDING_MASK) == 0)
			snprintf(ts2str, sizeof(ts2str),
				 "(" GSM_TS_NAME_FMT ",pchan=%s as %s)",
				 GSM_TS_NAME_ARGS(ts),
				 gsm_pchan_name(ts->pchan),
				 (ts->flags & TS_F_PDCH_ACTIVE)? "PDCH"
							       : "TCH/F");
		else
			snprintf(ts2str, sizeof(ts2str),
				 "(" GSM_TS_NAME_FMT ",pchan=%s switching %s -> %s)",
				 GSM_TS_NAME_ARGS(ts),
				 gsm_pchan_name(ts->pchan),
				 (ts->flags & TS_F_PDCH_ACTIVE)? "PDCH"
							       : "TCH/F",
				 (ts->flags & TS_F_PDCH_ACT_PENDING)? "PDCH"
								    : "TCH/F");
		break;
	default:
		snprintf(ts2str, sizeof(ts2str), "(" GSM_TS_NAME_FMT ",pchan=%s)",
			 GSM_TS_NAME_ARGS(ts), gsm_pchan_name(ts->pchan));
		break;
	}

	return ts2str;
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

uint8_t gsm_lchan2chan_nr(const struct gsm_lchan *lchan)
{
	uint8_t chan_nr;

	switch (lchan->ts->pchan) {
	case GSM_PCHAN_OSMO_DYN:
		/* Return chan_nr reflecting the current TS pchan, either a standard TCH kind, or the
		 * nonstandard value reflecting PDCH for Osmocom style dyn TS. */
		chan_nr = gsm_lchan_as_pchan2chan_nr(lchan, lchan->ts->dyn.pchan_is);
		break;
	case GSM_PCHAN_TCH_F_PDCH:
		/* For ip.access style dyn TS, we always want to use the chan_nr as if it was TCH/F.
		 * We're using custom PDCH ACT and DEACT messages that use the usual chan_nr values. */
		chan_nr = gsm_lchan_as_pchan2chan_nr(lchan, GSM_PCHAN_TCH_F);
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

/* determine logical channel based on TRX and channel number IE */
struct gsm_lchan *rsl_lchan_lookup(struct gsm_bts_trx *trx, uint8_t chan_nr,
				   int *rc)
{
	uint8_t ts_nr = chan_nr & 0x07;
	uint8_t cbits = chan_nr >> 3;
	uint8_t lch_idx;
	struct gsm_bts_trx_ts *ts = &trx->ts[ts_nr];
	bool ok = true;

	if (rc)
		*rc = -EINVAL;

	switch (cbits) {
	case ABIS_RSL_CHAN_NR_CBITS_OSMO_VAMOS_Bm_ACCHs:
		if (ts->vamos.peer == NULL)
			return NULL;
		ts = ts->vamos.peer;
		/* fall-through */
	case ABIS_RSL_CHAN_NR_CBITS_Bm_ACCHs:
		lch_idx = 0;	/* TCH/F */
		if (ts->pchan != GSM_PCHAN_TCH_F &&
		    ts->pchan != GSM_PCHAN_PDCH &&
		    ts->pchan != GSM_PCHAN_TCH_F_PDCH &&
		    ts->pchan != GSM_PCHAN_OSMO_DYN)
			ok = false;
		break;
	case ABIS_RSL_CHAN_NR_CBITS_OSMO_VAMOS_Lm_ACCHs(0):
	case ABIS_RSL_CHAN_NR_CBITS_OSMO_VAMOS_Lm_ACCHs(1):
		if (ts->vamos.peer == NULL)
			return NULL;
		ts = ts->vamos.peer;
		/* fall-through */
	case ABIS_RSL_CHAN_NR_CBITS_Lm_ACCHs(0):
	case ABIS_RSL_CHAN_NR_CBITS_Lm_ACCHs(1):
		lch_idx = cbits & 0x1;	/* TCH/H */
		if (ts->pchan != GSM_PCHAN_TCH_H &&
		    ts->pchan != GSM_PCHAN_OSMO_DYN)
			ok = false;
		break;
	case ABIS_RSL_CHAN_NR_CBITS_SDCCH4_ACCH(0):
	case ABIS_RSL_CHAN_NR_CBITS_SDCCH4_ACCH(1):
	case ABIS_RSL_CHAN_NR_CBITS_SDCCH4_ACCH(2):
	case ABIS_RSL_CHAN_NR_CBITS_SDCCH4_ACCH(3):
		lch_idx = cbits & 0x3;	/* SDCCH/4 */
		if (ts->pchan != GSM_PCHAN_CCCH_SDCCH4 &&
		    ts->pchan != GSM_PCHAN_CCCH_SDCCH4_CBCH)
			ok = false;
		break;
	case ABIS_RSL_CHAN_NR_CBITS_SDCCH8_ACCH(0):
	case ABIS_RSL_CHAN_NR_CBITS_SDCCH8_ACCH(1):
	case ABIS_RSL_CHAN_NR_CBITS_SDCCH8_ACCH(2):
	case ABIS_RSL_CHAN_NR_CBITS_SDCCH8_ACCH(3):
	case ABIS_RSL_CHAN_NR_CBITS_SDCCH8_ACCH(4):
	case ABIS_RSL_CHAN_NR_CBITS_SDCCH8_ACCH(5):
	case ABIS_RSL_CHAN_NR_CBITS_SDCCH8_ACCH(6):
	case ABIS_RSL_CHAN_NR_CBITS_SDCCH8_ACCH(7):
		lch_idx = cbits & 0x7;	/* SDCCH/8 */
		if (ts->pchan != GSM_PCHAN_SDCCH8_SACCH8C &&
		    ts->pchan != GSM_PCHAN_SDCCH8_SACCH8C_CBCH &&
		    ts->pchan != GSM_PCHAN_OSMO_DYN)
			ok = false;
		break;
	case ABIS_RSL_CHAN_NR_CBITS_BCCH:
	case ABIS_RSL_CHAN_NR_CBITS_RACH:
	case ABIS_RSL_CHAN_NR_CBITS_PCH_AGCH:
		lch_idx = 0;
		if (ts->pchan != GSM_PCHAN_CCCH &&
		    ts->pchan != GSM_PCHAN_CCCH_SDCCH4 &&
		    ts->pchan != GSM_PCHAN_CCCH_SDCCH4_CBCH)
			ok = false;
		/* FIXME: we should not return first sdcch4 !!! */
		break;
	case ABIS_RSL_CHAN_NR_CBITS_OSMO_PDCH:
		lch_idx = 0;
		if (ts->pchan != GSM_PCHAN_OSMO_DYN)
			ok = false;
		break;
	default:
		return NULL;
	}

	if (rc && ok)
		*rc = 0;

	return &ts->lchan[lch_idx];
}

static const uint8_t subslots_per_pchan[] = {
	[GSM_PCHAN_NONE] = 0,
	[GSM_PCHAN_CCCH] = 0,
	[GSM_PCHAN_PDCH] = 0,
	[GSM_PCHAN_CCCH_SDCCH4] = 4,
	[GSM_PCHAN_TCH_F] = 1,
	[GSM_PCHAN_TCH_H] = 2,
	[GSM_PCHAN_SDCCH8_SACCH8C] = 8,
	[GSM_PCHAN_CCCH_SDCCH4_CBCH] = 4,
	[GSM_PCHAN_SDCCH8_SACCH8C_CBCH] = 8,
	/*
	 * GSM_PCHAN_TCH_F_PDCH and GSM_PCHAN_OSMO_DYN should not be
	 * part of this, those TS are handled according to their dynamic state.
	 */
};

/*! Return the actual pchan type, also heeding dynamic TS. */
enum gsm_phys_chan_config ts_pchan(const struct gsm_bts_trx_ts *ts)
{
	switch (ts->pchan) {
	case GSM_PCHAN_OSMO_DYN:
		return ts->dyn.pchan_is;
	case GSM_PCHAN_TCH_F_PDCH:
		if (ts->flags & TS_F_PDCH_ACTIVE)
			return GSM_PCHAN_PDCH;
		else
			return GSM_PCHAN_TCH_F;
	default:
		return ts->pchan;
	}
}

/*! According to ts->pchan and possibly ts->dyn_pchan, return the number of
 * logical channels available in the timeslot. */
uint8_t ts_subslots(const struct gsm_bts_trx_ts *ts)
{
	return subslots_per_pchan[ts_pchan(ts)];
}

static bool pchan_is_tch(enum gsm_phys_chan_config pchan)
{
	switch (pchan) {
	case GSM_PCHAN_TCH_F:
	case GSM_PCHAN_TCH_H:
		return true;
	default:
		return false;
	}
}

bool ts_is_tch(const struct gsm_bts_trx_ts *ts)
{
	return pchan_is_tch(ts_pchan(ts));
}

const struct value_string lchan_ciph_state_names[] = {
	{ LCHAN_CIPH_NONE,	"NONE" },
	{ LCHAN_CIPH_RX_REQ,	"RX_REQ" },
	{ LCHAN_CIPH_RX_CONF,	"RX_CONF" },
	{ LCHAN_CIPH_RXTX_REQ,	"RXTX_REQ" },
	{ LCHAN_CIPH_RX_CONF_TX_REQ,	"RX_CONF_TX_REQ" },
	{ LCHAN_CIPH_RXTX_CONF,	"RXTX_CONF" },
	{ 0, NULL }
};

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

/* Default MS/BS Power Control parameters (see 3GPP TS 45.008, table A.1) */
const struct gsm_power_ctrl_params power_ctrl_params_def = {

	.ctrl_interval = 1, /* Trigger loop every second SACCH block. TS 45.008 sec 4.7.1 */

	/* Power increasing/reducing step size (optimal defaults) */
	.inc_step_size_db = 4, /* quickly increase MS/BS power */
	.red_step_size_db = 2, /* slowly decrease MS/BS power */

	/* RxLev measurement parameters */
	.rxlev_meas = {
		/* Thresholds for RxLev (see 3GPP TS 45.008, A.3.2.1) */
		.lower_thresh = 32, /* L_RXLEV_XX_P (-78 dBm) */
		.upper_thresh = 38, /* U_RXLEV_XX_P (-72 dBm) */

		/* NOTE: only Osmocom specific EWMA is supported */
		.algo = GSM_PWR_CTRL_MEAS_AVG_ALGO_OSMO_EWMA,
		.ewma.alpha = 50, /* Smoothing factor 50% */
	},

	/* RxQual measurement parameters */
	.rxqual_meas = {
		/* Thresholds for RxQual (see 3GPP TS 45.008, A.3.2.1) */
		.lower_thresh = 3, /* L_RXQUAL_XX_P (0.8% <= BER < 1.6%) */
		.upper_thresh = 0, /* U_RXQUAL_XX_P (BER < 0.2%) */

		/* No averaging (filtering) by default.
		 * NOTE: only Osmocom specific EWMA is supported */
		.algo = GSM_PWR_CTRL_MEAS_AVG_ALGO_NONE,
	},

	/* C/I measurement parameters.
	 * Target C/I retrieved from "GSM/EDGE: Evolution and Performance" Table 10.3.
	 * Set lower and upper so that (lower + upper) / 2 is equal or slightly
	 * above the target.
	 */
	.ci_fr_meas = { /* FR: Target C/I = 15 dB, Soft blocking threshold = 10 dB */
		.lower_thresh = 13,
		.upper_thresh = 17,

		/* Increase {UL,DL}_TXPWR if at least LOWER_CMP_P averages
		 * out of LOWER_CMP_N averages are lower than L_CI_FR_XX_P */
		.lower_cmp_p = 5, /* P3 as in 3GPP TS 45.008, A.3.2.1 (case c) */
		.lower_cmp_n = 7, /* N3 as in 3GPP TS 45.008, A.3.2.1 (case c) */
		/* Decrease {UL,DL}_TXPWR if at least UPPER_CMP_P averages
		 * out of UPPER_CMP_N averages are greater than L_CI_FR_XX_P */
		.upper_cmp_p = 15, /* P4 as in 3GPP TS 45.008, A.3.2.1 (case d) */
		.upper_cmp_n = 18, /* N4 as in 3GPP TS 45.008, A.3.2.1 (case d) */

		/* No averaging (filtering) by default */
		.algo = GSM_PWR_CTRL_MEAS_AVG_ALGO_NONE,

		/* Hreqave: the period over which an average is produced */
		.h_reqave = 4, /* TODO: investigate a reasonable default value */
		/* Hreqt: the number of averaged results maintained */
		.h_reqt = 6, /* TODO: investigate a reasonable default value */
	},
	.ci_hr_meas = { /* HR: Target C/I = 18 dB, Soft blocking threshold = 13 dB */
		.lower_thresh = 16,
		.upper_thresh = 21,

		/* Increase {UL,DL}_TXPWR if at least LOWER_CMP_P averages
		 * out of LOWER_CMP_N averages are lower than L_CI_HR_XX_P */
		.lower_cmp_p = 5, /* P3 as in 3GPP TS 45.008, A.3.2.1 (case c) */
		.lower_cmp_n = 7, /* N3 as in 3GPP TS 45.008, A.3.2.1 (case c) */
		/* Decrease {UL,DL}_TXPWR if at least UPPER_CMP_P averages
		 * out of UPPER_CMP_N averages are greater than L_CI_HR_XX_P */
		.upper_cmp_p = 15, /* P4 as in 3GPP TS 45.008, A.3.2.1 (case d) */
		.upper_cmp_n = 18, /* N4 as in 3GPP TS 45.008, A.3.2.1 (case d) */

		/* No averaging (filtering) by default */
		.algo = GSM_PWR_CTRL_MEAS_AVG_ALGO_NONE,

		/* Hreqave: the period over which an average is produced */
		.h_reqave = 4, /* TODO: investigate a reasonable default value */
		/* Hreqt: the number of averaged results maintained */
		.h_reqt = 6, /* TODO: investigate a reasonable default value */
	},
	.ci_amr_fr_meas = { /* AMR-FR: Target C/I = 9 dB, Soft blocking threshold = 4 dB */
		.lower_thresh = 7,
		.upper_thresh = 11,

		/* Increase {UL,DL}_TXPWR if at least LOWER_CMP_P averages
		 * out of LOWER_CMP_N averages are lower than L_CI_AMR_FR_XX_P */
		.lower_cmp_p = 5, /* P3 as in 3GPP TS 45.008, A.3.2.1 (case c) */
		.lower_cmp_n = 7, /* N3 as in 3GPP TS 45.008, A.3.2.1 (case c) */
		/* Decrease {UL,DL}_TXPWR if at least UPPER_CMP_P averages
		 * out of UPPER_CMP_N averages are greater than L_CI_AMR_FR_XX_P */
		.upper_cmp_p = 15, /* P4 as in 3GPP TS 45.008, A.3.2.1 (case d) */
		.upper_cmp_n = 18, /* N4 as in 3GPP TS 45.008, A.3.2.1 (case d) */

		/* No averaging (filtering) by default */
		.algo = GSM_PWR_CTRL_MEAS_AVG_ALGO_NONE,

		/* Hreqave: the period over which an average is produced */
		.h_reqave = 4, /* TODO: investigate a reasonable default value */
		/* Hreqt: the number of averaged results maintained */
		.h_reqt = 6, /* TODO: investigate a reasonable default value */
	},
	.ci_amr_hr_meas = { /* AMR-HR: Target C/I = 15 dB, Soft blocking threshold = 10 dB */
		.lower_thresh = 13,
		.upper_thresh = 17,

		/* Increase {UL,DL}_TXPWR if at least LOWER_CMP_P averages
		 * out of LOWER_CMP_N averages are lower than L_CI_AMR_HR_XX_P */
		.lower_cmp_p = 5, /* P3 as in 3GPP TS 45.008, A.3.2.1 (case c) */
		.lower_cmp_n = 7, /* N3 as in 3GPP TS 45.008, A.3.2.1 (case c) */
		/* Decrease {UL,DL}_TXPWR if at least UPPER_CMP_P averages
		 * out of UPPER_CMP_N averages are greater than L_CI_AMR_HR_XX_P */
		.upper_cmp_p = 15, /* P4 as in 3GPP TS 45.008, A.3.2.1 (case d) */
		.upper_cmp_n = 18, /* N4 as in 3GPP TS 45.008, A.3.2.1 (case d) */

		/* No averaging (filtering) by default */
		.algo = GSM_PWR_CTRL_MEAS_AVG_ALGO_NONE,

		/* Hreqave: the period over which an average is produced */
		.h_reqave = 4, /* TODO: investigate a reasonable default value */
		/* Hreqt: the number of averaged results maintained */
		.h_reqt = 6, /* TODO: investigate a reasonable default value */
	},
	.ci_sdcch_meas = { /* SDCCH: Target C/I = 14 dB, Soft blocking threshold = 9 dB */
		.lower_thresh = 12,
		.upper_thresh = 16,

		/* Increase {UL,DL}_TXPWR if at least LOWER_CMP_P averages
		 * out of LOWER_CMP_N averages are lower than L_CI_SDCCH_XX_P */
		.lower_cmp_p = 5, /* P3 as in 3GPP TS 45.008, A.3.2.1 (case c) */
		.lower_cmp_n = 7, /* N3 as in 3GPP TS 45.008, A.3.2.1 (case c) */
		/* Decrease {UL,DL}_TXPWR if at least UPPER_CMP_P averages
		 * out of UPPER_CMP_N averages are greater than L_CI_SDCCH_XX_P */
		.upper_cmp_p = 15, /* P4 as in 3GPP TS 45.008, A.3.2.1 (case d) */
		.upper_cmp_n = 18, /* N4 as in 3GPP TS 45.008, A.3.2.1 (case d) */

		/* No averaging (filtering) by default */
		.algo = GSM_PWR_CTRL_MEAS_AVG_ALGO_NONE,

		/* Hreqave: the period over which an average is produced */
		.h_reqave = 4, /* TODO: investigate a reasonable default value */
		/* Hreqt: the number of averaged results maintained */
		.h_reqt = 6, /* TODO: investigate a reasonable default value */
	},
	.ci_gprs_meas = { /* GPRS: Target C/I = 20 dB, Soft blocking threshold = 15 dB */
		.lower_thresh = 18,
		.upper_thresh = 24,

		/* Increase {UL,DL}_TXPWR if at least LOWER_CMP_P averages
		 * out of LOWER_CMP_N averages are lower than L_CI_GPRS_XX_P */
		.lower_cmp_p = 5, /* P3 as in 3GPP TS 45.008, A.3.2.1 (case c) */
		.lower_cmp_n = 7, /* N3 as in 3GPP TS 45.008, A.3.2.1 (case c) */
		/* Decrease {UL,DL}_TXPWR if at least UPPER_CMP_P averages
		 * out of UPPER_CMP_N averages are greater than L_CI_GPRS_XX_P */
		.upper_cmp_p = 15, /* P4 as in 3GPP TS 45.008, A.3.2.1 (case d) */
		.upper_cmp_n = 18, /* N4 as in 3GPP TS 45.008, A.3.2.1 (case d) */

		/* No averaging (filtering) by default */
		.algo = GSM_PWR_CTRL_MEAS_AVG_ALGO_NONE,

		/* Hreqave: the period over which an average is produced */
		.h_reqave = 4, /* TODO: investigate a reasonable default value */
		/* Hreqt: the number of averaged results maintained */
		.h_reqt = 6, /* TODO: investigate a reasonable default value */
	},
};
