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
	{ GSM_PCHAN_TCH_F_TCH_H_PDCH, "TCH/F_TCH/H_PDCH" },
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
	{ GSM_PCHAN_TCH_F_TCH_H_PDCH, "Dynamic TCH/F or TCH/H or GPRS PDCH" },
	{ 0,			NULL }
};

const char *gsm_pchan_name(enum gsm_phys_chan_config c)
{
	return get_value_string(gsm_pchant_names, c);
}

enum gsm_phys_chan_config gsm_pchan_parse(const char *name)
{
	return get_string_value(gsm_pchant_names, name);
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
	{ LCHAN_S_INACTIVE,	"INACTIVE" },
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
	snprintf(ts2str, sizeof(ts2str), "(bts=%d,trx=%d,ts=%d)",
		 ts->trx->bts->nr, ts->trx->nr, ts->nr);

	return ts2str;
}

/*! Log timeslot number with full pchan information */
char *gsm_ts_and_pchan_name(const struct gsm_bts_trx_ts *ts)
{
	switch (ts->pchan) {
	case GSM_PCHAN_TCH_F_TCH_H_PDCH:
		if (ts->dyn.pchan_is == ts->dyn.pchan_want)
			snprintf(ts2str, sizeof(ts2str),
				 "(bts=%d,trx=%d,ts=%d,pchan=%s as %s)",
				 ts->trx->bts->nr, ts->trx->nr, ts->nr,
				 gsm_pchan_name(ts->pchan),
				 gsm_pchan_name(ts->dyn.pchan_is));
		else
			snprintf(ts2str, sizeof(ts2str),
				 "(bts=%d,trx=%d,ts=%d,pchan=%s"
				 " switching %s -> %s)",
				 ts->trx->bts->nr, ts->trx->nr, ts->nr,
				 gsm_pchan_name(ts->pchan),
				 gsm_pchan_name(ts->dyn.pchan_is),
				 gsm_pchan_name(ts->dyn.pchan_want));
		break;
	case GSM_PCHAN_TCH_F_PDCH:
		if ((ts->flags & TS_F_PDCH_PENDING_MASK) == 0)
			snprintf(ts2str, sizeof(ts2str),
				 "(bts=%d,trx=%d,ts=%d,pchan=%s as %s)",
				 ts->trx->bts->nr, ts->trx->nr, ts->nr,
				 gsm_pchan_name(ts->pchan),
				 (ts->flags & TS_F_PDCH_ACTIVE)? "PDCH"
							       : "TCH/F");
		else
			snprintf(ts2str, sizeof(ts2str),
				 "(bts=%d,trx=%d,ts=%d,pchan=%s"
				 " switching %s -> %s)",
				 ts->trx->bts->nr, ts->trx->nr, ts->nr,
				 gsm_pchan_name(ts->pchan),
				 (ts->flags & TS_F_PDCH_ACTIVE)? "PDCH"
							       : "TCH/F",
				 (ts->flags & TS_F_PDCH_ACT_PENDING)? "PDCH"
								    : "TCH/F");
		break;
	default:
		snprintf(ts2str, sizeof(ts2str), "(bts=%d,trx=%d,ts=%d,pchan=%s)",
			 ts->trx->bts->nr, ts->trx->nr, ts->nr,
			 gsm_pchan_name(ts->pchan));
		break;
	}

	return ts2str;
}

char *gsm_lchan_name_compute(const struct gsm_lchan *lchan)
{
	struct gsm_bts_trx_ts *ts = lchan->ts;

	snprintf(ts2str, sizeof(ts2str), "(bts=%d,trx=%d,ts=%d,ss=%d)",
		 ts->trx->bts->nr, ts->trx->nr, ts->nr, lchan->nr);

	return ts2str;
}

/* See Table 10.5.25 of GSM04.08 */
static uint8_t gsm_pchan2chan_nr(enum gsm_phys_chan_config pchan,
			  uint8_t ts_nr, uint8_t lchan_nr)
{
	uint8_t cbits, chan_nr;

	OSMO_ASSERT(pchan != GSM_PCHAN_TCH_F_TCH_H_PDCH);
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
	switch (lchan->ts->pchan) {
	case GSM_PCHAN_TCH_F_TCH_H_PDCH:
		/* Return chan_nr reflecting the current TS pchan, either a standard TCH kind, or the
		 * nonstandard value reflecting PDCH for Osmocom style dyn TS. */
		return gsm_lchan_as_pchan2chan_nr(lchan,
						  lchan->ts->dyn.pchan_is);
	case GSM_PCHAN_TCH_F_PDCH:
		/* For ip.access style dyn TS, we always want to use the chan_nr as if it was TCH/F.
		 * We're using custom PDCH ACT and DEACT messages that use the usual chan_nr values. */
		return gsm_lchan_as_pchan2chan_nr(lchan, GSM_PCHAN_TCH_F);
	default:
		return gsm_pchan2chan_nr(lchan->ts->pchan, lchan->ts->nr, lchan->nr);
	}
}

uint8_t gsm_lchan_as_pchan2chan_nr(const struct gsm_lchan *lchan,
				   enum gsm_phys_chan_config as_pchan)
{
	if (lchan->ts->pchan == GSM_PCHAN_TCH_F_TCH_H_PDCH
	    && as_pchan == GSM_PCHAN_PDCH)
		return RSL_CHAN_OSMO_PDCH | (lchan->ts->nr & ~RSL_CHAN_NR_MASK);
	return gsm_pchan2chan_nr(as_pchan, lchan->ts->nr, lchan->nr);
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

	if (cbits == ABIS_RSL_CHAN_NR_CBITS_Bm_ACCHs) {
		lch_idx = 0;	/* TCH/F */
		if (ts->pchan != GSM_PCHAN_TCH_F &&
		    ts->pchan != GSM_PCHAN_PDCH &&
		    ts->pchan != GSM_PCHAN_TCH_F_PDCH &&
		    ts->pchan != GSM_PCHAN_TCH_F_TCH_H_PDCH)
			ok = false;
	} else if ((cbits & 0x1e) == ABIS_RSL_CHAN_NR_CBITS_Lm_ACCHs(0)) {
		lch_idx = cbits & 0x1;	/* TCH/H */
		if (ts->pchan != GSM_PCHAN_TCH_H &&
		    ts->pchan != GSM_PCHAN_TCH_F_TCH_H_PDCH)
			ok = false;
	} else if ((cbits & 0x1c) == ABIS_RSL_CHAN_NR_CBITS_SDCCH4_ACCH(0)) {
		lch_idx = cbits & 0x3;	/* SDCCH/4 */
		if (ts->pchan != GSM_PCHAN_CCCH_SDCCH4 &&
		    ts->pchan != GSM_PCHAN_CCCH_SDCCH4_CBCH)
			ok = false;
	} else if ((cbits & 0x18) == ABIS_RSL_CHAN_NR_CBITS_SDCCH8_ACCH(0)) {
		lch_idx = cbits & 0x7;	/* SDCCH/8 */
		if (ts->pchan != GSM_PCHAN_SDCCH8_SACCH8C &&
		    ts->pchan != GSM_PCHAN_SDCCH8_SACCH8C_CBCH)
			ok = false;
	} else if (cbits == 0x10 || cbits == 0x11 || cbits == 0x12) {
		lch_idx = 0;
		if (ts->pchan != GSM_PCHAN_CCCH &&
		    ts->pchan != GSM_PCHAN_CCCH_SDCCH4 &&
		    ts->pchan != GSM_PCHAN_CCCH_SDCCH4_CBCH)
			ok = false;
		/* FIXME: we should not return first sdcch4 !!! */
	} else if ((chan_nr & RSL_CHAN_NR_MASK) == RSL_CHAN_OSMO_PDCH) {
		lch_idx = 0;
		if (ts->pchan != GSM_PCHAN_TCH_F_TCH_H_PDCH)
			ok = false;
	} else
		return NULL;

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
	 * GSM_PCHAN_TCH_F_PDCH and GSM_PCHAN_TCH_F_TCH_H_PDCH should not be
	 * part of this, those TS are handled according to their dynamic state.
	 */
};

/*! Return the actual pchan type, also heeding dynamic TS. */
enum gsm_phys_chan_config ts_pchan(const struct gsm_bts_trx_ts *ts)
{
	switch (ts->pchan) {
	case GSM_PCHAN_TCH_F_TCH_H_PDCH:
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

		/* FIXME: RxQual averaging is not yet implemented */
		.algo = GSM_PWR_CTRL_MEAS_AVG_ALGO_NONE,
	},
};
