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
