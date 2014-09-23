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
#include <osmocom/gsm/gsm_utils.h>
#include <osmocom/gsm/abis_nm.h>
#include <osmocom/core/statistics.h>

#include <openbsc/gsm_data.h>

void gsm_abis_mo_reset(struct gsm_abis_mo *mo)
{
	mo->nm_state.operational = NM_OPSTATE_NULL;
	mo->nm_state.availability = NM_AVSTATE_POWER_OFF;
}

static void gsm_mo_init(struct gsm_abis_mo *mo, struct gsm_bts *bts,
			uint8_t obj_class, uint8_t p1, uint8_t p2, uint8_t p3)
{
	mo->bts = bts;
	mo->obj_class = obj_class;
	mo->obj_inst.bts_nr = p1;
	mo->obj_inst.trx_nr = p2;
	mo->obj_inst.ts_nr = p3;
	gsm_abis_mo_reset(mo);
}

const struct value_string gsm_pchant_names[10] = {
	{ GSM_PCHAN_NONE,	"NONE" },
	{ GSM_PCHAN_CCCH,	"CCCH" },
	{ GSM_PCHAN_CCCH_SDCCH4,"CCCH+SDCCH4" },
	{ GSM_PCHAN_TCH_F,	"TCH/F" },
	{ GSM_PCHAN_TCH_H,	"TCH/H" },
	{ GSM_PCHAN_SDCCH8_SACCH8C, "SDCCH8" },
	{ GSM_PCHAN_PDCH,	"PDCH" },
	{ GSM_PCHAN_TCH_F_PDCH,	"TCH/F_PDCH" },
	{ GSM_PCHAN_UNKNOWN,	"UNKNOWN" },
	{ 0,			NULL }
};

const struct value_string gsm_pchant_descs[10] = {
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

const struct value_string gsm_lchant_names[6] = {
	{ GSM_LCHAN_NONE,	"NONE" },
	{ GSM_LCHAN_SDCCH,	"SDCCH" },
	{ GSM_LCHAN_TCH_F,	"TCH/F" },
	{ GSM_LCHAN_TCH_H,	"TCH/H" },
	{ GSM_LCHAN_UNKNOWN,	"UNKNOWN" },
	{ 0,			NULL }
};

const char *gsm_lchant_name(enum gsm_chan_t c)
{
	return get_value_string(gsm_lchant_names, c);
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

static const struct value_string chreq_names[] = {
	{ GSM_CHREQ_REASON_EMERG,	"EMERGENCY" },
	{ GSM_CHREQ_REASON_PAG,		"PAGING" },
	{ GSM_CHREQ_REASON_CALL,	"CALL" },
	{ GSM_CHREQ_REASON_LOCATION_UPD,"LOCATION_UPDATE" },
	{ GSM_CHREQ_REASON_OTHER,	"OTHER" },
	{ 0,				NULL }
};

const char *gsm_chreq_name(enum gsm_chreq_reason_t c)
{
	return get_value_string(chreq_names, c);
}

struct gsm_bts_trx *gsm_bts_trx_alloc(struct gsm_bts *bts)
{
	struct gsm_bts_trx *trx = talloc_zero(bts, struct gsm_bts_trx);
	int k;

	if (!trx)
		return NULL;

	trx->bts = bts;
	trx->nr = bts->num_trx++;
	trx->mo.nm_state.administrative = NM_STATE_UNLOCKED;

	gsm_mo_init(&trx->mo, bts, NM_OC_RADIO_CARRIER,
		    bts->nr, trx->nr, 0xff);
	gsm_mo_init(&trx->bb_transc.mo, bts, NM_OC_BASEB_TRANSC,
		    bts->nr, trx->nr, 0xff);

	for (k = 0; k < TRX_NR_TS; k++) {
		struct gsm_bts_trx_ts *ts = &trx->ts[k];
		int l;

		ts->trx = trx;
		ts->nr = k;
		ts->pchan = GSM_PCHAN_NONE;
		ts->tsc = -1;

		gsm_mo_init(&ts->mo, bts, NM_OC_CHANNEL,
			    bts->nr, trx->nr, ts->nr);

		ts->hopping.arfcns.data_len = sizeof(ts->hopping.arfcns_data);
		ts->hopping.arfcns.data = ts->hopping.arfcns_data;
		ts->hopping.ma.data_len = sizeof(ts->hopping.ma_data);
		ts->hopping.ma.data = ts->hopping.ma_data;

		for (l = 0; l < TS_MAX_LCHAN; l++) {
			struct gsm_lchan *lchan;
			lchan = &ts->lchan[l];

			lchan->ts = ts;
			lchan->nr = l;
			lchan->type = GSM_LCHAN_NONE;

#ifndef ROLE_BSC
			INIT_LLIST_HEAD(&lchan->sapi_cmds);
#endif
		}
	}

	if (trx->nr != 0)
		trx->nominal_power = bts->c0->nominal_power;

	llist_add_tail(&trx->list, &bts->trx_list);

	return trx;
}


static const uint8_t bts_nse_timer_default[] = { 3, 3, 3, 3, 30, 3, 10 };
static const uint8_t bts_cell_timer_default[] =
				{ 3, 3, 3, 3, 3, 10, 3, 10, 3, 10, 3 };
static const struct gprs_rlc_cfg rlc_cfg_default = {
	.parameter = {
		[RLC_T3142] = 20,
		[RLC_T3169] = 5,
		[RLC_T3191] = 5,
		[RLC_T3193] = 160, /* 10ms */
		[RLC_T3195] = 5,
		[RLC_N3101] = 10,
		[RLC_N3103] = 4,
		[RLC_N3105] = 8,
		[CV_COUNTDOWN] = 15,
		[T_DL_TBF_EXT] = 250 * 10, /* ms */
		[T_UL_TBF_EXT] = 250 * 10, /* ms */
	},
	.paging = {
		.repeat_time = 5 * 50, /* ms */
		.repeat_count = 3,
	},
	.cs_mask = 0x1fff,
	.initial_cs = 2,
	.initial_mcs = 6,
};

struct gsm_bts *gsm_bts_alloc(void *ctx)
{
	struct gsm_bts *bts = talloc_zero(ctx, struct gsm_bts);
	int i;

	if (!bts)
		return NULL;

	bts->num_trx = 0;
	INIT_LLIST_HEAD(&bts->trx_list);
	bts->ms_max_power = 15;	/* dBm */

	gsm_mo_init(&bts->mo, bts, NM_OC_BTS,
			bts->nr, 0xff, 0xff);
	gsm_mo_init(&bts->site_mgr.mo, bts, NM_OC_SITE_MANAGER,
			0xff, 0xff, 0xff);

	for (i = 0; i < ARRAY_SIZE(bts->gprs.nsvc); i++) {
		bts->gprs.nsvc[i].bts = bts;
		bts->gprs.nsvc[i].id = i;
		gsm_mo_init(&bts->gprs.nsvc[i].mo, bts, NM_OC_GPRS_NSVC,
				bts->nr, i, 0xff);
	}
	memcpy(&bts->gprs.nse.timer, bts_nse_timer_default,
		sizeof(bts->gprs.nse.timer));
	gsm_mo_init(&bts->gprs.nse.mo, bts, NM_OC_GPRS_NSE,
			bts->nr, 0xff, 0xff);
	memcpy(&bts->gprs.cell.timer, bts_cell_timer_default,
		sizeof(bts->gprs.cell.timer));
	gsm_mo_init(&bts->gprs.cell.mo, bts, NM_OC_GPRS_CELL,
			bts->nr, 0xff, 0xff);
	memcpy(&bts->gprs.cell.rlc_cfg, &rlc_cfg_default,
		sizeof(bts->gprs.cell.rlc_cfg));

	/* create our primary TRX */
	bts->c0 = gsm_bts_trx_alloc(bts);
	if (!bts->c0) {
		talloc_free(bts);
		return NULL;
	}
	bts->c0->ts[0].pchan = GSM_PCHAN_CCCH_SDCCH4;

	bts->rach_b_thresh = -1;
	bts->rach_ldavg_slots = -1;
	bts->paging.free_chans_need = -1;

	return bts;
}

/* reset the state of all MO in the BTS */
void gsm_bts_mo_reset(struct gsm_bts *bts)
{
	struct gsm_bts_trx *trx;
	unsigned int i;

	gsm_abis_mo_reset(&bts->mo);
	gsm_abis_mo_reset(&bts->site_mgr.mo);
	for (i = 0; i < ARRAY_SIZE(bts->gprs.nsvc); i++)
		gsm_abis_mo_reset(&bts->gprs.nsvc[i].mo);
	gsm_abis_mo_reset(&bts->gprs.nse.mo);
	gsm_abis_mo_reset(&bts->gprs.cell.mo);

	llist_for_each_entry(trx, &bts->trx_list, list) {
		gsm_abis_mo_reset(&trx->mo);
		gsm_abis_mo_reset(&trx->bb_transc.mo);

		for (i = 0; i < ARRAY_SIZE(trx->ts); i++) {
			struct gsm_bts_trx_ts *ts = &trx->ts[i];
			gsm_abis_mo_reset(&ts->mo);
		}
	}
}

struct gsm_bts_trx *gsm_bts_trx_num(const struct gsm_bts *bts, int num)
{
	struct gsm_bts_trx *trx;

	if (num >= bts->num_trx)
		return NULL;

	llist_for_each_entry(trx, &bts->trx_list, list) {
		if (trx->nr == num)
			return trx;
	}

	return NULL;
}

static char ts2str[255];

char *gsm_trx_name(const struct gsm_bts_trx *trx)
{
	snprintf(ts2str, sizeof(ts2str), "(bts=%d,trx=%d)",
		 trx->bts->nr, trx->nr);

	return ts2str;
}


char *gsm_ts_name(const struct gsm_bts_trx_ts *ts)
{
	snprintf(ts2str, sizeof(ts2str), "(bts=%d,trx=%d,ts=%d)",
		 ts->trx->bts->nr, ts->trx->nr, ts->nr);

	return ts2str;
}

char *gsm_lchan_name(const struct gsm_lchan *lchan)
{
	struct gsm_bts_trx_ts *ts = lchan->ts;

	snprintf(ts2str, sizeof(ts2str), "(bts=%d,trx=%d,ts=%d,ss=%d)",
		 ts->trx->bts->nr, ts->trx->nr, ts->nr, lchan->nr);

	return ts2str;
}

/* obtain the MO structure for a given object instance */
struct gsm_abis_mo *
gsm_objclass2mo(struct gsm_bts *bts, uint8_t obj_class,
	    const struct abis_om_obj_inst *obj_inst)
{
	struct gsm_bts_trx *trx;
	struct gsm_abis_mo *mo = NULL;

	switch (obj_class) {
	case NM_OC_BTS:
		mo = &bts->mo;
		break;
	case NM_OC_RADIO_CARRIER:
		if (obj_inst->trx_nr >= bts->num_trx) {
			return NULL;
		}
		trx = gsm_bts_trx_num(bts, obj_inst->trx_nr);
		mo = &trx->mo;
		break;
	case NM_OC_BASEB_TRANSC:
		if (obj_inst->trx_nr >= bts->num_trx) {
			return NULL;
		}
		trx = gsm_bts_trx_num(bts, obj_inst->trx_nr);
		mo = &trx->bb_transc.mo;
		break;
	case NM_OC_CHANNEL:
		if (obj_inst->trx_nr >= bts->num_trx) {
			return NULL;
		}
		trx = gsm_bts_trx_num(bts, obj_inst->trx_nr);
		if (obj_inst->ts_nr >= TRX_NR_TS)
			return NULL;
		mo = &trx->ts[obj_inst->ts_nr].mo;
		break;
	case NM_OC_SITE_MANAGER:
		mo = &bts->site_mgr.mo;
		break;
	case NM_OC_BS11:
		switch (obj_inst->bts_nr) {
		case BS11_OBJ_CCLK:
			mo = &bts->bs11.cclk.mo;
			break;
		case BS11_OBJ_BBSIG:
			if (obj_inst->ts_nr > bts->num_trx)
				return NULL;
			trx = gsm_bts_trx_num(bts, obj_inst->trx_nr);
			mo = &trx->bs11.bbsig.mo;
			break;
		case BS11_OBJ_PA:
			if (obj_inst->ts_nr > bts->num_trx)
				return NULL;
			trx = gsm_bts_trx_num(bts, obj_inst->trx_nr);
			mo = &trx->bs11.pa.mo;
			break;
		default:
			return NULL;
		}
		break;
	case NM_OC_BS11_RACK:
		mo = &bts->bs11.rack.mo;
		break;
	case NM_OC_BS11_ENVABTSE:
		if (obj_inst->trx_nr >= ARRAY_SIZE(bts->bs11.envabtse))
			return NULL;
		mo = &bts->bs11.envabtse[obj_inst->trx_nr].mo;
		break;
	case NM_OC_GPRS_NSE:
		mo = &bts->gprs.nse.mo;
		break;
	case NM_OC_GPRS_CELL:
		mo = &bts->gprs.cell.mo;
		break;
	case NM_OC_GPRS_NSVC:
		if (obj_inst->trx_nr >= ARRAY_SIZE(bts->gprs.nsvc))
			return NULL;
		mo = &bts->gprs.nsvc[obj_inst->trx_nr].mo;
		break;
	}
	return mo;
}

/* obtain the gsm_nm_state data structure for a given object instance */
struct gsm_nm_state *
gsm_objclass2nmstate(struct gsm_bts *bts, uint8_t obj_class,
		 const struct abis_om_obj_inst *obj_inst)
{
	struct gsm_abis_mo *mo;

	mo = gsm_objclass2mo(bts, obj_class, obj_inst);
	if (!mo)
		return NULL;

	return &mo->nm_state;
}

/* obtain the in-memory data structure of a given object instance */
void *
gsm_objclass2obj(struct gsm_bts *bts, uint8_t obj_class,
	     const struct abis_om_obj_inst *obj_inst)
{
	struct gsm_bts_trx *trx;
	void *obj = NULL;

	switch (obj_class) {
	case NM_OC_BTS:
		obj = bts;
		break;
	case NM_OC_RADIO_CARRIER:
		if (obj_inst->trx_nr >= bts->num_trx) {
			return NULL;
		}
		trx = gsm_bts_trx_num(bts, obj_inst->trx_nr);
		obj = trx;
		break;
	case NM_OC_BASEB_TRANSC:
		if (obj_inst->trx_nr >= bts->num_trx) {
			return NULL;
		}
		trx = gsm_bts_trx_num(bts, obj_inst->trx_nr);
		obj = &trx->bb_transc;
		break;
	case NM_OC_CHANNEL:
		if (obj_inst->trx_nr >= bts->num_trx) {
			return NULL;
		}
		trx = gsm_bts_trx_num(bts, obj_inst->trx_nr);
		if (obj_inst->ts_nr >= TRX_NR_TS)
			return NULL;
		obj = &trx->ts[obj_inst->ts_nr];
		break;
	case NM_OC_SITE_MANAGER:
		obj = &bts->site_mgr;
		break;
	case NM_OC_GPRS_NSE:
		obj = &bts->gprs.nse;
		break;
	case NM_OC_GPRS_CELL:
		obj = &bts->gprs.cell;
		break;
	case NM_OC_GPRS_NSVC:
		if (obj_inst->trx_nr >= ARRAY_SIZE(bts->gprs.nsvc))
			return NULL;
		obj = &bts->gprs.nsvc[obj_inst->trx_nr];
		break;
	}
	return obj;
}

/* See Table 10.5.25 of GSM04.08 */
uint8_t gsm_ts2chan_nr(const struct gsm_bts_trx_ts *ts, uint8_t lchan_nr)
{
	uint8_t cbits, chan_nr;

	switch (ts->pchan) {
	case GSM_PCHAN_TCH_F:
	case GSM_PCHAN_PDCH:
	case GSM_PCHAN_TCH_F_PDCH:
		cbits = 0x01;
		break;
	case GSM_PCHAN_TCH_H:
		cbits = 0x02;
		cbits += lchan_nr;
		break;
	case GSM_PCHAN_CCCH_SDCCH4:
		cbits = 0x04;
		cbits += lchan_nr;
		break;
	case GSM_PCHAN_SDCCH8_SACCH8C:
		cbits = 0x08;
		cbits += lchan_nr;
		break;
	default:
	case GSM_PCHAN_CCCH:
		cbits = 0x10;
		break;
	}

	chan_nr = (cbits << 3) | (ts->nr & 0x7);

	return chan_nr;
}

uint8_t gsm_lchan2chan_nr(const struct gsm_lchan *lchan)
{
	return gsm_ts2chan_nr(lchan->ts, lchan->nr);
}
