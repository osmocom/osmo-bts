/* BTS support code common to all supported BTS models */

/* (C) 2011 by Andreas Eversberg <jolly@eversberg.eu>
 * (C) 2011 by Harald Welte <laforge@gnumonks.org>
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

#include <errno.h>
#include <unistd.h>
#include <stdio.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <osmocom/core/select.h>
#include <osmocom/core/timer.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/talloc.h>
#include <osmocom/core/tdef.h>
#include <osmocom/core/stats.h>
#include <osmocom/core/rate_ctr.h>
#include <osmocom/gsm/protocol/gsm_12_21.h>
#include <osmocom/gsm/gsm48.h>
#include <osmocom/gsm/lapdm.h>
#include <osmocom/trau/osmo_ortp.h>

#include <osmo-bts/logging.h>
#include <osmo-bts/abis.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/bts_model.h>
#include <osmo-bts/bts_sm.h>
#include <osmo-bts/dtx_dl_amr_fsm.h>
#include <osmo-bts/pcuif_proto.h>
#include <osmo-bts/rsl.h>
#include <osmo-bts/oml.h>
#include <osmo-bts/signal.h>
#include <osmo-bts/dtx_dl_amr_fsm.h>
#include <osmo-bts/cbch.h>
#include <osmo-bts/bts_shutdown_fsm.h>
#include <osmo-bts/nm_common_fsm.h>
#include <osmo-bts/power_control.h>
#include <osmo-bts/osmux.h>
#include <osmo-bts/notification.h>

#define MIN_QUAL_RACH	 50 /* minimum link quality (in centiBels) for Access Bursts */
#define MIN_QUAL_NORM	 -5 /* minimum link quality (in centiBels) for Normal Bursts */

static void bts_update_agch_max_queue_length(struct gsm_bts *bts);

void *tall_bts_ctx;

/* Table 3.1 TS 04.08: Values of parameter S */
static const uint8_t tx_integer[] = {
	3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 16, 20, 25, 32, 50,
};

static const uint8_t s_values[][2] = {
	{ 55, 41 }, { 76, 52 }, { 109, 58 }, { 163, 86 }, { 217, 115 },
};

static int bts_signal_cbfn(unsigned int subsys, unsigned int signal,
			   void *hdlr_data, void *signal_data)
{
	if (subsys == SS_GLOBAL && signal == S_NEW_SYSINFO) {
		struct gsm_bts *bts = signal_data;

		bts_update_agch_max_queue_length(bts);
	}
	return 0;
}

static const struct rate_ctr_desc bts_ctr_desc[] = {
	[BTS_CTR_PAGING_RCVD] =		{"paging:rcvd", "Received paging requests (Abis)"},
	[BTS_CTR_PAGING_DROP] =		{"paging:drop", "Dropped paging requests (Abis)"},
	[BTS_CTR_PAGING_DROP_PS] =	{"paging:drop-ps", "Dropped paging requests (PS/PCU)"},
	[BTS_CTR_PAGING_CONG] =		{"paging:cong", "Paging congestion detected (Abis)"},
	[BTS_CTR_PAGING_SENT] =		{"paging:sent", "Sent paging requests (Um)"},

	[BTS_CTR_RACH_RCVD] =		{"rach:rcvd", "Received RACH requests (Um)"},
	[BTS_CTR_RACH_DROP] =		{"rach:drop", "Dropped RACH requests (Um)"},
	[BTS_CTR_RACH_HO] =		{"rach:handover", "Received RACH requests (Handover)"},
	[BTS_CTR_RACH_VGCS] =		{"rach:vgcs", "Received RACH requests (VGCS)"},
	[BTS_CTR_RACH_CS] =		{"rach:cs", "Received RACH requests (CS/Abis)"},
	[BTS_CTR_RACH_PS] =		{"rach:ps", "Received RACH requests (PS/PCU)"},

	[BTS_CTR_AGCH_RCVD] =		{"agch:rcvd", "Received AGCH requests (Abis)"},
	[BTS_CTR_AGCH_SENT] =		{"agch:sent", "Sent AGCH requests (Abis)"},
	[BTS_CTR_AGCH_DELETED] =	{"agch:delete", "Sent AGCH DELETE IND (Abis)"},
};
static const struct rate_ctr_group_desc bts_ctrg_desc = {
	"bts",
	"base transceiver station",
	OSMO_STATS_CLASS_GLOBAL,
	ARRAY_SIZE(bts_ctr_desc),
	bts_ctr_desc
};

static const struct rate_ctr_desc cbch_ctr_desc[] = {
	[CBCH_CTR_RCVD_QUEUED] =	{"cbch:rcvd_queued", "Received + queued CBCH messages (Abis)" },
	[CBCH_CTR_RCVD_DROPPED] =	{"cbch:rcvd_dropped", "Received + dropped CBCH messages (Abis)" },

	[CBCH_CTR_SENT_SINGLE] =	{"cbch:sent_single", "Sent single CBCH messages (Um)" },
	[CBCH_CTR_SENT_DEFAULT] =	{"cbch:sent_default", "Sent default CBCH messages (Um)" },
	[CBCH_CTR_SENT_NULL] =		{"cbch:sent_null", "Sent NULL CBCH messages (Um)" },
};
static const struct rate_ctr_group_desc cbch_ctrg_desc = {
	"cbch",
	"cell broadcast channel",
	OSMO_STATS_CLASS_GLOBAL,
	ARRAY_SIZE(cbch_ctr_desc),
	cbch_ctr_desc
};

struct osmo_tdef bts_T_defs[] = {
	/* T-1: FIXME: Ideally should be dynamically calculated per trx at
	 * shutdown start based on params below, and highest trx value taken:
	 * + VTY's power-ramp step-interval.
	 * + Amount of steps needed (taking into account how many dB each step moves).
	 * + Extra time to get response back for each step.
	 * For now we simply give 5 mins, which should be enough for any
	 * acceptable setup, while still ensuring will timeout at some point if
	 * something fails in the ramp down procedure.
	 */
	{ .T=-1, .default_val=300, .desc="Time after which osmo-bts exits if regular ramp down during shut down process does not finish (s)" },
	{ .T=-2, .default_val=3, .desc="Time after which osmo-bts exits if requesting transceivers to stop during shut down process does not finish (s)" },
	{}
};

struct osmo_tdef abis_T_defs[] = {
	{ .T=-15, .default_val=0, .unit=OSMO_TDEF_MS, .desc="Time to wait between Channel Activation and dispatching a cached early Immediate Assignment" },
	{}
};

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

const struct value_string osmo_bts_variant_names[_NUM_BTS_VARIANT + 1] = {
	{ BTS_UNKNOWN,		"unknown" },
	{ BTS_OSMO_LITECELL15,	"osmo-bts-lc15" },
	{ BTS_OSMO_OC2G,	"osmo-bts-oc2g" },
	{ BTS_OSMO_OCTPHY,	"osmo-bts-octphy" },
	{ BTS_OSMO_SYSMO,	"osmo-bts-sysmo" },
	{ BTS_OSMO_TRX,		"osmo-bts-trx" },
	{ BTS_OSMO_VIRTUAL,	"osmo-bts-virtual" },
	{ BTS_OSMO_OMLDUMMY,	"osmo-bts-omldummy" },
	{ 0, NULL }
};

const char *btsvariant2str(enum gsm_bts_type_variant v)
{
	return get_value_string(osmo_bts_variant_names, v);
}

const struct value_string bts_attribute_names[] = {
	OSMO_VALUE_STRING(BTS_TYPE_VARIANT),
	OSMO_VALUE_STRING(BTS_SUB_MODEL),
	OSMO_VALUE_STRING(TRX_PHY_VERSION),
	{ 0, NULL }
};

const char *btsatttr2str(enum bts_attribute v)
{
	return get_value_string(bts_attribute_names, v);
}

const struct value_string bts_impl_flag_desc[] = {
	{ BTS_INTERNAL_FLAG_MS_PWR_CTRL_DSP,	"DSP/HW based MS Power Control Loop" },
	{ BTS_INTERNAL_FLAG_MEAS_PAYLOAD_COMB,	"Measurement and Payload data combined" },
	{ BTS_INTERNAL_FLAG_NM_RCHANNEL_DEPENDS_RCARRIER, "OML RadioChannel MO depends on RadioCarrier MO" },
	{ BTS_INTERNAL_FLAG_INTERF_MEAS,	"Uplink interference measurements" },
	{ 0, NULL }
};

/* Ensure that all BTS_INTERNAL_FLAG_* entries are present in bts_impl_flag_desc[] */
osmo_static_assert(ARRAY_SIZE(bts_impl_flag_desc) == _BTS_INTERNAL_FLAG_NUM + 1, _bts_impl_flag_desc);

static int gsm_bts_talloc_destructor(struct gsm_bts *bts)
{
	if (bts->mo.fi) {
		osmo_fsm_inst_free(bts->mo.fi);
		bts->mo.fi = NULL;
	}
	if (bts->shutdown_fi) {
		osmo_fsm_inst_free(bts->shutdown_fi);
		bts->shutdown_fi = NULL;
	}

	bts_osmux_release(bts);

	llist_del(&bts->list);
	g_bts_sm->num_bts--;
	return 0;
}

struct gsm_bts *gsm_bts_alloc(struct gsm_bts_sm *bts_sm, uint8_t bts_num)
{
	struct gsm_bts *bts = talloc_zero(bts_sm, struct gsm_bts);

	if (!bts)
		return NULL;

	talloc_set_destructor(bts, gsm_bts_talloc_destructor);

	/* add to list of BTSs */
	llist_add_tail(&bts->list, &bts_sm->bts_list);
	g_bts_sm->num_bts++;

	bts->site_mgr = bts_sm;
	bts->nr = bts_num;
	bts->num_trx = 0;
	INIT_LLIST_HEAD(&bts->trx_list);
	bts->ms_max_power = 15;	/* dBm */

	bts->T_defs = bts_T_defs;
	osmo_tdefs_reset(bts->T_defs);
	osmo_tdefs_reset(abis_T_defs);
	bts->shutdown_fi = osmo_fsm_inst_alloc(&bts_shutdown_fsm, bts, bts,
					       LOGL_INFO, NULL);
	osmo_fsm_inst_update_id_f(bts->shutdown_fi, "bts%d", bts->nr);

	/* NM BTS */
	bts->mo.fi = osmo_fsm_inst_alloc(&nm_bts_fsm, bts, bts,
					 LOGL_INFO, NULL);
	osmo_fsm_inst_update_id_f(bts->mo.fi, "bts%d", bts->nr);
	gsm_mo_init(&bts->mo, bts, NM_OC_BTS, bts->nr, 0xff, 0xff);

	/* NM GPRS CELL */
	bts->gprs.cell.mo.fi = osmo_fsm_inst_alloc(&nm_gprs_cell_fsm, bts, &bts->gprs.cell,
						  LOGL_INFO, NULL);
	osmo_fsm_inst_update_id_f(bts->gprs.cell.mo.fi, "gprs_cell%d-0", bts->nr);
	gsm_mo_init(&bts->gprs.cell.mo, bts, NM_OC_GPRS_CELL, bts->nr, 0, 0xff);
	memcpy(&bts->gprs.cell.rlc_cfg, &rlc_cfg_default, sizeof(bts->gprs.cell.rlc_cfg));
	memcpy(&bts->gprs.cell.timer, bts_cell_timer_default, sizeof(bts->gprs.cell.timer));

	/* create our primary TRX. It will be initialized during bts_init() */
	bts->c0 = gsm_bts_trx_alloc(bts);
	if (!bts->c0) {
		talloc_free(bts);
		return NULL;
	}
	bts->c0->ts[0].pchan = GSM_PCHAN_CCCH_SDCCH4;

	bts->features = bitvec_alloc(MAX_BTS_FEATURES / 8, bts);
	OSMO_ASSERT(bts->features != NULL);

	return bts;
}

struct gsm_bts *gsm_bts_num(const struct gsm_bts_sm *bts_sm, int num)
{
	struct gsm_bts *bts;

	if (num >= bts_sm->num_bts)
		return NULL;

	llist_for_each_entry(bts, &bts_sm->bts_list, list) {
		if (bts->nr == num)
			return bts;
	}

	return NULL;
}

/* Initialize the BTS data structures, called before config
 * file reading */
int bts_init(struct gsm_bts *bts)
{
	int rc, i;
	static int initialized = 0;
	void *tall_rtp_ctx;

	bts->band = GSM_BAND_1800;

	INIT_LLIST_HEAD(&bts->agch_queue.queue);
	bts->agch_queue.length = 0;

	bts->ctrs = rate_ctr_group_alloc(bts, &bts_ctrg_desc, bts->nr);
	if (!bts->ctrs)
		return -1;

	/* enable management with default levels,
	 * raise threshold to GSM_BTS_AGCH_QUEUE_THRESH_LEVEL_DISABLE to
	 * disable this feature.
	 */
	bts->agch_queue.low_level = GSM_BTS_AGCH_QUEUE_LOW_LEVEL_DEFAULT;
	bts->agch_queue.high_level = GSM_BTS_AGCH_QUEUE_HIGH_LEVEL_DEFAULT;
	bts->agch_queue.thresh_level = GSM_BTS_AGCH_QUEUE_THRESH_LEVEL_DEFAULT;

	/* configurable via VTY */
	bts->paging_state = paging_init(bts, 200, 0);
	bts->rtp_jitter_adaptive = false;
	bts->rtp_port_range_start = 16384;
	bts->rtp_port_range_end = 17407;
	bts->rtp_port_range_next = bts->rtp_port_range_start;
	bts->rtp_ip_dscp = -1;
	bts->rtp_priority = -1;
	bts->emit_hr_rfc5993 = true;

	/* Default (fall-back) MS/BS Power control parameters */
	power_ctrl_params_def_reset(&bts->bs_dpc_params, true);
	power_ctrl_params_def_reset(&bts->ms_dpc_params, false);

	/* configurable via OML */
	bts->bsic = 0xff; /* invalid value, guarded by bsc_configured=false */
	bts->bsic_configured = false;
	bts->load.ccch.load_ind_period = 112;
	bts->rtp_jitter_buf_ms = 100;
	bts->max_ta = 63;
	bts->ny1 = 4;
	bts->ny2 = 4;
	bts->t3105_ms = 300;
	bts->t3115_ms = 300;
	bts->min_qual_rach = MIN_QUAL_RACH;
	bts->min_qual_norm = MIN_QUAL_NORM;
	bts->max_ber10k_rach = 1707; /* 7 of 41 bits is Eb/N0 of 0 dB = 0.1707 */
	bts->pcu.sock_path = talloc_strdup(bts, PCU_SOCK_DEFAULT);
	bts->pcu.sock_wqueue_len_max = BTS_PCU_SOCK_WQUEUE_LEN_DEFAULT;
	for (i = 0; i < ARRAY_SIZE(bts->t200_ms); i++)
		bts->t200_ms[i] = oml_default_t200_ms[i];

	/* default RADIO_LINK_TIMEOUT */
	bts->radio_link_timeout.oml = 32;
	bts->radio_link_timeout.current = bts->radio_link_timeout.oml;

	/* Start with the BTS */
	oml_mo_state_init(&bts->mo, NM_OPSTATE_DISABLED, NM_AVSTATE_NOT_INSTALLED);
	oml_mo_state_init(&bts->gprs.cell.mo, NM_OPSTATE_DISABLED, NM_AVSTATE_NOT_INSTALLED);

	/* allocate a talloc pool for ORTP to ensure it doesn't have to go back
	 * to the libc malloc all the time */
	tall_rtp_ctx = talloc_pool(tall_bts_ctx, 262144);
	osmo_rtp_init(tall_rtp_ctx);

	/* Osmux */
	rc = bts_osmux_init(bts);
	if (rc < 0)
		return rc;

	/* features implemented in 'common', available for all models,
	 * order alphabetically */
	osmo_bts_set_feature(bts->features, BTS_FEAT_ABIS_OSMO_PCU);
	osmo_bts_set_feature(bts->features, BTS_FEAT_CCN);
	osmo_bts_set_feature(bts->features, BTS_FEAT_DYN_TS_SDCCH8);
	osmo_bts_set_feature(bts->features, BTS_FEAT_ETWS_PN);
	osmo_bts_set_feature(bts->features, BTS_FEAT_IPV6_NSVC);
	osmo_bts_set_feature(bts->features, BTS_FEAT_PAGING_COORDINATION);

	rc = bts_model_init(bts);
	if (rc < 0)
		return rc;

	/* TRX0 was allocated early during gsm_bts_alloc, not later through VTY */
	bts_model_trx_init(bts->c0);

	if (!initialized) {
		osmo_signal_register_handler(SS_GLOBAL, bts_signal_cbfn, NULL);
		initialized = 1;
	}

	INIT_LLIST_HEAD(&bts->smscb_basic.queue);
	bts->smscb_basic.ctrs = rate_ctr_group_alloc(bts, &cbch_ctrg_desc, 0);
	OSMO_ASSERT(bts->smscb_basic.ctrs);
	INIT_LLIST_HEAD(&bts->smscb_extended.queue);
	bts->smscb_extended.ctrs = rate_ctr_group_alloc(bts, &cbch_ctrg_desc, 1);
	OSMO_ASSERT(bts->smscb_extended.ctrs);
	bts->smscb_queue_max_len = 15;
	bts->smscb_queue_tgt_len = 2;
	bts->smscb_queue_hyst = 2;

	bts->asci.pos_nch = -ENOTSUP;
	INIT_LLIST_HEAD(&bts->asci.notifications);

	INIT_LLIST_HEAD(&bts->bsc_oml_hosts);

	/* register DTX DL FSM */
	rc = osmo_fsm_register(&dtx_dl_amr_fsm);
	OSMO_ASSERT(rc == 0);

	bts->fn_stats.min = INT32_MAX;
	bts->fn_stats.max = INT32_MIN;
	bts->fn_stats.avg_count = 0;
	bts->fn_stats.avg_window = 256;

	return rc;
}

/* main link is established, send status report */
int bts_link_estab(struct gsm_bts *bts)
{
	LOGP(DOML, LOGL_INFO, "Main link established, sending NM Status\n");

	/* Signal OML UP to BTS SITE MGR. It will automatically SW_ACT repoort
	 * and become Disabled-Offline, then dispatch same event to its children
	 * objects.
	 */
	osmo_fsm_inst_dispatch(bts->site_mgr->mo.fi, NM_EV_OML_UP, NULL);

	return bts_model_oml_estab(bts);
}

#define CCCH_RACH_RATIO_COMBINED256      (256*1/9)
#define CCCH_RACH_RATIO_SEPARATE256      (256*10/55)

int bts_agch_max_queue_length(int T, int bcch_conf)
{
	int S, ccch_rach_ratio256, i;
	int T_group = 0;
	int is_ccch_comb = 0;

	if (bcch_conf == RSL_BCCH_CCCH_CONF_1_C)
		is_ccch_comb = 1;

	/*
	 * The calculation is based on the ratio of the number RACH slots and
	 * CCCH blocks per time:
	 *   Lmax = (T + 2*S) / R_RACH * R_CCCH
	 * where
	 *   T3126_min = (T + 2*S) / R_RACH, as defined in GSM 04.08, 11.1.1
	 *   R_RACH is the RACH slot rate (e.g. RACHs per multiframe)
	 *   R_CCCH is the CCCH block rate (same time base like R_RACH)
	 *   S and T are defined in GSM 04.08, 3.3.1.1.2
	 * The ratio is mainly influenced by the downlink only channels
	 * (BCCH, FCCH, SCH, CBCH) that can not be used for CCCH.
	 * An estimation with an error of < 10% is used:
	 *   ~ 1/9 if CCCH is combined with SDCCH, and
	 *   ~ 1/5.5 otherwise.
	 */
	ccch_rach_ratio256 = is_ccch_comb ?
		CCCH_RACH_RATIO_COMBINED256 :
		CCCH_RACH_RATIO_SEPARATE256;

	for (i = 0; i < ARRAY_SIZE(tx_integer); i++) {
		if (tx_integer[i] == T) {
			T_group = i % 5;
			break;
		}
	}
	S = s_values[T_group][is_ccch_comb];

	return (T + 2 * S) * ccch_rach_ratio256 / 256;
}

static void bts_update_agch_max_queue_length(struct gsm_bts *bts)
{
	struct gsm48_system_information_type_3 *si3;
	int old_max_length = bts->agch_queue.max_length;

	if (!(bts->si_valid & (1<<SYSINFO_TYPE_3)))
		return;

	si3 = GSM_BTS_SI(bts, SYSINFO_TYPE_3);

	bts->agch_queue.max_length =
		bts_agch_max_queue_length(si3->rach_control.tx_integer,
					  si3->control_channel_desc.ccch_conf);

	if (bts->agch_queue.max_length != old_max_length)
		LOGP(DRSL, LOGL_INFO, "Updated AGCH max queue length to %d\n",
		     bts->agch_queue.max_length);
}

#define REQ_REFS_PER_IMM_ASS_REJ 4
static int store_imm_ass_rej_refs(struct gsm48_imm_ass_rej *rej,
				    struct gsm48_req_ref *req_refs,
				    uint8_t *wait_inds,
				    int count)
{
	switch (count) {
	case 0:
		/* TODO: Warning ? */
		return 0;
	default:
		count = 4;
		rej->req_ref4 = req_refs[3];
		rej->wait_ind4 = wait_inds[3];
		/* fall through */
	case 3:
		rej->req_ref3 = req_refs[2];
		rej->wait_ind3 = wait_inds[2];
		/* fall through */
	case 2:
		rej->req_ref2 = req_refs[1];
		rej->wait_ind2 = wait_inds[1];
		/* fall through */
	case 1:
		rej->req_ref1 = req_refs[0];
		rej->wait_ind1 = wait_inds[0];
		break;
	}

	switch (count) {
	case 1:
		rej->req_ref2 = req_refs[0];
		rej->wait_ind2 = wait_inds[0];
		/* fall through */
	case 2:
		rej->req_ref3 = req_refs[0];
		rej->wait_ind3 = wait_inds[0];
		/* fall through */
	case 3:
		rej->req_ref4 = req_refs[0];
		rej->wait_ind4 = wait_inds[0];
		/* fall through */
	default:
		break;
	}

	return count;
}

static int extract_imm_ass_rej_refs(struct gsm48_imm_ass_rej *rej,
				    struct gsm48_req_ref *req_refs,
				    uint8_t *wait_inds)
{
	int count = 0;
	req_refs[count] = rej->req_ref1;
	wait_inds[count] = rej->wait_ind1;
	count++;

	if (memcmp(&rej->req_ref1, &rej->req_ref2, sizeof(rej->req_ref2))) {
		req_refs[count] = rej->req_ref2;
		wait_inds[count] = rej->wait_ind2;
		count++;
	}

	if (memcmp(&rej->req_ref1, &rej->req_ref3, sizeof(rej->req_ref3)) &&
	    memcmp(&rej->req_ref2, &rej->req_ref3, sizeof(rej->req_ref3))) {
		req_refs[count] = rej->req_ref3;
		wait_inds[count] = rej->wait_ind3;
		count++;
	}

	if (memcmp(&rej->req_ref1, &rej->req_ref4, sizeof(rej->req_ref4)) &&
	    memcmp(&rej->req_ref2, &rej->req_ref4, sizeof(rej->req_ref4)) &&
	    memcmp(&rej->req_ref3, &rej->req_ref4, sizeof(rej->req_ref4))) {
		req_refs[count] = rej->req_ref4;
		wait_inds[count] = rej->wait_ind4;
		count++;
	}

	return count;
}

static int try_merge_imm_ass_rej(struct gsm48_imm_ass_rej *old_rej,
				 struct gsm48_imm_ass_rej *new_rej)
{
	struct gsm48_req_ref req_refs[2 * REQ_REFS_PER_IMM_ASS_REJ];
	uint8_t wait_inds[2 * REQ_REFS_PER_IMM_ASS_REJ];
	int count = 0;
	int stored = 0;

	if (new_rej->msg_type != GSM48_MT_RR_IMM_ASS_REJ)
		return 0;
	if (old_rej->msg_type != GSM48_MT_RR_IMM_ASS_REJ)
		return 0;

	/* GSM 08.58, 5.7
	 * -> The BTS may combine several IMM.ASS.REJ messages
	 * -> Identical request refs in one message may be squeezed
	 *
	 * GSM 04.08, 9.1.20.2
	 * -> Request ref and wait ind are duplicated to fill the message
	 */

	/* Extract all entries */
	count = extract_imm_ass_rej_refs(old_rej,
					 &req_refs[count], &wait_inds[count]);
	if (count == REQ_REFS_PER_IMM_ASS_REJ)
		return 0;

	count += extract_imm_ass_rej_refs(new_rej,
					  &req_refs[count], &wait_inds[count]);

	/* Store entries into old message */
	stored = store_imm_ass_rej_refs(old_rej,
					&req_refs[stored], &wait_inds[stored],
					count);
	count -= stored;
	if (count == 0)
		return 1;

	/* Store remaining entries into new message */
	stored += store_imm_ass_rej_refs(new_rej,
					 &req_refs[stored], &wait_inds[stored],
					 count);
	return 0;
}

int bts_agch_enqueue(struct gsm_bts *bts, struct msgb *msg)
{
	int hard_limit = 100;
	struct gsm48_imm_ass_rej *imm_ass_cmd = msgb_l3(msg);

	if (bts->agch_queue.length > hard_limit) {
		LOGP(DRR, LOGL_ERROR,
		     "AGCH: too many messages in queue, "
		     "refusing message type %s, length = %d/%d\n",
		     gsm48_rr_msg_name(((struct gsm48_imm_ass *)msgb_l3(msg))->msg_type),
		     bts->agch_queue.length, bts->agch_queue.max_length);

		bts->agch_queue.rejected_msgs++;
		return -ENOMEM;
	}

	if (bts->agch_queue.length > 0) {
		struct msgb *last_msg =
			llist_entry(bts->agch_queue.queue.prev, struct msgb, list);
		struct gsm48_imm_ass_rej *last_imm_ass_rej = msgb_l3(last_msg);

		if (try_merge_imm_ass_rej(last_imm_ass_rej, imm_ass_cmd)) {
			bts->agch_queue.merged_msgs++;
			msgb_free(msg);
			return 0;
		}
	}

	msgb_enqueue(&bts->agch_queue.queue, msg);
	bts->agch_queue.length++;

	return 0;
}

struct msgb *bts_agch_dequeue(struct gsm_bts *bts)
{
	struct msgb *msg = msgb_dequeue(&bts->agch_queue.queue);
	if (!msg)
		return NULL;

	bts->agch_queue.length--;
	return msg;
}

/*
 * Remove lower prio messages if the queue has grown too long.
 *
 * \return 0 if the number of messages in the queue would fit into the AGCH
 *         reserved part of the CCCH.
 */
static void compact_agch_queue(struct gsm_bts *bts)
{
	struct msgb *msg, *msg2;
	int max_len, slope, offs;
	int level_low = bts->agch_queue.low_level;
	int level_high = bts->agch_queue.high_level;
	int level_thres = bts->agch_queue.thresh_level;

	max_len = bts->agch_queue.max_length;

	if (max_len == 0)
		max_len = 1;

	if (bts->agch_queue.length < max_len * level_thres / 100)
		return;

	/* p^
	 * 1+      /'''''
	 *  |     /
	 *  |    /
	 * 0+---/--+----+--> Q length
	 *    low high max_len
	 */

	offs = max_len * level_low / 100;
	if (level_high > level_low)
		slope = 0x10000 * 100 / (level_high - level_low);
	else
		slope = 0x10000 * max_len; /* p_drop >= 1 if len > offs */

	llist_for_each_entry_safe(msg, msg2, &bts->agch_queue.queue, list) {
		struct gsm48_imm_ass *imm_ass_cmd = msgb_l3(msg);
		int p_drop;

		p_drop = (bts->agch_queue.length - offs) * slope / max_len;

		if ((random() & 0xffff) >= p_drop)
			return;

		llist_del(&msg->list);
		bts->agch_queue.length--;
		rsl_tx_delete_ind(bts, (uint8_t *)imm_ass_cmd, msgb_l3len(msg));
		rate_ctr_inc2(bts->ctrs, BTS_CTR_AGCH_DELETED);
		msgb_free(msg);

		bts->agch_queue.dropped_msgs++;
	}
	return;
}

int bts_ccch_copy_msg(struct gsm_bts *bts, uint8_t *out_buf, struct gsm_time *gt, enum ccch_msgt ccch)
{
	struct msgb *msg = NULL;
	int rc = 0;
	int is_empty = 1;

	/* Do queue house keeping.
	 * This needs to be done every time a CCCH message is requested, since
	 * the queue max length is calculated based on the CCCH block rate and
	 * PCH messages also reduce the drain of the AGCH queue.
	 */
	compact_agch_queue(bts);

	switch (ccch) {
	case CCCH_MSGT_NCH:
		/* Send NCH message, it has priority over AGCH and does not overlap with PCH. */
		rc = bts_asci_notify_nch_gen_msg(bts, out_buf);
		return rc;
	case CCCH_MSGT_PCH:
		/* Check whether the block may be overwritten by AGCH. */
		rc = paging_gen_msg(bts->paging_state, out_buf, gt, &is_empty);
		if (!is_empty)
			return rc;
		/* fall-through */
	case CCCH_MSGT_AGCH:
		/* If fallen here and the AGCH queue is empty, return empty PCH message. */
		msg = bts_agch_dequeue(bts);
		if (!msg)
			return rc;
		/* Continue to return AGCH message. */
		break;
	}

	rate_ctr_inc2(bts->ctrs, BTS_CTR_AGCH_SENT);

	/* Copy AGCH message */
	memcpy(out_buf, msgb_l3(msg), msgb_l3len(msg));
	rc = msgb_l3len(msg);
	msgb_free(msg);

	if (ccch == CCCH_MSGT_AGCH)
		bts->agch_queue.agch_msgs++;
	else
		bts->agch_queue.pch_msgs++;

	return rc;
}

int bts_supports_cipher(struct gsm_bts *bts, int rsl_cipher)
{
	int sup;

	if (rsl_cipher < 1 || rsl_cipher > 8)
		return -ENOTSUP;

	/* No encryption is always supported */
	if (rsl_cipher == 1)
		return 1;

	sup =  (1 << (rsl_cipher - 2)) & bts->support.ciphers;
	return sup > 0;
}

struct gsm_time *get_time(struct gsm_bts *bts)
{
	return &bts->gsm_time;
}

bool bts_supports_cm_speech(const struct gsm_bts *bts,
			    const struct rsl_ie_chan_mode *cm)
{
	enum osmo_bts_features feature = _NUM_BTS_FEAT;

	/* Stage 1: check support for the requested channel type */
	switch (cm->chan_rt) {
	case RSL_CMOD_CRT_TCH_GROUP_Bm:
	case RSL_CMOD_CRT_TCH_GROUP_Lm:
		if (!osmo_bts_has_feature(bts->features, BTS_FEAT_VGCS))
			return false;
		break;
	case RSL_CMOD_CRT_TCH_BCAST_Bm:
	case RSL_CMOD_CRT_TCH_BCAST_Lm:
		if (!osmo_bts_has_feature(bts->features, BTS_FEAT_VBS))
			return false;
		break;
	case RSL_CMOD_CRT_OSMO_TCH_VAMOS_Bm:
	case RSL_CMOD_CRT_OSMO_TCH_VAMOS_Lm:
		if (!osmo_bts_has_feature(bts->features, BTS_FEAT_VAMOS))
			return false;
		break;
	}

	/* Stage 2: check support for the requested codec */
	switch (cm->chan_rt) {
	case RSL_CMOD_CRT_OSMO_TCH_VAMOS_Bm:
	case RSL_CMOD_CRT_TCH_GROUP_Bm:
	case RSL_CMOD_CRT_TCH_BCAST_Bm:
	case RSL_CMOD_CRT_TCH_Bm:
		switch (cm->chan_rate) {
		case RSL_CMOD_SP_GSM1:
			feature	= BTS_FEAT_SPEECH_F_V1;
			break;
		case RSL_CMOD_SP_GSM2:
			feature	= BTS_FEAT_SPEECH_F_EFR;
			break;
		case RSL_CMOD_SP_GSM3:
			feature = BTS_FEAT_SPEECH_F_AMR;
			break;
		default:
			/* Invalid speech codec type => Not supported! */
			return false;
		}
		break;

	case RSL_CMOD_CRT_OSMO_TCH_VAMOS_Lm:
	case RSL_CMOD_CRT_TCH_GROUP_Lm:
	case RSL_CMOD_CRT_TCH_BCAST_Lm:
	case RSL_CMOD_CRT_TCH_Lm:
		switch (cm->chan_rate) {
		case RSL_CMOD_SP_GSM1:
			feature	= BTS_FEAT_SPEECH_H_V1;
			break;
		case RSL_CMOD_SP_GSM3:
			feature = BTS_FEAT_SPEECH_H_AMR;
			break;
		default:
			/* Invalid speech codec type => Not supported! */
			return false;
		}
		break;

	default:
		LOGP(DRSL, LOGL_ERROR,
		     "Unhandled RSL channel type=0x%02x/rate=0x%02x\n",
		     cm->chan_rt, cm->chan_rate);
		return false;
	}

	/* Check if the feature is supported by this BTS */
	if (osmo_bts_has_feature(bts->features, feature))
		return true;

	return false;
}

static bool bts_supports_cm_data(const struct gsm_bts *bts,
				 const struct rsl_ie_chan_mode *cm)
{
	switch (bts->variant) {
	case BTS_OSMO_TRX:
		switch (cm->chan_rate) {
		/* TODO: RSL_CMOD_CSD_NT_14k5 */
		/* TODO: RSL_CMOD_CSD_T_14k4 */
		case RSL_CMOD_CSD_NT_12k0:
		case RSL_CMOD_CSD_T_9k6:
			if (cm->chan_rt != RSL_CMOD_CRT_TCH_Bm)
				return false; /* invalid */
			/* fall-through */
		case RSL_CMOD_CSD_NT_6k0:
		case RSL_CMOD_CSD_T_4k8:
		case RSL_CMOD_CSD_T_2k4:
		case RSL_CMOD_CSD_T_1k2:
		case RSL_CMOD_CSD_T_600:
		case RSL_CMOD_CSD_T_1200_75:
			return true;
		default:
			return false;
		}
	default:
		return 0;
	}
}

bool bts_supports_cm(const struct gsm_bts *bts,
		     const struct rsl_ie_chan_mode *cm)
{
	switch (cm->spd_ind) {
	case RSL_CMOD_SPD_SIGN:
		/* We assume that signalling support is mandatory,
		 * there is no BTS_FEAT_* definition to check that. */
		return true;
	case RSL_CMOD_SPD_SPEECH:
		return bts_supports_cm_speech(bts, cm);
	case RSL_CMOD_SPD_DATA:
		return bts_supports_cm_data(bts, cm);
	default:
		return false;
	}
}

/* return the gsm_lchan for the CBCH (if it exists at all) */
struct gsm_lchan *gsm_bts_get_cbch(struct gsm_bts *bts)
{
	struct gsm_bts_trx *trx = bts->c0;

	/* According to 3GPP TS 45.002, table 3, CBCH can be allocated
	 * either on C0/TS0 (CCCH+SDCCH4) or on C0..n/TS0..3 (SDCCH/8). */
	if (trx->ts[0].pchan == GSM_PCHAN_CCCH_SDCCH4_CBCH)
		return &trx->ts[0].lchan[2]; /* C0/TS0 */

	llist_for_each_entry(trx, &bts->trx_list, list) { /* C0..n */
		unsigned int tn;
		for (tn = 0; tn <= 3; tn++) { /* TS0..3 */
			struct gsm_bts_trx_ts *ts = &trx->ts[tn];
			if (ts->pchan == GSM_PCHAN_SDCCH8_SACCH8C_CBCH)
				return &ts->lchan[2];
		}
	}

	return NULL;
}

/* BCCH carrier power reduction (see 3GPP TS 45.008, section 7.1) */
int bts_set_c0_pwr_red(struct gsm_bts *bts, const uint8_t red)
{
	struct gsm_bts_trx *c0 = bts->c0;
	unsigned int tn;

	if (!osmo_bts_has_feature(bts->features, BTS_FEAT_BCCH_POWER_RED)) {
		LOGPTRX(c0, DRSL, LOGL_ERROR, "BCCH carrier power reduction "
			"is not supported by this BTS model\n");
		return -ENOTSUP;
	}

	if (red > 6 || red % 2 != 0) {
		LOGPTRX(c0, DRSL, LOGL_ERROR, "BCCH carrier power reduction "
			"value (%u dB) is incorrect or out of range\n", red);
		return -EINVAL;
	}

	LOGPTRX(c0, DRSL, LOGL_NOTICE, "BCCH carrier power reduction: "
		"%u dB (%s)\n", red, red ? "enabled" : "disabled");

	/* Timeslot 0 is always transmitting BCCH/CCCH */
	c0->ts[0].c0_power_red_db = 0;

	for (tn = 1; tn < ARRAY_SIZE(c0->ts); tn++) {
		struct gsm_bts_trx_ts *ts = &c0->ts[tn];
		struct gsm_bts_trx_ts *prev = ts - 1;

		switch (ts_pchan(ts)) {
		/* Not allowed on CCCH/BCCH */
		case GSM_PCHAN_CCCH:
			/* Preceeding timeslot shall not exceed 2 dB */
			if (prev->c0_power_red_db > 0)
				prev->c0_power_red_db = 2;
			/* fall-through */
		/* Not recommended on SDCCH/8 */
		case GSM_PCHAN_SDCCH8_SACCH8C:
		case GSM_PCHAN_SDCCH8_SACCH8C_CBCH:
			ts->c0_power_red_db = 0;
			break;
		default:
			ts->c0_power_red_db = red;
			break;
		}
	}

	/* Timeslot 7 is always preceding BCCH/CCCH */
	if (c0->ts[7].c0_power_red_db > 0)
		c0->ts[7].c0_power_red_db = 2;

	bts->c0_power_red_db = red;

	return 0;
}
