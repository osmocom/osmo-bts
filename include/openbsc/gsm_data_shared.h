#ifndef _GSM_DATA_SHAREDH
#define _GSM_DATA_SHAREDH

#include <regex.h>
#include <stdbool.h>
#include <stdint.h>

#include <osmocom/core/timer.h>
#include <osmocom/core/bitvec.h>
#include <osmocom/core/statistics.h>
#include <osmocom/core/utils.h>
#include <osmocom/gsm/gsm_utils.h>
#include <osmocom/gsm/tlv.h>
#include <osmocom/gsm/rxlev_stat.h>
#include <osmocom/gsm/sysinfo.h>
#include <osmocom/gsm/meas_rep.h>

#include <osmocom/gsm/protocol/gsm_08_58.h>
#include <osmocom/gsm/protocol/gsm_12_21.h>

#include <osmocom/abis/e1_input.h>

#ifndef ROLE_BSC
#include <osmocom/gsm/lapdm.h>
#endif

struct osmo_bsc_data;

struct osmo_bsc_sccp_con;
struct gsm_sms_queue;

/* RRLP mode of operation */
enum rrlp_mode {
	RRLP_MODE_NONE,
	RRLP_MODE_MS_BASED,
	RRLP_MODE_MS_PREF,
	RRLP_MODE_ASS_PREF,
};

/* Channel Request reason */
enum gsm_chreq_reason_t {
	GSM_CHREQ_REASON_EMERG,
	GSM_CHREQ_REASON_PAG,
	GSM_CHREQ_REASON_CALL,
	GSM_CHREQ_REASON_LOCATION_UPD,
	GSM_CHREQ_REASON_OTHER,
};

#define TRX_NR_TS	8
#define TS_MAX_LCHAN	8

#define HARDCODED_ARFCN 123
#define HARDCODED_TSC	7
#define HARDCODED_BSIC	0x3f	/* NCC = 7 / BCC = 7 */

/* for multi-drop config */
#define HARDCODED_BTS0_TS	1
#define HARDCODED_BTS1_TS	6
#define HARDCODED_BTS2_TS	11

enum gsm_hooks {
	GSM_HOOK_NM_SWLOAD,
	GSM_HOOK_RR_PAGING,
	GSM_HOOK_RR_SECURITY,
};

enum gsm_paging_event {
	GSM_PAGING_SUCCEEDED,
	GSM_PAGING_EXPIRED,
	GSM_PAGING_OOM,
	GSM_PAGING_BUSY,
};

enum bts_gprs_mode {
	BTS_GPRS_NONE = 0,
	BTS_GPRS_GPRS = 1,
	BTS_GPRS_EGPRS = 2,
};

struct gsm_lchan;
struct gsm_subscriber;
struct gsm_mncc;
struct osmo_rtp_socket;
struct rtp_socket;
struct bsc_api;

/* Network Management State */
struct gsm_nm_state {
	uint8_t operational;
	uint8_t administrative;
	uint8_t availability;
};

struct gsm_abis_mo {
	uint8_t obj_class;
	uint8_t procedure_pending;
	struct abis_om_obj_inst obj_inst;
	const char *name;
	struct gsm_nm_state nm_state;
	struct tlv_parsed *nm_attr;
	struct gsm_bts *bts;
};

#define MAX_A5_KEY_LEN	(128/8)
#define A38_XOR_MIN_KEY_LEN	12
#define A38_XOR_MAX_KEY_LEN	16
#define A38_COMP128_KEY_LEN	16
#define RSL_ENC_ALG_A5(x)	(x+1)

/* is the data link established? who established it? */
#define LCHAN_SAPI_UNUSED	0
#define LCHAN_SAPI_MS		1
#define LCHAN_SAPI_NET		2
#define LCHAN_SAPI_REL		3

/* state of a logical channel */
enum gsm_lchan_state {
	LCHAN_S_NONE,		/* channel is not active */
	LCHAN_S_ACT_REQ,	/* channel activation requested */
	LCHAN_S_ACTIVE,		/* channel is active and operational */
	LCHAN_S_REL_REQ,	/* channel release has been requested */
	LCHAN_S_REL_ERR,	/* channel is in an error state */
	LCHAN_S_BROKEN,		/* channel is somehow unusable */
	LCHAN_S_INACTIVE,	/* channel is set inactive */
};

/* BTS ONLY */
#define MAX_NUM_UL_MEAS	104
#define LC_UL_M_F_L1_VALID	(1 << 0)
#define LC_UL_M_F_RES_VALID	(1 << 1)

struct bts_ul_meas {
	/* BER in units of 0.01%: 10.000 == 100% ber, 0 == 0% ber */
	uint16_t ber10k;
	/* timing advance offset (in quarter bits) */
	int16_t ta_offs_qbits;
	/* C/I ratio in dB */
	float c_i;
	/* flags */
	uint8_t is_sub:1;
	/* RSSI in dBm * -1 */
	uint8_t inv_rssi;
};

struct bts_codec_conf {
	uint8_t hr;
	uint8_t efr;
	uint8_t amr;
};

struct amr_mode {
	uint8_t mode;
	uint8_t threshold;
	uint8_t hysteresis;
};
struct amr_multirate_conf {
	uint8_t gsm48_ie[2];
	struct amr_mode mode[4];
	uint8_t num_modes;
};
/* /BTS ONLY */

enum lchan_csd_mode {
	LCHAN_CSD_M_NT,
	LCHAN_CSD_M_T_1200_75,
	LCHAN_CSD_M_T_600,
	LCHAN_CSD_M_T_1200,
	LCHAN_CSD_M_T_2400,
	LCHAN_CSD_M_T_9600,
	LCHAN_CSD_M_T_14400,
	LCHAN_CSD_M_T_29000,
	LCHAN_CSD_M_T_32000,
};

/* State of the SAPIs in the lchan */
enum lchan_sapi_state {
	LCHAN_SAPI_S_NONE,
	LCHAN_SAPI_S_REQ,
	LCHAN_SAPI_S_ASSIGNED,
	LCHAN_SAPI_S_REL,
	LCHAN_SAPI_S_ERROR,
};

struct gsm_lchan {
	/* The TS that we're part of */
	struct gsm_bts_trx_ts *ts;
	/* The logical subslot number in the TS */
	uint8_t nr;
	/* The logical channel type */
	enum gsm_chan_t type;
	/* RSL channel mode */
	enum rsl_cmod_spd rsl_cmode;
	/* If TCH, traffic channel mode */
	enum gsm48_chan_mode tch_mode;
	enum lchan_csd_mode csd_mode;
	/* State */
	enum gsm_lchan_state state;
	const char *broken_reason;
	/* Power levels for MS and BTS */
	uint8_t bs_power;
	uint8_t ms_power;
	/* Encryption information */
	struct {
		uint8_t alg_id;
		uint8_t key_len;
		uint8_t key[MAX_A5_KEY_LEN];
	} encr;

	/* AMR bits */
	struct gsm48_multi_rate_conf mr_conf;

	/* Established data link layer services */
	uint8_t sapis[8];
	int sacch_deact;

	struct {
		uint32_t bound_ip;
		uint32_t connect_ip;
		uint16_t bound_port;
		uint16_t connect_port;
		uint16_t conn_id;
		uint8_t rtp_payload;
		uint8_t rtp_payload2;
		uint8_t speech_mode;
#ifdef ROLE_BSC
		struct rtp_socket *rtp_socket;
#else
		struct osmo_rtp_socket *rtp_socket;
#endif
	} abis_ip;

	uint8_t rqd_ta;

#ifdef ROLE_BSC
	struct osmo_timer_list T3101;
	struct osmo_timer_list T3109;
	struct osmo_timer_list T3111;
	struct osmo_timer_list error_timer;
	struct osmo_timer_list act_timer;
	struct osmo_timer_list rel_work;
	uint8_t error_cause;

	/* table of neighbor cell measurements */
	struct neigh_meas_proc neigh_meas[MAX_NEIGH_MEAS];

	/* cache of last measurement reports on this lchan */
	struct gsm_meas_rep meas_rep[6];
	int meas_rep_idx;

	/* GSM Random Access data */
	struct gsm48_req_ref *rqd_ref;

	struct gsm_subscriber_connection *conn;
#else
	/* Number of different GsmL1_Sapi_t used in osmo_bts_sysmo is 23.
	 * Currently we don't share these headers so this is a magic number. */
	struct llist_head sapi_cmds;
	uint8_t sapis_dl[23];
	uint8_t sapis_ul[23];
	struct lapdm_channel lapdm_ch;
	struct llist_head dl_tch_queue;
	struct {
		/* bitmask of all SI that are present/valid in si_buf */
		uint32_t valid;
		uint32_t last;
		/* buffers where we put the pre-computed SI */
		sysinfo_buf_t buf[_MAX_SYSINFO_TYPE];
	} si;
	struct {
		uint8_t flags;
		/* RSL measurment result number, 0 at lchan_act */
		uint8_t res_nr;
		/* current Tx power level of the BTS */
		uint8_t bts_tx_pwr;
		/* number of measurements stored in array below */
		uint8_t num_ul_meas;
		struct bts_ul_meas uplink[MAX_NUM_UL_MEAS];
		/* last L1 header from the MS */
		uint8_t l1_info[2];
		struct gsm_meas_rep_unidir ul_res;
	} meas;
	struct {
		struct amr_multirate_conf amr_mr;
		struct {
			uint8_t buf[16];
			uint8_t len;
		} last_sid;
		uint8_t last_cmr;
	} tch;
	/* BTS-side ciphering state (rx only, bi-directional, ...) */
	uint8_t ciph_state;
	uint8_t ciph_ns;
	uint8_t loopback;
	struct {
		uint8_t active;
		uint8_t ref;
		/* T3105: PHYS INF retransmission */
		struct osmo_timer_list t3105;
		/* counts up to Ny1 */
		unsigned int phys_info_count;
	} ho;
	/* S counter for link loss */
	int s;
	/* Kind of the release/activation. E.g. RSL or PCU */
	int rel_act_kind;

	/* power handling */
	struct {
		uint8_t current;
		uint8_t fixed;
	} ms_power_ctrl;
#endif
};

#define TS_F_PDCH_MODE	0x1000
/* One Timeslot in a TRX */
struct gsm_bts_trx_ts {
	struct gsm_bts_trx *trx;
	/* number of this timeslot at the TRX */
	uint8_t nr;

	enum gsm_phys_chan_config pchan;

	unsigned int flags;
	struct gsm_abis_mo mo;
	struct tlv_parsed nm_attr;
	uint8_t nm_chan_comb;
	int tsc;		/* -1 == use BTS TSC */

	struct {
		/* Parameters below are configured by VTY */
		int enabled;
		uint8_t maio;
		uint8_t hsn;
		struct bitvec arfcns;
		uint8_t arfcns_data[1024/8];
		/* This is the pre-computed MA for channel assignments */
		struct bitvec ma;
		uint8_t ma_len;	/* part of ma_data that is used */
		uint8_t ma_data[8];	/* 10.5.2.21: max 8 bytes value part */
	} hopping;

	/* To which E1 subslot are we connected */
	struct gsm_e1_subslot e1_link;

	struct gsm_lchan lchan[TS_MAX_LCHAN];
};

/* One TRX in a BTS */
struct gsm_bts_trx {
	/* list header in bts->trx_list */
	struct llist_head list;

	struct gsm_bts *bts;
	/* number of this TRX in the BTS */
	uint8_t nr;
	/* human readable name / description */
	char *description;
	/* how do we talk RSL with this TRX? */
	struct gsm_e1_subslot rsl_e1_link;
	uint8_t rsl_tei;
	struct e1inp_sign_link *rsl_link;

	/* Some BTS (specifically Ericsson RBS) have a per-TRX OML Link */
	struct e1inp_sign_link *oml_link;

	struct gsm_abis_mo mo;
	struct tlv_parsed nm_attr;
	struct {
		struct gsm_abis_mo mo;
	} bb_transc;

	uint16_t arfcn;
	int nominal_power;		/* in dBm */
	unsigned int max_power_red;	/* in actual dB */

#ifndef ROLE_BSC
	struct trx_power_params power_params;
	int ms_power_control;

	struct {
		void *l1h;
	} role_bts;
#endif

	union {
		struct {
			struct {
				struct gsm_abis_mo mo;
			} bbsig;
			struct {
				struct gsm_abis_mo mo;
			} pa;
		} bs11;
		struct {
			unsigned int test_state;
			uint8_t test_nr;
			struct rxlev_stats rxlev_stat;
		} ipaccess;
	};
	struct gsm_bts_trx_ts ts[TRX_NR_TS];
};

#define GSM_BTS_SI(bts, i)	(void *)(bts->si_buf[i])

enum gsm_bts_type {
	GSM_BTS_TYPE_UNKNOWN,
	GSM_BTS_TYPE_BS11,
	GSM_BTS_TYPE_NANOBTS,
	GSM_BTS_TYPE_RBS2000,
	GSM_BTS_TYPE_NOKIA_SITE,
	GSM_BTS_TYPE_OSMO_SYSMO,
	_NUM_GSM_BTS_TYPE
};

struct vty;

struct gsm_bts_model {
	struct llist_head list;

	enum gsm_bts_type type;
	const char *name;

	bool started;
	int (*start)(struct gsm_network *net);
	int (*oml_rcvmsg)(struct msgb *msg);

	void (*e1line_bind_ops)(struct e1inp_line *line);

	void (*config_write_bts)(struct vty *vty, struct gsm_bts *bts);
	void (*config_write_trx)(struct vty *vty, struct gsm_bts_trx *trx);
	void (*config_write_ts)(struct vty *vty, struct gsm_bts_trx_ts *ts);

	struct tlv_definition nm_att_tlvdef;

	struct bitvec features;
	uint8_t _features_data[128/8];
};

enum gsm_bts_features {
	BTS_FEAT_HSCSD,
	BTS_FEAT_GPRS,
	BTS_FEAT_EGPRS,
	BTS_FEAT_ECSD,
	BTS_FEAT_HOPPING,
	BTS_FEAT_MULTI_TSC,
};

/*
 * This keeps track of the paging status of one BTS. It
 * includes a number of pending requests, a back pointer
 * to the gsm_bts, a timer and some more state.
 */
struct gsm_bts_paging_state {
	/* pending requests */
	struct llist_head pending_requests;
	struct gsm_bts *bts;

	struct osmo_timer_list work_timer;
	struct osmo_timer_list credit_timer;

	/* free chans needed */
	int free_chans_need;

	/* load */
	uint16_t available_slots;
};

struct gsm_envabtse {
	struct gsm_abis_mo mo;
};

struct gsm_bts_gprs_nsvc {
	struct gsm_bts *bts;
	/* data read via VTY config file, to configure the BTS
	 * via OML from BSC */
	int id;
	uint16_t nsvci;
	uint16_t local_port;	/* on the BTS */
	uint16_t remote_port;	/* on the SGSN */
	uint32_t remote_ip;	/* on the SGSN */

	struct gsm_abis_mo mo;
};

enum gprs_rlc_par {
	RLC_T3142,
	RLC_T3169,
	RLC_T3191,
	RLC_T3193,
	RLC_T3195,
	RLC_N3101,
	RLC_N3103,
	RLC_N3105,
	CV_COUNTDOWN,
	T_DL_TBF_EXT,	/* ms */
	T_UL_TBF_EXT,	/* ms */
	_NUM_RLC_PAR
};

enum gprs_cs {
	GPRS_CS1,
	GPRS_CS2,
	GPRS_CS3,
	GPRS_CS4,
	GPRS_MCS1,
	GPRS_MCS2,
	GPRS_MCS3,
	GPRS_MCS4,
	GPRS_MCS5,
	GPRS_MCS6,
	GPRS_MCS7,
	GPRS_MCS8,
	GPRS_MCS9,
	_NUM_GRPS_CS
};

struct gprs_rlc_cfg {
	uint16_t parameter[_NUM_RLC_PAR];
	struct {
		uint16_t repeat_time; /* ms */
		uint8_t repeat_count;
	} paging;
	uint32_t cs_mask; /* bitmask of gprs_cs */
	uint8_t initial_cs;
	uint8_t initial_mcs;
};


enum neigh_list_manual_mode {
	NL_MODE_AUTOMATIC = 0,
	NL_MODE_MANUAL = 1,
	NL_MODE_MANUAL_SI5SEP = 2, /* SI2 and SI5 have separate neighbor lists */
};

enum bts_loc_fix {
	BTS_LOC_FIX_INVALID = 0,
	BTS_LOC_FIX_2D = 1,
	BTS_LOC_FIX_3D = 2,
};

extern const struct value_string bts_loc_fix_names[];

struct bts_location {
	struct llist_head list;
	time_t tstamp;
	enum bts_loc_fix valid;
	double lat;
	double lon;
	double height;
};

/* One BTS */
struct gsm_bts {
	/* list header in net->bts_list */
	struct llist_head list;

	/* Geographical location of the BTS */
	struct llist_head loc_list;

	/* number of ths BTS in network */
	uint8_t nr;
	/* human readable name / description */
	char *description;
	/* Cell Identity */
	uint16_t cell_identity;
	/* location area code of this BTS */
	uint16_t location_area_code;
	/* Training Sequence Code */
	uint8_t tsc;
	/* Base Station Identification Code (BSIC) */
	uint8_t bsic;
	/* type of BTS */
	enum gsm_bts_type type;
	struct gsm_bts_model *model;
	enum gsm_band band;
	/* maximum Tx power that the MS is permitted to use in this cell */
	int ms_max_power;

	/* how do we talk OML with this TRX? */
	struct gsm_e1_subslot oml_e1_link;
	uint8_t oml_tei;
	struct e1inp_sign_link *oml_link;

	/* Abis network management O&M handle */
	struct abis_nm_h *nmh;

	struct gsm_abis_mo mo;

	/* number of this BTS on given E1 link */
	uint8_t bts_nr;

	/* paging state and control */
	struct gsm_bts_paging_state paging;

	/* CCCH is on C0 */
	struct gsm_bts_trx *c0;

	struct {
		struct gsm_abis_mo mo;
	} site_mgr;

	/* bitmask of all SI that are present/valid in si_buf */
	uint32_t si_valid;
	/* buffers where we put the pre-computed SI */
	sysinfo_buf_t si_buf[_MAX_SYSINFO_TYPE];

	/* TimeZone hours, mins, and bts specific */
	struct {
		int hr;
		int mn;
		int override;
		int dst;
	} tz;

	/* ip.accesss Unit ID's have Site/BTS/TRX layout */
	union {
		struct {
			uint16_t site_id;
			uint16_t bts_id;
			uint32_t flags;
			uint32_t rsl_ip;
		} ip_access;
		struct {
			struct {
				struct gsm_abis_mo mo;
			} cclk;
			struct {
				struct gsm_abis_mo mo;
			} rack;
			struct gsm_envabtse envabtse[4];
		} bs11;
		struct {
			struct {
				struct gsm_abis_mo mo;
				struct llist_head conn_groups;
			} is;
			struct {
				struct gsm_abis_mo mo;
				struct llist_head conn_groups;
			} con;
			struct {
				struct gsm_abis_mo mo;
			} dp;
			struct {
				struct gsm_abis_mo mo;
			} tf;
		} rbs2000;
		struct {
			uint8_t bts_type;
			unsigned int configured:1,
				skip_reset:1,
				no_loc_rel_cnf:1,
				bts_reset_timer_cnf,
				did_reset:1,
				wait_reset:1;
			struct osmo_timer_list reset_timer;
		} nokia;
	};

	/* Not entirely sure how ip.access specific this is */
	struct {
		enum bts_gprs_mode mode;
		struct {
			struct gsm_abis_mo mo;
			uint16_t nsei;
			uint8_t timer[7];
		} nse;
		struct {
			struct gsm_abis_mo mo;
			uint16_t bvci;
			uint8_t timer[11];
			struct gprs_rlc_cfg rlc_cfg;
		} cell;
		struct gsm_bts_gprs_nsvc nsvc[2];
		uint8_t rac;
		uint8_t net_ctrl_ord;
	} gprs;

	/* RACH NM values */
	int rach_b_thresh;
	int rach_ldavg_slots;

	/* transceivers */
	int num_trx;
	struct llist_head trx_list;

	/* SI related items */
	int force_combined_si;
	int bcch_change_mark;

#ifdef ROLE_BSC
	/* Abis NM queue */
	struct llist_head abis_queue;
	int abis_nm_pend;

	struct gsm_network *network;

	/* should the channel allocator allocate channels from high TRX to TRX0,
	 * rather than starting from TRX0 and go upwards? */
	int chan_alloc_reverse;

	enum neigh_list_manual_mode neigh_list_manual_mode;
	/* parameters from which we build SYSTEM INFORMATION */
	struct {
		struct gsm48_rach_control rach_control;
		uint8_t ncc_permitted;
		struct gsm48_cell_sel_par cell_sel_par;
		struct gsm48_si_selection_params cell_ro_sel_par; /* rest octet */
		struct gsm48_cell_options cell_options;
		struct gsm48_control_channel_descr chan_desc;
		struct bitvec neigh_list;
		struct bitvec cell_alloc;
		struct bitvec si5_neigh_list;
		struct {
			/* bitmask large enough for all possible ARFCN's */
			uint8_t neigh_list[1024/8];
			uint8_t cell_alloc[1024/8];
			/* If the user wants a different neighbor list in SI5 than in SI2 */
			uint8_t si5_neigh_list[1024/8];
		} data;
	} si_common;

	/* do we use static (user-defined) system information messages? (bitmask) */
	uint32_t si_mode_static;

	/* exclude the BTS from the global RF Lock handling */
	int excl_from_rf_lock;

	/* supported codecs beside FR */
	struct bts_codec_conf codec;

	/* BTS dependencies bit field */
	uint32_t depends_on[256/(8*4)];
#endif /* ROLE_BSC */
	void *role;
};


struct gsm_bts *gsm_bts_alloc(void *talloc_ctx);
struct gsm_bts *gsm_bts_num(struct gsm_network *net, int num);

struct gsm_bts_trx *gsm_bts_trx_alloc(struct gsm_bts *bts);
struct gsm_bts_trx *gsm_bts_trx_num(const struct gsm_bts *bts, int num);


const struct value_string gsm_pchant_names[12];
const struct value_string gsm_pchant_descs[12];
const struct value_string gsm_lchant_names[8];
const char *gsm_pchan_name(enum gsm_phys_chan_config c);
enum gsm_phys_chan_config gsm_pchan_parse(const char *name);
const char *gsm_lchant_name(enum gsm_chan_t c);
const char *gsm_chreq_name(enum gsm_chreq_reason_t c);
char *gsm_trx_name(const struct gsm_bts_trx *trx);
char *gsm_ts_name(const struct gsm_bts_trx_ts *ts);
char *gsm_lchan_name(const struct gsm_lchan *lchan);
const char *gsm_lchans_name(enum gsm_lchan_state s);


void gsm_abis_mo_reset(struct gsm_abis_mo *mo);

struct gsm_abis_mo *
gsm_objclass2mo(struct gsm_bts *bts, uint8_t obj_class,
	    const struct abis_om_obj_inst *obj_inst);

struct gsm_nm_state *
gsm_objclass2nmstate(struct gsm_bts *bts, uint8_t obj_class,
		 const struct abis_om_obj_inst *obj_inst);
void *
gsm_objclass2obj(struct gsm_bts *bts, uint8_t obj_class,
	     const struct abis_om_obj_inst *obj_inst);

/* reset the state of all MO in the BTS */
void gsm_bts_mo_reset(struct gsm_bts *bts);

uint8_t gsm_ts2chan_nr(const struct gsm_bts_trx_ts *ts, uint8_t lchan_nr);
uint8_t gsm_lchan2chan_nr(const struct gsm_lchan *lchan);

/* return the gsm_lchan for the CBCH (if it exists at all) */
struct gsm_lchan *gsm_bts_get_cbch(struct gsm_bts *bts);

/*
 * help with parsing regexps
 */
int gsm_parse_reg(void *ctx, regex_t *reg, char **str,
		int argc, const char **argv) __attribute__ ((warn_unused_result));

static inline uint8_t gsm_ts_tsc(const struct gsm_bts_trx_ts *ts)
{
	if (ts->tsc != -1)
		return ts->tsc;
	else
		return ts->trx->bts->tsc;
}


#endif
