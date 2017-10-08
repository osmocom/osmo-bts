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
#include <osmocom/gsm/protocol/gsm_04_08.h>
#include <osmocom/gsm/protocol/gsm_08_58.h>
#include <osmocom/gsm/protocol/gsm_12_21.h>

#include <osmocom/abis/e1_input.h>

#ifndef ROLE_BSC
#include <osmocom/gsm/lapdm.h>
#endif

/* 16 is the max. number of SI2quater messages according to 3GPP TS 44.018 Table 10.5.2.33b.1:
   4-bit index is used (2#1111 = 10#15) */
#define SI2Q_MAX_NUM 16
/* length in bits (for single SI2quater message) */
#define SI2Q_MAX_LEN 160
#define SI2Q_MIN_LEN 18

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
	GSM_CHREQ_REASON_PDCH,
};

/* lchans 0..3 are SDCCH in combined channel configuration,
   use 4 as magic number for BCCH hack - see osmo-bts-../oml.c:opstart_compl() */
#define CCCH_LCHAN 4

#define TRX_NR_TS	8
#define TS_MAX_LCHAN	8

#define HARDCODED_ARFCN 123
#define HARDCODED_BSIC	0x3f	/* NCC = 7 / BCC = 7 */

/* for multi-drop config */
#define HARDCODED_BTS0_TS	1
#define HARDCODED_BTS1_TS	6
#define HARDCODED_BTS2_TS	11

#define MAX_VERSION_LENGTH 64

#define MAX_BTS_FEATURES 128

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

/* Ericsson OM2000 Managed Object */
struct abis_om2k_mo {
	uint8_t class;
	uint8_t bts;
	uint8_t assoc_so;
	uint8_t inst;
} __attribute__ ((packed));

struct om2k_mo {
	struct abis_om2k_mo addr;
	struct osmo_fsm_inst *fsm;
};

#define MAX_A5_KEY_LEN	(128/8)
#define A38_XOR_MIN_KEY_LEN	12
#define A38_XOR_MAX_KEY_LEN	16
#define A38_COMP128_KEY_LEN	16
#define RSL_ENC_ALG_A5(x)	(x+1)
#define MAX_EARFCN_LIST 32

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
	struct amr_mode ms_mode[4];
	struct amr_mode bts_mode[4];
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
	uint8_t mr_ms_lv[7];
	uint8_t mr_bts_lv[7];

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

	char *name;

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

	struct {
		/* channel activation type and handover ref */
		uint8_t act_type;
		uint8_t ho_ref;
		struct gsm48_req_ref *rqd_ref;
		uint8_t rqd_ta;
	} dyn;
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
		/* buffers where we put the pre-computed SI:
		   SI2Q_MAX_NUM is the max number of SI2quater messages (see 3GPP TS 44.018) */
		sysinfo_buf_t buf[_MAX_SYSINFO_TYPE][SI2Q_MAX_NUM];
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
			struct osmo_fsm_inst *dl_amr_fsm;
			/* TCH cache */
			uint8_t cache[20];
			/* FACCH cache */
			uint8_t facch[GSM_MACBLOCK_LEN];
			uint8_t len;
			uint32_t fn;
			bool is_update;
			/* set for each SID frame to detect talkspurt for codecs
			   without explicit ONSET event */
			bool ul_sid;
			/* indicates if DTXd was active during DL measurement
			   period */
			bool dl_active;
		} dtx;
		uint8_t last_cmr;
		uint32_t last_fn;
	} tch;

	/* 3GPP TS 48.058 § 9.3.37: [0; 255] ok, -1 means invalid*/
	int16_t ms_t_offs;
	/* 3GPP TS 45.010 § 1.2 round trip propagation delay (in symbols) or -1 */
	int16_t p_offs;

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
	/* RTP header Marker bit to indicate beginning of speech after pause  */
	bool rtp_tx_marker;
	/* power handling */
	struct {
		uint8_t current;
		uint8_t fixed;
	} ms_power_ctrl;

	struct msgb *pending_rel_ind_msg;
#endif
};

enum {
	TS_F_PDCH_ACTIVE =		0x1000,
	TS_F_PDCH_ACT_PENDING =		0x2000,
	TS_F_PDCH_DEACT_PENDING =	0x4000,
	TS_F_PDCH_PENDING_MASK =	0x6000 /*<
			TS_F_PDCH_ACT_PENDING | TS_F_PDCH_DEACT_PENDING */
} gsm_bts_trx_ts_flags;

/* One Timeslot in a TRX */
struct gsm_bts_trx_ts {
	struct gsm_bts_trx *trx;
	/* number of this timeslot at the TRX */
	uint8_t nr;

	enum gsm_phys_chan_config pchan;

	struct {
		enum gsm_phys_chan_config pchan_is;
		enum gsm_phys_chan_config pchan_want;
		struct msgb *pending_chan_activ;
	} dyn;

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

	union {
		struct {
			struct om2k_mo om2k_mo;
		} rbs2000;
	};

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
		struct {
			struct {
				struct om2k_mo om2k_mo;
			} trxc;
			struct {
				struct om2k_mo om2k_mo;
			} rx;
			struct {
				struct om2k_mo om2k_mo;
			} tx;
		} rbs2000;
	};
	struct gsm_bts_trx_ts ts[TRX_NR_TS];
};

#define GSM_BTS_SI2Q(bts, i)   (struct gsm48_system_information_type_2quater *)((bts)->si_buf[SYSINFO_TYPE_2quater][i])
#define GSM_BTS_HAS_SI(bts, i) ((bts)->si_valid & (1 << i))
#define GSM_BTS_SI(bts, i)     (void *)((bts)->si_buf[i][0])
#define GSM_LCHAN_SI(lchan, i) (void *)((lchan)->si.buf[i][0])

enum gsm_bts_type {
	GSM_BTS_TYPE_UNKNOWN,
	GSM_BTS_TYPE_BS11,
	GSM_BTS_TYPE_NANOBTS,
	GSM_BTS_TYPE_RBS2000,
	GSM_BTS_TYPE_NOKIA_SITE,
	GSM_BTS_TYPE_OSMOBTS,
	_NUM_GSM_BTS_TYPE
};

enum gsm_bts_type_variant {
	BTS_UNKNOWN,
	BTS_OSMO_LITECELL15,
	BTS_OSMO_OCTPHY,
	BTS_OSMO_SYSMO,
	BTS_OSMO_TRX,
	_NUM_BTS_VARIANT
};

/* Used by OML layer for BTS Attribute reporting */
enum bts_attribute {
	BTS_TYPE_VARIANT,
	BTS_SUB_MODEL,
	TRX_PHY_VERSION,
};

struct vty;

struct gsm_bts_model {
	struct llist_head list;

	enum gsm_bts_type type;
	enum gsm_bts_type_variant variant;
	const char *name;

	bool started;
	int (*start)(struct gsm_network *net);
	int (*oml_rcvmsg)(struct msgb *msg);

	void (*e1line_bind_ops)(struct e1inp_line *line);

	void (*config_write_bts)(struct vty *vty, struct gsm_bts *bts);
	void (*config_write_trx)(struct vty *vty, struct gsm_bts_trx *trx);
	void (*config_write_ts)(struct vty *vty, struct gsm_bts_trx_ts *ts);

	struct tlv_definition nm_att_tlvdef;

	/* features of a given BTS model set via gsm_bts_model_register() locally */
	struct bitvec features;
	uint8_t _features_data[MAX_BTS_FEATURES/8];
};

/* N. B: always add new features to the end of the list (right before _NUM_BTS_FEAT) to avoid breaking compatibility
   with BTS compiled against earlier version of this header */
enum gsm_bts_features {
	BTS_FEAT_HSCSD,
	BTS_FEAT_GPRS,
	BTS_FEAT_EGPRS,
	BTS_FEAT_ECSD,
	BTS_FEAT_HOPPING,
	BTS_FEAT_MULTI_TSC,
	BTS_FEAT_OML_ALERTS,
	BTS_FEAT_AGCH_PCH_PROP,
	BTS_FEAT_CBCH,
	_NUM_BTS_FEAT
};

extern const struct value_string gsm_bts_features_descs[];

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
	/* Base Station Identification Code (BSIC), lower 3 bits is BCC,
	 * which is used as TSC for the CCCH */
	uint8_t bsic;
	/* type of BTS */
	enum gsm_bts_type type;
	enum gsm_bts_type_variant variant;
	struct gsm_bts_model *model;
	enum gsm_band band;
	char version[MAX_VERSION_LENGTH];
	char sub_model[MAX_VERSION_LENGTH];

	/* features of a given BTS set/reported via OML */
	struct bitvec features;
	uint8_t _features_data[MAX_BTS_FEATURES/8];

	/* Connected PCU version (if any) */
	char pcu_version[MAX_VERSION_LENGTH];

	/* maximum Tx power that the MS is permitted to use in this cell */
	int ms_max_power;

	/* how do we talk OML with this TRX? */
	struct gsm_e1_subslot oml_e1_link;
	uint8_t oml_tei;
	struct e1inp_sign_link *oml_link;
	/* when OML link was established */
	time_t uptime;

	/* Abis network management O&M handle */
	struct abis_nm_h *nmh;

	struct gsm_abis_mo mo;

	/* number of this BTS on given E1 link */
	uint8_t bts_nr;

	/* DTX features of this BTS */
	enum gsm48_dtx_mode dtxu;
	bool dtxd;

	/* paging state and control */
	struct gsm_bts_paging_state paging;

	/* CCCH is on C0 */
	struct gsm_bts_trx *c0;

	struct {
		struct gsm_abis_mo mo;
	} site_mgr;

	/* bitmask of all SI that are present/valid in si_buf */
	uint32_t si_valid;
	/* 3GPP TS 44.018 Table 10.5.2.33b.1 INDEX and COUNT for SI2quater */
	uint8_t si2q_index; /* distinguish individual SI2quater messages */
	uint8_t si2q_count; /* si2q_index for the last (highest indexed) individual SI2quater message */
	/* buffers where we put the pre-computed SI */
	sysinfo_buf_t si_buf[_MAX_SYSINFO_TYPE][SI2Q_MAX_NUM];
	/* offsets used while generating SI2quater */
	size_t e_offset;
	size_t u_offset;

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
				struct om2k_mo om2k_mo;
				struct gsm_abis_mo mo;
				struct llist_head conn_groups;
			} cf;
			struct {
				struct om2k_mo om2k_mo;
				struct gsm_abis_mo mo;
				struct llist_head conn_groups;
			} is;
			struct {
				struct om2k_mo om2k_mo;
				struct gsm_abis_mo mo;
				struct llist_head conn_groups;
			} con;
			struct {
				struct om2k_mo om2k_mo;
				struct gsm_abis_mo mo;
			} dp;
			struct {
				struct om2k_mo om2k_mo;
				struct gsm_abis_mo mo;
			} tf;
			uint32_t use_superchannel:1;
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
		uint8_t supports_egprs_11bit_rach;
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
		bool ctrl_ack_type_use_block;
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
		struct osmo_earfcn_si2q si2quater_neigh_list;
		size_t uarfcn_length; /* index for uarfcn and scramble lists */
		struct {
			/* bitmask large enough for all possible ARFCN's */
			uint8_t neigh_list[1024/8];
			uint8_t cell_alloc[1024/8];
			/* If the user wants a different neighbor list in SI5 than in SI2 */
			uint8_t si5_neigh_list[1024/8];
			uint8_t meas_bw_list[MAX_EARFCN_LIST];
			uint16_t earfcn_list[MAX_EARFCN_LIST];
			uint16_t uarfcn_list[MAX_EARFCN_LIST];
			uint16_t scramble_list[MAX_EARFCN_LIST];
		} data;
	} si_common;
	bool early_classmark_allowed;
	/* for testing only: Have an infinitely long radio link timeout */
	bool infinite_radio_link_timeout;

	/* do we use static (user-defined) system information messages? (bitmask) */
	uint32_t si_mode_static;

	/* exclude the BTS from the global RF Lock handling */
	int excl_from_rf_lock;

	/* supported codecs beside FR */
	struct bts_codec_conf codec;

	/* BTS dependencies bit field */
	uint32_t depends_on[256/(8*4)];

	/* full and half rate multirate config */
	struct amr_multirate_conf mr_full;
	struct amr_multirate_conf mr_half;

	/* PCU socket state */
	char *pcu_sock_path;
	struct pcu_sock_state *pcu_state;

#endif /* ROLE_BSC */
	void *role;
};


struct gsm_bts *gsm_bts_alloc(void *talloc_ctx, uint8_t bts_num);
struct gsm_bts *gsm_bts_num(struct gsm_network *net, int num);

struct gsm_bts_trx *gsm_bts_trx_alloc(struct gsm_bts *bts);
struct gsm_bts_trx *gsm_bts_trx_num(const struct gsm_bts *bts, int num);

enum gsm_bts_type str2btstype(const char *arg);
const char *btstype2str(enum gsm_bts_type type);

enum bts_attribute str2btsattr(const char *s);
const char *btsatttr2str(enum bts_attribute v);

enum gsm_bts_type_variant str2btsvariant(const char *arg);
const char *btsvariant2str(enum gsm_bts_type_variant v);

extern const struct value_string gsm_chreq_descs[];
const struct value_string gsm_pchant_names[13];
const struct value_string gsm_pchant_descs[13];
const char *gsm_pchan_name(enum gsm_phys_chan_config c);
enum gsm_phys_chan_config gsm_pchan_parse(const char *name);
const char *gsm_lchant_name(enum gsm_chan_t c);
const char *gsm_chreq_name(enum gsm_chreq_reason_t c);
char *gsm_trx_name(const struct gsm_bts_trx *trx);
char *gsm_ts_name(const struct gsm_bts_trx_ts *ts);
char *gsm_ts_and_pchan_name(const struct gsm_bts_trx_ts *ts);
char *gsm_lchan_name_compute(const struct gsm_lchan *lchan);
const char *gsm_lchans_name(enum gsm_lchan_state s);

static inline char *gsm_lchan_name(const struct gsm_lchan *lchan)
{
	return lchan->name;
}

static inline int gsm_bts_set_feature(struct gsm_bts *bts, enum gsm_bts_features feat)
{
	OSMO_ASSERT(_NUM_BTS_FEAT < MAX_BTS_FEATURES);
	return bitvec_set_bit_pos(&bts->features, feat, 1);
}

static inline bool gsm_bts_has_feature(const struct gsm_bts *bts, enum gsm_bts_features feat)
{
	OSMO_ASSERT(_NUM_BTS_FEAT < MAX_BTS_FEATURES);
	return bitvec_get_bit_pos(&bts->features, feat);
}

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

uint8_t gsm_pchan2chan_nr(enum gsm_phys_chan_config pchan,
			  uint8_t ts_nr, uint8_t lchan_nr);
uint8_t gsm_lchan2chan_nr(const struct gsm_lchan *lchan);
uint8_t gsm_lchan_as_pchan2chan_nr(const struct gsm_lchan *lchan,
				   enum gsm_phys_chan_config as_pchan);

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
		return ts->trx->bts->bsic & 7;
}

struct gsm_lchan *rsl_lchan_lookup(struct gsm_bts_trx *trx, uint8_t chan_nr,
				   int *rc);

enum gsm_phys_chan_config ts_pchan(struct gsm_bts_trx_ts *ts);
uint8_t ts_subslots(struct gsm_bts_trx_ts *ts);
bool ts_is_tch(struct gsm_bts_trx_ts *ts);

#endif
