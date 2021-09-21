#ifndef _BTS_H
#define _BTS_H

#include <osmocom/core/rate_ctr.h>
#include <osmocom/core/socket.h>
#include <osmo-bts/gsm_data.h>
#include <osmo-bts/bts_trx.h>


struct gsm_bts_trx;

enum bts_global_status {
	BTS_STATUS_RF_ACTIVE,
	BTS_STATUS_RF_MUTE,
	BTS_STATUS_LAST,
};

enum {
	BTS_CTR_PAGING_RCVD,
	BTS_CTR_PAGING_DROP,
	BTS_CTR_PAGING_DROP_PS,
	BTS_CTR_PAGING_SENT,
	BTS_CTR_PAGING_CONG,
	BTS_CTR_RACH_RCVD,
	BTS_CTR_RACH_DROP,
	BTS_CTR_RACH_HO,
	BTS_CTR_RACH_CS,
	BTS_CTR_RACH_PS,
	BTS_CTR_AGCH_RCVD,
	BTS_CTR_AGCH_SENT,
	BTS_CTR_AGCH_DELETED,
};

/* Used by OML layer for BTS Attribute reporting */
enum bts_attribute {
	BTS_TYPE_VARIANT,
	BTS_SUB_MODEL,
	TRX_PHY_VERSION,
};
const char *btsatttr2str(enum bts_attribute v);

enum gsm_bts_type_variant {
	BTS_UNKNOWN,
	BTS_OSMO_LITECELL15,
        BTS_OSMO_OC2G,
	BTS_OSMO_OCTPHY,
	BTS_OSMO_SYSMO,
	BTS_OSMO_TRX,
	BTS_OSMO_VIRTUAL,
	BTS_OSMO_OMLDUMMY,
	_NUM_BTS_VARIANT
};
const char *btsvariant2str(enum gsm_bts_type_variant v);

/* TODO: add a brief description of this flag */
#define BTS_INTERNAL_FLAG_MS_PWR_CTRL_DSP		(1 << 0)
/* When this flag is set then the measurement data is included in
 * (PRIM_PH_DATA) and struct ph_tch_param (PRIM_TCH). Otherwise the
 * measurement data is passed using a separate MPH INFO MEAS IND.
 * (See also ticket: OS#2977) */
#define BTS_INTERNAL_FLAG_MEAS_PAYLOAD_COMB		(1 << 1)
/* Whether the BTS model requires RadioCarrier MO to be in Enabled state
 * (OPSTARTed) before OPSTARTing the RadioChannel MOs. See OS#5157 */
#define BTS_INTERNAL_FLAG_NM_RCHANNEL_DEPENDS_RCARRIER	(1 << 2)

/* BTS implementation flags (internal use, not exposed via OML) */
#define bts_internal_flag_get(bts, flag) \
	((bts->flags & (typeof(bts->flags)) flag) != 0)
#define bts_internal_flag_set(bts, flag) \
	bts->flags |= (typeof(bts->flags)) flag

struct gsm_bts_gprs_nsvc {
	struct gsm_bts *bts;
	/* data read via VTY config file, to configure the BTS
	 * via OML from BSC */
	int id;
	uint16_t nsvci;
	struct osmo_sockaddr local;	/* on the BTS */
	struct osmo_sockaddr remote;	/* on the SGSN */

	struct gsm_abis_mo mo;
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

struct bts_smscb_state {
	struct llist_head queue; /* list of struct smscb_msg */
	int queue_len;
	struct rate_ctr_group *ctrs;
	struct smscb_msg *cur_msg; /* current SMS-CB */
	struct smscb_msg *default_msg; /* default broadcast message; NULL if none */
};

/* Tx power filtering algorithm */
enum bts_pf_algo {
	BTS_PF_ALGO_NONE = 0,
	BTS_PF_ALGO_EWMA,
};

/* UL/DL power control parameters */
struct bts_power_ctrl_params {
	/* Target value to strive to */
	int target_dbm;
	/* Tolerated deviation from target */
	int hysteresis_db;
	/* How many dB do we raise power at maximum */
	int raise_step_max_db;
	/* How many dB do we lower power at maximum */
	int lower_step_max_db;
	/* RxLev filtering algorithm */
	enum bts_pf_algo pf_algo;
	/* (Optional) filtering parameters */
	union {
		/* Exponentially Weighted Moving Average */
		struct {
			/* Smoothing factor: higher the value - less smoothing */
			uint8_t alpha; /* 1 .. 99 (in %) */
		} ewma;
	} pf;
};

/* BTS Site Manager */
struct gsm_bts_sm {
	struct gsm_abis_mo mo;
};

/* Struct that holds one OML-Address (Address of the BSC) */
struct bsc_oml_host {
	struct llist_head list;
	char *addr;
};

/* One BTS */
struct gsm_bts {
	/* list header in net->bts_list */
	struct llist_head list;

	/* number of the BTS in network */
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
	enum gsm_bts_type_variant variant;
	enum gsm_band band;
	char version[MAX_VERSION_LENGTH];
	char sub_model[MAX_VERSION_LENGTH];

	/* public features of a given BTS (set/reported via OML) */
	struct bitvec *features;
	/* implementation flags of a given BTS (not exposed via OML) */
	uint16_t flags;

	/* Connected PCU version (if any) */
	char pcu_version[MAX_VERSION_LENGTH];

	/* maximum Tx power that the MS is permitted to use in this cell */
	int ms_max_power;

	/* how do we talk OML with this TRX? */
	struct e1inp_sign_link *oml_link;
	struct timespec oml_conn_established_timestamp;
	/* OSMO extenion link associated to same line as oml_link: */
	struct e1inp_sign_link *osmo_link;

	/* Abis network management O&M handle */
	struct gsm_abis_mo mo;

	/* number of this BTS on given E1 link */
	uint8_t bts_nr;

	/* DTX features of this BTS */
	bool dtxd;

	/* CCCH is on C0 */
	struct gsm_bts_trx *c0;

	struct gsm_bts_sm site_mgr;

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
	/* decoded SI rest octets - *unmodified* as received from BSC */
	struct osmo_gsm48_si_ro_info si3_ro_decoded;
	struct osmo_gsm48_si_ro_info si4_ro_decoded;
	/* is SI GPRS Indicator currently disabled due to lack of PCU connection? */
	bool si_gprs_ind_disabled;

	/* ip.access Unit ID's have Site/BTS/TRX layout */
	union {
		struct {
			uint16_t site_id;
			uint16_t bts_id;
			uint32_t flags;
			uint32_t rsl_ip;
		} ip_access;
	};

	/* Not entirely sure how ip.access specific this is */
	struct {
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
	} gprs;

	/* transceivers */
	int num_trx;
	struct llist_head trx_list;

	struct rate_ctr_group *ctrs;
	bool supp_meas_toa256;

	struct {
		/* Interference Boundaries for OML */
		int16_t boundary[6];
		uint8_t intave;
	} interference;
	unsigned int t200_ms[7];
	unsigned int t3105_ms;
	struct {
		uint8_t overload_period;
		struct {
			/* Input parameters from OML */
			uint8_t load_ind_thresh;	/* percent */
			uint8_t load_ind_period;	/* seconds */
			/* Internal data */
			struct osmo_timer_list timer;
			unsigned int pch_total;
			unsigned int pch_used;
		} ccch;
		struct {
			/* Input parameters from OML */
			int16_t busy_thresh;		/* in dBm */
			uint16_t averaging_slots;
			/* Internal data */
			unsigned int total;	/* total nr */
			unsigned int busy;	/* above busy_thresh */
			unsigned int access;	/* access bursts */
		} rach;
	} load;
	uint8_t ny1;
	uint8_t max_ta;

	/* AGCH queuing */
	struct {
		struct llist_head queue;
		int length;
		int max_length;

		int thresh_level;	/* Cleanup threshold in percent of max len */
		int low_level;		/* Low water mark in percent of max len */
		int high_level;		/* High water mark in percent of max len */

		/* TODO: Use a rate counter group instead */
		uint64_t dropped_msgs;
		uint64_t merged_msgs;
		uint64_t rejected_msgs;
		uint64_t agch_msgs;
		uint64_t pch_msgs;
	} agch_queue;

	struct {
		uint8_t *prim_notif;	/* ETWS primary notification (NULL if none) */
		ssize_t prim_notif_len;	/* Length of prim_notif; expected 56 bytes */
		uint8_t page_size;
		uint8_t num_pages;	/* total number of pages */
		uint8_t next_page;	/* next page number to be sent */
		bool pni;		/* Primary Notification Identifier */
	} etws;

	struct paging_state *paging_state;
	struct llist_head bsc_oml_hosts;
	unsigned int rtp_jitter_buf_ms;
	bool rtp_jitter_adaptive;

	uint16_t rtp_port_range_start;
	uint16_t rtp_port_range_end;
	uint16_t rtp_port_range_next;
	int rtp_ip_dscp;
	int rtp_priority;

	struct {
		uint8_t ciphers;	/* flags A5/1==0x1, A5/2==0x2, A5/3==0x4 */
	} support;
	struct {
		uint8_t tc4_ctr;
	} si;
	struct gsm_time gsm_time;
	/* frame number statistics (FN in PH-RTS.ind vs. PH-DATA.ind */
	struct {
		int32_t min;		/* minimum observed */
		int32_t max;		/* maximum observed */
		int32_t avg256;		/* accumulator */
		uint32_t avg_count;	/* number of samples accumulated in avg256 */
		uint32_t avg_window;	/* number of averages in avg_count */
	} fn_stats;
	/* Radio Link Timeout counter. -1 disables timeout for
	 * lab/measurement purpose */
	struct {
		int oml;		/* value communicated by BSC in OML */
		int current;		/* actual currently applied value */
		bool vty_override;	/* OML value overridden by VTY */
	} radio_link_timeout;

	/* Default (fall-back) Dynamic Power Control parameters for all transceivers */
	struct gsm_power_ctrl_params bs_dpc_params; /* BS Dynamic Power Control */
	struct gsm_power_ctrl_params ms_dpc_params; /* MS Dynamic Power Control */

	/* Maximum BCCH carrier power reduction */
	uint8_t c0_power_red_db;

	/* used by the sysmoBTS to adjust band */
	uint8_t auto_band;

	/* State for SMSCB (Cell Broadcast) for BASIC and EXTENDED channel */
	struct bts_smscb_state smscb_basic;
	struct bts_smscb_state smscb_extended;
	int smscb_queue_tgt_len; /* ideal/target queue length */
	int smscb_queue_max_len; /* maximum queue length */
	int smscb_queue_hyst; /* hysteresis for CBCH load indications */

	int16_t min_qual_rach;	/* minimum link quality (in centiBels) for Access Bursts */
	int16_t min_qual_norm;	/* minimum link quality (in centiBels) for Normal Bursts */
	uint16_t max_ber10k_rach;	/* Maximum permitted RACH BER in 0.01% */

	struct {
		char *sock_path;
	} pcu;

	/* GSMTAP Um logging (disabled by default) */
	struct {
		struct gsmtap_inst *inst;
		char *remote_host;
		uint32_t sapi_mask;
		uint8_t sapi_acch;
	} gsmtap;

	struct osmo_fsm_inst *shutdown_fi; /* FSM instance to manage shutdown procedure during process exit */
	bool shutdown_fi_exit_proc; /* exit process when shutdown_fsm is finished? */
	struct osmo_fsm_inst *abis_link_fi; /* FSM instance to manage abis connection during process startup and link failure */
	struct osmo_tdef *T_defs; /* Timer defines */

	void *model_priv; /* Allocated by bts_model, contains model specific data pointer */
};

extern const struct value_string bts_impl_flag_desc[];
extern void *tall_bts_ctx;

#define GSM_BTS_SI2Q(bts, i)   (struct gsm48_system_information_type_2quater *)((bts)->si_buf[SYSINFO_TYPE_2quater][i])
#define GSM_BTS_HAS_SI(bts, i) ((bts)->si_valid & (1 << i))
#define GSM_BTS_SI(bts, i)     (void *)((bts)->si_buf[i][0])

static inline struct gsm_bts *gsm_bts_sm_get_bts(struct gsm_bts_sm *site_mgr) {
	return (struct gsm_bts *)container_of(site_mgr, struct gsm_bts, site_mgr);
}

struct gsm_bts *gsm_bts_alloc(void *talloc_ctx, uint8_t bts_num);
struct gsm_bts *gsm_bts_num(const struct gsm_network *net, int num);

int bts_init(struct gsm_bts *bts);
void bts_shutdown(struct gsm_bts *bts, const char *reason);
void bts_shutdown_ext(struct gsm_bts *bts, const char *reason, bool exit_proc);

int bts_link_estab(struct gsm_bts *bts);

int bts_agch_enqueue(struct gsm_bts *bts, struct msgb *msg);
struct msgb *bts_agch_dequeue(struct gsm_bts *bts);
int bts_agch_max_queue_length(int T, int bcch_conf);
int bts_ccch_copy_msg(struct gsm_bts *bts, uint8_t *out_buf, struct gsm_time *gt,
		      int is_ag_res);
int bts_supports_cipher(struct gsm_bts *bts, int rsl_cipher);
uint8_t *bts_sysinfo_get(struct gsm_bts *bts, const struct gsm_time *g_time);
void regenerate_si3_restoctets(struct gsm_bts *bts);
void regenerate_si4_restoctets(struct gsm_bts *bts);
int get_si4_ro_offset(const uint8_t *si4_buf);
uint8_t *lchan_sacch_get(struct gsm_lchan *lchan);
int lchan_init_lapdm(struct gsm_lchan *lchan);

void load_timer_start(struct gsm_bts *bts);
void bts_update_status(enum bts_global_status which, int on);

struct gsm_time *get_time(struct gsm_bts *bts);

int bts_main(int argc, char **argv);

int bts_supports_cm(const struct gsm_bts *bts,
		    const struct rsl_ie_chan_mode *cm);

int32_t bts_get_avg_fn_advance(const struct gsm_bts *bts);

/* return the gsm_lchan for the CBCH (if it exists at all) */
struct gsm_lchan *gsm_bts_get_cbch(struct gsm_bts *bts);

int bts_set_c0_pwr_red(struct gsm_bts *bts, const uint8_t red);

#endif /* _BTS_H */
