#ifndef _GSM_DATA_H
#define _GSM_DATA_H

#include <stdbool.h>
#include <stdint.h>

#include <osmocom/core/timer.h>
#include <osmocom/core/bitvec.h>
#include <osmocom/core/statistics.h>
#include <osmocom/core/utils.h>
#include <osmocom/core/linuxlist.h>
#include <osmocom/codec/ecu.h>
#include <osmocom/gsm/lapdm.h>
#include <osmocom/gsm/gsm23003.h>
#include <osmocom/gsm/gsm0502.h>
#include <osmocom/gsm/gsm_utils.h>
#include <osmocom/gsm/tlv.h>
#include <osmocom/gsm/rxlev_stat.h>
#include <osmocom/gsm/sysinfo.h>
#include <osmocom/gsm/meas_rep.h>
#include <osmocom/gsm/bts_features.h>
#include <osmocom/gsm/gsm48_rest_octets.h>
#include <osmocom/gsm/protocol/gsm_04_08.h>
#include <osmocom/gsm/protocol/gsm_08_58.h>
#include <osmocom/gsm/protocol/gsm_12_21.h>

#include <osmocom/abis/e1_input.h>

#include <osmo-bts/paging.h>
#include <osmo-bts/tx_power.h>
#include <osmo-bts/oml.h>

#define GSM_FR_BITS	260
#define GSM_EFR_BITS	244

#define GSM_FR_BYTES	33	/* TS 101318 Chapter 5.1: 260 bits + 4bit sig */
#define GSM_HR_BYTES	14	/* TS 101318 Chapter 5.2: 112 bits, no sig */
#define GSM_EFR_BYTES	31	/* TS 101318 Chapter 5.3: 244 bits + 4bit sig */

#define GSM_BTS_AGCH_QUEUE_THRESH_LEVEL_DEFAULT 41
#define GSM_BTS_AGCH_QUEUE_THRESH_LEVEL_DISABLE 999999
#define GSM_BTS_AGCH_QUEUE_LOW_LEVEL_DEFAULT 41
#define GSM_BTS_AGCH_QUEUE_HIGH_LEVEL_DEFAULT 91

#define LOGPLCHAN(lchan, ss, lvl, fmt, args...) LOGP(ss, lvl, "%s " fmt, gsm_lchan_name(lchan), ## args)

struct gsm_network {
	struct llist_head bts_list;
	unsigned int num_bts;
	struct osmo_plmn_id plmn;
	struct pcu_sock_state *pcu_state;
};

enum lchan_ciph_state {
	LCHAN_CIPH_NONE,
	LCHAN_CIPH_RX_REQ,
	LCHAN_CIPH_RX_CONF,
	LCHAN_CIPH_RXTX_REQ,
	LCHAN_CIPH_RX_CONF_TX_REQ,
	LCHAN_CIPH_RXTX_CONF,
};

/* 16 is the max. number of SI2quater messages according to 3GPP TS 44.018 Table 10.5.2.33b.1:
   4-bit index is used (2#1111 = 10#15) */
#define SI2Q_MAX_NUM 16
/* length in bits (for single SI2quater message) */
#define SI2Q_MAX_LEN 160
#define SI2Q_MIN_LEN 18

/* lchans 0..3 are SDCCH in combined channel configuration,
   use 4 as magic number for BCCH hack - see osmo-bts-../oml.c:opstart_compl() */
#define CCCH_LCHAN 4

#define TRX_NR_TS	8
#define TS_MAX_LCHAN	8

#define MAX_VERSION_LENGTH 64

struct gsm_lchan;
struct osmo_rtp_socket;
struct pcu_sock_state;
struct smscb_msg;

#define MAX_A5_KEY_LEN	(128/8)
#define RSL_ENC_ALG_A5(x)	(x+1)

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

#define MAX_NUM_UL_MEAS	104
#define LC_UL_M_F_L1_VALID	(1 << 0)
#define LC_UL_M_F_RES_VALID	(1 << 1)
#define LC_UL_M_F_OSMO_EXT_VALID (1 << 2)

struct bts_ul_meas {
	/* BER in units of 0.01%: 10.000 == 100% ber, 0 == 0% ber */
	uint16_t ber10k;
	/* timing advance offset (in 1/256 bits) */
	int16_t ta_offs_256bits;
	/* C/I ratio in dB */
	float c_i;
	/* flags */
	uint8_t is_sub:1;
	/* RSSI in dBm * -1 */
	uint8_t inv_rssi;
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

/* What kind of release/activation is done? A silent one for
 * the PDCH or one triggered through RSL? */
enum lchan_rel_act_kind {
	LCHAN_REL_ACT_RSL,
	LCHAN_REL_ACT_PCU,
	LCHAN_REL_ACT_OML,
	LCHAN_REL_ACT_REACT, /* remove once auto-activation hack is removed from opstart_compl() */
};

struct gsm_rep_facch {
	struct msgb *msg;
	uint32_t fn;
};

/* MS/BS Power related measurement averaging algo */
enum gsm_power_ctrl_meas_avg_algo {
	GSM_PWR_CTRL_MEAS_AVG_ALGO_NONE			= 0x00,
	GSM_PWR_CTRL_MEAS_AVG_ALGO_UNWEIGHTED		= 0x01,
	GSM_PWR_CTRL_MEAS_AVG_ALGO_WEIGHTED		= 0x02,
	GSM_PWR_CTRL_MEAS_AVG_ALGO_MOD_MEDIAN		= 0x03,
	/* EWMA is an Osmocom specific algo */
	GSM_PWR_CTRL_MEAS_AVG_ALGO_OSMO_EWMA		= 0x04,
};

/* MS/BS Power related measurement parameters */
struct gsm_power_ctrl_meas_params {
	/* Thresholds (see 3GPP TS 45.008, section A.3.2.1) */
	uint8_t lower_thresh; /* lower (decreasing) direction */
	uint8_t upper_thresh; /* upper (increasing) direction */

	/* Threshold Comparators for lower (decreasing) direction */
	uint8_t lower_cmp_p; /* P1 for RxLev, P3 for RxQual */
	uint8_t lower_cmp_n; /* N1 for RxLev, N3 for RxQual */
	/* Threshold Comparators for upper (increasing) direction */
	uint8_t upper_cmp_p; /* P2 for RxLev, P4 for RxQual */
	uint8_t upper_cmp_n; /* N2 for RxLev, N4 for RxQual */

	/* Hreqave and Hreqt (see 3GPP TS 45.008, Annex A) */
	uint8_t h_reqave;
	uint8_t h_reqt;

	/* AVG algorithm and its specific parameters */
	enum gsm_power_ctrl_meas_avg_algo algo;
	union {
		/* Exponentially Weighted Moving Average */
		struct {
			/* Smoothing factor: higher the value - less smoothing */
			uint8_t alpha; /* 1 .. 99 (in %) */
		} ewma;
	};
};

/* MS/BS Power Control parameters */
struct gsm_power_ctrl_params {
	/* Minimum interval between power level changes */
	uint8_t ctrl_interval; /* 1 step is 2 SACCH periods */

	/* Power change step size (maximum) */
	uint8_t inc_step_size_db; /* increasing direction */
	uint8_t red_step_size_db; /* reducing direction */

	/* Measurement averaging parameters for RxLev & RxQual */
	struct gsm_power_ctrl_meas_params rxqual_meas;
	struct gsm_power_ctrl_meas_params rxlev_meas;
};

/* Default MS/BS Power Control parameters */
extern const struct gsm_power_ctrl_params power_ctrl_params_def;

/* Measurement pre-processing state */
struct gsm_power_ctrl_meas_proc_state {
	/* Number of measurements processed */
	unsigned int meas_num;
	/* Algorithm specific data */
	union {
		struct {
			/* Scaled up 100 times average value */
			int Avg100;
		} ewma;
	};
};

struct lchan_power_ctrl_state {
	/* Dynamic Power Control parameters (NULL in static mode) */
	const struct gsm_power_ctrl_params *dpc_params;
	/* Measurement pre-processing state (for dynamic mode) */
	struct gsm_power_ctrl_meas_proc_state rxlev_meas_proc;
	/* Number of SACCH blocks to skip (for dynamic mode) */
	int skip_block_num;

	/* Depending on the context (MS or BS power control), fields 'current' and 'max'
	 * reflect either the MS power level (magic numbers), or BS Power reduction level
	 * (attenuation, in dB). */
	uint8_t current;
	uint8_t max;
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
	/* Encryption information */
	struct {
		uint8_t alg_id;
		uint8_t key_len;
		uint8_t key[MAX_A5_KEY_LEN];
	} encr;

	struct {
		uint32_t bound_ip;
		uint32_t connect_ip;
		uint16_t bound_port;
		uint16_t connect_port;
		uint16_t conn_id;
		uint8_t rtp_payload;
		uint8_t rtp_payload2;
		uint8_t speech_mode;
		struct osmo_rtp_socket *rtp_socket;
	} abis_ip;

	uint8_t rqd_ta;

	char *name;

	/* For handover, activation is described in 3GPP TS 48.058 4.1.3 and 4.1.4:
	 *
	 *          |          | Access ||  transmit         |  activate
	 *          | MS Power | Delay  ||  on main channel  |  dl SACCH
	 * ----------------------------------------------------------------------
	 * async ho   no         *     -->  yes                 no
	 * async ho   yes        *     -->  yes                 may be started
	 * sync ho    no         no    -->  yes                 no
	 * sync ho    yes        no    -->  yes                 may be started
	 * sync ho    yes        yes   -->  yes                 shall be started
	 *
	 * Always start the main channel immediately.
	 * want_dl_sacch_active indicates whether dl SACCH should be activated on CHAN ACT.
	 */
	bool want_dl_sacch_active;

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
		/* bitmask of all SI that do not mirror the BTS-global SI values */
		uint32_t overridden;
		uint32_t last;
		/* buffers where we put the pre-computed SI:
		   SI2Q_MAX_NUM is the max number of SI2quater messages (see 3GPP TS 44.018) */
		sysinfo_buf_t buf[_MAX_SYSINFO_TYPE][SI2Q_MAX_NUM];
	} si;
	struct {
		uint8_t flags;
		/* RSL measurement result number, 0 at lchan_act */
		uint8_t res_nr;
		/* number of measurements stored in array below */
		uint8_t num_ul_meas;
		struct bts_ul_meas uplink[MAX_NUM_UL_MEAS];
		/* last L1 header from the MS */
	        struct rsl_l1_info l1_info;
		struct gsm_meas_rep_unidir ul_res;
		int16_t ms_toa256;
		/* Frame number of the last measurement indication receceived */
		uint32_t last_fn;
		/* Osmocom extended measurement results, see LC_UL_M_F_EXTD_VALID */
		struct {
			/* minimum value of toa256 during measurement period */
			int16_t toa256_min;
			/* maximum value of toa256 during measurement period */
			int16_t toa256_max;
			/* standard deviation of toa256 value during measurement period */
			uint16_t toa256_std_dev;
		} ext;
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
                        /* last UL SPEECH resume flag */
                        bool is_speech_resume;
		} dtx;
		uint8_t last_cmr;
		uint32_t last_fn;

		/* SLOT #0 and #1 to store FACCH for repetition */
		struct gsm_rep_facch rep_facch[2];

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
	enum lchan_rel_act_kind rel_act_kind;
	/* RTP header Marker bit to indicate beginning of speech after pause  */
	bool rtp_tx_marker;

	/* MS/BS power control state */
	struct lchan_power_ctrl_state ms_power_ctrl;
	struct lchan_power_ctrl_state bs_power_ctrl;

	/* MS/BS Dynamic Power Control parameters */
	struct gsm_power_ctrl_params ms_dpc_params;
	struct gsm_power_ctrl_params bs_dpc_params;

	struct msgb *pending_rel_ind_msg;

	/* ECU (Error Concealment Unit) state */
	struct osmo_ecu_state *ecu_state;

	struct abis_rsl_osmo_rep_acch_cap repeated_acch_capability;
	bool repeated_dl_facch_active;
	bool repeated_ul_sacch_active;
	bool repeated_dl_sacch_active;

	/* Message buffer to store DL-SACCH repeation candidate */
	struct msgb *rep_sacch;
};

extern const struct value_string lchan_ciph_state_names[];
static inline const char *lchan_ciph_state_name(uint8_t state) {
	return get_value_string(lchan_ciph_state_names, state);
}

enum gsm_bts_trx_ts_flags {
	TS_F_PDCH_ACTIVE =		0x1000,
	TS_F_PDCH_ACT_PENDING =		0x2000,
	TS_F_PDCH_DEACT_PENDING =	0x4000,
	TS_F_PDCH_PENDING_MASK =	0x6000 /*<
			TS_F_PDCH_ACT_PENDING | TS_F_PDCH_DEACT_PENDING */
};

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
	int tsc;		/* -1 == use BTS TSC */

	/* Frequency hopping parameters (configured via OML) */
	struct {
		bool enabled;
		uint8_t maio;
		uint8_t hsn;
		uint16_t arfcn_list[64];
		uint8_t arfcn_num;
	} hopping;

	/* Transceiver "cache" for frequency hopping */
	const struct gsm_bts_trx *fh_trx_list[64];

	/* Implementation specific structure(s) */
	void *priv;

	struct gsm_lchan lchan[TS_MAX_LCHAN];
};

#define GSM_LCHAN_SI(lchan, i) (void *)((lchan)->si.buf[i][0])

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

/* The amount of time within which a sudden disconnect of a newly established
 * OML connection will cause a special warning to be logged. */
#define OSMO_BTS_OML_CONN_EARLY_DISCONNECT 10	 /* in seconds */


extern const struct value_string gsm_pchant_names[13];
extern const struct value_string gsm_pchant_descs[13];
const char *gsm_pchan_name(enum gsm_phys_chan_config c);
enum gsm_phys_chan_config gsm_pchan_parse(const char *name);
const char *gsm_lchant_name(enum gsm_chan_t c);
char *gsm_ts_name(const struct gsm_bts_trx_ts *ts);
char *gsm_ts_and_pchan_name(const struct gsm_bts_trx_ts *ts);
char *gsm_lchan_name_compute(const struct gsm_lchan *lchan);
const char *gsm_lchans_name(enum gsm_lchan_state s);

static inline char *gsm_lchan_name(const struct gsm_lchan *lchan)
{
	return lchan->name;
}

uint8_t gsm_lchan2chan_nr(const struct gsm_lchan *lchan);
uint8_t gsm_lchan_as_pchan2chan_nr(const struct gsm_lchan *lchan,
				   enum gsm_phys_chan_config as_pchan);

#define BSIC2BCC(bsic) ((bsic) & 0x07)

uint8_t gsm_ts_tsc(const struct gsm_bts_trx_ts *ts);

struct gsm_lchan *rsl_lchan_lookup(struct gsm_bts_trx *trx, uint8_t chan_nr,
				   int *rc);

enum gsm_phys_chan_config ts_pchan(const struct gsm_bts_trx_ts *ts);
uint8_t ts_subslots(const struct gsm_bts_trx_ts *ts);
bool ts_is_tch(const struct gsm_bts_trx_ts *ts);

int lchan2ecu_codec(const struct gsm_lchan *lchan);

void lchan_set_state(struct gsm_lchan *lchan, enum gsm_lchan_state state);
int conf_lchans_as_pchan(struct gsm_bts_trx_ts *ts,
			 enum gsm_phys_chan_config pchan);

/* cipher code */
#define CIPHER_A5(x) (1 << (x-1))

bool ts_is_pdch(const struct gsm_bts_trx_ts *ts);

#endif /* _GSM_DATA_H */
