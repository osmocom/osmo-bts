#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <netinet/in.h>

#include <osmocom/core/bits.h>
#include <osmocom/core/timer.h>
#include <osmocom/core/linuxlist.h>
#include <osmocom/core/logging.h>
#include <osmocom/gsm/gsm_utils.h>
#include <osmocom/codec/codec.h>
#include <osmocom/codec/ecu.h>
#include <osmocom/gsm/lapdm.h>
#include <osmocom/gsm/sysinfo.h>
#include <osmocom/gsm/protocol/gsm_08_58.h>
#include <osmocom/gsm/gsm48_rest_octets.h>
#include <osmocom/gsm/protocol/gsm_04_08.h>
#include <osmocom/gsm/meas_rep.h>
#include <osmocom/netif/osmux.h>

#include <osmo-bts/power_control.h>

#define LOGPLCHAN(lchan, ss, lvl, fmt, args...) LOGP(ss, lvl, "%s " fmt, gsm_lchan_name(lchan), ## args)

enum lchan_ciph_state {
	LCHAN_CIPH_NONE,
	LCHAN_CIPH_RX_REQ,
	LCHAN_CIPH_RX_CONF,
	LCHAN_CIPH_RXTX_REQ,
	LCHAN_CIPH_RX_CONF_TX_REQ,
	LCHAN_CIPH_RXTX_CONF,
};

/* state of a logical channel */
enum gsm_lchan_state {
	LCHAN_S_NONE,		/* channel is not active */
	LCHAN_S_ACT_REQ,	/* channel activation requested */
	LCHAN_S_ACTIVE,		/* channel is active and operational */
	LCHAN_S_REL_REQ,	/* channel release has been requested */
	LCHAN_S_REL_ERR,	/* channel is in an error state */
	LCHAN_S_BROKEN,		/* channel is somehow unusable */
};

#define MAX_NUM_UL_MEAS	104
#define LC_UL_M_F_L1_VALID	(1 << 0)
#define LC_UL_M_F_RES_VALID	(1 << 1)
#define LC_UL_M_F_OSMO_EXT_VALID (1 << 2)

#define MAX_A5_KEY_LEN	(128/8)
#define RSL_ENC_ALG_A5(x)	(x+1)

struct bts_ul_meas {
	/* BER in units of 0.01%: 10.000 == 100% ber, 0 == 0% ber */
	uint16_t ber10k;
	/* timing advance offset (in 1/256 bits) */
	int16_t ta_offs_256bits;
	/* C/I ratio in cB */
	int16_t ci_cb;
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
	struct amr_mode mode[4];
	uint8_t num_modes;
};

enum lchan_csd_mode {
	LCHAN_CSD_M_NT = 0,
	LCHAN_CSD_M_T_1200_75,
	LCHAN_CSD_M_T_600,
	LCHAN_CSD_M_T_1200,
	LCHAN_CSD_M_T_2400,
	LCHAN_CSD_M_T_4800,
	LCHAN_CSD_M_T_9600,
	LCHAN_CSD_M_T_14400,
	LCHAN_CSD_M_T_29000,
	LCHAN_CSD_M_T_32000,
	_LCHAN_CSD_M_NUM,
};

extern const struct value_string lchan_csd_mode_descs[];
static inline const char *lchan_csd_mode_desc(enum lchan_csd_mode mode)
{
	return get_value_string(lchan_csd_mode_descs, mode);
}

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
	LCHAN_REL_ACT_REACT, /* FIXME: remove once auto-activation hack is removed from opstart_compl() (OS#1575) */
};

struct gsm_rep_facch {
	struct msgb *msg;
	uint32_t fn;
};


struct lchan_power_ctrl_state {
	/* Dynamic Power Control parameters (NULL in static mode) */
	const struct gsm_power_ctrl_params *dpc_params;
	/* Measurement pre-processing state (for dynamic mode) */
	struct gsm_power_ctrl_meas_proc_state rxlev_meas_proc;
	struct gsm_power_ctrl_meas_proc_state rxqual_meas_proc;
	struct gsm_power_ctrl_meas_proc_state ci_meas_proc;
	/* Number of SACCH blocks to skip (for dynamic mode) */
	int skip_block_num;

	/* Depending on the context (MS or BS power control), fields 'current' and 'max'
	 * reflect either the MS power level (magic numbers), or BS Power reduction level
	 * (attenuation, in dB). */
	uint8_t current;
	uint8_t max;
};

struct lchan_ta_ctrl_state {
	/* Number of SACCH blocks to skip */
	int skip_block_num;
	/* Currently requested TA */
	uint8_t current;
};

struct gsm_lchan {
	/* The TS that we're part of */
	struct gsm_bts_trx_ts *ts;
	/* The logical subslot number in the TS */
	uint8_t nr;
	/* The logical channel type */
	enum gsm_chan_t type;
	/* RSL channel rate and type */
	enum rsl_cmod_crt rsl_chan_rt;
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
		uint8_t rtp_extensions;
		uint8_t speech_mode;
		struct {
			bool use;
			uint8_t local_cid;
			uint8_t remote_cid;
			/* Rx Osmux -> RTP, one allocated & owned per lchan */
			struct osmux_out_handle *out;
			 /* Tx RTP -> Osmux, shared by all lchans sharing a
			  * remote endp (addr+port), see "struct osmux_handle" */
			struct osmux_in_handle *in;
			/* Used to build rtp messages we send to osmux */
			struct osmo_rtp_handle *rtpst;
		} osmux;
		struct osmo_rtp_socket *rtp_socket;
	} abis_ip;

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
	/* It is required to have L3 info with DL establishment. */
	bool l3_info_estab;
	struct llist_head dl_tch_queue;
	unsigned int dl_tch_queue_len;
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
		int16_t ul_ci_cb_full;
		int16_t ul_ci_cb_sub;
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
		/* Interference levels reported by PHY (in dBm) */
		int16_t interf_meas_avg_dbm; /* Average value */
		int16_t interf_meas_dbm[31]; /* Intave max is 31 */
		uint8_t interf_meas_num;
		uint8_t interf_band;
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
		struct {
			bool last_rtp_input_was_sid;
			uint8_t last_sid[GSM_FR_BYTES];
			uint8_t last_sid_len;
			uint8_t last_sid_age;
			/* A SID was transmitted on the DL in the period
			 * beginning with the last transmitted speech frame
			 * or the last mandatory-Tx position, whichever was
			 * more recent. */
			bool dl_sid_transmitted;
			/* The current frame in the DL is taken up by FACCH */
			bool dl_facch_stealing;
			/* UL SID filter to catch DTXu half-blocks,
			 * see tch_ul_fr_hr_efr() function. */
			bool ul_sid_filter;
		} dtx_fr_hr_efr;
		uint8_t last_cmr;
		uint32_t last_fn;
		struct {
			/* RLP GSMTAP mechanism */
			uint8_t rlp_buf_ul[576/8]; /* maximum size of RLP frame */
			uint8_t rlp_buf_dl[576/8]; /* maximum size of RLP frame */
			/* alignment of RLP frames in DL for NT modes */
			ubit_t rlpdl_data_bits[60 * 7];
			uint16_t rlpdl_align_bits;
			uint8_t rlpdl_fill_level;
			ubit_t tchf48_nt_2ndhalf[120];
		} csd;
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
	struct {
		bool listener_detected;
		uint8_t talker_active;
		uint8_t ref;
		uint32_t fn;
		/* T3115: VGCS UPLINK GRANT retransmission */
		struct osmo_timer_list t3115;
		/* counts up to Ny2 */
		unsigned int vgcs_ul_grant_count;
		/* uplink free message */
		bool uplink_free;
		uint8_t uplink_free_msg[GSM_MACBLOCK_LEN];
	} asci;
	/* S counter for link loss */
	int s;
	/* Kind of the release/activation. E.g. RSL or PCU */
	enum lchan_rel_act_kind rel_act_kind;
	/* Pending RSL CHANnel ACTIVation message */
	struct msgb *pending_chan_activ;
	/* RTP header Marker bit to indicate beginning of speech after pause  */
	bool rtp_tx_marker;

	/* TA Control Loop */
	struct lchan_ta_ctrl_state ta_ctrl;

	/* MS/BS power control state */
	struct lchan_power_ctrl_state ms_power_ctrl;
	struct lchan_power_ctrl_state bs_power_ctrl;

	/* MS/BS Dynamic Power Control parameters */
	struct gsm_power_ctrl_params ms_dpc_params;
	struct gsm_power_ctrl_params bs_dpc_params;

	/* Temporary ACCH overpower capabilities and state */
	struct abis_rsl_osmo_temp_ovp_acch_cap top_acch_cap;
	bool top_acch_active;

	struct msgb *pending_rel_ind_msg;

	/* ECU (Error Concealment Unit) state */
	struct osmo_ecu_state *ecu_state;

	/* Repeated ACCH capabilities and current state */
	struct abis_rsl_osmo_rep_acch_cap rep_acch_cap;
	struct {
		bool dl_facch_active;
		bool ul_sacch_active;
		bool dl_sacch_active;

		/* Message buffers to store repeation candidates */
		struct gsm_rep_facch dl_facch[2];
		struct msgb *dl_sacch_msg;
	} rep_acch;

	/* Cached early Immediate Assignment message: if the Immediate Assignment arrives before the channel is
	 * confirmed active, then cache it here and send it once the channel is confirmed to be active. This is related
	 * to the Early IA feature, see OsmoBSC config option 'immediate-assignment pre-chan-ack'. */
	struct msgb *early_rr_ia;
	struct osmo_timer_list early_rr_ia_delay;
};

extern const struct value_string lchan_ciph_state_names[];
static inline const char *lchan_ciph_state_name(uint8_t state)
{
	return get_value_string(lchan_ciph_state_names, state);
}

#define GSM_LCHAN_SI(lchan, i) (void *)((lchan)->si.buf[i][0])

void gsm_lchan_init(struct gsm_lchan *lchan, struct gsm_bts_trx_ts *ts, unsigned int lchan_nr);
void gsm_lchan_name_update(struct gsm_lchan *lchan);
int lchan_init_lapdm(struct gsm_lchan *lchan);
void gsm_lchan_release(struct gsm_lchan *lchan, enum lchan_rel_act_kind rel_kind);
int lchan_deactivate(struct gsm_lchan *lchan);
const char *gsm_lchans_name(enum gsm_lchan_state s);

static inline char *gsm_lchan_name(const struct gsm_lchan *lchan)
{
	return lchan->name;
}

uint8_t *lchan_sacch_get(struct gsm_lchan *lchan);

uint8_t gsm_lchan2chan_nr(const struct gsm_lchan *lchan);
uint8_t gsm_lchan2chan_nr_rsl(const struct gsm_lchan *lchan);
uint8_t gsm_lchan_as_pchan2chan_nr(const struct gsm_lchan *lchan,
				   enum gsm_phys_chan_config as_pchan);

void gsm_lchan_interf_meas_push(struct gsm_lchan *lchan, int dbm);
void gsm_lchan_interf_meas_calc_avg(struct gsm_lchan *lchan);

int lchan2ecu_codec(const struct gsm_lchan *lchan);

void lchan_set_state(struct gsm_lchan *lchan, enum gsm_lchan_state state);

int lchan_rtp_socket_create(struct gsm_lchan *lchan, const char *bind_ip);
int lchan_rtp_socket_connect(struct gsm_lchan *lchan, const struct in_addr *ia, uint16_t connect_port);
void lchan_rtp_socket_free(struct gsm_lchan *lchan);

void lchan_dl_tch_queue_enqueue(struct gsm_lchan *lchan, struct msgb *msg, unsigned int limit);

static inline bool lchan_is_dcch(const struct gsm_lchan *lchan)
{
	switch (lchan->type) {
	case GSM_LCHAN_SDCCH:
	case GSM_LCHAN_TCH_F:
	case GSM_LCHAN_TCH_H:
		return true;
	default:
		return false;
	}
}

#define lchan_is_tch(lchan) \
	((lchan)->type == GSM_LCHAN_TCH_F || (lchan)->type == GSM_LCHAN_TCH_H)
