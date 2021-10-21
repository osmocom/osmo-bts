#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <osmocom/core/timer.h>
#include <osmocom/core/linuxlist.h>
#include <osmocom/core/logging.h>
#include <osmocom/gsm/gsm_utils.h>
#include <osmocom/codec/ecu.h>
#include <osmocom/gsm/lapdm.h>
#include <osmocom/gsm/sysinfo.h>
#include <osmocom/gsm/protocol/gsm_08_58.h>
#include <osmocom/gsm/gsm48_rest_octets.h>
#include <osmocom/gsm/protocol/gsm_04_08.h>
#include <osmocom/gsm/meas_rep.h>

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
	int16_t c_i;
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

	/* Temporary Overpower for SACCH/FACCH */
	uint8_t bs_acch_overpower_db;

	struct msgb *pending_rel_ind_msg;

	/* ECU (Error Concealment Unit) state */
	struct osmo_ecu_state *ecu_state;

	struct abis_rsl_osmo_rep_acch_cap rep_acch_cap;
	bool repeated_dl_facch_active;
	bool repeated_ul_sacch_active;
	bool repeated_dl_sacch_active;

	/* Message buffer to store DL-SACCH repeation candidate */
	struct msgb *rep_sacch;

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
