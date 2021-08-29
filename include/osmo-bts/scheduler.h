#pragma once

#include <osmocom/core/utils.h>
#include <osmocom/core/rate_ctr.h>

#include <osmo-bts/gsm_data.h>

/* Whether a logical channel must be activated automatically */
#define TRX_CHAN_FLAG_AUTO_ACTIVE	(1 << 0)

/* FIXME: we should actually activate 'auto-active' channels */
#define TRX_CHAN_IS_ACTIVE(state, chan) \
	(trx_chan_desc[chan].flags & TRX_CHAN_FLAG_AUTO_ACTIVE || (state)->active)

#define TRX_GMSK_NB_TSC(br) \
	_sched_train_seq_gmsk_nb[(br)->tsc_set][(br)->tsc]

#define TRX_8PSK_NB_TSC(br) \
	_sched_train_seq_8psk_nb[(br)->tsc]

#define TRX_CHAN_IS_DEDIC(chan) \
	(chan >= TRXC_TCHF)

/* These types define the different channels on a multiframe.
 * Each channel has queues and can be activated individually.
 */
enum trx_chan_type {
	TRXC_IDLE = 0,
	TRXC_FCCH,
	TRXC_SCH,
	TRXC_BCCH,
	TRXC_RACH,
	TRXC_CCCH,
	TRXC_CBCH,
	TRXC_PDTCH,
	TRXC_PTCCH,
/* Dedicated channels start here */
	TRXC_TCHF,
	TRXC_TCHH_0,
	TRXC_TCHH_1,
	TRXC_SDCCH4_0,
	TRXC_SDCCH4_1,
	TRXC_SDCCH4_2,
	TRXC_SDCCH4_3,
	TRXC_SDCCH8_0,
	TRXC_SDCCH8_1,
	TRXC_SDCCH8_2,
	TRXC_SDCCH8_3,
	TRXC_SDCCH8_4,
	TRXC_SDCCH8_5,
	TRXC_SDCCH8_6,
	TRXC_SDCCH8_7,
	TRXC_SACCHTF,
	TRXC_SACCHTH_0,
	TRXC_SACCHTH_1,
	TRXC_SACCH4_0,
	TRXC_SACCH4_1,
	TRXC_SACCH4_2,
	TRXC_SACCH4_3,
	TRXC_SACCH8_0,
	TRXC_SACCH8_1,
	TRXC_SACCH8_2,
	TRXC_SACCH8_3,
	TRXC_SACCH8_4,
	TRXC_SACCH8_5,
	TRXC_SACCH8_6,
	TRXC_SACCH8_7,
	_TRX_CHAN_MAX
};

#define GSM_BURST_LEN		148
#define GPRS_BURST_LEN		GSM_BURST_LEN
#define EGPRS_BURST_LEN		444

enum trx_mod_type {
	TRX_MOD_T_GMSK,
	TRX_MOD_T_8PSK,
	TRX_MOD_T_AQPSK,
};

/* A set of measurements belonging to one Uplink burst */
struct l1sched_meas_set {
	int16_t			toa256;		/* Timing of Arrival (1/256 of a symbol) */
	int16_t			ci_cb;		/* Carrier-to-Interference (cB) */
	float			rssi;		/* RSSI (dBm) */
};

/* States each channel on a multiframe */
struct l1sched_chan_state {
	/* Pointer to the associated logical channel state from gsm_data_shared.
	 * Initialized during channel activation, thus may be NULL for inactive
	 * or auto-active channels. Always check before dereferencing! */
	struct gsm_lchan	*lchan;

	/* scheduler */
	bool			active;		/* Channel is active */
	ubit_t			*dl_bursts;	/* burst buffer for TX */
	enum trx_mod_type	dl_mod_type;	/* Downlink modulation type */
	sbit_t			*ul_bursts;	/* burst buffer for RX */
	sbit_t			*ul_bursts_prev;/* previous burst buffer for RX (repeated SACCH) */
	uint32_t		ul_first_fn;	/* fn of first burst */
	uint8_t			ul_mask;	/* mask of received bursts */

	/* loss detection */
	uint8_t			lost_frames;	/* how many L2 frames were lost */
	uint32_t		last_tdma_fn;	/* last processed TDMA frame number */
	uint32_t		proc_tdma_fs;	/* how many TDMA frames were processed */
	uint32_t		lost_tdma_fs;	/* how many TDMA frames were lost */

	/* mode */
	uint8_t			rsl_cmode, tch_mode; /* mode for TCH channels */

	/* AMR */
	uint8_t			codec[4];	/* 4 possible codecs for amr */
	int			codecs;		/* number of possible codecs */
	float			ber_sum;	/* sum of bit error rates */
	int			ber_num;	/* number of bit error rates */
	uint8_t			ul_ft;		/* current uplink FT index */
	uint8_t			dl_ft;		/* current downlink FT index */
	uint8_t			ul_cmr;		/* current uplink CMR index */
	uint8_t			dl_cmr;		/* current downlink CMR index */
	uint8_t			amr_loop;	/* if AMR loop is enabled */
	uint8_t			amr_last_dtx;	/* last received dtx frame type */

	/* TCH/H */
	uint8_t			dl_ongoing_facch; /* FACCH/H on downlink */
	uint8_t			ul_ongoing_facch; /* FACCH/H on uplink */
	struct l1sched_meas_set meas_avg_facch;   /* measurement results for last FACCH */
	uint16_t		ber10k_facch;	  /* bit error rate for last FACCH */

	uint8_t			dl_facch_bursts;  /* number of remaining DL FACCH bursts */

	/* encryption */
	int			ul_encr_algo;	/* A5/x encry algo downlink */
	int			dl_encr_algo;	/* A5/x encry algo uplink */
	int			ul_encr_key_len;
	int			dl_encr_key_len;
	uint8_t			ul_encr_key[MAX_A5_KEY_LEN];
	uint8_t			dl_encr_key[MAX_A5_KEY_LEN];

	/* Uplink measurements */
	struct {
		/* Active channel measurements (simple ring buffer) */
		struct l1sched_meas_set buf[8]; /* up to 8 entries */
		unsigned int current; /* current position */

		/* Interference measurements */
		int interf_avg; /* sliding average */
	} meas;

	/* handover */
	bool			ho_rach_detect;	/* if rach detection is on */
};

struct l1sched_ts {
	struct gsm_bts_trx_ts	*ts;		/* timeslot we belong to */

	uint8_t 		mf_index;	/* selected multiframe index */
	uint8_t			mf_period;	/* period of multiframe */
	const struct trx_sched_frame *mf_frames; /* pointer to frame layout */

	struct llist_head	dl_prims;	/* Queue primitives for TX */

	struct rate_ctr_group	*ctrs;		/* rate counters */

	/* Channel states for all logical channels */
	struct l1sched_chan_state chan_state[_TRX_CHAN_MAX];
};


/*! \brief Initialize the scheduler data structures */
void trx_sched_init(struct gsm_bts_trx *trx);

/*! \brief De-initialize the scheduler data structures */
void trx_sched_clean(struct gsm_bts_trx *trx);

/*! \brief Handle a PH-DATA.req from L2 down to L1 */
int trx_sched_ph_data_req(struct gsm_bts_trx *trx, struct osmo_phsap_prim *l1sap);

/*! \brief Handle a PH-TCH.req from L2 down to L1 */
int trx_sched_tch_req(struct gsm_bts_trx *trx, struct osmo_phsap_prim *l1sap);

/*! \brief PHY informs us of new (current) GSM frame number */
int trx_sched_clock(struct gsm_bts *bts, uint32_t fn);

/*! \brief PHY informs us clock indications should start to be received */
int trx_sched_clock_started(struct gsm_bts *bts);

/*! \brief PHY informs us no more clock indications should be received anymore */
int trx_sched_clock_stopped(struct gsm_bts *bts);

/*! \brief set multiframe scheduler to given physical channel config */
int trx_sched_set_pchan(struct gsm_bts_trx_ts *ts, enum gsm_phys_chan_config pchan);

/*! \brief set all matching logical channels active/inactive */
int trx_sched_set_lchan(struct gsm_lchan *lchan, uint8_t chan_nr, uint8_t link_id, bool active);

/*! \brief set mode of all matching logical channels to given mode(s) */
int trx_sched_set_mode(struct gsm_bts_trx_ts *ts, uint8_t chan_nr, uint8_t rsl_cmode,
	uint8_t tch_mode, int codecs, uint8_t codec0, uint8_t codec1,
	uint8_t codec2, uint8_t codec3, uint8_t initial_codec,
	uint8_t handover);

/*! \brief set ciphering on given logical channels */
int trx_sched_set_cipher(struct gsm_lchan *lchan, uint8_t chan_nr, bool downlink);

/* frame structures */
struct trx_sched_frame {
	/*! \brief downlink TRX channel type */
	enum trx_chan_type		dl_chan;
	/*! \brief downlink block ID */
	uint8_t				dl_bid;
	/*! \brief uplink TRX channel type */
	enum trx_chan_type		ul_chan;
	/*! \brief uplink block ID */
	uint8_t				ul_bid;
};

/* multiframe structure */
struct trx_sched_multiframe {
	/*! \brief physical channel config (channel combination) */
	enum gsm_phys_chan_config	pchan;
	/*! \brief applies to which timeslots? */
	uint8_t				slotmask;
	/*! \brief repeats how many frames */
	uint8_t				period;
	/*! \brief pointer to scheduling structure */
	const struct trx_sched_frame	*frames;
	/*! \brief human-readable name */
	const char 			*name;
};

int find_sched_mframe_idx(enum gsm_phys_chan_config pchan, uint8_t tn);

/*! Determine if given frame number contains SACCH (true) or other (false) burst */
bool trx_sched_is_sacch_fn(struct gsm_bts_trx_ts *ts, uint32_t fn, bool uplink);
extern const struct trx_sched_multiframe trx_sched_multiframes[];

#define TRX_BI_F_NOPE_IND	(1 << 0)
#define TRX_BI_F_MOD_TYPE	(1 << 1)
#define TRX_BI_F_TS_INFO	(1 << 2)
#define TRX_BI_F_CI_CB		(1 << 3)
#define TRX_BI_F_TRX_NUM	(1 << 4)
#define TRX_BI_F_BATCH_IND	(1 << 5)
#define TRX_BI_F_SHADOW_IND	(1 << 6)

/*! UL burst indication with the corresponding meta info */
struct trx_ul_burst_ind {
	/* Field presence bitmask (see TRX_BI_F_*) */
	uint8_t flags;

	/* Mandatory fields */
	uint32_t fn;		/*!< TDMA frame number */
	uint8_t tn;		/*!< TDMA time-slot number */
	int16_t toa256;		/*!< Timing of Arrival in units of 1/256 of symbol */
	int8_t rssi;		/*!< Received Signal Strength Indication */

	/* Optional fields (defined by flags) */
	enum trx_mod_type mod;	/*!< Modulation type */
	uint8_t tsc_set;	/*!< Training Sequence Set */
	uint8_t tsc;		/*!< Training Sequence Code */
	int16_t ci_cb;		/*!< Carrier-to-Interference ratio (in centiBels) */
	uint8_t trx_num;	/*!< TRX (RF channel) number */

	/* Used internally by the PDU parser */
	uint8_t _num_pdus;	/*!< Number of processed PDUs */

	/* Internally used by the scheduler */
	enum trx_chan_type chan;
	uint8_t bid;

	/*! Burst soft-bits buffer */
	sbit_t burst[EGPRS_BURST_LEN];
	size_t burst_len;
};

#define TRX_BR_F_FACCH		(1 << 0)

/*! DL burst request with the corresponding meta info */
struct trx_dl_burst_req {
	uint8_t flags;		/*!< see TRX_BR_F_* */

	/* Mandatory fields */
	uint32_t fn;		/*!< TDMA frame number */
	uint8_t tn;		/*!< TDMA timeslot number */
	uint8_t att;		/*!< Tx power attenuation */
	int8_t scpir;		/*!< SCPIR (for AQPSK only) */
	uint8_t trx_num;	/*!< TRX (RF channel) number */

	enum trx_mod_type mod;	/*!< Modulation type */
	uint8_t tsc_set;	/*!< Training Sequence Set */
	uint8_t tsc;		/*!< Training Sequence Code */

	/* Internally used by the scheduler */
	enum trx_chan_type chan;
	uint8_t bid;

	/*! Burst hard-bits buffer */
	ubit_t burst[EGPRS_BURST_LEN];
	size_t burst_len;
};

/*! Handle an UL burst received by PHY */
int trx_sched_route_burst_ind(const struct gsm_bts_trx *trx, struct trx_ul_burst_ind *bi);
int trx_sched_ul_burst(struct l1sched_ts *l1ts, struct trx_ul_burst_ind *bi);

/* Averaging mode for trx_sched_meas_avg() */
enum sched_meas_avg_mode {
	/* last 4 bursts (default for xCCH, TCH/H, PTCCH and PDTCH) */
	SCHED_MEAS_AVG_M_QUAD,
	/* last 8 bursts (default for TCH/F and FACCH/F) */
	SCHED_MEAS_AVG_M_OCTO,
	/* last 6 bursts (default for FACCH/H) */
	SCHED_MEAS_AVG_M_SIX,
	/* first 4 of last 8 bursts */
	SCHED_MEAS_AVG_M8_FIRST_QUAD,
	/* first 2 of last 6 bursts */
	SCHED_MEAS_AVG_M6_FIRST_TWO,
	/* middle 2 of last 6 bursts */
	SCHED_MEAS_AVG_M6_MIDDLE_TWO,
};

void trx_sched_meas_push(struct l1sched_chan_state *chan_state,
			 const struct trx_ul_burst_ind *bi);
void trx_sched_meas_avg(const struct l1sched_chan_state *chan_state,
			struct l1sched_meas_set *avg,
			enum sched_meas_avg_mode mode);
