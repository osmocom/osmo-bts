#ifndef TRX_SCHEDULER_H
#define TRX_SCHEDULER_H

#include <osmo-bts/gsm_data.h>

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
	TRXC_PDTCH,
	TRXC_PTCCH,
	_TRX_CHAN_MAX
};

/* States each channel on a multiframe */
struct l1sched_chan_state {
	/* scheduler */
	uint8_t			active;		/* Channel is active */
	ubit_t			*dl_bursts;	/* burst buffer for TX */
	sbit_t			*ul_bursts;	/* burst buffer for RX */
	uint32_t		ul_first_fn;	/* fn of first burst */
	uint8_t			ul_mask;	/* mask of received bursts */

	/* RSSI / TOA */
	uint8_t			rssi_num;	/* number of RSSI values */
	float			rssi_sum;	/* sum of RSSI values */
	uint8_t			toa_num;	/* number of TOA values */
	float			toa_sum;	/* sum of TOA values */

	/* loss detection */
	uint8_t			lost;		/* (SACCH) loss detection */

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

	/* TCH/H */
	uint8_t			dl_ongoing_facch; /* FACCH/H on downlink */
	uint8_t			ul_ongoing_facch; /* FACCH/H on uplink */

	/* encryption */
	int			ul_encr_algo;	/* A5/x encry algo downlink */
	int			dl_encr_algo;	/* A5/x encry algo uplink */
	int			ul_encr_key_len;
	int			dl_encr_key_len;
	uint8_t			ul_encr_key[MAX_A5_KEY_LEN];
	uint8_t			dl_encr_key[MAX_A5_KEY_LEN];

	/* measurements */
	struct {
		uint8_t		clock;		/* cyclic clock counter */
		int8_t		rssi[32];	/* last RSSI values */
		int		rssi_count;	/* received RSSI values */
		int		rssi_valid_count; /* number of stored value */
		int		rssi_got_burst; /* any burst received so far */
		float		toa_sum;	/* sum of TOA values */
		int		toa_num;	/* number of TOA value */
	} meas;

	/* handover */
	uint8_t			ho_rach_detect;	/* if rach detection is on */
};

struct l1sched_ts {
	uint8_t 		mf_index;	/* selected multiframe index */
	uint32_t 		mf_last_fn;	/* last received frame number */
	uint8_t			mf_period;	/* period of multiframe */
	const struct trx_sched_frame *mf_frames; /* pointer to frame layout */

	struct llist_head	dl_prims;	/* Queue primitves for TX */

	/* Channel states for all logical channels */
	struct l1sched_chan_state chan_state[_TRX_CHAN_MAX];
};

struct l1sched_trx {
	struct gsm_bts_trx	*trx;
	struct l1sched_ts       ts[TRX_NR_TS];
};

struct l1sched_ts *l1sched_trx_get_ts(struct l1sched_trx *l1t, uint8_t tn);

/*! \brief how many frame numbers in advance we should send bursts to PHY */
extern uint32_t trx_clock_advance;
/*! \brief advance RTS.ind to L2 by that many clocks */
extern uint32_t trx_rts_advance;
/*! \brief last frame number as received from PHY */
extern uint32_t transceiver_last_fn;


/*! \brief Initialize the scheduler data structures */
int trx_sched_init(struct l1sched_trx *l1t, struct gsm_bts_trx *trx);

/*! \brief De-initialize the scheduler data structures */
void trx_sched_exit(struct l1sched_trx *l1t);

/*! \brief Handle a PH-DATA.req from L2 down to L1 */
int trx_sched_ph_data_req(struct l1sched_trx *l1t, struct osmo_phsap_prim *l1sap);

/*! \brief Handle a PH-TCH.req from L2 down to L1 */
int trx_sched_tch_req(struct l1sched_trx *l1t, struct osmo_phsap_prim *l1sap);

/*! \brief PHY informs us of new (current) GSM freme nunmber */
int trx_sched_clock(struct gsm_bts *bts, uint32_t fn);

/*! \brief handle an UL burst received by PHY */
int trx_sched_ul_burst(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
        sbit_t *bits, int8_t rssi, float toa);

/*! \brief set multiframe scheduler to given physical channel config */
int trx_sched_set_pchan(struct l1sched_trx *l1t, uint8_t tn,
        enum gsm_phys_chan_config pchan);

/*! \brief set all matching logical channels active/inactive */
int trx_sched_set_lchan(struct l1sched_trx *l1t, uint8_t chan_nr, uint8_t link_id,
	int active);

/*! \brief set mode of all matching logical channels to given mode(s) */
int trx_sched_set_mode(struct l1sched_trx *l1t, uint8_t chan_nr, uint8_t rsl_cmode,
	uint8_t tch_mode, int codecs, uint8_t codec0, uint8_t codec1,
	uint8_t codec2, uint8_t codec3, uint8_t initial_codec,
	uint8_t handover);

/*! \brief set ciphering on given logical channels */
int trx_sched_set_cipher(struct l1sched_trx *l1t, uint8_t chan_nr, int downlink,
        int algo, uint8_t *key, int key_len);

/* \brief close all logical channels and reset timeslots */
void trx_sched_reset(struct l1sched_trx *l1t);

#endif /* TRX_SCHEDULER_H */
