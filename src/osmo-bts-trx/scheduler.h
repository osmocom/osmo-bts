#ifndef TRX_SCHEDULER_H
#define TRX_SCHEDULER_H

/*! \brief how many frame numbers in advance we should send bursts to PHY */
extern uint32_t trx_clock_advance;
/*! \brief advance RTS.ind to L2 by that many clocks */
extern uint32_t trx_rts_advance;
/*! \brief last frame number as received from PHY */
extern uint32_t transceiver_last_fn;


/*! \brief Initialize the scheudler data structures */
int trx_sched_init(struct trx_l1h *l1h);

/*! \brief De-initialize the scheudler data structures */
void trx_sched_exit(struct trx_l1h *l1h);

/*! \brief Handle a PH-DATA.req from L2 down to L1 */
int trx_sched_ph_data_req(struct trx_l1h *l1h, struct osmo_phsap_prim *l1sap);

/*! \brief Handle a PH-TCH.req from L2 down to L1 */
int trx_sched_tch_req(struct trx_l1h *l1h, struct osmo_phsap_prim *l1sap);

/*! \brief PHY informs us of new (current) GSM freme nunmber */
int trx_sched_clock(uint32_t fn);

/*! \brief handle an UL burst received by PHY */
int trx_sched_ul_burst(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
        sbit_t *bits, int8_t rssi, float toa);

/*! \brief set multiframe scheduler to given physical channel config */
int trx_sched_set_pchan(struct trx_l1h *l1h, uint8_t tn,
        enum gsm_phys_chan_config pchan);

/*! \brief set all matching logical channels active/inactive */
int trx_sched_set_lchan(struct trx_l1h *l1h, uint8_t chan_nr, uint8_t link_id,
	int active);

/*! \brief set mode of all matching logical channels to given mode(s) */
int trx_sched_set_mode(struct trx_l1h *l1h, uint8_t chan_nr, uint8_t rsl_cmode,
	uint8_t tch_mode, int codecs, uint8_t codec0, uint8_t codec1,
	uint8_t codec2, uint8_t codec3, uint8_t initial_codec,
	uint8_t handover);

/*! \brief set ciphering on given logical channels */
int trx_sched_set_cipher(struct trx_l1h *l1h, uint8_t chan_nr, int downlink,
        int algo, uint8_t *key, int key_len);

/* \brief close all logical channels and reset timeslots */
void trx_sched_reset(struct trx_l1h *l1h);

#endif /* TRX_SCHEDULER_H */
