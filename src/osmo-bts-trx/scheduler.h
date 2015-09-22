#ifndef TRX_SCHEDULER_H
#define TRX_SCHEDULER_H

extern uint32_t trx_clock_advance;
extern uint32_t trx_rts_advance;
extern uint32_t transceiver_last_fn;


int trx_sched_init(struct trx_l1h *l1h);

void trx_sched_exit(struct trx_l1h *l1h);

int trx_sched_ph_data_req(struct trx_l1h *l1h, struct osmo_phsap_prim *l1sap);

int trx_sched_tch_req(struct trx_l1h *l1h, struct osmo_phsap_prim *l1sap);

int trx_sched_clock(uint32_t fn);

int trx_sched_ul_burst(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
        sbit_t *bits, int8_t rssi, float toa);

/* set multiframe scheduler to given pchan */
int trx_sched_set_pchan(struct trx_l1h *l1h, uint8_t tn,
        enum gsm_phys_chan_config pchan);

/* setting all logical channels given attributes to active/inactive */
int trx_sched_set_lchan(struct trx_l1h *l1h, uint8_t chan_nr, uint8_t link_id,
	int active);

/* setting all logical channels given attributes to active/inactive */
int trx_sched_set_mode(struct trx_l1h *l1h, uint8_t chan_nr, uint8_t rsl_cmode,
	uint8_t tch_mode, int codecs, uint8_t codec0, uint8_t codec1,
	uint8_t codec2, uint8_t codec3, uint8_t initial_codec,
	uint8_t handover);

/* setting cipher on logical channels */
int trx_sched_set_cipher(struct trx_l1h *l1h, uint8_t chan_nr, int downlink,
        int algo, uint8_t *key, int key_len);

/* close all logical channels and reset timeslots */
void trx_sched_reset(struct trx_l1h *l1h);

#endif /* TRX_SCHEDULER_H */
