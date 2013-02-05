#ifndef TRX_SCHEDULER_H
#define TRX_SCHEDULER_H

extern uint32_t trx_clock_advance;
extern uint32_t tranceiver_last_fn;


int trx_sched_init(struct trx_l1h *l1h);

void trx_sched_exit(struct trx_l1h *l1h);

int trx_sched_ph_data_req(struct trx_l1h *l1h, struct osmo_phsap_prim *l1sap);

int trx_sched_tch_req(struct trx_l1h *l1h, struct osmo_phsap_prim *l1sap);

int trx_sched_clock(uint32_t fn);

int trx_sched_ul_burst(struct trx_l1h *l1h, uint8_t tn, uint32_t fn,
        sbit_t *bits, int8_t rssi, int16_t toa);

/* set multiframe scheduler to given pchan */
int trx_sched_set_pchan(struct trx_l1h *l1h, uint8_t tn,
        enum gsm_phys_chan_config pchan);

/* setting all logical channels given attributes to active/inactive */
int trx_sched_set_lchan(struct trx_l1h *l1h, uint8_t chan_nr, uint8_t link_id,
	int downlink, int active);

#endif /* TRX_SCHEDULER_H */
