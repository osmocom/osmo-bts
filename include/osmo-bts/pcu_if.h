#ifndef _PCU_IF_H
#define _PCU_IF_H

int pcu_tx_info_ind(void);
int pcu_tx_rts_req(struct gsm_bts_trx_ts *ts, uint8_t is_ptcch, uint32_t fn,
	uint16_t arfcn, uint8_t block_nr);
int pcu_tx_data_ind(struct gsm_bts_trx_ts *ts, uint8_t is_ptcch, uint32_t fn,
	uint16_t arfcn, uint8_t block_nr, uint8_t *data, uint8_t len);
int pcu_tx_rach_ind(struct gsm_bts *bts, int16_t qta, uint8_t ra, uint32_t fn);
int pcu_tx_time_ind(uint32_t fn);

int pcu_sock_init(void);
void pcu_sock_exit(void);

#endif /* _PCU_IF_H */
