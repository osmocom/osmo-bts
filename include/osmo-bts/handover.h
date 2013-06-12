#ifndef HANDOVER_H
#define HANDOVER_H

void handover_rach(struct gsm_bts_trx *trx, uint8_t chan_nr, 
	struct gsm_lchan *lchan, uint8_t ra, uint8_t acc_delay);
void handover_frame(struct gsm_lchan *lchan);

#endif /* HANDOVER_H */
