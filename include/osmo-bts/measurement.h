#ifndef OSMO_BTS_MEAS_H
#define OSMO_BTS_MEAS_H

int lchan_new_ul_meas(struct gsm_lchan *lchan, struct bts_ul_meas *ulm);

int trx_meas_check_compute(struct gsm_bts_trx *trx, uint32_t fn);

#endif
