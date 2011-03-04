#ifndef _RSL_H
#define _RSL_H

int down_rsl(struct osmobts_trx *trx, struct msgb *msg);
int rsl_tx_rf_res(struct osmobts_trx *trx);
int rsl_tx_chan_rqd(struct osmobts_trx *trx);
int rsl_tx_est_ind(struct osmobts_lchan *lchan, uint8_t link_id, uint8_t *data, int len);
int rsl_tx_rll(struct msgb *msg, struct osmol2_entity *l2_entity);
int rsl_tx_ipac_dlcx_ind(struct osmobts_lchan *lchan, uint8_t cause);

#endif // _RSL_H */

