#ifndef _RSL_H
#define _RSL_H

#define LCHAN_FN_DUMMY 0xFFFFFFFF
#define LCHAN_FN_WAIT 0xFFFFFFFE

int down_rsl(struct gsm_bts_trx *trx, struct msgb *msg);
int rsl_tx_rf_res(struct gsm_bts_trx *trx);
int rsl_tx_chan_rqd(struct gsm_bts_trx *trx, struct gsm_time *gtime,
		    uint8_t ra, uint8_t acc_delay);
int rsl_tx_est_ind(struct gsm_lchan *lchan, uint8_t link_id, uint8_t *data, int len);

int rsl_tx_chan_act_acknack(struct gsm_lchan *lchan, uint8_t cause);
int rsl_tx_conn_fail(const struct gsm_lchan *lchan, uint8_t cause);
int rsl_tx_rf_rel_ack(struct gsm_lchan *lchan);
int rsl_tx_hando_det(struct gsm_lchan *lchan, uint8_t *ho_delay);

/* call-back for LAPDm code, called when it wants to send msgs UP */
int lapdm_rll_tx_cb(struct msgb *msg, struct lapdm_entity *le, void *ctx);

int rsl_tx_ipac_dlcx_ind(struct gsm_lchan *lchan, uint8_t cause);
int rsl_tx_ccch_load_ind_pch(struct gsm_bts *bts, uint16_t paging_avail);
int rsl_tx_ccch_load_ind_rach(struct gsm_bts *bts, uint16_t total,
			      uint16_t busy, uint16_t access);
int rsl_tx_delete_ind(struct gsm_bts *bts, const uint8_t *ia, uint8_t ia_len);

void cb_ts_disconnected(struct gsm_bts_trx_ts *ts);
void cb_ts_connected(struct gsm_bts_trx_ts *ts, int rc);
void ipacc_dyn_pdch_complete(struct gsm_bts_trx_ts *ts, int rc);

int rsl_tx_cbch_load_indication(struct gsm_bts *bts, bool ext_cbch, bool overflow, uint8_t amount);

int rsl_tx_meas_res(struct gsm_lchan *lchan, const uint8_t *l3, int l3_len, int timing_offset);

#endif // _RSL_H */
