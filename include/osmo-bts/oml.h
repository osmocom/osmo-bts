#ifndef _OML_H
#define _OML_H

int down_oml(struct osmocom_bts *bts, struct msgb *msg);
int oml_tx_sw_act_rep(struct ipabis_link *link, uint8_t obj_class, uint8_t bts_nr, uint8_t trx_nr, uint8_t ts_nr);
int oml_tx_state_changed(struct ipabis_link *link, uint8_t op_state, uint8_t avail_status, uint8_t obj_class, uint8_t bts_nr, uint8_t trx_nr, uint8_t ts_nr);

#endif // _OML_H */

