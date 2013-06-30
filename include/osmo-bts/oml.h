#ifndef _OML_H
#define _OML_H

int oml_init(void);
int down_oml(struct gsm_bts *bts, struct msgb *msg);

struct msgb *oml_msgb_alloc(void);
int oml_send_msg(struct msgb *msg, int is_mauf);
int oml_mo_send_msg(struct gsm_abis_mo *mo, struct msgb *msg, uint8_t msg_type);
int oml_mo_opstart_ack(struct gsm_abis_mo *mo);
int oml_mo_opstart_nack(struct gsm_abis_mo *mo, uint8_t nack_cause);
int oml_mo_statechg_ack(struct gsm_abis_mo *mo);

/* Change the state and send STATE CHG REP */
int oml_mo_state_chg(struct gsm_abis_mo *mo, int op_state, int avail_state);

/* First initialization of MO, does _not_ generate state changes */
int oml_mo_state_init(struct gsm_abis_mo *mo, int op_state, int avail_state);

/* Transmit STATE CHG REP even if there was no state change */
int oml_tx_state_changed(struct gsm_abis_mo *mo);

int oml_mo_tx_sw_act_rep(struct gsm_abis_mo *mo);

int oml_fom_ack_nack(struct msgb *old_msg, uint8_t cause);

int oml_mo_fom_ack_nack(struct gsm_abis_mo *mo, uint8_t orig_msg_type,
			uint8_t cause);

#endif // _OML_H */
