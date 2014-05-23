#ifndef _OML_H
#define _OML_H

enum oml_message_type {
	OML_MSG_TYPE_ETSI,
	OML_MSG_TYPE_IPA,
	OML_MSG_TYPE_OSMO,
};

int oml_init(void);
int down_oml(struct gsm_bts *bts, struct msgb *msg);

int oml_add_manufacturer_id_label(struct msgb *msg,
				  enum oml_message_type vendor_type);
int oml_check_manuf(struct abis_om_hdr *hdr, size_t msg_size);
int oml_check_msg(struct msgb *msg);

struct msgb *oml_msgb_alloc(void);
int oml_send_msg(struct msgb *msg, int is_mauf);
int oml_mo_send_msg(struct gsm_abis_mo *mo, struct msgb *msg, uint8_t msg_type);
int oml_mo_opstart_ack(struct gsm_abis_mo *mo);
int oml_mo_opstart_nack(struct gsm_abis_mo *mo, uint8_t nack_cause);
int oml_mo_statechg_ack(struct gsm_abis_mo *mo);
int oml_mo_statechg_nack(struct gsm_abis_mo *mo, uint8_t nack_cause);

/* Change the state and send STATE CHG REP */
int oml_mo_state_chg(struct gsm_abis_mo *mo, int op_state, int avail_state);

/* First initialization of MO, does _not_ generate state changes */
void oml_mo_state_init(struct gsm_abis_mo *mo, int op_state, int avail_state);

/* Update admin state and send ACK/NACK */
int oml_mo_rf_lock_chg(struct gsm_abis_mo *mo, uint8_t mute_state[8],
		       int success);

/* Transmit STATE CHG REP even if there was no state change */
int oml_tx_state_changed(struct gsm_abis_mo *mo);

int oml_mo_tx_sw_act_rep(struct gsm_abis_mo *mo);

int oml_fom_ack_nack(struct msgb *old_msg, uint8_t cause);

int oml_mo_fom_ack_nack(struct gsm_abis_mo *mo, uint8_t orig_msg_type,
			uint8_t cause);

#endif // _OML_H */
