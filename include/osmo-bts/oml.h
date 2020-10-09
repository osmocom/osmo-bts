#ifndef _OML_H
#define _OML_H

#include <osmocom/gsm/protocol/gsm_12_21.h>

struct gsm_bts;
struct gsm_abis_mo;
struct msgb;
struct gsm_lchan;

/* Network Management State */
struct gsm_nm_state {
	enum abis_nm_op_state operational;
	enum abis_nm_adm_state administrative;
	enum abis_nm_avail_state availability;
};

struct gsm_abis_mo {
	/* A-bis OML Object Class */
	uint8_t obj_class;
	/* is there still some procedure pending? */
	uint8_t procedure_pending;
	/* A-bis OML Object Instance */
	struct abis_om_obj_inst obj_inst;
	/* human-readable name */
	const char *name;
	/* NM State */
	struct gsm_nm_state nm_state;
	/* Attributes configured in this MO */
	struct tlv_parsed *nm_attr;
	/* BTS to which this MO belongs */
	struct gsm_bts *bts;
};

int oml_init(void);
int down_oml(struct gsm_bts *bts, struct msgb *msg);

struct msgb *oml_msgb_alloc(void);
int oml_send_msg(struct msgb *msg, int is_mauf);
int oml_mo_send_msg(const struct gsm_abis_mo *mo, struct msgb *msg, uint8_t msg_type);
int oml_mo_opstart_ack(const struct gsm_abis_mo *mo);
int oml_mo_opstart_nack(const struct gsm_abis_mo *mo, uint8_t nack_cause);
int oml_mo_statechg_ack(const struct gsm_abis_mo *mo);
int oml_mo_statechg_nack(const struct gsm_abis_mo *mo, uint8_t nack_cause);

/* Change the state and send STATE CHG REP */
int oml_mo_state_chg(struct gsm_abis_mo *mo, int op_state, int avail_state);

/* First initialization of MO, does _not_ generate state changes */
void oml_mo_state_init(struct gsm_abis_mo *mo, int op_state, int avail_state);

/* Update admin state and send ACK/NACK */
int oml_mo_rf_lock_chg(struct gsm_abis_mo *mo, uint8_t mute_state[8],
		       int success);

/* Transmit STATE CHG REP even if there was no state change */
int oml_tx_state_changed(const struct gsm_abis_mo *mo);

int oml_mo_tx_sw_act_rep(const struct gsm_abis_mo *mo);

int oml_fom_ack_nack(struct msgb *old_msg, uint8_t cause);

int oml_mo_fom_ack_nack(const struct gsm_abis_mo *mo, uint8_t orig_msg_type,
			uint8_t cause);

extern const unsigned int oml_default_t200_ms[7];

/* Transmit failure event report */
int oml_tx_failure_event_rep(const struct gsm_abis_mo *mo, enum abis_nm_severity severity,
			     uint16_t cause_value, const char *fmt, ...);

void gsm_mo_init(struct gsm_abis_mo *mo, struct gsm_bts *bts,
		 uint8_t obj_class, uint8_t p1, uint8_t p2, uint8_t p3);

struct gsm_abis_mo *gsm_objclass2mo(struct gsm_bts *bts, uint8_t obj_class,
				    const struct abis_om_obj_inst *obj_inst);

struct gsm_nm_state *gsm_objclass2nmstate(struct gsm_bts *bts, uint8_t obj_class,
					  const struct abis_om_obj_inst *obj_inst);
void *gsm_objclass2obj(struct gsm_bts *bts, uint8_t obj_class,
		       const struct abis_om_obj_inst *obj_inst);

#endif // _OML_H */
