#ifndef BTS_MODEL_H
#define BTS_MODEL_H

#include <stdint.h>

#include <osmocom/gsm/tlv.h>
#include <osmocom/gsm/gsm_utils.h>

#include <osmo-bts/gsm_data.h>

/* BTS model specific functions needed by the common code */

int bts_model_init(struct gsm_bts *bts);

int bts_model_check_oml(struct gsm_bts *bts, uint8_t msg_type,
			struct tlv_parsed *old_attr, struct tlv_parsed *new_attr,
			void *obj);

int bts_model_apply_oml(struct gsm_bts *bts, struct msgb *msg,
			struct tlv_parsed *new_attr, int obj_kind, void *obj);

int bts_model_opstart(struct gsm_bts *bts, struct gsm_abis_mo *mo,
		      void *obj);

int bts_model_chg_adm_state(struct gsm_bts *bts, struct gsm_abis_mo *mo,
			    void *obj, uint8_t adm_state);

int bts_model_trx_deact_rf(struct gsm_bts_trx *trx);
int bts_model_trx_close(struct gsm_bts_trx *trx);

int bts_model_vty_init(struct gsm_bts *bts);

void bts_model_config_write_bts(struct vty *vty, struct gsm_bts *bts);
void bts_model_config_write_trx(struct vty *vty, struct gsm_bts_trx *trx);

int bts_model_oml_estab(struct gsm_bts *bts);

int bts_model_l1sap_down(struct gsm_bts_trx *trx, struct osmo_phsap_prim *l1sap);

void bts_model_abis_close(struct gsm_bts *bts);

#endif
