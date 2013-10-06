#include <osmo-bts/bts.h>

/*
 * Stubs to provide an empty bts model implementation for testing.
 * If we ever want to re-define such a symbol we can make them weak
 * here.
 */
const uint8_t abis_mac[6] = { 0,1,2,3,4,5 };
const char *software_version = "0815";

int bts_model_chg_adm_state(struct gsm_bts *bts, struct gsm_abis_mo *mo,
			    void *obj, uint8_t adm_state)
{ return 0; }
int bts_model_init(struct gsm_bts *bts)
{ return 0; }
int bts_model_apply_oml(struct gsm_bts *bts, struct msgb *msg,
			struct tlv_parsed *new_attr, void *obj)
{ return 0; }
int bts_model_rsl_chan_rel(struct gsm_lchan *lchan)
{ return 0;}

int bts_model_rsl_deact_sacch(struct gsm_lchan *lchan)
{ return 0; }

int bts_model_trx_deact_rf(struct gsm_bts_trx *trx)
{ return 0; }
int bts_model_trx_close(struct gsm_bts_trx *trx)
{ return 0; }
int bts_model_check_oml(struct gsm_bts *bts, uint8_t msg_type,
			struct tlv_parsed *old_attr, struct tlv_parsed *new_attr,
			void *obj)
{ return 0; }
int bts_model_opstart(struct gsm_bts *bts, struct gsm_abis_mo *mo,
		      void *obj)
{ return 0; }
int bts_model_rsl_chan_act(struct gsm_lchan *lchan, struct tlv_parsed *tp)
{ return 0; }
int bts_model_rsl_mode_modify(struct gsm_lchan *lchan)
{ return 0; }
void bts_model_rtp_rx_cb(struct osmo_rtp_socket *rs, const uint8_t *rtp_pl,
			 unsigned int rtp_pl_len) {}

int l1if_pdch_req(struct gsm_bts_trx_ts *ts, int is_ptcch, uint32_t fn,
        uint16_t arfcn, uint8_t block_nr, uint8_t *data, uint8_t len)
{ return 0; }

uint32_t trx_get_hlayer1(struct gsm_bts_trx *trx)
{ return 0; }

int bts_model_oml_estab(struct gsm_bts *bts)
{ return 0; }
