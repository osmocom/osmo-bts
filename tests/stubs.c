#include <osmo-bts/bts.h>
#include <osmo-bts/bts_model.h>

struct femtol1_hdl;
struct bts_model_set_dyn_pdch_data;
struct phy_link;
struct phy_instance;

/*
 * Stubs to provide an empty bts model implementation for testing.
 * If we ever want to re-define such a symbol we can make them weak
 * here.
 */
int bts_model_chg_adm_state(struct gsm_bts *bts, struct gsm_abis_mo *mo,
			    void *obj, uint8_t adm_state)
{ return 0; }
int bts_model_init(struct gsm_bts *bts)
{ return 0; }
int bts_model_trx_init(struct gsm_bts_trx *trx)
{ return 0; }
int bts_model_apply_oml(struct gsm_bts *bts, const struct msgb *msg,
			 struct gsm_abis_mo *mo, void *obj)
{ return 0; }

int bts_model_trx_deact_rf(struct gsm_bts_trx *trx)
{ return 0; }
void bts_model_trx_close(struct gsm_bts_trx *trx)
{ bts_model_trx_close_cb(trx, 0); }
int bts_model_check_oml(struct gsm_bts *bts, uint8_t msg_type,
			struct tlv_parsed *old_attr, struct tlv_parsed *new_attr,
			void *obj)
{ return 0; }
int bts_model_opstart(struct gsm_bts *bts, struct gsm_abis_mo *mo,
		      void *obj)
{ return 0; }
__attribute__((weak)) int bts_model_l1sap_down(struct gsm_bts_trx *trx, struct osmo_phsap_prim *l1sap)
{ return 0; }

uint32_t trx_get_hlayer1(const struct gsm_bts_trx *trx)
{ return 0; }

__attribute__((weak)) int bts_model_oml_estab(struct gsm_bts *bts)
{ return 0; }

__attribute__((weak)) int bts_model_change_power(struct gsm_bts_trx *trx, int p_trxout_mdBm)
{ return 0; }

int l1if_set_txpower(struct femtol1_hdl *fl1h, float tx_power)
{ return 0; }

int bts_model_lchan_deactivate(struct gsm_lchan *lchan) { return 0; }
int bts_model_lchan_deactivate_sacch(struct gsm_lchan *lchan) { return 0; }

int bts_model_adjst_ms_pwr(struct gsm_lchan *lchan)
{ return 0; }

__attribute__((weak)) void bts_model_abis_close(struct gsm_bts *bts)
{ }

int bts_model_ts_disconnect(struct gsm_bts_trx_ts *ts)
{ return 0; }

void bts_model_ts_connect(struct gsm_bts_trx_ts *ts,
			 enum gsm_phys_chan_config as_pchan)
{ return; }

void bts_model_phy_link_set_defaults(struct phy_link *plink)
{ return; }

void bts_model_phy_instance_set_defaults(struct phy_instance *pinst)
{ return; }

int bts_model_phy_link_open(struct phy_link *plink)
{ return 0; }
