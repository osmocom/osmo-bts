#pragma once

#include <osmo-bts/gsm_data.h>

/* One TRX in a BTS */
struct gsm_bts_trx {
	/* list header in bts->trx_list */
	struct llist_head list;

	struct gsm_bts *bts;
	/* number of this TRX in the BTS */
	uint8_t nr;
	/* human readable name / description */
	char *description;
	/* how do we talk RSL with this TRX? */
	uint8_t rsl_tei;
	struct e1inp_sign_link *rsl_link;

	/* Some BTS (specifically Ericsson RBS) have a per-TRX OML Link */
	struct e1inp_sign_link *oml_link;

	struct gsm_abis_mo mo;
	struct tlv_parsed nm_attr;
	struct {
		struct gsm_abis_mo mo;
	} bb_transc;

	uint16_t arfcn;
	int nominal_power;		/* in dBm */
	unsigned int max_power_red;	/* in actual dB */
        uint8_t max_power_backoff_8psk; /* in actual dB OC-2G only */
        uint8_t c0_idle_power_red;      /* in actual dB OC-2G only */


	struct trx_power_params power_params;
	bool ms_pwr_ctl_soft; /* is power control loop done by osmocom software? */

	struct {
		void *l1h;
	} role_bts;

	union {
		struct {
			unsigned int test_state;
			uint8_t test_nr;
			struct rxlev_stats rxlev_stat;
		} ipaccess;
	};
	struct gsm_bts_trx_ts ts[TRX_NR_TS];
};


struct gsm_bts_trx *gsm_bts_trx_alloc(struct gsm_bts *bts);
int bts_trx_init(struct gsm_bts_trx *trx);
struct gsm_bts_trx *gsm_bts_trx_num(const struct gsm_bts *bts, int num);
char *gsm_trx_name(const struct gsm_bts_trx *trx);
const char *gsm_trx_unit_id(struct gsm_bts_trx *trx);

int trx_link_estab(struct gsm_bts_trx *trx);
int trx_set_available(struct gsm_bts_trx *trx, int avail);

uint8_t num_agch(struct gsm_bts_trx *trx, const char * arg);
bool trx_ms_pwr_ctrl_is_osmo(struct gsm_bts_trx *trx);

#define LOGPTRX(trx, ss, lvl, fmt, args...) LOGP(ss, lvl, "%s " fmt, gsm_trx_name(trx), ## args)
