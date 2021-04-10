#pragma once

#include <osmo-bts/gsm_data.h>

/* For VAMOS, there is a "shadow TRX" for each normal TRX, to handle multiple MS on the same timeslot. The shadow TRX is
 * visible on OML and RSL, but there is only a single struct gsm_bts_trx for the TRX and the shadow TRX. Also, there is
 * only a single timeslot FSM handling both the "normal" and the secondary VAMOS lchans. */
#define TRX_SHADOW_NR(NR) (0x80 + (NR))

/* For any gsm_bts_trx (VAMOS shadow or primary trx), return the primary gsm_bts_trx pointer. Useful for all code that
 * handles CCHAN and TRXMGMT, which is always done on the primary TRX's RSL link. */
#define TRX_PRIMARY(TRX) ((TRX)->vamos.primary_trx ? : (TRX))

struct gsm_bts_bb_trx {
	struct gsm_abis_mo mo;
};

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

	/* NM Radio Carrier and Baseband Transciever */
	struct gsm_abis_mo mo;
	struct gsm_bts_bb_trx bb_transc;

	uint16_t arfcn;
	int nominal_power;		/* in dBm */
	unsigned int max_power_red;	/* in actual dB */
        uint8_t max_power_backoff_8psk; /* in actual dB OC-2G only */
        uint8_t c0_idle_power_red;      /* in actual dB OC-2G only */


	struct trx_power_params power_params;
	struct gsm_power_ctrl_params *bs_dpc_params; /* BS Dynamic Power Control */
	struct gsm_power_ctrl_params *ms_dpc_params; /* MS Dynamic Power Control */
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

	struct {
		/* If this is a primary TRX that has a shadow TRX, this points at the shadow TRX.
		 * NULL when this TRX is a shadow TRX itself, or when there is no shadow TRX set up. */
		struct gsm_bts_trx *shadow_trx;
		/* If this is a shadow TRX, this points at the primary TRX. NULL if this is a primary TRX. */
		struct gsm_bts_trx *primary_trx;
	} vamos;
};

static inline struct gsm_bts_trx *gsm_bts_bb_trx_get_trx(struct gsm_bts_bb_trx *bb_transc) {
	return (struct gsm_bts_trx *)container_of(bb_transc, struct gsm_bts_trx, bb_transc);
}

struct gsm_bts_trx *gsm_bts_trx_alloc(struct gsm_bts *bts, struct gsm_bts_trx *shadow_for_primary_trx);
int bts_trx_init(struct gsm_bts_trx *trx);
struct gsm_bts_trx *gsm_bts_trx_num(const struct gsm_bts *bts, int num);
char *gsm_trx_name(const struct gsm_bts_trx *trx);
const char *gsm_trx_unit_id(struct gsm_bts_trx *trx);

int trx_link_estab(struct gsm_bts_trx *trx);
void trx_operability_update(struct gsm_bts_trx *trx);

uint8_t num_agch(struct gsm_bts_trx *trx, const char * arg);
bool trx_ms_pwr_ctrl_is_osmo(const struct gsm_bts_trx *trx);

#define LOGPTRX(trx, ss, lvl, fmt, args...) LOGP(ss, lvl, "%s " fmt, gsm_trx_name(trx), ## args)
