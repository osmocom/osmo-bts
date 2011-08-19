#ifndef _BB_L1_H
#define _BB_L1_H

extern uint16_t ref_arfcn;

#include <osmocom/core/write_queue.h>

enum baseband_role {
	BASEBAND_TX,
	BASEBAND_RX,
};

struct osmo_l1l2_if {

	enum baseband_role bb_role;
	struct bbl1_hdl *bbl1h;
	struct osmo_wqueue l2_wq;
};

struct bbl1_hdl {
	struct gsm_bts_trx *trx;
	struct osmo_l1l2_if l1l2if_tx, l1l2if_rx;
	int reset_cnf_tx, reset_cnf_rx;
};

int l1if_setup(struct gsm_bts_trx *trx);
int l1if_open(struct gsm_bts_trx *trx,  const char *socket_path);
int l1if_close(struct gsm_bts_trx *trx);

#endif /* _BB_L1_H */
