#ifndef _BB_L1_IF_H
#define _BB_L1_IF_H

enum l1if_prim {
	L1IF_RESET_REQ,
	L1IF_RESET_IND,
	L1IF_RESET_CNF,
	L1IF_SETUP_REQ,
	L1IF_SETUP_CNF,
	L1IF_BCCH_REQ,
	L1IF_RACH_IND,
};

struct l1if_hdr {
	uint8_t msg_type;
	uint8_t data[0];
} __attribute__((packed));

struct osmo_l1_if {
	struct gsm_bts_trx *trx;
	struct osmo_l1ctl l1ctl_tx, l1ctl_rx;
	int reset_cnf_tx, reset_cnf_rx;
};

int l1if_setup(struct gsm_bts_trx *trx);
int l1if_reset(struct gsm_bts_trx *trx);
int l1if_recv(struct osmo_l1ctl *l1ctl, struct msgb *msg);
int l1if_open(struct gsm_bts_trx *trx,  const char *socket_path);
int l1if_close(struct gsm_bts_trx *trx);

#endif /* _BB_L1_IF_H */
