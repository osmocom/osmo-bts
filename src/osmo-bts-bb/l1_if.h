#ifndef _BB_L1_IF_H
#define _BB_L1_IF_H

enum l1if_prim {
	L1IF_NULL = 0,
	L1IF_RESET_REQ,		/* reset baseband */
	L1IF_RESET_IND,
	L1IF_RESET_CNF,
	L1IF_SETUP_REQ,		/* e.g.: set sync parameters */
	L1IF_SETUP_CNF,
	L1IF_BCCH_REQ,		/* set system information messages */
	L1IF_RACH_IND,		/* receive channel request */
	L1IF_DATA_REQ,		/* exchange data */
	L1IF_DATA_IND,
	L1IF_DATA_CNF,
	L1IF_DATA_FLUSH,	/* flush pending data */
	L1IF_TRAFFIC_REQ,	/* exchange traffic data */
	L1IF_TRAFFIC_IND,
};

struct l1if_hdr {
	uint8_t msg_type;
	uint8_t data[0];
} __attribute__((packed));

struct osmo_l1_if {
	struct gsm_bts_trx *trx;
	int num_phones;
	struct osmo_l1ctl l1ctl[8];
	int reset_cnf[8];
	uint8_t u_mask[8];
	uint8_t d_mask[8];
};

int l1if_setup(struct gsm_bts_trx *trx);
int l1if_data_req_cb(struct osmo_prim_hdr *oph, void *ctx);
int l1if_reset(struct gsm_bts_trx *trx);
int l1if_recv(struct osmo_l1ctl *l1ctl, struct msgb *msg);
int l1if_open(struct gsm_bts_trx *trx,  const char *socket_path);
int l1if_close(struct gsm_bts_trx *trx);

#endif /* _BB_L1_IF_H */
