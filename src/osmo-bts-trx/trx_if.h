#ifndef TRX_IF_H
#define TRX_IF_H

extern int transceiver_available;

struct trx_l1h;

struct trx_ctrl_msg {
	struct llist_head	list;
	char 			cmd[28];
	char 			params[100];
	int			cmd_len;
	int			params_len;
	int			critical;
	void 			*cb;
};

typedef void trx_if_cmd_poweronoff_cb(struct trx_l1h *l1h, bool poweronoff, int rc);
typedef void trx_if_cmd_setslot_cb(struct trx_l1h *l1h, uint8_t tn, uint8_t type, int rc);

void trx_if_init(struct trx_l1h *l1h);
int trx_if_cmd_poweroff(struct trx_l1h *l1h, trx_if_cmd_poweronoff_cb *cb);
int trx_if_cmd_poweron(struct trx_l1h *l1h, trx_if_cmd_poweronoff_cb *cb);
int trx_if_cmd_settsc(struct trx_l1h *l1h, uint8_t tsc);
int trx_if_cmd_setbsic(struct trx_l1h *l1h, uint8_t bsic);
int trx_if_cmd_setrxgain(struct trx_l1h *l1h, int db);
int trx_if_cmd_setpower(struct trx_l1h *l1h, int db);
int trx_if_cmd_setmaxdly(struct trx_l1h *l1h, int dly);
int trx_if_cmd_setmaxdlynb(struct trx_l1h *l1h, int dly);
int trx_if_cmd_setslot(struct trx_l1h *l1h, uint8_t tn, uint8_t type, trx_if_cmd_setslot_cb *cb);
int trx_if_cmd_rxtune(struct trx_l1h *l1h, uint16_t arfcn);
int trx_if_cmd_txtune(struct trx_l1h *l1h, uint16_t arfcn);
int trx_if_cmd_handover(struct trx_l1h *l1h, uint8_t tn, uint8_t ss);
int trx_if_cmd_nohandover(struct trx_l1h *l1h, uint8_t tn, uint8_t ss);
int trx_if_send_burst(struct trx_l1h *l1h, uint8_t tn, uint32_t fn, uint8_t pwr,
	const ubit_t *bits, uint16_t nbits);
int trx_if_powered(struct trx_l1h *l1h);

/* The latest supported TRXD header format version */
#define TRX_DATA_FORMAT_VER    1

/* Format negotiation command */
int trx_if_cmd_setformat(struct trx_l1h *l1h, uint8_t ver);

#endif /* TRX_IF_H */
