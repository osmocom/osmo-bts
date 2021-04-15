#pragma once

/* TRXC read/send buffer size */
#define TRXC_MSG_BUF_SIZE	1500
/* TRXD read/send buffer size */
#define TRXD_MSG_BUF_SIZE	512

struct trx_dl_burst_req;
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

typedef void trx_if_cmd_generic_cb(struct trx_l1h *l1h, int rc);
typedef void trx_if_cmd_poweronoff_cb(struct trx_l1h *l1h, bool poweronoff, int rc);
typedef void trx_if_cmd_setslot_cb(struct trx_l1h *l1h, uint8_t tn, uint8_t type, int rc);
typedef void trx_if_cmd_getnompower_cb(struct trx_l1h *l1h, int nominal_power, int rc);
typedef void trx_if_cmd_setpower_att_cb(struct trx_l1h *l1h, int power_att_db, int rc);

void trx_if_init(struct trx_l1h *l1h);
int trx_if_cmd_poweroff(struct trx_l1h *l1h, trx_if_cmd_poweronoff_cb *cb);
int trx_if_cmd_poweron(struct trx_l1h *l1h, trx_if_cmd_poweronoff_cb *cb);
int trx_if_cmd_settsc(struct trx_l1h *l1h, uint8_t tsc, trx_if_cmd_generic_cb *cb);
int trx_if_cmd_setbsic(struct trx_l1h *l1h, uint8_t bsic, trx_if_cmd_generic_cb *cb);
int trx_if_cmd_setrxgain(struct trx_l1h *l1h, int db);
int trx_if_cmd_getnompower(struct trx_l1h *l1h, trx_if_cmd_getnompower_cb *cb);
int trx_if_cmd_setpower_att(struct trx_l1h *l1h, int power_att_db, trx_if_cmd_setpower_att_cb *cb);
int trx_if_cmd_setmaxdly(struct trx_l1h *l1h, int dly);
int trx_if_cmd_setmaxdlynb(struct trx_l1h *l1h, int dly);
int trx_if_cmd_setslot(struct trx_l1h *l1h, uint8_t tn, uint8_t type, trx_if_cmd_setslot_cb *cb);
int trx_if_cmd_rxtune(struct trx_l1h *l1h, uint16_t arfcn, trx_if_cmd_generic_cb *cb);
int trx_if_cmd_txtune(struct trx_l1h *l1h, uint16_t arfcn, trx_if_cmd_generic_cb *cb);
int trx_if_cmd_handover(struct trx_l1h *l1h, uint8_t tn, uint8_t ss);
int trx_if_cmd_nohandover(struct trx_l1h *l1h, uint8_t tn, uint8_t ss);
int trx_if_cmd_rfmute(struct trx_l1h *l1h, bool mute);
int trx_if_send_burst(struct trx_l1h *l1h, const struct trx_dl_burst_req *br);
int trx_if_powered(struct trx_l1h *l1h);

/* The latest supported TRXD PDU version */
#define TRX_DATA_PDU_VER    1

/* Format negotiation command */
int trx_if_cmd_setformat(struct trx_l1h *l1h, uint8_t ver, trx_if_cmd_generic_cb *cb);
