#pragma once

#include <osmocom/core/utils.h>
#include <osmo-bts/scheduler.h>

/* NOTE: (2*GSM_BURST_LEN = VAMOS)
 * This ends up being EGPRS_BURST_LEN, 444 */
#define TRXD_BURST_SIZE_MAX	OSMO_MAX(2 * GSM_BURST_LEN, EGPRS_BURST_LEN)

/* Uplink TRXDv0 header length: TDMA TN + FN + RSSI + ToA256 */
#define TRXD_UL_V0HDR_LEN	(1 + 4 + 1 + 2)
/* Uplink TRXDv1 header length: additional MTS + C/I */
#define TRXD_UL_V1HDR_LEN	(TRXD_UL_V0HDR_LEN + 1 + 2)
/* Uplink TRXDv2 header length: TDMA TN + TRXN + MTS + RSSI + ToA256 + C/I */
#define TRXD_UL_V2HDR_LEN	(1 + 1 + 1 + 1 + 2 + 2)
/* NOTE: TRXDv0: a legacy transceiver may append two garbage bytes:
 * NOTE: TRXDv2: 4+ to account for optional TDMA Fn field: */
#define TRXD_UL_MAX_HDR_LEN	OSMO_MAX(TRXD_UL_V0HDR_LEN + 2, OSMO_MAX(TRXD_UL_V1HDR_LEN, 4 + TRXD_UL_V2HDR_LEN))
/* Note (OS#5827): once we support TRX batching, we miss here "* PCU_IF_NUM_TRX" */
#define TRXD_UL_MSG_BUF_SIZE	((TRXD_UL_MAX_HDR_LEN + TRXD_BURST_SIZE_MAX) * TRX_NR_TS)


/* Downlink TRXDv0/1 header length: TDMA TN + FN + Att */
#define TRXD_DL_V0HDR_LEN	(1 + 4 + 1)
#define TRXD_DL_V1HDR_LEN	TRXD_DL_V0HDR_LEN
/* Downlink TRXDv2 header length: TDMA TN + TRXN + MTS + Att + SCPIR + spare3 + FN (on first PDU) */
#define TRXD_DL_V2HDR_LEN	(1 + 1 + 1 + 1 + 1 + 3 + 4)
#define TRXD_DL_MAX_HDR_LEN	OSMO_MAX(TRXD_DL_V0HDR_LEN, OSMO_MAX(TRXD_DL_V1HDR_LEN, TRXD_DL_V2HDR_LEN))

/* Note (OS#5827): once we support TRX batching, we miss here "* PCU_IF_NUM_TRX" */
#define TRXD_DL_MSG_BUF_SIZE	((TRXD_DL_MAX_HDR_LEN + TRXD_BURST_SIZE_MAX) * TRX_NR_TS)

/* TRXC read/send buffer size */
#define TRXC_MSG_BUF_SIZE	1500

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
int trx_if_cmd_setslot(struct trx_l1h *l1h, uint8_t tn, trx_if_cmd_setslot_cb *cb);
int trx_if_cmd_rxtune(struct trx_l1h *l1h, uint16_t arfcn, trx_if_cmd_generic_cb *cb);
int trx_if_cmd_txtune(struct trx_l1h *l1h, uint16_t arfcn, trx_if_cmd_generic_cb *cb);
int trx_if_cmd_handover(struct trx_l1h *l1h, uint8_t tn, uint8_t ss);
int trx_if_cmd_nohandover(struct trx_l1h *l1h, uint8_t tn, uint8_t ss);
int trx_if_cmd_rfmute(struct trx_l1h *l1h, bool mute);
int trx_if_send_burst(struct trx_l1h *l1h, const struct trx_dl_burst_req *br);
int trx_if_powered(struct trx_l1h *l1h);

/* The latest supported TRXD PDU version */
#define TRX_DATA_PDU_VER    2

/* Format negotiation command */
int trx_if_cmd_setformat(struct trx_l1h *l1h, uint8_t ver, trx_if_cmd_generic_cb *cb);

int trx_ctrl_cmd_cb(struct trx_l1h *l1h, int critical, void *cb,
		    const char *cmd, const char *fmt, ...);
#define trx_ctrl_cmd(l1h, critical, cmd, fmt, ...) trx_ctrl_cmd_cb(l1h, critical, NULL, cmd, fmt, ##__VA_ARGS__)
