#ifndef _FEMTO_L1_H
#define _FEMTO_L1_H

#include <osmocom/core/select.h>
#include <osmocom/core/write_queue.h>
#include <osmocom/core/gsmtap_util.h>
#include <osmocom/gsm/gsm_utils.h>

enum {
	MQ_SYS_READ,
	MQ_L1_READ,
#ifndef HW_SYSMOBTS_V1
	MQ_TCH_READ,
	MQ_PDTCH_READ,
#endif
	_NUM_MQ_READ
};

enum {
	MQ_SYS_WRITE,
	MQ_L1_WRITE,
#ifndef HW_SYSMOBTS_V1
	MQ_TCH_WRITE,
	MQ_PDTCH_WRITE,
#endif
	_NUM_MQ_WRITE
};

struct femtol1_hdl {
	struct gsm_time gsm_time;
	uint32_t hLayer1;			/* handle to the L1 instance in the DSP */
	uint32_t dsp_trace_f;
	uint16_t clk_cal;
	uint8_t clk_src;
	struct llist_head wlc_list;

	struct gsmtap_inst *gsmtap;
	uint32_t gsmtap_sapi_mask;

	void *priv;			/* user reference */

	struct osmo_timer_list alive_timer;
	unsigned int alive_prim_cnt;

	struct osmo_fd read_ofd[_NUM_MQ_READ];	/* osmo file descriptors */
	struct osmo_wqueue write_q[_NUM_MQ_WRITE];

	struct {
		uint8_t dsp_version[3];
		uint8_t fpga_version[3];
		uint32_t band_support;	/* bitmask of GSM_BAND_* */
	} hw_info;
};

#define msgb_l1prim(msg)	((GsmL1_Prim_t *)(msg)->l1h)
#define msgb_sysprim(msg)	((SuperFemto_Prim_t *)(msg)->l1h)

typedef int l1if_compl_cb(struct msgb *l1_msg, void *data);

/* send a request primitive to the L1 and schedule completion call-back */
int l1if_req_compl(struct femtol1_hdl *fl1h, struct msgb *msg,
		   int is_system_prim, l1if_compl_cb *cb, void *data);

struct femtol1_hdl *l1if_open(void *priv);
int l1if_close(struct femtol1_hdl *hdl);
int l1if_reset(struct femtol1_hdl *hdl);
int l1if_activate_rf(struct femtol1_hdl *hdl, int on);
int l1if_set_trace_flags(struct femtol1_hdl *hdl, uint32_t flags);
int l1if_set_txpower(struct femtol1_hdl *fl1h, float tx_power);

struct msgb *l1p_msgb_alloc(void);
struct msgb *sysp_msgb_alloc(void);

uint32_t l1if_lchan_to_hLayer2(struct gsm_lchan *lchan);
struct gsm_lchan *l1if_hLayer2_to_lchan(struct gsm_bts_trx *trx, uint32_t hLayer2);

/* tch.c */
int l1if_tch_rx(struct gsm_lchan *lchan, struct msgb *l1p_msg);
int l1if_tch_fill(struct gsm_lchan *lchan, uint8_t *l1_buffer);
struct msgb *gen_empty_tch_msg(struct gsm_lchan *lchan);

#endif /* _FEMTO_L1_H */
