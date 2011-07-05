#ifndef _FEMTO_L1_H
#define _FEMTO_L1_H

#include <osmocom/core/select.h>
#include <osmocom/core/write_queue.h>
#include <osmocom/gsm/gsm_utils.h>

enum {
	MQ_SYS_READ,
	MQ_L1_READ,
	_NUM_MQ_READ
};

enum {
	MQ_SYS_WRITE,
	MQ_L1_WRITE,
	_NUM_MQ_WRITE
};

struct femtol1_hdl {
	struct gsm_time gsm_time;
	uint32_t hLayer1;			/* handle to the L1 instance in the DSP */
	uint32_t dsp_trace_f;
	struct llist_head wlc_list;

	void *priv;			/* user reference */

	struct osmo_timer_list alive_timer;
	unsigned int alive_prim_cnt;

	struct osmo_fd read_ofd[_NUM_MQ_READ];	/* osmo file descriptors */
	struct osmo_wqueue write_q[_NUM_MQ_WRITE];
};

#define msgb_l1prim(msg)	((GsmL1_Prim_t *)(msg)->l1h)
#define msgb_sysprim(msg)	((FemtoBts_Prim_t *)(msg)->l1h)

typedef int l1if_compl_cb(struct msgb *l1_msg, void *data);

/* send a request primitive to the L1 and schedule completion call-back */
int l1if_req_compl(struct femtol1_hdl *fl1h, struct msgb *msg,
		   int is_system_prim, l1if_compl_cb *cb, void *data);

struct femtol1_hdl *l1if_open(void *priv);
int l1if_close(struct femtol1_hdl *hdl);
int l1if_reset(struct femtol1_hdl *hdl);
int l1if_activate_rf(struct femtol1_hdl *hdl, int on);
int l1if_set_trace_flags(struct femtol1_hdl *hdl, uint32_t flags);

struct msgb *l1p_msgb_alloc(void);
struct msgb *sysp_msgb_alloc(void);

uint32_t l1if_lchan_to_hLayer2(struct gsm_lchan *lchan);
struct gsm_lchan *l1if_hLayer2_to_lchan(struct gsm_bts_trx *trx, uint32_t hLayer2);

#endif /* _FEMTO_L1_H */
