#ifndef L1_IF_H_TRX
#define L1_IF_H_TRX

#include <osmocom/core/rate_ctr.h>

#include <osmo-bts/scheduler.h>
#include <osmo-bts/phy_link.h>
#include "trx_if.h"

/*
 * TRX frame clock handling
 *
 * In a "normal" synchronous PHY layer, we would be polled every time
 * the PHY needs data for a given frame number.  However, the
 * OpenBTS-inherited TRX protocol works differently:  We (L1) must
 * autonomously send burst data based on our own clock, and every so
 * often (currently every ~ 216 frames), we get a clock indication from
 * the TRX.
 *
 * We're using a MONOTONIC timerfd interval timer for the 4.615ms frame
 * intervals, and then compute + send the 8 bursts for that frame.
 *
 * Upon receiving a clock indication from the TRX, we compensate
 * accordingly: If we were transmitting too fast, we're delaying the
 * next interval timer accordingly.  If we were too slow, we immediately
 * send burst data for the missing frame numbers.
 */

/* bts-trx specific rate counters */
enum {
	BTSTRX_CTR_SCHED_DL_MISS_FN,
};

/*! clock state of a given TRX */
struct osmo_trx_clock_state {
	/*! number of FN periods without TRX clock indication */
	uint32_t fn_without_clock_ind;
	struct {
		/*! last FN we processed based on FN period timer */
		uint32_t fn;
		/*! time at which we last processed FN */
		struct timespec tv;
	} last_fn_timer;
	struct {
		/*! last FN we received a clock indication for */
		uint32_t fn;
		/*! time at which we received the last clock indication */
		struct timespec tv;
	} last_clk_ind;
	/*! Osmocom FD wrapper for timerfd */
	struct osmo_fd fn_timer_ofd;
};

/* gsm_bts->model_priv, specific to osmo-bts-trx */
struct bts_trx_priv {
	struct osmo_trx_clock_state clk_s;
	struct rate_ctr_group *ctrs;		/* bts-trx specific rate counters */
};

struct trx_config {
	uint8_t			trxd_hdr_ver_req; /* requested TRXD header version */
	uint8_t			trxd_hdr_ver_use; /* actual TRXD header version in use */
	int			setformat_sent;

	bool			enabled;


	int			arfcn_valid;
	uint16_t		arfcn;
	int			arfcn_sent;

	int			tsc_valid;
	uint8_t			tsc;
	int			tsc_sent;

	int			bsic_valid;
	uint8_t			bsic;
	int			bsic_sent;

	int			rxgain_valid;
	uint8_t			rxgain;
	int			rxgain_sent;

	int			forced_max_power_red; /* -1 if not forced by VTY config (default) */

	bool			nominal_power_set_by_vty; /* whether nominal trx power was enforced/retreived from VTY config "nominal-tx-power" */

	int			maxdly_valid;
	int			maxdly;
	int			maxdly_sent;

	int			maxdlynb_valid;
	int			maxdlynb;
	int			maxdlynb_sent;

	uint8_t			slotmask;

	int			slottype_valid[TRX_NR_TS];
	uint8_t			slottype[TRX_NR_TS];
	int			slottype_sent[TRX_NR_TS];
};

struct trx_l1h {
	struct llist_head	trx_ctrl_list;
	/* Latest RSPed cmd, used to catch duplicate RSPs from sent retransmissions */
	struct trx_ctrl_msg 	*last_acked;

	//struct gsm_bts_trx	*trx;
	struct phy_instance	*phy_inst;

	struct osmo_fd		trx_ofd_ctrl;
	struct osmo_timer_list	trx_ctrl_timer;
	struct osmo_fd		trx_ofd_data;

	/* transceiver config */
	struct trx_config	config;
	struct osmo_fsm_inst	*provision_fi;

	struct l1sched_trx	l1s;
};

struct trx_l1h *trx_l1h_alloc(void *tall_ctx, struct phy_instance *pinst);
int l1if_provision_transceiver_trx(struct trx_l1h *l1h);
int l1if_mph_time_ind(struct gsm_bts *bts, uint32_t fn);
void l1if_trx_set_nominal_power(struct gsm_bts_trx *trx, int nominal_power);
int l1if_trx_start_power_ramp(struct gsm_bts_trx *trx, ramp_compl_cb_t ramp_compl_cb);
enum gsm_phys_chan_config transceiver_chan_type_2_pchan(uint8_t type);

static inline struct l1sched_trx *trx_l1sched_hdl(struct gsm_bts_trx *trx)
{
	struct phy_instance *pinst = trx->role_bts.l1h;
	struct trx_l1h *l1h = pinst->u.osmotrx.hdl;
	return &l1h->l1s;
}

#endif /* L1_IF_H_TRX */
