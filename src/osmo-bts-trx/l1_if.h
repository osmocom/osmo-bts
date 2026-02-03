#ifndef L1_IF_H_TRX
#define L1_IF_H_TRX

#include <osmocom/core/rate_ctr.h>
#include <osmocom/core/osmo_io.h>

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
	BTSTRX_CTR_SCHED_DL_FH_NO_CARRIER,
	BTSTRX_CTR_SCHED_DL_FH_CACHE_MISS,
	BTSTRX_CTR_SCHED_UL_FH_NO_CARRIER,
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
	uint8_t			trxd_pdu_ver_req; /* requested TRXD PDU version */
	uint8_t			trxd_pdu_ver_use; /* actual TRXD PDU version in use */
	bool			setformat_sent;
	bool			setformat_acked;

	bool			enabled;


	bool			arfcn_valid;
	uint16_t		arfcn;
	bool			rxtune_sent;
	bool			rxtune_acked;
	bool			txtune_sent;
	bool			txtune_acked;

	bool			tsc_valid;
	uint8_t			tsc;
	bool			tsc_sent;
	bool			tsc_acked;

	bool			bsic_valid;
	uint8_t			bsic;
	bool			bsic_sent;
	bool			bsic_acked;

	bool			rxgain_valid;
	uint8_t			rxgain;
	bool			rxgain_sent;

	int			forced_max_power_red; /* -1 if not forced by VTY config (default) */

	bool			nominal_power_set_by_vty; /* whether nominal trx power was enforced/retreived from VTY config "nominal-tx-power" */
	bool			nomtxpower_sent;
	bool			nomtxpower_acked;

	bool			setpower_sent;
	bool			setpower_acked;

	bool			maxdly_valid;
	int			maxdly;
	bool			maxdly_sent;

	bool			maxdlynb_valid;
	int			maxdlynb;
	bool			maxdlynb_sent;

	uint8_t			slotmask;

	bool			setslot_valid[TRX_NR_TS];
	struct {
		uint8_t slottype;
		uint8_t tsc_set;
		uint8_t tsc_val;
		bool tsc_valid;
	}			setslot[TRX_NR_TS];
	bool			setslot_sent[TRX_NR_TS];
};

struct trx_l1h {
	struct llist_head	trx_ctrl_list;
	/* Latest RSPed cmd, used to catch duplicate RSPs from sent retransmissions */
	struct trx_ctrl_msg 	*last_acked;
	/* Whether the code path is in the middle of handling a received message. */
	bool			in_trx_ctrl_read_cb;
	/* Whether the l1h->trx_ctrl_list was flushed by the callback handling a received message */
	bool			flushed_while_in_trx_ctrl_read_cb;

	struct {
		/* the Tx buffer used by trx_if_send_burst() */
		struct msgb	*sndbuf;
		/* number of PDUs in the Tx buffer */
		unsigned int	sndbuf_num_pdus;
	} data;

	//struct gsm_bts_trx	*trx;
	struct phy_instance	*phy_inst;

	struct osmo_io_fd	*trx_ctrl_iofd;
	struct osmo_timer_list	trx_ctrl_timer;
	struct osmo_io_fd	*trx_data_iofd;

	/* transceiver config */
	struct trx_config	config;
	struct osmo_fsm_inst	*provision_fi;
};

struct trx_l1h *trx_l1h_alloc(void *tall_ctx, struct phy_instance *pinst);
int l1if_provision_transceiver_trx(struct trx_l1h *l1h);
int l1if_mph_time_ind(struct gsm_bts *bts, uint32_t fn);
void l1if_trx_set_nominal_power(struct gsm_bts_trx *trx, int nominal_power);
int l1if_trx_start_power_ramp(struct gsm_bts_trx *trx, ramp_compl_cb_t ramp_compl_cb);
enum gsm_phys_chan_config transceiver_chan_type_2_pchan(uint8_t type);

#endif /* L1_IF_H_TRX */
