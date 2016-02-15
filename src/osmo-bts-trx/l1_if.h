#ifndef L1_IF_H_TRX
#define L1_IF_H_TRX

#include <osmo-bts/scheduler.h>

struct trx_config {
	uint8_t			poweron;	/* poweron(1) or poweroff(0) */
	int			poweron_sent;

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
	int			rxgain;
	int			rxgain_sent;

	int			power_valid;
	int			power;
	int			power_oml;
	int			power_sent;

	int			maxdly_valid;
	int			maxdly;
	int			maxdly_sent;

	uint8_t			slotmask;

	int			slottype_valid[TRX_NR_TS];
	uint8_t			slottype[TRX_NR_TS];
	int			slottype_sent[TRX_NR_TS];
};

struct trx_l1h {
	struct llist_head	trx_ctrl_list;

	struct gsm_bts_trx	*trx;

	struct osmo_fd		trx_ofd_ctrl;
	struct osmo_timer_list	trx_ctrl_timer;
	struct osmo_fd		trx_ofd_data;

	/* transceiver config */
	struct trx_config	config;
	uint8_t			ho_rach_detect[TRX_NR_TS][TS_MAX_LCHAN];

	struct l1sched_trx	l1s;
};

struct trx_l1h *l1if_open(struct gsm_bts_trx *trx);
void l1if_close(struct trx_l1h *l1h);
void l1if_reset(struct trx_l1h *l1h);
int check_transceiver_availability(struct gsm_bts *bts, int avail);
int l1if_provision_transceiver_trx(struct trx_l1h *l1h);
int l1if_provision_transceiver(struct gsm_bts *bts);
int l1if_mph_time_ind(struct gsm_bts *bts, uint32_t fn);
void l1if_fill_meas_res(struct osmo_phsap_prim *l1sap, uint8_t chan_nr, float ta,
	float ber, float rssi);
int l1if_process_meas_res(struct gsm_bts_trx *trx, uint8_t tn, uint32_t fn, uint8_t chan_nr,
	int n_errors, int n_bits_total, float rssi, float toa);

static inline struct l1sched_trx *trx_l1sched_hdl(struct gsm_bts_trx *trx)
{
	struct trx_l1h *l1h = trx_l1h_hdl(trx);
	return &l1h->l1s;
}

#endif /* L1_IF_H_TRX */
