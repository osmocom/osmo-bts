#pragma once

typedef int trx_sched_rts_func(struct l1sched_trx *l1t, uint8_t tn,
			       uint32_t fn, enum trx_chan_type chan);

typedef ubit_t *trx_sched_dl_func(struct l1sched_trx *l1t, uint8_t tn,
				  uint32_t fn, enum trx_chan_type chan,
				  uint8_t bid);

typedef int trx_sched_ul_func(struct l1sched_trx *l1t, uint8_t tn,
			      uint32_t fn, enum trx_chan_type chan,
			      uint8_t bid, sbit_t *bits, int8_t rssi,
			      float toa);

struct trx_chan_desc {
	/*! \brief Is this on a PDCH (PS) ? */
	int			pdch;
	/*! \brief TRX Channel Type */
	enum trx_chan_type	chan;
	/*! \brief Channel Number (like in RSL) */
	uint8_t			chan_nr;
	/*! \brief Link ID (like in RSL) */
	uint8_t			link_id;
	/*! \brief Human-readable name */
	const char		*name;
	/*! \brief function to call when we want to generate RTS.req to L2 */
	trx_sched_rts_func	*rts_fn;
	/*! \brief function to call when DATA.req received from L2 */
	trx_sched_dl_func	*dl_fn;
	/*! \brief function to call when burst received from PHY */
	trx_sched_ul_func	*ul_fn;
	/*! \breif is this channel automatically active at start? */
	int			auto_active;
};
extern const struct trx_chan_desc trx_chan_desc[_TRX_CHAN_MAX];

extern const ubit_t _sched_tsc[8][26];
const ubit_t _sched_fcch_burst[148];
const ubit_t _sched_sch_train[64];

struct msgb *_sched_dequeue_prim(struct l1sched_trx *l1t, int8_t tn, uint32_t fn,
				 enum trx_chan_type chan);

int _sched_compose_ph_data_ind(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
				enum trx_chan_type chan, uint8_t *l2, uint8_t l2_len, float rssi, enum osmo_ph_pres_info_type presence_info);

int _sched_compose_tch_ind(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
		    enum trx_chan_type chan, uint8_t *tch, uint8_t tch_len);

ubit_t *tx_idle_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid);
ubit_t *tx_fcch_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid);
ubit_t *tx_sch_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid);
ubit_t *tx_data_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid);
ubit_t *tx_pdtch_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid);
ubit_t *tx_tchf_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid);
ubit_t *tx_tchh_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid);
int rx_rach_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, sbit_t *bits, int8_t rssi,
	float toa);
int rx_data_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, sbit_t *bits, int8_t rssi,
	float toa);
int rx_pdtch_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, sbit_t *bits, int8_t rssi,
	float toa);
int rx_tchf_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, sbit_t *bits, int8_t rssi,
	float toa);
int rx_tchh_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, sbit_t *bits, int8_t rssi,
	float toa);

const ubit_t *_sched_dl_burst(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn);
int _sched_rts(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn);
void _sched_act_rach_det(struct l1sched_trx *l1t, uint8_t tn, uint8_t ss, int activate);
