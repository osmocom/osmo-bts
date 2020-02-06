#pragma once

#define LOGL1S(subsys, level, l1t, tn, chan, fn, fmt, args ...)	\
		LOGP(subsys, level, "%s %s %s: " fmt,		\
			gsm_fn_as_gsmtime_str(fn),		\
			gsm_ts_name(&(l1t)->trx->ts[tn]),	\
			chan >=0 ? trx_chan_desc[chan].name : "", ## args)

typedef int trx_sched_rts_func(struct l1sched_trx *l1t, uint8_t tn,
			       uint32_t fn, enum trx_chan_type chan);

typedef ubit_t *trx_sched_dl_func(struct l1sched_trx *l1t, uint8_t tn,
				  uint32_t fn, enum trx_chan_type chan,
				  uint8_t bid, uint16_t *nbits);

typedef int trx_sched_ul_func(struct l1sched_trx *l1t, enum trx_chan_type chan,
			      uint8_t bid, const struct trx_ul_burst_ind *bi);

struct trx_chan_desc {
	/*! \brief Human-readable name */
	const char		*name;
	/*! \brief Human-readable description */
	const char		*desc;
	/*! \brief Channel Number (like in RSL) */
	uint8_t			chan_nr;
	/*! \brief Link ID (like in RSL) */
	uint8_t			link_id;
	/*! \brief function to call when we want to generate RTS.req to L2 */
	trx_sched_rts_func	*rts_fn;
	/*! \brief function to call when DATA.req received from L2 */
	trx_sched_dl_func	*dl_fn;
	/*! \brief function to call when burst received from PHY */
	trx_sched_ul_func	*ul_fn;
	/*! \brief channel flags, see TRX_CHAN_FLAG_* */
	uint8_t			flags;
};
extern const struct trx_chan_desc trx_chan_desc[_TRX_CHAN_MAX];

extern const ubit_t _sched_tsc[8][26];
extern const ubit_t _sched_egprs_tsc[8][78];
extern const ubit_t _sched_fcch_burst[148];
extern const ubit_t _sched_sch_train[64];

struct msgb *_sched_dequeue_prim(struct l1sched_trx *l1t, int8_t tn, uint32_t fn,
				 enum trx_chan_type chan);

int _sched_compose_ph_data_ind(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
			       enum trx_chan_type chan, uint8_t *l2,
			       uint8_t l2_len, float rssi,
			       int16_t ta_offs_256bits, int16_t link_qual_cb,
			       uint16_t ber10k,
			       enum osmo_ph_pres_info_type presence_info);

int _sched_compose_tch_ind(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
			   enum trx_chan_type chan, uint8_t *tch, uint8_t tch_len,
			   int16_t ta_offs_256bits, uint16_t ber10k, float rssi,
			   uint8_t is_sub);

ubit_t *tx_idle_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, uint16_t *nbits);
ubit_t *tx_fcch_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, uint16_t *nbits);
ubit_t *tx_sch_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, uint16_t *nbits);
ubit_t *tx_data_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, uint16_t *nbits);
ubit_t *tx_pdtch_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, uint16_t *nbits);
ubit_t *tx_tchf_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, uint16_t *nbits);
ubit_t *tx_tchh_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, uint8_t bid, uint16_t *nbits);
int rx_rach_fn(struct l1sched_trx *l1t, enum trx_chan_type chan,
	       uint8_t bid, const struct trx_ul_burst_ind *bi);
int rx_data_fn(struct l1sched_trx *l1t, enum trx_chan_type chan,
	       uint8_t bid, const struct trx_ul_burst_ind *bi);
int rx_pdtch_fn(struct l1sched_trx *l1t, enum trx_chan_type chan,
	        uint8_t bid, const struct trx_ul_burst_ind *bi);
int rx_tchf_fn(struct l1sched_trx *l1t, enum trx_chan_type chan,
	       uint8_t bid, const struct trx_ul_burst_ind *bi);
int rx_tchh_fn(struct l1sched_trx *l1t, enum trx_chan_type chan,
	       uint8_t bid, const struct trx_ul_burst_ind *bi);

const ubit_t *_sched_dl_burst(struct l1sched_trx *l1t, uint8_t tn,
			      uint32_t fn, uint16_t *nbits);
int _sched_rts(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn);
void _sched_act_rach_det(struct l1sched_trx *l1t, uint8_t tn, uint8_t ss, int activate);
