#pragma once

#define LOGL1S(subsys, level, l1ts, chan, fn, fmt, args ...)	\
		LOGP(subsys, level, "%s %s %s: " fmt,		\
			gsm_fn_as_gsmtime_str(fn),		\
			gsm_ts_name((l1ts)->ts),		\
			chan >=0 ? trx_chan_desc[chan].name : "", ## args)

/* Logging helper adding context from trx_{ul,dl}_burst_{ind,req} */
#define LOGL1SB(subsys, level, l1ts, b, fmt, args ...) \
	LOGL1S(subsys, level, l1ts, (b)->chan, (b)->fn, fmt, ## args)

typedef int trx_sched_rts_func(const struct l1sched_ts *l1ts, const struct trx_dl_burst_req *br);
typedef int trx_sched_dl_func(struct l1sched_ts *l1ts, struct trx_dl_burst_req *br);
typedef int trx_sched_ul_func(struct l1sched_ts *l1ts, const struct trx_ul_burst_ind *bi);

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

extern const ubit_t _sched_dummy_burst[];
extern const ubit_t _sched_train_seq_gmsk_nb[4][8][26];
extern const ubit_t _sched_train_seq_8psk_nb[8][78];
extern const ubit_t _sched_train_seq_gmsk_sb[64];

struct msgb *_sched_dequeue_prim(struct l1sched_ts *l1ts, const struct trx_dl_burst_req *br);

int _sched_compose_ph_data_ind(struct l1sched_ts *l1ts, uint32_t fn,
			       enum trx_chan_type chan,
			       uint8_t *data, uint8_t data_len,
			       uint16_t ber10k, float rssi,
			       int16_t ta_offs_256bits, int16_t link_qual_cb,
			       enum osmo_ph_pres_info_type presence_info);

int _sched_compose_tch_ind(struct l1sched_ts *l1ts, uint32_t fn,
			   enum trx_chan_type chan,
			   uint8_t *data, uint8_t data_len,
			   uint16_t ber10k, float rssi,
			   int16_t ta_offs_256bits, int16_t link_qual_cb,
			   uint8_t is_sub);

int tx_fcch_fn(struct l1sched_ts *l1ts, struct trx_dl_burst_req *br);
int tx_sch_fn(struct l1sched_ts *l1ts, struct trx_dl_burst_req *br);
int tx_data_fn(struct l1sched_ts *l1ts, struct trx_dl_burst_req *br);
int tx_pdtch_fn(struct l1sched_ts *l1ts, struct trx_dl_burst_req *br);
int tx_tchf_fn(struct l1sched_ts *l1ts, struct trx_dl_burst_req *br);
int tx_tchh_fn(struct l1sched_ts *l1ts, struct trx_dl_burst_req *br);

int rx_rach_fn(struct l1sched_ts *l1ts, const struct trx_ul_burst_ind *bi);
int rx_data_fn(struct l1sched_ts *l1ts, const struct trx_ul_burst_ind *bi);
int rx_pdtch_fn(struct l1sched_ts *l1ts, const struct trx_ul_burst_ind *bi);
int rx_tchf_fn(struct l1sched_ts *l1ts, const struct trx_ul_burst_ind *bi);
int rx_tchh_fn(struct l1sched_ts *l1ts, const struct trx_ul_burst_ind *bi);

void _sched_dl_burst(struct l1sched_ts *l1ts, struct trx_dl_burst_req *br);
int _sched_rts(const struct l1sched_ts *l1ts, uint32_t fn);
void _sched_act_rach_det(struct gsm_bts_trx *trx, uint8_t tn, uint8_t ss, int activate);
