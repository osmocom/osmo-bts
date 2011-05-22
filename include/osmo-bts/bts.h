#ifndef _BTS_H
#define _BTS_H

#define BTS_SI_NUM 23 /* MUAR match the entries in BTS_SI_LIST */

#define BTS_SI_LIST { \
	RSL_SYSTEM_INFO_8, \
	RSL_SYSTEM_INFO_1, \
	RSL_SYSTEM_INFO_2, \
	RSL_SYSTEM_INFO_3, \
	RSL_SYSTEM_INFO_4, \
	RSL_SYSTEM_INFO_5, \
	RSL_SYSTEM_INFO_6, \
	RSL_SYSTEM_INFO_7, \
	RSL_SYSTEM_INFO_16, \
	RSL_SYSTEM_INFO_17, \
	RSL_SYSTEM_INFO_2bis, \
	RSL_SYSTEM_INFO_2ter, \
	RSL_SYSTEM_INFO_5bis, \
	RSL_SYSTEM_INFO_5ter, \
	RSL_SYSTEM_INFO_10, \
	REL_EXT_MEAS_ORDER, \
	RSL_MEAS_INFO, \
	RSL_SYSTEM_INFO_13, \
	RSL_SYSTEM_INFO_2quater, \
	RSL_SYSTEM_INFO_9, \
	RSL_SYSTEM_INFO_18, \
	RSL_SYSTEM_INFO_19, \
	RSL_SYSTEM_INFO_20, \
}

#define BTS_SI_NAME char *bts_si_name[] = { \
	"RSL_SYSTEM_INFO_8", \
	"RSL_SYSTEM_INFO_1", \
	"RSL_SYSTEM_INFO_2", \
	"RSL_SYSTEM_INFO_3", \
	"RSL_SYSTEM_INFO_4", \
	"RSL_SYSTEM_INFO_5", \
	"RSL_SYSTEM_INFO_6", \
	"RSL_SYSTEM_INFO_7", \
	"RSL_SYSTEM_INFO_16", \
	"RSL_SYSTEM_INFO_17", \
	"RSL_SYSTEM_INFO_2bis", \
	"RSL_SYSTEM_INFO_2ter", \
	"RSL_SYSTEM_INFO_5bis", \
	"RSL_SYSTEM_INFO_5ter", \
	"RSL_SYSTEM_INFO_10", \
	"REL_EXT_MEAS_ORDER", \
	"RSL_MEAS_INFO", \
	"RSL_SYSTEM_INFO_13", \
	"RSL_SYSTEM_INFO_2quater", \
	"RSL_SYSTEM_INFO_9", \
	"RSL_SYSTEM_INFO_18", \
	"RSL_SYSTEM_INFO_19", \
	"RSL_SYSTEM_INFO_20", \
}

#define BTS_SI_USE	1
#define BTS_SI_NEW	2

/* store sysinfos of a BTS */
struct osmobts_sysinfo {
	uint8_t			flags[BTS_SI_NUM];
	uint8_t			si[BTS_SI_NUM][23];
	struct osmo_timer_list	timer;
};

struct osmobts_slot;

/* one physical radio */
struct osmobts_ms {
	struct llist_head	entry;
	struct osmobts_trx	*trx;
};

/* one logical channel instance */
struct osmobts_lchan {
	struct osmobts_slot	*slot;
	uint8_t			lchan_nr;
	uint8_t			chan_nr; /* CBITS+TN */
	struct lapdm_channel	lapdm_channel;
	struct osmobts_rtp	rtp;
};

/* one timeslot instance */
struct osmobts_slot {
	struct osmobts_trx	*trx;
	uint8_t			slot_nr;
	uint8_t			acch_type; /* TS 08.58 9.3.1 (bits 8..4) */
	uint8_t			has_bcch;
	uint8_t			chan_comb;
	struct osmobts_lchan	*lchan[8];
	struct osmobts_ms	*tx_ms, *rx_ms;
};

/* one TRX instance */
struct osmobts_trx {
	struct osmocom_bts	*bts;
	uint8_t			trx_nr;
	struct osmobts_slot	slot[8];
	struct llist_head	ms_list;
	struct ipabis_link	link;
	struct osmobts_sysinfo	si;
	uint8_t			rf_red;
	uint16_t		arfcn_list[128];
	int			arfcn_num;
};

/* the BTS instance */
struct osmocom_bts {
	char *id;
	uint8_t num_trx;
	struct osmobts_trx	*trx[8];
	struct ipabis_link	link;
	uint8_t			max_ta;
	uint16_t		bcch_arfcn;
	uint8_t			bcc, ncc;
	uint16_t		start_time;
};

struct osmocom_bts *create_bts(uint8_t num_trx, char *id);
int create_ms(struct osmobts_trx *trx, int maskc, uint8_t *maskv_tx,
	uint8_t *maskv_rx);
void destroy_bts(struct osmocom_bts *bts);
int work_bts(struct osmocom_bts *bts);
int bts_link_estab(struct osmocom_bts *bts);
int trx_link_estab(struct osmobts_trx *trx);
void bts_new_si(void *arg);
void bts_setup_slot(struct osmobts_slot *slot, uint8_t comb);

#endif /* _BTS_H */

