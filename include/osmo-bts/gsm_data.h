#ifndef _GSM_DATA_H
#define _GSM_DATA_H

#include <osmocom/core/timer.h>
#include <osmocom/core/linuxlist.h>
#include <osmocom/gsm/lapdm.h>

#include <osmo-bts/paging.h>

#define GSM_BTS_AGCH_QUEUE_THRESH_LEVEL_DEFAULT 41
#define GSM_BTS_AGCH_QUEUE_THRESH_LEVEL_DISABLE 999999
#define GSM_BTS_AGCH_QUEUE_LOW_LEVEL_DEFAULT 41
#define GSM_BTS_AGCH_QUEUE_HIGH_LEVEL_DEFAULT 91

struct pcu_sock_state;

struct gsm_network {
	struct llist_head bts_list;
	unsigned int num_bts;
	uint16_t mcc, mnc;
	struct pcu_sock_state *pcu_state;
};

/* data structure for BTS related data specific to the BTS role */
struct gsm_bts_role_bts {
	struct {
		/* Interference Boundaries for OML */
		int16_t boundary[6];
		uint8_t intave;
	} interference;
	unsigned int t200_ms[7];
	unsigned int t3105_ms;
	struct {
		uint8_t overload_period;
		struct {
			/* Input parameters from OML */
			uint8_t load_ind_thresh;	/* percent */
			uint8_t load_ind_period;	/* seconds */
			/* Internal data */
			struct osmo_timer_list timer;
			unsigned int pch_total;
			unsigned int pch_used;
		} ccch;
		struct {
			/* Input parameters from OML */
			int16_t busy_thresh;		/* in dBm */
			uint16_t averaging_slots;
			/* Internal data */
			unsigned int total;	/* total nr */
			unsigned int busy;	/* above busy_thresh */
			unsigned int access;	/* access bursts */
		} rach;
	} load;
	uint8_t ny1;
	uint8_t max_ta;

	/* AGCH queuing */
	struct llist_head agch_queue;
	int agch_queue_length;
	int agch_max_queue_length;

	int agch_queue_thresh_level;	/* Cleanup threshold in percent of max len */
	int agch_queue_low_level;	/* Low water mark in percent of max len */
	int agch_queue_high_level;	/* High water mark in percent of max len */

	/* TODO: Use a rate counter group instead */
	uint64_t agch_queue_dropped_msgs;
	uint64_t agch_queue_merged_msgs;
	uint64_t agch_queue_rejected_msgs;
	uint64_t agch_queue_agch_msgs;
	uint64_t agch_queue_pch_msgs;

	struct paging_state *paging_state;
	char *bsc_oml_host;
	unsigned int rtp_jitter_buf_ms;
	struct {
		uint8_t ciphers;	/* flags A5/1==0x1, A5/2==0x2, A5/3==0x4 */
	} support;
	struct {
		uint8_t tc4_ctr;
	} si;
	struct gsm_time gsm_time;
	uint8_t radio_link_timeout;

	/* used by the sysmoBTS to adjust band */
	uint8_t auto_band;
};

enum lchan_ciph_state {
	LCHAN_CIPH_NONE,
	LCHAN_CIPH_RX_REQ,
	LCHAN_CIPH_RX_CONF,
	LCHAN_CIPH_RXTX_REQ,
	LCHAN_CIPH_RX_CONF_TX_REQ,
	LCHAN_CIPH_RXTX_CONF,
};

#define bts_role_bts(x)	((struct gsm_bts_role_bts *)(x)->role)

#include "openbsc/gsm_data_shared.h"

struct femtol1_hdl;

static inline struct femtol1_hdl *trx_femtol1_hdl(struct gsm_bts_trx *trx)
{
	return trx->role_bts.l1h;
}


void lchan_set_state(struct gsm_lchan *lchan, enum gsm_lchan_state state);

/* cipher code */
#define CIPHER_A5(x) (1 << (x-1))

int bts_supports_cipher(struct gsm_bts_role_bts *bts, int rsl_cipher);


#endif /* _GSM_DATA_H */
