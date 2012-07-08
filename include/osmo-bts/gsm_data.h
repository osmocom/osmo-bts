#ifndef _GSM_DATA_H
#define _GSM_DATA_H

#include <osmocom/core/timer.h>
#include <osmocom/core/linuxlist.h>
#include <osmocom/gsm/lapdm.h>

#include <osmo-bts/paging.h>

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
	struct llist_head agch_queue;
	struct paging_state *paging_state;
	char *bsc_oml_host;
	char *rtp_bind_host;
	unsigned int rtp_jitter_buf_ms;
	struct {
		uint8_t ciphers;
	} support;
	struct {
		uint8_t tc4_ctr;
	} si;
};

enum lchan_ciph_state {
	LCHAN_CIPH_NONE,
	LCHAN_CIPH_RX_REQ,
	LCHAN_CIPH_RX_CONF,
	LCHAN_CIPH_TXRX_REQ,
	LCHAN_CIPH_TXRX_CONF,
};

#define bts_role_bts(x)	((struct gsm_bts_role_bts *)(x)->role)

#include "../../openbsc/openbsc/include/openbsc/gsm_data_shared.h"

struct femtol1_hdl;

static inline struct femtol1_hdl *trx_femtol1_hdl(struct gsm_bts_trx *trx)
{
	return trx->role_bts.l1h;
}


void lchan_set_state(struct gsm_lchan *lchan, enum gsm_lchan_state state);

#endif /* _GSM_DATA_H */
