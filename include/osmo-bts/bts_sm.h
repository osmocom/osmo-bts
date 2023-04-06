#pragma once

#include <osmocom/core/linuxlist.h>
#include <osmocom/core/socket.h>
#include <osmocom/gsm/gsm23003.h>

#include <osmo-bts/oml.h>

struct pcu_sock_state;

/* GPRS NSVC; ip.access specific NM Object */
struct gsm_gprs_nse;
struct gsm_gprs_nsvc {
	struct gsm_abis_mo mo;
	struct gsm_gprs_nse *nse;
	/* data read via VTY config file, to configure the BTS
	 * via OML from BSC */
	int id;
	uint16_t nsvci;
	struct osmo_sockaddr local;	/* on the BTS */
	struct osmo_sockaddr remote;	/* on the SGSN */
};

/* GPRS NSE; ip.access specific NM Object */
struct gsm_gprs_nse {
	struct gsm_abis_mo mo;
	uint16_t nsei;
	uint8_t timer[7];
	struct gsm_gprs_nsvc nsvc[2];
};

struct gsm_bts *gsm_gprs_nse_get_bts(const struct gsm_gprs_nse *nse);

/* BTS Site Manager */
struct gsm_bts_sm {
	struct gsm_abis_mo mo;
	struct llist_head bts_list;
	unsigned int num_bts;
	struct osmo_plmn_id plmn;
	struct pcu_sock_state *pcu_state;
	struct {
		struct gsm_gprs_nse nse;
	} gprs;
};

extern struct gsm_bts_sm *g_bts_sm;

struct gsm_bts_sm *gsm_bts_sm_alloc(void *talloc_ctx);
