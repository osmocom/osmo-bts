#pragma once

#include <osmocom/core/linuxlist.h>
#include <osmocom/gsm/gsm23003.h>

#include <osmo-bts/oml.h>

struct pcu_sock_state;

/* BTS Site Manager */
struct gsm_bts_sm {
	struct gsm_abis_mo mo;
	struct llist_head bts_list;
	unsigned int num_bts;
	struct osmo_plmn_id plmn;
	struct pcu_sock_state *pcu_state;
};

extern struct gsm_bts_sm *g_bts_sm;

struct gsm_bts_sm *gsm_bts_sm_alloc(void *talloc_ctx);
