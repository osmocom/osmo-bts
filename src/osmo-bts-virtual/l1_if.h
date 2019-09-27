#pragma once

#include <osmo-bts/gsm_data.h>
#include <osmo-bts/scheduler.h>

#include "virtual_um.h"

/* gsm_bts->model_priv, specific to osmo-bts-virtual */
struct bts_virt_priv {
	uint32_t last_fn;
	struct timeval tv_clock;
	struct osmo_timer_list fn_timer;
};

struct vbts_l1h {
	struct gsm_bts_trx	*trx;
	struct l1sched_trx	l1s;
	struct virt_um_inst	*virt_um;
};

struct vbts_l1h *l1if_open(struct gsm_bts_trx *trx);
void l1if_close(struct vbts_l1h *l1h);
void l1if_reset(struct vbts_l1h *l1h);

int l1if_mph_time_ind(struct gsm_bts *bts, uint32_t fn);

int vbts_sched_start(struct gsm_bts *bts);
