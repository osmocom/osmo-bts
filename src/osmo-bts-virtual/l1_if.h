#pragma once

#include <osmo-bts/gsm_data.h>
#include <osmo-bts/scheduler.h>

#include "virtual_um.h"

struct vbts_l1h {
	struct gsm_bts_trx	*trx;
	struct l1sched_trx	l1s;
	struct virt_um_inst 	*virt_um;
};

struct vbts_l1h *l1if_open(struct gsm_bts_trx *trx);
void l1if_close(struct vbts_l1h *l1h);
void l1if_reset(struct vbts_l1h *l1h);

int vbts_sched_start(struct gsm_bts *bts);
