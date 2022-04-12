#pragma once

#include <osmo-bts/scheduler.h>

/*
 * calibration of loops
 */

/*
 * loops api
 */

void trx_loop_amr_input(struct l1sched_chan_state *chan_state,
			const struct l1sched_meas_set *meas_set);

void trx_loop_amr_set(struct l1sched_chan_state *chan_state, int loop);
