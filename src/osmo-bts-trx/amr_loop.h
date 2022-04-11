#pragma once

#include <osmo-bts/scheduler.h>

/*
 * calibration of loops
 */

/*
 * loops api
 */

void trx_loop_amr_input(struct l1sched_chan_state *chan_state,
			int n_errors, int n_bits_total);

void trx_loop_amr_set(struct l1sched_chan_state *chan_state, int loop);
