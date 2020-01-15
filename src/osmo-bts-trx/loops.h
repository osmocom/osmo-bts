#ifndef _TRX_LOOPS_H
#define _TRX_LOOPS_H

#include <osmo-bts/scheduler.h>

/*
 * calibration of loops
 */

/*
 * loops api
 */

void trx_loop_amr_input(struct l1sched_trx *l1t, uint8_t chan_nr,
	struct l1sched_chan_state *chan_state,
	int n_errors, int n_bits_total);

void trx_loop_amr_set(struct l1sched_chan_state *chan_state, int loop);

#endif /* _TRX_LOOPS_H */
