/* Scheduler worker functions for OsmoBTS-TRX */

/* (C) 2013 by Andreas Eversberg <jolly@eversberg.eu>
 * (C) 2015 by Alexander Chemeris <Alexander.Chemeris@fairwaves.co>
 * (C) 2015-2017 by Harald Welte <laforge@gnumonks.org>
 *
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#include <stdlib.h>
#include <unistd.h>
#include <limits.h>
#include <errno.h>
#include <stdint.h>
#include <ctype.h>
#include <inttypes.h>
#include <sys/timerfd.h>

#include <osmocom/core/msgb.h>
#include <osmocom/core/talloc.h>
#include <osmocom/core/timer_compat.h>
#include <osmocom/core/bits.h>
#include <osmocom/gsm/a5.h>


#include <osmo-bts/gsm_data.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/rsl.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/l1sap.h>
#include <osmo-bts/scheduler.h>
#include <osmo-bts/scheduler_backend.h>

#include "l1_if.h"
#include "trx_if.h"

/* an IDLE burst returns nothing. on C0 it is replaced by dummy burst */
int tx_idle_fn(struct l1sched_trx *l1t, enum trx_chan_type chan,
	       uint8_t bid, struct trx_dl_burst_req *br)
{
	LOGL1S(DL1P, LOGL_DEBUG, l1t, br->tn, chan, br->fn, "Transmitting IDLE\n");
	return 0;
}

/* schedule all frames of all TRX for given FN */
static void trx_sched_fn(struct gsm_bts *bts, const uint32_t fn)
{
	struct trx_dl_burst_req br;
	struct gsm_bts_trx *trx;
	uint8_t c0_mask = 0x00;
	uint32_t sched_fn;
	uint8_t tn;

	/* send time indication */
	l1if_mph_time_ind(bts, fn);

	/* process every TRX */
	llist_for_each_entry(trx, &bts->trx_list, list) {
		struct phy_instance *pinst = trx_phy_instance(trx);
		struct phy_link *plink = pinst->phy_link;
		struct trx_l1h *l1h = pinst->u.osmotrx.hdl;
		struct l1sched_trx *l1t = &l1h->l1s;

		/* we don't schedule, if power is off */
		if (!trx_if_powered(l1h))
			continue;

		/* advance frame number, so the transceiver has more
		 * time until it must be transmitted. */
		sched_fn = GSM_TDMA_FN_SUM(fn, plink->u.osmotrx.clock_advance);

		/* process every TS of TRX */
		for (tn = 0; tn < ARRAY_SIZE(l1t->ts); tn++) {
			/* ready-to-send */
			_sched_rts(l1t, tn, GSM_TDMA_FN_SUM(sched_fn, plink->u.osmotrx.rts_advance));

			/* All other parameters to be set by _sched_dl_burst() */
			br = (struct trx_dl_burst_req) {
				.fn = sched_fn,
				.tn = tn,
			};

			/* get burst for FN */
			_sched_dl_burst(l1t, &br);
			if (br.burst_len == 0) {
				/* if no bits, send no burst */
				continue;
			}

			/* update dummy burst mask for C0 */
			if (trx == bts->c0)
				c0_mask |= (1 << tn);

			trx_if_send_burst(l1h, &br);
		}
	}

	/* send dummy bursts on inactive timeslots of C0 */
	struct phy_instance *pinst = trx_phy_instance(bts->c0);
	struct trx_l1h *l1h = pinst->u.osmotrx.hdl;
	struct phy_link *plink = pinst->phy_link;

	br = (struct trx_dl_burst_req) {
		.fn = GSM_TDMA_FN_SUM(fn, plink->u.osmotrx.clock_advance),
		.burst_len = GSM_BURST_LEN,
	};

	memcpy(br.burst, _sched_dummy_burst, GSM_BURST_LEN);

	for (br.tn = 0; br.tn < TRX_NR_TS; br.tn++) {
		if (c0_mask & (1 << br.tn))
			continue;
		trx_if_send_burst(l1h, &br);
	}
}

/*! maximum number of 'missed' frame periods we can tolerate of OS doesn't schedule us*/
#define MAX_FN_SKEW		50
/*! maximum number of frame periods we can tolerate without TRX Clock Indication*/
#define TRX_LOSS_FRAMES		400

/*! compute the number of micro-seconds difference elapsed between \a last and \a now */
static inline int64_t compute_elapsed_us(const struct timespec *last, const struct timespec *now)
{
	struct timespec elapsed;

	timespecsub(now, last, &elapsed);
	return (int64_t)(elapsed.tv_sec * 1000000) + (elapsed.tv_nsec / 1000);
}

/*! compute the number of frame number intervals elapsed between \a last and \a now */
static inline int compute_elapsed_fn(const uint32_t last, const uint32_t now)
{
	int elapsed_fn = GSM_TDMA_FN_SUB(now, last);
	if (elapsed_fn >= 135774)
		elapsed_fn -= GSM_TDMA_HYPERFRAME;
	return elapsed_fn;
}

/*! normalise given 'struct timespec', i.e. carry nanoseconds into seconds */
static inline void normalize_timespec(struct timespec *ts)
{
	ts->tv_sec += ts->tv_nsec / 1000000000;
	ts->tv_nsec = ts->tv_nsec % 1000000000;
}

/*! this is the timerfd-callback firing for every FN to be processed */
static int trx_fn_timer_cb(struct osmo_fd *ofd, unsigned int what)
{
	struct gsm_bts *bts = ofd->data;
	struct bts_trx_priv *bts_trx = (struct bts_trx_priv *)bts->model_priv;
	struct osmo_trx_clock_state *tcs = &bts_trx->clk_s;
	struct timespec tv_now;
	uint64_t expire_count;
	int64_t elapsed_us, error_us;
	int rc, i;

	if (!(what & OSMO_FD_READ))
		return 0;

	/* read from timerfd: number of expirations of periodic timer */
	rc = read(ofd->fd, (void *) &expire_count, sizeof(expire_count));
	if (rc < 0 && errno == EAGAIN)
		return 0;
	OSMO_ASSERT(rc == sizeof(expire_count));

	if (expire_count > 1) {
		LOGP(DL1C, LOGL_NOTICE, "FN timer expire_count=%"PRIu64": We missed %"PRIu64" timers\n",
		     expire_count, expire_count - 1);
		rate_ctr_add(&bts_trx->ctrs->ctr[BTSTRX_CTR_SCHED_DL_MISS_FN], expire_count - 1);
	}

	/* check if transceiver is still alive */
	if (tcs->fn_without_clock_ind++ == TRX_LOSS_FRAMES) {
		LOGP(DL1C, LOGL_NOTICE, "No more clock from transceiver\n");
		goto no_clock;
	}

	/* compute actual elapsed time and resulting OS scheduling error */
	clock_gettime(CLOCK_MONOTONIC, &tv_now);
	elapsed_us = compute_elapsed_us(&tcs->last_fn_timer.tv, &tv_now);
	error_us = elapsed_us - GSM_TDMA_FN_DURATION_uS;
#ifdef DEBUG_CLOCK
	printf("%s(): %09ld, elapsed_us=%05" PRId64 ", error_us=%-d: fn=%d\n", __func__,
		tv_now.tv_nsec, elapsed_us, error_us, tcs->last_fn_timer.fn+1);
#endif
	tcs->last_fn_timer.tv = tv_now;

	/* if someone played with clock, or if the process stalled */
	if (elapsed_us > GSM_TDMA_FN_DURATION_uS * MAX_FN_SKEW || elapsed_us < 0) {
		LOGP(DL1C, LOGL_ERROR, "PC clock skew: elapsed_us=%" PRId64 ", error_us=%" PRId64 "\n",
			elapsed_us, error_us);
		goto no_clock;
	}

	/* call trx_sched_fn() for all expired FN */
	for (i = 0; i < expire_count; i++)
		trx_sched_fn(bts, GSM_TDMA_FN_INC(tcs->last_fn_timer.fn));

	return 0;

no_clock:
	osmo_timerfd_disable(&tcs->fn_timer_ofd);
	bts_shutdown(bts, "No clock from osmo-trx");
	return -1;
}

/*! \brief This is the cb of the initial timer set upon start. On timeout, it
 *  means it wasn't replaced and hence no CLOCK IND was received. */
static int trx_start_noclockind_to_cb(struct osmo_fd *ofd, unsigned int what)
{
	struct gsm_bts *bts = ofd->data;
	struct bts_trx_priv *bts_trx = (struct bts_trx_priv *)bts->model_priv;
	struct osmo_trx_clock_state *tcs = &bts_trx->clk_s;

	osmo_fd_close(&tcs->fn_timer_ofd); /* Avoid being called again */
	bts_shutdown(bts, "No clock since TRX was started");
	return -1;
}

/*! \brief PHY informs us clock indications should start to be received */
int trx_sched_clock_started(struct gsm_bts *bts)
{
	struct bts_trx_priv *bts_trx = (struct bts_trx_priv *)bts->model_priv;
	struct osmo_trx_clock_state *tcs = &bts_trx->clk_s;
	const struct timespec it_val = {3, 0};
	const struct timespec it_intval = {0, 0};

	LOGP(DL1C, LOGL_NOTICE, "GSM clock started, waiting for clock indications\n");
	osmo_fd_close(&tcs->fn_timer_ofd);
	memset(tcs, 0, sizeof(*tcs));
	tcs->fn_timer_ofd.fd = -1;
	/* Set up timeout to shutdown BTS if no clock ind is received in a few
	 * seconds. Upon clock ind receival, fn_timer_ofd will be reused and
	 * timeout won't trigger.
	 */
	osmo_timerfd_setup(&tcs->fn_timer_ofd, trx_start_noclockind_to_cb, bts);
	osmo_timerfd_schedule(&tcs->fn_timer_ofd, &it_val, &it_intval);
	return 0;
}

/*! \brief PHY informs us no more clock indications should be received anymore */
int trx_sched_clock_stopped(struct gsm_bts *bts)
{
	struct bts_trx_priv *bts_trx = (struct bts_trx_priv *)bts->model_priv;
	struct osmo_trx_clock_state *tcs = &bts_trx->clk_s;

	LOGP(DL1C, LOGL_NOTICE, "GSM clock stopped\n");
	osmo_fd_close(&tcs->fn_timer_ofd);

	return 0;
}

/*! reset clock with current fn and schedule it. Called when trx becomes
 *  available or when max clock skew is reached */
static int trx_setup_clock(struct gsm_bts *bts, struct osmo_trx_clock_state *tcs,
	struct timespec *tv_now, const struct timespec *interval, uint32_t fn)
{
	tcs->last_fn_timer.fn = fn;
	/* call trx cheduler function for new 'last' FN */
	trx_sched_fn(bts, tcs->last_fn_timer.fn);

	/* schedule first FN clock timer */
	osmo_timerfd_setup(&tcs->fn_timer_ofd, trx_fn_timer_cb, bts);
	osmo_timerfd_schedule(&tcs->fn_timer_ofd, NULL, interval);

	tcs->last_fn_timer.tv = *tv_now;
	tcs->last_clk_ind.tv = *tv_now;
	tcs->last_clk_ind.fn = fn;

	return 0;
}

/*! called every time we receive a clock indication from TRX */
int trx_sched_clock(struct gsm_bts *bts, uint32_t fn)
{
	struct bts_trx_priv *bts_trx = (struct bts_trx_priv *)bts->model_priv;
	struct osmo_trx_clock_state *tcs = &bts_trx->clk_s;
	struct timespec tv_now;
	int elapsed_fn;
	int64_t elapsed_us, elapsed_us_since_clk, elapsed_fn_since_clk, error_us_since_clk;
	unsigned int fn_caught_up = 0;
	const struct timespec interval = { .tv_sec = 0, .tv_nsec = GSM_TDMA_FN_DURATION_nS };

	/* reset lost counter */
	tcs->fn_without_clock_ind = 0;

	clock_gettime(CLOCK_MONOTONIC, &tv_now);

	/* calculate elapsed time +fn since last timer */
	elapsed_us = compute_elapsed_us(&tcs->last_fn_timer.tv, &tv_now);
	elapsed_fn = compute_elapsed_fn(tcs->last_fn_timer.fn, fn);
#ifdef DEBUG_CLOCK
	printf("%s(): LAST_TIMER %9ld, elapsed_us=%7d, elapsed_fn=%+3d\n", __func__,
		tv_now.tv_nsec, elapsed_us, elapsed_fn);
#endif
	/* negative elapsed_fn values mean that we've already processed
	 * more FN based on the local interval timer than what the TRX
	 * now reports in the clock indication.   Positive elapsed_fn
	 * values mean we still have a backlog to process */

	/* calculate elapsed time +fn since last clk ind */
	elapsed_us_since_clk = compute_elapsed_us(&tcs->last_clk_ind.tv, &tv_now);
	elapsed_fn_since_clk = compute_elapsed_fn(tcs->last_clk_ind.fn, fn);
	/* error (delta) between local clock since last CLK and CLK based on FN clock at TRX */
	error_us_since_clk = elapsed_us_since_clk - (GSM_TDMA_FN_DURATION_uS * elapsed_fn_since_clk);
	LOGP(DL1C, LOGL_INFO, "TRX Clock Ind: elapsed_us=%7"PRId64", "
		"elapsed_fn=%3"PRId64", error_us=%+5"PRId64"\n",
		elapsed_us_since_clk, elapsed_fn_since_clk, error_us_since_clk);

	/* TODO: put this computed error_us_since_clk into some filter
	 * function and use that to adjust our regular timer interval to
	 * compensate for clock drift between the PC clock and the
	 * TRX/SDR clock */

	tcs->last_clk_ind.tv = tv_now;
	tcs->last_clk_ind.fn = fn;

	/* check for max clock skew */
	if (elapsed_fn > MAX_FN_SKEW || elapsed_fn < -MAX_FN_SKEW) {
		LOGP(DL1C, LOGL_NOTICE, "GSM clock skew: old fn=%u, "
			"new fn=%u\n", tcs->last_fn_timer.fn, fn);
		return trx_setup_clock(bts, tcs, &tv_now, &interval, fn);
	}

	LOGP(DL1C, LOGL_INFO, "GSM clock jitter: %" PRId64 "us (elapsed_fn=%d)\n",
		elapsed_fn * GSM_TDMA_FN_DURATION_uS - elapsed_us, elapsed_fn);

	/* too many frames have been processed already */
	if (elapsed_fn < 0) {
		struct timespec first = interval;
		/* set clock to the time or last FN should have been
		 * transmitted. */
		first.tv_nsec += (0 - elapsed_fn) * GSM_TDMA_FN_DURATION_nS;
		normalize_timespec(&first);
		LOGP(DL1C, LOGL_NOTICE, "We were %d FN faster than TRX, compensating\n", -elapsed_fn);
		/* set time to the time our next FN has to be transmitted */
		osmo_timerfd_schedule(&tcs->fn_timer_ofd, &first, &interval);
		return 0;
	}

	/* transmit what we still need to transmit */
	while (fn != tcs->last_fn_timer.fn) {
		trx_sched_fn(bts, GSM_TDMA_FN_INC(tcs->last_fn_timer.fn));
		fn_caught_up++;
	}

	if (fn_caught_up) {
		LOGP(DL1C, LOGL_NOTICE, "We were %d FN slower than TRX, compensated\n", elapsed_fn);
		tcs->last_fn_timer.tv = tv_now;
	}

	return 0;
}

void _sched_act_rach_det(struct l1sched_trx *l1t, uint8_t tn, uint8_t ss, int activate)
{
	struct phy_instance *pinst = trx_phy_instance(l1t->trx);
	struct trx_l1h *l1h = pinst->u.osmotrx.hdl;

	if (activate)
		trx_if_cmd_handover(l1h, tn, ss);
	else
		trx_if_cmd_nohandover(l1h, tn, ss);
}
