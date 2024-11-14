/*
 * Themyscira Wireless jitter buffer implementation: main body.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <osmocom/core/linuxlist.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/talloc.h>
#include <osmocom/core/utils.h>

#include <themwi/rtp/twjit.h>

void twrtp_jibuf_init_defaults(struct twrtp_jibuf_config *config)
{
	memset(config, 0, sizeof(struct twrtp_jibuf_config));
	config->bd_start = 2;	/* smallest allowed */
	config->bd_hiwat = 3;	/* Nstart+1 is practically-useful minimum */
	config->thinning_int = 17;	/* prime number, usually 340 ms */
	config->max_future_sec = 10;	/* 10 s is a long time for voice */
}

/* create and destroy functions */

struct twrtp_jibuf_inst *
twrtp_jibuf_create(void *ctx, struct twrtp_jibuf_config *config)
{
	struct twrtp_jibuf_inst *twjit;

	twjit = talloc_zero(ctx, struct twrtp_jibuf_inst);
	if (!twjit)
		return NULL;

	twjit->ext_config = config;
	twjit->state = TWJIT_STATE_EMPTY;
	INIT_LLIST_HEAD(&twjit->sb[0].queue);
	INIT_LLIST_HEAD(&twjit->sb[1].queue);
	/* default of 8 kHz clock, 20 ms quantum */
	twrtp_jibuf_set_ts_quant(twjit, 8, 20);

	return twjit;
}

void twrtp_jibuf_destroy(struct twrtp_jibuf_inst *twjit)
{
	msgb_queue_free(&twjit->sb[0].queue);
	msgb_queue_free(&twjit->sb[1].queue);
	talloc_free(twjit);
}

/* basic housekeeping */

void twrtp_jibuf_set_ts_quant(struct twrtp_jibuf_inst *twjit,
			      uint16_t clock_khz, uint16_t quantum_ms)
{
	twjit->ts_quantum = (uint32_t) quantum_ms * clock_khz;
	twjit->quanta_per_sec = 1000 / quantum_ms;
	twjit->ts_units_per_ms = clock_khz;
	twjit->ts_units_per_sec = (uint32_t) clock_khz * 1000;
	twjit->ns_to_ts_units = 1000000 / clock_khz;
}

void twrtp_jibuf_reset(struct twrtp_jibuf_inst *twjit)
{
	msgb_queue_free(&twjit->sb[0].queue);
	msgb_queue_free(&twjit->sb[1].queue);
	twjit->state = TWJIT_STATE_EMPTY;
	twjit->sb[0].depth = 0;
	twjit->sb[1].depth = 0;
	twjit->got_first_packet = false;
	memset(&twjit->stats, 0, sizeof(struct twrtp_jibuf_stats));
}
