/*
 * Themyscira Wireless jitter buffer implementation: definition of
 * structures and API functions.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include <osmocom/core/linuxlist.h>
#include <osmocom/core/timer.h>

/*
 * twjit configuration tunings, usually set via vty.
 */
struct twrtp_jibuf_config {
	/* buffer depth: starting minimum and high watermark */
	uint16_t bd_start;
	uint16_t bd_hiwat;
	/* interval for thinning of too-deep standing queue */
	uint16_t thinning_int;
	/* guard against time traveler RTP packets */
	uint16_t max_future_sec;
	/* min and max time delta in starting state, 0 means not set */
	uint16_t start_min_delta;
	uint16_t start_max_delta;
};

/*
 * Stats collected during the lifetime of a twjit instance.
 */
struct twrtp_jibuf_stats {
	/* normal operation */
	uint32_t rx_packets;
	uint32_t delivered_pkt;
	uint32_t handovers_in;
	uint32_t handovers_out;
	/* undesirable, but not totally unexpected */
	uint32_t too_old;
	uint32_t underruns;
	uint32_t ho_underruns;
	uint32_t output_gaps;
	uint32_t thinning_drops;
	/* unusual error events */
	uint32_t bad_packets;
	uint32_t duplicate_ts;
	/* independent analysis of Rx packet stream */
	uint32_t ssrc_changes;
	uint32_t seq_skips;
	uint32_t seq_backwards;
	uint32_t seq_repeats;
	uint32_t intentional_gaps;
	uint32_t ts_resets;
	uint32_t jitter_max;
};

/*
 * Info collected from the incoming RTP data stream
 * for the purpose of generating RTCP reception report blocks.
 */
struct twrtp_jibuf_rr_info {
	uint32_t rx_packets;
	uint32_t base_seq;
	uint32_t max_seq_ext;
	uint32_t expected_pkt;
	uint32_t jitter_accum;
};

/*
 * Each twjit instance has two sub-buffers; each subbuf is a queue of
 * received RTP packets that have the same SSRC and whose timestamps
 * increment in the expected cadence, with each ts delta being an
 * integral multiple of the samples-per-quantum constant.
 */
struct twrtp_jibuf_sub {
	uint32_t ssrc;
	uint32_t head_ts;
	struct llist_head queue;
	uint32_t depth;
	uint32_t delta_ms;		/* used only in starting state */
	/* thinning mechanism */
	uint16_t drop_int_count;
	/* running config for this subbuf */
	struct twrtp_jibuf_config conf;
};

/*
 * Each twjit instance is in one of 4 fundamental states at any moment,
 * as enumerated here.
 */
enum twrtp_jibuf_state {
	TWJIT_STATE_EMPTY,
	TWJIT_STATE_HUNT,
	TWJIT_STATE_FLOWING,
	TWJIT_STATE_HANDOVER,
};

/* Main structure for one instance of twjit */
struct twrtp_jibuf_inst {
	/* pointer to config structure given to twrtp_jibuf_create(),
	 * memory must remain valid, but content can change at any time. */
	struct twrtp_jibuf_config *ext_config;
	/* count of RTP timestamp units per quantum */
	uint32_t ts_quantum;
	/* quanta per second, used to scale max_future_sec */
	uint16_t quanta_per_sec;
	/* scaling factors for time delta conversions */
	uint16_t ts_units_per_ms;
	uint32_t ts_units_per_sec;
	uint32_t ns_to_ts_units;
	/* operational state */
	enum twrtp_jibuf_state state;
	struct twrtp_jibuf_sub sb[2];
	uint8_t read_sb;	/* 0 or 1 */
	uint8_t write_sb;	/* ditto */
	/* info about the most recent Rx packet */
	uint32_t last_ssrc;
	uint32_t last_ts;
	uint16_t last_seq;
	bool got_first_packet;
	struct timespec last_arrival;
	uint32_t last_arrival_delta;
	/* analytics for RTCP RR */
	struct twrtp_jibuf_rr_info rr_info;
	/* stats over lifetime of this instance */
	struct twrtp_jibuf_stats stats;
};

/* twjit module API functions */

struct twrtp_jibuf_inst *
twrtp_jibuf_create(void *ctx, struct twrtp_jibuf_config *config);

/*
 * The default timescale is 8 kHz (8000 samples per second),
 * and the default quantum duration (packetization interval) is 20 ms.
 * The following function can be used to change these fundamental
 * parameters; it may be called only right after twrtp_jibuf_create(),
 * before any packets are fed to twrtp_jibuf_input().
 */
void twrtp_jibuf_set_ts_quant(struct twrtp_jibuf_inst *twjit,
			      uint16_t clock_khz, uint16_t quantum_ms);

void twrtp_jibuf_destroy(struct twrtp_jibuf_inst *twjit);

void twrtp_jibuf_reset(struct twrtp_jibuf_inst *twjit);

struct msgb;

/* RTP input, takes ownership of msgb */
void twrtp_jibuf_input(struct twrtp_jibuf_inst *twjit, struct msgb *msg);

/* output function, to be called by TDM/GSM/etc fixed-timing side */
struct msgb *twrtp_jibuf_output(struct twrtp_jibuf_inst *twjit);

/* vty configuration functions */

void twrtp_jibuf_init_defaults(struct twrtp_jibuf_config *config);

void twrtp_jibuf_vty_init(int twjit_node);

struct vty;

int twrtp_jibuf_config_write(struct vty *vty,
			     const struct twrtp_jibuf_config *conf,
			     const char *name, const char *prefix);
