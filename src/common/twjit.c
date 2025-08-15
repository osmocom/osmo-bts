/*
 * Themyscira Wireless RTP jitter buffer implementation: main body.
 *
 * This code was contributed to Osmocom Cellular Network Infrastructure
 * project by Mother Mychaela N. Falconia of Themyscira Wireless.
 * Mother Mychaela's contributions are NOT subject to copyright:
 * no rights reserved, all rights relinquished.
 *
 * There also exists a detailed document titled _Guide to ThemWi RTP
 * endpoint library_, located here:
 * https://www.freecalypso.org/TW-doc/twrtp-guide-latest.pdf
 * (See TW-doc directory listing for other formats and previous versions.)
 * This document is required reading for anyone seeking to properly
 * understand this code, and thus be able to modify it without breaking
 * it.  Specific section references to this document will be made
 * in subsequent comments.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <arpa/inet.h>	/* for network byte order functions */

#include <osmocom/core/linuxlist.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/talloc.h>
#include <osmocom/core/timer.h>
#include <osmocom/core/utils.h>

#include <osmocom/netif/rtp.h>

#include <twrtp-local/twjit.h>

/*! \addgroup twjit
 *  @{
 */

/*! \cond private */

/*! Each twjit instance has two sub-buffers; each subbuf is a queue of
 *  received RTP packets that have the same SSRC and whose timestamps
 *  increment in the expected cadence, with each ts delta being an
 *  integral multiple of the samples-per-quantum constant.
 *  See document section 2.3.2.
 */
struct twjit_subbuf {
	/*! SSRC of the stream portion captured in this subbuf */
	uint32_t ssrc;
	/*! Current timestamp at the head of the queue */
	uint32_t head_ts;
	/*! Queue of packets held by this subbuf */
	struct llist_head queue;
	/*! Current depth as defined in document section 2.3.2 */
	uint32_t depth;
	/*! Time delta in ms between arrival times of the two most recently
	 *  received packets, used only in starting state */
	uint32_t delta_ms;
	/*! thinning mechanism: countdown before next quantum deletion */
	uint16_t drop_int_count;
};

/*! Each twjit instance is in one of 4 fundamental states at any moment,
 *  as enumerated here.  See document section 2.3.1 for state transition
 *  diagram.
 */
enum twjit_state {
	/*! completely empty: neither subbuf is valid */
	TWJIT_STATE_EMPTY,
	/*! one subbuf is non-empty, but it hasn't started flowing out yet */
	TWJIT_STATE_HUNT,
	/*! one subbuf is both flowing out and accepting new packets */
	TWJIT_STATE_FLOWING,
	/*! one subbuf is flowing out while another receives new packets */
	TWJIT_STATE_HANDOVER,
};

/*! Main structure for one instance of twjit */
struct obts_twjit {
	/*! twjit tuning config, can be changed with obts_twjit_set_config() */
	struct obts_twjit_config config;
	/*! count of RTP timestamp units per quantum */
	uint32_t ts_quantum;
	/*! quanta per second, used to scale max_future_sec */
	uint16_t quanta_per_sec;
	/* scaling factors for time delta conversions */
	/*! RTP timestamp units per millisecond */
	uint16_t ts_units_per_ms;
	/*! RTP timestamp units per second */
	uint32_t ts_units_per_sec;
	/*! divisor to go from nanoseconds to RTP timestamp units */
	uint32_t ns_to_ts_units;
	/*! current fundamental state of this twjit instance */
	enum twjit_state state;
	/*! the two sub-buffers */
	struct twjit_subbuf sb[2];
	/*! current subbuf being read, 0 or 1 */
	uint8_t read_sb;
	/*! current subbuf being written, 0 or 1 */
	uint8_t write_sb;
	/*! RTP timestamp of the most recently received packet */
	uint32_t last_ts;
	/*! RTP sequence number of the most recently received packet */
	uint16_t last_seq;
	/*! Have we received at least one packet?  This bool serves as
	 *  the validity flag for last_ts, last_ts and rr_info. */
	bool got_first_packet;
	/*! CLOCK_MONOTONIC time of last packet arrival */
	struct timespec last_arrival;
	/*! Delta between the two most recent RTP packet arrival times,
	 *  converted to RTP timestamp units. */
	uint32_t last_arrival_delta;
	/*! analytics for RTCP RR, also remembers last SSRC */
	struct obts_twjit_rr_info rr_info;
	/*! stats over lifetime of this instance */
	struct obts_twjit_stats stats;
};

/*! \endcond */

/*! Initialize twjit config structure with defaults
 *
 * \param[out] config The structure to be filled
 *
 * A typical application will have a struct obts_twjit_config somewhere
 * in the application config data structures, editable via vty.
 * More complex applications may even have several such twjit config
 * structures, to be used in different contexts such as GSM vs PSTN.
 * On application startup prior to vty config file read, call the present
 * function to initialize your twjit config structure(s) with default values
 * for all parameters.
 */
void obts_twjit_config_init(struct obts_twjit_config *config)
{
	memset(config, 0, sizeof(struct obts_twjit_config));

	/* While the theoretical minimum starting fill level is 1, the
	 * practically useful minimum (achieving lowest latency, but not
	 * incurring underruns in normal healthy operation) is 2 for typical
	 * network configurations that combine elements with "perfect" 20 ms
	 * timing (T1/E1 interfaces, external IP-PSTN links, software
	 * transcoders timed by system clock etc) and GSM-to-IP OsmoBTS
	 * whose 20 ms timing contains the small inherent jitter of TDMA. */
	config->bd_start = 2;

	/* The high water mark setting determines when the standing queue
	 * thinning mechanism kicks in.  A standing queue that is longer
	 * than the starting fill level will occur when the flow starts
	 * during a network latency spike, but then the network latency
	 * goes down.  If this setting is too high, deep standing queues
	 * will persist, adding needless latency to speech or CSD.
	 * If this setting is too low, the thinning mechanism will be
	 * too invasive, needlessly and perhaps frequently deleting a quantum
	 * of speech or data from the stream and incurring a phase shift.
	 * Starting fill level plus 2 seems like a good default. */
	config->bd_hiwat = 4;

	/* When the standing queue thinning mechanism does kick in,
	 * it drops every Nth packet, where N is the thinning interval.
	 * Given that this mechanism forcibly deletes a quantum of speech
	 * or data from the stream, these induced disruptions should be
	 * spaced out, and the managing operator should also keep in mind
	 * that the incurred phase shift may be a problem for some
	 * applications, particularly CSD.  Our current default is
	 * a prime number, reducing the probability that the thinning
	 * mechanism will interfere badly with intrinsic features of the
	 * stream being thinned.  17 quantum units at 20 ms per quantum
	 * is 340 ms, which should be sufficiently long spacing to make
	 * speech quantum deletions tolerable. */
	config->thinning_int = 17;

	/* Guard against time traveler packets,
	 * see document section 2.3.4.3. */
	config->max_future_sec = 10;
}

/* create and destroy functions */

/*! \cond private */

/*! Validate twjit config structure passed by API users
 *
 * \param[in] conf User-provided config structure to be validated
 * \returns true if config is valid, false otherwise
 */
static bool config_is_valid(const struct obts_twjit_config *conf)
{
	if (conf->bd_start < 1)
		return false;
	if (conf->bd_hiwat < conf->bd_start)
		return false;
	if (conf->thinning_int < 2)
		return false;
	if (conf->max_future_sec < 1)
		return false;
	/* all checks passed */
	return true;
}

/*! \endcond */

/*! Create a twjit instance
 *
 * \param[in] ctx Parent talloc context under which struct obts_twjit
 * should be allocated.
 * \param[in] clock_khz RTP clock rate in kHz, i.e., number of RTP timestamp
 * units per millisecond. The most common value is 8.
 * \param[in] quantum_ms Duration of a single quantum (unit of speech or data
 * carried in one RTP packet) in milliseconds. The most common value is 20.
 * \param[in] config Set of tunable configuration parameters to be used.
 * \returns pointer to the newly created twjit instance, or NULL on errors.
 *
 * In contrast to the original Themyscira Wireless version, this version of
 * obts_twjit_create() copies the config structure, thus the application
 * is not required to maintain it in the originally passed memory.
 */
struct obts_twjit *obts_twjit_create(void *ctx, uint16_t clock_khz,
				     uint16_t quantum_ms,
				     const struct obts_twjit_config *config)
{
	struct obts_twjit *twjit;

	if (!config_is_valid(config))
		return NULL;

	twjit = talloc_zero(ctx, struct obts_twjit);
	if (!twjit)
		return NULL;

	memcpy(&twjit->config, config, sizeof(struct obts_twjit_config));
	twjit->state = TWJIT_STATE_EMPTY;
	INIT_LLIST_HEAD(&twjit->sb[0].queue);
	INIT_LLIST_HEAD(&twjit->sb[1].queue);
	twjit->ts_quantum = (uint32_t) quantum_ms * clock_khz;
	twjit->quanta_per_sec = 1000 / quantum_ms;
	twjit->ts_units_per_ms = clock_khz;
	twjit->ts_units_per_sec = (uint32_t) clock_khz * 1000;
	twjit->ns_to_ts_units = 1000000 / clock_khz;

	return twjit;
}

/*! Destroy a twjit instance
 *
 * \param[in] twjit Instance to be freed
 *
 * Memory freed by this function includes not only the instance structure,
 * but also any msgbs that are held by this instance.
 */
void obts_twjit_destroy(struct obts_twjit *twjit)
{
	if (!twjit)
		return;
	msgb_queue_free(&twjit->sb[0].queue);
	msgb_queue_free(&twjit->sb[1].queue);
	talloc_free(twjit);
}

/*! Change twjit config parameters
 *
 * \param[in] twjit Instance to be reconfigured
 * \param[in] config Structure with new tuning parameters to be used
 * \returns 0 if successful, negative on errors
 *
 * The intended use for this API is applications that can use one of
 * several different twjit config profiles depending on various
 * conditions, but might not know the correct choice of profile
 * at the time they create the twjit instance - or more likely,
 * the containing twrtp instance.
 */
int obts_twjit_set_config(struct obts_twjit *twjit,
			  const struct obts_twjit_config *config)
{
	if (!config_is_valid(config))
		return -EINVAL;
	memcpy(&twjit->config, config, sizeof(struct obts_twjit_config));
	return 0;
}

/*! Reset twjit instance to empty initial state
 *
 * \param[in] twjit Instance to be reset
 *
 * This reset function is intended to be called when the application
 * stops doing regular (once every time quantum) reads from the jitter
 * buffer, but may resume this activity later.  All packet Rx state and
 * queues are cleared, but "lifetime" statistical counters are NOT reset.
 */
void obts_twjit_reset(struct obts_twjit *twjit)
{
	msgb_queue_free(&twjit->sb[0].queue);
	msgb_queue_free(&twjit->sb[1].queue);
	twjit->state = TWJIT_STATE_EMPTY;
	twjit->sb[0].depth = 0;
	twjit->sb[1].depth = 0;
	twjit->got_first_packet = false;
}

/* input processing of received RTP packets */

/*! \cond private */

/* raw analytics on the Rx packet stream */

/* This "init" function is called for the very first RTP packet we receive,
 * as well as for received RTP packets that exhibit a change of SSRC
 * from the previously received stream.
 */
static void analytics_init(struct obts_twjit *twjit, uint32_t rx_ssrc,
			   uint16_t rx_seq)
{
	struct obts_twjit_rr_info *rri = &twjit->rr_info;

	rri->ssrc = rx_ssrc;
	rri->rx_packets = 1;
	rri->base_seq = rx_seq;
	rri->max_seq_ext = rx_seq;
	rri->expected_pkt = 1;
	rri->jitter_accum = 0;
}

/* This "continue" function is called for newly received RTP packets that
 * follow previously received ones with the same SSRC.
 */
static void analytics_cont(struct obts_twjit *twjit, uint16_t rx_seq,
			   uint32_t rx_ts, const struct timespec *now)
{
	struct obts_twjit_rr_info *rri = &twjit->rr_info;
	uint16_t seq_ext_lo = rri->max_seq_ext;
	uint16_t seq_ext_hi = rri->max_seq_ext >> 16;
	int16_t seq_delta = (int16_t)(rx_seq - twjit->last_seq);
	int16_t seq_delta2 = (int16_t)(rx_seq - seq_ext_lo);
	int32_t ts_delta = (int32_t)(rx_ts - twjit->last_ts);
	struct timespec time_delta;
	uint32_t time_delta_tsu;
	int32_t jitter_new, ts_delta_clamp;

	/* analytics for our own stats */
	if (seq_delta < 0)
		twjit->stats.seq_backwards++;
	else if (seq_delta == 0)
		twjit->stats.seq_repeats++;
	else if (seq_delta == 1) {
		if (ts_delta != twjit->ts_quantum) {
			if (ts_delta > 0 && (ts_delta % twjit->ts_quantum) == 0)
				twjit->stats.intentional_gaps++;
			else
				twjit->stats.ts_resets++;
		}
	} else
		twjit->stats.seq_skips++;

	/* analytics for RTCP RR: packet counts */
	rri->rx_packets++;
	if (seq_delta2 > 0) {
		if (rx_seq < seq_ext_lo)
			seq_ext_hi++;
		seq_ext_lo = rx_seq;
		rri->max_seq_ext = ((uint32_t) seq_ext_hi << 16) | seq_ext_lo;
		rri->expected_pkt = rri->max_seq_ext - rri->base_seq + 1;
	}

	/* time-of-arrival analytics */
	time_delta.tv_sec = now->tv_sec - twjit->last_arrival.tv_sec;
	time_delta.tv_nsec = now->tv_nsec - twjit->last_arrival.tv_nsec;
	if (time_delta.tv_nsec < 0) {
		time_delta.tv_sec--;
		time_delta.tv_nsec += 1000000000;
	}
	/* to avoid overflows in downstream math, clamp to 1 hour */
	if (time_delta.tv_sec >= 3600) {
		time_delta.tv_sec = 3600;
		time_delta.tv_nsec = 0;
	}
	/* convert to RTP timestamp units */
	time_delta_tsu = time_delta.tv_sec * twjit->ts_units_per_sec +
			 time_delta.tv_nsec / twjit->ns_to_ts_units;
	twjit->last_arrival_delta = time_delta_tsu;
	/* jitter calculation for RTCP RR */
	ts_delta_clamp = twjit->ts_units_per_sec * 3600;
	if (ts_delta > ts_delta_clamp)
		ts_delta = ts_delta_clamp;
	else if (ts_delta < -ts_delta_clamp)
		ts_delta = -ts_delta_clamp;
	jitter_new = time_delta_tsu - ts_delta;
	if (jitter_new < 0)
		jitter_new = -jitter_new;
	/* RFC 3550 section 6.4.1 prescribes a very specific algorithm
	 * for computing the interarrival jitter reported via RTCP.
	 * This prescribed algorithm is a type of leaky integrator.
	 * Here we implement the fixed-point (no floating point operations)
	 * version presented in section A.8 of the same RFC. */
	rri->jitter_accum += jitter_new - ((rri->jitter_accum + 8) >> 4);
	if (jitter_new > twjit->stats.jitter_max)
		twjit->stats.jitter_max = jitter_new;
}

/* actual twjit input logic */

static void
init_subbuf_first_packet(struct obts_twjit *twjit, struct msgb *msg,
			 uint32_t rx_ssrc, uint32_t rx_ts)
{
	struct twjit_subbuf *sb = &twjit->sb[twjit->write_sb];

	OSMO_ASSERT(llist_empty(&sb->queue));
	OSMO_ASSERT(sb->depth == 0);
	/* all good, proceed */
	sb->ssrc = rx_ssrc;
	sb->head_ts = rx_ts;
	msgb_enqueue(&sb->queue, msg);
	sb->depth = 1;
	sb->drop_int_count = 0;
	/* The setting of delta_ms is needed in order to pacify the check
	 * in starting_sb_is_ready() in configurations with bd_start=1.
	 * An alternative would be to enforce start_min_delta being not set
	 * with bd_start=1, but the present solution is simpler than doing
	 * cross-enforcement between two different parameter settings in vty.
	 */
	sb->delta_ms = UINT32_MAX;
}

enum input_decision {
	INPUT_CONTINUE,
	INPUT_TOO_OLD,
	INPUT_RESET,
};

static enum input_decision
check_input_for_subbuf(struct obts_twjit *twjit, bool starting,
			uint32_t rx_ssrc, uint32_t rx_ts)
{
	struct twjit_subbuf *sb = &twjit->sb[twjit->write_sb];
	int32_t ts_delta;

	if (rx_ssrc != sb->ssrc)
		return INPUT_RESET;
	sb->delta_ms = twjit->last_arrival_delta / twjit->ts_units_per_ms;
	ts_delta = (int32_t)(rx_ts - sb->head_ts);
	if (ts_delta < 0)
		return INPUT_TOO_OLD;
	if (ts_delta % twjit->ts_quantum)
		return INPUT_RESET;
	if (starting) {
		if (twjit->config.start_max_delta &&
		    sb->delta_ms > twjit->config.start_max_delta)
			return INPUT_RESET;
	} else {
		uint32_t fwd = ts_delta / twjit->ts_quantum;

		if (fwd >= (uint32_t) twjit->config.max_future_sec *
			   twjit->quanta_per_sec)
			return INPUT_RESET;
	}
	return INPUT_CONTINUE;
}

static void toss_write_queue(struct obts_twjit *twjit)
{
	struct twjit_subbuf *sb = &twjit->sb[twjit->write_sb];

	msgb_queue_free(&sb->queue);
	sb->depth = 0;
}

static void insert_pkt_write_sb(struct obts_twjit *twjit, struct msgb *new_msg,
				uint32_t rx_ts)
{
	struct twjit_subbuf *sb = &twjit->sb[twjit->write_sb];
	uint32_t ts_delta = rx_ts - sb->head_ts;
	uint32_t ins_depth = ts_delta / twjit->ts_quantum;
	struct msgb *old_msg;
	uint32_t old_ts_delta;

	/* are we increasing total depth, and can we do simple tail append? */
	if (ins_depth >= sb->depth) {
		msgb_enqueue(&sb->queue, new_msg);
		sb->depth = ins_depth + 1;
		return;
	}
	/* nope - do it the hard way */
	llist_for_each_entry(old_msg, &sb->queue, list) {
		old_ts_delta = old_msg->cb[0] - sb->head_ts;
		if (old_ts_delta == ts_delta) {
			/* two packets with the same timestamp! */
			twjit->stats.duplicate_ts++;
			msgb_free(new_msg);
			return;
		}
		if (old_ts_delta > ts_delta)
			break;
	}
	llist_add_tail(&new_msg->list, &old_msg->list);
}

static void trim_starting_sb(struct obts_twjit *twjit)
{
	struct twjit_subbuf *sb = &twjit->sb[twjit->write_sb];
	struct msgb *msg;
	uint32_t msg_ts, ts_adv, quantum_adv;

	while (sb->depth > twjit->config.bd_start) {
		msg = msgb_dequeue(&sb->queue);
		OSMO_ASSERT(msg);
		msgb_free(msg);
		OSMO_ASSERT(!llist_empty(&sb->queue));
		msg = llist_entry(sb->queue.next, struct msgb, list);
		msg_ts = msg->cb[0];
		ts_adv = msg_ts - sb->head_ts;
		quantum_adv = ts_adv / twjit->ts_quantum;
		OSMO_ASSERT(sb->depth > quantum_adv);
		sb->head_ts = msg_ts;
		sb->depth -= quantum_adv;
	}
}

/*! \endcond */

/*! Feed received RTP packet to twjit
 *
 * \param[in] twjit Instance to which input is being fed
 * \param[in] msg Message buffer containing the received packet
 *
 * The msgb fed to this API is always consumed by the called function:
 * if it isn't freed for being invalid or too old, it is queued to be
 * regurgitated some time later on the output side.  The design of
 * twjit assumes that this API will be called as soon as each incoming
 * RTP packet is received from the IP network, without any additional
 * delays; in most applications, thus function will be called by twrtp
 * layer from osmo_io Rx callback path.
 */
void obts_twjit_input(struct obts_twjit *twjit, struct msgb *msg)
{
	bool got_previous_input = twjit->got_first_packet;
	struct rtp_hdr *rtph;
	uint32_t rx_ssrc, rx_ts;
	uint16_t rx_seq;
	struct timespec now;
	enum input_decision id;

	rtph = osmo_rtp_get_hdr(msg);
	if (!rtph) {
		twjit->stats.bad_packets++;
		msgb_free(msg);
		return;
	}
	rx_ssrc = ntohl(rtph->ssrc);
	rx_ts = ntohl(rtph->timestamp);
	rx_seq = ntohs(rtph->sequence);
	osmo_clock_gettime(CLOCK_MONOTONIC, &now);
	if (!got_previous_input) {
		analytics_init(twjit, rx_ssrc, rx_seq);
		twjit->got_first_packet = true;
	} else if (rx_ssrc != twjit->rr_info.ssrc) {
		twjit->stats.ssrc_changes++;
		analytics_init(twjit, rx_ssrc, rx_seq);
	} else {
		analytics_cont(twjit, rx_seq, rx_ts, &now);
	}
	twjit->last_seq = rx_seq;
	twjit->last_ts = rx_ts;
	memcpy(&twjit->last_arrival, &now, sizeof(struct timespec));
	twjit->stats.rx_packets++;
	msg->cb[0] = rx_ts;

	switch (twjit->state) {
	case TWJIT_STATE_EMPTY:
		/* first packet into totally empty buffer */
		if (got_previous_input)
			twjit->stats.underruns++;
		twjit->state = TWJIT_STATE_HUNT;
		twjit->write_sb = 0;
		init_subbuf_first_packet(twjit, msg, rx_ssrc, rx_ts);
		return;
	case TWJIT_STATE_HUNT:
	case TWJIT_STATE_HANDOVER:
		id = check_input_for_subbuf(twjit, true, rx_ssrc, rx_ts);
		if (id == INPUT_TOO_OLD) {
			msgb_free(msg);
			return;
		}
		if (id == INPUT_RESET) {
			toss_write_queue(twjit);
			init_subbuf_first_packet(twjit, msg, rx_ssrc, rx_ts);
			return;
		}
		insert_pkt_write_sb(twjit, msg, rx_ts);
		trim_starting_sb(twjit);
		return;
	case TWJIT_STATE_FLOWING:
		id = check_input_for_subbuf(twjit, false, rx_ssrc, rx_ts);
		if (id == INPUT_TOO_OLD) {
			twjit->stats.too_old++;
			msgb_free(msg);
			return;
		}
		if (id == INPUT_RESET) {
			twjit->state = TWJIT_STATE_HANDOVER;
			twjit->write_sb = !twjit->write_sb;
			init_subbuf_first_packet(twjit, msg, rx_ssrc, rx_ts);
			twjit->stats.handovers_in++;
			return;
		}
		insert_pkt_write_sb(twjit, msg, rx_ts);
		return;
	default:
		OSMO_ASSERT(0);
	}
}

/* output to the fixed timing system */

/*! \cond private */

static bool starting_sb_is_ready(struct obts_twjit *twjit)
{
	struct twjit_subbuf *sb = &twjit->sb[twjit->write_sb];

	if (sb->depth < twjit->config.bd_start)
		return false;
	if (sb->delta_ms < twjit->config.start_min_delta)
		return false;
	return true;
}

static bool read_sb_is_empty(struct obts_twjit *twjit)
{
	struct twjit_subbuf *sb = &twjit->sb[twjit->read_sb];

	return sb->depth == 0;
}

static struct msgb *pull_from_read_sb(struct obts_twjit *twjit)
{
	struct twjit_subbuf *sb = &twjit->sb[twjit->read_sb];
	struct msgb *msg;

	OSMO_ASSERT(!llist_empty(&sb->queue));
	OSMO_ASSERT(sb->depth > 0);
	msg = llist_entry(sb->queue.next, struct msgb, list);
	if (msg->cb[0] == sb->head_ts) {
		llist_del(&msg->list);
		twjit->stats.delivered_pkt++;
	} else {
		msg = NULL;
		twjit->stats.output_gaps++;
	}
	sb->head_ts += twjit->ts_quantum;
	sb->depth--;
	return msg;
}

static void read_sb_thinning(struct obts_twjit *twjit)
{
	struct twjit_subbuf *sb = &twjit->sb[twjit->read_sb];
	struct msgb *msg;

	if (sb->drop_int_count) {
		sb->drop_int_count--;
		return;
	}
	if (sb->depth <= twjit->config.bd_hiwat)
		return;
	twjit->stats.thinning_drops++;
	msg = pull_from_read_sb(twjit);
	if (msg)
		msgb_free(msg);
	sb->drop_int_count = twjit->config.thinning_int - 2;
}

static void toss_read_queue(struct obts_twjit *twjit)
{
	struct twjit_subbuf *sb = &twjit->sb[twjit->read_sb];

	msgb_queue_free(&sb->queue);
	sb->depth = 0;
}

/*! \endcond */

/*! Fixed-timing output poll from twjit buffer
 *
 * \param[in] twjit Instance to poll
 * \returns pointer to msgb holding a previously received RTP packet that
 * was successfully mapped to the present quantum in the fixed-timing output,
 * or NULL if no such packet is available.
 */
struct msgb *obts_twjit_output(struct obts_twjit *twjit)
{
	switch (twjit->state) {
	case TWJIT_STATE_EMPTY:
		return NULL;
	case TWJIT_STATE_HUNT:
		if (!starting_sb_is_ready(twjit))
			return NULL;
		twjit->state = TWJIT_STATE_FLOWING;
		twjit->read_sb = twjit->write_sb;
		return pull_from_read_sb(twjit);
	case TWJIT_STATE_FLOWING:
		if (read_sb_is_empty(twjit)) {
			twjit->state = TWJIT_STATE_EMPTY;
			return NULL;
		}
		read_sb_thinning(twjit);
		return pull_from_read_sb(twjit);
	case TWJIT_STATE_HANDOVER:
		if (starting_sb_is_ready(twjit)) {
			toss_read_queue(twjit);
			twjit->stats.handovers_out++;
			twjit->state = TWJIT_STATE_FLOWING;
			twjit->read_sb = twjit->write_sb;
			return pull_from_read_sb(twjit);
		}
		if (read_sb_is_empty(twjit)) {
			twjit->state = TWJIT_STATE_HUNT;
			twjit->stats.ho_underruns++;
			return NULL;
		}
		read_sb_thinning(twjit);
		return pull_from_read_sb(twjit);
	default:
		OSMO_ASSERT(0);
	}
}

/* simple information retrieval functions */

/*! Retrieve lifetime stats from twjit instance
 *
 * \param[in] twjit Instance to query
 * \returns pointer to lifetime stats structure
 */
const struct obts_twjit_stats *
obts_twjit_get_stats(struct obts_twjit *twjit)
{
	return &twjit->stats;
}

/*! Retrieve RR info from twjit instance
 *
 * \param[in] twjit Instance to query
 * \returns pointer to RR info structure
 *
 * The structure retrieved with this API is called RR info because it contains
 * info that is needed in order to constuct RTCP reception reports describing
 * the RTP stream received by this twjit instance.  But of course this info
 * can also be used for other statistics-related or monitoring-related purposes.
 *
 * The structure returned by this API is fully valid only if
 * obts_twjit_got_any_input() returns true.  If that API returns false,
 * the RR info structure returned by the present API should be considered
 * invalid.  More precisely, the "invalid" RR info structure will be all
 * zeros on a freshly created twjit, or stale info if this twjit received
 * some RTP input prior to being reset.  There may be some applications
 * that retrieve the RR info structure to report some non-critical stats;
 * such uses are allowed even when this structure is invalid in the strict
 * sense.
 */
const struct obts_twjit_rr_info *
obts_twjit_get_rr_info(struct obts_twjit *twjit)
{
	return &twjit->rr_info;
}

/*! Did this twjit instance ever receive RTP input?
 *
 * \param[in] twjit Instance to query
 * \returns true if this twjit instance received RTP input since it was
 * created or last reset, false otherwise.
 */
bool obts_twjit_got_any_input(struct obts_twjit *twjit)
{
	return twjit->got_first_packet;
}

/*! @} */
