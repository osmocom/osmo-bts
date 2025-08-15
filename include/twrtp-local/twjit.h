/*
 * Themyscira Wireless RTP jitter buffer implementation:
 * public API definition for Osmocom-integrated version.
 *
 * This code was contributed to Osmocom Cellular Network Infrastructure
 * project by Mother Mychaela N. Falconia of Themyscira Wireless.
 * Mother Mychaela's contributions are NOT subject to copyright:
 * no rights reserved, all rights relinquished.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include <osmocom/core/msgb.h>

/*! \defgroup twjit Themyscira Wireless RTP jitter buffer implementation
 *  @{
 *
 *  The present twjit layer is an interface mechanism from an incoming
 *  RTP stream to an output application that has fixed timing requirements,
 *  e.g., the Tx side of GSM Um TCH or a T1/E1 TDM interface.
 *
 *  There also exists a detailed document titled _Guide to ThemWi RTP
 *  endpoint library_, located here:
 *  https://www.freecalypso.org/TW-doc/twrtp-guide-latest.pdf
 *  (See TW-doc directory listing for other formats and previous versions.)
 *  This document is required reading for anyone seeking to properly
 *  understand the present jitter buffer facility, its domain of application
 *  and how to use it.  Specific section references to this document
 *  will be made in subsequent comments.
 */

/*! Each instance of twjit in the present version exists as struct obts_twjit.
 *  This structure is opaque, and always constitutes a talloc context. */
struct obts_twjit;

/*! twjit configuration tunings, usually set via vty.
 *  This config structure always has to be provided in order to
 *  create a twjit instance.
 *
 *  The set of configurable parameters contained in this structure
 *  is covered in twrtp guide document section 2.4.
 */
struct obts_twjit_config {
	/*! buffer depth: starting minimum, formally called flow-starting
	 *  fill level.  Document section: 2.3.3. */
	uint16_t bd_start;
	/*! buffer depth: high water mark, formally called high water mark
	 *  fill level.  Document section: 2.3.4.2. */
	uint16_t bd_hiwat;
	/*! interval for thinning of too-deep standing queue;
	 *  document section: 2.3.4.2. */
	uint16_t thinning_int;
	/*! guard against time traveler RTP packets, 1 s units;
	 *  document section: 2.3.4.3. */
	uint16_t max_future_sec;
	/*! min time delta in starting state, 1 ms units, 0 means not set;
	 *  document section: 2.3.3.2. */
	uint16_t start_min_delta;
	/*! max time delta in starting state, 1 ms units, 0 means not set;
	 *  document section: 2.3.3.2. */
	uint16_t start_max_delta;
};

/*! Stats collected during the lifetime of a twjit instance.
 *  For a detailed description of each of these counters, see Chapter 3
 *  of twrtp guide document. */
struct obts_twjit_stats {
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

/*! Info collected from the incoming RTP data stream
 *  for the purpose of generating RTCP reception report blocks.
 *  See twrtp guide document section 5.1.
 *
 *  Key point: unlike the counters in struct obts_twjit_stats,
 *  all RR info is reset to initial whenever incoming SSRC changes,
 *  as necessitated by RTCP data model being organized per SSRC.
 */
struct obts_twjit_rr_info {
	/*! received SSRC to which all following info applies */
	uint32_t ssrc;
	/*! count of "received packets" for RTCP RR packet loss calculation */
	uint32_t rx_packets;
	/*! "base" sequence number for "expected packets" computation */
	uint32_t base_seq;
	/*! "extended highest sequence number" field of RTCP RR */
	uint32_t max_seq_ext;
	/*! count of "expected packets" for RTCP RR packet loss calculation */
	uint32_t expected_pkt;
	/*! "interarrival jitter" measure of RFC 3550, accumulator for the
	 *  leaky integrator algorithm prescribed by the RFC, sans-FP version.
	 *  Right-shift this accumulator by 4 bits when emitting RTCP RR. */
	uint32_t jitter_accum;
};

/* twjit module API functions */

struct obts_twjit *obts_twjit_create(void *ctx, uint16_t clock_khz,
				     uint16_t quantum_ms,
				     const struct obts_twjit_config *config);
void obts_twjit_destroy(struct obts_twjit *twjit);

int obts_twjit_set_config(struct obts_twjit *twjit,
			  const struct obts_twjit_config *config);
void obts_twjit_reset(struct obts_twjit *twjit);

/* RTP input, takes ownership of msgb */
void obts_twjit_input(struct obts_twjit *twjit, struct msgb *msg);

/* output function, to be called by TDM/GSM/etc fixed-timing side */
struct msgb *obts_twjit_output(struct obts_twjit *twjit);

/* Stats and RR info structures are contained inside opaque struct obts_twjit.
 * We need to provide access to these stats and RR info structures to API
 * users, but we don't want to make the whole twjit instance struct public.
 * Also we would like to have fast external access to these stats, hence an API
 * that copies our stats to caller-provided storage would be very inefficient.
 * Compromise: we allow direct external access to just these selected parts
 * of the full internal state structure by providing API functions that
 * return pointers to these selected parts.
 */
const struct obts_twjit_stats *
obts_twjit_get_stats(struct obts_twjit *twjit);

const struct obts_twjit_rr_info *
obts_twjit_get_rr_info(struct obts_twjit *twjit);

/* When we compose outgoing RTCP packets in the upper layer of twrtp,
 * we need to know whether or not we have received at least one valid
 * RTP data packet so far.  If we haven't received any RTP yet, then
 * we have no Rx SSRC, all data in struct obts_twjit_rr_info are invalid,
 * and we cannot send RTCP reception reports.
 */
bool obts_twjit_got_any_input(struct obts_twjit *twjit);

/* vty configuration functions */

void obts_twjit_config_init(struct obts_twjit_config *config);

void obts_twjit_vty_init(int twjit_node);

struct vty;

int obts_twjit_config_write(struct vty *vty,
			    const struct obts_twjit_config *conf,
			    const char *prefix);

/*! @} */
