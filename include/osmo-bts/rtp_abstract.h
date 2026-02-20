/*
 * This header file defines the RTP abstraction layer inside osmo-bts,
 * allowing a choice between two different RTP endpoint libraries:
 * Belledonne ortp or Themyscira twrtp.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <netinet/in.h>

/* The following preprocessor definitions are legacy from
 * <osmocom/trau/osmo_ortp.h>.  They have to be kept unchanged
 * in order to allow the rest of OsmoBTS code to use the present
 * abstraction header in the place of osmo_ortp.h - the latter
 * may not exist if libosmo-abis is built with --disable-ortp -
 * and to prevent conflicts when rtp_abstract.c has to include
 * both osmo_ortp.h and the present header in the default
 * ortp-enabled build config.
 */

/*! \brief default duration of a 20ms GSM codec frame */
#define GSM_RTP_DURATION 160

#define GSM_VOICE_SAMPLE_RATE_HZ 8000
#define GSM_VOICE_SAMPLES_PER_MS (GSM_VOICE_SAMPLE_RATE_HZ / 1000)
#define GSM_VOICE_MULTIFRAME 26
#define GSM_RTP_FRAME_DURATION_MS 20
#define GSM_SAMPLES_PER_RTP_FRAME (GSM_RTP_FRAME_DURATION_MS * GSM_VOICE_SAMPLES_PER_MS)
#define GSM_TDMA_FRAME_MS (120 / GSM_VOICE_MULTIFRAME)
#define GSM_MS_TO_SAMPLES(ms) ((ms) * GSM_VOICE_SAMPLES_PER_MS)
#define GSM_FN_TO_MS(fn) ((fn) * GSM_TDMA_FRAME_MS)

struct osmo_rtp_socket;
struct osmo_twrtp;
struct osmo_twjit_config;

/*! \brief Abstracted RTP socket structure */
struct rtp_abst_socket {
	/* user fields */
	void (*rx_cb)(struct rtp_abst_socket *rs, const uint8_t *payload,
		      unsigned int payload_len, uint16_t seq_number,
		      uint32_t timestamp, bool marker);
	void *priv;

	/* internal implementation */
	struct osmo_rtp_socket *ortp;
	struct osmo_twrtp *twrtp;
	struct sockaddr_in local_addr;
	uint32_t twrtp_rx_ticks;
	uint32_t twrtp_rx_pl_bytes;
	uint32_t twrtp_rx_bad_hdr;
	uint32_t twrtp_rx_wrong_pt;
	uint32_t twrtp_pl_extr_errors;
	uint8_t payload_type;
};

struct rtp_abst_socket *
rtp_abst_socket_create(void *talloc_ctx, bool use_twrtp,
			struct osmo_twjit_config *twjit_cfg);
void rtp_abst_socket_free(struct rtp_abst_socket *rs);

int rtp_abst_socket_bind(struct rtp_abst_socket *rs, const char *ipstr,
			 int port);
int rtp_abst_socket_connect(struct rtp_abst_socket *rs,
			    const struct in_addr *ip, uint16_t port);

int rtp_abst_socket_set_pt(struct rtp_abst_socket *rs, int payload_type);
int rtp_abst_socket_set_dscp(struct rtp_abst_socket *rs, int dscp);
int rtp_abst_socket_set_priority(struct rtp_abst_socket *rs, uint8_t prio);

void rtp_abst_socket_poll(struct rtp_abst_socket *rs);

int rtp_abst_send_frame(struct rtp_abst_socket *rs, const uint8_t *payload,
			unsigned int payload_len, bool marker);
int rtp_abst_skipped_frame(struct rtp_abst_socket *rs);

int rtp_abst_get_bound_ip_port(struct rtp_abst_socket *rs,
			       uint32_t *ip, int *port);

int rtp_abst_socket_set_param(struct rtp_abst_socket *rs,
			      bool jitter_adaptive, int jitter_ms);

void rtp_abst_socket_log_stats(struct rtp_abst_socket *rs, const char *cause);

void rtp_abst_socket_stats(struct rtp_abst_socket *rs,
				uint32_t *sent_packets, uint32_t *sent_octets,
				uint32_t *recv_packets, uint32_t *recv_octets,
				uint32_t *recv_lost, uint32_t *last_jitter);

void rtp_abst_set_source_desc(struct rtp_abst_socket *rs, const char *cname,
				const char *name, const char *email,
				const char *phone, const char *loc,
				const char *tool, const char *note);
