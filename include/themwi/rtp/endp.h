/*
 * twrtp_endp is the big abstraction provided by libtwrtp: a complete
 * RTP endpoint with RTP & RTCP sockets, sending and receiving both types
 * of packets, and incorporating twjit.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include <osmocom/core/osmo_io.h>
#include <osmocom/core/socket.h>
#include <osmocom/core/timer.h>

#include <themwi/rtp/twjit.h>

struct twrtp_endp_tx {
	uint32_t ssrc;
	uint32_t ts;
	uint16_t seq;
	bool started;
	bool restart;
};

struct twrtp_endp_rtcp_rx {
	uint32_t sr_ssrc;
	uint16_t sr_ntp_sec;
	uint16_t sr_ntp_fract;
	struct timespec sr_rx_time;
	uint32_t rr_lost_word;
	uint32_t rr_jitter;
	bool got_sr;
	bool got_rr;
};

struct twrtp_endp_rtcp_tx {
	uint32_t last_received;
	uint32_t last_expected;
};

struct twrtp_endp_stats {
	uint32_t rx_rtp_pkt;
	uint32_t rx_rtp_badsrc;
	uint32_t rx_rtcp_pkt;
	uint32_t rx_rtcp_badsrc;
	uint32_t rx_rtcp_invalid;
	uint32_t rx_rtcp_wrong_ssrc;
	uint32_t tx_rtp_pkt;
	uint32_t tx_rtp_bytes;
	uint32_t tx_rtcp_pkt;
};

struct twrtp_endp {
	/* the root of the matter: the two sockets */
	int rtp_fd;
	int rtcp_fd;
	struct osmo_io_fd *iofd_rtp;
	struct osmo_io_fd *iofd_rtcp;
	struct osmo_sockaddr rtp_remote;
	struct osmo_sockaddr rtcp_remote;
	/* Rx and Tx state */
	struct twrtp_jibuf_inst *twjit;
	struct twrtp_endp_tx tx;
	struct twrtp_endp_rtcp_rx rtcp_rx;
	struct twrtp_endp_rtcp_tx rtcp_tx;
	uint8_t *sdes_buf;
	uint16_t sdes_len;
	uint16_t auto_rtcp_interval;
	uint16_t auto_rtcp_count;
	/* always have to have stats */
	struct twrtp_endp_stats stats;
	/* bool flags at the end for structure packing optimization */
	bool register_done;
	bool remote_set;
	bool rx_enable;
};

/* public API functions */

struct twrtp_endp *twrtp_endp_create(void *ctx,
				     struct twrtp_jibuf_config *config);
void twrtp_endp_destroy(struct twrtp_endp *endp);

int twrtp_endp_register_fds(struct twrtp_endp *endp);
int twrtp_endp_bind_ip_port(struct twrtp_endp *endp, const char *ip,
			    uint16_t port);

void twrtp_endp_set_remote_ipv4(struct twrtp_endp *endp,
				const struct in_addr *ip, uint16_t port);

int twrtp_endp_tx_quantum(struct twrtp_endp *endp, const uint8_t *payload,
			  unsigned payload_len, uint8_t payload_type,
			  bool marker, bool auto_marker, bool send_rtcp);
void twrtp_endp_tx_skip(struct twrtp_endp *endp);

int twrtp_endp_set_sdes(struct twrtp_endp *endp, const char *cname,
			const char *name, const char *email, const char *phone,
			const char *loc, const char *tool, const char *note);
int twrtp_endp_send_rtcp_rr(struct twrtp_endp *endp);

int twrtp_endp_set_dscp(struct twrtp_endp *endp, uint8_t dscp);
int twrtp_endp_set_socket_prio(struct twrtp_endp *endp, int prio);
