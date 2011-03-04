#ifndef _RTP_H
#define _RTP_H

struct osmobts_lchan *lchan;

#define RTP_PT_GSM_FULL 3
#define RTP_PT_GSM_HALF 96
#define RTP_PT_GSM_EFR 97
#define RTP_PT_AMR_FULL 98
#define RTP_PT_AMR_HALF 99

struct osmobts_udp {
	struct osmobts_rtp	*rtp;
	struct bsc_fd		bfd;
	struct sockaddr_in	sin_local, sin_remote;
	struct llist_head	tx_queue;
};

struct osmobts_rtp {
	struct osmobts_lchan	*lchan;
	int			socket_created;
	struct osmobts_udp	rtp_udp, rtcp_udp;
	struct llist_head	dejitter_queue;
	int			dejitter_num;
	int			dejitter_min;
	int			dejitter_check;
	struct msgb		*last_frame;
	uint8_t			payload_type;
	uint16_t		sequence;
	uint32_t		timestamp;
	uint32_t		ssrc;
	int			voice_req;
};

int rtp_create_socket(struct osmobts_lchan *lchan, struct osmobts_rtp *rtp);
int rtp_bind_socket(struct osmobts_rtp *rtp);
int rtp_connect_socket(struct osmobts_rtp *rtp, uint32_t ip, uint16_t port);
int rtp_close_socket(struct osmobts_rtp *rtp);

struct gsm_data_frame {
	uint32_t	timestamp;
	uint8_t		payload_type;
	uint8_t		data[0];
};

#endif /* _RTP_H */

