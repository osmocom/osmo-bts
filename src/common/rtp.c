/*
 * (C) 2011 by Andreas Eversberg <jolly@eversberg.eu>
 *
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

/*
 * RTP peer
 */

#include <errno.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>

#include <osmocore/msgb.h>

#include <osmo-bts/logging.h>
//#include <osmocom/bb/common/osmocom_data.h>
#include <osmo-bts/support.h>
#include <osmo-bts/abis.h>
#include <osmo-bts/rtp.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/rsl.h>
#include <osmo-bts/oml.h>

#define RTP_PORTBASE 30000
static unsigned short next_udp_port = RTP_PORTBASE;

enum {
	UDP_PRIV_RTP,
	UDP_PRIV_RTCP,
};

#define RTP_ALLOC_SIZE  1500

/* according to RFC 1889 */
struct rtcp_hdr {
	u_int8_t byte0;
	u_int8_t type;
	u_int16_t length;
} __attribute__((packed));

#define RTCP_TYPE_SDES  202

#define RTCP_IE_CNAME   1

/* according to RFC 3550 */
struct rtp_hdr {
#if __BYTE_ORDER == __LITTLE_ENDIAN
	u_int8_t  csrc_count:4,
		  extension:1,
		  padding:1,
		  version:2;
	u_int8_t  payload_type:7,
		  marker:1;
#elif __BYTE_ORDER == __BIG_ENDIAN
	u_int8_t  version:2,
		  padding:1,
		  extension:1,
		  csrc_count:4;
	u_int8_t  marker:1,
		  payload_type:7;
#endif  
	u_int16_t sequence;
	u_int32_t timestamp;
	u_int32_t ssrc;

	uint8_t   data[0];
} __attribute__((packed));

struct rtp_x_hdr {
	u_int16_t by_profile;
	u_int16_t length;
} __attribute__((packed));

#define RTP_VERSION     2

int rtp_tx_rtp(struct osmobts_rtp *rtp, struct msgb *msg)
{
	struct gsm_data_frame *frame = (struct gsm_data_frame *) msg->data;
	int payload_len = msg->len - sizeof(*frame);
	uint8_t *payload = frame->data;
	struct msgb *nmsg = NULL;
	int duration = 0;
	struct rtp_hdr *nrtph;

	if (rtp->rtp_udp.bfd.fd <= 0) {
		LOGP(DRTP, LOGL_INFO, "Socket already closed, dropping\n");
		msgb_free(msg);
		return -EIO;
	}

	switch(rtp->payload_type) {
	case RTP_PT_GSM_FULL:
		nmsg = msgb_alloc(sizeof(struct rtp_hdr) + payload_len, "RTP-GSM-FULL");
		duration = 160;
		break;
	case RTP_PT_GSM_EFR:
		nmsg = msgb_alloc(sizeof(struct rtp_hdr) + payload_len, "RTP-GSM-FULL");
		duration = 160;
		break;
	}
	if (!msg) {
		msgb_free(msg);
		return -ENOMEM;
	}

	nrtph = (struct rtp_hdr *) msgb_put(nmsg, sizeof(*nrtph) + payload_len);
	nrtph->version = RTP_VERSION;
	nrtph->padding = 0;
	nrtph->extension = 0;
	nrtph->csrc_count = 0;
	nrtph->marker = 0;
	nrtph->payload_type = rtp->payload_type;
	nrtph->sequence = rtp->sequence++;
	nrtph->timestamp = rtp->timestamp;
	rtp->timestamp += duration;
	nrtph->ssrc = rtp->ssrc;
	memcpy(nrtph->data, payload, payload_len);
	msgb_enqueue(&rtp->rtp_udp.tx_queue, nmsg);
	rtp->rtp_udp.bfd.when |= BSC_FD_WRITE;

	msgb_free(msg);

	return 0;
}

/* received TCH frame */
static int rtp_rx_rtp(struct osmobts_rtp *rtp, struct msgb *msg)
{
	struct rtp_hdr *rtph = (struct rtp_hdr *) msg->data;
	struct rtp_x_hdr *rtpxh;
	uint8_t *payload;
	int payload_len, x_len;
	struct msgb *nmsg = NULL;
	struct gsm_data_frame *frame;
	time_t now;

	if (msg->len < 12) {
		LOGP(DRTP, LOGL_NOTICE, "RTP packet too short\n");
		return -EINVAL;
	}
	if (rtph->version != RTP_VERSION) {
		LOGP(DRTP, LOGL_NOTICE, "RTP version missmatch.\n");
		return -EINVAL;
	}
	payload = msg->data + sizeof(struct rtp_hdr) + (rtph->csrc_count << 2);
	payload_len = msg->len - sizeof(struct rtp_hdr) - (rtph->csrc_count << 2);
	if (payload_len < 0) {
		LOGP(DRTP, LOGL_NOTICE, "RTP payload too short.\n");
		return -EINVAL;
	}
	if (rtph->extension) {
		if (payload_len < sizeof(struct rtp_x_hdr)) {
			LOGP(DRTP, LOGL_NOTICE, "RTP frame for extension too short.\n");
			return -EINVAL;
		}
		rtpxh = (struct rtp_x_hdr *)payload;
		x_len = ntohs(rtpxh->length) * 4 + sizeof(struct rtp_x_hdr);
		payload += x_len;
		payload_len -= x_len;
		if (payload_len < 0) {
			LOGP(DRTP, LOGL_NOTICE, "RTP extension exceeds frame length.\n");
			return -EINVAL;
		}
	}
	if (rtph->padding) {
		if (payload_len == 0) {
			LOGP(DRTP, LOGL_NOTICE, "RTP frame too short for padding.\n");
			return -EINVAL;
		}
		payload_len -= payload[payload_len - 1];
		if (payload_len < 0) {
			LOGP(DRTP, LOGL_NOTICE, "RTP frame has padding greater than payload.\n");
			return -EINVAL;
		}
	}
	switch (rtph->payload_type) {
	case RTP_PT_GSM_FULL:
		if (payload_len != 33) {
			LOGP(DRTP, LOGL_NOTICE, "RTP full rate frame has invald payload length.\n");
		}
		nmsg = msgb_alloc(sizeof(*frame) + payload_len, "GSM-FULL");
		break;
		if (payload_len != 31) {
			LOGP(DRTP, LOGL_NOTICE, "RTP enhanced full rate frame has invald payload length.\n");
		}
		nmsg = msgb_alloc(sizeof(*frame) + payload_len, "GSM-EFR");
		break;
	}
	if (!nmsg)
		return -ENOMEM;
	frame = (struct gsm_data_frame *) msgb_put(nmsg, sizeof(*frame) + payload_len);
	frame->payload_type = rtph->payload_type;
	frame->timestamp = ntohl(rtph->timestamp);
	memcpy(frame->data, payload, payload_len);

	/* check jitterbuffer and adjust delay */
	time(&now);
	if (rtp->dejitter_check <= now) {
		struct msgb *fmsg;

		LOGP(DRTP, LOGL_INFO, "RTP dejitter: minimum fill: %d packets (dropping)\n", rtp->dejitter_min);
		while (rtp->dejitter_min && (fmsg = msgb_dequeue(&rtp->dejitter_queue))) {
			msgb_free(fmsg);
			rtp->dejitter_num--;
			rtp->dejitter_min--;
		}
		rtp->dejitter_check = now + 5;
		rtp->dejitter_min = 9999;
	}
	if (rtp->dejitter_num < rtp->dejitter_min)
		rtp->dejitter_min = rtp->dejitter_num;

	/* write to dejitter queue */
	msgb_enqueue(&rtp->dejitter_queue, nmsg);
	rtp->dejitter_num++;

	if (rtp->voice_req)
		return 0;

	// FIXME: send voice frame to l1ctl
	rtp->voice_req = 1;
	rtp->dejitter_num--;
	if (rtp->last_frame)
		msgb_free(rtp->last_frame);
	rtp->last_frame = nmsg;

	return 0;
}

/* on confirm, send next packet */
int rtp_voice_conf(struct osmobts_rtp *rtp)
{
	struct msgb *msg;


	msg = msgb_dequeue(&rtp->dejitter_queue);
	if (!msg) {
		msg = rtp->last_frame;
		if (!msg) {
			rtp->voice_req = 0;
			return 0;
		}
	} else {
		rtp->dejitter_num--;
		if (rtp->last_frame)
			msgb_free(rtp->last_frame);
		rtp->last_frame = msg;
	}

	// FIXME: send voice frame to l1ctl

	return 0;
}

static int udp_bfd_cb(struct bsc_fd *bfd, unsigned int flags)
{
	struct osmobts_udp *udp = bfd->data;
	struct msgb *msg = NULL;
	int rc;

	switch (bfd->priv_nr) {
	case UDP_PRIV_RTP:
		msg = msgb_alloc(RTP_ALLOC_SIZE, "RTP");
		break;
	case UDP_PRIV_RTCP:
		msg = msgb_alloc(RTP_ALLOC_SIZE, "RTCP");
		break;
	}
	if (!msg)
		return -ENOMEM;

	rc = read(bfd->fd, msg->data, RTP_ALLOC_SIZE);
	if (rc <= 0) {
		bfd->when &= ~BSC_FD_READ;
		return rc;
	}
	msgb_put(msg, rc);

	switch (bfd->priv_nr) {
	case UDP_PRIV_RTP:
		if ((flags & BSC_FD_READ))
			rc = rtp_rx_rtp(udp->rtp, msg);
		if ((flags & BSC_FD_WRITE)) {
			struct msgb *msg = msgb_dequeue(&udp->tx_queue);

			if (!msg) {
				bfd->when &= ~BSC_FD_WRITE;
				rc = 0;
				break;
			}
			rc = write(bfd->fd, msg->data, msg->len);
		}
		break;
	case UDP_PRIV_RTCP:
		// FIXME: implement RTCP
		rc = 0;
		break;
	}
	msgb_free(msg);

	return rc;
}

/* subfunction: create an UDP socket */
static int udp_create_socket(struct osmobts_rtp *rtp, struct osmobts_udp *udp, int priv_nr)
{
	int rc;

	if (udp->bfd.fd <= 0)
		return -EBUSY;

	rc = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (rc < 0)
		return rc;
	udp->bfd.fd = rc;
	udp->bfd.data = udp;
	udp->bfd.priv_nr = priv_nr;
	udp->bfd.cb = udp_bfd_cb;
	udp->bfd.when = BSC_FD_READ;

	udp->rtp = rtp;
	rc = bsc_register_fd(&udp->bfd);
	if (rc < 0) {
		close(udp->bfd.fd);
		udp->bfd.fd = 0;
	}
	INIT_LLIST_HEAD(&udp->tx_queue);

	return udp->bfd.fd;
}

/* subfunction: close an UDP socket */
static void udp_close_socket(struct osmobts_udp *udp)
{
	struct msgb *msg;

	if (udp->bfd.fd <= 0)
		return;

	while ((msg = msgb_dequeue(&udp->tx_queue)))
		msgb_free(msg);

	bsc_unregister_fd(&udp->bfd);
	close(udp->bfd.fd);
	udp->bfd.fd = 0;
}

/* create RTP/RTCP sockets */
int rtp_create_socket(struct osmobts_lchan *lchan, struct osmobts_rtp *rtp)
{
	int rc;

	LOGP(DRTP, LOGL_INFO, "Creating RTP/RTCP sockets\n");

	rc = udp_create_socket(rtp, &rtp->rtp_udp, UDP_PRIV_RTP);
	if (rc < 0) {
		LOGP(DRTP, LOGL_ERROR, "Failed to create RTP socket\n");
		return rc;
	}
	rc = udp_create_socket(rtp, &rtp->rtcp_udp, UDP_PRIV_RTCP);
	if (rc < 0) {
		LOGP(DRTP, LOGL_ERROR, "Failed to create RTCP socket\n");
		udp_close_socket(&rtp->rtp_udp);
		return rc;
	}
	rtp->socket_created = 1;
	rtp->lchan = lchan;
	rtp->ssrc = rand();
	rtp->sequence = random();
	rtp->timestamp = random();
	INIT_LLIST_HEAD(&rtp->dejitter_queue);
	rtp->dejitter_num = 0;
	rtp->dejitter_check = 0;

	return 0;
}

/* close RTP/RTCP sockets */
int rtp_close_socket(struct osmobts_rtp *rtp)
{
	struct msgb *msg;

	LOGP(DRTP, LOGL_INFO, "Closing RTP/RTCP sockets\n");

	while ((msg = msgb_dequeue(&rtp->dejitter_queue)))
		msgb_free(msg);
	if (rtp->last_frame) {
		msgb_free(rtp->last_frame);
		rtp->last_frame = NULL;
	}

	udp_close_socket(&rtp->rtp_udp);
	udp_close_socket(&rtp->rtcp_udp);
	rtp->socket_created = 0;

	return 0;
}

/* subfunction: bind an UDP socket */
static int udp_bind_socket(struct osmobts_udp *udp)
{
	socklen_t alen = sizeof(udp->sin_local);
	int rc;

	if (udp->bfd.fd <= 0)
		return -EINVAL;

	udp->sin_local.sin_family = AF_INET;
	udp->sin_local.sin_addr.s_addr = INADDR_ANY;
	udp->sin_local.sin_port = htons(next_udp_port++ & 0xffff);

	rc = bind(udp->bfd.fd, (struct sockaddr *)&udp->sin_local, sizeof(udp->sin_local));
	if (rc < 0)
		return rc;

	rc = getsockname(udp->bfd.fd, (struct sockaddr *)&udp->sin_local, &alen);

	return 0;
}

/* binds the local RTP/RTCP sockets */
int rtp_bind_socket(struct osmobts_rtp *rtp)
{
	int rc;
	int i;

	LOGP(DRTP, LOGL_INFO, "Binding RTP/RTCP sockets\n");

	if (!rtp->socket_created)
		return -EINVAL;

	for (i = 0; i < 1000; i++) {
		/* try RTP socket */
		rc = udp_bind_socket(&rtp->rtp_udp);
		if (rc < 0)
			continue;
		/* now try RTCP socket */
		rc = udp_bind_socket(&rtp->rtp_udp);
		if (rc == 0) {
			LOGP(DRTP, LOGL_INFO, "Sockets bount to: port(RTP)=%d port(RTCP)=%d\n", ntohs(rtp->rtp_udp.sin_local.sin_port), ntohs(rtp->rtcp_udp.sin_local.sin_port));
			return 0;
		}
		/* if fails, create a new RTP socket and start over */
		udp_close_socket(&rtp->rtp_udp);
		rc = udp_create_socket(rtp, &rtp->rtp_udp, UDP_PRIV_RTP);
		if (rc < 0)
			return rc;
	}

	return -EIO;
}


/* subfunction: connect an UDP socket */
static int udp_connect_socket(struct osmobts_udp *udp, uint32_t ip, uint16_t port)
{
	socklen_t alen = sizeof(udp->sin_local);
	int rc;

	if (udp->bfd.fd <= 0)
		return -EINVAL;

	udp->sin_remote.sin_family = AF_INET;
	udp->sin_remote.sin_addr.s_addr = htonl(ip);
	udp->sin_remote.sin_port = htons(port);

	rc = connect(udp->bfd.fd, (struct sockaddr *)&udp->sin_remote, sizeof(udp->sin_remote));
	if (rc < 0)
		return rc;

	rc = getsockname(udp->bfd.fd, (struct sockaddr *)&udp->sin_local, &alen);

	return 0;
}

/* connects the local RTP/RTCP sockets */
int rtp_connect_socket(struct osmobts_rtp *rtp, uint32_t ip, uint16_t port)
{
	int rc;

	LOGP(DRTP, LOGL_INFO, "Binding RTP/RTCP sockets\n");

	rc = udp_connect_socket(&rtp->rtp_udp, ip, port);
	if (rc < 0) {
		LOGP(DRTP, LOGL_ERROR, "Failed to connect RTP socket: ip=%08x, port=%d\n", ip, port);
		return rc;
	}

	rc = udp_connect_socket(&rtp->rtcp_udp, ip, port + 1);
	if (rc < 0) {
		LOGP(DRTP, LOGL_ERROR, "Failed to connect RTCP socket: ip=%08x, port=%d\n", ip, port);
		return rc;
	}

	LOGP(DRTP, LOGL_INFO, "Sockets connected to: ip=%d port(RTP)=%d port(RTCP)=%d\n", ip, port, port + 1);

	return 0;
}

