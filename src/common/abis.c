/* (C) 2011 by Andreas Eversberg <jolly@eversberg.eu>
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

#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

#include <osmocore/select.h>
#include <osmocore/timer.h>
#include <osmocore/msgb.h>
#include <osmocom/bb/common/logging.h>
#include <osmocom/bb/common/osmocom_data.h>
#include <osmocom/bb/bts/abis.h>
#include <osmocom/bb/bts/rtp.h>
#include <osmocom/bb/bts/bts.h>
#include <osmocom/bb/bts/rsl.h>
#include <osmocom/bb/bts/oml.h>

extern char *software_version;
extern uint8_t abis_mac[6];

/*
 * support
 */

#define ABIS_ALLOC_SIZE	900

/* send message to BSC */
int abis_tx(struct ipabis_link *link, struct msgb *msg)
{
	if (link->state != LINK_STATE_CONNECT) {
		LOGP(DABIS, LOGL_NOTICE, "Link down, dropping message.\n");
		msgb_free(msg);
		return -EIO;
	}
	msgb_enqueue(&link->tx_queue, msg);
	link->bfd.when |= BSC_FD_WRITE;

	return 0;
}


struct msgb *abis_msgb_alloc(int headroom)
{
	struct msgb *nmsg;

	headroom += sizeof(struct ipabis_head);

	nmsg = msgb_alloc_headroom(ABIS_ALLOC_SIZE + headroom,
		headroom, "Abis/IP");
	if (!nmsg)
		return NULL;
	return nmsg;
}

void abis_push_ipa(struct msgb *msg, uint8_t proto)
{
	struct ipabis_head *nhh;

	msg->l2h = msg->data;
	nhh = (struct ipabis_head *) msgb_push(msg, sizeof(*nhh));
	nhh->proto = proto;
	nhh->len = htons(msgb_l2len(msg));
}

/*
 * IPA related messages
 */

/* send ping/pong */
static int abis_tx_ipa_pingpong(struct ipabis_link *link, uint8_t pingpong)
{
	struct msgb *nmsg;

	nmsg = abis_msgb_alloc(0);
	if (!nmsg)
		return -ENOMEM;
	*msgb_put(nmsg, 1) = pingpong;
	abis_push_ipa(nmsg, IPA_PROTO_IPACCESS);

	return abis_tx(link, nmsg);
}

/* send ACK and ID RESP */
static int abis_rx_ipa_id_get(struct ipabis_link *link, uint8_t *data, int len)
{
	struct msgb *nmsg, *nmsg2;
	char str[64];
	uint8_t *tag;

	LOGP(DABIS, LOGL_INFO, "Reply to ID_GET\n");

	nmsg = abis_msgb_alloc(0);
	if (!nmsg)
		return -ENOMEM;
	*msgb_put(nmsg, 1) = IPA_MSGT_ID_RESP;
	while (len) {
		if (len < 2) {
			LOGP(DABIS, LOGL_NOTICE,
				"Short read of ipaccess tag\n");
			msgb_free(nmsg);
			return -EIO;
		}
		switch (data[1]) {
		case IPA_IDTAG_UNIT:
			sprintf(str, "%s/%d", (link->trx) ? link->trx->bts->id
							  : link->bts->id,
				(link->trx) ? link->trx->trx_nr : 0);
			break;
		case IPA_IDTAG_MACADDR:
			sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
				abis_mac[0], abis_mac[1], abis_mac[2],
				abis_mac[3], abis_mac[4], abis_mac[5]);
			break;
		case IPA_IDTAG_LOCATION1:
			strcpy(str, "osmoBTS");
			break;
		case IPA_IDTAG_LOCATION2:
			strcpy(str, "osmoBTS");
			break;
		case IPA_IDTAG_EQUIPVERS:
		case IPA_IDTAG_SWVERSION:
			strcpy(str, software_version);
			break;
		case IPA_IDTAG_UNITNAME:
			sprintf(str, "osmoBTS-%02x-%02x-%02x-%02x-%02x-%02x",
				abis_mac[0], abis_mac[1], abis_mac[2],
				abis_mac[3], abis_mac[4], abis_mac[5]);
			break;
		case IPA_IDTAG_SERNR:
			strcpy(str, "");
			break;
		default:
			LOGP(DABIS, LOGL_NOTICE,
				"Unknown ipaccess tag 0x%02x\n", *data);
			msgb_free(nmsg);
			return -EINVAL;
		}
		LOGP(DABIS, LOGL_INFO, " tag %d: %s\n", data[1], str);
		tag = msgb_put(nmsg, 3 + strlen(str) + 1);
		tag[0] = 0x00;
		tag[1] = 1 + strlen(str) + 1;
		tag[2] = data[1];
		memcpy(tag + 3, str, strlen(str) + 1);
		data += 2;
		len -= 2;
	}
	abis_push_ipa(nmsg, IPA_PROTO_IPACCESS);

	nmsg2 = abis_msgb_alloc(0);
	if (!nmsg2) {
		msgb_free(nmsg);
		return -ENOMEM;
	}
	*msgb_put(nmsg2, 1) = IPA_MSGT_ID_ACK;
	abis_push_ipa(nmsg2, IPA_PROTO_IPACCESS);

	link->id_resp = 1;

	abis_tx(link, nmsg2);
	return abis_tx(link, nmsg);
}

static int abis_rx_ipaccess(struct ipabis_link *link, struct msgb *msg)
{
	uint8_t *data = msgb_l2(msg);
	int len = msgb_l2len(msg);
	int ret = 0;

	if (len < 1) {
		LOGP(DABIS, LOGL_NOTICE, "Short read of ipaccess message\n");
		msgb_free(msg);
		return EIO;
	}

	switch (*data) {
	case IPA_MSGT_PONG:
#warning HACK
rsl_tx_chan_rqd(link->bts->trx[0]);
		LOGP(DABIS, LOGL_INFO, "PONG\n");
		link->pong = 1;
		break;
	case IPA_MSGT_PING:
		LOGP(DABIS, LOGL_INFO, "reply to ping request\n");
		ret = abis_tx_ipa_pingpong(link, IPA_MSGT_PONG);
		break;
	case IPA_MSGT_ID_GET:
		ret = abis_rx_ipa_id_get(link, data + 1, len - 1);
		break;
	case IPA_MSGT_ID_ACK:
		LOGP(DABIS, LOGL_INFO, "ID ACK\n");
		if (link->id_resp && link->bts)
			ret = bts_link_estab(link->bts);
		if (link->id_resp && link->trx)
			ret = trx_link_estab(link->trx);
		link->id_resp = 0;

		break;
	default:
		LOGP(DABIS, LOGL_NOTICE,
			"Unknown ipaccess message type 0x%02x\n", *data);
		ret = -EINVAL;
	}

	msgb_free(msg);

	return ret;
}

/*
 * A-Bis over IP implementation
 */

/* receive message from BSC */
static int abis_rx(struct ipabis_link *link, struct msgb *msg)
{
	struct ipabis_head *hh = (struct ipabis_head *) msg->data;
	int ret = 0;

	switch (hh->proto) {
	case IPA_PROTO_RSL:
		if (!link->trx) {
			LOGP(DABIS, LOGL_NOTICE,
				"Received RSL message not on RSL link\n");
			msgb_free(msg);
			ret = EIO;
			break;
		}
		ret = down_rsl(link->trx, msg);
		break;
	case IPA_PROTO_IPACCESS:
		ret = abis_rx_ipaccess(link, msg);
		break;
	case IPA_PROTO_SCCP:
		LOGP(DABIS, LOGL_INFO, "Received SCCP message\n");
		msgb_free(msg);
		break;
	case IPA_PROTO_OML:
		if (!link->bts) {
			LOGP(DABIS, LOGL_NOTICE,
				"Received OML message not on OML link\n");
			msgb_free(msg);
			ret = EIO;
			break;
		}
		ret = down_oml(link->bts, msg);
		break;
	default:
		LOGP(DABIS, LOGL_NOTICE, "Unknown Protocol %d\n", hh->proto);
		msgb_free(msg);
		ret = EINVAL;
	}

	return ret;
}

static void abis_timeout(void *arg)
{
	struct ipabis_link *link = arg;
	int ret;

	switch (link->state) {
	case LINK_STATE_RETRYING:
		ret = abis_open(link, link->ip);
		if (ret <= 0)
			bsc_schedule_timer(&link->timer, OML_RETRY_TIMER, 0);
		break;
	case LINK_STATE_CONNECT:
		if (link->ping && !link->pong) {
			LOGP(DABIS, LOGL_NOTICE,
				"No reply to PING. Link is lost\n");
			abis_close(link);
			ret = abis_open(link, link->ip);
			if (ret <= 0) {
				bsc_schedule_timer(&link->timer,
					OML_RETRY_TIMER, 0);
				link->state = LINK_STATE_RETRYING;
			}
			break;
		}
		link->ping = 1;
		link->pong = 0;
		LOGP(DABIS, LOGL_INFO, "PING\n");
		abis_tx_ipa_pingpong(link, IPA_MSGT_PING);
		bsc_schedule_timer(&link->timer, OML_PING_TIMER, 0);
		break;
	}	
}

static int abis_sock_cb(struct bsc_fd *bfd, unsigned int what)
{
	struct ipabis_link *link = bfd->data;
	struct ipabis_head *hh;
	struct msgb *msg;
	int ret = 0;

	if ((what & BSC_FD_WRITE) && link->state == LINK_STATE_CONNECTING) {
		if (link->bts) {
			if (bsc_timer_pending(&link->timer))
				bsc_del_timer(&link->timer);
//			bsc_schedule_timer(&link->timer, OML_PING_TIMER, 0);
#warning HACK
			bsc_schedule_timer(&link->timer, 3, 0);
			link->ping = link->pong = 0;
		}
		LOGP(DABIS, LOGL_INFO, "Abis socket now connected.\n");
		link->state = LINK_STATE_CONNECT;
	}
//printf("what %d\n", what);

	if ((what & BSC_FD_READ)) {
		if (!link->rx_msg) {
			link->rx_msg = msgb_alloc(ABIS_ALLOC_SIZE, "Abis/IP");
			if (!link->rx_msg)
				return -ENOMEM;
		}
		msg = link->rx_msg;
		hh = (struct ipabis_head *) msg->data;
		if (msg->len < sizeof(*hh)) {
			ret = recv(link->bfd.fd, msg->data, sizeof(*hh), 0);
			if (ret <= 0) {
				goto close;
			}
			msgb_put(msg, ret);
			if (msg->len < sizeof(*hh))
				return 0;
			msg->l2h = msg->data + sizeof(*hh);
			if (ntohs(hh->len) > msgb_tailroom(msg)) {
				LOGP(DABIS, LOGL_NOTICE, "Received packet from "
					"Abis socket too large.\n");
				goto close;
			}
		}
		ret = recv(link->bfd.fd, msg->tail,
			ntohs(hh->len) + sizeof(*hh) - msg->len, 0);
		if (ret == 0)
			goto close;
		if (ret < 0 && errno != EAGAIN)
			goto close;
		msgb_put(msg, ret);
		if (ntohs(hh->len) + sizeof(*hh) > msg->len)
			return 0;
		link->rx_msg = NULL;
		LOGP(DABIS, LOGL_INFO, "Received messages from Abis socket.\n");
		ret = abis_rx(link, msg);
	}
	if ((what & BSC_FD_WRITE)) {
		msg = msgb_dequeue(&link->tx_queue);
		if (msg) {
			LOGP(DABIS, LOGL_INFO, "Sending messages to Abis socket.\n");
			ret = send(link->bfd.fd, msg->data, msg->len, 0);
			if (ret < 0)
				goto close;
		} else
			link->bfd.when &= ~BSC_FD_WRITE;
	}
	if ((what & BSC_FD_EXCEPT)) {
		LOGP(DABIS, LOGL_INFO, "Abis socket received exception\n");
		goto close;
	}

	return ret;

close:
	abis_close(link);

	/* RSL link will just close and BSC is notified */
	if (link->trx) {
		LOGP(DABIS, LOGL_INFO, "Connection to BSC failed\n");
		return trx_link_estab(link->trx);
	}

	LOGP(DABIS, LOGL_INFO, "Connection to BSC failed, retrying in %d "
		"seconds.\n", OML_RETRY_TIMER);
	bsc_schedule_timer(&link->timer, OML_RETRY_TIMER, 0);
	link->state = LINK_STATE_RETRYING;

	return 0;
}

int abis_open(struct ipabis_link *link, uint32_t ip)
{
	unsigned int on = 1;
	struct sockaddr_in addr;
	int sock;
	int ret;

	if (link->bfd.fd > 0)
		return -EBUSY;

	INIT_LLIST_HEAD(&link->tx_queue);

	sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sock < 0)
		return sock;

	ret = ioctl(sock, FIONBIO, (unsigned char *)&on);
	if (ret < 0) {
		close(sock);
		return ret;
	}

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	if (link->bts)
		addr.sin_port = htons(IPA_TCP_PORT_OML);
	else
		addr.sin_port = htons(IPA_TCP_PORT_RSL);
	addr.sin_addr.s_addr = htonl(ip);

	ret = connect(sock, (struct sockaddr *)&addr, sizeof(addr));
	if (ret < 0 && errno != EINPROGRESS) {
		close(sock);
		return ret;
	}

	link->bfd.data = link;
	link->bfd.when = BSC_FD_READ | BSC_FD_WRITE | BSC_FD_EXCEPT;
	link->bfd.cb = abis_sock_cb;
	link->bfd.fd = sock;
	link->state = LINK_STATE_CONNECTING;
	link->ip = ip;
	link->timer.cb = abis_timeout;
	link->timer.data = link;

	bsc_register_fd(&link->bfd);

	LOGP(DABIS, LOGL_INFO, "Abis socket trying to reach BSC.\n");

	return sock;
}

void abis_close(struct ipabis_link *link)
{
	struct msgb *msg;
	int i;

	if (link->bfd.fd <= 0)
		return;
	
	LOGP(DABIS, LOGL_INFO, "Abis socket closed.\n");

#warning if any link fails, shutdown all links
	if (link->trx) {
		for (i = 0; i < 8; i++)
			bts_setup_slot(&link->trx->slot[i], 0xff);
	}

	if (link->rx_msg) {
		msgb_free(link->rx_msg);
		link->rx_msg = NULL;
	}

	while ((msg = msgb_dequeue(&link->tx_queue)))
		msgb_free(msg);

	bsc_unregister_fd(&link->bfd);
	
	close(link->bfd.fd);
	link->bfd.fd = -1; /* -1 or 0 indicates: 'close' */
	link->state = LINK_STATE_IDLE;

	if (bsc_timer_pending(&link->timer))
		bsc_del_timer(&link->timer);
}

