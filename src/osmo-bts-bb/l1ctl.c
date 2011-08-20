/* Interface to layer 1 of baseband */

/* (C) 2010 by Holger Hans Peter Freyther
 * (C) 2010 by Harald Welte <laforge@gnumonks.org>
 * (C) 2011 by Andreas Eversberg <jolly@eversberg.eu>
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

#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>

#include <arpa/inet.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/un.h>

#include <osmocom/core/talloc.h>
#include <osmocom/core/utils.h>
//#include <osmocom/core/select.h>
//#include <osmocom/core/timer.h>
#include <osmocom/core/write_queue.h>
#include <osmocom/gsm/gsm_utils.h>
#include <osmocom/gsm/lapdm.h>

#include <osmo-bts/logging.h>
//#include <osmo-bts/bts.h>
//#include <osmo-bts/oml.h>
//#include <osmo-bts/gsm_data.h>
//#include <osmo-bts/signal.h>

#include "l1ctl.h"
#include "l1_if.h"

/*
 * socket communication with baseband
 */

#define GSM_L2_LENGTH 256
#define GSM_L2_HEADROOM 32

static int l1ctl_recv(struct osmo_l1ctl *l1ctl, struct msgb *msg);

static int l1socket_read(struct osmo_fd *fd)
{
	struct msgb *msg;
	u_int16_t len;
	int rc;

	msg = msgb_alloc_headroom(GSM_L2_LENGTH+GSM_L2_HEADROOM, GSM_L2_HEADROOM, "Layer2");
	if (!msg) {
		LOGP(DL1C, LOGL_ERROR, "Failed to allocate msg.\n");
		return -ENOMEM;
	}

	rc = read(fd->fd, &len, sizeof(len));
	if (rc < sizeof(len)) {
		fprintf(stderr, "Layer2 socket failed\n");
		msgb_free(msg);
		if (rc >= 0)
			rc = -EIO;
		l1socket_close((struct osmo_l1ctl *) fd->data);
		return rc;
	}

	len = ntohs(len);
	if (len > GSM_L2_LENGTH) {
		LOGP(DL1C, LOGL_ERROR, "Length is too big: %u\n", len);
		msgb_free(msg);
		return -EINVAL;
	}


	msg->l1h = msgb_put(msg, len);
	rc = read(fd->fd, msg->l1h, msgb_l1len(msg));
	if (rc != msgb_l1len(msg)) {
		LOGP(DL1C, LOGL_ERROR, "Can not read data: len=%d rc=%d "
		     "errno=%d\n", len, rc, errno);
		msgb_free(msg);
		return rc;
	}

	l1ctl_recv((struct osmo_l1ctl *) fd->data, msg);

	return 0;
}

static int l1socket_write(struct osmo_fd *fd, struct msgb *msg)
{
	int rc;

	if (fd->fd <= 0)
		return -EINVAL;

	rc = write(fd->fd, msg->data, msg->len);
	if (rc != msg->len) {
		LOGP(DL1C, LOGL_ERROR, "Failed to write data: rc: %d\n", rc);
		return rc;
	}

	return 0;
}

int l1socket_open(struct osmo_l1ctl *l1ctl, const char *socket_path)
{
	int rc;
	struct sockaddr_un local;

	l1ctl->l2_wq.bfd.fd = socket(AF_UNIX, SOCK_STREAM, 0);
	if (l1ctl->l2_wq.bfd.fd < 0) {
		fprintf(stderr, "Failed to create unix domain socket.\n");
		return l1ctl->l2_wq.bfd.fd;
	}

	local.sun_family = AF_UNIX;
	strncpy(local.sun_path, socket_path, sizeof(local.sun_path));
	local.sun_path[sizeof(local.sun_path) - 1] = '\0';

	rc = connect(l1ctl->l2_wq.bfd.fd, (struct sockaddr *) &local,
		     sizeof(local));
	if (rc < 0) {
		fprintf(stderr, "Failed to connect to '%s': %s\n", local.sun_path,
			strerror(errno));
		fprintf(stderr, "Please run osmocon for this socket.\n");
		close(l1ctl->l2_wq.bfd.fd);
		return rc;
	}

	osmo_wqueue_init(&l1ctl->l2_wq, 100);
	l1ctl->l2_wq.bfd.data = l1ctl;
	l1ctl->l2_wq.bfd.when = BSC_FD_READ;
	l1ctl->l2_wq.read_cb = l1socket_read;
	l1ctl->l2_wq.write_cb = l1socket_write;

	rc = osmo_fd_register(&l1ctl->l2_wq.bfd);
	if (rc != 0) {
		fprintf(stderr, "Failed to register fd.\n");
		close(l1ctl->l2_wq.bfd.fd);
		return rc;
	}

	return 0;
}

int l1socket_close(struct osmo_l1ctl *l1ctl)
{
	if (l1ctl->l2_wq.bfd.fd <= 0)
		return -EINVAL;

	close(l1ctl->l2_wq.bfd.fd);
	l1ctl->l2_wq.bfd.fd = -1;
	osmo_fd_unregister(&l1ctl->l2_wq.bfd);

	return 0;
}

static int osmo_send_l1(struct osmo_l1ctl *l1ctl, struct msgb *msg)
{
	uint16_t *len;

	DEBUGP(DL1C, "Sending: '%s'\n", osmo_hexdump(msg->data, msg->len));

	if (msg->l1h != msg->data)
		LOGP(DL1C, LOGL_ERROR, "Message L1 header != Message Data\n");
	
	/* prepend 16bit length before sending */
	len = (uint16_t *) msgb_push(msg, sizeof(*len));
	*len = htons(msg->len - sizeof(*len));

	if (osmo_wqueue_enqueue(&l1ctl->l2_wq, msg) != 0) {
		LOGP(DL1C, LOGL_ERROR, "Failed to enqueue msg.\n");
		msgb_free(msg);
		return -1;
	}

	return 0;
}

/*
 * messages to and from layer 1
 */

static struct msgb *osmo_l1ctl_alloc(uint8_t msg_type)
{
	struct l1ctl_hdr *l1h;
	struct msgb *msg = msgb_alloc_headroom(256, 4, "osmo_l1ctl");

	if (!msg) {
		LOGP(DL1C, LOGL_ERROR, "Failed to allocate memory.\n");
		return NULL;
	}

	msg->l1h = msgb_put(msg, sizeof(*l1h));
	l1h = (struct l1ctl_hdr *) msg->l1h;
	l1h->msg_type = msg_type;
	
	return msg;
}

static int l1ctl_reset_req(struct osmo_l1ctl *l1ctl, struct msgb *msg, uint8_t msg_type)
{
	struct l1ctl_hdr *l1h;

	l1h = (struct l1ctl_hdr *) msgb_push(msg, sizeof(*l1h));
	msg->l1h = (uint8_t *) l1h;
	l1h->msg_type = L1CTL_RESET_REQ;

	osmo_send_l1(l1ctl, msg);

	return 0;
}

/* got data from upper L1 interface */
int l1ctl_send(struct osmo_l1ctl *l1ctl, struct msgb *msg)
{
	int rc = 0;
	struct l1if_hdr *l1h;
	uint8_t msg_type;

	l1h = (struct l1if_hdr *) msg->l1h;

	msg_type = l1h->msg_type;
	msg->l1h = NULL;
	msgb_pull(msg, sizeof(*l1h));

	switch (msg_type) {
	case L1IF_RESET_REQ:
		rc = l1ctl_reset_req(l1ctl, msg, msg_type);
		break;
	default:
		LOGP(DL1C, LOGL_ERROR, "Unknown MSG: %u\n", l1h->msg_type);
		msgb_free(msg);
		break;
	}

	return rc;
}
static int l1ctl_reset_ind(struct osmo_l1ctl *l1ctl, struct msgb *msg, uint8_t msg_type)
{
	struct l1if_hdr *l1h;

	if (msg_type == L1CTL_RESET_IND)
		msg_type = L1IF_RESET_IND;
	else
		msg_type = L1IF_RESET_CNF;

	l1h = (struct l1if_hdr *) msgb_push(msg, sizeof(*l1h));
	msg->l1h = (uint8_t *) l1h;
	l1h->msg_type = msg_type;

	return l1if_recv(l1ctl, msg);
}


/* Receive incoming data from L1 using L1CTL format */
static int l1ctl_recv(struct osmo_l1ctl *l1ctl, struct msgb *msg)
{
	int rc = 0;
	struct l1ctl_hdr *l1h;
	struct l1ctl_info_dl *dl;
	uint8_t msg_type;

	if (msgb_l2len(msg) < sizeof(*dl)) {
		LOGP(DL1C, LOGL_ERROR, "Short Layer2 message: %u\n",
			msgb_l2len(msg));
		msgb_free(msg);
		return -1;
	}

	l1h = (struct l1ctl_hdr *) msg->l1h;
	msg_type = l1h->msg_type;
	msg->l1h = NULL;
	msgb_pull(msg, sizeof(*l1h));

	switch (msg_type) {
	case L1CTL_RESET_IND:
	case L1CTL_RESET_CONF:
		rc = l1ctl_reset_ind(l1ctl, msg, msg_type);
		break;
	default:
		LOGP(DL1C, LOGL_ERROR, "Unknown MSG: %u\n", l1h->msg_type);
		msgb_free(msg);
		break;
	}

	return rc;
}
