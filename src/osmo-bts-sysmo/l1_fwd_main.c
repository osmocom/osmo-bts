/* Sysmocom femtobts L1 proxy */

/* (C) 2011 by Harald Welte <laforge@gnumonks.org>
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

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/stat.h>

#include <arpa/inet.h>
#include <netinet/in.h>

#include <osmocom/core/talloc.h>
#include <osmocom/core/utils.h>
#include <osmocom/core/select.h>
#include <osmocom/core/write_queue.h>
#include <osmocom/core/logging.h>
#include <osmocom/core/socket.h>
#include <osmocom/gsm/gsm_utils.h>

#include <osmo-bts/logging.h>
#include <osmo-bts/gsm_data.h>

#include <sysmocom/femtobts/superfemto.h>
#include <sysmocom/femtobts/gsml1prim.h>
#include <sysmocom/femtobts/gsml1const.h>
#include <sysmocom/femtobts/gsml1types.h>

#include "femtobts.h"
#include "l1_if.h"
#include "l1_transp.h"
#include "l1_fwd.h"

static const uint16_t fwd_udp_ports[_NUM_MQ_WRITE] = {
	[MQ_SYS_READ]	= L1FWD_SYS_PORT,
	[MQ_L1_READ]	= L1FWD_L1_PORT,
};

struct l1fwd_hdl {
	struct sockaddr_storage remote_sa;
	socklen_t remote_sa_len;

	struct osmo_wqueue udp_wq[_NUM_MQ_WRITE];

	struct femtol1_hdl *fl1h;
};


/* callback when there's a new L1 primitive coming in from the HW */
int l1if_handle_l1prim(struct femtol1_hdl *fl1h, struct msgb *msg)
{
	struct l1fwd_hdl *l1fh = fl1h->priv;

	/* Enqueue message to UDP socket */
	return osmo_wqueue_enqueue(&l1fh->udp_wq[MQ_L1_WRITE], msg);
}

/* callback when there's a new SYS primitive coming in from the HW */
int l1if_handle_sysprim(struct femtol1_hdl *fl1h, struct msgb *msg)
{
	struct l1fwd_hdl *l1fh = fl1h->priv;

	/* Enqueue message to UDP socket */
	return osmo_wqueue_enqueue(&l1fh->udp_wq[MQ_SYS_WRITE], msg);
}


/* data has arrived on the udp socket */
static int udp_read_cb(struct osmo_fd *ofd)
{
	struct msgb *msg = msgb_alloc_headroom(sizeof(SuperFemto_Prim_t) + 128, 128, "udp_rx");
	struct l1fwd_hdl *l1fh = ofd->data;
	struct femtol1_hdl *fl1h = l1fh->fl1h;
	int rc;

	if (!msg)
		return -ENOMEM;

	msg->l1h = msg->data;

	l1fh->remote_sa_len = sizeof(l1fh->remote_sa);
	rc = recvfrom(ofd->fd, msg->l1h, msgb_tailroom(msg), 0,
		      (struct sockaddr *) &l1fh->remote_sa, &l1fh->remote_sa_len);
	if (rc < 0) {
		perror("read from udp");
		msgb_free(msg);
		return rc;
	} else if (rc == 0) {
		perror("len=0 read from udp");
		msgb_free(msg);	
		return rc;
	}
	msgb_put(msg, rc);

	DEBUGP(DL1C, "UDP: Received %u bytes for %s queue\n", rc,
		ofd->priv_nr == MQ_SYS_WRITE ? "SYS" : "L1");

	/* put the message into the right queue */
	if (ofd->priv_nr == MQ_SYS_WRITE)
		rc = osmo_wqueue_enqueue(&fl1h->write_q[MQ_SYS_WRITE], msg);
	else
		rc = osmo_wqueue_enqueue(&fl1h->write_q[MQ_L1_WRITE], msg);
	
	return rc;
}

/* callback when we can write to the UDP socket */
static int udp_write_cb(struct osmo_fd *ofd, struct msgb *msg)
{
	int rc;
	struct l1fwd_hdl *l1fh = ofd->data;

	DEBUGP(DL1C, "UDP: Writing %u bytes for %s queue\n", msgb_l1len(msg),
		ofd->priv_nr == MQ_SYS_WRITE ? "SYS" : "L1");

	rc = sendto(ofd->fd, msg->l1h, msgb_l1len(msg), 0,
		    (const struct sockaddr *)&l1fh->remote_sa, l1fh->remote_sa_len);
	if (rc < 0) {
		LOGP(DL1C, LOGL_ERROR, "error writing to L1 msg_queue: %s\n",
			strerror(errno));
		return rc;
	} else if (rc < msgb_l1len(msg)) {
		LOGP(DL1C, LOGL_ERROR, "short write to L1 msg_queue: "
			"%u < %u\n", rc, msgb_l1len(msg));
		return -EIO;
	}

	return 0;
}

int main(int argc, char **argv)
{
	struct l1fwd_hdl *l1fh;
	struct femtol1_hdl *fl1h;
	int rc, i;

	printf("sizeof(GsmL1_Prim_t) = %zu\n", sizeof(GsmL1_Prim_t));
	printf("sizeof(SuperFemto_Prim_t) = %zu\n", sizeof(SuperFemto_Prim_t));

	bts_log_init(NULL);

	/* allocate new femtol1_handle */
	fl1h = talloc_zero(NULL, struct femtol1_hdl);
	INIT_LLIST_HEAD(&fl1h->wlc_list);

	/* open the actual hardware transport */
	rc = l1if_transport_open(fl1h);
	if (rc < 0)
		exit(1);

	/* create our fwd handle */
	l1fh = talloc_zero(NULL, struct l1fwd_hdl);

	l1fh->fl1h = fl1h;
	fl1h->priv = l1fh;

	/* Open UDP */
	for (i = 0; i < 2; i++) {
		struct osmo_wqueue *wq = &l1fh->udp_wq[i];

		osmo_wqueue_init(wq, 10);
		wq->write_cb = udp_write_cb;
		wq->read_cb = udp_read_cb;

		wq->bfd.when |= BSC_FD_READ;
		wq->bfd.data = l1fh;
		wq->bfd.priv_nr = i;
		rc = osmo_sock_init_ofd(&wq->bfd, AF_UNSPEC, SOCK_DGRAM,
					IPPROTO_UDP, NULL, fwd_udp_ports[i],
					OSMO_SOCK_F_BIND);
		if (rc < 0) {
			perror("sock_init");
			exit(1);
		}
	}

	while (1) {
		rc = osmo_select_main(0);		
		if (rc < 0) {
			perror("select");
			exit(1);
		}
	}
	exit(0);
}
