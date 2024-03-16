/* Routines for a Virtual Um interface over GSMTAP/UDP */

/* (C) 2015 by Harald Welte <laforge@gnumonks.org>
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
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <osmocom/core/select.h>
#include <osmocom/core/utils.h>
#include <osmocom/core/socket.h>
#include <osmocom/core/gsmtap.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/talloc.h>
#include "osmo_mcast_sock.h"
#include "virtual_um.h"

#include <unistd.h>
#include <errno.h>

/**
 * Virtual UM interface file descriptor read callback.
 */
static void virt_um_read_cb(int rc, struct msgb *msg, void *data)
{
	struct virt_um_inst *vui = data;
	msg->l1h = msg->data;

	/* call the l1 callback function for a received msg */
	vui->recv_cb(vui, msg);
}

struct virt_um_inst *virt_um_init(void *ctx, char *tx_mcast_group, uint16_t tx_mcast_port,
				  char *rx_mcast_group, uint16_t rx_mcast_port, int ttl, const char *dev_name,
				  void (*recv_cb)(struct virt_um_inst *vui, struct msgb *msg))
{
	struct virt_um_inst *vui = talloc_zero(ctx, struct virt_um_inst);
	int rc;

	vui->mcast_sock = mcast_bidir_sock_setup(ctx, tx_mcast_group, tx_mcast_port,
						 rx_mcast_group, rx_mcast_port, 1, virt_um_read_cb, vui);
	if (!vui->mcast_sock) {
		perror("Unable to create VirtualUm multicast socket");
		talloc_free(vui);
		return NULL;
	}
	vui->recv_cb = recv_cb;

	/* -1 means default, i.e. no TTL explicitly configured in VTY */
	if (ttl >= 0) {
		int txfd = osmo_iofd_get_fd(vui->mcast_sock->tx_iofd);
		rc = osmo_sock_mcast_ttl_set(txfd, ttl);
		if (rc < 0) {
			perror("Cannot set TTL of Virtual Um transmit socket");
			goto out_close;
		}
	}

	if (dev_name) {
		int txfd = osmo_iofd_get_fd(vui->mcast_sock->tx_iofd);
		rc = osmo_sock_mcast_iface_set(txfd, dev_name);
		if (rc < 0) {
			perror("Cannot bind multicast tx to given device");
			goto out_close;
		}
		int rxfd = osmo_iofd_get_fd(vui->mcast_sock->rx_iofd);
		rc = osmo_sock_mcast_iface_set(rxfd, dev_name);
		if (rc < 0) {
			perror("Cannot bind multicast rx to given device");
			goto out_close;
		}
	}

	return vui;

out_close:
	mcast_bidir_sock_close(vui->mcast_sock);
	talloc_free(vui);
	return NULL;
}

void virt_um_destroy(struct virt_um_inst *vui)
{
	mcast_bidir_sock_close(vui->mcast_sock);
	talloc_free(vui);
}

/**
 * Write msg to to multicast socket and free msg afterwards
 */
int virt_um_write_msg(struct virt_um_inst *vui, struct msgb *msg)
{
	int rc;

	rc = mcast_bidir_sock_tx_msg(vui->mcast_sock, msg);
	if (rc < 0) {
		msgb_free(msg);
		rc = -errno;
	}

	return rc;
}
