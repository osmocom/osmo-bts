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
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <net/if.h>

#include <osmocom/core/select.h>
#include <osmocom/core/utils.h>
#include <osmocom/core/socket.h>
#include <osmocom/core/gsmtap.h>

#include "virtual_um.h"

#define VIRT_UM_MSGB_SIZE	256

static int mcast_join_group(int fd, const char *group, const char *netdev)
{
	int ifindex = 0;
	int rc, af;
	socklen_t af_len = sizeof(af);

	if (netdev) {
		ifindex = if_nametoindex(netdev);
		if (!ifindex)
			return -1;
	}

	rc = getsockopt(fd, SOL_SOCKET, SO_DOMAIN, &af, &af_len);
	if (rc < 0)
		return rc;

	switch (af) {
	case AF_INET:
		{
			struct ip_mreqn mr;
			memset(&mr, 0, sizeof(mr));
			inet_pton(AF_INET, group, &mr.imr_multiaddr);
			if (ifindex)
				mr.imr_ifindex = ifindex;
			rc = setsockopt(fd, SOL_SOCKET, IP_ADD_MEMBERSHIP,
					&mr, sizeof(mr));
		}
		break;
	case AF_INET6:
		{
			struct ipv6_mreq mr;
			memset(&mr, 0, sizeof(mr));
			inet_pton(AF_INET6, group, &mr.ipv6mr_multiaddr);
			if (ifindex)
				mr.ipv6mr_interface = ifindex;
			rc = setsockopt(fd, SOL_SOCKET, IPV6_ADD_MEMBERSHIP,
					&mr, sizeof(mr));

		}
		break;
	default:
		rc = -1;
		break;
	}

	return rc;
}

static int mcast_connect(int fd, const char *group, uint16_t port)
{
	int rc, af;
	socklen_t af_len = sizeof(af);
	struct sockaddr_in sin;
	struct sockaddr_in6 sin6;

	rc = getsockopt(fd, SOL_SOCKET, SO_DOMAIN, &af, &af_len);
	if (rc < 0)
		return rc;

	switch (af) {
	case AF_INET:
		memset(&sin, 0, sizeof(sin));
		sin.sin_family = AF_INET;
		sin.sin_port = htons(port);
		inet_pton(AF_INET, group, &sin.sin_addr);
		rc = connect(fd, (struct sockaddr *) &sin, sizeof(sin));
		break;
	case AF_INET6:
		memset(&sin6, 0, sizeof(sin6));
		sin6.sin6_family = AF_INET6;
		sin6.sin6_port = htons(port);
		inet_pton(AF_INET6, group, &sin6.sin6_addr);
		rc = connect(fd, (struct sockaddr *) &sin6, sizeof(sin6));
		break;
	default:
		return -1;
	}

	return rc;
}

static int virt_um_fd_cb(struct osmo_fd *ofd, unsigned int what)
{
	struct virt_um_inst *vui = ofd->data;

	if (what & BSC_FD_READ) {
		struct msgb *msg = msgb_alloc(VIRT_UM_MSGB_SIZE, "Virtual UM Rx");
		int rc;

		rc = read(ofd->fd, msgb_data(msg), msgb_tailroom(msg));
		if (rc > 0) {
			msgb_put(msg, rc);
			vui->recv_cb(vui, msg);
		} else {
			vui->recv_cb(vui, NULL);
			osmo_fd_unregister(ofd);
			close(ofd->fd);
			ofd->fd = -1;
			ofd->when = 0;
		}
	}

	return 0;
}

struct virt_um_inst *virt_um_init(void *ctx, const char *group, uint16_t port,
				  const char *netdev, void *priv,
				  void (*recv_cb)(struct virt_um_inst *vui, struct msgb *msg))
{
	struct virt_um_inst *vui;
	int fd, rc;

	if (!port)
		port = GSMTAP_UDP_PORT;
	if (!group)
		group = "239.0.47.29";

	/* crate a socked and bind it to the multicast group. Do NOT
	 * specify a fixed port locally, to make stack choose a random
	 * free UDP port. */
	fd = osmo_sock_init(AF_UNSPEC, SOCK_DGRAM, IPPROTO_UDP, group,
			    0, OSMO_SOCK_F_BIND);
	if (fd < 0)
		return NULL;

	/* join the multicast group */
	rc = mcast_join_group(fd, group, netdev);
	if (rc < 0) {
		close(fd);
		return NULL;
	}

	/* and finally also connect */
	rc = mcast_connect(fd, group, port);
	if (rc < 0) {
		close(fd);
		return NULL;
	}

	vui = talloc_zero(ctx, struct virt_um_inst);
	vui->priv = priv;
	vui->recv_cb = recv_cb;
	vui->ofd.data = vui;
	vui->ofd.fd = fd;
	vui->ofd.when = BSC_FD_READ;
	vui->ofd.cb = virt_um_fd_cb;

	osmo_fd_register(&vui->ofd);

	return vui;
}

void virt_um_destroy(struct virt_um_inst *vui)
{
	struct osmo_fd *ofd = &vui->ofd;

	osmo_fd_unregister(ofd);
	close(ofd->fd);
	ofd->fd = -1;
	ofd->when = 0;

	talloc_free(vui);
}

int virt_um_write_msg(struct virt_um_inst *vui, struct msgb *msg)
{
	int rc;

	rc = write(vui->ofd.fd, msgb_data(msg), msgb_length(msg));
	msgb_free(msg);

	return rc;
}
