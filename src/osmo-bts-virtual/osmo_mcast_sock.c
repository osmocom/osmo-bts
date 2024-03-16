#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <osmocom/core/socket.h>
#include <osmocom/core/select.h>
#include <osmocom/core/osmo_io.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <talloc.h>
#include <unistd.h>
#include "osmo_mcast_sock.h"

static void noop_write_cb(struct osmo_io_fd *iofd, int res, struct msgb *msg)
{
	/* nothing */
}

static const struct osmo_io_ops srv_ioops = {
	/* no read call-back as we don't read from the socket */
	/* libosmcoore before change-id I0c071a29e508884bac331ada5e510bbfcf440bbf requires write call-back
	 * even if we don't care about it */
	.write_cb = noop_write_cb,
};

/* server socket is what we use for transmission. It is not subscribed
 * to a multicast group or locally bound, but it is just a normal UDP
 * socket that's connected to the remote mcast group + port */
static struct osmo_io_fd *
mcast_server_sock_setup(void *ctx, const char *tx_mcast_group, uint16_t tx_mcast_port, bool loopback)
{
	int rc, fd;
	unsigned int flags = OSMO_SOCK_F_CONNECT | OSMO_SOCK_F_UDP_REUSEADDR;
	struct osmo_io_fd *iofd;

	if (!loopback)
		flags |= OSMO_SOCK_F_NO_MCAST_LOOP;

	/* setup mcast server socket */
	rc = osmo_sock_init(AF_INET, SOCK_DGRAM, IPPROTO_UDP, tx_mcast_group, tx_mcast_port, flags);
	if (rc < 0) {
		perror("Failed to create Multicast Server Socket");
		return NULL;
	}
	fd = rc;

	iofd = osmo_iofd_setup(ctx, rc, "mcast_server_sock", OSMO_IO_FD_MODE_READ_WRITE, &srv_ioops, NULL);
	if (!iofd) {
		close(fd);
		return NULL;
	}

	osmo_iofd_register(iofd, -1);
	return iofd;
}

static void mcast_sock_read_cb(struct osmo_io_fd *iofd, int res, struct msgb *msg)
{
	struct mcast_bidir_sock *bidir_sock = osmo_iofd_get_data(iofd);
	bidir_sock->read_cb(res, msg, bidir_sock->data);
}

const struct osmo_io_ops clnt_ioops = {
	.read_cb = mcast_sock_read_cb,
	/* no write call-back as we don't write to the socket */
};

/* the client socket is what we use for reception.  It is a UDP socket
 * that's bound to the GSMTAP UDP port and subscribed to the respective
 * multicast group */
static struct osmo_io_fd *
mcast_client_sock_setup(void *ctx, const char *mcast_group, uint16_t mcast_port,
			void (*read_cb)(int rc, struct msgb *msg, void *data))
{
	int rc, fd;
	unsigned int flags = OSMO_SOCK_F_BIND | OSMO_SOCK_F_NO_MCAST_ALL | OSMO_SOCK_F_UDP_REUSEADDR;
	struct osmo_io_fd *iofd;

	/* Create mcast client socket */
	rc = osmo_sock_init(AF_INET, SOCK_DGRAM, IPPROTO_UDP, NULL, mcast_port, flags);
	if (rc < 0) {
		perror("Could not create mcast client socket");
		return NULL;
	}
	fd = rc;

	/* Configure and join the multicast group */
	rc = osmo_sock_mcast_subscribe(fd, mcast_group);
	if (rc < 0) {
		perror("Failed to join to mcast goup");
		close(fd);
		return NULL;
	}

	iofd = osmo_iofd_setup(ctx, fd, "mcast_client_sock", OSMO_IO_FD_MODE_READ_WRITE, &clnt_ioops, ctx);
	if (!iofd) {
		close(fd);
		return NULL;
	}

	osmo_iofd_register(iofd, -1);
	return iofd;
}

struct mcast_bidir_sock *
mcast_bidir_sock_setup(void *ctx, const char *tx_mcast_group, uint16_t tx_mcast_port,
			const char *rx_mcast_group, uint16_t rx_mcast_port, bool loopback,
			void (*read_cb)(int rc, struct msgb *msg, void *data),
			void *data)
{
	struct mcast_bidir_sock *bidir_sock = talloc(ctx, struct mcast_bidir_sock);

	if (!bidir_sock)
		return NULL;

	bidir_sock->read_cb = read_cb;
	bidir_sock->data = data;

	bidir_sock->rx_iofd = mcast_client_sock_setup(bidir_sock, rx_mcast_group, rx_mcast_port, read_cb);
	if (!bidir_sock->rx_iofd) {
		talloc_free(bidir_sock);
		return NULL;
	}
	bidir_sock->tx_iofd = mcast_server_sock_setup(bidir_sock, tx_mcast_group, tx_mcast_port, loopback);
	if (!bidir_sock->tx_iofd) {
		osmo_iofd_free(bidir_sock->rx_iofd);
		talloc_free(bidir_sock);
		return NULL;
	}
	return bidir_sock;
}

int mcast_bidir_sock_tx_msg(struct mcast_bidir_sock *bidir_sock, struct msgb *msg)
{
	return osmo_iofd_write_msgb(bidir_sock->tx_iofd, msg);
}

void mcast_bidir_sock_close(struct mcast_bidir_sock *bidir_sock)
{
	osmo_iofd_free(bidir_sock->tx_iofd);
	osmo_iofd_free(bidir_sock->rx_iofd);
	talloc_free(bidir_sock);
}
