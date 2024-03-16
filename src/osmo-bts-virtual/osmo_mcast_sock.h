#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <netinet/in.h>
#include <osmocom/core/select.h>
#include <osmocom/core/osmo_io.h>

struct mcast_bidir_sock {
	struct osmo_io_fd *tx_iofd;
	struct osmo_io_fd *rx_iofd;
	void (*read_cb)(int rc, struct msgb *msg, void *data);
	void *data;
};

struct mcast_bidir_sock *
mcast_bidir_sock_setup(void *ctx, const char *tx_mcast_group, uint16_t tx_mcast_port,
			const char *rx_mcast_group, uint16_t rx_mcast_port, bool loopback,
			void (*read_cb)(int rc, struct msgb *msg, void *data),
			void *data);

int mcast_bidir_sock_tx_msg(struct mcast_bidir_sock *bidir_sock, struct msgb *msg);
void mcast_bidir_sock_close(struct mcast_bidir_sock* bidir_sock);
