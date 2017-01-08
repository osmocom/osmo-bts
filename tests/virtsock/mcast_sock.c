#include <netinet/in.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <bits/sockaddr.h>

#include "mcast_sock.h"

struct mcast_server_sock *mcast_server_sock_setup(char* tx_mcast_group,
                                                  int tx_mcast_port,
                                                  int loopback)
{
	struct mcast_server_sock *serv_sock = malloc(
	                sizeof(struct mcast_server_sock));
	struct sockaddr_in *tx_mcast_sock_conf = malloc(
	                sizeof(struct sockaddr_in));

	// setup mcast server socket
	serv_sock->sd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (serv_sock->sd == -1) {
		perror("Failed to create Multicast Socket.\n");
		return NULL;
	}
	memset(tx_mcast_sock_conf, 0, sizeof(*tx_mcast_sock_conf));
	tx_mcast_sock_conf->sin_family = AF_INET;
	tx_mcast_sock_conf->sin_addr.s_addr = inet_addr(tx_mcast_group);
	tx_mcast_sock_conf->sin_port = htons(tx_mcast_port);
	serv_sock->sock_conf = tx_mcast_sock_conf;

	// disable loopback
	if (loopback) {
		char loop = 0;
		if (setsockopt(serv_sock->sd, IPPROTO_IP,
		IP_MULTICAST_LOOP, &loop, sizeof(loop)) < 0) {
			perror("Failed to disable loopback.\n");
			return NULL;
		}
	}

//	// setup rx unix domain socket
//	serv_sock->sd = socket(AF_LOCAL, SOCK_STREAM, IPPROTO_UDP);
//	if (serv_sock->sd < 0) {
//		perror("Failed to create Unix Domain Socket.\n");
//		return NULL;
//	}
//	rx_sock_conf->sun_family = AF_LOCAL;
//	strcpy(rx_sock_conf->sun_path, rx_unix_path);
//	unlink(rx_sock_conf->sun_path);
//
//	if (bind(serv_sock->sd, (struct sockaddr *)&rx_sock_conf,
//	                sizeof(rx_sock_conf)) != 0) {
//		perror("Failed to bind the unix domain socket. '%s'\n",
//		                rx_sock_conf->sun_path);
//		return NULL;
//	}
//
//	if (listen(serv_sock->sd, 0) != 0) {
//		perror("Failed to listen.\n");
//		return NULL;
//	}
	return serv_sock;
}

struct mcast_client_sock *mcast_client_sock_setup(char* mcast_group,
                                                  int mcast_port)
{
	struct mcast_client_sock *client_sock = malloc(
	                sizeof(struct mcast_client_sock));
	struct sockaddr_in *rx_sock_conf = malloc(sizeof(struct sockaddr_in));

	struct ip_mreq *group_conf = malloc(sizeof(struct ip_mreq));
	int rc, opt = 1;

	// setup mcast client socket
	client_sock->sd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (client_sock->sd == -1) {
		return NULL;
	}

	memset(rx_sock_conf, 0, sizeof(*rx_sock_conf));
	rx_sock_conf->sin_family = AF_INET;
	// INADDR_ANY -> don't specify an interface, let OS choose the proper one
	rx_sock_conf->sin_addr.s_addr = htonl(INADDR_ANY);
	rx_sock_conf->sin_port = htons(mcast_port);
	client_sock->sock_conf = rx_sock_conf;

	/* let multiple processes use the same port */
	rc = setsockopt(client_sock->sd,
	SOL_SOCKET,
	SO_REUSEADDR, &opt, sizeof(opt));
	if (rc < 0) {
		return NULL;
	}
	rc = bind(client_sock->sd, (struct sockaddr *)client_sock->sock_conf,
	                sizeof(*client_sock->sock_conf));
	if (rc < 0) {
		return NULL;
	}

	/* configure broadcast on this machine */
	opt = 1;
	rc = setsockopt(client_sock->sd,
	IPPROTO_IP,
	IP_MULTICAST_LOOP, &opt, sizeof(opt));
	if (rc < 0) {
		return NULL;
	}

	/* configure the broadcast group */
	group_conf->imr_multiaddr.s_addr = inet_addr(mcast_group);
	group_conf->imr_interface.s_addr = htonl(INADDR_ANY);
	if (group_conf->imr_multiaddr.s_addr == -1) {
		return NULL;
	}
	client_sock->mcast_group = group_conf;

	/* join the broadcast group */
	rc = setsockopt(client_sock->sd,
	IPPROTO_IP,
	IP_ADD_MEMBERSHIP, client_sock->mcast_group,
	                sizeof(*client_sock->mcast_group));
	if (rc < 0) {
		return NULL;
	}

//	// setup tx unix domain socket
//	client_sock->tx_sd = socket(AF_LOCAL, SOCK_STREAM, IPPROTO_UDP);
//	if (client_sock->tx_sd < 0) {
//		perror("Failed to create Unix Domain Socket.\n");
//		return NULL;
//	}
//	tx_sock_conf.sun_family = AF_LOCAL;
//	strcpy(tx_sock_conf.sun_path, tx_unix_path);
//	unlink(tx_sock_conf.sun_path);
//
//	if (bind(client_sock->tx_sd, (struct sockaddr *)&tx_sock_conf,
//	                sizeof(tx_sock_conf)) != 0) {
//		perror("Failed to bind the unix domain socket. '%s'\n",
//		                tx_sock_conf.sun_path);
//		return NULL;
//	}
//
//	if ((client_sock->tx_sd, (struct sockaddr *)&tx_sock_conf, sizeof(tx_sock_conf))
//	                != 0) {
//		perror("Failed to listen.\n");
//		return NULL;
//	}

	return client_sock;
}

struct mcast_bidir_sock *mcast_bidir_sock_setup(char* tx_mcast_group,
                                                int tx_mcast_port,
                                                char* rx_mcast_group,
                                                int rx_mcast_port)
{
	struct mcast_bidir_sock *bidir_sock = malloc(
	                sizeof(struct mcast_bidir_sock));
	bidir_sock->rx_sock = mcast_client_sock_setup(rx_mcast_group,
	                rx_mcast_port);
	bidir_sock->tx_sock = mcast_server_sock_setup(tx_mcast_group,
	                tx_mcast_port, 0);
	if (!bidir_sock->rx_sock || !bidir_sock->tx_sock) {
		return NULL;
	}
	return bidir_sock;

}

int mcast_client_sock_rx(struct mcast_client_sock *client_sock, void* buf,
                         int buf_len)
{
	return recv(client_sock->sd, buf, buf_len, 0);
}

int mcast_server_sock_tx(struct mcast_server_sock *serv_sock, void* data,
                         int data_len)
{
	return sendto(serv_sock->sd, data, data_len, 0,
	                (struct sockaddr *)serv_sock->sock_conf,
	                sizeof(*serv_sock->sock_conf));
}

int mcast_bidir_sock_tx(struct mcast_bidir_sock *bidir_sock, void* data,
                        int data_len)
{
	return mcast_server_sock_tx(bidir_sock->tx_sock, data, data_len);
}
int mcast_bidir_sock_rx(struct mcast_bidir_sock *bidir_sock, void* buf,
                        int buf_len)
{
	return mcast_client_sock_rx(bidir_sock->rx_sock, buf, buf_len);
}

int mcast_client_sock_close(struct mcast_client_sock *client_sock)
{
	free(client_sock->mcast_group);
	free(client_sock->sock_conf);
	free(client_sock);
	return setsockopt(client_sock->sd,
	IPPROTO_IP,
	IP_DROP_MEMBERSHIP, client_sock->mcast_group,
	                sizeof(*client_sock->mcast_group))
	                + close(client_sock->sd);

}
int mcast_server_sock_close(struct mcast_server_sock *serv_sock)
{
	free(serv_sock->sock_conf);
	free(serv_sock);
	return close(serv_sock->sd) + close(serv_sock->sd);
}

int mcast_bidir_sock_close(struct mcast_bidir_sock *bidir_sock)
{
	free(bidir_sock);
	return mcast_client_sock_close(bidir_sock->rx_sock)
	                + mcast_server_sock_close(bidir_sock->tx_sock);

}
