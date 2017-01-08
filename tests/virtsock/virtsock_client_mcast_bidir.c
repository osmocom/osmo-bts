/* client.c */
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>

#include "mcast_sock.h"

static char *tx_mcast_group = "224.0.0.1";
static int tx_mcast_port = 6666;
static char *rx_mcast_group = "225.0.0.1";
static int rx_mcast_port = 6667;

// main prog for a client program continuously receiving data from a multicast socket and responding to them answers on another
int main(void)
{
	int i = 0;
	char rx_buf[256], tx_buf[256];
	struct mcast_bidir_sock *sock = mcast_bidir_sock_setup(tx_mcast_group,
	                tx_mcast_port, rx_mcast_group, rx_mcast_port);
	if (!sock) {
		perror("Error initializing bidirectional sock.");
	}

	while (++i) {
		printf("Waiting for message NR. %d\n", i);
		if (mcast_bidir_sock_rx(sock, rx_buf, sizeof(rx_buf)) < 0) {
			perror("Error receiving message from server.");
		}
		printf("Received MSG from server: \"%s\"\n", rx_buf);

		strcpy(tx_buf,"ACK: \"");
		strcat(tx_buf, rx_buf);
		strcat(tx_buf, "\"");
		if (mcast_bidir_sock_tx(sock, tx_buf, sizeof(tx_buf)) < 0) {
			perror("Error transmitting message to server.");
		}
		printf("Sent ACK to server: \"%s\"\n", tx_buf);
	}
	if(mcast_bidir_sock_close(sock)) {
		perror("Error closing sockets.");
	}

}
