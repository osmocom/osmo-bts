#include <osmocom/core/msgb.h>
#include <osmocom/core/select.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "../../src/osmo-bts-virtual/virtual_um.h"

static char *tx_mcast_group = "224.0.0.1";
static int tx_mcast_port = 6666;
static char *rx_mcast_group = "225.0.0.1";
static int rx_mcast_port = 6667;

void virt_um_rx_cb(struct virt_um_inst *vui, struct msgb *msg)
{
	struct msgb *outmsg = msgb_alloc(VIRT_UM_MSGB_SIZE, "Virtual UM Rx");
	char *data_loc, strbuf[VIRT_UM_MSGB_SIZE], addrbuf[32];
	int rc, port;

	inet_ntop(AF_INET, &vui->mcast_sock->rx_sock->mcast_group->imr_multiaddr, addrbuf, 32);

	printf("Received MSG from BTS(%s): \"%s\"\n", addrbuf, msgb_data(msg));

	sprintf(strbuf, "ACK: (%s)", msgb_data(msg));
	data_loc = (char *)msgb_put(outmsg, strlen(strbuf)+1);
	memcpy(data_loc, strbuf, strlen(strbuf)+1);

	inet_ntop(AF_INET, &vui->mcast_sock->tx_sock->sock_conf->sin_addr, addrbuf, 32);
	port = vui->mcast_sock->tx_sock->sock_conf->sin_port;
	rc = virt_um_write_msg(vui, outmsg);
	if (rc < 0) {
		perror("Error sending ACK");
	} else if (rc == 0) {
		printf("Nothing sent: \"%s\"\n", msgb_data(outmsg));
	} else {
		printf("Sent ACK to BTS(%s:%d): \"%s\"\n", addrbuf, port, msgb_data(outmsg));
	}
}

// main prog for a client program continuously receiving data from a multicast socket and responding to them answers on another
int main(void)
{
	int i = 0;
	struct virt_um_inst *vui = virt_um_init(NULL, tx_mcast_group,
	                tx_mcast_port, rx_mcast_group, rx_mcast_port,
	                virt_um_rx_cb);
	if (!vui) {
		perror("Error initializing vui.");
	}
	while (++i < 1000) {
		printf("Waiting for MSG NR. %d\n", i);
		osmo_select_main(0);
		// fallback -> will the communication work without the fd callback?
//		{
//		char rx_buf[256], tx_buf[256];
//		while(mcast_bidir_sock_rx(vui->mcast_sock, rx_buf, sizeof(rx_buf)) < 0) {
//			perror("Error receiving message from server.");
//		}
//		printf("Received MSG from server: \"%s\"\n", rx_buf);
//
//		strcpy(tx_buf,"ACK: \"");
//		strcat(tx_buf, rx_buf);
//		strcat(tx_buf, "\"");
//		if (mcast_bidir_sock_tx(vui->mcast_sock, tx_buf, sizeof(tx_buf)) < 0) {
//			perror("Error transmitting message to server.");
//		}
//		printf("Sent ACK to server: \"%s\"\n", tx_buf);
//		}
	}
	virt_um_destroy(vui);
	return 0;
}
