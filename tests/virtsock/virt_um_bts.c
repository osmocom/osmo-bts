#include <osmocom/core/msgb.h>
#include <osmocom/core/select.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <talloc.h>
#include <unistd.h>

#include "../../src/osmo-bts-virtual/virtual_um.h"

static char *rx_mcast_group = "224.0.0.1";
static int rx_mcast_port = 6666;
static char *tx_mcast_group = "225.0.0.1";
static int tx_mcast_port = 6667;

static void virt_um_rx_cb(struct virt_um_inst *vui, struct msgb *msg)
{
	char addrbuf[32];
	inet_ntop(AF_INET, &vui->mcast_sock->rx_sock->mcast_group->imr_multiaddr, addrbuf, 32);
	printf("Received ACK from MS(%s): \"%s\"\n", addrbuf, msgb_data(msg));
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
		struct msgb *outmsg = msgb_alloc(VIRT_UM_MSGB_SIZE,
		                "Virtual UM Rx");
		char strbuf[64], addrbuf[32], *data_loc;
		int port, rc;
		sprintf(strbuf, "MSG: (%u)", i);
		data_loc = (char *)msgb_put(outmsg, strlen(strbuf)+1);
		memcpy(data_loc, strbuf, strlen(strbuf)+1);

		rc = virt_um_write_msg(vui, outmsg);
		inet_ntop(AF_INET, &vui->mcast_sock->tx_sock->sock_conf->sin_addr, addrbuf, 32);
		port = vui->mcast_sock->tx_sock->sock_conf->sin_port;
		if (rc < 0) {
			perror("Error sending MSG");
		} else if (rc == 0) {
			printf("Nothing sent: \"%s\"\n", msgb_data(outmsg));
		} else {
			printf("Sent MSG to MS(%s:%d): \"%s\"\n", addrbuf, port, msgb_data(outmsg));
		}
		// receive all pending acks
		while(osmo_select_main(1));

		sleep(1);
	}

	virt_um_destroy(vui);
	return 0;
}
