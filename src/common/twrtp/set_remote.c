/*
 * Here we implement functions for setting the remote end on themwi_endp,
 * initially only IPv4, then possibly IPv6 in the future.
 */

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <themwi/rtp/endp.h>

void twrtp_endp_set_remote_ipv4(struct twrtp_endp *endp,
				const struct in_addr *ip, uint16_t port)
{
	endp->rtp_remote.u.sin.sin_family = AF_INET;
	memcpy(&endp->rtp_remote.u.sin.sin_addr, ip, sizeof(struct in_addr));
	endp->rtp_remote.u.sin.sin_port = htons(port);

	endp->rtcp_remote.u.sin.sin_family = AF_INET;
	memcpy(&endp->rtcp_remote.u.sin.sin_addr, ip, sizeof(struct in_addr));
	endp->rtcp_remote.u.sin.sin_port = htons(port + 1);

	endp->remote_set = true;
}
