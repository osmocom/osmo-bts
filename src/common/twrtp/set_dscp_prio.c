/*
 * Wrapper functions for setting DSCP and socket priority
 * on both RTP and RTCP sockets.
 */

#include <stdint.h>

#include <osmocom/core/socket.h>

#include <themwi/rtp/endp.h>

int twrtp_endp_set_dscp(struct twrtp_endp *endp, uint8_t dscp)
{
	int rc;

	rc = osmo_sock_set_dscp(endp->rtp_fd, dscp);
	if (rc < 0)
		return rc;
	return osmo_sock_set_dscp(endp->rtcp_fd, dscp);
}

int twrtp_endp_set_socket_prio(struct twrtp_endp *endp, int prio)
{
	int rc;

	rc = osmo_sock_set_priority(endp->rtp_fd, prio);
	if (rc < 0)
		return rc;
	return osmo_sock_set_priority(endp->rtcp_fd, prio);
}
