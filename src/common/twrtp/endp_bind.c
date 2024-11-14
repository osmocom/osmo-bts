/*
 * Here we implement the wrapper function that first obtains a pair of fds
 * bound to IP & port for RTP & RTCP, then registers them with twrtp_endp.
 */

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include <themwi/rtp/endp.h>
#include <themwi/rtp/fdpair.h>

int twrtp_endp_bind_ip_port(struct twrtp_endp *endp, const char *ip,
			    uint16_t port)
{
	int rc;

	if (endp->register_done)
		return -EBUSY;

	rc = twrtp_bind_fdpair(ip, port, &endp->rtp_fd, &endp->rtcp_fd);
	if (rc < 0)
		return rc;

	return twrtp_endp_register_fds(endp);
}
