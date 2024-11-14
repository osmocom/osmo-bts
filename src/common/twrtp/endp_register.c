/*
 * Here we implement the step of fd registration in twrtp_endp.
 */

#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>

#include <osmocom/core/osmo_io.h>
#include <osmocom/core/utils.h>

#include <themwi/rtp/endp.h>

int twrtp_endp_register_fds(struct twrtp_endp *endp)
{
	int rc;

	if (endp->register_done)
		return 0;

	rc = osmo_iofd_register(endp->iofd_rtp, endp->rtp_fd);
	if (rc < 0) {
		close(endp->rtp_fd);
		close(endp->rtcp_fd);
		return rc;
	}

	rc = osmo_iofd_register(endp->iofd_rtcp, endp->rtcp_fd);
	if (rc < 0) {
		osmo_iofd_close(endp->iofd_rtp);
		close(endp->rtcp_fd);
		return rc;
	}

	endp->register_done = true;
	return 0;
}
