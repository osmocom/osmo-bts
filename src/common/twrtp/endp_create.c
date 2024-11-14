/*
 * Create and destroy functions for twrtp_endp.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include <osmocom/core/talloc.h>
#include <osmocom/core/osmo_io.h>
#include <osmocom/core/utils.h>

#include <themwi/rtp/endp.h>
#include <themwi/rtp/twjit.h>
#include "endp_internal.h"

struct twrtp_endp *twrtp_endp_create(void *ctx,
				     struct twrtp_jibuf_config *config)
{
	struct twrtp_endp *endp;

	endp = talloc_zero(ctx, struct twrtp_endp);
	if (!endp)
		return NULL;

	endp->iofd_rtp = osmo_iofd_setup(endp, -1, NULL,
					 OSMO_IO_FD_MODE_RECVFROM_SENDTO,
					 &_twrtp_endp_iops_rtp, endp);
	if (!endp->iofd_rtp) {
		talloc_free(endp);
		return NULL;
	}

	endp->iofd_rtcp = osmo_iofd_setup(endp, -1, NULL,
					  OSMO_IO_FD_MODE_RECVFROM_SENDTO,
					  &_twrtp_endp_iops_rtcp, endp);
	if (!endp->iofd_rtcp) {
		osmo_iofd_free(endp->iofd_rtp);
		talloc_free(endp);
		return NULL;
	}

	endp->twjit = twrtp_jibuf_create(endp, config);
	if (!endp->twjit) {
		osmo_iofd_free(endp->iofd_rtp);
		osmo_iofd_free(endp->iofd_rtcp);
		talloc_free(endp);
		return NULL;
	}

	endp->tx.ssrc = random();
	return endp;
}

void twrtp_endp_destroy(struct twrtp_endp *endp)
{
	osmo_iofd_free(endp->iofd_rtp);
	osmo_iofd_free(endp->iofd_rtcp);
	twrtp_jibuf_destroy(endp->twjit);
	talloc_free(endp);
}
