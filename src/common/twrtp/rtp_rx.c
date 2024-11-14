/*
 * Here we implement RTP Rx path via osmo_io callback.
 */

#include <stdint.h>
#include <stdbool.h>

#include <osmocom/core/msgb.h>
#include <osmocom/core/osmo_io.h>
#include <osmocom/core/socket.h>
#include <osmocom/core/utils.h>

#include <themwi/rtp/endp.h>
#include <themwi/rtp/twjit.h>
#include "endp_internal.h"

static void rtp_rx_cb(struct osmo_io_fd *iofd, int res, struct msgb *msg,
		      const struct osmo_sockaddr *saddr)
{
	struct twrtp_endp *endp = osmo_iofd_get_data(iofd);

	if (!msg)
		return;
	if (!endp->remote_set) {
		msgb_free(msg);
		return;
	}
	if (osmo_sockaddr_cmp(saddr, &endp->rtp_remote)) {
		endp->stats.rx_rtp_badsrc++;
		msgb_free(msg);
		return;
	}
	endp->stats.rx_rtp_pkt++;
	if (endp->rx_enable)
		twrtp_jibuf_input(endp->twjit, msg);
	else
		msgb_free(msg);
}

const struct osmo_io_ops _twrtp_endp_iops_rtp = {
	.recvfrom_cb = rtp_rx_cb,
};
