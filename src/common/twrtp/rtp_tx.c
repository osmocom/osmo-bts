/*
 * Here we implement RTP Tx functionality.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <arpa/inet.h>	/* for network byte order functions */

#include <osmocom/core/msgb.h>
#include <osmocom/core/osmo_io.h>
#include <osmocom/core/timer.h>

#include <themwi/rtp/endp.h>
#include <themwi/rtp/rtp_basic_hdr.h>
#include <themwi/rtp/twjit.h>
#include "endp_internal.h"

static uint32_t gen_timestamp(struct timespec *now,
			      struct twrtp_jibuf_inst *twjit)
{
	uint32_t ts;

	ts = now->tv_sec * twjit->ts_units_per_sec +
	     now->tv_nsec / twjit->ns_to_ts_units;
	return ts;
}

int twrtp_endp_tx_quantum(struct twrtp_endp *endp, const uint8_t *payload,
			  unsigned payload_len, uint8_t payload_type,
			  bool marker, bool auto_marker, bool send_rtcp)
{
	uint32_t ts_quantum = endp->twjit->ts_quantum;
	struct msgb *msg;
	struct timespec now;
	uint32_t restart_ts;
	int32_t ts_delta;
	struct rtp_basic_hdr *rtph;
	uint8_t *pl_out;
	int rc;

	if (!endp->register_done || !endp->remote_set)
		return -EINVAL;
	msg = msgb_alloc_c(endp, sizeof(struct rtp_basic_hdr) + payload_len,
			   "ThemWi-RTP-Tx");
	if (!msg) {
		twrtp_endp_tx_skip(endp);
		return -ENOMEM;
	}

	/* timestamp generation is where we do some trickery */
	osmo_clock_gettime(CLOCK_REALTIME, &now);
	if (!endp->tx.started) {
		endp->tx.ts = gen_timestamp(&now, endp->twjit);
		endp->tx.started = true;
		endp->tx.restart = false;
		if (auto_marker)
			marker = true;
	} else if (endp->tx.restart) {
		restart_ts = gen_timestamp(&now, endp->twjit);
		ts_delta = (int32_t)(restart_ts - endp->tx.ts);
		if (ts_delta <= 0) {
			/* shouldn't happen, unless something funky w/clock */
			endp->tx.ts++;
		} else {
			if (ts_delta % ts_quantum == 0)
				restart_ts++;
			endp->tx.ts = restart_ts;
		}
		endp->tx.restart = false;
		if (auto_marker)
			marker = true;
	}

	rtph = (struct rtp_basic_hdr *)
			msgb_put(msg, sizeof(struct rtp_basic_hdr));
	rtph->v_p_x_cc = 0x80;
	rtph->m_pt = payload_type;
	if (marker)
		rtph->m_pt |= 0x80;
	rtph->ssrc = htonl(endp->tx.ssrc);
	rtph->seq = htons(endp->tx.seq);
	rtph->tstamp = htonl(endp->tx.ts);
	pl_out = msgb_put(msg, payload_len);
	memcpy(pl_out, payload, payload_len);
	endp->tx.seq++;
	endp->tx.ts += ts_quantum;

	rc = osmo_iofd_sendto_msgb(endp->iofd_rtp, msg, 0, &endp->rtp_remote);
	if (rc < 0) {
		msgb_free(msg);
		return rc;
	}
	endp->stats.tx_rtp_pkt++;
	endp->stats.tx_rtp_bytes += payload_len;

	if (endp->auto_rtcp_interval) {
		endp->auto_rtcp_count++;
		if (endp->auto_rtcp_count >= endp->auto_rtcp_interval) {
			endp->auto_rtcp_count = 0;
			send_rtcp = true;
		}
	}
	if (send_rtcp) {
		_twrtp_endp_send_rtcp(endp, true, &now,
				      endp->tx.ts - ts_quantum);
	}

	return 0;
}

void twrtp_endp_tx_skip(struct twrtp_endp *endp)
{
	if (!endp->tx.started || endp->tx.restart)
		return;
	endp->tx.ts += endp->twjit->ts_quantum;
}
