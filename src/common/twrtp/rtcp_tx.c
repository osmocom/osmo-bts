/*
 * Here we implement RTCP Tx functionality.
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
#include <themwi/rtp/rtcp_defs.h>
#include <themwi/rtp/twjit.h>
#include "endp_internal.h"

#define	NTP_EPOCH_MJD	15020
#define	UNIX_EPOCH_MJD	40587

#define	NTP_UNIX_EPOCH_DIFF	((UNIX_EPOCH_MJD-NTP_EPOCH_MJD) * 86400UL)
#define	TWO_TO_32_DOUBLE	4294967296.0

static void fill_rr_block(struct twrtp_endp *endp, struct rtcp_rr_block *rr)
{
	struct twrtp_jibuf_inst *twjit = endp->twjit;
	struct twrtp_jibuf_rr_info *rri = &twjit->rr_info;
	struct twrtp_endp_rtcp_rx *rxs = &endp->rtcp_rx;
	struct twrtp_endp_rtcp_tx *txs = &endp->rtcp_tx;
	uint32_t delta_expect, delta_rcvd;
	int32_t cumulative_lost, newly_lost;
	uint32_t lost_fract, lost_word;
	struct timespec now, time_delta;

	cumulative_lost = (int32_t)(rri->expected_pkt - rri->rx_packets);
	if (cumulative_lost > 0x7FFFFF)
		cumulative_lost = 0x7FFFFF;
	else if (cumulative_lost < -0x800000)
		cumulative_lost = -0x800000;
	delta_expect = rri->expected_pkt - txs->last_expected;
	txs->last_expected = rri->expected_pkt;
	delta_rcvd = rri->rx_packets - txs->last_received;
	txs->last_received = rri->rx_packets;
	newly_lost = (int32_t)(delta_expect - delta_rcvd);
	if (delta_expect == 0 || newly_lost <= 0)
		lost_fract = 0;
	else
		lost_fract = (newly_lost << 8) / delta_expect;
	lost_word = (lost_fract << 8) | (cumulative_lost & 0xFFFFFF);

	rr->ssrc = htonl(twjit->last_ssrc);
	rr->lost_word = htonl(lost_word);
	rr->max_seq_ext = htonl(rri->max_seq_ext);
	rr->jitter = htonl(rri->jitter_accum >> 4);

	if (rxs->got_sr && rxs->sr_ssrc == twjit->last_ssrc) {
		osmo_clock_gettime(CLOCK_MONOTONIC, &now);
		time_delta.tv_sec = now.tv_sec - rxs->sr_rx_time.tv_sec;
		time_delta.tv_nsec = now.tv_nsec - rxs->sr_rx_time.tv_nsec;
		if (time_delta.tv_nsec < 0) {
			time_delta.tv_sec--;
			time_delta.tv_nsec += 1000000000;
		}
		rr->lsr_sec = htons(rxs->sr_ntp_sec);
		rr->lsr_fract = htons(rxs->sr_ntp_fract);
		rr->dlsr_sec = htons(time_delta.tv_sec);
		rr->dlsr_fract = htons(time_delta.tv_nsec / 1000000000.0f *
					65536.0f);
	} else {
		rr->lsr_sec = 0;
		rr->lsr_fract = 0;
		rr->dlsr_sec = 0;
		rr->dlsr_fract = 0;
	}
}

int _twrtp_endp_send_rtcp(struct twrtp_endp *endp, bool send_sr,
			  const struct timespec *utc, uint32_t rtp_ts)
{
	bool send_rr = endp->twjit->got_first_packet;
	struct msgb *msg;
	struct rtcp_sr_rr_hdr *hdr;
	struct rtcp_sr_block *sr;
	struct rtcp_rr_block *rr;
	uint8_t *sdes_out;
	int rc;

	if (!endp->register_done || !endp->remote_set || !endp->sdes_buf)
		return -EINVAL;
	if (!send_sr && !send_rr)
		return -ENODATA;	/* nothing to send, neither SR nor RR */
	msg = msgb_alloc_c(endp, sizeof(struct rtcp_sr_rr_hdr) +
			   sizeof(struct rtcp_sr_block) +
			   sizeof(struct rtcp_rr_block) + endp->sdes_len,
			   "ThemWi-RTCP-Tx");
	if (!msg)
		return -ENOMEM;

	hdr = (struct rtcp_sr_rr_hdr *)
			msgb_put(msg, sizeof(struct rtcp_sr_rr_hdr));
	hdr->v_p_rc = send_rr ? 0x81 : 0x80;
	if (send_sr) {
		hdr->pt = RTCP_PT_SR;
		hdr->len = htons(send_rr ? 12 : 6);
	} else {
		hdr->pt = RTCP_PT_RR;
		hdr->len = htons(7);
	}
	hdr->ssrc = htonl(endp->tx.ssrc);
	if (send_sr) {
		sr = (struct rtcp_sr_block *)
				msgb_put(msg, sizeof(struct rtcp_sr_block));
		sr->ntp_sec = htonl(utc->tv_sec + NTP_UNIX_EPOCH_DIFF);
		sr->ntp_fract = htonl(utc->tv_nsec / 1000000000.0 *
					TWO_TO_32_DOUBLE);
		sr->rtp_ts = htonl(rtp_ts);
		sr->pkt_count = htonl(endp->stats.tx_rtp_pkt);
		sr->octet_count = htonl(endp->stats.tx_rtp_bytes);
	}
	if (send_rr) {
		rr = (struct rtcp_rr_block *)
				msgb_put(msg, sizeof(struct rtcp_rr_block));
		fill_rr_block(endp, rr);
	}
	sdes_out = msgb_put(msg, endp->sdes_len);
	memcpy(sdes_out, endp->sdes_buf, endp->sdes_len);

	rc = osmo_iofd_sendto_msgb(endp->iofd_rtcp, msg, 0, &endp->rtcp_remote);
	if (rc < 0) {
		msgb_free(msg);
		return rc;
	}
	endp->stats.tx_rtcp_pkt++;
	return 0;
}

int twrtp_endp_send_rtcp_rr(struct twrtp_endp *endp)
{
	return _twrtp_endp_send_rtcp(endp, false, NULL, 0);
}
