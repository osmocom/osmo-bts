/*
 * Implementation of OsmoBTS-internal RTP abstraction layer, allowing
 * both compile-time and runtime selection between ortp and twrtp.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define HAVE_ORTP

#include <osmocom/core/logging.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/talloc.h>
#include <osmocom/netif/rtp.h>
#include <osmocom/netif/twrtp.h>
#include <osmocom/netif/twjit.h>
#ifdef HAVE_ORTP
#include <osmocom/trau/osmo_ortp.h>
#endif

#include <osmo-bts/logging.h>
#include <osmo-bts/rtp_abstract.h>

#ifdef HAVE_ORTP
/*! \brief call-back function for incoming RTP from libortp */
static void ortp_rx_cb(struct osmo_rtp_socket *ors, const uint8_t *rtp_pl,
			unsigned int rtp_pl_len, uint16_t seq_number,
			uint32_t timestamp, bool marker)
{
	struct rtp_abst_socket *rs = ors->priv;

	if (rs->rx_cb) {
		rs->rx_cb(rs, rtp_pl, rtp_pl_len, seq_number, timestamp,
			  marker);
	}
}
#endif

struct rtp_abst_socket *
rtp_abst_socket_create(void *talloc_ctx, bool use_twrtp,
			struct osmo_twjit_config *twjit_cfg)
{
	struct rtp_abst_socket *rs;

	rs = talloc_zero(talloc_ctx, struct rtp_abst_socket);
	if (rs == NULL)
		return NULL;

#ifdef HAVE_ORTP
	if (!use_twrtp) {
		rs->ortp = osmo_rtp_socket_create(rs, OSMO_RTP_F_POLL);
		if (rs->ortp == NULL) {
			talloc_free(rs);
			return NULL;
		}
		rs->ortp->priv = rs;
		rs->ortp->rx_cb = ortp_rx_cb;
		return rs;
	}
#endif

	rs->twrtp = osmo_twrtp_create(rs, 8, 20, true, twjit_cfg);
	if (rs->twrtp == NULL) {
		talloc_free(rs);
		return NULL;
	}
	return rs;
}

void rtp_abst_socket_free(struct rtp_abst_socket *rs)
{
#ifdef HAVE_ORTP
	if (rs->ortp)
		osmo_rtp_socket_free(rs->ortp);
#endif
	if (rs->twrtp)
		osmo_twrtp_destroy(rs->twrtp);
	talloc_free(rs);
}

int rtp_abst_socket_bind(struct rtp_abst_socket *rs, const char *ipstr,
			 int port)
{
	int rc;

#ifdef HAVE_ORTP
	if (rs->ortp)
		return osmo_rtp_socket_bind(rs->ortp, ipstr, port);
#endif

	rs->local_addr.sin_family = AF_INET;
	rc = inet_aton(ipstr, &rs->local_addr.sin_addr);
	if (!rc)
		return -EINVAL;
	rs->local_addr.sin_port = htons(port);
	rc = osmo_twrtp_bind_local(rs->twrtp,
			(const struct osmo_sockaddr *) &rs->local_addr, true);
	if (rc < 0)
		return rc;
	return 0;
}

int rtp_abst_socket_connect(struct rtp_abst_socket *rs,
			    const struct in_addr *ip, uint16_t port)
{
	struct sockaddr_in remote;

#ifdef HAVE_ORTP
	if (rs->ortp)
		return osmo_rtp_socket_connect(rs->ortp, inet_ntoa(*ip), port);
#endif

	memset(&remote, 0, sizeof(remote));
	remote.sin_family = AF_INET;
	memcpy(&remote.sin_addr, ip, sizeof(struct in_addr));
	remote.sin_port = htons(port);
	return osmo_twrtp_set_remote(rs->twrtp,
				(const struct osmo_sockaddr *) &remote);
}

int rtp_abst_socket_set_pt(struct rtp_abst_socket *rs, int payload_type)
{
	rs->payload_type = payload_type;
#ifdef HAVE_ORTP
	if (rs->ortp)
		return osmo_rtp_socket_set_pt(rs->ortp, payload_type);
#endif
	return 0;
}

int rtp_abst_socket_set_dscp(struct rtp_abst_socket *rs, int dscp)
{
#ifdef HAVE_ORTP
	if (rs->ortp)
		return osmo_rtp_socket_set_dscp(rs->ortp, dscp);
#endif
	return osmo_twrtp_set_dscp(rs->twrtp, dscp);
}

int rtp_abst_socket_set_priority(struct rtp_abst_socket *rs, uint8_t prio)
{
#ifdef HAVE_ORTP
	if (rs->ortp)
		return osmo_rtp_socket_set_priority(rs->ortp, prio);
#endif
	return osmo_twrtp_set_socket_prio(rs->twrtp, prio);
}

void rtp_abst_socket_poll(struct rtp_abst_socket *rs)
{
	struct msgb *msg;
	struct rtp_hdr *rtph;
	const uint8_t *rtp_pl;
	uint32_t rtp_pl_len;

#ifdef HAVE_ORTP
	if (rs->ortp) {
		osmo_rtp_socket_poll(rs->ortp);
		rs->ortp->rx_user_ts += GSM_RTP_DURATION;
		return;
	}
#endif

	/* This step does not need to be done on every poll, it is only
	 * needed once.  However, once we enable twjit Rx, we have to
	 * commit to regular polling - hence it would be bad to call
	 * twjit Rx enable on socket creation (or bind or connect step),
	 * then have some delay pass before we start polling.  OTOH,
	 * calling twjit Rx enable on every poll is harmless, despite
	 * being unnecessary - and seems like the safest solution
	 * at the moment. */
	osmo_twrtp_twjit_rx_ctrl(rs->twrtp, true);

	rs->twrtp_rx_ticks++;
	msg = osmo_twrtp_twjit_rx_poll(rs->twrtp);
	if (!msg)
		return;

	rtph = osmo_rtp_get_hdr(msg);
	if (!rtph) {
		rs->twrtp_rx_bad_hdr++;
		msgb_free(msg);
		return;
	}

	if (rtph->payload_type != rs->payload_type) {
		rs->twrtp_rx_wrong_pt++;
		msgb_free(msg);
		return;
	}

	rtp_pl = osmo_rtp_get_payload(rtph, msg, &rtp_pl_len);
	if (!rtp_pl) {
		rs->twrtp_pl_extr_errors++;
		msgb_free(msg);
		return;
	}
	rs->twrtp_rx_pl_bytes += rtp_pl_len;

	if (rs->rx_cb) {
		rs->rx_cb(rs, rtp_pl, rtp_pl_len, rtph->sequence,
			  rtph->timestamp, rtph->marker);
	}
	msgb_free(msg);
}

int rtp_abst_send_frame(struct rtp_abst_socket *rs, const uint8_t *payload,
			unsigned int payload_len, bool marker)
{
#ifdef HAVE_ORTP
	if (rs->ortp) {
		return osmo_rtp_send_frame_ext(rs->ortp, payload, payload_len,
						GSM_RTP_DURATION, marker);
	}
#endif
	return osmo_twrtp_tx_quantum(rs->twrtp, payload, payload_len,
				     rs->payload_type, marker, false, false);
}

int rtp_abst_skipped_frame(struct rtp_abst_socket *rs)
{
#ifdef HAVE_ORTP
	if (rs->ortp)
		return osmo_rtp_skipped_frame(rs->ortp, GSM_RTP_DURATION);
#endif
	osmo_twrtp_tx_skip(rs->twrtp);
	return 0;
}

int rtp_abst_get_bound_ip_port(struct rtp_abst_socket *rs,
			       uint32_t *ip, int *port)
{
#ifdef HAVE_ORTP
	if (rs->ortp)
		return osmo_rtp_get_bound_ip_port(rs->ortp, ip, port);
#endif
	*ip = ntohl(rs->local_addr.sin_addr.s_addr);
	*port = ntohs(rs->local_addr.sin_port);
	return 0;
}

int rtp_abst_socket_set_param(struct rtp_abst_socket *rs,
			      bool jitter_adaptive, int jitter_ms)
{
#ifdef HAVE_ORTP
	if (rs->ortp) {
		return osmo_rtp_socket_set_param(rs->ortp,
				jitter_adaptive ? OSMO_RTP_P_JIT_ADAP
						: OSMO_RTP_P_JITBUF,
				jitter_ms);
	}
#endif
	return 0;	/* not applicable to twrtp */
}

void rtp_abst_socket_log_stats(struct rtp_abst_socket *rs, const char *cause)
{
	struct osmo_twjit *twjit;
	const struct osmo_twrtp_stats *twrtp_stats;
	const struct osmo_twjit_stats *twjit_stats;

#ifdef HAVE_ORTP
	if (rs->ortp) {
		char prefix[80];

		snprintf(prefix, sizeof(prefix), "Closing RTP socket on %s ",
			 cause);
		osmo_rtp_socket_log_stats(rs->ortp, DRTP, LOGL_INFO, prefix);
		return;
	}
#endif

	twjit = osmo_twrtp_get_twjit(rs->twrtp);
	twrtp_stats = osmo_twrtp_get_stats(rs->twrtp);
	twjit_stats = osmo_twjit_get_stats(twjit);

	LOGP(DRTP, LOGL_INFO, "RTP session complete with %s\n", cause);
	/* normal Rx stats */
	LOGP(DRTP, LOGL_INFO, "Rx active for %u ticks\n", rs->twrtp_rx_ticks);
	LOGP(DRTP, LOGL_INFO, "twjit rx_packets=%u delivered_pkt=%u\n",
	     twjit_stats->rx_packets, twjit_stats->delivered_pkt);
	if (twjit_stats->handovers_in || twjit_stats->handovers_out) {
		LOGP(DRTP, LOGL_INFO,
		     "twjit handovers_in=%u handovers_out=%u\n",
		     twjit_stats->handovers_in, twjit_stats->handovers_out);
	}
	if (twjit_stats->too_old) {
		LOGP(DRTP, LOGL_INFO, "twjit too_old=%u\n",
		     twjit_stats->too_old);
	}
	if (twjit_stats->underruns) {
		LOGP(DRTP, LOGL_INFO, "twjit underruns=%u\n",
		     twjit_stats->underruns);
	}
	if (twjit_stats->ho_underruns) {
		LOGP(DRTP, LOGL_INFO, "twjit ho_underruns=%u\n",
		     twjit_stats->ho_underruns);
	}
	if (twjit_stats->soft_underruns) {
		LOGP(DRTP, LOGL_INFO, "twjit soft_underruns=%u\n",
		     twjit_stats->soft_underruns);
	}
	if (twjit_stats->output_gaps) {
		LOGP(DRTP, LOGL_INFO, "twjit output_gaps=%u\n",
		     twjit_stats->output_gaps);
	}
	if (twjit_stats->thinning_drops) {
		LOGP(DRTP, LOGL_INFO, "twjit thinning_drops=%u\n",
		     twjit_stats->thinning_drops);
	}
	if (twjit_stats->duplicate_ts) {
		LOGP(DRTP, LOGL_INFO, "twjit duplicate_ts=%u\n",
		     twjit_stats->duplicate_ts);
	}
	if (twjit_stats->ssrc_changes) {
		LOGP(DRTP, LOGL_INFO, "twjit ssrc_changes=%u\n",
		     twjit_stats->ssrc_changes);
	}
	if (twjit_stats->seq_skips) {
		LOGP(DRTP, LOGL_INFO, "twjit seq_skips=%u\n",
		     twjit_stats->seq_skips);
	}
	if (twjit_stats->seq_backwards) {
		LOGP(DRTP, LOGL_INFO, "twjit seq_backwards=%u\n",
		     twjit_stats->seq_backwards);
	}
	if (twjit_stats->seq_repeats) {
		LOGP(DRTP, LOGL_INFO, "twjit seq_repeats=%u\n",
		     twjit_stats->seq_repeats);
	}
	if (twjit_stats->intentional_gaps) {
		LOGP(DRTP, LOGL_INFO, "twjit intentional_gaps=%u\n",
		     twjit_stats->intentional_gaps);
	}
	if (twjit_stats->ts_resets) {
		LOGP(DRTP, LOGL_INFO, "twjit ts_resets=%u\n",
		     twjit_stats->ts_resets);
	}
	LOGP(DRTP, LOGL_INFO, "Rx max jitter %u.%03u ms\n",
	     twjit_stats->jitter_max >> 3, (twjit_stats->jitter_max & 7) * 125);
	/* Rx error stats */
	if (twrtp_stats->rx_rtp_badsrc) {
		LOGP(DRTP, LOGL_ERROR, "Rx %u RTP packets from wrong src\n",
		     twrtp_stats->rx_rtp_badsrc);
	}
	if (twrtp_stats->rx_rtcp_badsrc) {
		LOGP(DRTP, LOGL_ERROR, "Rx %u RTCP packets from wrong src\n",
		     twrtp_stats->rx_rtcp_badsrc);
	}
	if (twjit_stats->bad_packets) {
		LOGP(DRTP, LOGL_ERROR, "Rx %u bad RTP packets (twjit)\n",
		     twjit_stats->bad_packets);
	}
	if (rs->twrtp_rx_bad_hdr) {
		LOGP(DRTP, LOGL_ERROR, "Rx %u bad RTP packets (after twjit)\n",
		     rs->twrtp_rx_bad_hdr);
	}
	if (rs->twrtp_rx_wrong_pt) {
		LOGP(DRTP, LOGL_ERROR, "Rx wrong pt number (%u times)\n",
		     rs->twrtp_rx_wrong_pt);
	}
	if (rs->twrtp_pl_extr_errors) {
		LOGP(DRTP, LOGL_ERROR,
		     "Rx failed to extract payload (%u times)\n",
		     rs->twrtp_pl_extr_errors);
	}
	if (twrtp_stats->rx_rtcp_invalid) {
		LOGP(DRTP, LOGL_ERROR, "Rx %u bad RTCP packets\n",
		     twrtp_stats->rx_rtcp_invalid);
	}
	if (twrtp_stats->rx_rtcp_wrong_ssrc) {
		LOGP(DRTP, LOGL_ERROR, "Rx RTCP RR for wrong SSRC (%u times)\n",
		     twrtp_stats->rx_rtcp_wrong_ssrc);
	}
	/* Tx stats */
	LOGP(DRTP, LOGL_INFO, "Tx %u RTP packets\n", twrtp_stats->tx_rtp_pkt);
}

static uint32_t compute_lost_count(struct osmo_twjit *twjit)
{
	const struct osmo_twjit_rr_info *rri;
	int32_t cumulative_lost;

	rri = osmo_twjit_get_rr_info(twjit);
	cumulative_lost = (int32_t)(rri->expected_pkt - rri->rx_packets);
	if (cumulative_lost < 0)
		cumulative_lost = 0;
	return cumulative_lost;
}

void rtp_abst_socket_stats(struct rtp_abst_socket *rs,
				uint32_t *sent_packets, uint32_t *sent_octets,
				uint32_t *recv_packets, uint32_t *recv_octets,
				uint32_t *recv_lost, uint32_t *last_jitter)
{
	struct osmo_twjit *twjit;
	const struct osmo_twrtp_stats *twrtp_stats;
	const struct osmo_twjit_stats *twjit_stats;

#ifdef HAVE_ORTP
	if (rs->ortp) {
		osmo_rtp_socket_stats(rs->ortp, sent_packets, sent_octets,
					recv_packets, recv_octets,
					recv_lost, last_jitter);
		return;
	}
#endif

	twjit = osmo_twrtp_get_twjit(rs->twrtp);
	twrtp_stats = osmo_twrtp_get_stats(rs->twrtp);
	twjit_stats = osmo_twjit_get_stats(twjit);

	*sent_packets = twrtp_stats->tx_rtp_pkt;
	*sent_octets = twrtp_stats->tx_rtp_bytes;
	*recv_packets = twjit_stats->rx_packets;
	*recv_octets = rs->twrtp_rx_pl_bytes;
	*recv_lost = compute_lost_count(twjit);
	*last_jitter = twjit_stats->jitter_max;
}

void rtp_abst_set_source_desc(struct rtp_abst_socket *rs, const char *cname,
				const char *name, const char *email,
				const char *phone, const char *loc,
				const char *tool, const char *note)
{
#ifdef HAVE_ORTP
	if (rs->ortp) {
		osmo_rtp_set_source_desc(rs->ortp, cname, name, email, phone,
					 loc, tool, note);
		return;
	}
#endif
	osmo_twrtp_set_sdes(rs->twrtp, cname, name, email, phone,
			    loc, tool, note);
}
