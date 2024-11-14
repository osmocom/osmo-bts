/*
 * Implementation of OsmoBTS-internal RTP abstraction layer, allowing
 * both compile-time and runtime selection between ortp and twrtp.
 */

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define	HAVE_ORTP

#include <osmocom/core/logging.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/talloc.h>
#include <osmocom/netif/rtp.h>
#include <themwi/rtp/endp.h>
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
			struct twrtp_jibuf_config *twjit_cfg)
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

	rs->twrtp = twrtp_endp_create(rs, twjit_cfg);
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
		twrtp_endp_destroy(rs->twrtp);
	talloc_free(rs);
}

int rtp_abst_socket_bind(struct rtp_abst_socket *rs, const char *ip, int port)
{
#ifdef HAVE_ORTP
	if (rs->ortp)
		return osmo_rtp_socket_bind(rs->ortp, ip, port);
#endif
	return twrtp_endp_bind_ip_port(rs->twrtp, ip, port);
}

int rtp_abst_socket_connect(struct rtp_abst_socket *rs,
			    const struct in_addr *ip, uint16_t port)
{
#ifdef HAVE_ORTP
	if (rs->ortp)
		return osmo_rtp_socket_connect(rs->ortp, inet_ntoa(*ip), port);
#endif
	twrtp_endp_set_remote_ipv4(rs->twrtp, ip, port);
	return 0;
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
	return twrtp_endp_set_dscp(rs->twrtp, dscp);
}

int rtp_abst_socket_set_priority(struct rtp_abst_socket *rs, uint8_t prio)
{
#ifdef HAVE_ORTP
	if (rs->ortp)
		return osmo_rtp_socket_set_priority(rs->ortp, prio);
#endif
	return twrtp_endp_set_socket_prio(rs->twrtp, prio);
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

	rs->twrtp->rx_enable = true;
	msg = twrtp_jibuf_output(rs->twrtp->twjit);
	if (!msg)
		return;

	rtph = osmo_rtp_get_hdr(msg);
	if (!rtph) {
		LOGP(DRTP, LOGL_ERROR, "got RTP packet with invalid header\n");
		msgb_free(msg);
		return;
	}

	if (rtph->payload_type != rs->payload_type) {
		LOGP(DRTP, LOGL_ERROR,
		     "Rx RTP packet has wrong payload type\n");
		msgb_free(msg);
		return;
	}

	rtp_pl = osmo_rtp_get_payload(rtph, msg, &rtp_pl_len);
	if (!rtp_pl) {
		LOGP(DRTP, LOGL_ERROR,
		     "error retrieving payload from RTP packet\n");
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
	return twrtp_endp_tx_quantum(rs->twrtp, payload, payload_len,
				     rs->payload_type, marker, false, false);
}

int rtp_abst_skipped_frame(struct rtp_abst_socket *rs)
{
#ifdef HAVE_ORTP
	if (rs->ortp)
		return osmo_rtp_skipped_frame(rs->ortp, GSM_RTP_DURATION);
#endif
	twrtp_endp_tx_skip(rs->twrtp);
	return 0;
}

int rtp_abst_get_bound_ip_port(struct rtp_abst_socket *rs,
			       uint32_t *ip, int *port)
{
	struct sockaddr_in sin;
	socklen_t slen;
	int rc;

#ifdef HAVE_ORTP
	if (rs->ortp)
		return osmo_rtp_get_bound_ip_port(rs->ortp, ip, port);
#endif

	slen = sizeof(sin);
	rc = getsockname(rs->twrtp->rtp_fd, (struct sockaddr *) &sin, &slen);
	if (rc < 0)
		return rc;
	if (sin.sin_family != AF_INET)
		return -EIO;
	*ip = ntohl(sin.sin_addr.s_addr);
	*port = ntohs(sin.sin_port);
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

void rtp_abst_socket_log_stats(struct rtp_abst_socket *rs,
				int subsys, int level,
				const char *pfx)
{
#ifdef HAVE_ORTP
	if (rs->ortp) {
		osmo_rtp_socket_log_stats(rs->ortp, subsys, level, pfx);
		return;
	}
#endif
	/* FIXME: implement stats log output for twrtp */
}

static uint32_t compute_lost_count(struct twrtp_endp *twrtp)
{
	struct twrtp_jibuf_inst *twjit = twrtp->twjit;
	struct twrtp_jibuf_rr_info *rri = &twjit->rr_info;
	int32_t cumulative_lost;

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
	struct twrtp_endp *twrtp = rs->twrtp;

#ifdef HAVE_ORTP
	if (rs->ortp) {
		osmo_rtp_socket_stats(rs->ortp, sent_packets, sent_octets,
					recv_packets, recv_octets,
					recv_lost, last_jitter);
		return;
	}
#endif

	*sent_packets = twrtp->stats.tx_rtp_pkt;
	*sent_octets = twrtp->stats.tx_rtp_bytes;
	*recv_packets = twrtp->twjit->stats.rx_packets;
	*recv_octets = rs->twrtp_rx_pl_bytes;
	*recv_lost = compute_lost_count(twrtp);
	*last_jitter = twrtp->twjit->stats.jitter_max;
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
	twrtp_endp_set_sdes(rs->twrtp, cname, name, email, phone,
			    loc, tool, note);
}
