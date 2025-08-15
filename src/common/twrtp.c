/*
 * Themyscira Wireless RTP endpoint implementation: main body.
 *
 * This code was contributed to Osmocom Cellular Network Infrastructure
 * project by Mother Mychaela N. Falconia of Themyscira Wireless.
 * Mother Mychaela's contributions are NOT subject to copyright:
 * no rights reserved, all rights relinquished.
 *
 * There also exists a detailed document titled _Guide to ThemWi RTP
 * endpoint library_, located here:
 * https://www.freecalypso.org/TW-doc/twrtp-guide-latest.pdf
 * (See TW-doc directory listing for other formats and previous versions.)
 * This document is required reading for anyone seeking to properly
 * understand this code, and thus be able to modify it without breaking
 * it.  Specific section references to this document will be made
 * in subsequent comments.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h>	/* for network byte order functions */

#include <osmocom/core/msgb.h>
#include <osmocom/core/osmo_io.h>
#include <osmocom/core/socket.h>
#include <osmocom/core/talloc.h>
#include <osmocom/core/timer.h>
#include <osmocom/core/utils.h>

#include <osmocom/netif/rtp.h>

#include <twrtp-local/twrtp.h>
#include <twrtp-local/twjit.h>
#include <twrtp-local/rtcp_defs.h>

/*! \addgroup twrtp
 *  @{
 */

/*! \cond private */

/*! This structure captures info that has been extracted from
 *  received and decoded RTCP SR and/or RR packets.
 *  See twrtp guide document section 5.2.
 */
struct rtcp_rx_state {
	/* sender info extracted from SR packets */
	/*! sender SSRC from the most recently received SR packet */
	uint32_t sr_ssrc;
	/*! NTP timestamp from SR: lower 16 bits of seconds word */
	uint16_t sr_ntp_sec;
	/*! NTP timestamp from SR: upper 16 bits of fraction word */
	uint16_t sr_ntp_fract;
	/*! CLOCK_MONOTONIC time-of-arrival of most recently received SR */
	struct timespec sr_rx_time;
	/* info extracted from reception report blocks in SR/RR packets */
	/*! lost packets word from most recently received RR block */
	uint32_t rr_lost_word;
	/*! interarrival jitter from most recently received RR block */
	uint32_t rr_jitter;
	/*! highest value received in the interarrival jitter word */
	uint32_t rr_jitter_max;
	/* bool flags at the end for structure packing optimization */
	/*! received at least one SR packet */
	bool got_sr;
	/*! received at least one RR block */
	bool got_rr;
};

/*! This structure holds state for emission of RTCP reception report
 *  blocks based on RR info from twjit.  This additional state element
 *  (beyond struct obts_twjit_rr_info) is needed in order to compute
 *  the "fraction lost" field, which is based on the delta since
 *  the last emitted report and thus requires memory of previous
 *  "received" and "expected" counts.
 */
struct rtcp_tx_state {
	uint32_t last_received;
	uint32_t last_expected;
};

/*! Main structure for one instance of twrtp */
struct obts_twrtp {
	/*! UDP socket for RTP data packets */
	struct osmo_io_fd *iofd_rtp;
	/*! UDP socket for RTCP */
	struct osmo_io_fd *iofd_rtcp;
	/*! remote IP:port address for RTP data packets */
	struct osmo_sockaddr rtp_remote;
	/*! remote IP:port address for RTCP */
	struct osmo_sockaddr rtcp_remote;
	/*! count of RTP timestamp units per quantum */
	uint32_t ts_quantum;
	/* scaling factors for RTP Tx timestamp computation */
	/*! RTP timestamp units per second */
	uint32_t ts_units_per_sec;
	/*! divisor to go from nanoseconds to RTP timestamp units */
	uint32_t ns_to_ts_units;
	/* RTP Rx path: twjit and raw options */
	/*! twjit instance owned by the present twrtp instance */
	struct obts_twjit *twjit;
	/*! registered callback for unbuffered/non-delayed Rx path */
	obts_twrtp_raw_rx_cb raw_rx_cb;
	/*! user data for \ref raw_rx_cb */
	void *raw_rx_cb_data;
	/*! RTP Tx state */
	struct {
		/*! random SSRC chosen for RTP output we generate */
		uint32_t ssrc;
		/*! output stream current timestamp */
		uint32_t ts;
		/*! random addend for RTP timestamps we emit */
		uint32_t ts_addend;
		/*! output stream current sequence number */
		uint16_t seq;
		/*! we started output with obts_twrtp_tx_quantum() */
		bool started;
		/*! output timestamp needs to be reset discontinuously */
		bool restart;
	} tx;
	/* RTCP info */
	/*! RTCP Rx state */
	struct rtcp_rx_state rtcp_rx;
	/*! RTCP Tx state */
	struct rtcp_tx_state rtcp_tx;
	/*! buffer holding SDES packet of RFC 3550 section 6.5 */
	uint8_t *sdes_buf;
	/*! length (in bytes) of SDES packet in \ref sdes_buf */
	uint16_t sdes_len;
	/*! automatically emit RTCP SR after this many RTP data packets */
	uint16_t auto_rtcp_interval;
	/*! current count within \ref auto_rtcp_interval */
	uint16_t auto_rtcp_count;
	/*! twrtp-level stats over lifetime of this instance */
	struct obts_twrtp_stats stats;
	/* bool flags at the end for structure packing optimization */
	/*! osmo_iofd_register() done on \ref iofd_rtp and \ref iofd_rtcp */
	bool register_done;
	/*! \ref rtp_remote and \ref rtcp_remote are valid */
	bool remote_set;
	/*! received RTP packets are to be fed to our subordinate twjit */
	bool twjit_rx_enable;
};

/* We need to know maximum expected sizes of RTP and RTCP Rx packets
 * for osmo_io msgb allocation.  For RTP, the largest packet size in
 * 3GPP and IP-PSTN applications is 176 bytes: 12 bytes of RTP header
 * plus 160 bytes of payload for 20 ms of uncompressed G.711 audio
 * or CSData.  Of course there may be other applications that use
 * larger RTP packets, in which case we may have to add an API function
 * that overrides our default msgb alloc size setting - but let's
 * cross that bridge if and when we actually have such users.
 *
 * In case of RTCP, we fully process all received packets inside
 * the present library, hence we can set osmo_io msgb alloc size
 * based on what our RTCP Rx code can parse and make use of.  Any
 * additional RTCP Rx data, such as very long SDES strings, will
 * simply be truncated at osmo_io level - but the subsequent parsing
 * code will never get to those bits anyway.
 */

#define	MAX_RTP_RX_PACKET	(sizeof(struct rtp_hdr) + 160)
#define	MAX_RTCP_RX_PACKET	(sizeof(struct rtcp_sr_rr_hdr) + \
					sizeof(struct rtcp_sr_block) + \
					sizeof(struct rtcp_rr_block) * 31)

/* RTCP includes NTP timestamps, hence we need to convert from Unix-style
 * CLOCK_REALTIME into NTP time format. */

#define	NTP_EPOCH_MJD	15020
#define	UNIX_EPOCH_MJD	40587

#define	NTP_UNIX_EPOCH_DIFF	((UNIX_EPOCH_MJD-NTP_EPOCH_MJD) * 86400UL)
#define	TWO_TO_32_DOUBLE	4294967296.0

/* forward declarations for internal functions */

static void rtp_rx_cb(struct osmo_io_fd *iofd, int res, struct msgb *msg,
		      const struct osmo_sockaddr *saddr);
static void rtcp_rx_cb(struct osmo_io_fd *iofd, int res, struct msgb *msg,
			const struct osmo_sockaddr *saddr);
static int send_rtcp_sr_rr(struct obts_twrtp *endp, bool send_sr,
			   const struct timespec *utc, uint32_t rtp_ts);

static const struct osmo_io_ops twrtp_iops_rtp = {
	.recvfrom_cb = rtp_rx_cb,
};

static const struct osmo_io_ops twrtp_iops_rtcp = {
	.recvfrom_cb = rtcp_rx_cb,
};

/*! \endcond */

/*! Create a twrtp endpoint
 *
 * \param[in] ctx Parent talloc context under which struct obts_twrtp
 * should be allocated.
 * \param[in] clock_khz RTP clock rate in kHz, i.e., number of RTP timestamp
 * units per millisecond. The most common value is 8.
 * \param[in] quantum_ms Duration of a single quantum (unit of speech or data
 * carried in one RTP packet) in milliseconds. The most common value is 20.
 * \param[in] random_ts_seq For RTP packets we generate and emit, randomize
 * not only SSRC for this session, but also the starting timestamp and the
 * starting sequence number. Pass true to satisfy the SHOULD directive in
 * RFC 3550 and for feature parity with ortp, or false for ease of debugging.
 * \param[in] twjit_config If this RTP endpoint is to be equipped with twjit,
 * pass twjit config structure with tunable parameters here. If a sans-twjit
 * RTP endpoint is to be created, pass NULL here.
 * \returns pointer to the newly created twrtp instance, or NULL on errors.
 *
 * Every twrtp endpoint is always capable of sending and receiving RTP and RTCP
 * packets on the IP network, but it may be either twjit-equipped or sans-twjit.
 * The decision to have or not have a twjit instance as part of a newly created
 * twrtp instance must be made at the time of creation with this function, by
 * passing either a valid twjit config structure or NULL as \ref twjit_config.
 *
 * Parameters \ref clock_khz and \ref quantum_ms are passed through to twjit
 * if a twjit instance is created inside the new twrtp instance, but they are
 * mandatory even if no twjit instance is to be created.  With or without twjit,
 * these parameters are used for locally generated RTP packets, i.e., those
 * emitted via obts_twrtp_tx_quantum() as opposed to obts_twrtp_tx_forward().
 */
struct obts_twrtp *
obts_twrtp_create(void *ctx, uint16_t clock_khz, uint16_t quantum_ms,
		  bool random_ts_seq,
		  const struct obts_twjit_config *twjit_config)
{
	struct obts_twrtp *endp;

	endp = talloc_zero(ctx, struct obts_twrtp);
	if (!endp)
		return NULL;

	endp->iofd_rtp = osmo_iofd_setup(endp, -1, NULL,
					 OSMO_IO_FD_MODE_RECVFROM_SENDTO,
					 &twrtp_iops_rtp, endp);
	if (!endp->iofd_rtp) {
		talloc_free(endp);
		return NULL;
	}
	osmo_iofd_set_alloc_info(endp->iofd_rtp, MAX_RTP_RX_PACKET, 0);

	endp->iofd_rtcp = osmo_iofd_setup(endp, -1, NULL,
					  OSMO_IO_FD_MODE_RECVFROM_SENDTO,
					  &twrtp_iops_rtcp, endp);
	if (!endp->iofd_rtcp) {
		osmo_iofd_free(endp->iofd_rtp);
		talloc_free(endp);
		return NULL;
	}
	osmo_iofd_set_alloc_info(endp->iofd_rtcp, MAX_RTCP_RX_PACKET, 0);

	if (twjit_config) {
		endp->twjit = obts_twjit_create(endp, clock_khz, quantum_ms,
						twjit_config);
		if (!endp->twjit) {
			osmo_iofd_free(endp->iofd_rtp);
			osmo_iofd_free(endp->iofd_rtcp);
			talloc_free(endp);
			return NULL;
		}
	}

	endp->ts_quantum = (uint32_t) quantum_ms * clock_khz;
	endp->ts_units_per_sec = (uint32_t) clock_khz * 1000;
	endp->ns_to_ts_units = 1000000 / clock_khz;

	endp->tx.ssrc = random();
	if (random_ts_seq) {
		endp->tx.ts_addend = random();
		endp->tx.seq = random();
	}

	return endp;
}

/*! Destroy a twrtp endpoint
 *
 * \param[in] endp Instance (endpoint) to be freed
 */
void obts_twrtp_destroy(struct obts_twrtp *endp)
{
	if (!endp)
		return;
	osmo_iofd_free(endp->iofd_rtp);
	osmo_iofd_free(endp->iofd_rtcp);
	if (endp->twjit)
		obts_twjit_destroy(endp->twjit);
	talloc_free(endp);
}

/*! Equip twrtp endpoint with RTP and RTCP sockets (supplied file descriptors)
 *
 * \param[in] endp Endpoint to operate on
 * \param[in] rtp_fd OS file descriptor for UDP socket for RTP
 * \param[in] rtcp_fd OS file descriptor for UDP socket for RTCP
 * \returns 0 if successful, negative on errors
 *
 * This function equips a newly created twrtp endpoint with file descriptors
 * for RTP and RTCP sockets.  Most applications will use the high-level API
 * obts_twrtp_bind_local() that creates and binds the right type of sockets,
 * then calls the present function - however, some applications may call this
 * function directly.  In Themyscira Wireless CN environment, there is a
 * separate daemon process that manages the pool of local UDP ports for
 * RTP+RTCP pairs, and that daemon passes allocated sockets to its clients
 * via UNIX domain socket file descriptor passing mechanism - hence twrtp layer
 * must have a public API that takes in already-bound file descriptor pairs.
 *
 * This function always "consumes" the two file descriptors that are passed
 * to it.  If the operation succeeds, each of these fds becomes wrapped in
 * an osmo_io_fd subordinate to struct obts_twrtp, and both will eventually
 * be closed upon obts_twrtp_destroy().  OTOH, if the present function fails,
 * it closes both fds before returning its error indication.  The latter
 * behavior may seem wrong, but it is more convenient for all current users,
 * and consistent with the original twrtp-proto version.  If we get a user
 * application that prefers the other alternative (keeping the fds intact
 * on EBUSY or if osmo_iofd_register() operations fail), we can create
 * another variant of this API with that alternative behavior.
 */
int obts_twrtp_supply_fds(struct obts_twrtp *endp, int rtp_fd, int rtcp_fd)
{
	int rc;

	if (endp->register_done) {
		close(rtp_fd);
		close(rtcp_fd);
		return -EBUSY;
	}

	rc = osmo_iofd_register(endp->iofd_rtp, rtp_fd);
	if (rc < 0) {
		close(rtp_fd);
		close(rtcp_fd);
		return rc;
	}

	rc = osmo_iofd_register(endp->iofd_rtcp, rtcp_fd);
	if (rc < 0) {
		osmo_iofd_close(endp->iofd_rtp);
		close(rtcp_fd);
		return rc;
	}

	endp->register_done = true;
	return 0;
}

/*! Equip twrtp endpoint with locally bound RTP and RTCP sockets
 *
 * \param[in] endp Endpoint to operate on
 * \param[in] rtp_addr IP:port address to be bound locally for RTP socket;
 * the corresponding address for RTCP (port increment by 1) will be derived
 * internally.
 * \returns 0 if successful, negative on errors
 *
 * This function creates a pair of UDP sockets of the right address family
 * (IPv4 or IPv6) for RTP and RTCP, binds them locally and installs them
 * in the twrtp endpoint.  Either the present API or the lower-level
 * alternative obts_twrtp_supply_fds() must be called after obts_twrtp_create()
 * in order for the endpoint to become functional, but neither function can be
 * used again once this step is done.
 */
int obts_twrtp_bind_local(struct obts_twrtp *endp,
			  const struct osmo_sockaddr *rtp_addr)
{
	struct osmo_sockaddr rtcp_addr;
	int rtp_fd, rtcp_fd;

	switch (rtp_addr->u.sa.sa_family) {
	case AF_INET:
		memcpy(&rtcp_addr, rtp_addr, sizeof(struct sockaddr_in));
		rtcp_addr.u.sin.sin_port =
			htons(ntohs(rtp_addr->u.sin.sin_port) + 1);
		break;
	case AF_INET6:
		memcpy(&rtcp_addr, rtp_addr, sizeof(struct sockaddr_in6));
		rtcp_addr.u.sin6.sin6_port =
			htons(ntohs(rtp_addr->u.sin6.sin6_port) + 1);
		break;
	default:
		return -EAFNOSUPPORT;
	}

	rtp_fd = osmo_sock_init_osa(SOCK_DGRAM, IPPROTO_UDP, rtp_addr, NULL,
				    OSMO_SOCK_F_BIND | OSMO_SOCK_F_NONBLOCK);
	if (rtp_fd < 0)
		return rtp_fd;

	rtcp_fd = osmo_sock_init_osa(SOCK_DGRAM, IPPROTO_UDP, &rtcp_addr, NULL,
				     OSMO_SOCK_F_BIND | OSMO_SOCK_F_NONBLOCK);
	if (rtcp_fd < 0) {
		close(rtp_fd);
		return rtcp_fd;
	}

	return obts_twrtp_supply_fds(endp, rtp_fd, rtcp_fd);
}

static void set_remote_ipv4(struct obts_twrtp *endp,
			    const struct sockaddr_in *user_addr)
{
	memcpy(&endp->rtp_remote.u.sin, user_addr, sizeof(struct sockaddr_in));
	memcpy(&endp->rtcp_remote.u.sin, user_addr, sizeof(struct sockaddr_in));
	endp->rtcp_remote.u.sin.sin_port =
		htons(ntohs(user_addr->sin_port) + 1);
}

static void set_remote_ipv6(struct obts_twrtp *endp,
			    const struct sockaddr_in6 *user_addr)
{
	memcpy(&endp->rtp_remote.u.sin6, user_addr,
		sizeof(struct sockaddr_in6));
	memcpy(&endp->rtcp_remote.u.sin6, user_addr,
		sizeof(struct sockaddr_in6));
	endp->rtcp_remote.u.sin6.sin6_port =
		htons(ntohs(user_addr->sin6_port) + 1);
}

/*! Set RTP remote address
 *
 * \param[in] endp Endpoint to operate on
 * \param[in] rtp_addr IP:port address to be set as the remote for RTP socket;
 * the corresponding address for RTCP (port increment by 1) will be derived
 * internally.
 * \returns 0 if successful, negative on errors
 *
 * This function needs to be called at some point in order for the endpoint
 * to become functional, but unlike obts_twrtp_bind_local(), it can be called
 * again to change the remote address as needed.
 */
int obts_twrtp_set_remote(struct obts_twrtp *endp,
			  const struct osmo_sockaddr *rtp_addr)
{
	switch (rtp_addr->u.sa.sa_family) {
	case AF_INET:
		set_remote_ipv4(endp, &rtp_addr->u.sin);
		break;
	case AF_INET6:
		set_remote_ipv6(endp, &rtp_addr->u.sin6);
		break;
	default:
		return -EAFNOSUPPORT;
	}

	endp->remote_set = true;
	return 0;
}

/* RTP Rx path via osmo_io callback */

static void rtp_rx_cb(struct osmo_io_fd *iofd, int res, struct msgb *msg,
		      const struct osmo_sockaddr *saddr)
{
	struct obts_twrtp *endp = osmo_iofd_get_data(iofd);

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
	if (endp->raw_rx_cb) {
		bool swallow;

		swallow = endp->raw_rx_cb(endp, endp->raw_rx_cb_data, msg);
		if (swallow)	/* did the cb consume the msgb we fed it? */
			return;
	}
	if (endp->twjit_rx_enable)
		obts_twjit_input(endp->twjit, msg);
	else
		msgb_free(msg);
}

/* Rx-related API functions */

/*! Enable or disable Rx via twjit
 *
 * \param[in] endp Endpoint to operate on
 * \param[in] rx_enable Self-explanatory Boolean flag
 *
 * This API is valid only for twrtp endpoints that were equipped with twjit
 * at the time of creation.
 */
void obts_twrtp_twjit_rx_ctrl(struct obts_twrtp *endp, bool rx_enable)
{
	OSMO_ASSERT(endp->twjit);
	endp->twjit_rx_enable = rx_enable;
	if (!rx_enable)
		obts_twjit_reset(endp->twjit);
}

/*! Fixed-timing output poll from the twrtp endpoint's twjit buffer
 *
 * \param[in] endp Endpoint to poll
 * \returns pointer to msgb holding a previously received RTP packet that
 * was successfully mapped to the present quantum in the fixed-timing output,
 * or NULL if no such packet is available.
 *
 * This API is valid only for twrtp endpoints that were equipped with twjit
 * at the time of creation.
 */
struct msgb *obts_twrtp_twjit_rx_poll(struct obts_twrtp *endp)
{
	return obts_twjit_output(endp->twjit);
}

/*! Set callback function for unbuffered/non-delayed Rx path
 *
 * \param[in] endp Endpoint to operate on
 * \param[in] cb The callback function, or NULL to cancel this callback
 * mechanism.
 * \param[in] user_data Opaque user data for \ref cb function
 *
 * The callback function set with this API will be called from osmo_io Rx
 * callback path whenever an RTP packet is received.  If the callback function
 * consumes (takes ownership of) the msgb passed to it, it must return true,
 * otherwise it must return false.  If the callback function returns true,
 * osmo_io Rx processing ends there; if it returns false, twrtp's regular
 * osmo_io Rx callback path passes the msgb to twjit if this endpoint is
 * equipped with such and twjit Rx is enabled, or frees the msgb otherwise.
 *
 * It is possible to use twjit and this unbuffered/non-delayed Rx path
 * at the same time.  Consider a speech transcoder that supports AMR codec
 * on the RAN side: such TC will use twjit to feed the incoming RTP stream
 * to the speech decoder function that runs on fixed timing, but the
 * non-delayed Rx path can also be used to "peek" at received RTP packets
 * as they come in and extract the CMR field - to be fed to the speech
 * encoder element, which is separate from the speech decoder fed via twjit.
 */
void obts_twrtp_set_raw_rx_cb(struct obts_twrtp *endp, obts_twrtp_raw_rx_cb cb,
			      void *user_data)
{
	endp->raw_rx_cb = cb;
	endp->raw_rx_cb_data = user_data;
}

/* RTP Tx path */

static uint32_t gen_timestamp(const struct timespec *now,
			      const struct obts_twrtp *endp)
{
	uint32_t ts;

	ts = now->tv_sec * endp->ts_units_per_sec +
	     now->tv_nsec / endp->ns_to_ts_units;
	ts += endp->tx.ts_addend;
	return ts;
}

/*! Emit RTP packet carrying a locally sourced quantum of speech/data
 *
 * \param[in] endp Endpoint to operate on
 * \param[in] payload The payload to emit in RTP, can be NULL iff
 * \ref payload_len == 0.
 * \param[in] payload_len The length of \ref payload in bytes.
 * \param[in] payload_type The payload type number to be emitted in the
 * generated RTP packet.
 * \param[in] marker Value of the M bit to be emitted.
 * \param[in] auto_marker Automatically set the M bit if the packet we are
 * emitting is our very first or follows obts_twrtp_tx_restart().
 * \param[in] send_rtcp Emit RTCP SR along with this RTP data packet.
 * \returns 0 if successful, negative on errors.
 *
 * The design of the library assumes that RTP payloads sent out via this API
 * originate from a fixed timing system such as GSM Um TCH, T1/E1 TDM or a
 * software application driven by a CLOCK_MONOTONIC timerfd, such that once
 * the application calls the present function, subsequent calls to the same
 * will follow every 20 ms (or whatever other quantum duration is set at the
 * time of obts_twrtp_create()) without fail.
 *
 * The M bit will be set in the generated RTP packet if \ref marker argument
 * is true OR if \ref auto_marker is true and the conditions for automatic
 * marker setting are met.
 *
 * RTCP SR packets are emitted by the endpoint only as a result of this
 * function being called, and not along any other path.  An RTCP SR will be
 * emitted if \ref send_rtcp argument is true OR if automatic RTCP SR
 * generation was enabled with obts_twrtp_set_auto_rtcp_interval() and
 * it is time to emit RTCP SR per the count of emitted RTP data packets.
 */
int obts_twrtp_tx_quantum(struct obts_twrtp *endp, const uint8_t *payload,
			  unsigned payload_len, uint8_t payload_type,
			  bool marker, bool auto_marker, bool send_rtcp)
{
	struct msgb *msg;
	struct timespec now;
	uint32_t restart_ts;
	int32_t ts_delta;
	struct rtp_hdr *rtph;
	uint8_t *pl_out;
	int rc;

	if (!endp->register_done || !endp->remote_set)
		return -EINVAL;
	msg = msgb_alloc_c(endp, sizeof(struct rtp_hdr) + payload_len,
			   "ThemWi-RTP-Tx");
	if (!msg) {
		obts_twrtp_tx_skip(endp);
		return -ENOMEM;
	}

	/* timestamp generation is where we do some trickery */
	osmo_clock_gettime(CLOCK_REALTIME, &now);
	if (!endp->tx.started) {
		endp->tx.ts = gen_timestamp(&now, endp);
		endp->tx.started = true;
		endp->tx.restart = false;
		if (auto_marker)
			marker = true;
	} else if (endp->tx.restart) {
		restart_ts = gen_timestamp(&now, endp);
		ts_delta = (int32_t)(restart_ts - endp->tx.ts);
		if (ts_delta <= 0) {
			/* shouldn't happen, unless something funky w/clock */
			endp->tx.ts++;
		} else {
			if (ts_delta % endp->ts_quantum == 0)
				restart_ts++;
			endp->tx.ts = restart_ts;
		}
		endp->tx.restart = false;
		if (auto_marker)
			marker = true;
	}

	rtph = (struct rtp_hdr *) msgb_put(msg, sizeof(struct rtp_hdr));
	rtph->version = RTP_VERSION;
	rtph->padding = 0;
	rtph->extension = 0;
	rtph->csrc_count = 0;
	rtph->marker = marker;
	rtph->payload_type = payload_type;
	rtph->sequence = htons(endp->tx.seq);
	rtph->timestamp = htonl(endp->tx.ts);
	rtph->ssrc = htonl(endp->tx.ssrc);
	pl_out = msgb_put(msg, payload_len);
	memcpy(pl_out, payload, payload_len);
	endp->tx.seq++;
	endp->tx.ts += endp->ts_quantum;

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
		send_rtcp_sr_rr(endp, true, &now,
				endp->tx.ts - endp->ts_quantum);
	}

	return 0;
}

/*! Incur an intentional gap in the emitted RTP stream
 *
 * \param[in] endp Endpoint to operate on
 *
 * Many RTP profiles call for behavior where a stream sender incurs an
 * intentional gap in its output (does not emit the otherwise-expected
 * RTP packet for a given timestamp in the expected cadence of timestamp
 * quantum increments) if the corresponding quantum carries speech silence,
 * or if the data source has errors.  Such operation is non-native to
 * twrtp and generally recommended against (it creates an adverse condition
 * for twjit on the receiving end), but the library provides mechanism
 * rather than policy, hence the ability to incur intentional gaps is
 * supported.  This function advances the output timestamp by one quantum,
 * thereby creating the requested intentional gap.
 */
void obts_twrtp_tx_skip(struct obts_twrtp *endp)
{
	if (!endp->tx.started || endp->tx.restart)
		return;
	endp->tx.ts += endp->ts_quantum;
}

/*! Reset output stream cadence
 *
 * \param[in] endp Endpoint to operate on
 *
 * This function needs to be called if the application wishes to restart
 * or resume output after it previously stopped calling obts_twrtp_tx_quantum()
 * or obts_twrtp_tx_skip() every 20 ms, and it should also be called if
 * the cadence of quantum-sized timestamp increments needs to be broken
 * for some reason.
 */
void obts_twrtp_tx_restart(struct obts_twrtp *endp)
{
	endp->tx.restart = true;
}

/*! Forward RTP packet between endpoints
 *
 * \param[in] endp Endpoint on which the packet should be sent out
 * \param[in] msg RTP packet received from another endpoint
 * \returns 0 if successful, negative on errors
 *
 * If an application needs to forward RTP packets from one endpoint to another
 * without buffering delay, it should call the present function from the
 * callback registered on the other endpoint with obts_twrtp_set_raw_rx_cb().
 *
 * This function always consumes the msgb passed to it - if the sending
 * operation fails, the msgb is freed here.
 */
int obts_twrtp_tx_forward(struct obts_twrtp *endp, struct msgb *msg)
{
	int rc;

	rc = osmo_iofd_sendto_msgb(endp->iofd_rtp, msg, 0, &endp->rtp_remote);
	if (rc < 0)
		msgb_free(msg);
	return rc;
}

/* RTCP Rx path via osmo_io callback */

static void parse_rtcp(struct obts_twrtp *endp, struct msgb *msg)
{
	struct rtcp_rx_state *rxs = &endp->rtcp_rx;
	struct rtcp_sr_rr_hdr *base_hdr;
	struct rtcp_sr_block *sr;
	struct rtcp_rr_block *rr;
	unsigned rc, i;

	if (msg->len < sizeof(struct rtcp_sr_rr_hdr)) {
invalid:	endp->stats.rx_rtcp_invalid++;
		return;
	}
	base_hdr = (struct rtcp_sr_rr_hdr *) msg->data;
	msgb_pull(msg, sizeof(struct rtcp_sr_rr_hdr));
	if ((base_hdr->v_p_rc & 0xC0) != 0x80)
		goto invalid;
	switch (base_hdr->pt) {
	case RTCP_PT_SR:
		if (msg->len < sizeof(struct rtcp_sr_block))
			goto invalid;
		sr = (struct rtcp_sr_block *) msg->data;
		msgb_pull(msg, sizeof(struct rtcp_sr_block));
		rxs->got_sr = true;
		osmo_clock_gettime(CLOCK_MONOTONIC, &rxs->sr_rx_time);
		rxs->sr_ssrc = ntohl(base_hdr->ssrc);
		rxs->sr_ntp_sec = ntohl(sr->ntp_sec);
		rxs->sr_ntp_fract = ntohl(sr->ntp_fract) >> 16;
		break;
	case RTCP_PT_RR:
		break;
	default:
		goto invalid;
	}
	rc = base_hdr->v_p_rc & 0x1F;
	if (msg->len < sizeof(struct rtcp_rr_block) * rc)
		goto invalid;
	for (i = 0; i < rc; i++) {
		rr = (struct rtcp_rr_block *) msg->data;
		msgb_pull(msg, sizeof(struct rtcp_rr_block));
		if (ntohl(rr->ssrc) != endp->tx.ssrc) {
			endp->stats.rx_rtcp_wrong_ssrc++;
			continue;
		}
		rxs->got_rr = true;
		rxs->rr_lost_word = ntohl(rr->lost_word);
		rxs->rr_jitter = ntohl(rr->jitter);
		if (rxs->rr_jitter > rxs->rr_jitter_max)
			rxs->rr_jitter_max = rxs->rr_jitter;
	}
}

static void rtcp_rx_cb(struct osmo_io_fd *iofd, int res, struct msgb *msg,
			const struct osmo_sockaddr *saddr)
{
	struct obts_twrtp *endp = osmo_iofd_get_data(iofd);

	if (!msg)
		return;
	if (!endp->remote_set) {
		msgb_free(msg);
		return;
	}
	if (osmo_sockaddr_cmp(saddr, &endp->rtcp_remote)) {
		endp->stats.rx_rtcp_badsrc++;
		msgb_free(msg);
		return;
	}
	endp->stats.rx_rtcp_pkt++;
	parse_rtcp(endp, msg);
	msgb_free(msg);
}

/* RTCP Tx functions */

static void fill_rr_block(struct obts_twrtp *endp, struct rtcp_rr_block *rr)
{
	struct obts_twjit *twjit = endp->twjit;
	const struct obts_twjit_rr_info *rri = obts_twjit_get_rr_info(twjit);
	const struct rtcp_rx_state *rxs = &endp->rtcp_rx;
	struct rtcp_tx_state *txs = &endp->rtcp_tx;
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

	rr->ssrc = htonl(rri->ssrc);
	rr->lost_word = htonl(lost_word);
	rr->max_seq_ext = htonl(rri->max_seq_ext);
	rr->jitter = htonl(rri->jitter_accum >> 4);

	if (rxs->got_sr && rxs->sr_ssrc == rri->ssrc) {
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

static int send_rtcp_sr_rr(struct obts_twrtp *endp, bool send_sr,
			   const struct timespec *utc, uint32_t rtp_ts)
{
	bool send_rr = false;
	struct msgb *msg;
	struct rtcp_sr_rr_hdr *hdr;
	struct rtcp_sr_block *sr;
	struct rtcp_rr_block *rr;
	uint8_t *sdes_out;
	int rc;

	if (!endp->register_done || !endp->remote_set || !endp->sdes_buf)
		return -EINVAL;
	if (endp->twjit && obts_twjit_got_any_input(endp->twjit))
		send_rr = true;
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

/*! Emit RTCP RR packet
 *
 * \param[in] endp Endpoint to operate on
 * \returns 0 if successful, negative on errors
 *
 * This function is safe to call on any twrtp endpoint, but it will actually
 * result in an RTCP RR packet being emitted only if (1) the twrtp endpoint
 * is equipped with twjit and (2) some RTP data packets have been successfully
 * received and header-decoded.  If these conditions aren't met, no RTCP
 * packet will be emitted and the function will return -ENODATA.
 *
 * This API is rarely needed: in most RTCP-enabled RTP applications,
 * it is more useful to enable automatic SR generation with
 * obts_twrtp_set_auto_rtcp_interval(), in which case the library will emit
 * RTCP SR packets that also include the same reception report block as
 * those standalone RR packets that are emitted by the present function.
 * However, the present API is provided in case an application receives
 * RTP traffic via twrtp+twjit, but does not emit any RTP traffic of its
 * own - in this case only RTCP RR can be generated, not SR.
 */
int obts_twrtp_send_rtcp_rr(struct obts_twrtp *endp)
{
	return send_rtcp_sr_rr(endp, false, NULL, 0);
}

/*! Configure automatic emission of periodic RTCP SR packets
 *
 * \param[in] endp Endpoint to operate on
 * \param[in] interval Automatically emit RTCP SR after this many RTP data
 * packets, or 0 to turn off this mechanism.
 */
void obts_twrtp_set_auto_rtcp_interval(struct obts_twrtp *endp,
					uint16_t interval)
{
	endp->auto_rtcp_interval = interval;
}

/*! Set SDES strings for RTCP SR and RR packet generation
 *
 * \param[in] endp Endpoint to operate on
 * \param[in] cname Per RFC 3550 section 6.5.1
 * \param[in] name Per RFC 3550 section 6.5.2
 * \param[in] email Per RFC 3550 section 6.5.3
 * \param[in] phone Per RFC 3550 section 6.5.4
 * \param[in] loc Per RFC 3550 section 6.5.5
 * \param[in] tool Per RFC 3550 section 6.5.6
 * \param[in] note Per RFC 3550 section 6.5.7
 * \returns 0 if successful, negative on errors
 *
 * RFC 3550 section 6.1 stipulates that every RTCP SR or RR packet also
 * needs to include an SDES block, containing at least a CNAME string.
 * The present function sets the full complement of SDES strings: the
 * mandatory CNAME string and 6 optional ones per RFC 3550.  This function
 * must be called successfully before any RTCP SR or RR packets will be
 * emitted.
 */
int obts_twrtp_set_sdes(struct obts_twrtp *endp, const char *cname,
			const char *name, const char *email, const char *phone,
			const char *loc, const char *tool, const char *note)
{
	uint16_t len_str, len_padded, len_with_hdr, len;
	struct rtcp_sr_rr_hdr *hdr;
	uint8_t *dp;

	if (!cname)
		return -EINVAL;
	len_str = strlen(cname) + 2;
	if (name)
		len_str += strlen(name) + 2;
	if (email)
		len_str += strlen(email) + 2;
	if (phone)
		len_str += strlen(phone) + 2;
	if (loc)
		len_str += strlen(loc) + 2;
	if (tool)
		len_str += strlen(tool) + 2;
	if (note)
		len_str += strlen(note) + 2;
	len_padded = (len_str + 4) & ~3;
	len_with_hdr = len_padded + sizeof(struct rtcp_sr_rr_hdr);

	if (endp->sdes_buf)
		talloc_free(endp->sdes_buf);
	endp->sdes_buf = talloc_size(endp, len_with_hdr);
	if (!endp->sdes_buf)
		return -ENOMEM;

	hdr = (struct rtcp_sr_rr_hdr *) endp->sdes_buf;
	hdr->v_p_rc = 0x81;
	hdr->pt = RTCP_PT_SDES;
	hdr->len = htons(len_with_hdr / 4 - 1);
	hdr->ssrc = htonl(endp->tx.ssrc);
	dp = endp->sdes_buf + sizeof(struct rtcp_sr_rr_hdr);
	*dp++ = SDES_ITEM_CNAME;
	*dp++ = len = strlen(cname);
	memcpy(dp, cname, len);
	dp += len;
	if (name) {
		*dp++ = SDES_ITEM_NAME;
		*dp++ = len = strlen(name);
		memcpy(dp, name, len);
		dp += len;
	}
	if (email) {
		*dp++ = SDES_ITEM_EMAIL;
		*dp++ = len = strlen(email);
		memcpy(dp, email, len);
		dp += len;
	}
	if (phone) {
		*dp++ = SDES_ITEM_PHONE;
		*dp++ = len = strlen(phone);
		memcpy(dp, phone, len);
		dp += len;
	}
	if (loc) {
		*dp++ = SDES_ITEM_LOC;
		*dp++ = len = strlen(loc);
		memcpy(dp, loc, len);
		dp += len;
	}
	if (tool) {
		*dp++ = SDES_ITEM_TOOL;
		*dp++ = len = strlen(tool);
		memcpy(dp, tool, len);
		dp += len;
	}
	if (note) {
		*dp++ = SDES_ITEM_NOTE;
		*dp++ = len = strlen(note);
		memcpy(dp, note, len);
		dp += len;
	}
	memset(dp, 0, len_padded - len_str);

	endp->sdes_len = len_with_hdr;
	return 0;
}

/* retrieving info from reception reports we got from the peer */

/*! Have we received any RTCP RR?
 *
 * \param[in] endp Endpoint to query
 * \returns true if at least one reception report block has been received
 * whose SSRC matches that of our locally generated RTP output,
 * false otherwise.
 */
bool obts_twrtp_got_rtcp_rr(struct obts_twrtp *endp)
{
	return endp->rtcp_rx.got_rr;
}

/*! Info from received RTCP RR: lost packets word
 *
 * \param[in] endp Endpoint to query
 * \returns lost packets word from the most recently received RR block.
 *
 * This API returns the 32-bit word from received RTCP RR in its raw form,
 * exactly as it appears in RFC 3550 section 6.4.1: fraction lost in the
 * upper 8 bits, cumulative number of packets lost in the lower 24 bits.
 *
 * If no RTCP RR has been received, this function returns 0.
 */
uint32_t obts_twrtp_rr_lost_word(struct obts_twrtp *endp)
{
	return endp->rtcp_rx.rr_lost_word;
}

/*! Info from received RTCP RR: cumulative number of packets lost
 *
 * \param[in] endp Endpoint to query
 * \returns "cumulative number of packets lost" value from the most recently
 * received RR block, extracted and converted to a proper signed type. This
 * number can be negative because of the way it is defined in RFC 3550.
 *
 * If no RTCP RR has been received, this function returns 0.
 */
int32_t obts_twrtp_rr_lost_cumulative(struct obts_twrtp *endp)
{
	int32_t lost_count;

	lost_count = endp->rtcp_rx.rr_lost_word & 0xFFFFFF;
	if (lost_count & 0x800000)
		lost_count |= 0xFF000000;
	return lost_count;
}

/*! Info from received RTCP RR: interarrival jitter, most recent
 *
 * \param[in] endp Endpoint to query
 * \returns "interarrival jitter" value from the most recently received
 * RR block.
 */
uint32_t obts_twrtp_rr_jitter_last(struct obts_twrtp *endp)
{
	return endp->rtcp_rx.rr_jitter;
}

/*! Info from received RTCP RR: interarrival jitter, highest received
 *
 * \param[in] endp Endpoint to query
 * \returns "interarrival jitter" value from received RR blocks, the highest
 * value that was received in this session.
 */
uint32_t obts_twrtp_rr_jitter_max(struct obts_twrtp *endp)
{
	return endp->rtcp_rx.rr_jitter_max;
}

/* misc functions */

/*! Get twjit from twrtp
 *
 * \param[in] endp Endpoint to query
 * \returns pointer to twjit instance owned by this twrtp endpoint,
 * or NULL if this twrtp endpoint has no associated twjit.
 *
 * The twjit instance made accessible via this function is still owned
 * by the parent twrtp instance - calling obts_twjit_destroy() on it
 * would cause a crash the next time twrtp tries to use it!  Safe twjit
 * APIs are as follows, for dynamic reconfiguration and information
 * retrieval:
 *
 * obts_twjit_set_config()
 * obts_twjit_get_stats()
 * obts_twjit_get_rr_info()
 * obts_twjit_got_any_input()
 */
struct obts_twjit *obts_twrtp_get_twjit(struct obts_twrtp *endp)
{
	return endp->twjit;
}

/*! Retrieve lifetime stats from twrtp instance
 *
 * \param[in] endp Endpoint to query
 * \returns pointer to lifetime stats structure
 *
 * Note that a twrtp endpoint equipped with twjit has two levels of stats:
 * there are stats at twrtp level and at twjit level.  The present function
 * retrieves twrtp stats; to get twjit stats, call obts_twrtp_get_twjit()
 * followed by obts_twjit_get_stats().
 */
const struct obts_twrtp_stats *obts_twrtp_get_stats(struct obts_twrtp *endp)
{
	return &endp->stats;
}

/*! Retrieve file descriptor for RTP UDP socket
 *
 * \param[in] endp Endpoint to query
 * \returns OS-level file descriptor of the UDP socket used for RTP.
 *
 * The file descriptor made accessible via this function is still owned
 * by the parent twrtp instance - closing it, or otherwise manipulating it
 * too heavily (e.g., doing kernel-level connect on it) will break the library.
 */
int obts_twrtp_get_rtp_fd(struct obts_twrtp *endp)
{
	return osmo_iofd_get_fd(endp->iofd_rtp);
}

/*! Retrieve file descriptor for RTCP UDP socket
 *
 * \param[in] endp Endpoint to query
 * \returns OS-level file descriptor of the UDP socket used for RTCP.
 *
 * The file descriptor made accessible via this function is still owned
 * by the parent twrtp instance - closing it, or otherwise manipulating it
 * too heavily (e.g., doing kernel-level connect on it) will break the library.
 */
int obts_twrtp_get_rtcp_fd(struct obts_twrtp *endp)
{
	return osmo_iofd_get_fd(endp->iofd_rtcp);
}

/*! Set DSCP (Differentiated Services Code Point) for emitted RTP and RTCP
 *  packets
 *
 * \param[in] endp Endpoint to operate on
 * \param[in] dscp DSCP value
 * \returns 0 if successful, negative on errors
 *
 * This function exists for feature parity with osmo_ortp: when migrating
 * from osmo_ortp to obts_twrtp, use this function in the place of
 * osmo_rtp_socket_set_dscp().
 */
int obts_twrtp_set_dscp(struct obts_twrtp *endp, uint8_t dscp)
{
	int rc;

	rc = osmo_sock_set_dscp(osmo_iofd_get_fd(endp->iofd_rtp), dscp);
	if (rc < 0)
		return rc;
	return osmo_sock_set_dscp(osmo_iofd_get_fd(endp->iofd_rtcp), dscp);
}

/*! Set socket priority for emitted RTP and RTCP packets
 *
 * \param[in] endp Endpoint to operate on
 * \param[in] prio Socket priority
 * \returns 0 if successful, negative on errors
 *
 * This function exists for feature parity with osmo_ortp: when migrating
 * from osmo_ortp to obts_twrtp, use this function in the place of
 * osmo_rtp_socket_set_priority().
 */
int obts_twrtp_set_socket_prio(struct obts_twrtp *endp, int prio)
{
	int rc;

	rc = osmo_sock_set_priority(osmo_iofd_get_fd(endp->iofd_rtp), prio);
	if (rc < 0)
		return rc;
	return osmo_sock_set_priority(osmo_iofd_get_fd(endp->iofd_rtcp), prio);
}

/*! @} */
