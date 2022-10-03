/* Osmux related routines & logic */

/* (C) 2022 by sysmocom - s.m.f.c. GmbH <info@sysmocom.de>
 * All Rights Reserved
 * Author: Pau Espin Pedrol <pespin@sysmocom.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */


#include <errno.h>
#include <sys/socket.h>
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>

#include <osmocom/core/logging.h>
#include <osmocom/core/utils.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/socket.h>
#include <osmocom/netif/rtp.h>

#include <osmo-bts/bts.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/osmux.h>
#include <osmo-bts/lchan.h>
#include <osmo-bts/msg_utils.h>
#include <osmo-bts/l1sap.h>

/* Bitmask containing Allocated Osmux circuit ID. +7 to round up to 8 bit boundary. */
static uint8_t osmux_cid_bitmap[OSMO_BYTES_FOR_BITS(OSMUX_CID_MAX + 1)];

/*! Find and reserve a free OSMUX cid.
 *  \returns OSMUX cid */
static int osmux_get_local_cid(void)
{
	int i, j;

	for (i = 0; i < sizeof(osmux_cid_bitmap); i++) {
		for (j = 0; j < 8; j++) {
			if (osmux_cid_bitmap[i] & (1 << j))
				continue;

			osmux_cid_bitmap[i] |= (1 << j);
			LOGP(DOSMUX, LOGL_DEBUG,
			     "Allocating Osmux CID %u from pool\n", (i * 8) + j);
			return (i * 8) + j;
		}
	}

	LOGP(DOSMUX, LOGL_ERROR, "All Osmux circuits are in use!\n");
	return -1;
}

/*! put back a no longer used OSMUX cid.
 *  \param[in] osmux_cid OSMUX cid */
void osmux_put_local_cid(uint8_t osmux_cid)
{
	LOGP(DOSMUX, LOGL_DEBUG, "Osmux CID %u is back to the pool\n", osmux_cid);
	osmux_cid_bitmap[osmux_cid / 8] &= ~(1 << (osmux_cid % 8));
}

/* Deliver OSMUX batch to the remote end */
static void osmux_deliver_cb(struct msgb *batch_msg, void *data)
{
	struct osmux_handle *handle = data;
	struct gsm_bts *bts = handle->bts;
	socklen_t dest_len;
	ssize_t rc;

	switch (handle->rem_addr.u.sa.sa_family) {
	case AF_INET6:
		dest_len = sizeof(handle->rem_addr.u.sin6);
		break;
	case AF_INET:
	default:
		dest_len = sizeof(handle->rem_addr.u.sin);
		break;
	}
	rc = sendto(bts->osmux.fd.fd, batch_msg->data, batch_msg->len, 0,
		(struct sockaddr *)&handle->rem_addr.u.sa, dest_len);
	if (rc < 0) {
		char errbuf[129];
		strerror_r(errno, errbuf, sizeof(errbuf));
		LOGP(DOSMUX, LOGL_ERROR, "osmux sendto(%s) failed: %s\n",
			 osmo_sockaddr_to_str(&handle->rem_addr), errbuf);
	}
	msgb_free(batch_msg);
}

/* Lookup existing OSMUX handle for specified destination address. */
static struct osmux_handle *osmux_handle_find_get(const struct gsm_bts *bts,
						  const struct osmo_sockaddr *rem_addr)
{
	struct osmux_handle *h;

	llist_for_each_entry(h, &bts->osmux.osmux_handle_list, head) {
		if (osmo_sockaddr_cmp(&h->rem_addr, rem_addr) == 0) {
			LOGP(DOSMUX, LOGL_DEBUG,
			     "Using existing OSMUX handle for rem_addr=%s\n",
				osmo_sockaddr_to_str(rem_addr));
			h->refcnt++;
			return h;
		}
	}

	return NULL;
}

/* Put down no longer needed OSMUX handle */
static void osmux_handle_put(struct gsm_bts *bts, struct osmux_in_handle *in)
{
	struct osmux_handle *h;

	llist_for_each_entry(h, &bts->osmux.osmux_handle_list, head) {
		if (h->in == in) {
			if (--h->refcnt == 0) {
				LOGP(DOSMUX, LOGL_INFO,
				     "Releasing unused osmux handle for %s\n",
				     osmo_sockaddr_to_str(&h->rem_addr));
				llist_del(&h->head);
				TALLOC_FREE(h->in);
				talloc_free(h);
			}
			return;
		}
	}
	LOGP(DOSMUX, LOGL_ERROR, "Cannot find Osmux input handle %p\n", in);
}

/* Allocate free OSMUX handle */
static struct osmux_handle *osmux_handle_alloc(struct gsm_bts *bts, const struct osmo_sockaddr *rem_addr)
{
	struct osmux_handle *h;

	h = talloc_zero(bts, struct osmux_handle);
	if (!h)
		return NULL;
	h->bts = bts;
	h->rem_addr = *rem_addr;
	h->refcnt++;

	h->in = osmux_xfrm_input_alloc(h);
	if (!h->in) {
		talloc_free(h);
		return NULL;
	}
	/* sequence number to start OSMUX message from */
	osmux_xfrm_input_set_initial_seqnum(h->in, 0);
	osmux_xfrm_input_set_batch_factor(h->in, bts->osmux.batch_factor);
	/* If batch size is zero, the library defaults to 1472 bytes. */
	osmux_xfrm_input_set_batch_size(h->in, bts->osmux.batch_size);
	osmux_xfrm_input_set_deliver_cb(h->in, osmux_deliver_cb, h);

	llist_add(&h->head, &bts->osmux.osmux_handle_list);

	LOGP(DOSMUX, LOGL_DEBUG, "Created new OSMUX handle for rem_addr=%s\n",
		osmo_sockaddr_to_str(rem_addr));

	return h;
}

/* Lookup existing handle for a specified address, if the handle can not be
 * found, the function will automatically allocate one */
static struct osmux_in_handle *
osmux_handle_find_or_create(struct gsm_bts *bts, const struct osmo_sockaddr *rem_addr)
{
	struct osmux_handle *h;

	if (rem_addr->u.sa.sa_family != AF_INET) {
		LOGP(DOSMUX, LOGL_DEBUG, "IPv6 not supported in osmux yet!\n");
		return NULL;
	}

	h = osmux_handle_find_get(bts, rem_addr);
	if (h != NULL)
		return h->in;

	h = osmux_handle_alloc(bts, rem_addr);
	if (h == NULL)
		return NULL;

	return h->in;
}


static struct msgb *osmux_recv(struct osmo_fd *ofd, struct osmo_sockaddr *addr)
{
	struct msgb *msg;
	socklen_t slen = sizeof(addr->u.sas);
	int ret;

	msg = msgb_alloc(4096, "OSMUX"); /* TODO: pool? */
	if (!msg) {
		LOGP(DOSMUX, LOGL_ERROR, "cannot allocate message\n");
		return NULL;
	}
	ret = recvfrom(ofd->fd, msg->data, msg->data_len, 0, &addr->u.sa, &slen);
	if (ret <= 0) {
		msgb_free(msg);
		LOGP(DOSMUX, LOGL_ERROR, "cannot receive message\n");
		return NULL;
	}
	msgb_put(msg, ret);

	return msg;
}

static struct gsm_lchan *osmux_lchan_find(struct gsm_bts *bts, const struct osmo_sockaddr *rem_addr, uint8_t osmux_cid)
{
	/* TODO: Optimize this by maintaining a hashmap local_cid->lchan in bts */
	struct gsm_bts_trx *trx;

	llist_for_each_entry(trx, &bts->trx_list, list) { /* C0..n */
		unsigned int tn;
		for (tn = 0; tn < ARRAY_SIZE(trx->ts); tn++) {
			struct gsm_bts_trx_ts *ts = &trx->ts[tn];
			uint8_t subslot, subslots;
			if (!ts_is_tch(ts))
				continue;

			subslots = ts_subslots(ts);
			for (subslot = 0; subslot < subslots; subslot++) {
				struct gsm_lchan *lchan = &ts->lchan[subslot];
				struct osmux_handle *h;
				if (!lchan->abis_ip.osmux.use)
					continue;
				if (lchan->abis_ip.osmux.local_cid != osmux_cid)
					continue;
				h = osmux_xfrm_input_get_deliver_cb_data(lchan->abis_ip.osmux.in);
				if (osmo_sockaddr_cmp(&h->rem_addr, rem_addr) != 0)
					continue;
				return lchan; /* Found it! */
			}
		}
	}
	return NULL;
}

static int osmux_read_fd_cb(struct osmo_fd *ofd, unsigned int what)
{
	struct msgb *msg;
	struct osmux_hdr *osmuxh;
	struct osmo_sockaddr rem_addr;
	struct gsm_bts *bts = ofd->data;

	msg = osmux_recv(ofd, &rem_addr);
	if (!msg)
		return -1;

	while ((osmuxh = osmux_xfrm_output_pull(msg)) != NULL) {
		struct gsm_lchan *lchan = osmux_lchan_find(bts, &rem_addr, osmuxh->circuit_id);
		if (!lchan) {
			LOGP(DOSMUX, LOGL_NOTICE,
			     "Cannot find lchan for circuit_id=%d\n",
			     osmuxh->circuit_id);
			continue;
		}
		osmux_xfrm_output_sched(lchan->abis_ip.osmux.out, osmuxh);
	}
	msgb_free(msg);
	return 0;
}

/* Called before config file read, set defaults */
int bts_osmux_init(struct gsm_bts *bts)
{
	bts->osmux.use = OSMUX_USAGE_OFF;
	bts->osmux.local_addr = talloc_strdup(bts, "127.0.0.1");
	bts->osmux.local_port = OSMUX_DEFAULT_PORT;
	bts->osmux.batch_factor = 4;
	bts->osmux.batch_size = OSMUX_BATCH_DEFAULT_MAX;
	bts->osmux.dummy_padding = false;
	INIT_LLIST_HEAD(&bts->osmux.osmux_handle_list);
	return 0;
}

void bts_osmux_release(struct gsm_bts *bts)
{
	/* FIXME: not needed? YES,we probably need to iterare over
	   bts->osmux.osmux_handle_list and free everything there, see
	   osmux_handle_put() */
}

/* Called after config file read, start services */
int bts_osmux_open(struct gsm_bts *bts)
{
	int rc;

	/* If Osmux is not enabled by VTY, don't initialize stuff */
	if (bts->osmux.use == OSMUX_USAGE_OFF)
		return 0;

	bts->osmux.fd.cb = osmux_read_fd_cb;
	bts->osmux.fd.data = bts;
	rc = osmo_sock_init2_ofd(&bts->osmux.fd, AF_UNSPEC, SOCK_DGRAM, IPPROTO_UDP,
				 bts->osmux.local_addr, bts->osmux.local_port,
				 NULL, 0, OSMO_SOCK_F_BIND);
	if (rc < 0) {
		LOGP(DOSMUX, LOGL_ERROR,
		     "Failed binding Osmux socket to %s:%u\n",
		     bts->osmux.local_addr ? : "*", bts->osmux.local_port);
		return rc;
	}

	LOGP(DOSMUX, LOGL_INFO,
	     "Osmux socket listening on %s:%u\n",
	     bts->osmux.local_addr ? : "*", bts->osmux.local_port);

	osmo_bts_set_feature(bts->features, BTS_FEAT_OSMUX);
	return rc;
}

static struct msgb *osmux_rtp_msgb_alloc_cb(void *rtp_msgb_alloc_priv_data,
					    unsigned int msg_len)
{
	struct msgb *msg;
	msg = l1sap_msgb_alloc(msg_len);
	/* We have size for "struct osmo_phsap_prim" reserved & aligned at the
	 * start of the msg. Osmux will start filling RTP Header at the tail.
	 * Later on, when pushing it down the stack (scheduled_from_osmux_tx_rtp_cb)
	 * we'll want to get rid of the RTP header and have RTP payload
	 * immediately follow the the struct osmo_phsap_prim. Hence, we rework
	 * reserved space so that end of RTP header (12 bytes) filled by Osmux
	 * ends up at the same position where "struct osmo_phsap_prim" currently
	 * ends up */
	msg->l2h = msgb_get(msg, sizeof(struct rtp_hdr));
	return msg;
}

static void scheduled_from_osmux_tx_rtp_cb(struct msgb *msg, void *data)
{
	struct gsm_lchan *lchan = data;
	struct rtp_hdr *rtph;

	/* if we're in loopback mode, we don't accept frames from the
	 * RTP socket anymore */
	if (lchan->loopback) {
		msgb_free(msg);
		return;
	}

	/* This is where start of rtp_hdr was prepared in osmux_rtp_msgb_alloc_cb() */
	rtph = (struct rtp_hdr *)msg->l2h;
	if (msgb_l2len(msg) < sizeof(*rtph)) {
		LOGPLCHAN(lchan, DOSMUX, LOGL_ERROR, "received RTP frame too short (len = %d)\n",
			  msgb_l2len(msg));
		msgb_free(msg);
		return;
	}

	/* Store RTP header Marker bit in control buffer */
	rtpmsg_marker_bit(msg) = rtph->marker;
	/* Store RTP header Sequence Number in control buffer */
	rtpmsg_seq(msg) = ntohs(rtph->sequence);
	/* Store RTP header Timestamp in control buffer */
	rtpmsg_ts(msg) = ntohl(rtph->timestamp);

	/* No need to pull() rtph out of msg here, because it was written inside
	 * initial space reserved for "struct osmo_phsap_prim". We need to pull
	 * the whole "struct osmo_phsap_prim" since it will be pushed and filled
	 * by lower layers:
	 */
	msgb_pull(msg, sizeof(struct osmo_phsap_prim));

	/* enqueue making sure the queue doesn't get too long */
	lchan_dl_tch_queue_enqueue(lchan, msg, 16);
}

int lchan_osmux_init(struct gsm_lchan *lchan, uint8_t rtp_payload)
{
	struct gsm_bts_trx *trx = lchan->ts->trx;
	int local_cid = osmux_get_local_cid();
	struct in_addr ia;

	if (local_cid < 0)
		return local_cid;

	if (inet_pton(AF_INET, trx->bts->osmux.local_addr, &ia) != 1)
		return -1;

	lchan->abis_ip.osmux.out = osmux_xfrm_output_alloc(trx);
	osmux_xfrm_output_set_rtp_ssrc(lchan->abis_ip.osmux.out, random() /*TODO: SSRC */);
	osmux_xfrm_output_set_rtp_pl_type(lchan->abis_ip.osmux.out, rtp_payload);
	osmux_xfrm_output_set_tx_cb(lchan->abis_ip.osmux.out, scheduled_from_osmux_tx_rtp_cb, lchan);
	osmux_xfrm_output_set_rtp_msgb_alloc_cb(lchan->abis_ip.osmux.out, osmux_rtp_msgb_alloc_cb, lchan);

	lchan->abis_ip.bound_ip = ntohl(ia.s_addr);
	lchan->abis_ip.bound_port = trx->bts->osmux.local_port;
	lchan->abis_ip.osmux.local_cid = local_cid;
	lchan->abis_ip.osmux.rtpst = osmo_rtp_handle_create(trx);
	lchan->abis_ip.osmux.use = true;
	return 0;
}

void lchan_osmux_release(struct gsm_lchan *lchan)
{
	struct gsm_bts *bts = lchan->ts->trx->bts;
	OSMO_ASSERT(lchan->abis_ip.osmux.use);
	/* We are closing, we don't need pending RTP packets to be transmitted */
	osmux_xfrm_output_set_tx_cb(lchan->abis_ip.osmux.out, NULL, NULL);
	TALLOC_FREE(lchan->abis_ip.osmux.out);

	msgb_queue_free(&lchan->dl_tch_queue);
	lchan->dl_tch_queue_len = 0;

	osmux_put_local_cid(lchan->abis_ip.osmux.local_cid);

	/* Now the remote / tx part, if ever set (connected): */
	if (lchan->abis_ip.osmux.in) {
		osmux_xfrm_input_close_circuit(lchan->abis_ip.osmux.in,
					       lchan->abis_ip.osmux.remote_cid);
		osmux_handle_put(bts, lchan->abis_ip.osmux.in);
		lchan->abis_ip.osmux.in = NULL;
	}
	if (lchan->abis_ip.osmux.rtpst)
		osmo_rtp_handle_free(lchan->abis_ip.osmux.rtpst);

	lchan->abis_ip.osmux.use = false;
}

bool lchan_osmux_connected(const struct gsm_lchan *lchan)
{
	return lchan->abis_ip.osmux.in != NULL;
}

int lchan_osmux_connect(struct gsm_lchan *lchan)
{
	struct osmo_sockaddr rem_addr;
	struct gsm_bts *bts = lchan->ts->trx->bts;
	OSMO_ASSERT(lchan->abis_ip.connect_ip != 0);
	OSMO_ASSERT(lchan->abis_ip.connect_port != 0);

	memset(&rem_addr, 0, sizeof(rem_addr));
	rem_addr.u.sa.sa_family = AF_INET;
	rem_addr.u.sin.sin_addr.s_addr = lchan->abis_ip.connect_ip;
	rem_addr.u.sin.sin_port = htons(lchan->abis_ip.connect_port);
	lchan->abis_ip.osmux.in = osmux_handle_find_or_create(bts, &rem_addr);
	if (!lchan->abis_ip.osmux.in) {
		LOGPLCHAN(lchan, DOSMUX, LOGL_ERROR, "Cannot allocate input osmux handle\n");
		return -1;
	}
	if (osmux_xfrm_input_open_circuit(lchan->abis_ip.osmux.in,
					  lchan->abis_ip.osmux.remote_cid,
					  bts->osmux.dummy_padding) < 0) {
		LOGPLCHAN(lchan, DOSMUX, LOGL_ERROR, "Cannot open osmux circuit %u\n",
			  lchan->abis_ip.osmux.remote_cid);
		osmux_handle_put(bts, lchan->abis_ip.osmux.in);
		lchan->abis_ip.osmux.in = NULL;
		return -1;
	}
	return 0;
}

/* Create RTP packet from l1sap payload and feed it to osmux */
int lchan_osmux_send_frame(struct gsm_lchan *lchan, const uint8_t *payload,
			   unsigned int payload_len, unsigned int duration, bool marker)
{
	struct msgb *msg;
	struct rtp_hdr *rtph;
	int rc;

	msg = osmo_rtp_build(lchan->abis_ip.osmux.rtpst, lchan->abis_ip.rtp_payload,
			     payload_len, payload, duration);
	if (!msg)
		return -1;

	/* Set marker bit: */
	rtph = (struct rtp_hdr *)msgb_data(msg);
	rtph->marker = marker;

	while ((rc = osmux_xfrm_input(lchan->abis_ip.osmux.in, msg,
				      lchan->abis_ip.osmux.remote_cid)) > 0) {
		/* batch full, build and deliver it */
		osmux_xfrm_input_deliver(lchan->abis_ip.osmux.in);
	}
	return 0;
}

int lchan_osmux_skipped_frame(struct gsm_lchan *lchan, unsigned int duration)
{
	struct msgb *msg;

	/* Let osmo_rtp_handle take care of updating state, and send nothing: */
	msg = osmo_rtp_build(lchan->abis_ip.osmux.rtpst, lchan->abis_ip.rtp_payload,
			     0, NULL, duration);
	if (!msg)
		return -1;
	msgb_free(msg);
	return 0;
}
