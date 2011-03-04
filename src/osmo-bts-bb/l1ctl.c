/* Layer1 control code, talking L1CTL protocol with L1 on the phone */

/* (C) 2010 by Holger Hans Peter Freyther <zecke@selfish.org>
 * (C) 2010 by Harald Welte <laforge@gnumonks.org>
 * (C) 2011 by Andreas Eversberg <jolly@eversberg.eu>
 *
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>

#include <arpa/inet.h>

#include <l1ctl_proto.h>

#include <osmocore/signal.h>
#include <osmocore/logging.h>
#include <osmocore/timer.h>
#include <osmocore/msgb.h>
#include <osmocore/tlv.h>
#include <osmocore/gsm_utils.h>
#include <osmocore/gsmtap_util.h>
#include <osmocore/protocol/gsm_04_08.h>
#include <osmocore/protocol/gsm_08_58.h>
#include <osmocore/rsl.h>

#include <osmocom/bb/common/l1ctl.h>
#include <osmocom/bb/common/osmocom_data.h>
#include <osmocom/bb/common/l1l2_interface.h>
#include <osmocom/bb/common/lapdm.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/abis.h>
#include <osmo-bts/rtp.h>
#include <osmo-bts/bts.h>

static struct msgb *osmo_l1_alloc(uint8_t msg_type)
{
	struct l1ctl_hdr *l1h;
	struct msgb *msg = msgb_alloc_headroom(256, 4, "osmo_l1");

	if (!msg) {
		LOGP(DL1C, LOGL_ERROR, "Failed to allocate memory.\n");
		return NULL;
	}

	msg->l1h = msgb_put(msg, sizeof(*l1h));
	l1h = (struct l1ctl_hdr *) msg->l1h;
	l1h->msg_type = msg_type;
	
	return msg;
}


static int osmo_make_band_arfcn(struct osmocom_ms *ms, uint16_t arfcn)
{
	/* TODO: Include the band */
	return arfcn;
}

/* Receive L1CTL_DATA_IND (Data Indication from L1) */
int rx_ph_data_ind(struct osmocom_ms *ms, struct msgb *msg)
{
	struct l1ctl_info_dl *dl, dl_cpy;
	struct l1ctl_data_ind *ccch;
	struct lapdm_entity *le;
	struct rx_meas_stat *meas = &ms->meas;
	uint8_t chan_type, chan_ts, chan_ss;
	uint8_t gsmtap_chan_type;
	struct gsm_time tm;
	struct osmobts_ms *bts_ms = container_of(ms, struct osmobts_ms, ms);
	struct osmobts_lchan *lchan;

	if (msgb_l3len(msg) < sizeof(*ccch)) {
		LOGP(DL1C, LOGL_ERROR, "MSG too short Data Ind: %u\n",
			msgb_l3len(msg));
		msgb_free(msg);
		return -1;
	}

	dl = (struct l1ctl_info_dl *) msg->l1h;
	msg->l2h = dl->payload;
	ccch = (struct l1ctl_data_ind *) msg->l2h;

	gsm_fn2gsmtime(&tm, ntohl(dl->frame_nr));
	rsl_dec_chan_nr(dl->chan_nr, &chan_type, &chan_ss, &chan_ts);
	lchan = bts_ms->trx->slot[chan_ts].lchan[chan_ss];
	if (!lchan) {
		LOGP(DL1C, LOGL_ERROR, "Data IND for non existing lchan\n");
		msgb_free(msg);
		return -1;
	}
	LOGP(DL1C, LOGL_NOTICE, "RX: %s | %s(%.4u/%.2u/%.2u) %d dBm\n",
		rsl_chan_nr_str(dl->chan_nr),
		hexdump(ccch->data, sizeof(ccch->data)),
		tm.t1, tm.t2, tm.t3, (int)dl->rx_level-110);

	meas->last_fn = ntohl(dl->frame_nr);
	meas->frames++;
	meas->snr += dl->snr;
	meas->berr += dl->num_biterr;
	meas->rxlev += dl->rx_level;

	if (dl->fire_crc >= 2) {
		LOGP(DL1C, LOGL_NOTICE, "Dropping frame with %u bit errors\n",
			dl->num_biterr);
		msgb_free(msg);
		return 0;
	}

	/* send CCCH data via GSMTAP */
	gsmtap_chan_type = chantype_rsl2gsmtap(chan_type, dl->link_id);
	gsmtap_sendmsg(ntohs(dl->band_arfcn), chan_ts, gsmtap_chan_type, chan_ss,
			tm.fn, dl->rx_level-110, dl->snr, ccch->data,
			sizeof(ccch->data));

	/* determine LAPDm entity based on SACCH or not */
	if (dl->link_id & 0x40)
		le = &lchan->l2_entity.lapdm_acch;
	else
		le = &lchan->l2_entity.lapdm_dcch;
	/* make local stack copy of l1ctl_info_dl, as LAPDm will
	 * overwrite skb hdr */
	memcpy(&dl_cpy, dl, sizeof(dl_cpy));

	/* pull the L1 header from the msgb */
	msgb_pull(msg, msg->l2h - (msg->l1h-sizeof(struct l1ctl_hdr)));
	msg->l1h = NULL;

	/* send it up into LAPDm */
	l2_ph_data_ind(msg, le, &dl_cpy);

	return 0;
}

/* Receive L1CTL_DATA_CONF (Data Confirm from L1) */
static int rx_ph_data_conf(struct osmocom_ms *ms, struct msgb *msg)
{
	struct l1ctl_info_dl *dl;
	struct lapdm_entity *le;
	uint8_t chan_type, chan_ts, chan_ss;
	struct osmobts_ms *bts_ms = container_of(ms, struct osmobts_ms, ms);
	struct osmobts_lchan *lchan;

	dl = (struct l1ctl_info_dl *) msg->l1h;

	rsl_dec_chan_nr(dl->chan_nr, &chan_type, &chan_ss, &chan_ts);
	lchan = bts_ms->trx->slot[chan_ts].lchan[chan_ss];
	if (!lchan) {
		LOGP(DL1C, LOGL_ERROR, "Data IND for non existing lchan\n");
		msgb_free(msg);
		return -1;
	}

	/* determine LAPDm entity based on SACCH or not */
	if (dl->link_id & 0x40)
		le = &lchan->l2_entity.lapdm_acch;
	else
		le = &lchan->l2_entity.lapdm_dcch;

	/* send it up into LAPDm */
	l2_ph_data_conf(msg, le);

	return 0;
}

/* Transmit L1CTL_DATA_REQ */
int l1ctl_tx_data_req(struct osmocom_ms *ms, struct msgb *msg,
                      uint8_t chan_nr, uint8_t link_id)
{
	struct l1ctl_hdr *l1h;
	struct l1ctl_info_ul *l1i_ul;
	uint8_t chan_type, chan_ts, chan_ss;
	uint8_t gsmtap_chan_type;

	if (msgb_l2len(msg) > 23) {
		LOGP(DL1C, LOGL_ERROR, "L1 cannot handle message length "
			"> 23 (%u)\n", msgb_l2len(msg));
		msgb_free(msg);
		return -EINVAL;
	} else if (msgb_l2len(msg) < 23)
		LOGP(DL1C, LOGL_ERROR, "L1 message length < 23 (%u) "
			"doesn't seem right!\n", msgb_l2len(msg));

	/* send copy via GSMTAP */
	rsl_dec_chan_nr(chan_nr, &chan_type, &chan_ss, &chan_ts);
	gsmtap_chan_type = chantype_rsl2gsmtap(chan_type, link_id);
	gsmtap_sendmsg(0|0x4000, chan_ts, gsmtap_chan_type, chan_ss,
			0, 127, 255, msg->l2h, msgb_l2len(msg));

	LOGP(DL1C, LOGL_NOTICE, "TX: %s | %s\n", rsl_chan_nr_str(chan_nr),
		hexdump(msg->l2h, msgb_l2len(msg)));

	/* prepend uplink info header */
	l1i_ul = (struct l1ctl_info_ul *) msgb_push(msg, sizeof(*l1i_ul));

	l1i_ul->chan_nr = chan_nr;
	l1i_ul->link_id = link_id;

	/* prepend l1 header */
	msg->l1h = msgb_push(msg, sizeof(*l1h));
	l1h = (struct l1ctl_hdr *) msg->l1h;
	l1h->msg_type = L1CTL_DATA_REQ;

	printf("todo: transcode message\n");

	return osmo_send_l1(ms, msg);
}

/* Transmit L1CTL_RESET_REQ */
int l1ctl_tx_reset_req(struct osmocom_ms *ms, uint8_t type)
{
	struct msgb *msg;
	struct l1ctl_reset *res;

	msg = osmo_l1_alloc(L1CTL_RESET_REQ);
	if (!msg)
		return -1;

	LOGP(DL1C, LOGL_INFO, "Tx Reset Req (%u)\n", type);
	res = (struct l1ctl_reset *) msgb_put(msg, sizeof(*res));
	res->type = type;

	return osmo_send_l1(ms, msg);
}

/* Receive L1CTL_RESET_IND */
static int rx_l1_reset(struct osmocom_ms *ms)
{
	LOGP(DL1C, LOGL_INFO, "Layer1 Reset indication\n");
	dispatch_signal(SS_L1CTL, S_L1CTL_RESET, ms);

	return 0;
}

/* Receive incoming data from L1 using L1CTL format */
int l1ctl_recv(struct osmocom_ms *ms, struct msgb *msg)
{
	int rc = 0;
	struct l1ctl_hdr *l1h;
	struct l1ctl_info_dl *dl;

	if (msgb_l2len(msg) < sizeof(*dl)) {
		LOGP(DL1C, LOGL_ERROR, "Short Layer2 message: %u\n",
			msgb_l2len(msg));
		msgb_free(msg);
		return -1;
	}

	l1h = (struct l1ctl_hdr *) msg->l1h;

	/* move the l1 header pointer to point _BEHIND_ l1ctl_hdr,
	   as the l1ctl header is of no interest to subsequent code */
	msg->l1h = l1h->data;

	switch (l1h->msg_type) {
	case L1CTL_DATA_IND:
		rc = rx_ph_data_ind(ms, msg);
		break;
	case L1CTL_DATA_CONF:
		rc = rx_ph_data_conf(ms, msg);
		break;
	case L1CTL_RESET_IND:
	case L1CTL_RESET_CONF:
		rc = rx_l1_reset(ms);
		msgb_free(msg);
		break;
	default:
		LOGP(DL1C, LOGL_ERROR, "Unknown MSG: %u\n", l1h->msg_type);
		msgb_free(msg);
		break;
	}

	return rc;
}

int l1ctl_tx_param_req(struct osmocom_ms *ms, uint8_t ta, uint8_t tx_power)
{
	return -ENOTSUP;
}

int l1ctl_tx_rach_req(struct osmocom_ms *ms, uint8_t ra, uint16_t offset,
	uint8_t combined)
{
	return -ENOTSUP;
}


