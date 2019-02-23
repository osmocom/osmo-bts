/* RTP based GSM BER testing for osmo-bts, implementing ideas described in
 * https://osmocom.org/projects/osmobts/wiki/BER_Testing
 *
 * In short: The command transmits a PRBS sequence encapsulated in RTP frames, which are sent
 * to the BTS, which transmits that data in the (unimpaired) downlink.  The mobile station
 * receives the data and is instructed to loop it back in the (possibly impaired) uplink.
 * The BTS receives that uplink, puts in in RTP frames which end up being received back by this
 * very tool.  By correlating the received RTP with the PRBS sequence, this tool can compute
 * the BER (Bit Error Rate) of the (possibly impaired) uplink.  Doing this with different
 * RF channel model simulators in the uplink allows to establish BER at different levels and
 * channel conditions. */

/* (C) 2019 sysmocom - s.f.m.c. GmbH; Author: Sylvain Munaut
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
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <osmocom/codec/codec.h>
#include <osmocom/core/logging.h>
#include <osmocom/core/prbs.h>
#include <osmocom/core/timer.h>
#include <osmocom/core/utils.h>
#include <osmocom/trau/osmo_ortp.h>

#include <codec_bit_class.h>


struct app_state {
	struct osmo_rtp_socket *rs;

	enum {
		WAIT_CONN = 0,	/* Wait for incoming connection */
		WAIT_LOOP,	/* Wait for a somewhat valid packet to start measuring */
		RUNNING,	/* Main state */
	} state;

	int pt;

	int     ref_len;
	uint8_t ref_bytes[GSM_FR_BYTES];	/* FR is the largest possible one */
	ubit_t  ref_bits[8*GSM_FR_BYTES];

	struct osmo_timer_list rtp_timer;

	uint16_t rx_last_seq;
	uint32_t rx_last_ts;
	uint32_t rx_idx;
	uint32_t tx_idx;

	const int *err_tbl;	/* Classification table */
	int err_frames;		/* Number of accumulated frames */
	int err_cnt[3];		/* Bit error counter */
	int err_tot[3];		/* Total # bits in that class */
};

#define FLOW_REG_TX_MAX_ADVANCE		200
#define FLOW_REG_TX_MIN_ADVANCE		 50


const struct log_info log_info;


static const uint8_t amr_size_by_ft[] = {
	[0]	= 12,	/* 4.75 */
	[1]	= 13,	/* 5.15 */
	[2]	= 15,	/* 5.9 */
	[3]	= 17,	/* 6.7 */
	[4]	= 19,	/* 7.4 */
	[5]	= 20,	/* 7.95 */
	[6]	= 26,	/* 10.2 */
	[7]	= 31,	/* 12.2 */
	[8]	= 5,	/* SID */
};

static const char * const amr_rate_by_ft[] = {
	[0]	= "4.75",
	[1]	= "5.15",
	[2]	= "5.9",
	[3]	= "6.7",
	[4]	= "7.4",
	[5]	= "7.95",
	[6]	= "10.2",
	[7]	= "12.2",
	[8]	= "SID",
};


static void
_gsm_fr_gen_ref(struct app_state *as)
{
	struct osmo_prbs_state pn9;

	/* Length */
	as->ref_len = GSM_FR_BYTES;

	/* Marker */
	as->ref_bits[0] = 1;
	as->ref_bits[1] = 1;
	as->ref_bits[2] = 0;
	as->ref_bits[3] = 1;

	/* PN */
	osmo_prbs_state_init(&pn9, &osmo_prbs9);
	pn9.state = 31;
	osmo_prbs_get_ubits(&as->ref_bits[4], 260, &pn9);

	/* Convert to bytes */
	osmo_ubit2pbit_ext(as->ref_bytes, 0, as->ref_bits, 0, 8*GSM_FR_BYTES, 0);

	/* Init error classes */
	as->err_tot[0] =  50;
	as->err_tot[1] = 132;
	as->err_tot[2] =  78;
	as->err_tbl = gsm_fr_bitclass;
}

static void
_gsm_efr_gen_ref(struct app_state *as)
{
	struct osmo_prbs_state pn9;

	/* Length */
	as->ref_len = GSM_EFR_BYTES;

	/* Marker */
	as->ref_bits[0] = 1;
	as->ref_bits[1] = 1;
	as->ref_bits[2] = 0;
	as->ref_bits[3] = 0;

	/* PN */
	osmo_prbs_state_init(&pn9, &osmo_prbs9);
	pn9.state = 31;
	osmo_prbs_get_ubits(&as->ref_bits[4], 244, &pn9);

	/* Convert to bytes */
	osmo_ubit2pbit_ext(as->ref_bytes, 0, as->ref_bits, 0, 8*GSM_EFR_BYTES, 0);

	/* Init error classes */
	as->err_tot[0] =  50;
	as->err_tot[1] = 125;
	as->err_tot[2] =  73;
	as->err_tbl = gsm_efr_bitclass;
}

static void
_gsm_amr_gen_ref(struct app_state *as)
{
	struct osmo_prbs_state pn9;
	uint8_t hdr[2];

	/* Length */
	as->ref_len = 33;

	/* Header */
	hdr[0] = 0x70;
	hdr[1] = 0x3c;
	osmo_pbit2ubit_ext(as->ref_bits, 0, hdr, 0, 16, 0);

	/* PN */
	osmo_prbs_state_init(&pn9, &osmo_prbs9);
	pn9.state = 31;
	osmo_prbs_get_ubits(&as->ref_bits[16], 244, &pn9);

	/* Unused bits */
	as->ref_bits[260] = 0;
	as->ref_bits[261] = 0;
	as->ref_bits[262] = 0;
	as->ref_bits[263] = 0;

	/* Convert to bytes */
	osmo_ubit2pbit_ext(as->ref_bytes, 0, as->ref_bits, 0, 264, 0);

	/* Init error classes */
	as->err_tot[0] =  81;
	as->err_tot[1] = 163;
	as->err_tot[2] =  -1;
	as->err_tbl = gsm_amr_12_2_bitclass;
}


static void
_gsm_gen_ref(struct app_state *as)
{
	switch (as->pt) {
	case RTP_PT_GSM_FULL:
		_gsm_fr_gen_ref(as);
		break;
	case RTP_PT_GSM_EFR:
		_gsm_efr_gen_ref(as);
		break;
	case RTP_PT_AMR:
		_gsm_amr_gen_ref(as);
		break;
	default:
		fprintf(stderr, "[!] Unsupported payload type for BER measurement\n");
	}
}

static int
_gsm_ber(struct app_state *as, const uint8_t *payload, unsigned int payload_len)
{
	ubit_t rx_bits[8*33];
	int err[3];	/* Class 1a, 1b, 2 */
	int ones;
	int i, j;

	if (payload) {
		/* Process real-payload */
		osmo_pbit2ubit_ext(rx_bits, 0, payload, 0, 8*payload_len, 0);

		err[0] = err[1] = err[2] = 0;
		ones = 0;

		for (i = 0; i < 8 * payload_len; i++) {
			j = as->err_tbl[i];
			if (j >= 0) {
				err[j] += rx_bits[i] ^ as->ref_bits[i];
				ones += rx_bits[i];
			}
		}

		if (ones < 32) { // This frames is probably us underrunning Tx, don't use it
			fprintf(stderr, "[w] Frame ignored as probably TX underrun %d %d\n", as->tx_idx, as->rx_idx);
			return 1;
		}
	} else {
		/* No payload -> Lost frame completely */
		err[0] = as->err_tot[0] / 2;
		err[1] = as->err_tot[1] / 2;
		err[2] = as->err_tot[2] / 2;
	}

	if (as->state == RUNNING) {
		/* Update records */
		if (err[0] != 0) {
			/* Class 1a bits bad -> Frame error */
			as->err_cnt[0]++;
		}

		as->err_cnt[1] += err[1];	/* Class 1b */
		as->err_cnt[2] += err[2];	/* class 2  */

		as->err_frames++;

		/* Enough for a read-out ? */
		if (as->err_frames == 200) {
			printf("FBER: %4.2f    C1b RBER: %5.3f    C2 RBER: %5.3f\n",
				100.0f * as->err_cnt[0] / as->err_frames,
				100.0f * as->err_cnt[1] / (as->err_tot[1] * as->err_frames),
				100.0f * as->err_cnt[2] / (as->err_tot[2] * as->err_frames)
			);
			memset(as->err_cnt, 0, sizeof(as->err_cnt));
			as->err_frames = 0;
		}
	}

	return err[0] != 0;
}

static int
_rtp_check_payload_type(const uint8_t *payload, unsigned int payload_len)
{
	uint8_t ft;
	int pt = -1;

	switch (payload_len) {
	case GSM_FR_BYTES: /* FR  or AMR 12.2k */
		/* Check for AMR */
		ft = (payload[1] >> 3) & 0xf;
		if (ft == 7)
			pt = RTP_PT_AMR;

		/* Check for FR */
		else if ((payload[0] & 0xF0) == 0xD0)
			pt = RTP_PT_GSM_FULL;

		/* None of the above */
		else
			fprintf(stderr, "[!] FR without 0xD0 signature or AMR with unknwon Frame Type ?!?\n");

		break;
	case GSM_EFR_BYTES: /* EFR */
		if ((payload[0] & 0xF0) != 0xC0)
			fprintf(stderr, "[!] EFR without 0xC0 signature ?!?\n");
		pt = RTP_PT_GSM_EFR;
		break;
	case GSM_HR_BYTES: /* HR */
		pt = RTP_PT_GSM_HALF;
		break;
	default: /* AMR */
		{
			uint8_t cmr, cmi, sti;
			cmr = payload[0] >> 4;
			ft = (payload[1] >> 3) & 0xf;

			if (payload_len != amr_size_by_ft[ft]+2)
				fprintf(stderr, "AMR FT %u(%s) but size %u\n",
					ft, amr_rate_by_ft[ft], payload_len);

			switch (ft) {
			case 0: case 1: case 2: case 3:
			case 4: case 5: case 6: case 7:
				cmi = ft;
				printf("AMR SPEECH with FT/CMI %u(%s), "
					"CMR %u\n",
					cmi, amr_rate_by_ft[cmi],
					cmr);
				break;
			case 8: /* SID */
				cmi = (payload[2+4] >> 1) & 0x7;
				sti = payload[2+4] & 0x10;
				printf("AMR SID %s with CMI %u(%s), CMR %u(%s)\n",
					sti ? "UPDATE" : "FIRST",
					cmi, amr_rate_by_ft[cmi],
					cmr, amr_rate_by_ft[cmr]);
				break;
			}
		}
		break;
	}

	return pt;
}

static void
rtp_timer_cb(void *priv)
{
	struct app_state *as = (struct app_state *)priv;

	/* Send at least one frame if we're not too far ahead */
	if (as->tx_idx < (as->rx_idx + FLOW_REG_TX_MAX_ADVANCE)) {
		osmo_rtp_send_frame(as->rs, as->ref_bytes, as->ref_len, GSM_RTP_DURATION);
		as->tx_idx++;
	} else {
		fprintf(stderr, "Skipped\n");
	}

	/* Then maybe a second one to try and catch up to RX */
	if (as->tx_idx < (as->rx_idx + FLOW_REG_TX_MIN_ADVANCE)) {
		osmo_rtp_send_frame(as->rs, as->ref_bytes, as->ref_len, GSM_RTP_DURATION);
		as->tx_idx++;
	}

	/* Re-schedule */
	osmo_timer_schedule(&as->rtp_timer, 0, 20000);
}

static int
rtp_seq_num_diff(uint16_t new, uint16_t old)
{
	int d = (int)new - (int)old;
	while (d > 49152)
		d -= 65536;
	while (d < -49152)
		d += 65536;
	return d;
}

static void
rtp_rx_cb(struct osmo_rtp_socket *rs,
	const uint8_t *payload, unsigned int payload_len,
	uint16_t seq_number, uint32_t timestamp, bool marker)
{
	struct app_state *as = (struct app_state *)rs->priv;
	int pt, rc, d;

//	printf("Rx(%u, %d, %d, %d): %s\n", payload_len, seq_number, timestamp, marker, osmo_hexdump(payload, payload_len));

	/* Identify payload */
	pt = _rtp_check_payload_type(payload, payload_len);

	/* First packet ? */
	if (as->state == WAIT_CONN) {
		/* Setup for this payload type */
		as->pt = pt;
		osmo_rtp_socket_set_pt(as->rs, pt);
		_gsm_gen_ref(as);

		/* Timer every 20 ms */
		osmo_timer_setup(&as->rtp_timer, rtp_timer_cb, as);
		osmo_timer_add(&as->rtp_timer);
		osmo_timer_schedule(&as->rtp_timer, 0, 20000);

		/* Init our time tracking */
		as->rx_last_seq = seq_number;
		as->rx_last_ts  = timestamp;

		/* Now we wait for a loop */
		as->state = WAIT_LOOP;
	}

	/* RX sequence & timstamp tracking */
	if (rtp_seq_num_diff(seq_number, as->rx_last_seq) > 1)
		fprintf(stderr, "[!] RTP sequence number discontinuity (%d -> %d)\n", as->rx_last_seq, seq_number);

	d = (timestamp - as->rx_last_ts) / GSM_RTP_DURATION;

	as->rx_idx += d;
	as->rx_last_seq = seq_number;
	as->rx_last_ts  = timestamp;

	/* Account for missing frames in BER tracking */
	if (d > 1) {
		fprintf(stderr, "[!] RTP %d missing frames assumed lost @%d\n", d-1, seq_number);
		while (--d)
			_gsm_ber(as, NULL, 0);
	}

	/* BER analysis */
	rc = _gsm_ber(as, payload, payload_len);

	if ((as->state == WAIT_LOOP) && (rc == 0))
		as->state = RUNNING;
}


int main(int argc, char **argv)
{
	struct app_state _as, *as = &_as;
	int rc, port;

	/* Args */
	if (argc < 2)
		return -1;

	port = atoi(argv[1]);

	/* App init */
	memset(as, 0x00, sizeof(struct app_state));

	log_init(&log_info, NULL);
	osmo_rtp_init(NULL);

	/* Start auto-connect RTP socket */
	as->rs = osmo_rtp_socket_create(NULL, 0);

	as->rs->priv = as;
	as->rs->rx_cb = rtp_rx_cb;

		/* Jitter buffer gets in the way, we want the raw traffic */
	osmo_rtp_socket_set_param(as->rs, OSMO_RTP_P_JIT_ADAP, 0);
	osmo_rtp_socket_set_param(as->rs, OSMO_RTP_P_JITBUF, 0);

		/* Bind to requested port */
	fprintf(stderr, "[+] Binding RTP socket on port %u...\n", port);
	rc = osmo_rtp_socket_bind(as->rs, "0.0.0.0", port);
	if (rc < 0) {
		fprintf(stderr, "[!] error binding RTP socket: %d\n", rc);
		return rc;
	}

		/* We 'connect' to the first source we hear from */
	osmo_rtp_socket_autoconnect(as->rs);

	/* Main loop */
	while (1)
		osmo_select_main(0);

	return 0;
}
