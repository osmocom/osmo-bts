/*
 * This module implements a helper function for the RTP input path:
 * validates incoming RTP payloads, makes the accept-or-drop decision,
 * and for some codecs signals additional required actions such as
 * dropping one header octet.
 *
 * Author: Mychaela N. Falconia <falcon@freecalypso.org>, 2023 - however,
 * Mother Mychaela's contributions are NOT subject to copyright.
 * No rights reserved, all rights relinquished.
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
 */

#include <stdbool.h>
#include <stdint.h>

#include <osmocom/core/logging.h>
#include <osmocom/core/utils.h>

#include <osmocom/codec/codec.h>

#include <osmo-bts/lchan.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/rtp_input_preen.h>

static bool amr_is_octet_aligned(const uint8_t *rtp_pl, unsigned payload_len)
{
	/*
	 * Logic: If 1st bit padding is not zero, packet is either:
	 * - bandwidth-efficient AMR payload.
	 * - malformed packet.
	 * However, Bandwidth-efficient AMR 4,75 frame last in payload(F=0, FT=0)
	 * with 4th,5ht,6th AMR payload to 0 matches padding==0.
	 * Furthermore, both AMR 4,75 bw-efficient and octet alignment are 14 bytes long (AMR 4,75 encodes 95b):
	 * bw-efficient: 95b, + 4b hdr + 6b ToC = 105b, + padding = 112b = 14B.
	 * octet-aligned: 1B hdr + 1B ToC + 95b = 111b, + padding = 112b = 14B.
	 * We cannot use other fields to match since they are inside the AMR
	 * payload bits which are unknown.
	 * As a result, this function may return false positive (true) for some AMR
	 * 4,75 AMR frames, but given the length, CMR and FT read is the same as a
	 * consequence, the damage in here is harmless other than being unable to
	 * decode the audio at the other side.
	 */
	#define AMR_PADDING1(rtp_pl) (rtp_pl[0] & 0x0f)
	#define AMR_PADDING2(rtp_pl) (rtp_pl[1] & 0x03)

	if (payload_len < 2 || AMR_PADDING1(rtp_pl) || AMR_PADDING2(rtp_pl))
		return false;

	return true;
}

static enum pl_input_decision
input_preen_fr(const uint8_t *rtp_pl, unsigned rtp_pl_len)
{
	if (rtp_pl_len != GSM_FR_BYTES)
		return PL_DECISION_DROP;
	if ((rtp_pl[0] & 0xF0) != 0xD0)
		return PL_DECISION_DROP;
	return PL_DECISION_ACCEPT;
}

static enum pl_input_decision
input_preen_efr(const uint8_t *rtp_pl, unsigned rtp_pl_len)
{
	if (rtp_pl_len != GSM_EFR_BYTES)
		return PL_DECISION_DROP;
	if ((rtp_pl[0] & 0xF0) != 0xC0)
		return PL_DECISION_DROP;
	return PL_DECISION_ACCEPT;
}

static enum pl_input_decision
input_preen_hr(const uint8_t *rtp_pl, unsigned rtp_pl_len)
{
	switch (rtp_pl_len) {
	case GSM_HR_BYTES:
		/* RTP input matches our internal format - we are good */
		return PL_DECISION_ACCEPT;
	case GSM_HR_BYTES_RTP_RFC5993:
		/* Strip ToC octet, leaving only "pure" TS 101 318 payload. */
		return PL_DECISION_STRIP_HDR_OCTET;
	default:
		/* invalid payload */
		return PL_DECISION_DROP;
	}
}

enum pl_input_decision
rtp_payload_input_preen(struct gsm_lchan *lchan, const uint8_t *rtp_pl,
			unsigned rtp_pl_len)
{
	/* If rtp continuous-streaming is enabled, we shall emit RTP packets
	 * with zero-length payloads as BFI markers. In a TrFO scenario such
	 * RTP packets sent by call leg A will be received by call leg B,
	 * hence we need to handle them gracefully. For the purposes of a BTS
	 * that runs on its own TDMA timing and does not need timing ticks from
	 * an incoming RTP stream, the correct action upon receiving such
	 * timing-tick-only RTP packets should be the same as when receiving
	 * no RTP packet at all. The simplest way to produce that behavior
	 * is to treat zero-length RTP payloads as invalid. */
	if (rtp_pl_len == 0)
		return PL_DECISION_DROP;

	switch (lchan->tch_mode) {
	case GSM48_CMODE_SPEECH_V1:
		if (lchan->type == GSM_LCHAN_TCH_F)
			return input_preen_fr(rtp_pl, rtp_pl_len);
		else
			return input_preen_hr(rtp_pl, rtp_pl_len);
	case GSM48_CMODE_SPEECH_EFR:
		return input_preen_efr(rtp_pl, rtp_pl_len);
	case GSM48_CMODE_SPEECH_AMR:
		/* Avoid forwarding bw-efficient AMR to lower layers,
		 * most bts models don't support it. */
		if (!amr_is_octet_aligned(rtp_pl, rtp_pl_len)) {
			LOGPLCHAN(lchan, DL1P, LOGL_NOTICE,
				  "RTP->L1: Dropping unexpected AMR encoding (bw-efficient?) %s\n",
				  osmo_hexdump(rtp_pl, rtp_pl_len));
			return PL_DECISION_DROP;
		}
		return PL_DECISION_ACCEPT;
	default:
		return PL_DECISION_ACCEPT;
	}
}
