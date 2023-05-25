/*
 * RTP input validation function: makes the accept-or-drop decision,
 * and for some codecs signals additional required actions such as
 * dropping one header octet.
 */

#pragma once

#include <stdint.h>
#include <osmo-bts/lchan.h>

enum pl_input_decision {
	PL_DECISION_DROP,
	PL_DECISION_ACCEPT,
	PL_DECISION_STRIP_HDR_OCTET,
};

enum pl_input_decision
rtp_payload_input_preen(struct gsm_lchan *lchan, const uint8_t *rtp_pl,
			unsigned rtp_pl_len);
