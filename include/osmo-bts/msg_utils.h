/*
 * Routines to check the structurally integrity of messages
 */

#pragma once

#include <osmo-bts/gsm_data.h>
#include <osmo-bts/dtx_dl_amr_fsm.h>

#include <osmocom/codec/codec.h>

#include <stdbool.h>

struct msgb;

/****************************************************************
* Accessor macros for control buffer words in RTP input path (DL)
*****************************************************************/

/* Storing RTP header fields in the path from RTP and Osmux
 * Rx callback functions to TCH-RTS.ind handling.
 * FIXME: do we really need this RTP header info downstream
 * of the jitter buffer mechanism in the RTP endpoint library?
 */
#define rtpmsg_marker_bit(x) ((x)->cb[0])
#define rtpmsg_seq(x)        ((x)->cb[1])
#define rtpmsg_ts(x)         ((x)->cb[2])

/* l1sap_rtp_rx_cb() does some preening or preparsing on some
 * RTP payloads, and in two cases (HR with RFC 5993 input and
 * CSD NT modes) this preparsing step produces some metadata
 * that need to be passed to TCH-RTS.ind handling.
 */
#define rtpmsg_is_rfc5993_sid(x) ((x)->cb[3])
#define rtpmsg_csd_align_bits(x) ((x)->cb[4])

/********************************************************
* Accessor macros for control buffer words in TCH UL path
*********************************************************/

/* We provide an ability for BTS models to indicate BFI along with payload
 * bits just like in GSM 08.60 TRAU-UL frames, and the same BFI flag can
 * then be set by model-independent functions for higher-level BFI
 * conditions.  This cb word shall act as a Boolean flag.
 */
#define tch_ul_msg_bfi(x) ((x)->cb[0])

/* For HRv1 codec, we have to pass SID classification from the function
 * that makes the initial determination to TS 101 318, RFC 5993 and
 * TW-TS-002 output functions.  Per classic GSM specs, common across
 * FR/HR/EFR, SID classification code is an integer equal to 0, 1 or 2;
 * in Osmocom it is enum osmo_gsm631_sid_class.
 *
 * NOTE: while the actual SID ternary classification exists in exactly
 * the same form across all 3 of FR/HR/EFR, we store it in a cb word
 * only for HR codec where we need it for RTP output functions.
 */
#define tch_ul_msg_hr_sid(x) ((x)->cb[1])

/**
 * Classification of OML message. ETSI for plain GSM 12.21
 * messages and IPA/Osmo for manufacturer messages.
 */
enum {
	OML_MSG_TYPE_ETSI,
	OML_MSG_TYPE_IPA,
	OML_MSG_TYPE_OSMO,
};

void lchan_set_marker(bool t, struct gsm_lchan *lchan);
bool dtx_dl_amr_enabled(const struct gsm_lchan *lchan);
void dtx_dispatch(struct gsm_lchan *lchan, enum dtx_dl_amr_fsm_events e);
bool dtx_recursion(const struct gsm_lchan *lchan);
void dtx_int_signal(struct gsm_lchan *lchan);
bool dtx_is_first_p1(const struct gsm_lchan *lchan);
void dtx_cache_payload(struct gsm_lchan *lchan, const uint8_t *l1_payload,
		       size_t length, uint32_t fn, int update);
int dtx_dl_amr_fsm_step(struct gsm_lchan *lchan, const uint8_t *rtp_pl,
			size_t rtp_pl_len, uint32_t fn, uint8_t *l1_payload,
			bool marker, uint8_t *len, uint8_t *ft_out);
uint8_t repeat_last_sid(struct gsm_lchan *lchan, uint8_t *dst, uint32_t fn);
int msg_verify_ipa_structure(struct msgb *msg);
int msg_verify_oml_structure(struct msgb *msg);
