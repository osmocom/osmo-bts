/*
 * Routines to check the structurally integrity of messages
 */

#pragma once

#include <osmo-bts/gsm_data.h>

#include <osmocom/codec/codec.h>

#include <stdbool.h>

struct msgb;

/* Access 1st byte of msgb control buffer */
#define rtpmsg_marker_bit(x) ((x)->cb[0])

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
void dtx_cache_payload(struct gsm_lchan *lchan, const uint8_t *l1_payload,
		       size_t length, uint32_t fn, int update);
int dtx_dl_amr_fsm_step(struct gsm_lchan *lchan, const uint8_t *rtp_pl,
			size_t rtp_pl_len, uint32_t fn, uint8_t *l1_payload,
			bool marker, uint8_t *len, uint8_t *ft_out);
uint8_t repeat_last_sid(struct gsm_lchan *lchan, uint8_t *dst, uint32_t fn);
int msg_verify_ipa_structure(struct msgb *msg);
int msg_verify_oml_structure(struct msgb *msg);
