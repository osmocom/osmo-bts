/*
 * Routines to check the structurally integrity of messages
 */

#pragma once

#include <osmo-bts/gsm_data.h>

#include <stdbool.h>

struct msgb;

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
void save_last_sid(struct gsm_lchan *lchan, uint8_t *l1_payload, size_t length,
		   uint32_t fn, bool update);
bool dtx_sched_optional(struct gsm_lchan *lchan, uint32_t fn);
int msg_verify_ipa_structure(struct msgb *msg);
int msg_verify_oml_structure(struct msgb *msg);
