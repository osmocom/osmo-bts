/*
 * Routines to check the structurally integrity of messages
 */

#pragma once

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

int msg_verify_ipa_structure(struct msgb *msg);
int msg_verify_oml_structure(struct msgb *msg);
