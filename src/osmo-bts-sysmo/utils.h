#ifndef SYSMOBTS_UTILS_H
#define SYSMOBTS_UTILS_H

#include <stdint.h>
#include "femtobts.h"

struct gsm_bts_trx;

int band_femto2osmo(GsmL1_FreqBand_t band);

int sysmobts_select_femto_band(struct gsm_bts_trx *trx, uint16_t arfcn);

int sysmobts_get_nominal_power(struct gsm_bts_trx *trx);

int sysmobts_get_power_trx(struct gsm_bts_trx *trx);

struct msgb;

enum manuf_type_id {
	MANUF_ID_IPA,
	MANUF_ID_OSMO,
};

enum oml_message_type {
	OML_MSG_TYPE_ETSI,
	OML_MSG_TYPE_IPA,
	OML_MSG_TYPE_OSMO,
};

static const char osmocom_magic[] = "org.osmocom";
static const char ipaccess_magic[] = "com.ipaccess";

int add_manufacturer_id_label(struct msgb *msg, enum manuf_type_id type_id);

void prepend_oml_ipa_header(struct msgb *msg);

int check_oml_msg(struct msgb *msg);
int check_ipa_header(struct msgb *msg);
#endif
