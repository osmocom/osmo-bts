#ifndef SYSMOBTS_UTILS_H
#define SYSMOBTS_UTILS_H

#include <stdint.h>

struct gsm_bts_trx;

int sysmobts_select_femto_band(struct gsm_bts_trx *trx, uint16_t arfcn);

#endif
