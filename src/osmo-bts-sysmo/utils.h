#ifndef SYSMOBTS_UTILS_H
#define SYSMOBTS_UTILS_H

#include <stdint.h>

struct gsm_bts;

int sysmobts_select_femto_band(struct gsm_bts *bts, uint16_t arfcn);

#endif
