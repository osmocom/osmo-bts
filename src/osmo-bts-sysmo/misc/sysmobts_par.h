#ifndef _SYSMOBTS_PAR_H
#define _SYSMOBTS_PAR_H

#include <stdint.h>

enum sysmobts_par {
	SYSMOBTS_PAR_MAC,
	SYSMOBTS_PAR_CLK_FACTORY,
	SYSMOBTS_PAR_TEMP_DIG_MAX,
	SYSMOBTS_PAR_TEMP_RF_MAX,
	SYSMOBTS_PAR_SERNR,
	SYSMOBTS_PAR_HOURS,
	SYSMOBTS_PAR_BOOTS,
	SYSMOBTS_PAR_KEY,
	SYSMOBTS_PAR_MODEL_NR,
	SYSMOBTS_PAR_MODEL_FLAGS,
	SYSMOBTS_PAR_TRX_NR,
	_NUM_SYSMOBTS_PAR
};


int sysmobts_par_get_int(enum sysmobts_par par, int *ret);
int sysmobts_par_set_int(enum sysmobts_par par, int val);
int sysmobts_par_get_buf(enum sysmobts_par par, uint8_t *buf,
			 unsigned int size);
int sysmobts_par_set_buf(enum sysmobts_par par, const uint8_t *buf,
			 unsigned int size);

#endif
