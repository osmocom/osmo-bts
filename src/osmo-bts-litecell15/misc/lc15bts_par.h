#ifndef _LC15BTS_PAR_H
#define _LC15BTS_PAR_H

#include <osmocom/core/utils.h>

#define FACTORY_ROM_PATH	"/mnt/rom/factory"
#define USER_ROM_PATH		"/mnt/storage/var/run/lc15bts-mgr"

enum lc15bts_par {
	LC15BTS_PAR_TEMP_SUPPLY_MAX,
	LC15BTS_PAR_TEMP_SOC_MAX,
	LC15BTS_PAR_TEMP_FPGA_MAX,
	LC15BTS_PAR_TEMP_LOGRF_MAX,
	LC15BTS_PAR_TEMP_OCXO_MAX,
	LC15BTS_PAR_TEMP_TX0_MAX,
	LC15BTS_PAR_TEMP_TX1_MAX,
	LC15BTS_PAR_TEMP_PA0_MAX,
	LC15BTS_PAR_TEMP_PA1_MAX,
	LC15BTS_PAR_SERNR,
	LC15BTS_PAR_HOURS,
	LC15BTS_PAR_BOOTS,
	LC15BTS_PAR_KEY,
	_NUM_LC15BTS_PAR
};

extern const struct value_string lc15bts_par_names[_NUM_LC15BTS_PAR+1];

int lc15bts_par_get_int(void *ctx, enum lc15bts_par par, int *ret);
int lc15bts_par_set_int(void *ctx, enum lc15bts_par par, int val);
int lc15bts_par_get_buf(void *ctx, enum lc15bts_par par, uint8_t *buf, unsigned int size);
int lc15bts_par_set_buf(void *ctx, enum lc15bts_par par, const uint8_t *buf, unsigned int size);

int lc15bts_par_is_int(enum lc15bts_par par);

#endif
