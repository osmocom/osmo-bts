#pragma once

#include <stdint.h>
#include <osmo-bts/gsm_data.h>

/* How many dB do we raise/lower power as maximum */
#define PWR_RAISE_MAX_DB 4
#define PWR_LOWER_MAX_DB 8

int lchan_ms_pwr_ctrl(struct gsm_lchan *lchan,
		      const uint8_t ms_power_lvl,
		      const int8_t ul_rssi_dbm);

int lchan_bs_pwr_ctrl(struct gsm_lchan *lchan,
		      const struct gsm48_hdr *gh);
