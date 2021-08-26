#pragma once

#include <stdint.h>
#include <osmo-bts/gsm_data.h>

int lchan_ms_pwr_ctrl(struct gsm_lchan *lchan,
		      const uint8_t ms_power_lvl,
		      const int8_t ul_rssi_dbm,
		      const int16_t ul_lqual_cb);

int lchan_bs_pwr_ctrl(struct gsm_lchan *lchan,
		      const struct gsm48_hdr *gh);
