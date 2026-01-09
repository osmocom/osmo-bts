#pragma once

#include <osmo-bts/gsm_data.h>

void lchan_ms_ta_ctrl_reset(struct gsm_lchan *lchan);

void lchan_ms_ta_ctrl(struct gsm_lchan *lchan, uint8_t ms_tx_ta, int16_t toa256);
