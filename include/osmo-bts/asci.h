#pragma once
#include <stdint.h>

#include <osmo-bts/lchan.h>

enum {
	VGCS_TALKER_NONE = 0,
	VGCS_TALKER_WAIT_FRAME,
	VGCS_TALKER_ACTIVE,
};

void vgcs_rach(struct gsm_lchan *lchan, uint8_t ra, uint8_t acc_delay, uint32_t fn);

void vgcs_talker_frame(struct gsm_lchan *lchan);

void vgcs_talker_reset(struct gsm_lchan *lchan);
