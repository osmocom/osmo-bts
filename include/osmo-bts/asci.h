#pragma once
#include <stdint.h>

#include <osmo-bts/lchan.h>

enum {
	VGCS_TALKER_NONE = 0,
	VGCS_TALKER_WAIT_FRAME,
	VGCS_TALKER_ACTIVE,
};

void vgcs_rach(struct gsm_lchan *lchan, uint8_t ra, uint8_t acc_delay, uint32_t fn);

void vgcs_lchan_react(struct gsm_lchan *lchan);

void vgcs_talker_frame(struct gsm_lchan *lchan);

void vgcs_talker_reset(struct gsm_lchan *lchan);

void vgcs_listener_reset(struct gsm_lchan *lchan);

static inline bool vgcs_is_uplink_free(struct gsm_lchan *lchan)
{
	return lchan->asci.uplink_free;
}

static inline void vgcs_uplink_free_get(struct gsm_lchan *lchan, uint8_t *msg)
{
	memcpy(msg, lchan->asci.uplink_free_msg, GSM_MACBLOCK_LEN);
}

static inline void vgcs_uplink_free_set(struct gsm_lchan *lchan, uint8_t *msg)
{
	memcpy(lchan->asci.uplink_free_msg, msg, GSM_MACBLOCK_LEN);
	lchan->asci.uplink_free = true;
}

static inline void vgcs_uplink_free_reset(struct gsm_lchan *lchan)
{
	lchan->asci.uplink_free = false;
}
