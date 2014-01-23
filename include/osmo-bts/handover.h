#pragma once

enum {
	HANDOVER_NONE = 0,
	HANDOVER_ENABLED,
	HANDOVER_WAIT_FRAME,
};

void handover_reset(struct gsm_lchan *lchan);

