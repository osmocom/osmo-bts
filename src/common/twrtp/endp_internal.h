/*
 * Internal declarations for twrtp_endp library component.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include <osmocom/core/osmo_io.h>
#include <osmocom/core/timer.h>

#include <themwi/rtp/endp.h>

extern const struct osmo_io_ops _twrtp_endp_iops_rtp;
extern const struct osmo_io_ops _twrtp_endp_iops_rtcp;

int _twrtp_endp_send_rtcp(struct twrtp_endp *endp, bool send_sr,
			  const struct timespec *utc, uint32_t rtp_ts);
