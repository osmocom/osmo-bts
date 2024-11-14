/*
 * This header file provides a minimal definition for the basic RTP
 * header of RFC 3550.  This basic definition does not use any bit
 * fields; only byte-or-larger fields are broken out.
 */

#pragma once

#include <stdint.h>

struct rtp_basic_hdr {
	uint8_t		v_p_x_cc;
	uint8_t		m_pt;
	uint16_t	seq;
	uint32_t	tstamp;
	uint32_t	ssrc;
};
