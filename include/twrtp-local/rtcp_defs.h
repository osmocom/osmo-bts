/*
 * Some definitions for RTCP, just enough to implement SR and RR
 * generation and parsing in twrtp.
 */

#pragma once

#include <stdint.h>

struct rtcp_sr_rr_hdr {
	uint8_t		v_p_rc;
	uint8_t		pt;
	uint16_t	len;
	uint32_t	ssrc;
} __attribute__((packed));

struct rtcp_sr_block {
	uint32_t	ntp_sec;
	uint32_t	ntp_fract;
	uint32_t	rtp_ts;
	uint32_t	pkt_count;
	uint32_t	octet_count;
} __attribute__((packed));

struct rtcp_rr_block {
	uint32_t	ssrc;
	uint32_t	lost_word;
	uint32_t	max_seq_ext;
	uint32_t	jitter;
	uint16_t	lsr_sec;
	uint16_t	lsr_fract;
	uint16_t	dlsr_sec;
	uint16_t	dlsr_fract;
} __attribute__((packed));

#define	RTCP_PT_SR	200
#define	RTCP_PT_RR	201
#define	RTCP_PT_SDES	202
#define	RTCP_PT_BYE	203
#define	RTCP_PT_APP	204

#define	SDES_ITEM_CNAME	1
#define	SDES_ITEM_NAME	2
#define	SDES_ITEM_EMAIL	3
#define	SDES_ITEM_PHONE	4
#define	SDES_ITEM_LOC	5
#define	SDES_ITEM_TOOL	6
#define	SDES_ITEM_NOTE	7
#define	SDES_ITEM_PRIV	8
