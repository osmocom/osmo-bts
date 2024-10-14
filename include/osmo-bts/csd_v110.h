#pragma once

/* RFC4040 "clearmode" RTP payload length */
#define RFC4040_RTP_PLEN 160

struct gsm_lchan;

struct csd_v110_lchan_desc {
	uint16_t num_blocks;
	uint16_t num_bits;
	uint8_t ra2_ir;
};

extern const struct csd_v110_lchan_desc csd_v110_lchan_desc[256];

int csd_v110_rtp_encode(const struct gsm_lchan *lchan, uint8_t *rtp,
			const uint8_t *data, size_t data_len);
int csd_v110_rtp_decode(const struct gsm_lchan *lchan, uint8_t *data,
			const uint8_t *rtp, size_t rtp_len);
