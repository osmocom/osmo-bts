#pragma once

/* RFC4040 "clearmode" RTP payload length */
#define RFC4040_RTP_PLEN 160

struct gsm_lchan;

struct csd_v110_lchan_desc {
	uint16_t num_frames;		/* number of V.110 frames in a radio block */
	uint16_t num_frame_bits;	/* number of bits in each V.110 frame */
	uint8_t ra2_ir;			/* intermediate rate (8 or 16 kbit/s) for RA2 step */
};

extern const struct csd_v110_lchan_desc csd_v110_lchan_desc[256];

#define CSD_V110_NUM_BITS(desc) \
	((desc)->num_frames * (desc)->num_frame_bits)

int csd_v110_rtp_encode(const struct gsm_lchan *lchan, uint8_t *rtp,
			const uint8_t *data, size_t data_len,
			uint8_t nt48_half_num);
int csd_v110_rtp_decode(const struct gsm_lchan *lchan, uint8_t *data,
			uint8_t *align_bits, const uint8_t *rtp, size_t rtp_len);
