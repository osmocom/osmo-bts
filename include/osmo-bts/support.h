#ifndef _BTS_SUPPORT_H
#define _BTS_SUPPORT_H

struct bts_support {
	/* crypto supprot */
	uint8_t a5_1;
	uint8_t a5_2;
	uint8_t a5_3;
	uint8_t a5_4;
	uint8_t a5_5;
	uint8_t a5_6;
	uint8_t a5_7;
	/* radio support */
	uint8_t freq_map[128];
	/* codecs */
	uint8_t chan_comb[256];
	uint8_t full_v1;
	uint8_t full_v2;
	uint8_t full_v3;
	uint8_t half_v1;
	uint8_t half_v3;
};

extern struct bts_support bts_support;
void bts_support_init(void);
char *bts_support_comb_name(uint8_t chan_comb);

#endif /* _SUPPORT_H */


