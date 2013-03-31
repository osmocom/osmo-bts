#ifndef _0503_CODING_H
#define _0503_CODING_H

int xcch_decode(uint8_t *l2_data, sbit_t *bursts);
int xcch_encode(ubit_t *bursts, uint8_t *l2_data);
int pdtch_decode(uint8_t *l2_data, sbit_t *bursts, uint8_t *usf_p);
int pdtch_encode(ubit_t *bursts, uint8_t *l2_data, uint8_t l2_len);
int tch_fr_decode(uint8_t *tch_data, sbit_t *bursts, int net_order, int efr);
int tch_fr_encode(ubit_t *bursts, uint8_t *tch_data, int len, int net_order);
int tch_hr_decode(uint8_t *tch_data, sbit_t *bursts, int odd);
int tch_hr_encode(ubit_t *bursts, uint8_t *tch_data, int len);
int tch_afs_decode(uint8_t *tch_data, sbit_t *bursts, int codec_mode_req,
	uint8_t *codec, int codecs, uint8_t *ft, uint8_t *cmr, float *ber);
int tch_afs_encode(ubit_t *bursts, uint8_t *tch_data, int len,
	int codec_mode_req, uint8_t *codec, int codecs, uint8_t ft,
	uint8_t cmr);
int rach_decode(uint8_t *ra, sbit_t *burst, uint8_t bsic);
int rach_encode(ubit_t *burst, uint8_t *ra, uint8_t bsic);
int sch_decode(uint8_t *sb_info, sbit_t *burst);
int sch_encode(ubit_t *burst, uint8_t *sb_info);

#endif /* _0503_CODING_H */
