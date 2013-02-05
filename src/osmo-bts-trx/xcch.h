#ifndef _XCCH_H
#define _XCCH_H

int xcch_decode_cB(uint8_t *l2_data, sbit_t *cB);
int xcch_decode(uint8_t *l2_data, sbit_t *bursts);
int xcch_encode_cB(ubit_t *cB, uint8_t *l2_data);
int xcch_encode(ubit_t *bursts, uint8_t *l2_data);

#endif /* _XCCH_H */

