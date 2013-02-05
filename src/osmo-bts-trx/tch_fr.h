#ifndef _TCH_FR_H
#define _TCH_FR_H

int tch_fr_decode(uint8_t *tch_data, sbit_t *bursts);
int tch_fr_encode(ubit_t *bursts, uint8_t *tch_data, int len);

#endif /* _TCH_FR_H */
