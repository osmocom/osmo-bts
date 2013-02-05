#ifndef _RACH_H
#define _RACH_H

int rach_decode(uint8_t *ra, sbit_t *burst, uint8_t bsic);
int rach_encode(ubit_t *burst, uint8_t *ra, uint8_t bsic);

#endif /* _RACH_H */
