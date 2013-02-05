#ifndef _PXXCH_H
#define _PXXCH_H

int pdch_decode(uint8_t *l2_data, sbit_t *bursts, uint8_t *usf_p);
int pdtch_encode(ubit_t *bursts, uint8_t *l2_data, uint8_t l2_len);

#endif /* _PXXCH_H */
