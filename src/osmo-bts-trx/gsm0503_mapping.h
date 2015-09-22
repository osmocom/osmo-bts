#ifndef _0503_MAPPING_H
#define _0503_MAPPING_H

void gsm0503_xcch_burst_unmap(sbit_t *iB, sbit_t *eB, sbit_t *hl, sbit_t *hn);
void gsm0503_xcch_burst_map(ubit_t *iB, ubit_t *eB, const ubit_t *hl,
	const ubit_t *hn);
void gsm0503_tch_burst_unmap(sbit_t *iB, sbit_t *eB, sbit_t *h, int odd);
void gsm0503_tch_burst_map(ubit_t *iB, ubit_t *eB, const ubit_t *h, int odd);

#endif /* _0503_INTERLEAVING_H */
