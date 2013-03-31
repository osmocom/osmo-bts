#ifndef _0503_INTERLEAVING_H
#define _0503_INTERLEAVING_H

void gsm0503_xcch_deinterleave(sbit_t *cB, sbit_t *iB);
void gsm0503_xcch_interleave(ubit_t *cB, ubit_t *iB);
void gsm0503_tch_fr_deinterleave(sbit_t *cB, sbit_t *iB);
void gsm0503_tch_fr_interleave(ubit_t *cB, ubit_t *iB);
void gsm0503_tch_hr_deinterleave(sbit_t *cB, sbit_t *iB);
void gsm0503_tch_hr_interleave(ubit_t *cB, ubit_t *iB);

#endif /* _0503_INTERLEAVING_H */
