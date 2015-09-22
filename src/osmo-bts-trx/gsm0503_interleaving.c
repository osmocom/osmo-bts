
#include <stdint.h>

#include <osmocom/core/bits.h>

#include "gsm0503_tables.h"
#include "gsm0503_interleaving.h"

/*
 * GSM xCCH interleaving and burst mapping
 *
 * Interleaving:
 *
 * Given 456 coded input bits, form 4 blocks of 114 bits:
 *
 *      i(B, j) = c(n, k)       k = 0, ..., 455
 *                              n = 0, ..., N, N + 1, ...
 *                              B = B_0 + 4n + (k mod 4)
 *                              j = 2(49k mod 57) + ((k mod 8) div 4)
 *
 * Mapping on Burst:
 *
 *      e(B, j) = i(B, j)
 *      e(B, 59 + j) = i(B, 57 + j)     j = 0, ..., 56
 *      e(B, 57) = h_l(B)
 *      e(B, 58) = h_n(B)
 *
 * Where hl(B) and hn(B) are bits in burst B indicating flags.
 */

void gsm0503_xcch_deinterleave(sbit_t *cB, sbit_t *iB)
{
	int j, k, B;

	for (k=0; k<456; k++) {
		B = k & 3;
		j = 2 * ((49 * k) % 57) + ((k & 7) >> 2);
		cB[k] = iB[B * 114 + j];
	}
}

void gsm0503_xcch_interleave(ubit_t *cB, ubit_t *iB)
{
	int j, k, B;

	for (k=0; k<456; k++) {
		B = k & 3;
		j = 2 * ((49 * k) % 57) + ((k & 7) >> 2);
		iB[B * 114 + j] = cB[k];
	}
}

/*
 * GSM TCH FR/EFR/AFS interleaving and burst mapping
 *
 * Interleaving:
 *
 * Given 456 coded input bits, form 8 blocks of 114 bits,
 * where even bits of the first 4 blocks and odd bits of the last 4 blocks
 * are used:
 *
 *      i(B, j) = c(n, k)       k = 0, ..., 455
 *                              n = 0, ..., N, N + 1, ...
 *                              B = B_0 + 4n + (k mod 8)
 *                              j = 2(49k mod 57) + ((k mod 8) div 4)
 *
 * Mapping on Burst:
 *
 *      e(B, j) = i(B, j)
 *      e(B, 59 + j) = i(B, 57 + j)     j = 0, ..., 56
 *      e(B, 57) = h_l(B)
 *      e(B, 58) = h_n(B)
 *
 * Where hl(B) and hn(B) are bits in burst B indicating flags.
 */

void gsm0503_tch_fr_deinterleave(sbit_t *cB, sbit_t *iB)
{
	int j, k, B;

	for (k=0; k<456; k++) {
		B = k & 7;
		j = 2 * ((49 * k) % 57) + ((k & 7) >> 2);
		cB[k] = iB[B * 114 + j];
	}
}

void gsm0503_tch_fr_interleave(ubit_t *cB, ubit_t *iB)
{
	int j, k, B;

	for (k=0; k<456; k++) {
		B = k & 7;
		j = 2 * ((49 * k) % 57) + ((k & 7) >> 2);
		iB[B * 114 + j] = cB[k];
	}
}

/*
 * GSM TCH HR/AHS interleaving and burst mapping
 *
 * Interleaving:
 *
 * Given 288 coded input bits, form 4 blocks of 114 bits,
 * where even bits of the first 2 blocks and odd bits of the last 2 blocks
 * are used:
 *
 *      i(B, j) = c(n, k)       k = 0, ..., 227
 *                              n = 0, ..., N, N + 1, ...
 *                              B = B_0 + 2n + b
 *                              j, b = table[k];
 *
 * Mapping on Burst:
 *
 *      e(B, j) = i(B, j)
 *      e(B, 59 + j) = i(B, 57 + j)     j = 0, ..., 56
 *      e(B, 57) = h_l(B)
 *      e(B, 58) = h_n(B)
 *
 * Where hl(B) and hn(B) are bits in burst B indicating flags.
 */

void gsm0503_tch_hr_deinterleave(sbit_t *cB, sbit_t *iB)
{
	int j, k, B;

	for (k=0; k<228; k++) {
		B = gsm0503_tch_hr_interleaving[k][1];
		j = gsm0503_tch_hr_interleaving[k][0];
		cB[k] = iB[B * 114 + j];
	}
}

void gsm0503_tch_hr_interleave(ubit_t *cB, ubit_t *iB)
{
	int j, k, B;

	for (k=0; k<228; k++) {
		B = gsm0503_tch_hr_interleaving[k][1];
		j = gsm0503_tch_hr_interleaving[k][0];
		iB[B * 114 + j] = cB[k];
	}
}

