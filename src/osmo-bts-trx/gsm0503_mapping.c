
#include <stdint.h>
#include <string.h>

#include <osmocom/core/bits.h>

#include "gsm0503_mapping.h"

void gsm0503_xcch_burst_unmap(sbit_t *iB, sbit_t *eB, sbit_t *hl, sbit_t *hn)
{
	memcpy(iB,    eB,    57);
	memcpy(iB+57, eB+59, 57);

	if (hl)
		*hl = eB[57];

	if (hn)
		*hn = eB[58];
}

void gsm0503_xcch_burst_map(ubit_t *iB, ubit_t *eB, const ubit_t *hl,
	const ubit_t *hn)
{
	memcpy(eB,    iB,    57);
	memcpy(eB+59, iB+57, 57);

	if (hl)
		eB[57] = *hl;
	if (hn)
		eB[58] = *hn;
}

void gsm0503_tch_burst_unmap(sbit_t *iB, sbit_t *eB, sbit_t *h, int odd)
{
	int i;

	/* brainfuck: only copy even or odd bits */
	if (iB) {
		for (i=odd; i<57; i+=2)
			iB[i] = eB[i];
		for (i=58-odd; i<114; i+=2)
			iB[i] = eB[i+2];
	}

	if (h) {
		if (!odd)
			*h = eB[58];
		else
			*h = eB[57];
	}
}

void gsm0503_tch_burst_map(ubit_t *iB, ubit_t *eB, const ubit_t *h, int odd)
{
	int i;

	/* brainfuck: only copy even or odd bits */
	if (eB) {
		for (i=odd; i<57; i+=2)
			eB[i] = iB[i];
		for (i=58-odd; i<114; i+=2)
			eB[i+2] = iB[i];
	}

	if (h) {
		if (!odd)
			eB[58] = *h;
		else
			eB[57] = *h;
	}
}

