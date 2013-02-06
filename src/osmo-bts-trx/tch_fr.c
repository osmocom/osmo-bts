/*
 * tch_fr.c
 *
 * Copyright (c) 2013  Andreas Eversberg <jolly@eversberg.eu>
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <osmocom/core/bits.h>
#include <osmocom/core/conv.h>
#include <osmocom/core/crcgen.h>

#include "xcch.h"
#include "tch_fr.h"


/*
 * GSM TCH FR/EFR parity
 *
 * g(x) = x^3 + x + 1
 */

const struct osmo_crc8gen_code tch_fr_crc3 = {
	.bits = 3,
	.poly = 0x2,
	.init = 0x0,
	.remainder = 0x7,
};


/*
 * GSM TCH FR/EFR convolutional coding
 *
 * G_0 = 1 + x^3 + x^4
 * G_1 = 1 + x + x^3 + x^4
 */

static const uint8_t conv_tch_fr_next_output[][2] = {
	{ 0, 3 }, { 1, 2 }, { 0, 3 }, { 1, 2 },
	{ 3, 0 }, { 2, 1 }, { 3, 0 }, { 2, 1 },
	{ 3, 0 }, { 2, 1 }, { 3, 0 }, { 2, 1 },
	{ 0, 3 }, { 1, 2 }, { 0, 3 }, { 1, 2 },
};

static const uint8_t conv_tch_fr_next_state[][2] = {
	{  0,  1 }, {  2,  3 }, {  4,  5 }, {  6,  7 },
	{  8,  9 }, { 10, 11 }, { 12, 13 }, { 14, 15 },
	{  0,  1 }, {  2,  3 }, {  4,  5 }, {  6,  7 },
	{  8,  9 }, { 10, 11 }, { 12, 13 }, { 14, 15 },
};

static const struct osmo_conv_code conv_tch_fr = {
	.N = 2,
	.K = 5,
	.len = 185,
	.next_output = conv_tch_fr_next_output,
	.next_state  = conv_tch_fr_next_state,
};


/*
 * GSM TCH FR/EFR interleaving and burst mapping
 *
 * Interleaving:
 *
 * Given 456 coded input bits, form 8 blocks of 114 bits,
 * where event bits of the first 4 block and off bits of the last 4 block
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

static void
tch_fr_deinterleave(sbit_t *cB, sbit_t *iB)
{
	int j, k, B;

	for (k=0; k<456; k++) {
		B = k & 7;
		j = 2 * ((49 * k) % 57) + ((k & 7) >> 2);
		cB[k] = iB[B * 114 + j];
	}
}

static void
tch_fr_interleave(ubit_t *cB, ubit_t *iB)
{
	int j, k, B;

	for (k=0; k<456; k++) {
		B = k & 7;
		j = 2 * ((49 * k) % 57) + ((k & 7) >> 2);
		iB[B * 114 + j] = cB[k];
	}
}

static void
tch_fr_burst_unmap(sbit_t *iB, sbit_t *eB, sbit_t *h, int odd)
{
	int i;

	/* brainfuck: only copy even or odd bits */
	for (i=odd; i<57; i+=2)
		iB[i] = eB[i];
	for (i=58-odd; i<114; i+=2)
		iB[i] = eB[i+2];

	if (h && !odd)
		*h = eB[57];

	if (h && odd)
		*h = eB[58];
}

static void
tch_fr_burst_map(ubit_t *iB, ubit_t *eB, ubit_t *h, int odd)
{
	int i;

	/* brainfuck: only copy even or odd bits */
	for (i=odd; i<57; i+=2)
		eB[i] = iB[i];
	for (i=58-odd; i<114; i+=2)
		eB[i+2] = iB[i];

	if (h && !odd)
		eB[57] = *h;
	if (h && odd)
		eB[58] = *h;
}

/* this corresponds to the bit-lengths of the individual codec
 * parameters as indicated in Table 1.1 of TS 06.10 */
static const uint8_t gsm_fr_map[] = {
	6, 6, 5, 5, 4, 4, 3, 3,
	7, 2, 2, 6, 3, 3, 3, 3,
	3, 3, 3, 3, 3, 3, 3, 3,
	3, 7, 2, 2, 6, 3, 3, 3,
	3, 3, 3, 3, 3, 3, 3, 3,
	3, 3, 7, 2, 2, 6, 3, 3,
	3, 3, 3, 3, 3, 3, 3, 3,
	3, 3, 3, 7, 2, 2, 6, 3,
	3, 3, 3, 3, 3, 3, 3, 3,
	3, 3, 3, 3
};

static void
tch_fr_reassemble(uint8_t *tch_data, ubit_t *b_bits)
{
	int i, j, k, l, o;

	tch_data[0] = 0xd << 4;
	memset(tch_data + 1, 0, 32);
	/* reassemble d-bits */
	i = 0; /* counts bits */
	j = 4; /* counts output bits */
	k = gsm_fr_map[0]-1; /* current number bit in element */
	l = 0; /* counts element bits */
	o = 0; /* offset input bits */
	while (i < 260) {
		tch_data[j>>3] |= (b_bits[k+o] << (7-(j&7)));
		if (--k < 0) {
			o += gsm_fr_map[l];
			k = gsm_fr_map[++l]-1;
		}
		i++;
		j++;
	}

	/* rearrange according to Table 2 of TS 05.03 */
}

static void
tch_fr_disassemble(ubit_t *b_bits, uint8_t *tch_data)
{
	int i, j, k, l, o;

	i = 0; /* counts bits */
	j = 4; /* counts input bits */
	k = gsm_fr_map[0]-1; /* current number bit in element */
	l = 0; /* counts element bits */
	o = 0; /* offset output bits */
	while (i < 260) {
		b_bits[k+o] = (tch_data[j>>3] >> (7-(j&7))) & 1;
		if (--k < 0) {
			o += gsm_fr_map[l];
			k = gsm_fr_map[++l]-1;
		}
		i++;
		j++;
	}

}


/* b(0..259) from d(0..259) according to (corrected) Table 2 of T 05.03 */
static uint16_t d_to_b_index[260] = {
	  5, 52,108,164,220,  4, 11, 16,  3, 10, 15, 21, 42, 98,154,210,
	 51,107,163,219,  9, 25, 29, 41, 97,153,209, 40, 96,152,208, 39,
	 95,151,207, 38, 94,150,206, 50,106,162,218,  2, 20, 32, 37, 93,
	149,205, 24, 28, 44,100,156,212, 36, 92,148,204, 46,102,158,214,
	  1,  8, 14, 35, 19, 23, 31, 43, 99,155,211, 49,105,161,217, 55,
	 58, 61, 64, 67, 70, 73, 76, 79, 82, 85, 88, 91,111,114,117,120,
	123,126,129,132,135,138,141,144,147,167,170,173,176,179,182,185,
	188,191,194,197,200,203,223,226,229,232,235,238,241,244,247,250,
	253,256,259, 45,101,157,213, 48,104,160,216, 54, 57, 60, 63, 66,
	 69, 72, 75, 78, 81, 84, 87, 90,110,113,116,119,122,125,128,131,
	134,137,140,143,146,166,169,172,175,178,181,184,187,190,193,196,
	199,202,222,225,228,231,234,237,240,243,246,249,252,255,258,  0,
	  7, 13, 27, 30, 34, 33, 12, 18, 17, 22, 47,103,159,215, 53, 56,
	 59, 62, 65, 68, 71, 74, 77, 80, 83, 86, 89,109,112,115,118,121,
	124,127,130,133,136,139,142,145,165,168,171,174,177,180,183,186,
	189,192,195,198,201,221,224,227,230,233,236,239,242,245,248,251,
	254,257,  6, 26,
};


static void
tch_fr_d_to_b(ubit_t *b_bits, ubit_t *d_bits)
{
	int i;

	for (i = 0; i < 260; i++)
		b_bits[d_to_b_index[i]] = d_bits[i];
}

static void
tch_fr_b_to_d(ubit_t *d_bits, ubit_t *b_bits)
{
	int i;

	for (i = 0; i < 260; i++)
		d_bits[i] = b_bits[d_to_b_index[i]];
}

static void
tch_fr_unreorder(ubit_t *d, ubit_t *p, ubit_t *u)
{
	int i;

	for (i=0; i<91; i++) {
		d[i<<1] = u[i];
		d[(i<<1)+1] = u[184-i];
	}
	for (i=0; i<3; i++)
		p[i] = u[91+i];
}

static void
tch_fr_reorder(ubit_t *u, ubit_t *d, ubit_t *p)
{
	int i;

	for (i=0; i<91; i++) {
		u[i] = d[i<<1];
		u[184-i] = d[(i<<1)+1];
	}
	for (i=0; i<3; i++)
		u[91+i] = p[i];
}

int
tch_fr_decode(uint8_t *tch_data, sbit_t *bursts)
{
	sbit_t iB[912], cB[456], h;
	ubit_t conv[185], b[260], d[260], p[3];
	int i, rv, len, steal = 0;

	for (i=0; i<8; i++) {
		tch_fr_burst_unmap(&iB[i * 114], &bursts[i * 116], &h, i>>2);
		if (h < 0)
			steal++;
	}

	tch_fr_deinterleave(cB, iB);

	if (steal < 4) {
		osmo_conv_decode(&conv_tch_fr, cB, conv);

		tch_fr_unreorder(d, p, conv);

		for (i=0; i<78; i++)
			d[i+182] = (cB[i+378] < 0) ? 1:0;

		rv = osmo_crc8gen_check_bits(&tch_fr_crc3, d, 50, p);
		if (rv)
			return -1;

		tch_fr_d_to_b(b, d);

		tch_fr_reassemble(tch_data, b);

		len = 33;
	} else {
		rv = xcch_decode_cB(tch_data, cB);
		if (rv)
			return -1;

		len = 23;
	}

	return len;
}

int
tch_fr_encode(ubit_t *bursts, uint8_t *tch_data, int len)
{
	ubit_t iB[912], cB[456], h;
	ubit_t conv[185], b[260], d[260], p[3];
	int i;

	switch (len) {
	case 33: /* TCH FR */
		tch_fr_disassemble(b, tch_data);

		tch_fr_b_to_d(d, b);

		osmo_crc8gen_set_bits(&tch_fr_crc3, d, 50, p);

		tch_fr_reorder(conv, d, p);

		memcpy(cB+378, d+182, 78);

		osmo_conv_encode(&conv_tch_fr, conv, cB);

		h = 0;

		break;
	case 23: /* FACCH */
		xcch_encode_cB(cB, tch_data);

		h = 1;

		break;
	default:
		return -1;
	}

	tch_fr_interleave(cB, iB);

	for (i=0; i<8; i++)
		tch_fr_burst_map(&iB[i * 114], &bursts[i * 116], &h, i>>2);

	return 0;
}
