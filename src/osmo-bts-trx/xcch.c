/*
 * xcch.c
 *
 * Copyright (c) 2011  Sylvain Munaut <tnt@246tNt.com>
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <osmocom/core/bits.h>
#include <osmocom/core/conv.h>
#include <osmocom/core/crcgen.h>

#include "xcch.h"


/*
 * GSM xCCH parity (FIRE code)
 *
 * g(x) = (x^23 + 1)(x^17 + x^3 + 1)
 *      = x^40 + x^26 + x^23 + x^17 + x^3 + 1
 */

const struct osmo_crc64gen_code xcch_crc40 = {
	.bits = 40,
	.poly = 0x0004820009ULL,
	.init = 0x0000000000ULL,
	.remainder = 0xffffffffffULL,
};


/*
 * GSM xCCH convolutional coding
 *
 * G_0 = 1 + x^3 + x^4
 * G_1 = 1 + x + x^3 + x^4
 */

static const uint8_t conv_xcch_next_output[][2] = {
	{ 0, 3 }, { 1, 2 }, { 0, 3 }, { 1, 2 },
	{ 3, 0 }, { 2, 1 }, { 3, 0 }, { 2, 1 },
	{ 3, 0 }, { 2, 1 }, { 3, 0 }, { 2, 1 },
	{ 0, 3 }, { 1, 2 }, { 0, 3 }, { 1, 2 },
};

static const uint8_t conv_xcch_next_state[][2] = {
	{  0,  1 }, {  2,  3 }, {  4,  5 }, {  6,  7 },
	{  8,  9 }, { 10, 11 }, { 12, 13 }, { 14, 15 },
	{  0,  1 }, {  2,  3 }, {  4,  5 }, {  6,  7 },
	{  8,  9 }, { 10, 11 }, { 12, 13 }, { 14, 15 },
};

static const struct osmo_conv_code conv_xcch = {
	.N = 2,
	.K = 5,
	.len = 224,
	.next_output = conv_xcch_next_output,
	.next_state  = conv_xcch_next_state,
};


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

static void
xcch_deinterleave(sbit_t *cB, sbit_t *iB)
{
	int j, k, B;

	for (k=0; k<456; k++) {
		B = k & 3;
		j = 2 * ((49 * k) % 57) + ((k & 7) >> 2);
		cB[k] = iB[B * 114 + j];
	}
}

static void
xcch_interleave(ubit_t *cB, ubit_t *iB)
{
	int j, k, B;

	for (k=0; k<456; k++) {
		B = k & 3;
		j = 2 * ((49 * k) % 57) + ((k & 7) >> 2);
		iB[B * 114 + j] = cB[k];
	}
}

static void
xcch_burst_unmap(sbit_t *iB, sbit_t *eB, sbit_t *hl, sbit_t *hn)
{
	memcpy(iB,    eB,    57);
	memcpy(iB+57, eB+59, 57);

	if (hl)
		*hl = eB[57];

	if (hn)
		*hn = eB[58];
}

static void
xcch_burst_map(ubit_t *iB, ubit_t *eB, ubit_t *hl, ubit_t *hn)
{
	memcpy(eB,    iB,    57);
	memcpy(eB+59, iB+57, 57);

	if (hl)
		eB[57] = *hl;
	if (hn)
		eB[58] = *hn;
}

int
xcch_decode_cB(uint8_t *l2_data, sbit_t *cB)
{
	ubit_t conv[224];
	int rv;

	osmo_conv_decode(&conv_xcch, cB, conv);

	rv = osmo_crc64gen_check_bits(&xcch_crc40, conv, 184, conv+184);
	if (rv)
		return -1;

	osmo_ubit2pbit_ext(l2_data, 0, conv, 0, 184, 1);

	return 0;
}

int
xcch_decode(uint8_t *l2_data, sbit_t *bursts)
{
	sbit_t iB[456], cB[456];
	int i;

	for (i=0; i<4; i++)
		xcch_burst_unmap(&iB[i * 114], &bursts[i * 116], NULL, NULL);

	xcch_deinterleave(cB, iB);

	return xcch_decode_cB(l2_data, cB);
}

int
xcch_encode_cB(ubit_t *cB, uint8_t *l2_data)
{
	ubit_t conv[224];

	osmo_pbit2ubit_ext(conv, 0, l2_data, 0, 184, 1);

	osmo_crc64gen_set_bits(&xcch_crc40, conv, 184, conv+184);

	osmo_conv_encode(&conv_xcch, conv, cB);

	return 0;
}

int
xcch_encode(ubit_t *bursts, uint8_t *l2_data)
{
	ubit_t iB[456], cB[456], hl = 1, hn = 1;
	int i;

	xcch_encode_cB(cB, l2_data);

	xcch_interleave(cB, iB);

	for (i=0; i<4; i++)
		xcch_burst_map(&iB[i * 114], &bursts[i * 116], &hl, &hn);

	return 0;
}
