/*
 * pxxch.c
 *
 * Copyright (c) 2013  Andreas Eversberg <jolly@eversberg.eu>
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <osmocom/core/bits.h>
#include <osmocom/core/conv.h>
#include <osmocom/core/crcgen.h>

#include "pxxch.h"


/*
 * GSM PDTCH parity (FIRE code)
 *
 * g(x) = (x^23 + 1)(x^17 + x^3 + 1)
 *      = x^40 + x^26 + x^23 + x^17 + x^3 + 1
 */

const struct osmo_crc64gen_code pxxch_crc40 = {
	.bits = 40,
	.poly = 0x0004820009ULL,
	.init = 0x0000000000ULL,
	.remainder = 0xffffffffffULL,
};


/*
 * GSM PDTCH CS-2, CS-3, CS-4 parity
 *
 * g(x) = x^16 + x^12 + x^5 + 1
 */

const struct osmo_crc16gen_code pdtch_crc16 = {
	.bits = 16,
	.poly = 0x1021,
	.init = 0x0000,
	.remainder = 0xffff,
};


/*
 * GSM PDTCH convolutional coding
 *
 * G_0 = 1 + x^3 + x^4
 * G_1 = 1 + x + x^3 + x^4
 */

static const uint8_t conv_cs1_next_output[][2] = {
	{ 0, 3 }, { 1, 2 }, { 0, 3 }, { 1, 2 },
	{ 3, 0 }, { 2, 1 }, { 3, 0 }, { 2, 1 },
	{ 3, 0 }, { 2, 1 }, { 3, 0 }, { 2, 1 },
	{ 0, 3 }, { 1, 2 }, { 0, 3 }, { 1, 2 },
};

static const uint8_t conv_cs1_next_state[][2] = {
	{  0,  1 }, {  2,  3 }, {  4,  5 }, {  6,  7 },
	{  8,  9 }, { 10, 11 }, { 12, 13 }, { 14, 15 },
	{  0,  1 }, {  2,  3 }, {  4,  5 }, {  6,  7 },
	{  8,  9 }, { 10, 11 }, { 12, 13 }, { 14, 15 },
};

static const struct osmo_conv_code conv_cs1 = {
	.N = 2,
	.K = 5,
	.len = 224,
	.next_output = conv_cs1_next_output,
	.next_state  = conv_cs1_next_state,
};

static const uint8_t conv_cs2_next_output[][2] = {
	{ 0, 3 }, { 1, 2 }, { 0, 3 }, { 1, 2 },
	{ 3, 0 }, { 2, 1 }, { 3, 0 }, { 2, 1 },
	{ 3, 0 }, { 2, 1 }, { 3, 0 }, { 2, 1 },
	{ 0, 3 }, { 1, 2 }, { 0, 3 }, { 1, 2 },
};

static const uint8_t conv_cs2_next_state[][2] = {
	{  0,  1 }, {  2,  3 }, {  4,  5 }, {  6,  7 },
	{  8,  9 }, { 10, 11 }, { 12, 13 }, { 14, 15 },
	{  0,  1 }, {  2,  3 }, {  4,  5 }, {  6,  7 },
	{  8,  9 }, { 10, 11 }, { 12, 13 }, { 14, 15 },
};

static const struct osmo_conv_code conv_cs2 = {
	.N = 2,
	.K = 5,
	.len = 290,
	.next_output = conv_cs2_next_output,
	.next_state  = conv_cs2_next_state,
};

static const uint8_t conv_cs3_next_output[][2] = {
	{ 0, 3 }, { 1, 2 }, { 0, 3 }, { 1, 2 },
	{ 3, 0 }, { 2, 1 }, { 3, 0 }, { 2, 1 },
	{ 3, 0 }, { 2, 1 }, { 3, 0 }, { 2, 1 },
	{ 0, 3 }, { 1, 2 }, { 0, 3 }, { 1, 2 },
};

static const uint8_t conv_cs3_next_state[][2] = {
	{  0,  1 }, {  2,  3 }, {  4,  5 }, {  6,  7 },
	{  8,  9 }, { 10, 11 }, { 12, 13 }, { 14, 15 },
	{  0,  1 }, {  2,  3 }, {  4,  5 }, {  6,  7 },
	{  8,  9 }, { 10, 11 }, { 12, 13 }, { 14, 15 },
};

static const struct osmo_conv_code conv_cs3 = {
	.N = 2,
	.K = 5,
	.len = 334,
	.next_output = conv_cs3_next_output,
	.next_state  = conv_cs3_next_state,
};


/*
 * GSM PxxCH interleaving and burst mapping
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
pxxch_deinterleave(sbit_t *cB, sbit_t *iB)
{
	int j, k, B;

	for (k=0; k<456; k++) {
		B = k & 3;
		j = 2 * ((49 * k) % 57) + ((k & 7) >> 2);
		cB[k] = iB[B * 114 + j];
	}
}

static void
pxxch_interleave(ubit_t *cB, ubit_t *iB)
{
	int j, k, B;

	for (k=0; k<456; k++) {
		B = k & 3;
		j = 2 * ((49 * k) % 57) + ((k & 7) >> 2);
		iB[B * 114 + j] = cB[k];
	}
}

static void
pxxch_burst_unmap(sbit_t *iB, sbit_t *eB, sbit_t *hl, sbit_t *hn)
{
	memcpy(iB,    eB,    57);
	memcpy(iB+57, eB+59, 57);

	if (hl)
		*hl = eB[57];

	if (hn)
		*hn = eB[58];
}

static void
pxxch_burst_map(ubit_t *iB, ubit_t *eB, ubit_t *hl, ubit_t *hn)
{
	memcpy(eB,    iB,    57);
	memcpy(eB+59, iB+57, 57);

	if (hl)
		eB[57] = *hl;
	if (hn)
		eB[58] = *hn;
}

static ubit_t pdtch_hl_hn_ubit[4][8] = {
	{ 1,1, 1,1, 1,1, 1,1 },
	{ 1,1, 0,0, 1,0, 0,0 },
	{ 0,0, 1,0, 0,0, 0,1 },
	{ 0,0, 0,1, 0,1, 1,0 },
};

static sbit_t pdtch_hl_hn_sbit[4][8] = {
	{ -127,-127, -127,-127, -127,-127, -127,-127 },
	{ -127,-127,  127, 127, -127, 127,  127, 127 },
	{  127, 127, -127, 127,  127, 127,  127,-127 },
	{  127, 127,  127,-127,  127,-127, -127, 127 },
};

static ubit_t usf2six[8][6] = {
	{ 0,0,0, 0,0,0 },
	{ 1,0,0, 1,0,1 },
	{ 0,1,0, 1,1,0 },
	{ 1,1,0, 0,1,1 },
	{ 0,0,1, 0,1,1 },
	{ 1,0,1, 1,1,0 },
	{ 0,1,1, 1,0,1 },
	{ 1,1,1, 0,0,0 },
};

static ubit_t usf2twelve_ubit[8][12] = {
	{ 0,0,0, 0,0,0, 0,0,0, 0,0,0 },
	{ 1,1,0, 1,0,0, 0,0,1, 0,1,1 },
	{ 0,0,1, 1,0,1, 1,1,0, 1,1,0 },
	{ 1,1,1, 0,0,1, 1,1,1, 1,0,1 },
	{ 0,0,0, 0,1,1, 0,1,1, 1,0,1 },
	{ 1,1,0, 1,1,1, 0,1,0, 1,1,0 },
	{ 0,0,1, 1,1,0, 1,0,1, 0,1,1 },
	{ 1,1,1, 0,1,0, 1,0,0, 0,0,0 },
};

static sbit_t usf2twelve_sbit[8][12] = {
	{  127, 127, 127,  127, 127, 127,  127, 127, 127,  127, 127, 127 },
	{ -127,-127, 127, -127, 127, 127,  127, 127,-127,  127,-127,-127 },
	{  127, 127,-127, -127, 127,-127, -127,-127, 127, -127,-127, 127 },
	{ -127,-127,-127,  127, 127,-127, -127,-127,-127, -127, 127,-127 },
	{  127, 127, 127,  127,-127,-127,  127,-127,-127, -127, 127,-127 },
	{ -127,-127, 127, -127,-127,-127,  127,-127, 127, -127,-127, 127 },
	{  127, 127,-127, -127,-127, 127, -127, 127,-127,  127,-127,-127 },
	{ -127,-127,-127,  127,-127, 127, -127, 127, 127,  127, 127, 127 },
};

static uint8_t puncture_cs2[588] = {
	0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1,
	0,0,0,1, 0,0,0,0, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1,
	0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,0, 0,0,0,1, 0,0,0,1,
	0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1,
	0,0,0,1, 0,0,0,0, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1,
	0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,0, 0,0,0,1, 0,0,0,1,
	0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1,
	0,0,0,1, 0,0,0,0, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1,
	0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,0, 0,0,0,1, 0,0,0,1,
	0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1,
	0,0,0,1, 0,0,0,0, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1,
	0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,0, 0,0,0,1, 0,0,0,1,
	0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1,
	0,0,0,1, 0,0,0,0, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1,
	0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,0, 0,0,0,1, 0,0,0,1,
	0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1,
	0,0,0,1, 0,0,0,0, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1,
	0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,0, 0,0,0,1, 0,0,0,1,
	0,0,0,1, 0,0,0,1, 0,0,0,1
};

static uint8_t puncture_cs3[676] = {
	0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1,
	0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1,
	0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1,
	0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1,
	0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1,
	0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1,
	0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1,
	0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1,
	0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1,
	0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1,
	0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1,
	0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1,
	0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1,
	0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1,
	0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1,
	0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1,
	0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1,
	0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1,
	0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1,
	0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1,
	0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1,
	0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,1,0,1,
	0,0,0,1,0,1, 0,0,0,1,0,1, 0,0,0,0
};

int
pdtch_decode(uint8_t *l2_data, sbit_t *bursts, uint8_t *usf_p)
{
	sbit_t iB[456], cB[676], hl_hn[8];
	ubit_t conv[456];
	int i, j, k, rv, best = 0, cs = 0, usf = 0; /* make GCC happy */

	for (i=0; i<4; i++)
		pxxch_burst_unmap(&iB[i * 114], &bursts[i * 116], hl_hn + i*2,
			hl_hn + i*2 + 1);

	for (i=0; i<4; i++) {
		for (j=0, k=0; j<8; j++)
			k += abs(((int)pdtch_hl_hn_sbit[i][j]) - ((int)hl_hn[j]));
		if (i == 0 || k < best) {
			best = k;
			cs = i+1;
		}
	}

	pxxch_deinterleave(cB, iB);

	switch (cs) {
	case 1:
		osmo_conv_decode(&conv_cs1, cB, conv);

		rv = osmo_crc64gen_check_bits(&pxxch_crc40, conv, 184,
			conv+184);
		if (rv)
			return -1;

		osmo_ubit2pbit_ext(l2_data, 0, conv, 0, 184, 1);

		return 23;
	case 2:
		for (i=587, j=455; i>=0; i--)
			if (!puncture_cs2[i])
				cB[i] = cB[j--];
			else
				cB[i] = 0;

		osmo_conv_decode(&conv_cs2, cB, conv);

		for (i=0; i<8; i++) {
			for (j=0, k=0; j<6; j++)
				k += abs(((int)usf2six[i][j]) - ((int)conv[j]));
			if (i == 0 || k < best) {
				best = k;
				usf = i;
			}
		}

		conv[3] = usf & 1;
		conv[4] = (usf >> 1) & 1;
		conv[5] = (usf >> 2) & 1;
		if (usf_p)
			*usf_p = usf;

		rv = osmo_crc16gen_check_bits(&pdtch_crc16, conv+3, 271,
			conv+3+271);
		if (rv)
			return -1;

		osmo_ubit2pbit_ext(l2_data, 0, conv, 3, 271, 1);

		return 34;
	case 3:
		for (i=675, j=455; i>=0; i--)
			if (!puncture_cs3[i])
				cB[i] = cB[j--];
			else
				cB[i] = 0;

		osmo_conv_decode(&conv_cs3, cB, conv);

		for (i=0; i<8; i++) {
			for (j=0, k=0; j<6; j++)
				k += abs(((int)usf2six[i][j]) - ((int)conv[j]));
			if (i == 0 || k < best) {
				best = k;
				usf = i;
			}
		}

		conv[3] = usf & 1;
		conv[4] = (usf >> 1) & 1;
		conv[5] = (usf >> 2) & 1;
		if (usf_p)
			*usf_p = usf;

		rv = osmo_crc16gen_check_bits(&pdtch_crc16, conv+3, 315,
			conv+3+315);
		if (rv)
			return -1;

		osmo_ubit2pbit_ext(l2_data, 0, conv, 3, 315, 1);

		return 40;
	case 4:
		for (i=12; i<456;i++)
			conv[i] = (cB[i] < 0) ? 1:0;

		for (i=0; i<8; i++) {
			for (j=0, k=0; j<12; j++)
				k += abs(((int)usf2twelve_sbit[i][j]) -
								((int)cB[j]));
			if (i == 0 || k < best) {
				best = k;
				usf = i;
			}
		}

		conv[9] = usf & 1;
		conv[10] = (usf >> 1) & 1;
		conv[11] = (usf >> 2) & 1;
		if (usf_p)
			*usf_p = usf;

		rv = osmo_crc16gen_check_bits(&pdtch_crc16, conv+9, 431,
			conv+9+431);
		if (rv)
			return -1;

		osmo_ubit2pbit_ext(l2_data, 0, conv, 9, 431, 1);

		return 54;
	}

	return -1;
}

int
pdtch_encode(ubit_t *bursts, uint8_t *l2_data, uint8_t l2_len)
{
	ubit_t iB[456], cB[676], *hl_hn;
	ubit_t conv[334];
	int i, j, usf;

	switch (l2_len) {
	case 23:
		osmo_pbit2ubit_ext(conv, 0, l2_data, 0, 184, 1);

		osmo_crc64gen_set_bits(&pxxch_crc40, conv, 184, conv+184);

		osmo_conv_encode(&conv_cs1, conv, cB);

		hl_hn = pdtch_hl_hn_ubit[0];

		break;
	case 34:
		osmo_pbit2ubit_ext(conv, 3, l2_data, 0, 271, 1);
		usf = l2_data[0] & 0x7;

		osmo_crc16gen_set_bits(&pdtch_crc16, conv+3, 271, conv+3+271);

		memcpy(conv, usf2six[usf], 6);

		osmo_conv_encode(&conv_cs2, conv, cB);

		for (i=0, j=0; i<588; i++)
			if (!puncture_cs2[i])
				cB[j++] = cB[i];

		hl_hn = pdtch_hl_hn_ubit[1];

		break;
	case 40:
		osmo_pbit2ubit_ext(conv, 3, l2_data, 0, 315, 1);
		usf = l2_data[0] & 0x7;

		osmo_crc16gen_set_bits(&pdtch_crc16, conv+3, 315, conv+3+315);

		memcpy(conv, usf2six[usf], 6);

		osmo_conv_encode(&conv_cs3, conv, cB);

		for (i=0, j=0; i<676; i++)
			if (!puncture_cs3[i])
				cB[j++] = cB[i];

		hl_hn = pdtch_hl_hn_ubit[2];

		break;
	case 54:
		osmo_pbit2ubit_ext(cB, 9, l2_data, 0, 431, 1);
		usf = l2_data[0] & 0x7;

		osmo_crc16gen_set_bits(&pdtch_crc16, cB+9, 431, cB+9+431);

		memcpy(cB, usf2twelve_ubit[usf], 12);

		hl_hn = pdtch_hl_hn_ubit[3];

		break;
	default:
		return -1;
	}

	pxxch_interleave(cB, iB);

	for (i=0; i<4; i++)
		pxxch_burst_map(&iB[i * 114], &bursts[i * 116], hl_hn + i*2,
			hl_hn + i*2 + 1);

	return 0;
}
