/*
 * conv_rach.c
 *
 * Convolutional code tables for RACH channels
 *
 * Copyright (C) 2011  Sylvain Munaut <tnt@246tNt.com>
 */
     
     
#include <stdio.h>
#include <stdint.h>
#include <string.h>
     
#include <osmocom/core/bits.h>
#include <osmocom/core/conv.h>
#include <osmocom/core/crcgen.h>

#include "rach.h"


/*
 * GSM RACH parity
 *
 * g(x) = x^6 + x^5 + x^3 + x^2 + x^1 + 1
 */

static const struct osmo_crc8gen_code rach_crc6 = {
	.bits = 6,
	.poly = 0x2f,
	.init = 0x00,
	.remainder = 0x3f,
};


/*
 * GSM RACH convolutional coding
 *
 * G_0 = 1 + x^3 + x^4
 * G_1 = 1 + x + x^3 + x^4
 */

static const uint8_t conv_rach_next_output[][2] = {
	{ 0, 3 }, { 1, 2 }, { 0, 3 }, { 1, 2 },
	{ 3, 0 }, { 2, 1 }, { 3, 0 }, { 2, 1 },
	{ 3, 0 }, { 2, 1 }, { 3, 0 }, { 2, 1 },
	{ 0, 3 }, { 1, 2 }, { 0, 3 }, { 1, 2 },
};
     
static const uint8_t conv_rach_next_state[][2] = {
	{  0,  1 }, {  2,  3 }, {  4,  5 }, {  6,  7 },
	{  8,  9 }, { 10, 11 }, { 12, 13 }, { 14, 15 },
	{  0,  1 }, {  2,  3 }, {  4,  5 }, {  6,  7 },
	{  8,  9 }, { 10, 11 }, { 12, 13 }, { 14, 15 },
};
     
const struct osmo_conv_code conv_rach = {
	.N = 2,
	.K = 5,
	.len = 14,
	.next_output = conv_rach_next_output,
	.next_state  = conv_rach_next_state,
};
 

/*
 * GSM RACH apply BSIC to parity
 *
 * p(j) = p(j) xor b(j)     j = 0, ..., 5
 * b(0) = MSB of PLMN colour code
 * b(5) = LSB of BS colour code
 */

static int
rach_apply_bsic(ubit_t *d, uint8_t bsic)
{
	int i;

	/* Apply it */
	for (i=0; i<6; i++)
		d[8+i] ^= ((bsic >> (5-i)) & 1);

	return 0;
}

int
rach_decode(uint8_t *ra, sbit_t *burst, uint8_t bsic)
{
	ubit_t conv[14];
	int rv;

	osmo_conv_decode(&conv_rach, burst, conv);

	rach_apply_bsic(conv, bsic);

	rv = osmo_crc8gen_check_bits(&rach_crc6, conv, 8, conv+8);
	if (rv)
		return -1;

	osmo_ubit2pbit_ext(ra, 0, conv, 0, 8, 1);

	return 0;
}

int
rach_encode(ubit_t *burst, uint8_t *ra, uint8_t bsic)
{
	ubit_t conv[14];

	osmo_pbit2ubit_ext(conv, 0, ra, 0, 8, 1);

	osmo_crc8gen_set_bits(&rach_crc6, conv, 8, conv+8);

	rach_apply_bsic(conv, bsic);

	osmo_conv_encode(&conv_rach, conv, burst);

	return 0;
}
