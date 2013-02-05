/*
 */
     
     
#include <stdio.h>
#include <stdint.h>
#include <string.h>
     
#include <osmocom/core/bits.h>
#include <osmocom/core/conv.h>
#include <osmocom/core/crcgen.h>

#include "sch.h"


/*
 * GSM SCH parity
 *
 * g(x) = x^10 + x^8 + x^6 + x^5 + x^4 + x^2 + 1
 */

const struct osmo_crc16gen_code sch_crc10 = {
	.bits = 10,
	.poly = 0x175,
	.init = 0x000,
	.remainder = 0x3ff,
};


/*
 * GSM SCH convolutional coding
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
     
static const struct osmo_conv_code conv_sch = {
	.N = 2,
	.K = 5,
	.len = 35,
	.next_output = conv_rach_next_output,
	.next_state  = conv_rach_next_state,
};
 

int
sch_decode(uint8_t *sb_info, sbit_t *burst)
{
	ubit_t conv[35];
	int rv;

	osmo_conv_decode(&conv_sch, burst, conv);

	rv = osmo_crc16gen_check_bits(&sch_crc10, conv, 25, conv+25);
	if (rv)
		return -1;

	osmo_ubit2pbit_ext(sb_info, 0, conv, 0, 25, 1);

	return 0;
}

int
sch_encode(ubit_t *burst, uint8_t *sb_info)
{
	ubit_t conv[35];

	osmo_pbit2ubit_ext(conv, 0, sb_info, 0, 25, 1);

	osmo_crc16gen_set_bits(&sch_crc10, conv, 25, conv+25);

	osmo_conv_encode(&conv_sch, conv, burst);

	return 0;
}
