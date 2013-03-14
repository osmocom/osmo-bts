
#include <stdint.h>

#include <osmocom/core/conv.h>

#include "gsm0503_conv.h"

/*
 * GSM convolutional coding
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


const struct osmo_conv_code gsm0503_conv_xcch = {
	.N = 2,
	.K = 5,
	.len = 224,
	.next_output = conv_xcch_next_output,
	.next_state  = conv_xcch_next_state,
};


const struct osmo_conv_code gsm0503_conv_cs2 = {
	.N = 2,
	.K = 5,
	.len = 290,
	.next_output = conv_xcch_next_output,
	.next_state  = conv_xcch_next_state,
};


const struct osmo_conv_code gsm0503_conv_cs3 = {
	.N = 2,
	.K = 5,
	.len = 334,
	.next_output = conv_xcch_next_output,
	.next_state  = conv_xcch_next_state,
};


const struct osmo_conv_code gsm0503_conv_rach = {
	.N = 2,
	.K = 5,
	.len = 14,
	.next_output = conv_xcch_next_output,
	.next_state  = conv_xcch_next_state,
};


const struct osmo_conv_code gsm0503_conv_sch = {
	.N = 2,
	.K = 5,
	.len = 35,
	.next_output = conv_xcch_next_output,
	.next_state  = conv_xcch_next_state,
};


const struct osmo_conv_code gsm0503_conv_tch_fr = {
	.N = 2,
	.K = 5,
	.len = 185,
	.next_output = conv_xcch_next_output,
	.next_state  = conv_xcch_next_state,
};


