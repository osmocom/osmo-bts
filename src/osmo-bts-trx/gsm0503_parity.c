
#include <stdint.h>

#include <osmocom/core/crcgen.h>

#include "gsm0503_parity.h"

/*
 * GSM (SACCH) parity (FIRE code)
 *
 * g(x) = (x^23 + 1)(x^17 + x^3 + 1)
 *      = x^40 + x^26 + x^23 + x^17 + x^3 + a1
 */

const struct osmo_crc64gen_code gsm0503_fire_crc40 = {
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

const struct osmo_crc16gen_code gsm0503_cs234_crc16 = {
	.bits = 16,
	.poly = 0x1021,
	.init = 0x0000,
	.remainder = 0xffff,
};


/*
 * GSM RACH parity
 *
 * g(x) = x^6 + x^5 + x^3 + x^2 + x^1 + 1
 */

const struct osmo_crc8gen_code gsm0503_rach_crc6 = {
	.bits = 6,
	.poly = 0x2f,
	.init = 0x00,
	.remainder = 0x3f,
};


/*
 * GSM SCH parity
 *
 * g(x) = x^10 + x^8 + x^6 + x^5 + x^4 + x^2 + 1
 */

const struct osmo_crc16gen_code gsm0503_sch_crc10 = {
	.bits = 10,
	.poly = 0x175,
	.init = 0x000,
	.remainder = 0x3ff,
};


/*
 * GSM TCH FR/EFR parity
 *
 * g(x) = x^3 + x + 1
 */

const struct osmo_crc8gen_code gsm0503_tch_fr_crc3 = {
	.bits = 3,
	.poly = 0x3,
	.init = 0x0,
	.remainder = 0x7,
};

