/*
 * (C) 2023 by sysmocom - s.f.m.c. GmbH <info@sysmocom.de>
 * Author: Vadim Yanitskiy <vyanitskiy@sysmocom.de>
 *
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdint.h>
#include <errno.h>

#include <osmocom/core/bits.h>
#include <osmocom/core/utils.h>
#include <osmocom/gsm/gsm_utils.h>
#include <osmocom/gsm/protocol/gsm_04_08.h>
#include <osmocom/isdn/v110.h>

#include <osmo-bts/csd_v110.h>
#include <osmo-bts/lchan.h>

#define BBUF_MAX 290

struct test_case {
	const char *name;
	enum gsm_chan_t lchan_type;
	enum gsm48_chan_mode tch_mode;
	enum lchan_csd_mode csd_mode;
};

static const struct test_case tests[] = {
	{
		.name = "TCH/F14.4",
		.lchan_type = GSM_LCHAN_TCH_F,
		.tch_mode = GSM48_CMODE_DATA_14k5,
		.csd_mode = LCHAN_CSD_M_T_14400,
	},
	{
		.name = "TCH/F9.6",
		.lchan_type = GSM_LCHAN_TCH_F,
		.tch_mode = GSM48_CMODE_DATA_12k0,
		.csd_mode = LCHAN_CSD_M_T_9600,
	},
	{
		.name = "TCH/F4.8",
		.lchan_type = GSM_LCHAN_TCH_F,
		.tch_mode = GSM48_CMODE_DATA_6k0,
		.csd_mode = LCHAN_CSD_M_T_4800,
	},
	{
		.name = "TCH/H4.8",
		.lchan_type = GSM_LCHAN_TCH_H,
		.tch_mode = GSM48_CMODE_DATA_6k0,
		.csd_mode = LCHAN_CSD_M_T_4800,
	},
	{
		.name = "TCH/F2.4",
		.lchan_type = GSM_LCHAN_TCH_F,
		.tch_mode = GSM48_CMODE_DATA_3k6,
		.csd_mode = LCHAN_CSD_M_T_2400,
	},
	{
		.name = "TCH/H2.4",
		.lchan_type = GSM_LCHAN_TCH_H,
		.tch_mode = GSM48_CMODE_DATA_3k6,
		.csd_mode = LCHAN_CSD_M_T_600,
	},
};

static void exec_test_case(const struct test_case *tc)
{
	const struct csd_v110_frame_desc *desc;
	uint8_t rtp[RFC4040_RTP_PLEN] = { 0 };
	ubit_t data_enc[BBUF_MAX];
	ubit_t data_dec[BBUF_MAX];
	int rc;

	/* obtain a V.110 frame description for the given channel type/rate */
	OSMO_ASSERT(tc->tch_mode < ARRAY_SIZE(csd_v110_lchan_desc));
	if (tc->lchan_type == GSM_LCHAN_TCH_F)
		desc = &csd_v110_lchan_desc[tc->tch_mode].fr;
	else
		desc = &csd_v110_lchan_desc[tc->tch_mode].hr;

	/* total number of bits carried by a radio interface block */
	const unsigned int bit_num = desc->num_bits * desc->num_blocks;
	if (bit_num == 0) {
		fprintf(stderr, "[i] Skipping '%s' (not implemented)\n", tc->name);
		return;
	}

	fprintf(stderr, "[i] Testing '%s' (bitnum=%u)\n", tc->name, bit_num);

	struct gsm_lchan lchan = {
		.type = tc->lchan_type,
		.tch_mode = tc->tch_mode,
		.csd_mode = tc->csd_mode,
	};

	/* populate the data_enc[] buffer with some bits */
	OSMO_ASSERT(bit_num <= BBUF_MAX);
	for (unsigned int i = 0; i < bit_num; i++)
		data_enc[i] = i & 0x01;

	/* encode an RTP frame and print it */
	rc = csd_v110_rtp_encode(&lchan, &rtp[0], &data_enc[0], bit_num);
	fprintf(stderr, "[i] csd_v110_rtp_encode() returns %d\n", rc);
	if (rc != RFC4040_RTP_PLEN)
		return;
	/* print the encoded RTP frame (16 bytes per row) */
	for (unsigned int i = 0; i < sizeof(rtp) / 16; i++)
		fprintf(stderr, "    %s\n", osmo_hexdump(&rtp[i * 16], 16));

	/* decode the encoded RTP frame */
	rc = csd_v110_rtp_decode(&lchan, &data_dec[0], &rtp[0], sizeof(rtp));
	fprintf(stderr, "[i] csd_v110_rtp_decode() returns %d\n", rc);
	if (rc != bit_num)
		return;
	/* compare data_dec[] vs data_enc[] */
	for (unsigned int i = 0; i < bit_num; i++) {
		if (data_dec[i] == data_enc[i])
			continue;
		fprintf(stderr, "[!] Data mismatch @ %03u: D%u vs E%u\n",
			i, data_dec[i], data_enc[i]);
	}

	fprintf(stderr, "[i] Testing '%s' (IDLE)\n", tc->name);

	/* encode an idle RTP frame and print it */
	rc = csd_v110_rtp_encode(&lchan, &rtp[0], &data_enc[0], 0);
	fprintf(stderr, "[i] csd_v110_rtp_encode() returns %d\n", rc);
	if (rc != RFC4040_RTP_PLEN)
		return;
	/* print the encoded RTP frame (16 bytes per row) */
	for (unsigned int i = 0; i < sizeof(rtp) / 16; i++)
		fprintf(stderr, "    %s\n", osmo_hexdump(&rtp[i * 16], 16));
}

int main(int argc, char **argv)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(tests); i++)
		exec_test_case(&tests[i]);

	return 0;
}
