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
#include <stdbool.h>
#include <errno.h>

#include <osmocom/core/bits.h>
#include <osmocom/core/utils.h>
#include <osmocom/gsm/gsm44021.h>
#include <osmocom/gsm/gsm_utils.h>
#include <osmocom/gsm/protocol/gsm_04_08.h>
#include <osmocom/isdn/v110.h>
#include <osmocom/trau/csd_ra2.h>
#include <osmocom/trau/csd_raa_prime.h>

#include <osmo-bts/csd_v110.h>
#include <osmo-bts/lchan.h>

/* key is enum gsm48_chan_mode, so assuming a value in range 0..255 */
const struct csd_v110_lchan_desc csd_v110_lchan_desc[256] = {
	[GSM48_CMODE_DATA_14k5] = {
		/* TCH/F14.4: 8 * 36 + 2 bits every 20 ms (14.5 kbit/s) */
		.num_frames = 8,
		.num_frame_bits = 36,	/* D-bits */
		.num_other_bits = 2,	/* M-bits */
		.ra2_ir = 16,
	},
	[GSM48_CMODE_DATA_12k0] = {
		/* TCH/F9.6: 4 * 60 bits every 20 ms (12.0 kbit/s) */
		.num_frames = 4,
		.num_frame_bits = 60,
		.ra2_ir = 16,
	},
	[GSM48_CMODE_DATA_6k0] = {
		/* TCH/[FH]4.8: 2 * 60 bits every 20 ms (6.0 kbit/s) */
		.num_frames = 2,
		.num_frame_bits = 60,
		.ra2_ir = 8,
	},
	[GSM48_CMODE_DATA_3k6] = {
		/* TCH/[FH]2.4: 2 * 36 bits every 20 ms (3.6 kbit/s) */
		.num_frames = 2,
		.num_frame_bits = 36,
		.ra2_ir = 8,
	},
};

/* 3GPP TS 44.021, Figure 4: Coding of data rates (E1/E2/E3 bits) */
static const uint8_t e1e2e3_map[_LCHAN_CSD_M_NUM][3] = {
	[LCHAN_CSD_M_T_600]	= { 1, 0, 0 },
	[LCHAN_CSD_M_T_1200]	= { 0, 1, 0 },
	[LCHAN_CSD_M_T_2400]	= { 1, 1, 0 },
	[LCHAN_CSD_M_T_4800]	= { 0, 1, 1 },
	[LCHAN_CSD_M_T_9600]	= { 0, 1, 1 },
#if 0
	[LCHAN_CSD_M_T_19200]	= { 0, 1, 1 },
	[LCHAN_CSD_M_T_38400]	= { 0, 1, 1 },
	[LCHAN_CSD_M_T_14400]	= { 1, 0, 1 },
	[LCHAN_CSD_M_T_28800]	= { 1, 0, 1 },
#endif
};

int csd_v110_rtp_encode(const struct gsm_lchan *lchan, uint8_t *rtp,
			const uint8_t *data, size_t data_len,
			uint8_t nt48_half_num)
{
	const struct csd_v110_lchan_desc *desc;
	ubit_t ra_bits[80 * 4];

	OSMO_ASSERT(lchan->tch_mode < ARRAY_SIZE(csd_v110_lchan_desc));
	desc = &csd_v110_lchan_desc[lchan->tch_mode];
	if (OSMO_UNLIKELY(desc->num_frames == 0))
		return -ENOTSUP;

	/* TCH/F14.4 is special: RAA' function is employed */
	if (lchan->tch_mode == GSM48_CMODE_DATA_14k5) {
		/* 3GPP TS 44.021, section 10.3 "TCH/F14.4 channel coding"
		 * 3GPP TS 48.020, chapter 11 "THE RAA' FUNCTION" */
		const ubit_t *m_bits = &data[0]; /* M-bits */
		const ubit_t *d_bits = &data[2]; /* D-bits */
		ubit_t c4, c5;

		/* 3GPP TS 48.020, Table 3
		 * | C4 | Date Rate |
		 * | =1 | 14,4 kbit/s |
		 * | =0 | 14.4 kbit/s idle (IWF to BSS only) | */
		c4 = 1;
		/* 3GPP TS 48.020, Table 4
		 * | C5 | BSS to IWF FT | IWF to BSS UFE |
		 * | =1 | idle | framing error |
		 * | =0 | data | no framing error | */
		c5 = 0;

		/* Unless there is a bug, it's highly unlikely */
		OSMO_ASSERT(data_len == CSD_V110_NUM_BITS(desc));

		osmo_csd144_to_atrau_bits(&ra_bits[0], m_bits, d_bits, c4, c5);
		goto ra1_ra2;
	}

	/* handle empty/incomplete Uplink frames gracefully */
	if (OSMO_UNLIKELY(data_len < CSD_V110_NUM_BITS(desc))) {
		/* encode N idle frames as per 3GPP TS 44.021, section 8.1.6 */
		memset(&ra_bits[0], 0x01, sizeof(ra_bits));
		for (unsigned int i = 0; i < desc->num_frames; i++)
			memset(&ra_bits[i * 80], 0x00, 8); /* alignment pattern */
		goto ra1_ra2;
	}

	/* RA1'/RA1: convert from radio rate to an intermediate data rate */
	for (unsigned int i = 0; i < desc->num_frames; i++) {
		struct osmo_v110_decoded_frame df;

		/* convert a V.110 36-/60-bit frame to a V.110 80-bit frame */
		if (desc->num_frame_bits == 60)
			osmo_csd_12k_6k_decode_frame(&df, &data[i * 60], 60);
		else /* desc->num_frame_bits == 36 */
			osmo_csd_3k6_decode_frame(&df, &data[i * 36], 36);

		/* E1 .. E3 must set by out-of-band knowledge */
		if (lchan->csd_mode == LCHAN_CSD_M_NT) {
			/* non-transparent: as per 3GPP TS 48.020, Table 7 */
			/* E1: as per 15.1.2, shall be set to 0 (for BSS-MSC) */
			df.e_bits[0] = 0;
			/* E2: 0 for Q1/Q2, 1 for Q3/Q4 */
			if (desc->num_frames == 4)
				df.e_bits[1] = (i >> 1) & 0x01;
			else
				df.e_bits[1] = nt48_half_num;
			/* E3: 0 for Q1/Q3, 1 for Q2/Q4 */
			df.e_bits[2] = (i >> 0) & 0x01;
		} else {
			/* transparent: as per 3GPP TS 44.021, Figure 4 */
			df.e_bits[0] = e1e2e3_map[lchan->csd_mode][0]; /* E1 */
			df.e_bits[1] = e1e2e3_map[lchan->csd_mode][1]; /* E2 */
			df.e_bits[2] = e1e2e3_map[lchan->csd_mode][2]; /* E3 */
		}

		osmo_v110_encode_frame(&ra_bits[i * 80], 80, &df);
	}

ra1_ra2:
	/* RA1/RA2: convert from an intermediate rate to 64 kbit/s */
	if (desc->ra2_ir == 16)
		osmo_csd_ra2_16k_pack(&rtp[0], &ra_bits[0], RFC4040_RTP_PLEN);
	else /* desc->ra2_ir == 8 */
		osmo_csd_ra2_8k_pack(&rtp[0], &ra_bits[0], RFC4040_RTP_PLEN);

	return RFC4040_RTP_PLEN;
}

static bool check_v110_align(const ubit_t *ra_bits)
{
	int i;
	ubit_t bit0 = 0, bit1 = 1;

	/* The weird code structure is for performance optimization,
	 * to avoid conditionals inside loops. */
	for (i = 0; i < 8; i++)
		bit0 |= ra_bits[i];
	for (i = 1; i < 10; i++)
		bit1 &= ra_bits[i * 8];
	return (bit0 == 0) && (bit1 == 1);
}

int csd_v110_rtp_decode(const struct gsm_lchan *lchan, uint8_t *data,
			uint8_t *align_bits, const uint8_t *rtp, size_t rtp_len)
{
	const struct csd_v110_lchan_desc *desc;
	ubit_t ra_bits[80 * 4];
	uint8_t align_accum = 0;

	OSMO_ASSERT(lchan->tch_mode < ARRAY_SIZE(csd_v110_lchan_desc));
	desc = &csd_v110_lchan_desc[lchan->tch_mode];
	if (OSMO_UNLIKELY(desc->num_frames == 0))
		return -ENOTSUP;

	if (OSMO_UNLIKELY(rtp_len != RFC4040_RTP_PLEN))
		return -EINVAL;

	/* RA1/RA2: convert from 64 kbit/s to an intermediate rate */
	if (desc->ra2_ir == 16)
		osmo_csd_ra2_16k_unpack(&ra_bits[0], &rtp[0], RFC4040_RTP_PLEN);
	else /* desc->ra2_ir == 8 */
		osmo_csd_ra2_8k_unpack(&ra_bits[0], &rtp[0], RFC4040_RTP_PLEN);

	/* TCH/F14.4 is special: RAA' function is employed */
	if (lchan->tch_mode == GSM48_CMODE_DATA_14k5) {
		/* 3GPP TS 44.021, section 10.3 "TCH/F14.4 channel coding"
		 * 3GPP TS 48.020, chapter 11 "THE RAA' FUNCTION" */
		ubit_t *m_bits = &data[0]; /* M-bits */
		ubit_t *d_bits = &data[2]; /* D-bits */
		int rc;

		rc = osmo_csd144_from_atrau_bits(m_bits, d_bits, NULL, NULL, &ra_bits[0]);
		return rc == 0 ? CSD_V110_NUM_BITS(desc) : rc;
	}

	/* RA1'/RA1: convert from an intermediate rate to radio rate */
	for (unsigned int i = 0; i < desc->num_frames; i++) {
		struct osmo_v110_decoded_frame df;

		/* We require our RTP input to consist of aligned V.110
		 * frames.  If we get misaligned input, let's catch it
		 * explicitly, rather than send garbage downstream. */
		if (!check_v110_align(&ra_bits[i * 80]))
			return -EINVAL;
		/* convert a V.110 80-bit frame to a V.110 36-/60-bit frame */
		osmo_v110_decode_frame(&df, &ra_bits[i * 80], 80);
		if (desc->num_frame_bits == 60)
			osmo_csd_12k_6k_encode_frame(&data[i * 60], 60, &df);
		else /* desc->num_frame_bits == 36 */
			osmo_csd_3k6_encode_frame(&data[i * 36], 36, &df);
		/* save bits E2 & E3 that may be needed for RLP alignment */
		align_accum <<= 2;
		align_accum |= df.e_bits[1] << 1;
		align_accum |= df.e_bits[2] << 0;
	}

	if (align_bits)
		*align_bits = align_accum;
	return CSD_V110_NUM_BITS(desc);
}
