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
#include <osmocom/gsm/gsm44021.h>
#include <osmocom/gsm/gsm_utils.h>
#include <osmocom/gsm/protocol/gsm_04_08.h>
#include <osmocom/isdn/v110.h>

#include <osmo-bts/csd_v110.h>
#include <osmo-bts/lchan.h>

/* key is enum gsm48_chan_mode, so assuming a value in range 0..255 */
const struct csd_v110_lchan_desc csd_v110_lchan_desc[256] = {
#if 0
	[GSM48_CMODE_DATA_14k5] = {
		/* TCH/F14.4: 290 bits every 20 ms (14.5 kbit/s) */
		.fr = { .num_blocks = 1, .num_bits = 290 },
	},
#endif
	[GSM48_CMODE_DATA_12k0] = {
		/* TCH/F9.6: 4 * 60 bits every 20 ms (12.0 kbit/s) */
		.fr = { .num_blocks = 4, .num_bits = 60 },
	},
	[GSM48_CMODE_DATA_6k0] = {
		/* TCH/F4.8: 2 * 60 bits every 20 ms (6.0 kbit/s) */
		.fr = { .num_blocks = 2, .num_bits = 60 },
		/* TCH/H4.8: 4 * 60 bits every 40 ms (6.0 kbit/s) */
		.hr = { .num_blocks = 4, .num_bits = 60 },
	},
	[GSM48_CMODE_DATA_3k6] = {
		/* TCH/F2.4: 2 * 36 bits every 20 ms (3.6 kbit/s) */
		.fr = { .num_blocks = 2, .num_bits = 36 },
		/* TCH/H2.4: 4 * 36 bits every 40 ms (3.6 kbit/s) */
		.hr = { .num_blocks = 4, .num_bits = 36 },
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
			const uint8_t *data, size_t data_len)
{
	const struct csd_v110_frame_desc *desc;
	ubit_t ra_bits[80 * 4];

	OSMO_ASSERT(lchan->tch_mode < ARRAY_SIZE(csd_v110_lchan_desc));
	if (lchan->type == GSM_LCHAN_TCH_F)
		desc = &csd_v110_lchan_desc[lchan->tch_mode].fr;
	else
		desc = &csd_v110_lchan_desc[lchan->tch_mode].hr;
	if (OSMO_UNLIKELY(desc->num_blocks == 0))
		return -ENOTSUP;

	/* handle empty/incomplete Uplink frames gracefully */
	if (OSMO_UNLIKELY(data_len < (desc->num_blocks * desc->num_bits))) {
		/* encode N idle frames as per 3GPP TS 44.021, section 8.1.6 */
		memset(&ra_bits[0], 0x01, sizeof(ra_bits));
		for (unsigned int i = 0; i < desc->num_blocks; i++)
			memset(&ra_bits[i * 80], 0x00, 8); /* alignment pattern */
		goto ra1_ra2;
	}

	/* RA1'/RA1: convert from radio rate to an intermediate data rate */
	for (unsigned int i = 0; i < desc->num_blocks; i++) {
		struct osmo_v110_decoded_frame df;

		/* convert a V.110 36-/60-bit frame to a V.110 80-bit frame */
		if (desc->num_bits == 60)
			osmo_csd_12k_6k_decode_frame(&df, &data[i * 60], 60);
		else /* desc->num_bits == 36 */
			osmo_csd_3k6_decode_frame(&df, &data[i * 36], 36);

		/* E1 .. E3 must set by out-of-band knowledge */
		if (lchan->csd_mode == LCHAN_CSD_M_NT) {
			/* non-transparent: as per 3GPP TS 48.020, Table 7 */
			df.e_bits[0] = 0; /* E1: as per 15.1.2, shall be set to 0 (for BSS-MSC) */
			df.e_bits[1] = (i >> 1) & 0x01; /* E2: 0 for Q1/Q2, 1 for Q3/Q4 */
			df.e_bits[2] = (i >> 0) & 0x01; /* E3: 0 for Q1/Q3, 1 for Q2/Q4 */
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
	if (desc->num_blocks == 4) {
		/* 4 * 80 bits (16 kbit/s) => 2 bits per octet */
		for (unsigned int i = 0, j = 0; i < RFC4040_RTP_PLEN; i++) {
			rtp[i]  = (0xff >> 2);
			rtp[i] |= (ra_bits[j++] << 7);
			rtp[i] |= (ra_bits[j++] << 6);
		}
	} else {
		/* 2 * 80 bits (8 kbit/s) => 1 bit per octet */
		for (unsigned int i = 0; i < RFC4040_RTP_PLEN; i++) {
			rtp[i]  = (0xff >> 1);
			rtp[i] |= (ra_bits[i] << 7);
		}
	}

	return RFC4040_RTP_PLEN;
}

int csd_v110_rtp_decode(const struct gsm_lchan *lchan, uint8_t *data,
			const uint8_t *rtp, size_t rtp_len)
{
	const struct csd_v110_frame_desc *desc;
	ubit_t ra_bits[80 * 4];

	OSMO_ASSERT(lchan->tch_mode < ARRAY_SIZE(csd_v110_lchan_desc));
	if (lchan->type == GSM_LCHAN_TCH_F)
		desc = &csd_v110_lchan_desc[lchan->tch_mode].fr;
	else
		desc = &csd_v110_lchan_desc[lchan->tch_mode].hr;
	if (OSMO_UNLIKELY(desc->num_blocks == 0))
		return -ENOTSUP;

	if (OSMO_UNLIKELY(rtp_len != RFC4040_RTP_PLEN))
		return -EINVAL;

	/* RA1/RA2: convert from 64 kbit/s to an intermediate rate */
	if (desc->num_blocks == 4) {
		/* 4 * 80 bits (16 kbit/s) => 2 bits per octet */
		for (unsigned int i = 0, j = 0; i < RFC4040_RTP_PLEN; i++) {
			ra_bits[j++] = (rtp[i] >> 7);
			ra_bits[j++] = (rtp[i] >> 6) & 0x01;
		}
	} else {
		/* 2 * 80 bits (8 kbit/s) => 1 bit per octet */
		for (unsigned int i = 0; i < RFC4040_RTP_PLEN; i++)
			ra_bits[i] = (rtp[i] >> 7);
	}

	/* RA1'/RA1: convert from an intermediate rate to radio rate */
	for (unsigned int i = 0; i < desc->num_blocks; i++) {
		struct osmo_v110_decoded_frame df;

		/* convert a V.110 80-bit frame to a V.110 36-/60-bit frame */
		osmo_v110_decode_frame(&df, &ra_bits[i * 80], 80);
		if (desc->num_bits == 60)
			osmo_csd_12k_6k_encode_frame(&data[i * 60], 60, &df);
		else /* desc->num_bits == 36 */
			osmo_csd_3k6_encode_frame(&data[i * 36], 36, &df);
	}

	return desc->num_blocks * desc->num_bits;
}
