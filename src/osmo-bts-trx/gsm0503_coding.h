/* (C) 2013 by Andreas Eversberg <jolly@eversberg.eu>
 * (C) 2015 by Alexander Chemeris <Alexander.Chemeris@fairwaves.co>
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
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef _0503_CODING_H
#define _0503_CODING_H

#define GSM0503_GPRS_BURSTS_NBITS	(116 * 4)
#define GSM0503_EGPRS_BURSTS_NBITS	(348 * 4)

struct osmo_conv_code;

enum egprs_mcs {
	EGPRS_MCS0,
	EGPRS_MCS1,
	EGPRS_MCS2,
	EGPRS_MCS3,
	EGPRS_MCS4,
	EGPRS_MCS5,
	EGPRS_MCS6,
	EGPRS_MCS7,
	EGPRS_MCS8,
	EGPRS_MCS9,
	EGPRS_NUM_MCS,
};

int xcch_decode(uint8_t *l2_data, sbit_t *bursts,
	int *n_errors, int *n_bits_total);
int xcch_encode(ubit_t *bursts, uint8_t *l2_data);
int pdtch_decode(uint8_t *l2_data, sbit_t *bursts, uint8_t *usf_p,
	int *n_errors, int *n_bits_total);
int pdtch_egprs_decode(uint8_t *l2_data, sbit_t *bursts, uint16_t nbits,
	uint8_t *usf_p, int *n_errors, int *n_bits_total);
int pdtch_encode(ubit_t *bursts, uint8_t *l2_data, uint8_t l2_len);
int pdtch_egprs_encode(ubit_t *bursts, uint8_t *l2_data, uint8_t l2_len);
int tch_fr_decode(uint8_t *tch_data, sbit_t *bursts, int net_order,
	int efr, int *n_errors, int *n_bits_total);
int tch_fr_encode(ubit_t *bursts, uint8_t *tch_data, int len, int net_order);
int tch_hr_decode(uint8_t *tch_data, sbit_t *bursts, int odd,
	int *n_errors, int *n_bits_total);
int tch_hr_encode(ubit_t *bursts, uint8_t *tch_data, int len);
int tch_afs_decode(uint8_t *tch_data, sbit_t *bursts, int codec_mode_req,
	uint8_t *codec, int codecs, uint8_t *ft, uint8_t *cmr,
	int *n_errors, int *n_bits_total);
int tch_afs_encode(ubit_t *bursts, uint8_t *tch_data, int len,
	int codec_mode_req, uint8_t *codec, int codecs, uint8_t ft,
	uint8_t cmr);
int tch_ahs_decode(uint8_t *tch_data, sbit_t *bursts, int odd,
	int codec_mode_req, uint8_t *codec, int codecs, uint8_t *ft, 
	uint8_t *cmr, int *n_errors, int *n_bits_total);
int tch_ahs_encode(ubit_t *bursts, uint8_t *tch_data, int len,
	int codec_mode_req, uint8_t *codec, int codecs, uint8_t ft,
	uint8_t cmr);
int rach_decode(uint8_t *ra, sbit_t *burst, uint8_t bsic);
int rach_encode(ubit_t *burst, uint8_t *ra, uint8_t bsic);
int sch_decode(uint8_t *sb_info, sbit_t *burst);
int sch_encode(ubit_t *burst, uint8_t *sb_info);

#endif /* _0503_CODING_H */
