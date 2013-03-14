
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include <osmocom/core/bits.h>
#include <osmocom/core/conv.h>
#include <osmocom/core/crcgen.h>
#include <osmocom/codec/codec.h>

#include "gsm0503_conv.h"
#include "gsm0503_parity.h"
#include "gsm0503_mapping.h"
#include "gsm0503_interleaving.h"
#include "gsm0503_tables.h"
#include "gsm0503_coding.h"

static int _xcch_decode_cB(uint8_t *l2_data, sbit_t *cB)
{
	ubit_t conv[224];
	int rv;

	osmo_conv_decode(&gsm0503_conv_xcch, cB, conv);

	rv = osmo_crc64gen_check_bits(&gsm0503_fire_crc40, conv, 184, conv+184);
	if (rv)
		return -1;

	osmo_ubit2pbit_ext(l2_data, 0, conv, 0, 184, 1);

	return 0;
}

static int _xcch_encode_cB(ubit_t *cB, uint8_t *l2_data)
{
	ubit_t conv[224];

	osmo_pbit2ubit_ext(conv, 0, l2_data, 0, 184, 1);

	osmo_crc64gen_set_bits(&gsm0503_fire_crc40, conv, 184, conv+184);

	osmo_conv_encode(&gsm0503_conv_xcch, conv, cB);

	return 0;
}


/*
 * GSM xCCH block transcoding
 */

int xcch_decode(uint8_t *l2_data, sbit_t *bursts)
{
	sbit_t iB[456], cB[456];
	int i;

	for (i=0; i<4; i++)
		gsm0503_xcch_burst_unmap(&iB[i * 114], &bursts[i * 116], NULL,
			NULL);

	gsm0503_xcch_deinterleave(cB, iB);

	return _xcch_decode_cB(l2_data, cB);
}

int xcch_encode(ubit_t *bursts, uint8_t *l2_data)
{
	ubit_t iB[456], cB[456], hl = 1, hn = 1;
	int i;

	_xcch_encode_cB(cB, l2_data);

	gsm0503_xcch_interleave(cB, iB);

	for (i=0; i<4; i++)
		gsm0503_xcch_burst_map(&iB[i * 114], &bursts[i * 116], &hl,
			&hn);

	return 0;
}


/*
 * GSM PDTCH block transcoding
 */

int pdtch_decode(uint8_t *l2_data, sbit_t *bursts, uint8_t *usf_p)
{
	sbit_t iB[456], cB[676], hl_hn[8];
	ubit_t conv[456];
	int i, j, k, rv, best = 0, cs = 0, usf = 0; /* make GCC happy */

	for (i=0; i<4; i++)
		gsm0503_xcch_burst_unmap(&iB[i * 114], &bursts[i * 116],
			hl_hn + i*2, hl_hn + i*2 + 1);

	for (i=0; i<4; i++) {
		for (j=0, k=0; j<8; j++)
			k += abs(((int)gsm0503_pdtch_hl_hn_sbit[i][j]) -
							((int)hl_hn[j]));
		if (i == 0 || k < best) {
			best = k;
			cs = i+1;
		}
	}

	gsm0503_xcch_deinterleave(cB, iB);

	switch (cs) {
	case 1:
		osmo_conv_decode(&gsm0503_conv_xcch, cB, conv);

		rv = osmo_crc64gen_check_bits(&gsm0503_fire_crc40, conv, 184,
			conv+184);
		if (rv)
			return -1;

		osmo_ubit2pbit_ext(l2_data, 0, conv, 0, 184, 1);

		return 23;
	case 2:
		for (i=587, j=455; i>=0; i--)
			if (!gsm0503_puncture_cs2[i])
				cB[i] = cB[j--];
			else
				cB[i] = 0;

		osmo_conv_decode(&gsm0503_conv_cs2, cB, conv);

		for (i=0; i<8; i++) {
			for (j=0, k=0; j<6; j++)
				k += abs(((int)gsm0503_usf2six[i][j]) -
							((int)conv[j]));
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

		rv = osmo_crc16gen_check_bits(&gsm0503_cs234_crc16, conv+3, 271,
			conv+3+271);
		if (rv)
			return -1;

		osmo_ubit2pbit_ext(l2_data, 0, conv, 3, 271, 1);

		return 34;
	case 3:
		for (i=675, j=455; i>=0; i--)
			if (!gsm0503_puncture_cs3[i])
				cB[i] = cB[j--];
			else
				cB[i] = 0;

		osmo_conv_decode(&gsm0503_conv_cs3, cB, conv);

		for (i=0; i<8; i++) {
			for (j=0, k=0; j<6; j++)
				k += abs(((int)gsm0503_usf2six[i][j]) -
							((int)conv[j]));
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

		rv = osmo_crc16gen_check_bits(&gsm0503_cs234_crc16, conv+3, 315,
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
				k += abs(((int)gsm0503_usf2twelve_sbit[i][j]) -
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

		rv = osmo_crc16gen_check_bits(&gsm0503_cs234_crc16, conv+9, 431,
			conv+9+431);
		if (rv)
			return -1;

		osmo_ubit2pbit_ext(l2_data, 0, conv, 9, 431, 1);

		return 54;
	}

	return -1;
}

int pdtch_encode(ubit_t *bursts, uint8_t *l2_data, uint8_t l2_len)
{
	ubit_t iB[456], cB[676];
	const ubit_t *hl_hn;
	ubit_t conv[334];
	int i, j, usf;

	switch (l2_len) {
	case 23:
		osmo_pbit2ubit_ext(conv, 0, l2_data, 0, 184, 1);

		osmo_crc64gen_set_bits(&gsm0503_fire_crc40, conv, 184,
			conv+184);

		osmo_conv_encode(&gsm0503_conv_xcch, conv, cB);

		hl_hn = gsm0503_pdtch_hl_hn_ubit[0];

		break;
	case 34:
		osmo_pbit2ubit_ext(conv, 3, l2_data, 0, 271, 1);
		usf = l2_data[0] & 0x7;

		osmo_crc16gen_set_bits(&gsm0503_cs234_crc16, conv+3, 271,
			conv+3+271);

		memcpy(conv, gsm0503_usf2six[usf], 6);

		osmo_conv_encode(&gsm0503_conv_cs2, conv, cB);

		for (i=0, j=0; i<588; i++)
			if (!gsm0503_puncture_cs2[i])
				cB[j++] = cB[i];

		hl_hn = gsm0503_pdtch_hl_hn_ubit[1];

		break;
	case 40:
		osmo_pbit2ubit_ext(conv, 3, l2_data, 0, 315, 1);
		usf = l2_data[0] & 0x7;

		osmo_crc16gen_set_bits(&gsm0503_cs234_crc16, conv+3, 315,
			conv+3+315);

		memcpy(conv, gsm0503_usf2six[usf], 6);

		osmo_conv_encode(&gsm0503_conv_cs3, conv, cB);

		for (i=0, j=0; i<676; i++)
			if (!gsm0503_puncture_cs3[i])
				cB[j++] = cB[i];

		hl_hn = gsm0503_pdtch_hl_hn_ubit[2];

		break;
	case 54:
		osmo_pbit2ubit_ext(cB, 9, l2_data, 0, 431, 1);
		usf = l2_data[0] & 0x7;

		osmo_crc16gen_set_bits(&gsm0503_cs234_crc16, cB+9, 431,
			cB+9+431);

		memcpy(cB, gsm0503_usf2twelve_ubit[usf], 12);

		hl_hn = gsm0503_pdtch_hl_hn_ubit[3];

		break;
	default:
		return -1;
	}

	gsm0503_xcch_interleave(cB, iB);

	for (i=0; i<4; i++)
		gsm0503_xcch_burst_map(&iB[i * 114], &bursts[i * 116],
			hl_hn + i*2, hl_hn + i*2 + 1);

	return 0;
}


/*
 * GSM TCH/F FR/EFR transcoding
 */

static void tch_fr_reassemble(uint8_t *tch_data, ubit_t *b_bits, int net_order)
{
	int i, j, k, l, o;

	tch_data[0] = 0xd << 4;
	memset(tch_data + 1, 0, 32);

	if (net_order) {
		i = 0; /* counts bits */
		j = 4; /* counts output bits */
		while (i < 260) {
			tch_data[j>>3] |= (b_bits[i] << (7-(j&7)));
			i++;
			j++;
		}
		return;
	}

	/* reassemble d-bits */
	i = 0; /* counts bits */
	j = 4; /* counts output bits */
	k = gsm0503_gsm_fr_map[0]-1; /* current number bit in element */
	l = 0; /* counts element bits */
	o = 0; /* offset input bits */
	while (i < 260) {
		tch_data[j>>3] |= (b_bits[k+o] << (7-(j&7)));
		if (--k < 0) {
			o += gsm0503_gsm_fr_map[l];
			k = gsm0503_gsm_fr_map[++l]-1;
		}
		i++;
		j++;
	}
}

static void tch_efr_reassemble(uint8_t *tch_data, ubit_t *b_bits)
{
	int i, j;

	tch_data[0] = 0xc << 4;
	memset(tch_data + 1, 0, 30);

	i = 0; /* counts bits */
	j = 4; /* counts output bits */
	while (i < 244) {
		tch_data[j>>3] |= (b_bits[i] << (7-(j&7)));
		i++;
		j++;
	}
}

static void tch_efr_disassemble(ubit_t *b_bits, uint8_t *tch_data)
{
	int i, j;

	i = 0; /* counts bits */
	j = 4; /* counts output bits */
	while (i < 244) {
		b_bits[i] = (tch_data[j>>3] >> (7-(j&7))) & 1;
		i++;
		j++;
	}
}

static void tch_fr_disassemble(ubit_t *b_bits, uint8_t *tch_data, int net_order)
{
	int i, j, k, l, o;

	if (net_order) {
		i = 0; /* counts bits */
		j = 4; /* counts output bits */
		while (i < 260) {
			b_bits[i] = (tch_data[j>>3] >> (7-(j&7))) & 1;
			i++;
			j++;
		}
		return;
	}

	i = 0; /* counts bits */
	j = 4; /* counts input bits */
	k = gsm0503_gsm_fr_map[0]-1; /* current number bit in element */
	l = 0; /* counts element bits */
	o = 0; /* offset output bits */
	while (i < 260) {
		b_bits[k+o] = (tch_data[j>>3] >> (7-(j&7))) & 1;
		if (--k < 0) {
			o += gsm0503_gsm_fr_map[l];
			k = gsm0503_gsm_fr_map[++l]-1;
		}
		i++;
		j++;
	}

}

static void tch_fr_d_to_b(ubit_t *b_bits, ubit_t *d_bits)
{
	int i;

	for (i = 0; i < 260; i++)
		b_bits[gsm610_bitorder[i]] = d_bits[i];
}

static void tch_fr_b_to_d(ubit_t *d_bits, ubit_t *b_bits)
{
	int i;

	for (i = 0; i < 260; i++)
		d_bits[i] = b_bits[gsm610_bitorder[i]];
}

static void tch_efr_d_to_w(ubit_t *b_bits, ubit_t *d_bits)
{
	int i;

	for (i = 0; i < 260; i++)
		b_bits[gsm660_bitorder[i]] = d_bits[i];
}

static void tch_efr_w_to_d(ubit_t *d_bits, ubit_t *b_bits)
{
	int i;

	for (i = 0; i < 260; i++)
		d_bits[i] = b_bits[gsm660_bitorder[i]];
}

static void tch_efr_protected(ubit_t *s_bits, ubit_t *b_bits)
{
	int i;

	for (i = 0; i < 65; i++)
		b_bits[i] = s_bits[gsm0503_gsm_efr_protected_bits[i]-1];
}

static void tch_fr_unreorder(ubit_t *d, ubit_t *p, ubit_t *u)
{
	int i;

	for (i=0; i<91; i++) {
		d[i<<1] = u[i];
		d[(i<<1)+1] = u[184-i];
	}
	for (i=0; i<3; i++)
		p[i] = u[91+i];
}

static void tch_fr_reorder(ubit_t *u, ubit_t *d, ubit_t *p)
{
	int i;

	for (i=0; i<91; i++) {
		u[i] = d[i<<1];
		u[184-i] = d[(i<<1)+1];
	}
	for (i=0; i<3; i++)
		u[91+i] = p[i];
}

static void tch_efr_reorder(ubit_t *w, ubit_t *s, ubit_t *p)
{
	memcpy(w, s, 71);
	w[71] = w[72] = s[69];
	memcpy(w+73, s+71, 50);
	w[123] = w[124] = s[119];
	memcpy(w+125, s+121, 53);
	w[178] = w[179] = s[172];
	memcpy(w+180, s+174, 50);
	w[230] = w[231] = s[222];
	memcpy(w+232, s+224, 20);
	memcpy(w+252, p, 8);
}

static void tch_efr_unreorder(ubit_t *s, ubit_t *p, ubit_t *w)
{
	int sum;

	memcpy(s, w, 71);
	sum = s[69] + w[71] + w[72];
	s[69] = (sum > 2);
	memcpy(s+71, w+73, 50);
	sum = s[119] + w[123] + w[124];
	s[119] = (sum > 2);
	memcpy(s+121, w+125, 53);
	sum = s[172] + w[178] + w[179];
	s[172] = (sum > 2);
	memcpy(s+174, w+180, 50);
	sum = s[220] + w[230] + w[231];
	s[222] = (sum > 2);
	memcpy(s+224, w+232, 20);
	memcpy(p, w+252, 8);
}
int tch_fr_decode(uint8_t *tch_data, sbit_t *bursts, int net_order, int efr)
{
	sbit_t iB[912], cB[456], h;
	ubit_t conv[185], s[244], w[260], b[65], d[260], p[8];
	int i, rv, len, steal = 0;

	for (i=0; i<8; i++) {
		gsm0503_tch_burst_unmap(&iB[i * 114], &bursts[i * 116], &h,
			i>>2);
		steal -= h;
	}

	gsm0503_tch_fr_deinterleave(cB, iB);

	if (steal > 0) {
		rv = _xcch_decode_cB(tch_data, cB);
		if (rv)
			return -1;

		return 23;
	}

	osmo_conv_decode(&gsm0503_conv_tch_fr, cB, conv);

	tch_fr_unreorder(d, p, conv);

	for (i=0; i<78; i++)
		d[i+182] = (cB[i+378] < 0) ? 1:0;

	rv = osmo_crc8gen_check_bits(&gsm0503_tch_fr_crc3, d, 50, p);
	if (rv)
		return -1;


	if (efr) {
		tch_efr_d_to_w(w, d);

		tch_efr_unreorder(s, p, w);

		tch_efr_protected(s, b);

		rv = osmo_crc8gen_check_bits(&gsm0503_tch_efr_crc8, b,
			65, p);
		if (rv)
			return -1;

		tch_efr_reassemble(tch_data, s);

		len = 31;
	} else {
		tch_fr_d_to_b(w, d);

		tch_fr_reassemble(tch_data, w, net_order);

		len = 33;
	}

	return len;
}

int tch_fr_encode(ubit_t *bursts, uint8_t *tch_data, int len, int net_order)
{
	ubit_t iB[912], cB[456], h;
	ubit_t conv[185], w[260], b[65], s[244], d[260], p[8];
	int i;

	switch (len) {
	case 31: /* TCH EFR */

		tch_efr_disassemble(s, tch_data);

		tch_efr_protected(s, b);

		osmo_crc8gen_set_bits(&gsm0503_tch_efr_crc8, b, 65, p);

		tch_efr_reorder(w, s, p);

		tch_efr_w_to_d(d, w);

		goto coding_efr_fr;
	case 33: /* TCH FR */
		tch_fr_disassemble(w, tch_data, net_order);

		tch_fr_b_to_d(d, w);

coding_efr_fr:
		osmo_crc8gen_set_bits(&gsm0503_tch_fr_crc3, d, 50, p);

		tch_fr_reorder(conv, d, p);

		memcpy(cB+378, d+182, 78);

		osmo_conv_encode(&gsm0503_conv_tch_fr, conv, cB);

		h = 0;

		break;
	case 23: /* FACCH */
		_xcch_encode_cB(cB, tch_data);

		h = 1;

		break;
	default:
		return -1;
	}

	gsm0503_tch_fr_interleave(cB, iB);

	for (i=0; i<8; i++)
		gsm0503_tch_burst_map(&iB[i * 114], &bursts[i * 116], &h, i>>2);

	return 0;
}

/*
 * GSM RACH transcoding
 */

/*
 * GSM RACH apply BSIC to parity
 *
 * p(j) = p(j) xor b(j)     j = 0, ..., 5
 * b(0) = MSB of PLMN colour code
 * b(5) = LSB of BS colour code
 */

static int rach_apply_bsic(ubit_t *d, uint8_t bsic)
{
	int i;

	/* Apply it */
	for (i=0; i<6; i++)
		d[8+i] ^= ((bsic >> (5-i)) & 1);

	return 0;
}

int rach_decode(uint8_t *ra, sbit_t *burst, uint8_t bsic)
{
	ubit_t conv[14];
	int rv;

	osmo_conv_decode(&gsm0503_conv_rach, burst, conv);

	rach_apply_bsic(conv, bsic);

	rv = osmo_crc8gen_check_bits(&gsm0503_rach_crc6, conv, 8, conv+8);
	if (rv)
		return -1;

	osmo_ubit2pbit_ext(ra, 0, conv, 0, 8, 1);

	return 0;
}

int rach_encode(ubit_t *burst, uint8_t *ra, uint8_t bsic)
{
	ubit_t conv[14];

	osmo_pbit2ubit_ext(conv, 0, ra, 0, 8, 1);

	osmo_crc8gen_set_bits(&gsm0503_rach_crc6, conv, 8, conv+8);

	rach_apply_bsic(conv, bsic);

	osmo_conv_encode(&gsm0503_conv_rach, conv, burst);

	return 0;
}


/*
 * GSM SCH transcoding
 */

int sch_decode(uint8_t *sb_info, sbit_t *burst)
{
	ubit_t conv[35];
	int rv;

	osmo_conv_decode(&gsm0503_conv_sch, burst, conv);

	rv = osmo_crc16gen_check_bits(&gsm0503_sch_crc10, conv, 25, conv+25);
	if (rv)
		return -1;

	osmo_ubit2pbit_ext(sb_info, 0, conv, 0, 25, 1);

	return 0;
}

int sch_encode(ubit_t *burst, uint8_t *sb_info)
{
	ubit_t conv[35];

	osmo_pbit2ubit_ext(conv, 0, sb_info, 0, 25, 1);

	osmo_crc16gen_set_bits(&gsm0503_sch_crc10, conv, 25, conv+25);

	osmo_conv_encode(&gsm0503_conv_sch, conv, burst);

	return 0;
}

