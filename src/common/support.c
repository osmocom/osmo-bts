/* (C) 2011 by Andreas Eversberg <jolly@eversberg.eu>
 *
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#include <sys/types.h>
#include <string.h>
#include <osmocom/gsm/protocol/gsm_12_21.h>
//#include <osmocom/bb/common/osmocom_data.h>
#include <osmo-bts/support.h>

struct bts_support bts_support;

void bts_support_init(void)
{
	struct bts_support *sup = &bts_support;
	int i;

	memset(sup, 0, sizeof(*sup));

	/* crypto supprot */
	sup->a5_1 = 0;
	sup->a5_2 = 0;
	sup->a5_3 = 0;
	sup->a5_4 = 0;
	sup->a5_5 = 0;
	sup->a5_6 = 0;
	sup->a5_7 = 0;
	/* set supported frequencies */
	for(i = 1; i <= 124; i++) // P-GSM
		sup->freq_map[i >> 3] |= (1 << (i & 7));
	for(i = 512; i <= 885; i++) // DCS
		sup->freq_map[i >> 3] |= (1 << (i & 7));
	for(i = 975; i <= 1023; i++) // E-GSM extension
		sup->freq_map[i >> 3] |= (1 << (i & 7));
	sup->freq_map[0] |= 1; // channel 0
	for(i = 955; i <= 974; i++) // R-GSM extension
		sup->freq_map[i >> 3] |= (1 << (i & 7));
	/* channel combinations */
	sup->chan_comb[NM_CHANC_mainBCCH] = 1;
	sup->chan_comb[NM_CHANC_BCCHComb] = 1;
	sup->chan_comb[NM_CHANC_SDCCH] = 1;
	sup->chan_comb[NM_CHANC_TCHFull] = 1;
	sup->chan_comb[NM_CHANC_TCHHalf] = 1;
	/* codec */
	sup->full_v1 = 1;
	sup->full_v2 = 1;
	sup->full_v3 = 1;
	sup->half_v1 = 1;
	sup->half_v3 = 1;
}

char *bts_support_comb_name(uint8_t chan_comb)
{
	if (chan_comb == NM_CHANC_mainBCCH)
		return("BCCH");
	if (chan_comb == NM_CHANC_BCCHComb)
		return("BCCH+SDCCH/4");
	if (chan_comb == NM_CHANC_SDCCH)
		return("SDCCH/8");
	if (chan_comb == NM_CHANC_TCHFull)
		return("TCH/F");
	if (chan_comb == NM_CHANC_TCHHalf)
		return("TCH/H");
	return "Unknown";
}


