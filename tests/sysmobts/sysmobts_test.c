/*
 * (C) 2013 by Holger Hans Peter Freyther
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
 */

#include <osmo-bts/bts.h>

#include "femtobts.h"
#include "l1_if.h"
#include "utils.h"

#include <stdio.h>

static int direct_map[][3] = {
	{ GSM_BAND_850,		GsmL1_FreqBand_850,	128	},
	{ GSM_BAND_900,		GsmL1_FreqBand_900,	1	},
	{ GSM_BAND_1800, 	GsmL1_FreqBand_1800,	600	},
	{ GSM_BAND_1900,	GsmL1_FreqBand_1900,	600	},
};

static int dcs_to_dcs[][3] = {
	{ GSM_BAND_900,		GsmL1_FreqBand_1800,	600	},
	{ GSM_BAND_1800,	GsmL1_FreqBand_900,	1	},
	{ GSM_BAND_900,		-1,			438	},
};

static int pcs_to_pcs[][3] = {
	{ GSM_BAND_850,		GsmL1_FreqBand_1900,	512	},
	{ GSM_BAND_1900,	GsmL1_FreqBand_850,	128	},
	{ GSM_BAND_900,		-1,			438	},
};

static void test_sysmobts_auto_band(void)
{
	struct gsm_bts bts;
	struct gsm_bts_role_bts btsb; 
	struct gsm_bts_trx trx;
	struct femtol1_hdl hdl;
	int i;

	memset(&bts, 0, sizeof(bts));
	memset(&btsb, 0, sizeof(btsb));
	memset(&trx, 0, sizeof(trx));
	memset(&hdl, 0, sizeof(hdl));
	bts.role = &btsb;
	trx.bts = &bts;
	trx.role_bts.l1h = &hdl;

	/* claim to support all hw_info's */
	hdl.hw_info.band_support = GSM_BAND_850 | GSM_BAND_900 |
					GSM_BAND_1800 | GSM_BAND_1900;

	/* start with the current option */
	printf("Testing the no auto-band mapping.\n");
	for (i = 0; i < ARRAY_SIZE(direct_map); ++i) {
		uint16_t arfcn;
		int res;

		btsb.auto_band = 0;
		bts.band = direct_map[i][0];
		arfcn = direct_map[i][2];
		res = sysmobts_select_femto_band(&trx, arfcn);
		printf("No auto-band band(%d) arfcn(%u) want(%d) got(%d)\n",
			bts.band, arfcn, direct_map[i][1], res);
		OSMO_ASSERT(res == direct_map[i][1]);
	}

	/* Check if auto-band does not break things */
	printf("Checking the mapping with auto-band.\n");
	for (i = 0; i < ARRAY_SIZE(direct_map); ++i) {
		uint16_t arfcn;
		int res;

		btsb.auto_band = 1;
		bts.band = direct_map[i][0];
		arfcn = direct_map[i][2];
		res = sysmobts_select_femto_band(&trx, arfcn);
		printf("Auto-band band(%d) arfcn(%u) want(%d) got(%d)\n",
			bts.band, arfcn, direct_map[i][1], res);
		OSMO_ASSERT(res == direct_map[i][1]);
	}

	/* Check DCS to DCS change */
	printf("Checking DCS to DCS\n");
	for (i = 0; i < ARRAY_SIZE(dcs_to_dcs); ++i) {
		uint16_t arfcn;
		int res;

		btsb.auto_band = 1;
		bts.band = dcs_to_dcs[i][0];
		arfcn = dcs_to_dcs[i][2];
		res = sysmobts_select_femto_band(&trx, arfcn);
		printf("DCS to DCS band(%d) arfcn(%u) want(%d) got(%d)\n",
			bts.band, arfcn, dcs_to_dcs[i][1], res);
		OSMO_ASSERT(res == dcs_to_dcs[i][1]);
	}

	/* Check for a PCS to PCS change */
	printf("Checking PCS to PCS\n");
	for (i = 0; i < ARRAY_SIZE(pcs_to_pcs); ++i) {
		uint16_t arfcn;
		int res;

		btsb.auto_band = 1;
		bts.band = pcs_to_pcs[i][0];
		arfcn = pcs_to_pcs[i][2];
		res = sysmobts_select_femto_band(&trx, arfcn);
		printf("PCS to PCS band(%d) arfcn(%u) want(%d) got(%d)\n",
			bts.band, arfcn, pcs_to_pcs[i][1], res);
		OSMO_ASSERT(res == pcs_to_pcs[i][1]);
	}
}

int main(int argc, char **argv)
{
	printf("Testing sysmobts routines\n");
	test_sysmobts_auto_band();
	return 0;
}
