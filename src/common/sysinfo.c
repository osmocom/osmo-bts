/* (C) 2011 by Harald Welte <laforge@gnumonks.org>
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

#include <stdint.h>

#include <osmocom/gsm/gsm_utils.h>
#include <osmocom/gsm/sysinfo.h>

#include <osmo-bts/gsm_data.h>

uint8_t *bts_sysinfo_get(struct gsm_bts *bts, struct gsm_time *g_time)
{
	/* Apply the rules from 05.02 6.3.1.3 Mapping of BCCH Data */
	switch (g_time->tc) {
	case 0:
		return GSM_BTS_SI(bts, SYSINFO_TYPE_1);
	case 1:
		return GSM_BTS_SI(bts, SYSINFO_TYPE_2);
	case 2:
		return GSM_BTS_SI(bts, SYSINFO_TYPE_3);
	case 3:
		return GSM_BTS_SI(bts, SYSINFO_TYPE_4);
	case 4:
		/* 2ter, 2quater, 9, 13 */
		break;
	case 5:
		/* 2ter, 2quater */
		break;
	case 6:
		return GSM_BTS_SI(bts, SYSINFO_TYPE_3);
	case 7:
		return GSM_BTS_SI(bts, SYSINFO_TYPE_4);
	}

	return NULL;
}

uint8_t *lchan_sacch_get(struct gsm_lchan *lchan, struct gsm_time *g_time)
{
	uint32_t tmp;

	for (tmp = lchan->si.last + 1; tmp != lchan->si.last; tmp = (tmp + 1) % 32) {
		if (lchan->si.valid & (1 << tmp)) {
			lchan->si.last = tmp;
			return lchan->si.buf[tmp];
		}
	}
	return NULL;
}
