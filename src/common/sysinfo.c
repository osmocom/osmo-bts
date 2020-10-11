/* (C) 2011-2019 by Harald Welte <laforge@gnumonks.org>
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

#include <osmo-bts/logging.h>
#include <osmo-bts/gsm_data.h>
#include <osmo-bts/pcu_if.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/bts_trx.h>

/* properly increment SI2q index and return SI2q data for scheduling */
static inline uint8_t *get_si2q_inc_index(struct gsm_bts *bts)
{
	uint8_t i = bts->si2q_index;
	/* si2q_count is the max si2q_index value, not the number of messages */
	bts->si2q_index = (bts->si2q_index + 1) % (bts->si2q_count + 1);

	return (uint8_t *)GSM_BTS_SI2Q(bts, i);
}

/* Apply the rules from 05.02 6.3.1.3 Mapping of BCCH Data */
uint8_t *bts_sysinfo_get(struct gsm_bts *bts, const struct gsm_time *g_time)
{
	unsigned int tc4_cnt = 0;
	unsigned int tc4_sub[4];

	/* System information type 2 bis or 2 ter messages are sent if
	 * needed, as determined by the system operator.  If only one of
	 * them is needed, it is sent when TC = 5.  If both are needed,
	 * 2bis is sent when TC = 5 and 2ter is sent at least once
	 * within any of 4 consecutive occurrences of TC = 4.  */
	/* System information type 2 quater is sent if needed, as
	 * determined by the system operator. If sent on BCCH Norm, it
	 * shall be sent when TC = 5 if neither of 2bis and 2ter are
	 * used, otherwise it shall be sent at least once within any of
	 * 4 consecutive occurrences of TC = 4. If sent on BCCH Ext, it
	 * is sent at least once within any of 4 consecutive occurrences
	 * of TC = 5. */
	/* System Information type 9 is sent in those blocks with
	 * TC = 4 which are specified in system information type 3 as
	 * defined in 3GPP TS 04.08.  */
	/* System Information Type 13 need only be sent if GPRS support
	 * is indicated in one or more of System Information Type 3 or 4
	 * or 7 or 8 messages. These messages also indicate if the
	 * message is sent on the BCCH Norm or if the message is
	 * transmitted on the BCCH Ext. In the case that the message is
	 * sent on the BCCH Norm, it is sent at least once within any of
	 * 4 consecutive occurrences of TC = 4. */

	/* We only implement BCCH Norm at this time */
	switch (g_time->tc) {
	case 0:
		/* System Information Type 1 need only be sent if
		 * frequency hopping is in use or when the NCH is
		 * present in a cell. If the MS finds another message
		 * when TC = 0, it can assume that System Information
		 * Type 1 is not in use.  */
		if (GSM_BTS_HAS_SI(bts, SYSINFO_TYPE_1))
			return GSM_BTS_SI(bts, SYSINFO_TYPE_1);
		return GSM_BTS_SI(bts, SYSINFO_TYPE_2);
	case 1:
		/* A SI 2 message will be sent at least every time TC = 1. */
		return GSM_BTS_SI(bts, SYSINFO_TYPE_2);
	case 2:
		return GSM_BTS_SI(bts, SYSINFO_TYPE_3);
	case 3:
		return GSM_BTS_SI(bts, SYSINFO_TYPE_4);
	case 4:
		/* iterate over 2ter, 2quater, 9, 13 */
		/* determine how many SI we need to send on TC=4,
		 * and which of them we send when */
		if (GSM_BTS_HAS_SI(bts, SYSINFO_TYPE_2ter) && GSM_BTS_HAS_SI(bts, SYSINFO_TYPE_2bis)) {
			tc4_sub[tc4_cnt] = SYSINFO_TYPE_2ter;
			tc4_cnt += 1;
		}
		if (GSM_BTS_HAS_SI(bts, SYSINFO_TYPE_2quater) &&
		    (GSM_BTS_HAS_SI(bts, SYSINFO_TYPE_2bis) || GSM_BTS_HAS_SI(bts, SYSINFO_TYPE_2ter))) {
			tc4_sub[tc4_cnt] = SYSINFO_TYPE_2quater;
			tc4_cnt += 1;
		}
		if (GSM_BTS_HAS_SI(bts, SYSINFO_TYPE_13) && pcu_connected()) {
			tc4_sub[tc4_cnt] = SYSINFO_TYPE_13;
			tc4_cnt += 1;
		}
		if (GSM_BTS_HAS_SI(bts, SYSINFO_TYPE_9)) {
			/* FIXME: check SI3 scheduling info! */
			tc4_sub[tc4_cnt] = SYSINFO_TYPE_9;
			tc4_cnt += 1;
		}
		/* simply send SI2 if we have nothing else to send */
		if (tc4_cnt == 0)
			return GSM_BTS_SI(bts, SYSINFO_TYPE_2);
		else {
			/* increment static counter by one, modulo count */
			bts->si.tc4_ctr = (bts->si.tc4_ctr + 1) % tc4_cnt;

			if (tc4_sub[bts->si.tc4_ctr] == SYSINFO_TYPE_2quater)
				return get_si2q_inc_index(bts);

			return GSM_BTS_SI(bts, tc4_sub[bts->si.tc4_ctr]);
		}
	case 5:
		/* 2bis, 2ter, 2quater */
		if (GSM_BTS_HAS_SI(bts, SYSINFO_TYPE_2bis) && !GSM_BTS_HAS_SI(bts, SYSINFO_TYPE_2ter))
			return GSM_BTS_SI(bts, SYSINFO_TYPE_2bis);

		else if (GSM_BTS_HAS_SI(bts, SYSINFO_TYPE_2ter) && !GSM_BTS_HAS_SI(bts, SYSINFO_TYPE_2bis))
			return GSM_BTS_SI(bts, SYSINFO_TYPE_2ter);

		else if (GSM_BTS_HAS_SI(bts, SYSINFO_TYPE_2bis) && GSM_BTS_HAS_SI(bts, SYSINFO_TYPE_2ter))
			return GSM_BTS_SI(bts, SYSINFO_TYPE_2bis);

		else if (GSM_BTS_HAS_SI(bts, SYSINFO_TYPE_2quater) &&
			 !GSM_BTS_HAS_SI(bts, SYSINFO_TYPE_2bis) && !GSM_BTS_HAS_SI(bts, SYSINFO_TYPE_2ter))
			return get_si2q_inc_index(bts);

			/* simply send SI2 if we have nothing else to send */
		else
			return GSM_BTS_SI(bts, SYSINFO_TYPE_2);
		break;
	case 6:
		return GSM_BTS_SI(bts, SYSINFO_TYPE_3);
	case 7:
		return GSM_BTS_SI(bts, SYSINFO_TYPE_4);
	}

	/* this should never bve reached. We must transmit a BCCH
	 * message on the normal BCCH in all cases. */
	OSMO_ASSERT(0);
	return 0;
}

uint8_t num_agch(struct gsm_bts_trx *trx, const char * arg)
{
	struct gsm_bts *b = trx->bts;
	struct gsm48_system_information_type_3 *si3;
	if (GSM_BTS_HAS_SI(b, SYSINFO_TYPE_3)) {
		si3 = GSM_BTS_SI(b, SYSINFO_TYPE_3);
		return si3->control_channel_desc.bs_ag_blks_res;
	}
	LOGP(DL1P, LOGL_NOTICE, "%s: Unable to determine actual BS_AG_BLKS_RES "
	     "value as SI3 is not available yet, fallback to 1\n", arg);
	return 1;
}

/* obtain the next to-be transmitted dowlink SACCH frame (L2 hdr + L3); returns pointer to lchan->si buffer */
uint8_t *lchan_sacch_get(struct gsm_lchan *lchan)
{
	uint32_t tmp, i;

	for (i = 0; i < _MAX_SYSINFO_TYPE; i++) {
		tmp = (lchan->si.last + 1 + i) % _MAX_SYSINFO_TYPE;
		if (!(lchan->si.valid & (1 << tmp)))
			continue;
		lchan->si.last = tmp;
		return GSM_LCHAN_SI(lchan, tmp);
	}
	LOGPLCHAN(lchan, DL1P, LOGL_NOTICE, "SACCH no SI available\n");
	return NULL;
}

/* re-generate SI3 restoctets with GPRS indicator depending on the PCU socket connection state */
void regenerate_si3_restoctets(struct gsm_bts *bts)
{
	uint8_t *si3_buf = GSM_BTS_SI(bts, SYSINFO_TYPE_3);
	size_t si3_size = offsetof(struct gsm48_system_information_type_3, rest_octets);
	struct osmo_gsm48_si_ro_info si3ro_tmp;

	/* If BSC has never set SI3, there's nothing to patch */
	if (!GSM_BTS_HAS_SI(bts, SYSINFO_TYPE_3))
		return;

	/* If SI3 from BSC doesn't have a GPRS indicator, we won't have anything to patch */
	if (!bts->si3_ro_decoded.gprs_ind.present)
		return;

	/* Create a temporary copy and patch that, if no PCU is around */
	si3ro_tmp = bts->si3_ro_decoded;
	if (!pcu_connected()) {
		if (!bts->si_gprs_ind_disabled)
			LOGP(DPCU, LOGL_NOTICE, "Disabling GPRS Indicator in SI (No PCU connected)\n");
		bts->si_gprs_ind_disabled = true;
		si3ro_tmp.gprs_ind.present = 0;
	} else {
		if (bts->si_gprs_ind_disabled)
			LOGP(DPCU, LOGL_NOTICE, "Enabling GPRS Indicator in SI (PCU connected)\n");
		bts->si_gprs_ind_disabled = false;
		si3ro_tmp.gprs_ind.present = 1; /* is a no-op as we copy from bts->si3_ro_decoded */
	}

	/* re-generate the binary SI3 rest octets */
	osmo_gsm48_rest_octets_si3_encode(si3_buf + si3_size, &si3ro_tmp);
}

/* re-generate SI4 restoctets with GPRS indicator depending on the PCU socket connection state */
void regenerate_si4_restoctets(struct gsm_bts *bts)
{
	uint8_t *si4_buf = GSM_BTS_SI(bts, SYSINFO_TYPE_4);
	size_t si4_size = offsetof(struct gsm48_system_information_type_4, data);
	struct osmo_gsm48_si_ro_info si4ro_tmp;

	/* If BSC has never set SI4, there's nothing to patch */
	if (!GSM_BTS_HAS_SI(bts, SYSINFO_TYPE_4))
		return;

	/* If SI4 from BSC doesn't have a GPRS indicator, we won't have anything to patch */
	if (!bts->si4_ro_decoded.gprs_ind.present)
		return;

	/* Create a temporary copy and patch that, if no PCU is around */
	si4ro_tmp = bts->si4_ro_decoded;
	if (!pcu_connected()) {
		if (!bts->si_gprs_ind_disabled)
			LOGP(DPCU, LOGL_NOTICE, "Disabling GPRS Indicator in SI (No PCU connected)\n");
		bts->si_gprs_ind_disabled = true;
		si4ro_tmp.gprs_ind.present = 0;
	} else {
		if (bts->si_gprs_ind_disabled)
			LOGP(DPCU, LOGL_NOTICE, "Enabling GPRS Indicator in SI (PCU connected)\n");
		bts->si_gprs_ind_disabled = false;
		si4ro_tmp.gprs_ind.present = 1; /* is a no-op as we copy from bts->si4_ro_decoded */
	}

	/* re-generate the binary SI4 rest octets */
	osmo_gsm48_rest_octets_si4_encode(si4_buf + si4_size, &si4ro_tmp, GSM_MACBLOCK_LEN - si4_size);
}
