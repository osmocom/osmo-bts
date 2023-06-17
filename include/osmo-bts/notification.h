/* Maintain and generate ASCI notifications */

/*
 * (C) 2023 by sysmocom - s.f.m.c. GmbH <info@sysmocom.de>
 * All Rights Reserved
 *
 * SPDX-License-Identifier: AGPL-3.0+
 *
 * Author: Harald Welte
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
 */

#pragma once

/* one [concurrent] ASCI (VBS/VGCS) notification */
struct asci_notification {
	struct llist_head list;	/* linked to bts->asci.notifications */

	/* Group call reference (TS 24.008 10.5.1.9 "Descriptive group or broadcast call reference") */
	uint8_t group_call_ref[5];

	/* Group Channel Description (TS 44.018 10.5.2.14b) */
	struct {
		bool present;
		uint8_t value[255];
		uint8_t len;
	} chan_desc;

	/* NCH DRX Information (TS 48.058 9.3.47) */
	struct {
		bool present;
		struct rsl_ie_nch_drx_info value;
	} nch_drx_info;
};

int bts_asci_notification_add(struct gsm_bts *bts, const uint8_t *group_call_ref, const uint8_t *chan_desc,
			      uint8_t chan_desc_len, const struct rsl_ie_nch_drx_info *nch_drx_info);

int bts_asci_notification_del(struct gsm_bts *bts, const uint8_t *group_call_ref);

int bts_asci_notification_reset(struct gsm_bts *bts);

const struct asci_notification *bts_asci_notification_get_next(struct gsm_bts *bts);

void append_group_call_information(struct bitvec *bv, const uint8_t *gcr, const uint8_t *ch_desc, uint8_t ch_desc_len);

int bts_asci_notify_nch_gen_msg(struct gsm_bts *bts, uint8_t *out_buf);
int bts_asci_notify_facch_gen_msg(struct gsm_bts *bts, uint8_t *out_buf, const uint8_t *group_call_ref,
				  const uint8_t *chan_desc, uint8_t chan_desc_len);
