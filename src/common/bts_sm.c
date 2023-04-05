/* BTS support code common to all supported BTS models */

/* (C) 2023 by sysmocom - s.m.f.c. GmbH <info@sysmocom.de>
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

#include <osmocom/core/talloc.h>
#include <osmocom/core/linuxlist.h>
#include <osmocom/core/fsm.h>

#include <osmo-bts/bts_sm.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/nm_common_fsm.h>

struct gsm_bts_sm *g_bts_sm;

static int gsm_bts_sm_talloc_destructor(struct gsm_bts_sm *bts_sm)
{
	struct gsm_bts *bts;

	while ((bts = llist_first_entry_or_null(&bts_sm->bts_list, struct gsm_bts, list)))
		talloc_free(bts);

	if (bts_sm->mo.fi) {
		osmo_fsm_inst_free(bts_sm->mo.fi);
		bts_sm->mo.fi = NULL;
	}

	return 0;
}

struct gsm_bts_sm *gsm_bts_sm_alloc(void *talloc_ctx)
{
	struct gsm_bts_sm *bts_sm = talloc_zero(talloc_ctx, struct gsm_bts_sm);

	if (!bts_sm)
		return NULL;

	talloc_set_destructor(bts_sm, gsm_bts_sm_talloc_destructor);

	INIT_LLIST_HEAD(&bts_sm->bts_list);

	/* NM SITE_MGR */
	bts_sm->mo.fi = osmo_fsm_inst_alloc(&nm_bts_sm_fsm, bts_sm, bts_sm,
					    LOGL_INFO, "bts_sm");
	gsm_mo_init(&bts_sm->mo, NULL, NM_OC_SITE_MANAGER,
		    0xff, 0xff, 0xff);

	oml_mo_state_init(&bts_sm->mo, NM_OPSTATE_DISABLED, NM_AVSTATE_NOT_INSTALLED);

	return bts_sm;
}
