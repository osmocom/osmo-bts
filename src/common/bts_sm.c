/* BTS support code common to all supported BTS models */

/* (C) 2023 by sysmocom - s.f.m.c. GmbH <info@sysmocom.de>
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

static const uint8_t nse_timer_default[] = { 3, 3, 3, 3, 30, 3, 10 };

struct gsm_bts *gsm_gprs_nse_get_bts(const struct gsm_gprs_nse *nse)
{
	return gsm_bts_num(g_bts_sm, nse->mo.obj_inst.bts_nr);
}

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
	struct gsm_gprs_nse *nse = &bts_sm->gprs.nse;
	unsigned int i;

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

	/* NM GPRS NSE */
	nse->mo.fi = osmo_fsm_inst_alloc(&nm_gprs_nse_fsm, bts_sm, nse,
					 LOGL_INFO, "gprs_nse0");
	gsm_mo_init(&nse->mo, NULL, NM_OC_GPRS_NSE, 0, 0xff, 0xff);
	oml_mo_state_init(&nse->mo, NM_OPSTATE_DISABLED, NM_AVSTATE_NOT_INSTALLED);
	memcpy(&nse->timer, nse_timer_default, sizeof(nse->timer));

	/* NM GPRS NSVCs */
	for (i = 0; i < ARRAY_SIZE(nse->nsvc); i++) {
		struct gsm_gprs_nsvc *nsvc = &nse->nsvc[i];
		nsvc->nse = nse;
		nsvc->id = i;
		nsvc->mo.fi = osmo_fsm_inst_alloc(&nm_gprs_nsvc_fsm, bts_sm, nsvc,
						  LOGL_INFO, NULL);
		osmo_fsm_inst_update_id_f(nsvc->mo.fi, "gprs_nsvc%d-%d",
					  nse->mo.obj_inst.bts_nr, i);
		gsm_mo_init(&nsvc->mo, NULL, NM_OC_GPRS_NSVC, nse->mo.obj_inst.bts_nr, i, 0xff);
		oml_mo_state_init(&nsvc->mo, NM_OPSTATE_DISABLED, NM_AVSTATE_NOT_INSTALLED);
	}

	return bts_sm;
}
