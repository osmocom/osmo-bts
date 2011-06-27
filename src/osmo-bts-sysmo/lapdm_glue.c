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

#include <osmocom/gsm/prim.h>

#include <osmocom/bb/common/l1ctl.h>
#include <osmocom/bb/common/lapdm.h>

/* LAPDm wants to send a PH-* primitive to the physical layer (L1) */
int sysmol1_ph_prim_cb(struct osmo_prim_hdr *oph, void *ctx)
{
	struct osmocom_ms *ms = ctx;
	struct osmo_phsap_prim *pp = (struct osmo_phsap_prim *) oph;
	int rc = 0;

	if (oph->sap != SAP_GSM_PH)
		return -ENODEV;

	if (oph->operation != PRIM_OP_REQUEST)
		return -EINVAL;

	switch (oph->primitive) {
	case PRIM_PH_RACH:
		/* A BTS never transmits RACH */
	case PRIM_PH_DATA:
		/* we use the LAPDm code in polling only, we should never
		 * get a PH-DATA.req */
	default:
		LOGP(DLAPDM, LOGL_ERROR, "LAPDm sends unknown prim %u\n",
			oph->primitive);
		rc = -EINVAL;
		break;
	}

	return rc;
}
