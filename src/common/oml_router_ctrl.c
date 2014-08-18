/* OML Router Control (client side) */

/* (C) 2014 by Harald Welte <laforge@gnumonks.org>
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


#include <osmocom/core/msgb.h>
#include <osmocom/gsm/protocol/ipaccess.h>

#include <osmo-bts/oml_routing.h>
#include <osmo-bts/oml_router_ctrl.h>
#include <osmo-bts/gsm_data.h>

/* send a OML Router Control message via Abis */
int abis_orc_sendmsg(struct msgb *msg)
{
	struct gsm_bts *bts = msg->trx->bts;
	struct ipaccess_head_ext *he;

	/* push the extended IPA header to the front of the msgb */
	he = (struct ipaccess_head_ext *) msgb_push(msg, sizeof(*he));
	he->proto = IPAC_PROTO_EXT_ORC;

	/* osmo-bts uses msg->trx internally, but libosmo-abis uses the
	 * signalling like at msg->dst */
	msg->dst = bts->osmo_link;
	return abis_sendmsg(msg);
}

static struct msgb *orc_msgb_alloc(void)
{
	return msgb_alloc_headroom(1000+40, 40, "OML Router Ctrl");
}

static struct msgb *__gen_orc_route(const struct oml_route *rt_in, int del)
{
	struct msgb *msg = orc_msgb_alloc();
	struct osmo_omlrctrl_hdr *oh;
	struct oml_route *rt;

	oh = (struct osmo_omlrctrl_hdr *) msgb_put(msg, sizeof(*oh));
	rt = (struct oml_route *) msgb_put(msg, sizeof(*rt));

	oh->version = 0;
	if (del)
		oh->msg_type = OSMO_ORC_MSGT_ROUTE_DEL_REQ;
	else
		oh->msg_type = OSMO_ORC_MSGT_ROUTE_ADD_REQ;
	oh->data_len = sizeof(*rt);

	memcpy(rt, rt_in, sizeof (*rt));

	return msg;
}

/* generate msgb with OML Router Control ROUTE_ADD_REQ */
struct msgb *gen_orc_route_add(const struct oml_route *rt_in)
{
	return __gen_orc_route(rt_in, 0);
}

/* generate msgb with OML Router Control ROUTE_DEL_REQ */
struct msgb *gen_orc_route_del(const struct oml_route *rt_in)
{
	return __gen_orc_route(rt_in, 1);
}

/* Request a route for the given MO from the OML router */
int orc_add_route_mo(struct gsm_abis_mo *mo)
{
	struct oml_route rt;
	struct msgb *msg;

	rt.flags = OML_RTF_MDISC | OML_RTF_OBJ_CLASS;
	rt.key.mdisc = ABIS_OM_MDISC_FOM;
	rt.key.obj_class = mo->obj_class;

	memcpy(&rt.key.obj_inst, &mo->obj_inst, sizeof(rt.key.obj_inst));

	/* do w really need this?  why not simply register for 0xFF ... */
	switch (mo->obj_class) {
	case NM_OC_SITE_MANAGER:
		break;
	case NM_OC_BTS:
	case NM_OC_GPRS_NSE:
	case NM_OC_GPRS_CELL:
		rt.flags |= OML_RTF_BTS_NR;
		break;
	case NM_OC_RADIO_CARRIER:
	case NM_OC_BASEB_TRANSC:
	case NM_OC_GPRS_NSVC:
		rt.flags |= OML_RTF_BTS_NR | OML_RTF_TRX_NR;
		break;
	case NM_OC_CHANNEL:
		rt.flags |= OML_RTF_BTS_NR | OML_RTF_TRX_NR | OML_RTF_TS_NR;
		break;
	}

	msg = gen_orc_route_add(&rt);
	if (!msg)
		return -1;

	msg->trx = mo->bts->c0;
	return abis_orc_sendmsg(msg);
}
