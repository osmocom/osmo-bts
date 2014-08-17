/* OML Message routing for osmo-bts */

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

#include <stdint.h>
#include <unistd.h>
#include <errno.h>

#include <osmocom/core/talloc.h>
#include <osmocom/core/utils.h>

#include <osmocom/gsm/abis_nm.h>
#include <osmocom/gsm/protocol/gsm_12_21.h>

#include <osmocom/vty/vty.h>
#include <osmocom/vty/command.h>

#include <osmo-bts/logging.h>
#include <osmo-bts/oml_routing.h>


/* an OML router instance */
struct oml_routing_inst {
	struct llist_head routes;
	void *priv;
};

/* FIXME: This must go! */
static struct oml_routing_inst *g_inst;

/* match given routing key against given route */
static int oml_route_match(const struct oml_routing_key *key,
			   const struct oml_route *route)
{
	if (route->flags & OML_RTF_MDISC)
		if (route->key.mdisc != key->mdisc)
			return 0;

	if (route->flags & OML_RTF_OBJ_CLASS) {
		if (route->key.obj_class != key->obj_class)
			return 0;
		if (route->key.obj_class == ABIS_OM_MDISC_MANUF &&
		    route->flags & OML_RTF_VENDOR) {
			if (route->key.vendor_lv[0] != key->vendor_lv[0])
				return 0;
			if (memcmp(route->key.vendor_lv+1, key->vendor_lv+1,
				   OSMO_MIN(sizeof(route->key.vendor_lv)-1,
					    route->key.vendor_lv[0])))
				return 0;
		}
	}

	if (route->flags & OML_RTF_BTS_NR)
		if (route->key.obj_inst.bts_nr != key->obj_inst.bts_nr)
			return 0;

	if (route->flags & OML_RTF_TRX_NR)
		if (route->key.obj_inst.trx_nr != key->obj_inst.trx_nr)
			return 0;

	if (route->flags & OML_RTF_TS_NR)
		if (route->key.obj_inst.ts_nr != key->obj_inst.ts_nr)
			return 0;

	return 1;
}

/* are two given routes identical? */
static int oml_route_ident(const struct oml_route *a, const struct oml_route *b)
{
	if (a->flags != b->flags)
		return 0;

	return oml_route_match(&a->key, b);
}

/* add a route from a router instance */
int oml_route_add(struct oml_routing_inst *inst, const struct oml_route *route,
		  struct oml_client *client)
{
	struct oml_route_entry *e;

	llist_for_each_entry(e, &inst->routes, list) {
		if (oml_route_ident(route, &e->route))
			return -EEXIST;
	}

	e = talloc_zero(inst, struct oml_route_entry);
	memcpy(&e->route, route, sizeof(e->route));
	e->client = client;
	/* FIXME: insert in order of integer of routing_flags */
	llist_add(&e->list, &inst->routes);

	return 0;
}

/* delete a route from a router instance */
int oml_route_del(struct oml_routing_inst *inst, const struct oml_route *route)
{
	struct oml_route_entry *e;

	/* no safe iteration needed as we stop at first match */
	llist_for_each_entry(e, &inst->routes, list) {
		if (oml_route_ident(route, &e->route)) {
			llist_del(&e->list);
			talloc_free(e);
			return 1;
		}
	}

	return -ENODEV;
}

int oml_route_del_client(struct oml_routing_inst *inst,
			 const struct oml_client *client)
{
	struct oml_route_entry *e, *f;
	int num = 0;

	/* no safe iteration needed as we stop at first match */
	llist_for_each_entry_safe(e, f, &inst->routes, list) {
		if (e->client == client) {
			llist_del(&e->list);
			talloc_free(e);
			num++;
		}
	}
	return num;
}


/* perform routing of 'key' against router instance */
void *oml_route(struct oml_routing_inst *inst,
		const struct oml_routing_key *key)
{
	struct oml_route_entry *e;

	llist_for_each_entry(e, &inst->routes, list) {
		if (oml_route_match(key, &e->route))
			return e->client;
	}

	return NULL;
}

DEFUN(show_oml_routing, show_oml_routing_cmd,
	"show oml-routes",
	SHOW_STR "Show the currently configured OML routing")
{
	struct oml_route_entry *e;

	llist_for_each_entry(e, &g_inst->routes, list) {
		struct oml_route *rt = &e->route;

		vty_out(vty, " %s OC=%s INST=(%02x/%02x/%02x) -> %s/%s%s",
			rt->flags & OML_RTF_MDISC ?
				get_value_string(abis_nm_msg_disc_names,
						 rt->key.mdisc) : "ANY",
			rt->flags & OML_RTF_OBJ_CLASS ?
				get_value_string(abis_nm_obj_class_names,
						 rt->key.obj_class) : "ANY",
			rt->flags & OML_RTF_BTS_NR ? rt->key.obj_inst.bts_nr : 255,
			rt->flags & OML_RTF_TRX_NR ? rt->key.obj_inst.trx_nr : 255,
			rt->flags & OML_RTF_TS_NR ? rt->key.obj_inst.ts_nr : 255,
			oml_route_client_addr(e->client),
			oml_route_client_name(e->client), VTY_NEWLINE);
	}
	return CMD_SUCCESS;
}

#define MDISC_STR "(any|fom|mmi|trau|manuf|<0-255>)"
#define CLASS_STR "(any|site-manager|bts|radio-carrier|baseband-transceiver|channel|gprs-nse|gprs-cell|gprs-nsvc|<0-255>)"

#define OMLROUTE_CMD "oml-route (add|del) mdisc "MDISC_STR" class "CLASS_STR" bts (any|<0-255>) trx (any|<0-255>]) ts (any|<0-255>) DEST"

#define OMLROUTE_HELP	\
	"OML-Routing\n"	\
	"Add a route\n"	\
	"Delete a route\n" \
	"Message Discriminator\n" \
	"Message Discriminator: Match any message discriminator\n" \
	"Message Discriminator: Formatted OML messages\n" \
	"Message Discriminator: Man-Machine-Interface OML messages\n" \
	"Message Discriminator: Transcoder OML Message\n" \
	"Message Discriminator: Manufacturer-specific OML messages\n" \
	"Message Discriminator: Specific numeric message discrimniator\n" \
	"Object Class\n" \
	"Object Class: Match any object class\n" \
	"Object Class: Site Manager\n" \
	"Object Class: Base Transceiver Station\n" \
	"Object Class: Radio Carrier\n" \
	"Object Class: Baseband Transceiver\n" \
	"Object Class: Channel (Um Timeslot)\n" \
	"Object Class: GPRS NS Entity\n" \
	"Object Class: GPRS Cell\n" \
	"Object Class: GPRS NS Virtual Circuit\n" \
	"Object Class: Specific numeric object class\n" \
	"Object Instance(BTS)\n" \
	"Object Instance(BTS): Any BTS number\n" \
	"Object Instance(BTS): Specific BTS number\n" \
	"Object Instance(TRX)\n" \
	"Object Instance(TRX): Any TRX number\n" \
	"Object Instance(TRX): Specific TRX number\n" \
	"Object Instance(TS)\n" \
	"Object Instance(TS): Any TS number\n" \
	"Object Instance(TS): Specific TS number\n" \
	"Name of destination for this route\n"

DEFUN(cfg_omlr_route, cfg_omlr_route_cmd,
	OMLROUTE_CMD, OMLROUTE_HELP)
{
	struct oml_route _r, *r = &_r;
	int rc;

	memset(r, 0, sizeof(*r));

	if (strcmp(argv[1], "any")) {
		r->flags |= OML_RTF_MDISC;
		rc = get_string_value(abis_nm_msg_disc_names, argv[1]);
		if (rc < 0)
			rc = atoi(argv[1]);
		r->key.mdisc = rc;
	}

	if (strcmp(argv[2], "any")) {
		r->flags |= OML_RTF_OBJ_CLASS;
		rc = get_string_value(abis_nm_obj_class_names, argv[2]);
		if (rc < 0)
			rc = atoi(argv[2]);
		r->key.obj_class = rc;
	}

	if (strcmp(argv[3], "any")) {
		r->flags |= OML_RTF_BTS_NR;
		r->key.obj_inst.bts_nr = atoi(argv[3]);
	};

	if (strcmp(argv[4], "any")) {
		r->flags |= OML_RTF_TRX_NR;
		r->key.obj_inst.trx_nr = atoi(argv[4]);
	};

	if (strcmp(argv[5], "any")) {
		r->flags |= OML_RTF_TS_NR;
		r->key.obj_inst.ts_nr = atoi(argv[5]);
	};

	if (!strcmp(argv[0], "add"))
		rc = oml_route_add(g_inst, r, NULL);
	else
		rc = oml_route_del(g_inst, r);

	if (rc < 0) {
		vty_out(vty, "Error %sing route: %d%s",
			!strcmp(argv[0], "add") ? "add" : "delet",
			rc, VTY_NEWLINE);
		return CMD_WARNING;
	}

	return CMD_SUCCESS;
}

struct oml_routing_inst *oml_route_init(void *ctx, void *priv)
{
	g_inst = talloc_zero(ctx, struct oml_routing_inst);

	INIT_LLIST_HEAD(&g_inst->routes);

	g_inst->priv = priv;

	/* FIXME: add separate node, as this is the only way to make settings
	 * persistent via the node-specific save callback */
	install_element_ve(&show_oml_routing_cmd);
	install_element(ENABLE_NODE, &cfg_omlr_route_cmd);

	return g_inst;
}
