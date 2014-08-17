#ifndef OML_ROUTING_H
#define OML_ROUTING_H

#include <stdint.h>
#include <osmocom/gsm/abis_nm.h>

enum oml_routing_flags {
	OML_RTF_MDISC		= 0x00000001,
	OML_RTF_OBJ_CLASS	= 0x00000002,
	OML_RTF_BTS_NR		= 0x00000004,
	OML_RTF_TRX_NR		= 0x00000008,
	OML_RTF_TS_NR		= 0x00000010,
	OML_RTF_VENDOR		= 0x00000020,
};

struct oml_routing_key {
	uint8_t mdisc;				/* abis_om_hdr.mdisc */
	uint8_t obj_class;			/* abis_om_fom_hdr.obj_class */
	struct abis_om_obj_inst obj_inst;	/* abis_om_fom_hdr.obj_inst */
	uint8_t vendor_lv[64];			/* vendor length-value */
} __attribute__ ((packed));

struct oml_route {
	uint32_t flags;				/* bitmask of oml_routing_flags */
	struct oml_routing_key key;
} __attribute ((packed));

struct oml_routing_inst;
struct oml_client;

/* an OML route as it is used internally */
struct oml_route_entry {
	struct llist_head list;
	struct oml_route route;
	struct oml_client *client;
};

int oml_route_add(struct oml_routing_inst *inst, const struct oml_route *route,
		  struct oml_client *client);

int oml_route_del(struct oml_routing_inst *inst, const struct oml_route *route);
int oml_route_del_client(struct oml_routing_inst *inst, const struct oml_client *client);

void *oml_route(struct oml_routing_inst *inst,
		const struct oml_routing_key *key);

struct oml_routing_inst *oml_route_init(void *ctx, void *priv);

char *oml_route_client_name(struct oml_client *client);

#endif
