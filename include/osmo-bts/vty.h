#ifndef OSMOBTS_VTY_H
#define OSMOBTS_VTY_H

#include <osmocom/vty/vty.h>
#include <osmocom/vty/command.h>

enum bts_vty_node {
	/* PHY_NODE must come before BTS node to ensure the phy
	 * instances are created at the time the TRX nodes want to refer
	 * to them */
	PHY_NODE = _LAST_OSMOVTY_NODE + 1,
	PHY_INST_NODE,
	BTS_NODE,
	TRX_NODE,
	OSMUX_NODE,
};

extern struct cmd_element cfg_bts_auto_band_cmd;
extern struct cmd_element cfg_bts_no_auto_band_cmd;

struct phy_instance *vty_get_phy_instance(struct vty *vty, int phy_nr, int inst_nr);

int bts_vty_go_parent(struct vty *vty);
int bts_vty_is_config_node(struct vty *vty, int node);

int bts_vty_init(void *ctx);

extern struct vty_app_info bts_vty_info;
extern struct gsm_bts *g_bts;

enum bts_vty_cmd_attr {
	BTS_VTY_ATTR_NEW_LCHAN,
	BTS_VTY_TRX_POWERCYCLE,
	/* NOTE: up to 32 entries */
};

#endif
