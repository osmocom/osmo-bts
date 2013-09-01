#ifndef OSMOBTS_VTY_H
#define OSMOBTS_VTY_H

#include <osmocom/vty/vty.h>
#include <osmocom/vty/command.h>

enum bts_vty_node {
	BTS_NODE = _LAST_OSMOVTY_NODE + 1,
	TRX_NODE,
};

extern struct cmd_element ournode_exit_cmd;
extern struct cmd_element ournode_end_cmd;

enum node_type bts_vty_go_parent(struct vty *vty);
int bts_vty_is_config_node(struct vty *vty, int node);

int bts_vty_init(struct gsm_bts *bts, const struct log_info *cat);

extern struct vty_app_info bts_vty_info;

#endif
