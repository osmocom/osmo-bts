#ifndef _SYSMOBTS_MGR_H
#define _SYSMOBTS_MGR_H

#include <osmocom/vty/vty.h>
#include <osmocom/vty/command.h>

enum {
	DTEMP,
	DFW,
	DFIND,
};

enum {
	SYSMO_MGR_DISCONNECTED = 0,
	SYSMO_MGR_CONNECTED,
};

enum {
	SBTS2050_DISABLE_CHANGE_POWER = 0,
	SBTS2050_ENABLE_CHANGE_POWER,
};

#define SOCKET_PATH		"/var/run/bts_oml"

struct sbts2050_config_info;

enum mgr_vty_node {
	MGR_NODE = _LAST_OSMOVTY_NODE + 1,
};

enum node_type mgr_vty_go_parent(struct vty *vty);
int mgr_vty_is_config_node(struct vty *vty, int node);
int sysmobts_mgr_vty_init(void);
int sysmobts_mgr_parse_config(const char *config_file,
			      struct sbts2050_config_info *cfg);

struct sysmobts_mgr_instance {
	const char *config_file;
};
#endif
