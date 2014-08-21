#ifndef _SYSMOBTS_MGR_H
#define _SYSMOBTS_MGR_H

#include <osmocom/vty/vty.h>
#include <osmocom/vty/command.h>

enum {
	DTEMP,
	DFW,
	DFIND,
};

enum mgr_vty_node {
	MGR_NODE = _LAST_OSMOVTY_NODE + 1,
};

int sysmobts_mgr_vty_init(void);
int sysmobts_mgr_parse_config(const char *config_file);

struct sysmobts_mgr_instance {
	const char *config_file;
};
#endif
