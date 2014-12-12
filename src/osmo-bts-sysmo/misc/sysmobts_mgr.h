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
	TEMP_ACT_PWR_CONTRL	=	0x1,
	TEMP_ACT_MASTER_OFF	=	0x2,
	TEMP_ACT_SLAVE_OFF	=	0x4,
	TEMP_ACT_PA_OFF		=	0x8,
	TEMP_ACT_BTS_SRV_OFF	=	0x10,
};

enum sysmobts_temp_state {
	STATE_NORMAL,		/* Everything is fine */
	STATE_WARNING_HYST,	/* Go back to normal next? */
	STATE_WARNING,		/* We are above the warning threshold */
	STATE_CRITICAL,		/* We have an issue. Wait for below warning */
};

/**
 * Temperature Limits. We separate from a threshold
 * that will generate a warning and one that is so
 * severe that an action will be taken.
 */
struct sysmobts_temp_limit {
	int thresh_warn;
	int thresh_crit;
};

enum mgr_vty_node {
	MGR_NODE = _LAST_OSMOVTY_NODE + 1,

	ACT_WARN_NODE,
	ACT_CRIT_NODE,
	LIMIT_RF_NODE,
	LIMIT_DIGITAL_NODE,
	LIMIT_BOARD_NODE,
	LIMIT_PA_NODE,
};

struct sysmobts_mgr_instance {
	const char *config_file;

	struct sysmobts_temp_limit rf_limit;
	struct sysmobts_temp_limit digital_limit;

	/* Only available on sysmobts 2050 */
	struct sysmobts_temp_limit board_limit;
	struct sysmobts_temp_limit pa_limit;

	int action_warn;
	int action_crit;

	enum sysmobts_temp_state state;
};

int sysmobts_mgr_vty_init(void);
int sysmobts_mgr_parse_config(struct sysmobts_mgr_instance *mgr);
int sysmobts_mgr_nl_init(void);
int sysmobts_mgr_temp_init(struct sysmobts_mgr_instance *mgr);
const char *sysmobts_mgr_temp_get_state(enum sysmobts_temp_state state);

#endif
