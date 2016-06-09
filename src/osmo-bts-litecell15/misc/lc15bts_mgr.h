#ifndef _LC15BTS_MGR_H
#define _LC15BTS_MGR_H

#include <osmocom/vty/vty.h>
#include <osmocom/vty/command.h>

#include <osmocom/core/select.h>
#include <osmocom/core/timer.h>

#include <stdint.h>

enum {
	DTEMP,
	DFW,
	DFIND,
	DCALIB,
};

// TODO NTQD: Define new actions like reducing output power, limit ARM core speed, shutdown second TRX/PA, ... 
enum {
#if 0
	TEMP_ACT_PWR_CONTRL	=	0x1,
#endif
	TEMP_ACT_PA0_OFF	=	0x2,
	TEMP_ACT_PA1_OFF	=	0x4,
	TEMP_ACT_BTS_SRV_OFF	=	0x10,
};

/* actions only for normal state */
enum {
#if 0
	TEMP_ACT_NORM_PW_CONTRL	=	0x1,
#endif
	TEMP_ACT_NORM_PA0_ON	=	0x2,
	TEMP_ACT_NORM_PA1_ON	=	0x4,
	TEMP_ACT_NORM_BTS_SRV_ON=	0x10,
};

enum lc15bts_temp_state {
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
struct lc15bts_temp_limit {
	int thresh_warn;
	int thresh_crit;
};

enum mgr_vty_node {
	MGR_NODE = _LAST_OSMOVTY_NODE + 1,

	ACT_NORM_NODE,
	ACT_WARN_NODE,
	ACT_CRIT_NODE,
	LIMIT_SUPPLY_NODE,
	LIMIT_SOC_NODE,
	LIMIT_FPGA_NODE,
	LIMIT_LOGRF_NODE,
	LIMIT_OCXO_NODE,
	LIMIT_TX0_NODE,
	LIMIT_TX1_NODE,
	LIMIT_PA0_NODE,
	LIMIT_PA1_NODE,
};

struct lc15bts_mgr_instance {
	const char *config_file;

	struct {
		int action_norm;
		int action_warn;
		int action_crit;

		enum lc15bts_temp_state state;

		struct lc15bts_temp_limit supply_limit;
		struct lc15bts_temp_limit soc_limit;
		struct lc15bts_temp_limit fpga_limit;
		struct lc15bts_temp_limit logrf_limit;
		struct lc15bts_temp_limit ocxo_limit;
		struct lc15bts_temp_limit tx0_limit;
		struct lc15bts_temp_limit tx1_limit;
		struct lc15bts_temp_limit pa0_limit;
		struct lc15bts_temp_limit pa1_limit;
	} temp;

	struct {
		int state;
		int calib_from_loop;
		struct osmo_timer_list calib_timeout;
	} calib;
};

int lc15bts_mgr_vty_init(void);
int lc15bts_mgr_parse_config(struct lc15bts_mgr_instance *mgr);
int lc15bts_mgr_nl_init(void);
int lc15bts_mgr_temp_init(struct lc15bts_mgr_instance *mgr);
const char *lc15bts_mgr_temp_get_state(enum lc15bts_temp_state state);


int lc15bts_mgr_calib_init(struct lc15bts_mgr_instance *mgr);
int lc15bts_mgr_calib_run(struct lc15bts_mgr_instance *mgr);

extern void *tall_mgr_ctx;

#endif
