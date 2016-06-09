/* Temperature control for NuRAN Litecell 1.5 BTS management daemon */

/* Copyright (C) 2015 by Yves Godin <support@nuranwireless.com>
 * 
 * Based on sysmoBTS:
 *     sysmobts_mgr_temp.c
 *     (C) 2014 by Holger Hans Peter Freyther
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

#include "misc/lc15bts_mgr.h"
#include "misc/lc15bts_misc.h"
#include "misc/lc15bts_temp.h"
#include "misc/lc15bts_power.h"

#include <osmo-bts/logging.h>

#include <osmocom/core/timer.h>
#include <osmocom/core/utils.h>

static struct lc15bts_mgr_instance *s_mgr;
static struct osmo_timer_list temp_ctrl_timer;

static const struct value_string state_names[] = {
	{ STATE_NORMAL,			"NORMAL" },
	{ STATE_WARNING_HYST,		"WARNING (HYST)" },
	{ STATE_WARNING,		"WARNING" },
	{ STATE_CRITICAL,		"CRITICAL" },
	{ 0, NULL }
};

const char *lc15bts_mgr_temp_get_state(enum lc15bts_temp_state state)
{
	return get_value_string(state_names, state);
}

static int next_state(enum lc15bts_temp_state current_state, int critical, int warning)
{
	int next_state = -1;
	switch (current_state) {
	case STATE_NORMAL:
		if (critical)
			next_state = STATE_CRITICAL;
		else if (warning)
			next_state = STATE_WARNING;
		break;
	case STATE_WARNING_HYST:
		if (critical)
			next_state = STATE_CRITICAL;
		else if (warning)
			next_state = STATE_WARNING;
		else
			next_state = STATE_NORMAL;
		break;
	case STATE_WARNING:
		if (critical)
			next_state = STATE_CRITICAL;
		else if (!warning)
			next_state = STATE_WARNING_HYST;
		break;
	case STATE_CRITICAL:
		if (!critical && !warning)
			next_state = STATE_WARNING;
		break;
	};

	return next_state;
}

static void handle_normal_actions(int actions)
{
	/* switch on the PA */
	if (actions & TEMP_ACT_NORM_PA0_ON) {
		if (lc15bts_power_set(LC15BTS_POWER_PA0, 1) != 0) {
			LOGP(DTEMP, LOGL_ERROR,
				"Failed to switch on the PA #0\n");
		} else {
			LOGP(DTEMP, LOGL_NOTICE,
				"Switched on the PA #0 as normal action.\n");
		}
	}

	if (actions & TEMP_ACT_NORM_PA1_ON) {
		if (lc15bts_power_set(LC15BTS_POWER_PA1, 1) != 0) {
			LOGP(DTEMP, LOGL_ERROR,
				"Failed to switch on the PA #1\n");
		} else {
			LOGP(DTEMP, LOGL_NOTICE,
				"Switched on the PA #1 as normal action.\n");
		}
	}

	if (actions & TEMP_ACT_NORM_BTS_SRV_ON) {
		LOGP(DTEMP, LOGL_NOTICE,
		"Going to switch on the BTS service\n");
		/*
		 * TODO: use/create something like nspawn that serializes
		 * and used SIGCHLD/waitpid to pick up the dead processes
		 * without invoking shell.
		 */
		system("/bin/systemctl start lc15bts.service");
	}
}

static void handle_actions(int actions)
{
	/* switch off the PA */
	if (actions & TEMP_ACT_PA1_OFF) {
		if (lc15bts_power_set(LC15BTS_POWER_PA1, 0) != 0) {
			LOGP(DTEMP, LOGL_ERROR,
				"Failed to switch off the PA #1. Stop BTS?\n");
		} else {
			LOGP(DTEMP, LOGL_NOTICE,
				"Switched off the PA #1 due temperature.\n");
		}
	}

	if (actions & TEMP_ACT_PA0_OFF) {
		if (lc15bts_power_set(LC15BTS_POWER_PA0, 0) != 0) {
			LOGP(DTEMP, LOGL_ERROR,
				"Failed to switch off the PA #0. Stop BTS?\n");
		} else {
			LOGP(DTEMP, LOGL_NOTICE,
				"Switched off the PA #0 due temperature.\n");
		}
	}

	if (actions & TEMP_ACT_BTS_SRV_OFF) {
		LOGP(DTEMP, LOGL_NOTICE,
			"Going to switch off the BTS service\n");
		/*
		 * TODO: use/create something like nspawn that serializes
		 * and used SIGCHLD/waitpid to pick up the dead processes
		 * without invoking shell.
		 */
		system("/bin/systemctl stop lc15bts.service");
	}
}

/**
 * Go back to normal! Depending on the configuration execute the normal
 * actions that could (start to) undo everything we did in the other
 * states. What is still missing is the power increase/decrease depending
 * on the state. E.g. starting from WARNING_HYST we might want to slowly
 * ramp up the output power again.
 */
static void execute_normal_act(struct lc15bts_mgr_instance *manager)
{
	LOGP(DTEMP, LOGL_NOTICE, "System is back to normal temperature.\n");
	handle_normal_actions(manager->temp.action_norm);
}

static void execute_warning_act(struct lc15bts_mgr_instance *manager)
{
	LOGP(DTEMP, LOGL_NOTICE, "System has reached temperature warning.\n");
	handle_actions(manager->temp.action_warn);
}

static void execute_critical_act(struct lc15bts_mgr_instance *manager)
{
	LOGP(DTEMP, LOGL_NOTICE, "System has reached critical warning.\n");
	handle_actions(manager->temp.action_crit);
}

static void lc15bts_mgr_temp_handle(struct lc15bts_mgr_instance *manager,
			int critical, int warning)
{
	int new_state = next_state(manager->temp.state, critical, warning);

	/* Nothing changed */
	if (new_state < 0)
		return;

	LOGP(DTEMP, LOGL_NOTICE, "Moving from state %s to %s.\n",
		get_value_string(state_names, manager->temp.state),
		get_value_string(state_names, new_state));
	manager->temp.state = new_state;
	switch (manager->temp.state) {
	case STATE_NORMAL:
		execute_normal_act(manager);
		break;
	case STATE_WARNING_HYST:
		/* do nothing? Maybe start to increase transmit power? */
		break;
	case STATE_WARNING:
		execute_warning_act(manager);
		break;
	case STATE_CRITICAL:
		execute_critical_act(manager);
		break;
	};
} 

static void temp_ctrl_check()
{
	int rc;
	int warn_thresh_passed = 0;
	int crit_thresh_passed = 0;

	LOGP(DTEMP, LOGL_DEBUG, "Going to check the temperature.\n");

	/* Read the current supply temperature */
	rc = lc15bts_temp_get(LC15BTS_TEMP_SUPPLY);
	if (rc < 0) {
		LOGP(DTEMP, LOGL_ERROR,
			"Failed to read the supply temperature. rc=%d\n", rc);
		warn_thresh_passed = crit_thresh_passed = 1;
	} else {
		int temp = rc / 1000;
		if (temp > s_mgr->temp.supply_limit.thresh_warn)
			warn_thresh_passed = 1;
		if (temp > s_mgr->temp.supply_limit.thresh_crit)
			crit_thresh_passed = 1;
		LOGP(DTEMP, LOGL_DEBUG, "Supply temperature is: %d\n", temp);
	}

	/* Read the current SoC temperature */
	rc = lc15bts_temp_get(LC15BTS_TEMP_SOC);
	if (rc < 0) {
		LOGP(DTEMP, LOGL_ERROR,
			"Failed to read the SoC temperature. rc=%d\n", rc);
		warn_thresh_passed = crit_thresh_passed = 1;
	} else {
		int temp = rc / 1000;
		if (temp > s_mgr->temp.soc_limit.thresh_warn)
			warn_thresh_passed = 1;
		if (temp > s_mgr->temp.soc_limit.thresh_crit)
			crit_thresh_passed = 1;
		LOGP(DTEMP, LOGL_DEBUG, "SoC temperature is: %d\n", temp);
	}

	/* Read the current fpga temperature */
	rc = lc15bts_temp_get(LC15BTS_TEMP_FPGA);
	if (rc < 0) {
		LOGP(DTEMP, LOGL_ERROR,
			"Failed to read the fpga temperature. rc=%d\n", rc);
		warn_thresh_passed = crit_thresh_passed = 1;
	} else {
		int temp = rc / 1000;
		if (temp > s_mgr->temp.fpga_limit.thresh_warn)
			warn_thresh_passed = 1;
		if (temp > s_mgr->temp.fpga_limit.thresh_crit)
			crit_thresh_passed = 1;
		LOGP(DTEMP, LOGL_DEBUG, "FPGA temperature is: %d\n", temp);
	}

	/* Read the current RF log detector temperature */
	rc = lc15bts_temp_get(LC15BTS_TEMP_LOGRF);
	if (rc < 0) {
		LOGP(DTEMP, LOGL_ERROR,
			"Failed to read the RF log detector temperature. rc=%d\n", rc);
		warn_thresh_passed = crit_thresh_passed = 1;
	} else {
		int temp = rc / 1000;
		if (temp > s_mgr->temp.logrf_limit.thresh_warn)
			warn_thresh_passed = 1;
		if (temp > s_mgr->temp.logrf_limit.thresh_crit)
			crit_thresh_passed = 1;
		LOGP(DTEMP, LOGL_DEBUG, "RF log detector temperature is: %d\n", temp);
	}

	/* Read the current OCXO temperature */
	rc = lc15bts_temp_get(LC15BTS_TEMP_OCXO);
	if (rc < 0) {
		LOGP(DTEMP, LOGL_ERROR,
			"Failed to read the OCXO temperature. rc=%d\n", rc);
		warn_thresh_passed = crit_thresh_passed = 1;
	} else {
		int temp = rc / 1000;
		if (temp > s_mgr->temp.ocxo_limit.thresh_warn)
			warn_thresh_passed = 1;
		if (temp > s_mgr->temp.ocxo_limit.thresh_crit)
			crit_thresh_passed = 1;
		LOGP(DTEMP, LOGL_DEBUG, "OCXO temperature is: %d\n", temp);
	}

	/* Read the current TX #1 temperature */
	rc = lc15bts_temp_get(LC15BTS_TEMP_TX0);
	if (rc < 0) {
		LOGP(DTEMP, LOGL_ERROR,
			"Failed to read the TX #0 temperature. rc=%d\n", rc);
		warn_thresh_passed = crit_thresh_passed = 1;
	} else {
		int temp = rc / 1000;
		if (temp > s_mgr->temp.tx0_limit.thresh_warn)
			warn_thresh_passed = 1;
		if (temp > s_mgr->temp.tx0_limit.thresh_crit)
			crit_thresh_passed = 1;
		LOGP(DTEMP, LOGL_DEBUG, "TX #0 temperature is: %d\n", temp);
	}

	/* Read the current TX #2 temperature */
	rc = lc15bts_temp_get(LC15BTS_TEMP_TX1);
	if (rc < 0) {
		LOGP(DTEMP, LOGL_ERROR,
			"Failed to read the TX #1 temperature. rc=%d\n", rc);
		warn_thresh_passed = crit_thresh_passed = 1;
	} else {
		int temp = rc / 1000;
		if (temp > s_mgr->temp.tx1_limit.thresh_warn)
			warn_thresh_passed = 1;
		if (temp > s_mgr->temp.tx1_limit.thresh_crit)
			crit_thresh_passed = 1;
		LOGP(DTEMP, LOGL_DEBUG, "TX #1 temperature is: %d\n", temp);
	}

	/* Read the current PA #1 temperature */
	rc = lc15bts_temp_get(LC15BTS_TEMP_PA0);
	if (rc < 0) {
		LOGP(DTEMP, LOGL_ERROR,
			"Failed to read the PA #0 temperature. rc=%d\n", rc);
		warn_thresh_passed = crit_thresh_passed = 1;
	} else {
		int temp = rc / 1000;
		if (temp > s_mgr->temp.pa0_limit.thresh_warn)
			warn_thresh_passed = 1;
		if (temp > s_mgr->temp.pa0_limit.thresh_crit)
			crit_thresh_passed = 1;
		LOGP(DTEMP, LOGL_DEBUG, "PA #0 temperature is: %d\n", temp);
	}

	/* Read the current PA #2 temperature */
	rc = lc15bts_temp_get(LC15BTS_TEMP_PA1);
	if (rc < 0) {
		LOGP(DTEMP, LOGL_ERROR,
			"Failed to read the PA #1 temperature. rc=%d\n", rc);
		warn_thresh_passed = crit_thresh_passed = 1;
	} else {
		int temp = rc / 1000;
		if (temp > s_mgr->temp.pa1_limit.thresh_warn)
			warn_thresh_passed = 1;
		if (temp > s_mgr->temp.pa1_limit.thresh_crit)
			crit_thresh_passed = 1;
		LOGP(DTEMP, LOGL_DEBUG, "PA #1 temperature is: %d\n", temp);
	}

	lc15bts_mgr_temp_handle(s_mgr, crit_thresh_passed, warn_thresh_passed);	
}

static void temp_ctrl_check_cb(void *unused)
{
	temp_ctrl_check();
	/* Check every two minutes? XXX make it configurable! */
	osmo_timer_schedule(&temp_ctrl_timer, 2 * 60, 0);
}

int lc15bts_mgr_temp_init(struct lc15bts_mgr_instance *mgr)
{
	s_mgr = mgr;
	temp_ctrl_timer.cb = temp_ctrl_check_cb;
	temp_ctrl_check_cb(NULL);
	return 0;
}
