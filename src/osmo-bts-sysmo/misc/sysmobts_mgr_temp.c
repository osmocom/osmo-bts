/* Temperature control for SysmoBTS management daemon */

/*
 * (C) 2014 by Holger Hans Peter Freyther
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

#include "misc/sysmobts_mgr.h"
#include "misc/sysmobts_misc.h"

#include <osmo-bts/logging.h>

#include <osmocom/core/timer.h>
#include <osmocom/core/utils.h>

static struct sysmobts_mgr_instance *s_mgr;
static struct osmo_timer_list temp_ctrl_timer;

static const struct value_string state_names[] = {
	{ STATE_NORMAL,			"NORMAL" },
	{ STATE_WARNING_HYST,		"WARNING (HYST)" },
	{ STATE_WARNING,		"WARNING" },
	{ STATE_CRITICAL,		"CRITICAL" },
	{ 0, NULL }
};

static int next_state(enum sysmobts_temp_state current_state, int critical, int warning)
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

/**
 * Go back to normal! Undo everything we did in the other states. For
 * reducint the transmit power, the question is if we should slowly set
 * it back to normal, let the BTS slowly increase it.. or handle it here
 * as well?
 */
static void execute_normal_act(struct sysmobts_mgr_instance *manager)
{
	LOGP(DTEMP, LOGL_NOTICE, "System is back to normal temperature.\n");
}

static void execute_warning_act(struct sysmobts_mgr_instance *manager)
{
	LOGP(DTEMP, LOGL_NOTICE, "System has reached temperature warning.\n");
}

static void execute_critical_act(struct sysmobts_mgr_instance *manager)
{
	LOGP(DTEMP, LOGL_NOTICE, "System has reached critical warning.\n");
}

static void sysmobts_mgr_temp_handle(struct sysmobts_mgr_instance *manager,
			int critical, int warning)
{
	int new_state = next_state(manager->state, critical, warning);

	/* Nothing changed */
	if (new_state < 0)
		return;

	LOGP(DTEMP, LOGL_NOTICE, "Moving from state %s to %s.\n",
		get_value_string(state_names, manager->state),
		get_value_string(state_names, new_state));
	manager->state = new_state;
	switch (manager->state) {
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

	/* Read the current digital temperature */
	rc = sysmobts_temp_get(SYSMOBTS_TEMP_DIGITAL, SYSMOBTS_TEMP_INPUT);
	if (rc < 0) {
		LOGP(DTEMP, LOGL_ERROR,
			"Failed to read the digital temperature. rc=%d\n", rc);
		warn_thresh_passed = crit_thresh_passed = 1;
	} else {
		int temp = rc / 1000;
		if (temp > s_mgr->digital_limit.thresh_warn)
			warn_thresh_passed = 1;
		if (temp > s_mgr->digital_limit.thresh_crit)
			crit_thresh_passed = 1;
		LOGP(DTEMP, LOGL_DEBUG, "Digital temperature is: %d\n", temp);
	}

	/* Read the current RF temperature */
	rc = sysmobts_temp_get(SYSMOBTS_TEMP_RF, SYSMOBTS_TEMP_INPUT);
	if (rc < 0) {
		LOGP(DTEMP, LOGL_ERROR,
			"Failed to read the RF temperature. rc=%d\n", rc);
		warn_thresh_passed = crit_thresh_passed = 1;
	} else {
		int temp = rc / 1000;
		if (temp > s_mgr->rf_limit.thresh_warn)
			warn_thresh_passed = 1;
		if (temp > s_mgr->rf_limit.thresh_crit)
			crit_thresh_passed = 1;
		LOGP(DTEMP, LOGL_DEBUG, "RF temperature is: %d\n", temp);
	}

	if (is_sbts2050_master()) {
		int temp_pa, temp_board;

		rc = sbts2050_uc_check_temp(&temp_pa, &temp_board);
		if (rc != 0) {
			/* XXX what do here? */
			LOGP(DTEMP, LOGL_ERROR,
				"Failed to read the temperature! Reboot?!\n");
			warn_thresh_passed = 1;
			crit_thresh_passed = 1;
		} else {
			LOGP(DTEMP, LOGL_DEBUG, "SBTS2050 board(%d) PA(%d)\n",
				temp_board, temp_pa);
			if (temp_pa > s_mgr->pa_limit.thresh_warn)
				warn_thresh_passed = 1;
			if (temp_pa > s_mgr->pa_limit.thresh_crit)
				crit_thresh_passed = 1;
			if (temp_board > s_mgr->board_limit.thresh_warn)
				warn_thresh_passed = 1;
			if (temp_board > s_mgr->board_limit.thresh_crit)
				crit_thresh_passed = 1;
		}
	}

	sysmobts_mgr_temp_handle(s_mgr, crit_thresh_passed, warn_thresh_passed);	
}

static void temp_ctrl_check_cb(void *unused)
{
	temp_ctrl_check();
	/* Check every two minutes? XXX make it configurable! */
	osmo_timer_schedule(&temp_ctrl_timer, 2 * 60, 0);
}

int sysmobts_mgr_temp_init(struct sysmobts_mgr_instance *mgr)
{
	s_mgr = mgr;
	temp_ctrl_timer.cb = temp_ctrl_check_cb;
	temp_ctrl_check_cb(NULL);
	return 0;
}
