/* Copyright (C) 2015 by Yves Godin <support@nuranwireless.com>
 * 
 * Based on sysmoBTS:
 *     sysmobts_mgr_vty.c
 *     (C) 2014 by lc15com - s.f.m.c. GmbH
 *
 * All Rights Reserved
 *
 * Author: Alvaro Neira Ayuso <anayuso@lc15com.de>
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

#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <osmocom/vty/vty.h>
#include <osmocom/vty/command.h>
#include <osmocom/vty/misc.h>

#include <osmo-bts/logging.h>

#include "lc15bts_misc.h"
#include "lc15bts_mgr.h"
#include "lc15bts_temp.h"
#include "lc15bts_power.h"
#include "btsconfig.h"

static struct lc15bts_mgr_instance *s_mgr;

static const char copyright[] =
	"(C) 2012 by Harald Welte <laforge@gnumonks.org>\r\n"
	"(C) 2014 by Holger Hans Peter Freyther\r\n"
	"(C) 2015 by Yves Godin <support@nuranwireless.com>\r\n"
	"License AGPLv3+: GNU AGPL version 2 or later <http://gnu.org/licenses/agpl-3.0.html>\r\n"
	"This is free software: you are free to change and redistribute it.\r\n"
	"There is NO WARRANTY, to the extent permitted by law.\r\n";

static int go_to_parent(struct vty *vty)
{
	switch (vty->node) {
	case MGR_NODE:
		vty->node = CONFIG_NODE;
		break;
	case ACT_NORM_NODE:
	case ACT_WARN_NODE:
	case ACT_CRIT_NODE:
	case LIMIT_SUPPLY_NODE:
	case LIMIT_SOC_NODE:
	case LIMIT_FPGA_NODE:
	case LIMIT_MEMORY_NODE:
	case LIMIT_TX1_NODE:
	case LIMIT_TX2_NODE:
	case LIMIT_PA1_NODE:
	case LIMIT_PA2_NODE:
		vty->node = MGR_NODE;
		break;
	default:
		vty->node = CONFIG_NODE;
	}
	return vty->node;
}

static int is_config_node(struct vty *vty, int node)
{
	switch (node) {
	case MGR_NODE:
	case ACT_NORM_NODE:
	case ACT_WARN_NODE:
	case ACT_CRIT_NODE:
	case LIMIT_SUPPLY_NODE:
	case LIMIT_SOC_NODE:
	case LIMIT_FPGA_NODE:
	case LIMIT_MEMORY_NODE:
	case LIMIT_TX1_NODE:
	case LIMIT_TX2_NODE:
	case LIMIT_PA1_NODE:
	case LIMIT_PA2_NODE:
		return 1;
	default:
		return 0;
	}
}

static struct vty_app_info vty_info = {
	.name           = "lc15bts-mgr",
	.version        = PACKAGE_VERSION,
	.go_parent_cb   = go_to_parent,
	.is_config_node = is_config_node,
	.copyright	= copyright,
};


#define MGR_STR			"Configure lc15bts-mgr\n"

static struct cmd_node mgr_node = {
	MGR_NODE,
	"%s(lc15bts-mgr)# ",
	1,
};

static struct cmd_node act_norm_node = {
	ACT_NORM_NODE,
	"%s(action-normal)# ",
	1,
};

static struct cmd_node act_warn_node = {
	ACT_WARN_NODE,
	"%s(action-warn)# ",
	1,
};

static struct cmd_node act_crit_node = {
	ACT_CRIT_NODE,
	"%s(action-critical)# ",
	1,
};

static struct cmd_node limit_supply_node = {
	LIMIT_SUPPLY_NODE,
	"%s(limit-supply)# ",
	1,
};

static struct cmd_node limit_soc_node = {
	LIMIT_SOC_NODE,
	"%s(limit-soc)# ",
	1,
};

static struct cmd_node limit_fpga_node = {
	LIMIT_FPGA_NODE,
	"%s(limit-fpga)# ",
	1,
};

static struct cmd_node limit_memory_node = {
	LIMIT_MEMORY_NODE,
	"%s(limit-memory)# ",
	1,
};

static struct cmd_node limit_tx1_node = {
	LIMIT_TX1_NODE,
	"%s(limit-tx1)# ",
	1,
};
static struct cmd_node limit_tx2_node = {
	LIMIT_TX2_NODE,
	"%s(limit-tx2)# ",
	1,
};
static struct cmd_node limit_pa1_node = {
	LIMIT_PA1_NODE,
	"%s(limit-pa1)# ",
	1,
};
static struct cmd_node limit_pa2_node = {
	LIMIT_PA2_NODE,
	"%s(limit-pa2)# ",
	1,
};

DEFUN(cfg_mgr, cfg_mgr_cmd,
	"lc15bts-mgr",
	MGR_STR)
{
	vty->node = MGR_NODE;
	return CMD_SUCCESS;
}

static void write_temp_limit(struct vty *vty, const char *name,
				struct lc15bts_temp_limit *limit)
{
	vty_out(vty, " %s%s", name, VTY_NEWLINE);
	vty_out(vty, "   threshold warning %d%s",
		limit->thresh_warn, VTY_NEWLINE);
	vty_out(vty, "   threshold critical %d%s",
		limit->thresh_crit, VTY_NEWLINE);
}

static void write_norm_action(struct vty *vty, const char *name, int actions)
{
	vty_out(vty, " %s%s", name, VTY_NEWLINE);
	vty_out(vty, "  %spa1-on%s",
		(actions & TEMP_ACT_NORM_PA1_ON) ? "" : "no ", VTY_NEWLINE);
	vty_out(vty, "  %spa2-on%s",
		(actions & TEMP_ACT_NORM_PA2_ON) ? "" : "no ", VTY_NEWLINE);
	vty_out(vty, "  %sbts-service-on%s",
		(actions & TEMP_ACT_NORM_BTS_SRV_ON) ? "" : "no ", VTY_NEWLINE);
}

static void write_action(struct vty *vty, const char *name, int actions)
{
	vty_out(vty, " %s%s", name, VTY_NEWLINE);
	vty_out(vty, "  %spa1-off%s",
		(actions & TEMP_ACT_PA1_OFF) ? "" : "no ", VTY_NEWLINE);
	vty_out(vty, "  %spa2-off%s",
		(actions & TEMP_ACT_PA2_OFF) ? "" : "no ", VTY_NEWLINE);
	vty_out(vty, "  %sbts-service-off%s",
		(actions & TEMP_ACT_BTS_SRV_OFF) ? "" : "no ", VTY_NEWLINE);
}

static int config_write_mgr(struct vty *vty)
{
	vty_out(vty, "lc15bts-mgr%s", VTY_NEWLINE);

	write_temp_limit(vty, "limits supply", &s_mgr->temp.supply_limit);
	write_temp_limit(vty, "limits soc", &s_mgr->temp.soc_limit);
	write_temp_limit(vty, "limits fpga", &s_mgr->temp.fpga_limit);
	write_temp_limit(vty, "limits memory", &s_mgr->temp.memory_limit);
	write_temp_limit(vty, "limits tx1", &s_mgr->temp.tx1_limit);
	write_temp_limit(vty, "limits tx2", &s_mgr->temp.tx2_limit);
	write_temp_limit(vty, "limits pa1", &s_mgr->temp.pa1_limit);
	write_temp_limit(vty, "limits pa2", &s_mgr->temp.pa2_limit);

	write_norm_action(vty, "actions normal", s_mgr->temp.action_norm);
	write_action(vty, "actions warn", s_mgr->temp.action_warn);
	write_action(vty, "actions critical", s_mgr->temp.action_crit);

	return CMD_SUCCESS;
}

static int config_write_dummy(struct vty *vty)
{
	return CMD_SUCCESS;
}

#define CFG_LIMIT(name, expl, switch_to, variable)			\
DEFUN(cfg_limit_##name, cfg_limit_##name##_cmd,				\
	"limits " #name,						\
	"Configure Limits\n" expl)					\
{									\
	vty->node = switch_to;						\
	vty->index = &s_mgr->temp.variable;				\
	return CMD_SUCCESS;						\
}

CFG_LIMIT(supply, "SUPPLY\n", LIMIT_SUPPLY_NODE, supply_limit)
CFG_LIMIT(soc, "SOC\n", LIMIT_SOC_NODE, soc_limit)
CFG_LIMIT(fpga, "FPGA\n", LIMIT_FPGA_NODE, fpga_limit)
CFG_LIMIT(memory, "MEMORY\n", LIMIT_MEMORY_NODE, memory_limit)
CFG_LIMIT(tx1, "TX1\n", LIMIT_TX1_NODE, tx1_limit)
CFG_LIMIT(tx2, "TX2\n", LIMIT_TX2_NODE, tx2_limit)
CFG_LIMIT(pa1, "PA1\n", LIMIT_PA1_NODE, pa1_limit)
CFG_LIMIT(pa2, "PA2\n", LIMIT_PA2_NODE, pa2_limit)
#undef CFG_LIMIT

DEFUN(cfg_limit_warning, cfg_thresh_warning_cmd,
	"threshold warning <0-200>",
	"Threshold to reach\n" "Warning level\n" "Range\n")
{
	struct lc15bts_temp_limit *limit = vty->index;
	limit->thresh_warn = atoi(argv[0]);
	return CMD_SUCCESS;
}

DEFUN(cfg_limit_crit, cfg_thresh_crit_cmd,
	"threshold critical <0-200>",
	"Threshold to reach\n" "Severe level\n" "Range\n")
{
	struct lc15bts_temp_limit *limit = vty->index;
	limit->thresh_crit = atoi(argv[0]);
	return CMD_SUCCESS;
}

#define CFG_ACTION(name, expl, switch_to, variable)			\
DEFUN(cfg_action_##name, cfg_action_##name##_cmd,			\
	"actions " #name,						\
	"Configure Actions\n" expl)					\
{									\
	vty->node = switch_to;						\
	vty->index = &s_mgr->temp.variable;				\
	return CMD_SUCCESS;						\
}
CFG_ACTION(normal, "Normal Actions\n", ACT_NORM_NODE, action_norm)
CFG_ACTION(warn, "Warning Actions\n", ACT_WARN_NODE, action_warn)
CFG_ACTION(critical, "Critical Actions\n", ACT_CRIT_NODE, action_crit)
#undef CFG_ACTION

DEFUN(cfg_action_pa1_on, cfg_action_pa1_on_cmd,
	"pa1-on",
	"Switch the Power Amplifier #1 on\n")
{
	int *action = vty->index;
	*action |= TEMP_ACT_NORM_PA1_ON;
	return CMD_SUCCESS;
}

DEFUN(cfg_no_action_pa1_on, cfg_no_action_pa1_on_cmd,
	"no pa1-on",
	NO_STR "Switch the Power Amplifieri #1 on\n")
{
	int *action = vty->index;
	*action &= ~TEMP_ACT_NORM_PA1_ON;
	return CMD_SUCCESS;
}

DEFUN(cfg_action_pa2_on, cfg_action_pa2_on_cmd,
	"pa2-on",
	"Switch the Power Amplifier #2 on\n")
{
	int *action = vty->index;
	*action |= TEMP_ACT_NORM_PA2_ON;
	return CMD_SUCCESS;
}

DEFUN(cfg_no_action_pa2_on, cfg_no_action_pa2_on_cmd,
	"no pa2-on",
	NO_STR "Switch the Power Amplifieri #2 on\n")
{
	int *action = vty->index;
	*action &= ~TEMP_ACT_NORM_PA2_ON;
	return CMD_SUCCESS;
}

DEFUN(cfg_action_bts_srv_on, cfg_action_bts_srv_on_cmd,
	"bts-service-on",
	"Start the systemd lc15bts.service\n")
{
	int *action = vty->index;
	*action |= TEMP_ACT_NORM_BTS_SRV_ON;
	return CMD_SUCCESS;
}

DEFUN(cfg_no_action_bts_srv_on, cfg_no_action_bts_srv_on_cmd,
	"no bts-service-on",
	NO_STR "Start the systemd lc15bts.service\n")
{
	int *action = vty->index;
	*action &= ~TEMP_ACT_NORM_BTS_SRV_ON;
	return CMD_SUCCESS;
}

DEFUN(cfg_action_pa1_off, cfg_action_pa1_off_cmd,
	"pa1-off",
	"Switch the Power Amplifier #1 off\n")
{
	int *action = vty->index;
	*action |= TEMP_ACT_PA1_OFF;
	return CMD_SUCCESS;
}

DEFUN(cfg_no_action_pa1_off, cfg_no_action_pa1_off_cmd,
	"no pa1-off",
	NO_STR "Do not switch off the Power Amplifier #1\n")
{
	int *action = vty->index;
	*action &= ~TEMP_ACT_PA1_OFF;
	return CMD_SUCCESS;
}

DEFUN(cfg_action_pa2_off, cfg_action_pa2_off_cmd,
        "pa2-off",
        "Switch the Power Amplifier #2 off\n")
{
        int *action = vty->index;
        *action |= TEMP_ACT_PA2_OFF;
        return CMD_SUCCESS;
}

DEFUN(cfg_no_action_pa2_off, cfg_no_action_pa2_off_cmd,
        "no pa2-off",
        NO_STR "Do not switch off the Power Amplifier #2\n")
{
        int *action = vty->index;
        *action &= ~TEMP_ACT_PA2_OFF;
        return CMD_SUCCESS;
}

DEFUN(cfg_action_bts_srv_off, cfg_action_bts_srv_off_cmd,
	"bts-service-off",
	"Stop the systemd lc15bts.service\n")
{
	int *action = vty->index;
	*action |= TEMP_ACT_BTS_SRV_OFF;
	return CMD_SUCCESS;
}

DEFUN(cfg_no_action_bts_srv_off, cfg_no_action_bts_srv_off_cmd,
	"no bts-service-off",
	NO_STR "Stop the systemd lc15bts.service\n")
{
	int *action = vty->index;
	*action &= ~TEMP_ACT_BTS_SRV_OFF;
	return CMD_SUCCESS;
}

DEFUN(show_mgr, show_mgr_cmd, "show manager",
      SHOW_STR "Display information about the manager")
{
	vty_out(vty, "Temperature control state: %s%s",
		lc15bts_mgr_temp_get_state(s_mgr->temp.state), VTY_NEWLINE);
	vty_out(vty, "Current Temperatures%s", VTY_NEWLINE);
	vty_out(vty, " Main Supply : %f Celcius%s",
		lc15bts_temp_get(LC15BTS_TEMP_SUPPLY,
			LC15BTS_TEMP_INPUT) / 1000.0f, 
		VTY_NEWLINE);
	vty_out(vty, " SoC         : %f Celcius%s",
		lc15bts_temp_get(LC15BTS_TEMP_SOC,
			LC15BTS_TEMP_INPUT) / 1000.0f, 
		VTY_NEWLINE);
	vty_out(vty, " FPGA        : %f Celcius%s",
		lc15bts_temp_get(LC15BTS_TEMP_FPGA,
			LC15BTS_TEMP_INPUT) / 1000.0f, 
		VTY_NEWLINE);
	vty_out(vty, " Memory (DDR): %f Celcius%s",
		lc15bts_temp_get(LC15BTS_TEMP_MEMORY,
			LC15BTS_TEMP_INPUT) / 1000.0f, 
		VTY_NEWLINE);
	vty_out(vty, " TX 1        : %f Celcius%s",
		lc15bts_temp_get(LC15BTS_TEMP_TX1,
			LC15BTS_TEMP_INPUT) / 1000.0f, 
		VTY_NEWLINE);
	vty_out(vty, " TX 2        : %f Celcius%s",
		lc15bts_temp_get(LC15BTS_TEMP_TX2,
			LC15BTS_TEMP_INPUT) / 1000.0f, 
		VTY_NEWLINE);
	vty_out(vty, " Power Amp #1: %f Celcius%s",
		lc15bts_temp_get(LC15BTS_TEMP_PA1,
			LC15BTS_TEMP_INPUT) / 1000.0f, 
		VTY_NEWLINE);
	vty_out(vty, " Power Amp #2: %f Celcius%s",
		lc15bts_temp_get(LC15BTS_TEMP_PA2,
			LC15BTS_TEMP_INPUT) / 1000.0f, 
		VTY_NEWLINE);

	vty_out(vty, "Power Status%s", VTY_NEWLINE);
	vty_out(vty, " Main Supply : (ON) [%6.2f Vdc, %4.2f A, %6.2f W]%s",
		lc15bts_power_sensor_get(LC15BTS_POWER_SUPPLY, 
			LC15BTS_POWER_VOLTAGE)/1000.0f,
		lc15bts_power_sensor_get(LC15BTS_POWER_SUPPLY, 
			LC15BTS_POWER_CURRENT)/1000.0f,
		lc15bts_power_sensor_get(LC15BTS_POWER_SUPPLY, 
			LC15BTS_POWER_POWER)/1000000.0f,
		VTY_NEWLINE);
	vty_out(vty, " Power Amp #1:  %s [%6.2f Vdc, %4.2f A, %6.2f W]%s",
		lc15bts_power_get(LC15BTS_POWER_PA1) ? "ON " : "OFF",
		lc15bts_power_sensor_get(LC15BTS_POWER_PA1, 
			LC15BTS_POWER_VOLTAGE)/1000.0f,
		lc15bts_power_sensor_get(LC15BTS_POWER_PA1, 
			LC15BTS_POWER_CURRENT)/1000.0f,
		lc15bts_power_sensor_get(LC15BTS_POWER_PA1, 
			LC15BTS_POWER_POWER)/1000000.0f,
		VTY_NEWLINE);
	vty_out(vty, " Power Amp #2:  %s [%6.2f Vdc, %4.2f A, %6.2f W]%s",
		lc15bts_power_get(LC15BTS_POWER_PA2) ? "ON " : "OFF",
		lc15bts_power_sensor_get(LC15BTS_POWER_PA2, 
			LC15BTS_POWER_VOLTAGE)/1000.0f,
		lc15bts_power_sensor_get(LC15BTS_POWER_PA2, 
			LC15BTS_POWER_CURRENT)/1000.0f,
		lc15bts_power_sensor_get(LC15BTS_POWER_PA2, 
			LC15BTS_POWER_POWER)/1000000.0f,
		VTY_NEWLINE);

	return CMD_SUCCESS;
}

DEFUN(calibrate_clock, calibrate_clock_cmd,
      "calibrate clock",
      "Calibration commands\n" 
      "Calibrate clock against GPS PPS\n")
{
	if (lc15bts_mgr_calib_run(s_mgr) < 0) {
		vty_out(vty, "%%Failed to start calibration.%s", VTY_NEWLINE);
		return CMD_WARNING;
	}
	return CMD_SUCCESS;
}

static void register_limit(int limit)
{
	install_element(limit, &cfg_thresh_warning_cmd);
	install_element(limit, &cfg_thresh_crit_cmd);
}

static void register_normal_action(int act)
{
	install_element(act, &cfg_action_pa1_on_cmd);
	install_element(act, &cfg_no_action_pa1_on_cmd);
	install_element(act, &cfg_action_pa2_on_cmd);
	install_element(act, &cfg_no_action_pa2_on_cmd);
	install_element(act, &cfg_action_bts_srv_on_cmd);
	install_element(act, &cfg_no_action_bts_srv_on_cmd);
}

static void register_action(int act)
{
	install_element(act, &cfg_action_pa1_off_cmd);
	install_element(act, &cfg_no_action_pa1_off_cmd);
	install_element(act, &cfg_action_pa2_off_cmd);
	install_element(act, &cfg_no_action_pa2_off_cmd);
	install_element(act, &cfg_action_bts_srv_off_cmd);
	install_element(act, &cfg_no_action_bts_srv_off_cmd);
}

int lc15bts_mgr_vty_init(void)
{
	vty_init(&vty_info);

	install_element_ve(&show_mgr_cmd);

	install_element(ENABLE_NODE, &calibrate_clock_cmd);

	install_node(&mgr_node, config_write_mgr);
	install_element(CONFIG_NODE, &cfg_mgr_cmd);
	vty_install_default(MGR_NODE);

	/* install the limit nodes */
	install_node(&limit_supply_node, config_write_dummy);
	install_element(MGR_NODE, &cfg_limit_supply_cmd);
	register_limit(LIMIT_SUPPLY_NODE);
	vty_install_default(LIMIT_SUPPLY_NODE);

	install_node(&limit_soc_node, config_write_dummy);
	install_element(MGR_NODE, &cfg_limit_soc_cmd);
	register_limit(LIMIT_SOC_NODE);
	vty_install_default(LIMIT_SOC_NODE);

	install_node(&limit_fpga_node, config_write_dummy);
	install_element(MGR_NODE, &cfg_limit_fpga_cmd);
	register_limit(LIMIT_FPGA_NODE);
	vty_install_default(LIMIT_FPGA_NODE);

	install_node(&limit_memory_node, config_write_dummy);
	install_element(MGR_NODE, &cfg_limit_memory_cmd);
	register_limit(LIMIT_MEMORY_NODE);
	vty_install_default(LIMIT_MEMORY_NODE);

	install_node(&limit_tx1_node, config_write_dummy);
	install_element(MGR_NODE, &cfg_limit_tx1_cmd);
	register_limit(LIMIT_TX1_NODE);
	vty_install_default(LIMIT_TX1_NODE);

	install_node(&limit_tx2_node, config_write_dummy);
	install_element(MGR_NODE, &cfg_limit_tx2_cmd);
	register_limit(LIMIT_TX2_NODE);
	vty_install_default(LIMIT_TX2_NODE);

	install_node(&limit_pa1_node, config_write_dummy);
	install_element(MGR_NODE, &cfg_limit_pa1_cmd);
	register_limit(LIMIT_PA1_NODE);
	vty_install_default(LIMIT_PA1_NODE);

	install_node(&limit_pa2_node, config_write_dummy);
	install_element(MGR_NODE, &cfg_limit_pa2_cmd);
	register_limit(LIMIT_PA2_NODE);
	vty_install_default(LIMIT_PA2_NODE);

	/* install the normal node */
	install_node(&act_norm_node, config_write_dummy);
	install_element(MGR_NODE, &cfg_action_normal_cmd);
	register_normal_action(ACT_NORM_NODE);

	/* install the warning and critical node */
	install_node(&act_warn_node, config_write_dummy);
	install_element(MGR_NODE, &cfg_action_warn_cmd);
	register_action(ACT_WARN_NODE);
	vty_install_default(ACT_WARN_NODE);

	install_node(&act_crit_node, config_write_dummy);
	install_element(MGR_NODE, &cfg_action_critical_cmd);
	register_action(ACT_CRIT_NODE);
	vty_install_default(ACT_CRIT_NODE);

	return 0;
}

int lc15bts_mgr_parse_config(struct lc15bts_mgr_instance *manager)
{
	int rc;

	s_mgr = manager;
	rc = vty_read_config_file(s_mgr->config_file, NULL);
	if (rc < 0) {
		fprintf(stderr, "Failed to parse the config file: '%s'\n",
				s_mgr->config_file);
		return rc;
	}

	return 0;
}
