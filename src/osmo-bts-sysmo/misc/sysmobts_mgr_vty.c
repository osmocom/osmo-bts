/* (C) 2014 by sysmocom - s.f.m.c. GmbH
 *
 * All Rights Reserved
 *
 * Author: Alvaro Neira Ayuso <anayuso@sysmocom.de>
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

#include "sysmobts_misc.h"
#include "sysmobts_mgr.h"

static struct sbts2050_config_info *mgr_cfg;

enum node_type mgr_vty_go_parent(struct vty *vty)
{
	switch (vty->node) {
	case MGR_NODE:
		vty->node = CONFIG_NODE;
		break;
	default:
		vty->node = CONFIG_NODE;
	}
	return vty->node;
}

int mgr_vty_is_config_node(struct vty *vty, int node)
{
	switch (node) {
	case MGR_NODE:
		return 1;
	default:
		return 0;
	}
}

#define MGR_STR			"Configure sysmobts-mgr\n"
#define MGR_TEMP_WARN_STR	"Configure the temperature warning limits\n"
#define MGR_TEMP_SEVERE_STR	"Configure the temperature severe limits\n"

static struct cmd_node mgr_node = {
	MGR_NODE,
	"%s(config-mgr)# ",
	1,
};

DEFUN(cfg_mgr, cfg_mgr_cmd,
	"config-mgr",
	MGR_STR)
{
	vty->node = MGR_NODE;
	return CMD_SUCCESS;
}

DEFUN(show_mgr, show_mgr_cmd, "show mgr",
      SHOW_STR "Display information about the mgr")
{
	vty_out(vty, " temperature-warning board Min:%d Max:%d%s",
		mgr_cfg->temp_min_board_warn_limit,
		mgr_cfg->temp_max_board_warn_limit,
		VTY_NEWLINE);

	vty_out(vty, " temperature-severe board Min:%d Max:%d%s",
		mgr_cfg->temp_min_board_severe_limit,
		mgr_cfg->temp_max_board_severe_limit,
		VTY_NEWLINE);

	vty_out(vty, " temperature-warning pa Min:%d Max:%d%s",
		mgr_cfg->temp_min_pa_warn_limit,
		mgr_cfg->temp_max_pa_warn_limit,
		VTY_NEWLINE);

	vty_out(vty, " temperature-severe pa Min:%d Max:%d%s",
		mgr_cfg->temp_min_pa_severe_limit,
		mgr_cfg->temp_max_pa_severe_limit,
		VTY_NEWLINE);

	vty_out(vty, " power-action Master:%s Slave:%s PA:%s%s",
		mgr_cfg->master_power_act ? "on" : "off",
		mgr_cfg->slave_power_act ? "on" : "off",
		mgr_cfg->pa_power_act ? "on" : "off",
		VTY_NEWLINE);

	vty_out(vty, " power-reduce-transmitter %d%s",
		mgr_cfg->reduce_max_power,
		VTY_NEWLINE);
	return CMD_SUCCESS;
}

static int config_write_mgr(struct vty *vty)
{
	vty_out(vty, "config-mgr%s", VTY_NEWLINE);

	vty_out(vty, " temperature-warning board %d %d%s",
		mgr_cfg->temp_min_board_warn_limit,
		mgr_cfg->temp_max_board_warn_limit,
		VTY_NEWLINE);

	vty_out(vty, " temperature-severe board %d %d%s",
		mgr_cfg->temp_min_board_severe_limit,
		mgr_cfg->temp_max_board_severe_limit,
		VTY_NEWLINE);

	vty_out(vty, " temperature-warning pa %d %d%s",
		mgr_cfg->temp_min_pa_warn_limit,
		mgr_cfg->temp_max_pa_warn_limit,
		VTY_NEWLINE);

	vty_out(vty, " temperature-severe pa %d %d%s",
		mgr_cfg->temp_min_pa_severe_limit,
		mgr_cfg->temp_max_pa_severe_limit,
		VTY_NEWLINE);

	vty_out(vty, " power-action %s %s %s%s",
		mgr_cfg->master_power_act ? "on" : "off",
		mgr_cfg->slave_power_act ? "on" : "off",
		mgr_cfg->pa_power_act ? "on" : "off",
		VTY_NEWLINE);

	vty_out(vty, " power-reduce-transmitter %d%s",
		mgr_cfg->reduce_max_power,
		VTY_NEWLINE);

	return CMD_SUCCESS;
}

DEFUN(cfg_mgr_board_temp_warn, cfg_mgr_board_temp_warn_cmd,
	"temperature-warning board <-255-255> <-255-255>",
	MGR_TEMP_WARN_STR
	"Set warning temperature limits on the Board\n"
	"Warning temperature low limit on the Board\n"
	"Warning temperature high limit on the Board\n")
{
	mgr_cfg->temp_min_board_warn_limit = atoi(argv[0]);
	mgr_cfg->temp_max_board_warn_limit = atoi(argv[1]);

	return CMD_SUCCESS;
}

DEFUN(cfg_mgr_board_temp_sever, cfg_mgr_board_temp_sever_cmd,
	"temperature-severe board <-255-255> <-255-255>",
	MGR_TEMP_SEVERE_STR
	"Set severe temperature limits on the Board\n"
	"Severe temperature low limit on the Board\n"
	"Severe Temperature  high limit on the Board\n")
{
	mgr_cfg->temp_min_board_severe_limit = atoi(argv[0]);
	mgr_cfg->temp_max_board_severe_limit = atoi(argv[1]);

	return CMD_SUCCESS;
}

DEFUN(cfg_mgr_pa_temp_warn, cfg_mgr_pa_temp_warn_cmd,
	"temperature-warning pa <-255-255> <-255-255>",
	MGR_TEMP_WARN_STR
	"Set warning temperature limits on the PA\n"
	"Warning temperature low limit on the PA\n"
	"Warning temperature high limit on the PA\n")
{
	mgr_cfg->temp_min_pa_warn_limit = atoi(argv[0]);
	mgr_cfg->temp_max_pa_warn_limit = atoi(argv[1]);

	return CMD_SUCCESS;
}

DEFUN(cfg_mgr_pa_temp_sever, cfg_mgr_pa_temp_sever_cmd,
	"temperature-severe pa <-255-255> <-255-255>",
	MGR_TEMP_SEVERE_STR
	"Set severe temperature limits on the Board\n"
	"Severe temperature low limit on the PA\n"
	"Severe temperature high limit on the PA\n")
{
	mgr_cfg->temp_min_pa_severe_limit = atoi(argv[0]);
	mgr_cfg->temp_max_pa_severe_limit = atoi(argv[1]);

	return CMD_SUCCESS;
}

DEFUN(cfg_mgr_pwr_action, cfg_mgr_pwr_action_cmd,
	"power-action (on|off) (on|off) (on|off)",
	"Configure which devices we want to turn on/off in several situation\n"
	"Turn on the Master\n"
	"Turn off the Master\n"
	"Turn on the Slave\n"
	"Turn off the Slave\n"
	"Turn on the PA\n"
	"Turn off the PA\n")
{
	if (strcmp(argv[0], "on") == 0)
		mgr_cfg->master_power_act = 1;
	else if (strcmp(argv[0], "off") == 0)
		mgr_cfg->master_power_act = 0;

	if (strcmp(argv[1], "on") == 0)
		mgr_cfg->slave_power_act = 1;
	else if (strcmp(argv[1], "off") == 0)
		mgr_cfg->slave_power_act = 0;

	if (strcmp(argv[2], "on") == 0)
		mgr_cfg->pa_power_act = 1;
	else if (strcmp(argv[2], "off") == 0)
		mgr_cfg->pa_power_act = 0;

	return CMD_SUCCESS;
}

DEFUN(cfg_mgr_pa_baud_action, cfg_mgr_pa_baud_action_cmd,
	"power-reduce-transmitter <0-255>",
	"Configure the power that we want to reduce in warning situation\n"
	"Power baud transmition that we want to reduce in the PA\n")
{
	mgr_cfg->reduce_max_power = atoi(argv[0]);

	return CMD_SUCCESS;
}

int sysmobts_mgr_vty_init(void)

{
	install_element_ve(&show_mgr_cmd);

	install_node(&mgr_node, config_write_mgr);
	install_element(CONFIG_NODE, &cfg_mgr_cmd);
	vty_install_default(MGR_NODE);

	install_element(MGR_NODE, &cfg_mgr_board_temp_warn_cmd);
	install_element(MGR_NODE, &cfg_mgr_board_temp_sever_cmd);
	install_element(MGR_NODE, &cfg_mgr_pa_temp_warn_cmd);
	install_element(MGR_NODE, &cfg_mgr_pa_temp_sever_cmd);
	install_element(MGR_NODE, &cfg_mgr_pwr_action_cmd);
	install_element(MGR_NODE, &cfg_mgr_pa_baud_action_cmd);

	return 0;
}

int sysmobts_mgr_parse_config(const char *config_file,
			      struct sbts2050_config_info *cfg)
{
	int rc;

	mgr_cfg = cfg;

	rc = vty_read_config_file(config_file, NULL);
	if (rc < 0) {
		fprintf(stderr, "Failed to parse the config file: '%s'\n",
				config_file);
		return rc;
	}

	return 0;
}
