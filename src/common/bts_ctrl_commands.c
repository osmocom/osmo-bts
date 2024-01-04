/* Control Interface for osmo-bts */

/* (C) 2014 by Harald Welte <laforge@gnumonks.org>
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

#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>

#include <osmocom/gsm/protocol/gsm_12_21.h>
#include <osmocom/ctrl/control_cmd.h>

#include <osmo-bts/logging.h>
#include <osmo-bts/gsm_data.h>
#include <osmo-bts/tx_power.h>
#include <osmo-bts/signal.h>
#include <osmo-bts/oml.h>
#include <osmo-bts/bts.h>

static struct gsm_bts *g_bts;

CTRL_CMD_DEFINE(therm_att, "thermal-attenuation");
static int get_therm_att(struct ctrl_cmd *cmd, void *data)
{
	struct gsm_bts_trx *trx = cmd->node;
	struct trx_power_params *tpp = &trx->power_params;

	cmd->reply = talloc_asprintf(cmd, "%d", tpp->thermal_attenuation_mdB);

	return CTRL_CMD_REPLY;
}

static int set_therm_att(struct ctrl_cmd *cmd, void *data)
{
	struct gsm_bts_trx *trx = cmd->node;
	struct trx_power_params *tpp = &trx->power_params;
	int val = atoi(cmd->value);

	printf("set_therm_att(trx=%p, tpp=%p)\n", trx, tpp);

	tpp->thermal_attenuation_mdB = val;

	power_ramp_start(trx, tpp->p_total_cur_mdBm, 0, NULL);

	return get_therm_att(cmd, data);
}

static int verify_therm_att(struct ctrl_cmd *cmd, const char *value, void *data)
{
	int val = atoi(value);

	/* permit between 0 to 40 dB attenuation */
	if (val < 0 || val > to_mdB(40))
		return 1;

	return 0;
}

CTRL_CMD_DEFINE_WO_NOVRF(oml_alert, "oml-alert");
static int set_oml_alert(struct ctrl_cmd *cmd, void *data)
{
	/* Note: we expect signal dispatch to be synchronous */
	oml_tx_failure_event_rep(&g_bts->mo, NM_SEVER_INDETERMINATE, OSMO_EVT_EXT_ALARM, cmd->value);

	cmd->reply = "OK";

	return CTRL_CMD_REPLY;
}

static int verify_max_ber10k_rach(struct ctrl_cmd *cmd, const char *value, void *_data)
{
	int max_ber10k_rach = atoi(cmd->value);

	if (max_ber10k_rach < 0 || max_ber10k_rach > 10000) {
		cmd->reply = "Value is out of range";
		return 1;
	}

	return 0;
}

static int get_max_ber10k_rach(struct ctrl_cmd *cmd, void *data)
{
	cmd->reply = talloc_asprintf(cmd, "%u", g_bts->max_ber10k_rach);
	if (!cmd->reply) {
		cmd->reply = "OOM";
		return CTRL_CMD_ERROR;
	}

	return CTRL_CMD_REPLY;
}

static int set_max_ber10k_rach(struct ctrl_cmd *cmd, void *data)
{
	g_bts->max_ber10k_rach = atoi(cmd->value);
	cmd->reply = "OK";
	return CTRL_CMD_REPLY;
}

CTRL_CMD_DEFINE(max_ber10k_rach, "max-ber10k-rach");

int bts_ctrl_cmds_install(struct gsm_bts *bts)
{
	int rc = 0;

	rc |= ctrl_cmd_install(CTRL_NODE_TRX, &cmd_therm_att);
	rc |= ctrl_cmd_install(CTRL_NODE_ROOT, &cmd_oml_alert);
	rc |= ctrl_cmd_install(CTRL_NODE_ROOT, &cmd_max_ber10k_rach);
	g_bts = bts;

	return rc;
}
