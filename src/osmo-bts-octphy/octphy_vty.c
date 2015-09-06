/* VTY interface for osmo-bts OCTPHY integration */

/* (C) 2015 by Harald Welte <laforge@gnumonks.org>
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
 * GNU Affero General Public License for more details.
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

#include <arpa/inet.h>

#include <osmocom/core/msgb.h>
#include <osmocom/core/talloc.h>
#include <osmocom/core/select.h>
#include <osmocom/core/rate_ctr.h>
#include <osmocom/core/macaddr.h>

#include <osmocom/vty/vty.h>
#include <osmocom/vty/command.h>
#include <osmocom/vty/misc.h>

#include <osmo-bts/gsm_data.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/vty.h>

#include "l1_if.h"
#include "l1_utils.h"
#include "octphy_hw_api.h"

#define TRX_STR "Transceiver related commands\n" "TRX number\n"

#define SHOW_TRX_STR				\
	SHOW_STR				\
	TRX_STR

static struct gsm_bts *vty_bts;

/* configuration */

DEFUN(cfg_bts_phy_hwaddr, cfg_bts_phy_hwaddr_cmd,
	"phy-hw-addr HWADDR",
	"Configure the hardware addess of the OCTPHY\n"
	"hardware address in aa:bb:cc:dd:ee:ff format\n")
{
	struct gsm_bts *bts = vty->index;
	struct gsm_bts_trx *trx = bts->c0;
	struct octphy_hdl *fl1h = trx_octphy_hdl(trx);
	int rc;

	rc = osmo_macaddr_parse(fl1h->phy_addr.sll_addr, argv[0]);
	if (rc < 0)
		return CMD_WARNING;

	return CMD_SUCCESS;
}

DEFUN(cfg_bts_phy_netdev, cfg_bts_phy_netdev_cmd,
	"phy-netdev NAME",
	"Configure the hardware device towards the OCTPHY\n"
	"Ethernet device name\n")
{
	struct gsm_bts *bts = vty->index;
	struct gsm_bts_trx *trx = bts->c0;
	struct octphy_hdl *fl1h = trx_octphy_hdl(trx);

	if (fl1h->netdev_name)
		talloc_free(fl1h->netdev_name);
	fl1h->netdev_name = talloc_strdup(fl1h, argv[0]);

	return CMD_SUCCESS;
}

DEFUN(cfg_trx_rf_port_idx, cfg_trx_rf_port_idx_cmd,
	"rf-port-index <0-255>",
	"Configure the RF Port for this TRX\n"
	"RF Port Index\n")
{
	struct gsm_bts_trx *trx = vty->index;
	struct octphy_hdl *fl1h = trx_octphy_hdl(trx);

	fl1h->config.rf_port_index = atoi(argv[0]);

	return CMD_SUCCESS;
}

DEFUN(cfg_trx_rx_gain_db, cfg_trx_rx_gain_db_cmd,
	"rx-gain <0-73>",
	"Configure the Rx Gain in dB\n"
	"Rx gain in dB\n")
{
	struct gsm_bts_trx *trx = vty->index;
	struct octphy_hdl *fl1h = trx_octphy_hdl(trx);

	fl1h->config.rx_gain_db = atoi(argv[0]);

	return CMD_SUCCESS;
}

DEFUN(cfg_trx_tx_atten_db, cfg_trx_tx_atten_db_cmd,
	"tx-attenuation <0-359>",
	"Configure the Tx Attenuation in quarter-dB\n"
	"Tx attenuation in quarter-dB\n")
{
	struct gsm_bts_trx *trx = vty->index;
	struct octphy_hdl *fl1h = trx_octphy_hdl(trx);

	fl1h->config.tx_atten_db = atoi(argv[0]);

	return CMD_SUCCESS;
}

DEFUN(get_rf_port_stats, get_rf_port_stats_cmd,
	"get-rf-port-stats <0-1>",
	"Obtain statistics for the RF Port\n"
	"RF Port Number\n")
{
	struct octphy_hdl *fl1h = trx_octphy_hdl(vty_bts->c0);

	octphy_hw_get_rf_port_stats(fl1h, atoi(argv[0]));

	return CMD_SUCCESS;
}

DEFUN(get_clk_sync_stats, get_clk_sync_stats_cmd,
	"get-clk-sync-stats",
	"Obtain statistics for the Clock Sync Manager\n")
{
	struct octphy_hdl *fl1h = trx_octphy_hdl(vty_bts->c0);

	octphy_hw_get_clock_sync_stats(fl1h);

	return CMD_SUCCESS;
}

void bts_model_config_write_bts(struct vty *vty, struct gsm_bts *bts)
{
	struct gsm_bts_role_bts *btsb = bts_role_bts(bts);
	struct octphy_hdl *fl1h = trx_octphy_hdl(bts->c0);

	if (btsb->auto_band)
		vty_out(vty, " auto-band%s", VTY_NEWLINE);

	vty_out(vty, " phy-hw-addr %02x:%02x:%02x:%02x:%02x:%02x%s",
		fl1h->phy_addr.sll_addr[0], fl1h->phy_addr.sll_addr[1],
		fl1h->phy_addr.sll_addr[2], fl1h->phy_addr.sll_addr[3],
		fl1h->phy_addr.sll_addr[4], fl1h->phy_addr.sll_addr[5],
		VTY_NEWLINE);
}

void bts_model_config_write_trx(struct vty *vty, struct gsm_bts_trx *trx)
{
	struct octphy_hdl *fl1h = trx_octphy_hdl(trx);

	vty_out(vty, "  rx-gain %u%s", fl1h->config.rx_gain_db,
		VTY_NEWLINE);
	vty_out(vty, "  tx-attenuation %u%s", fl1h->config.tx_atten_db,
		VTY_NEWLINE);
}

int bts_model_vty_init(struct gsm_bts *bts)
{
	vty_bts = bts;

	install_element(BTS_NODE, &cfg_bts_phy_hwaddr_cmd);
	install_element(BTS_NODE, &cfg_bts_phy_netdev_cmd);

	install_element(TRX_NODE, &cfg_trx_rf_port_idx_cmd);
	install_element(TRX_NODE, &cfg_trx_rx_gain_db_cmd);
	install_element(TRX_NODE, &cfg_trx_tx_atten_db_cmd);

	install_element_ve(&get_rf_port_stats_cmd);
	install_element_ve(&get_clk_sync_stats_cmd);

	return 0;
}
