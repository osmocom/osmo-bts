/* VTY interface for virtual OsmoBTS */

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

#include <osmocom/gsm/tlv.h>

#include <osmocom/vty/vty.h>
#include <osmocom/vty/command.h>
#include <osmocom/vty/misc.h>

#include <osmo-bts/gsm_data.h>
#include <osmo-bts/phy_link.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/vty.h>

#define TRX_STR "Transceiver related commands\n" "TRX number\n"

#define SHOW_TRX_STR				\
	SHOW_STR				\
	TRX_STR

static struct gsm_bts *vty_bts;

void bts_model_config_write_bts(struct vty *vty, struct gsm_bts *bts)
{
}

void bts_model_config_write_trx(struct vty *vty, struct gsm_bts_trx *trx)
{
}

void bts_model_config_write_phy(struct vty *vty, struct phy_link *plink)
{
	if (plink->u.virt.mcast_dev)
		vty_out(vty, " virtual-um net-device %s%s",
			plink->u.virt.mcast_dev, VTY_NEWLINE);
	if (plink->u.virt.mcast_group)
		vty_out(vty, " virtual-um multicast-group %s%s",
			plink->u.virt.mcast_group, VTY_NEWLINE);
	if (plink->u.virt.mcast_port)
		vty_out(vty, " virtual-um udp-port %u%s",
			plink->u.virt.mcast_port, VTY_NEWLINE);
}

#define VUM_STR	"Virtual Um layer\n"

DEFUN(cfg_phy_mcast_group, cfg_phy_mcast_group_cmd,
	"virtual-um multicast-group GROUP",
	VUM_STR "Configure the multicast group\n")
{
	struct phy_link *plink = vty->index;

	if (plink->state != PHY_LINK_SHUTDOWN) {
		vty_out(vty, "Can only reconfigure a PHY link that is down%s",
			VTY_NEWLINE);
		return CMD_WARNING;
	}

	if (plink->u.virt.mcast_group)
		talloc_free(plink->u.virt.mcast_group);
	plink->u.virt.mcast_group = talloc_strdup(plink, argv[0]);

	return CMD_SUCCESS;
}


DEFUN(cfg_phy_mcast_port, cfg_phy_mcast_port_cmd,
	"virtual-um udp-port <0-65535>",
	VUM_STR "Configure the UDP port\n")
{
	struct phy_link *plink = vty->index;

	if (plink->state != PHY_LINK_SHUTDOWN) {
		vty_out(vty, "Can only reconfigure a PHY link that is down%s",
			VTY_NEWLINE);
		return CMD_WARNING;
	}

	plink->u.virt.mcast_port = atoi(argv[0]);

	return CMD_SUCCESS;
}

DEFUN(cfg_phy_mcast_dev, cfg_phy_mcast_dev_cmd,
	"virtual-um net-device NETDEV",
	VUM_STR "Configure the network device\n")
{
	struct phy_link *plink = vty->index;

	if (plink->state != PHY_LINK_SHUTDOWN) {
		vty_out(vty, "Can only reconfigure a PHY link that is down%s",
			VTY_NEWLINE);
		return CMD_WARNING;
	}

	if (plink->u.virt.mcast_dev)
		talloc_free(plink->u.virt.mcast_dev);
	plink->u.virt.mcast_dev = talloc_strdup(plink, argv[0]);

	return CMD_SUCCESS;
}

int bts_model_vty_init(struct gsm_bts *bts)
{
	vty_bts = bts;

	install_element(PHY_NODE, &cfg_phy_mcast_group_cmd);
	install_element(PHY_NODE, &cfg_phy_mcast_port_cmd);
	install_element(PHY_NODE, &cfg_phy_mcast_dev_cmd);

	return 0;
}
