/* NM FSM, common bits */

/* (C) 2020 by sysmocom - s.m.f.c. GmbH <info@sysmocom.de>
 * Author: Pau Espin Pedrol <pespin@sysmocom.de>
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
#include <osmocom/core/utils.h>
#include <osmo-bts/nm_common_fsm.h>


const struct value_string nm_fsm_event_names[] = {
	{ NM_EV_SW_ACT, "SW_ACT" },
	{ NM_EV_OPSTART_ACK, "OPSTART_ACK" },
	{ NM_EV_OPSTART_NACK, "OPSTART_NACK" },
	{ NM_EV_SHUTDOWN_START, "SHUTDOWN_START" },
	{ NM_EV_SHUTDOWN_FINISH, "SHUTDOWN_FINISH" },
	{ NM_EV_RSL_UP, "RSL_UP" },
	{ NM_EV_RSL_DOWN, "RSL_DOWN" },
	{ NM_EV_PHYLINK_UP, "PHYLINK_UP" },
	{ NM_EV_PHYLINK_DOWN, "PHYLINK_DOWN" },
	{ NM_EV_DISABLE, "DISABLE" },
	{ NM_EV_BBTRANSC_INSTALLED, "BBTRANSC_INSTALLED" },
	{ NM_EV_BBTRANSC_ENABLED, "BBTRANSC_ENABLED" },
	{ NM_EV_BBTRANSC_DISABLED, "BBTRANSC_DISABLED" },
	{ NM_EV_RCARRIER_ENABLED, "RCARRIER_ENABLED" },
	{ NM_EV_RCARRIER_DISABLED, "RCARRIER_DISABLED" },
	{ 0, NULL }
};
