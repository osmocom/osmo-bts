/* BTS shutdown FSM */

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

#pragma once

#include <osmocom/core/fsm.h>

/* 3GPP TS 24.008 § 4.1.3.3 GMM mobility management states on the network side */
enum bts_shutdown_fsm_states {
	BTS_SHUTDOWN_ST_NONE,
	BTS_SHUTDOWN_ST_WAIT_RAMP_DOWN_COMPL,
	BTS_SHUTDOWN_ST_WAIT_TRX_CLOSED,
	BTS_SHUTDOWN_ST_EXIT,
};

enum bts_shutdown_fsm_events {
	BTS_SHUTDOWN_EV_START,
	BTS_SHUTDOWN_EV_TRX_RAMP_COMPL,
	BTS_SHUTDOWN_EV_TRX_CLOSED,
};

extern struct osmo_fsm bts_shutdown_fsm;
