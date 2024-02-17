/* Provision TRX over TRXC protocol FSM */

/* (C) 2020 by sysmocom - s.f.m.c. GmbH <info@sysmocom.de>
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
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#pragma once

#include <stdbool.h>

#include <osmocom/core/fsm.h>

enum trx_provision_fsm_states {
	TRX_PROV_ST_CLOSED,
	TRX_PROV_ST_OPEN_POWEROFF,
	TRX_PROV_ST_OPEN_WAIT_POWERON_CNF,
	TRX_PROV_ST_OPEN_POWERON,
	TRX_PROV_ST_OPEN_WAIT_POWEROFF_CNF,
};

struct trx_prov_ev_cfg_ts_data {
	uint8_t tn;
	uint8_t slottype;

	/* Training Sequence Code and Set */
	uint8_t tsc_set;
	uint8_t tsc_val;
	bool tsc_valid;
};

enum trx_provision_fsm_events {
	TRX_PROV_EV_OTHER_TRX_READY,
	TRX_PROV_EV_OPEN,
	TRX_PROV_EV_CFG_ENABLE,
	TRX_PROV_EV_CFG_BSIC,
	TRX_PROV_EV_CFG_ARFCN,
	TRX_PROV_EV_CFG_TSC,
	TRX_PROV_EV_CFG_TS,
	TRX_PROV_EV_CFG_RXGAIN,
	TRX_PROV_EV_CFG_SETMAXDLY,
	TRX_PROV_EV_RXTUNE_CNF,
	TRX_PROV_EV_TXTUNE_CNF,
	TRX_PROV_EV_NOMTXPOWER_CNF,
	TRX_PROV_EV_SETBSIC_CNF,
	TRX_PROV_EV_SETTSC_CNF,
	TRX_PROV_EV_SETFORMAT_CNF,
	TRX_PROV_EV_POWERON_CNF,
	TRX_PROV_EV_POWEROFF_CNF,
	TRX_PROV_EV_CLOSE,
};

extern struct osmo_fsm trx_prov_fsm;
