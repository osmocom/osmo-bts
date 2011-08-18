/* Interface to layer 1 of baseband */

/* (C) 2011 by Andreas Eversberg <jolly@eversberg.eu>
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

#include <sys/types.h>
#include <sys/stat.h>

#include <osmocom/core/talloc.h>
#include <osmocom/core/utils.h>
#include <osmocom/core/select.h>
#include <osmocom/core/timer.h>
#include <osmocom/core/write_queue.h>
#include <osmocom/gsm/gsm_utils.h>
#include <osmocom/gsm/lapdm.h>

#include <osmo-bts/logging.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/oml.h>
#include <osmo-bts/gsm_data.h>
#include <osmo-bts/signal.h>

#include "l1_if.h"

uint16_t ref_arfcn;

/* l1 confirms setup */
static int l1if_setup_conf(struct gsm_bts_trx *trx)
{

	trx_init_complete(trx, 0);

	int i;

	LOGP(DL1C, LOGL_INFO, "L1 Setup confirm\n");
	/* signal availability */
	oml_mo_state_chg(&trx->mo, NM_OPSTATE_ENABLED, NM_AVSTATE_OK);
	oml_mo_tx_sw_act_rep(&trx->mo);
	oml_mo_state_chg(&trx->bb_transc.mo, NM_OPSTATE_ENABLED, NM_AVSTATE_OK);
	oml_mo_tx_sw_act_rep(&trx->bb_transc.mo);

	return 0;
}

/* sending parameters to init L1's TRX */
int l1if_setup(struct gsm_bts_trx *trx)
{
	uint16_t arfcn = trx->arfcn;
	uint8_t bsic = trx->bts->bsic;

	LOGP(DL1C, LOGL_INFO, "Setup TRX: arfcn=%d bsic=%d/%d\n",
		arfcn, bsic / 8, bsic & 7);
	
// Sylvain: put your code here: init trx, then call on a confirm:
	l1if_setup_conf(trx);

	return 0;
}

static int l1if_new_si(struct gsm_bts_trx *trx, enum osmo_sysinfo_type osmo_si)
{
	char *name = "";

	switch (osmo_si) {
	case SYSINFO_TYPE_1:
		name = "1";
// Sylvain: put your code here
		break;
	case SYSINFO_TYPE_2:
		name = "2";
		break;
	case SYSINFO_TYPE_2bis:
		name = "2bis";
		break;
	case SYSINFO_TYPE_2ter:
		name = "2ter";
		break;
	case SYSINFO_TYPE_3:
		name = "3";
		break;
	case SYSINFO_TYPE_4:
		name = "4";
		break;
	case SYSINFO_TYPE_5:
		name = "5";
		break;
	case SYSINFO_TYPE_5bis:
		name = "5bis";
		break;
	case SYSINFO_TYPE_5ter:
		name = "5ter";
		break;
	case SYSINFO_TYPE_6:
		name = "6";
		break;
	case SYSINFO_TYPE_7:
	case SYSINFO_TYPE_8:
	case SYSINFO_TYPE_9:
	case SYSINFO_TYPE_10:
	case SYSINFO_TYPE_13:
	case SYSINFO_TYPE_16:
	case SYSINFO_TYPE_17:
	case SYSINFO_TYPE_18:
	case SYSINFO_TYPE_19:
	case SYSINFO_TYPE_20:
	case SYSINFO_TYPE_2quater:
	case SYSINFO_TYPE_EMO:
	case SYSINFO_TYPE_MEAS_INFO:
	default:
		LOGP(DL1C, LOGL_INFO, "Sysinfo (osmo_si %d) not supported.\n",
			osmo_si);
		return -ENOTSUP;
	}

	LOGP(DL1C, LOGL_INFO, "Providing Sysinfo %s to L1\n",
		name);
// FIXME: prepare bursts and send
// Sylvain: put your code here

	return 0;
}

static int l1if_signal_cbfn(unsigned int subsys, unsigned int signal, void *hdlr_data,
				void *signal_data)
{
	if (subsys == SS_GLOBAL && signal == S_NEW_SYSINFO) {
		struct osmo_signal_new_si *new_si = signal_data;
		struct gsm_bts_trx *trx = new_si->trx;

		return l1if_new_si(trx, new_si->osmo_si);
	}
	return 0;
}

//FIXME: perform reset. the confirm schould trigger the state change
int l1if_reset(struct gsm_bts_trx *trx)
{
	int on = 1;
	if (on) {
		int i;
		/* signal availability */
		oml_mo_state_chg(&trx->mo, NM_OPSTATE_DISABLED, NM_AVSTATE_OK);
		oml_mo_tx_sw_act_rep(&trx->mo);
		oml_mo_state_chg(&trx->bb_transc.mo, -1, NM_AVSTATE_OK);
		oml_mo_tx_sw_act_rep(&trx->bb_transc.mo);

		for (i = 0; i < ARRAY_SIZE(trx->ts); i++)
			oml_mo_state_chg(&trx->ts[i].mo, NM_OPSTATE_DISABLED, NM_AVSTATE_DEPENDENCY);
	} else {
		oml_mo_state_chg(&trx->mo, NM_OPSTATE_DISABLED, NM_AVSTATE_OFF_LINE);
		oml_mo_state_chg(&trx->bb_transc.mo, NM_OPSTATE_DISABLED, NM_AVSTATE_OFF_LINE);
	}
}

int l1if_open(void)
{
	LOGP(DL1C, LOGL_INFO, "Open connection to L1.\n");
	osmo_signal_register_handler(SS_GLOBAL, l1if_signal_cbfn, NULL);
// Sylvain: put your code here
// open all l1 stuff, but do not send a reset. it will be done by main.c
	return 0;
}

int l1if_close(void)
{
	osmo_signal_unregister_handler(SS_GLOBAL, l1if_signal_cbfn, NULL);
	return 0;
}
