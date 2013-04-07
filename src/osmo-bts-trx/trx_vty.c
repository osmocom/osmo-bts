/* VTY interface for sysmoBTS */

/* (C) 2013 by Andreas Eversberg <jolly@eversberg.eu>
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
#include <osmocom/core/bits.h>

#include <osmocom/vty/vty.h>
#include <osmocom/vty/command.h>
#include <osmocom/vty/misc.h>

#include <osmo-bts/gsm_data.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/vty.h>

#include "l1_if.h"
#include "scheduler.h"
#include "trx_if.h"
#include "loops.h"

static struct gsm_bts *vty_bts;

DEFUN(show_tranceiver, show_tranceiver_cmd, "show tranceiver",
	SHOW_STR "Display information about tranceivers\n")
{
	struct gsm_bts *bts = vty_bts;
	struct gsm_bts_trx *trx;
	struct trx_l1h *l1h;
	uint8_t tn;

	if (!tranceiver_available) {
		vty_out(vty, "Tranceiver is not connected%s", VTY_NEWLINE);
	} else {
		vty_out(vty, "Tranceiver is connected, current fn=%u%s",
			tranceiver_last_fn, VTY_NEWLINE);
	}

	llist_for_each_entry(trx, &bts->trx_list, list) {
		l1h = trx_l1h_hdl(trx);
		vty_out(vty, "TRX %d%s", trx->nr, VTY_NEWLINE);
		vty_out(vty, " %s%s",
			(l1h->config.poweron) ? "poweron":"poweroff",
			VTY_NEWLINE);
		if (l1h->config.arfcn_valid)
			vty_out(vty, " arfcn  : %d%s%s",
				(l1h->config.arfcn & ~ARFCN_PCS),
				(l1h->config.arfcn & ARFCN_PCS) ? " (PCS)" : "",
				VTY_NEWLINE);
		else
			vty_out(vty, " arfcn  : undefined%s", VTY_NEWLINE);
		if (l1h->config.tsc_valid)
			vty_out(vty, " tsc    : %d%s", l1h->config.tsc,
				VTY_NEWLINE);
		else
			vty_out(vty, " tsc    : undefined%s", VTY_NEWLINE);
		if (l1h->config.bsic_valid)
			vty_out(vty, " bsic   : %d%s", l1h->config.bsic,
				VTY_NEWLINE);
		else
			vty_out(vty, " bisc   : undefined%s", VTY_NEWLINE);
		if (l1h->config.rxgain_valid)
			vty_out(vty, " rxgain : %d%s", l1h->config.rxgain,
				VTY_NEWLINE);
		else
			vty_out(vty, " rxgain : undefined%s", VTY_NEWLINE);
		if (l1h->config.power_valid)
			vty_out(vty, " power  : %d%s", l1h->config.power,
				VTY_NEWLINE);
		else
			vty_out(vty, " power  : undefined%s", VTY_NEWLINE);
		if (l1h->config.maxdly_valid)
			vty_out(vty, " maxdly : %d%s", l1h->config.maxdly,
				VTY_NEWLINE);
		else
			vty_out(vty, " maxdly : undefined%s", VTY_NEWLINE);
		for (tn = 0; tn < 8; tn++) {
			if (!((1 << tn) & l1h->config.slotmask))
				vty_out(vty, " slot #%d: unsupported%s", tn,
					VTY_NEWLINE);
			else if (l1h->config.slottype_valid[tn])
				vty_out(vty, " slot #%d: type %d%s", tn,
					l1h->config.slottype[tn],
					VTY_NEWLINE);
			else
				vty_out(vty, " slot #%d: undefined%s", tn,
					VTY_NEWLINE);
		}
	}

	return CMD_SUCCESS;
}

DEFUN(cfg_bts_fn_advance, cfg_bts_fn_advance_cmd,
	"fn-advance <0-30>",
	"Set the number of frames to be transmitted in advance of current FN\n"
	"Advance in frames\n")
{
	trx_clock_advance = atoi(argv[0]);

	return CMD_SUCCESS;
}

DEFUN(cfg_bts_ms_power_loop, cfg_bts_ms_power_loop_cmd,
	"ms-power-loop <-127-127>",
	"Enable MS power control loop\nTarget RSSI value (tranceiver specific, "
	"should be 6dB or more above noise floor)\n")
{
	trx_ms_power_loop = 1;
	trx_target_rssi = atoi(argv[0]);

	return CMD_SUCCESS;
}

DEFUN(cfg_bts_no_ms_power_loop, cfg_bts_no_ms_power_loop_cmd,
	"no ms-power-loop",
	NO_STR "Disable MS power control loop\n")
{
	trx_ms_power_loop = 0;

	return CMD_SUCCESS;
}

DEFUN(cfg_bts_timing_advance_loop, cfg_bts_timing_advance_loop_cmd,
	"timing-advance-loop",
	"Enable timing advance control loop\n")
{
	trx_ta_loop = 1;

	return CMD_SUCCESS;
}
DEFUN(cfg_bts_no_timing_advance_loop, cfg_bts_no_timing_advance_loop_cmd,
	"no timing-advance-loop",
	NO_STR "Disable timing advance control loop\n")
{
	trx_ta_loop = 0;

	return CMD_SUCCESS;
}

DEFUN(cfg_bts_settsc, cfg_bts_settsc_cmd,
	"settsc",
	"Use SETTSC to configure transceiver\n")
{
	settsc_enabled = 1;

	return CMD_SUCCESS;
}

DEFUN(cfg_bts_setbsic, cfg_bts_setbsic_cmd,
	"setbsic",
	"Use SETBSIC to configure transceiver\n")
{
	setbsic_enabled = 1;

	return CMD_SUCCESS;
}

DEFUN(cfg_bts_no_settsc, cfg_bts_no_settsc_cmd,
	"no settsc",
	NO_STR "Disable SETTSC to configure transceiver\n")
{
	settsc_enabled = 0;
	if (!setbsic_enabled) {
		vty_out(vty, "%% Auto enabling SETBSIC.%s", VTY_NEWLINE);
		setbsic_enabled = 1;
	}

	return CMD_SUCCESS;
}

DEFUN(cfg_bts_no_setbsic, cfg_bts_no_setbsic_cmd,
	"no setbsic",
	NO_STR "Disable SETBSIC to configure transceiver\n")
{
	setbsic_enabled = 0;
	if (!settsc_enabled) {
		vty_out(vty, "%% Auto enabling SETTSC.%s", VTY_NEWLINE);
		settsc_enabled = 1;
	}

	return CMD_SUCCESS;
}

DEFUN(cfg_trx_rxgain, cfg_trx_rxgain_cmd,
	"rxgain <0-50>",
	"Set the receiver gain in dB\n"
	"Gain in dB\n")
{
	struct gsm_bts_trx *trx = vty->index;
	struct trx_l1h *l1h = trx_l1h_hdl(trx);

	l1h->config.rxgain = atoi(argv[0]);
	l1h->config.rxgain_valid = 1;
	l1h->config.rxgain_sent = 0;
	l1if_provision_tranceiver_trx(l1h);

	return CMD_SUCCESS;
}

DEFUN(cfg_trx_power, cfg_trx_power_cmd,
	"power <0-50>",
	"Set the transmitter power dampening\n"
	"Power dampening in dB\n")
{
	struct gsm_bts_trx *trx = vty->index;
	struct trx_l1h *l1h = trx_l1h_hdl(trx);

	l1h->config.power = atoi(argv[0]);
	l1h->config.power_valid = 1;
	l1h->config.power_sent = 0;
	l1if_provision_tranceiver_trx(l1h);

	return CMD_SUCCESS;
}

DEFUN(cfg_trx_maxdly, cfg_trx_maxdly_cmd,
	"maxdly <0-31>",
	"Set the maximum delay of GSM symbols\n"
	"GSM symbols (approx. 1.1km per symbol)\n")
{
	struct gsm_bts_trx *trx = vty->index;
	struct trx_l1h *l1h = trx_l1h_hdl(trx);

	l1h->config.maxdly = atoi(argv[0]);
	l1h->config.maxdly_valid = 1;
	l1h->config.maxdly_sent = 0;
	l1if_provision_tranceiver_trx(l1h);

	return CMD_SUCCESS;
}

DEFUN(cfg_trx_slotmask, cfg_trx_slotmask_cmd,
	"slotmask (1|0) (1|0) (1|0) (1|0) (1|0) (1|0) (1|0) (1|0)",
	"Set the supported slots\n"
	"TS0 supported\nTS0 unsupported\nTS1 supported\nTS1 unsupported\n"
	"TS2 supported\nTS2 unsupported\nTS3 supported\nTS3 unsupported\n"
	"TS4 supported\nTS4 unsupported\nTS5 supported\nTS5 unsupported\n"
	"TS6 supported\nTS6 unsupported\nTS7 supported\nTS7 unsupported\n")
{
	struct gsm_bts_trx *trx = vty->index;
	struct trx_l1h *l1h = trx_l1h_hdl(trx);
	uint8_t tn;

	l1h->config.slotmask = 0;
	for (tn = 0; tn < 8; tn++)
		if (argv[tn][0] == '1')
			l1h->config.slotmask |= (1 << tn);

	return CMD_SUCCESS;
}

DEFUN(cfg_trx_no_rxgain, cfg_trx_no_rxgain_cmd,
	"no rxgain <0-50>",
	NO_STR "Unset the receiver gain in dB\n"
	"Gain in dB\n")
{
	struct gsm_bts_trx *trx = vty->index;
	struct trx_l1h *l1h = trx_l1h_hdl(trx);

	l1h->config.rxgain_valid = 0;

	return CMD_SUCCESS;
}

DEFUN(cfg_trx_no_power, cfg_trx_no_power_cmd,
	"no power <0-50>",
	NO_STR "Unset the transmitter power dampening\n"
	"Power dampening in dB\n")
{
	struct gsm_bts_trx *trx = vty->index;
	struct trx_l1h *l1h = trx_l1h_hdl(trx);

	l1h->config.power_valid = 0;

	return CMD_SUCCESS;
}

DEFUN(cfg_trx_no_maxdly, cfg_trx_no_maxdly_cmd,
	"no maxdly <0-31>",
	NO_STR "Unset the maximum delay of GSM symbols\n"
	"GSM symbols (approx. 1.1km per symbol)\n")
{
	struct gsm_bts_trx *trx = vty->index;
	struct trx_l1h *l1h = trx_l1h_hdl(trx);

	l1h->config.maxdly_valid = 0;

	return CMD_SUCCESS;
}

void bts_model_config_write_bts(struct vty *vty, struct gsm_bts *bts)
{
	vty_out(vty, " fn-advance %d%s", trx_clock_advance, VTY_NEWLINE);

	if (trx_ms_power_loop)
		vty_out(vty, " ms-power-loop %d%s", trx_target_rssi,
			VTY_NEWLINE);
	else
		vty_out(vty, " no ms-power-loop%s", VTY_NEWLINE);
	vty_out(vty, " %stiming-advance-loop%s", (trx_ta_loop) ? "":"no ",
		VTY_NEWLINE);
	if (settsc_enabled)
		vty_out(vty, " settsc%s", VTY_NEWLINE);
	if (setbsic_enabled)
		vty_out(vty, " setbsic%s", VTY_NEWLINE);
}

void bts_model_config_write_trx(struct vty *vty, struct gsm_bts_trx *trx)
{
	struct trx_l1h *l1h = trx_l1h_hdl(trx);

	if (l1h->config.rxgain_valid)
		vty_out(vty, "  rxgain %d%s", l1h->config.rxgain, VTY_NEWLINE);
	if (l1h->config.power_valid)
		vty_out(vty, "  power %d%s", l1h->config.power, VTY_NEWLINE);
	if (l1h->config.maxdly_valid)
		vty_out(vty, "  maxdly %d%s", l1h->config.maxdly, VTY_NEWLINE);
	if (l1h->config.slotmask != 0xff)
		vty_out(vty, "  slotmask %d %d %d %d %d %d %d %d%s",
			l1h->config.slotmask & 1,
			(l1h->config.slotmask >> 1) & 1,
			(l1h->config.slotmask >> 2) & 1,
			(l1h->config.slotmask >> 3) & 1,
			(l1h->config.slotmask >> 4) & 1,
			(l1h->config.slotmask >> 5) & 1,
			(l1h->config.slotmask >> 6) & 1,
			l1h->config.slotmask >> 7,
			VTY_NEWLINE);
}

int bts_model_vty_init(struct gsm_bts *bts)
{
	vty_bts = bts;

	install_element_ve(&show_tranceiver_cmd);

	install_element(BTS_NODE, &cfg_bts_fn_advance_cmd);
	install_element(BTS_NODE, &cfg_bts_ms_power_loop_cmd);
	install_element(BTS_NODE, &cfg_bts_no_ms_power_loop_cmd);
	install_element(BTS_NODE, &cfg_bts_timing_advance_loop_cmd);
	install_element(BTS_NODE, &cfg_bts_no_timing_advance_loop_cmd);
	install_element(BTS_NODE, &cfg_bts_settsc_cmd);
	install_element(BTS_NODE, &cfg_bts_setbsic_cmd);
	install_element(BTS_NODE, &cfg_bts_no_settsc_cmd);
	install_element(BTS_NODE, &cfg_bts_no_setbsic_cmd);

	install_element(TRX_NODE, &cfg_trx_rxgain_cmd);
	install_element(TRX_NODE, &cfg_trx_power_cmd);
	install_element(TRX_NODE, &cfg_trx_maxdly_cmd);
	install_element(TRX_NODE, &cfg_trx_slotmask_cmd);
	install_element(TRX_NODE, &cfg_trx_no_rxgain_cmd);
	install_element(TRX_NODE, &cfg_trx_no_power_cmd);
	install_element(TRX_NODE, &cfg_trx_no_maxdly_cmd);

	return 0;
}
