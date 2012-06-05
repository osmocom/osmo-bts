/* VTY interface for sysmoBTS */

/* (C) 2011 by Harald Welte <laforge@gnumonks.org>
 * (C) 2012 by Holger Hans Peter Freyther
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

#include <osmo-bts/gsm_data.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/vty.h>

#include "femtobts.h"
#include "l1_if.h"

#define TRX_STR "Transceiver related commands\n" "TRX number\n"

#define SHOW_TRX_STR				\
	SHOW_STR				\
	TRX_STR

static struct gsm_bts *vty_bts;

/* This generates the logging command string for VTY. */
const char *vty_cmd_string_from_valstr(const struct value_string *vals,
					const char *prefix)
{
	int len = 0, offset = 0, ret, rem;
	int size = strlen(prefix);
	const struct value_string *vs;
	char *str;

	for (vs = vals; vs->value || vs->str; vs++)
		size += strlen(vs->str) + 1;

	rem = size;
	str = talloc_zero_size(vty_bts, size);
	if (!str)
		return NULL;

	ret = snprintf(str + offset, rem, prefix);
	if (ret < 0)
		goto err;
	OSMO_SNPRINTF_RET(ret, rem, offset, len);

	for (vs = vals; vs->value || vs->str; vs++) {
		if (vs->str) {
			int j, name_len = strlen(vs->str)+1;
			char name[name_len];

			for (j = 0; j < name_len; j++)
				name[j] = tolower(vs->str[j]);

			name[name_len-1] = '\0';
			ret = snprintf(str + offset, rem, "%s|", name);
			if (ret < 0)
				goto err;
			OSMO_SNPRINTF_RET(ret, rem, offset, len);
		}
	}
	offset--;	/* to remove the trailing | */
	rem++;

	ret = snprintf(str + offset, rem, ")");
	if (ret < 0)
		goto err;
	OSMO_SNPRINTF_RET(ret, rem, offset, len);
err:
	str[size-1] = '\0';
	return str;
}

/* configuration */

DEFUN(cfg_trx_gsmtap_sapi, cfg_trx_gsmtap_sapi_cmd,
	"HIDDEN", "HIDDEN")
{
	struct gsm_bts_trx *trx = vty->index;
	struct femtol1_hdl *fl1h = trx_femtol1_hdl(trx);
	int sapi;

	sapi = get_string_value(femtobts_tracef_names, argv[0]);

	fl1h->gsmtap_sapi_mask |= (1 << sapi);

	return CMD_SUCCESS;
}

DEFUN(cfg_trx_no_gsmtap_sapi, cfg_trx_no_gsmtap_sapi_cmd,
	"HIDDEN", "HIDDEN")
{
	struct gsm_bts_trx *trx = vty->index;
	struct femtol1_hdl *fl1h = trx_femtol1_hdl(trx);
	int sapi;

	sapi = get_string_value(femtobts_tracef_names, argv[1]);

	fl1h->gsmtap_sapi_mask &= ~(1 << sapi);

	return CMD_SUCCESS;
}

DEFUN(cfg_trx_clkcal_def, cfg_trx_clkcal_def_cmd,
	"clock-calibration default",
	"Set the clock calibration value\n" "Default Clock DAC value\n")
{
	struct gsm_bts_trx *trx = vty->index;
	struct femtol1_hdl *fl1h = trx_femtol1_hdl(trx);

	fl1h->clk_cal = 0xffff;

	return CMD_SUCCESS;
}

#ifdef HW_SYSMOBTS_V1
DEFUN(cfg_trx_clkcal, cfg_trx_clkcal_cmd,
	"clock-calibration <0-4095>",
	"Set the clock calibration value\n" "Clock DAC value\n")
{
	unsigned int clkcal = atoi(argv[0]);
	struct gsm_bts_trx *trx = vty->index;
	struct femtol1_hdl *fl1h = trx_femtol1_hdl(trx);

	fl1h->clk_cal = clkcal & 0xfff;

	return CMD_SUCCESS;
}
#else
DEFUN(cfg_trx_clkcal, cfg_trx_clkcal_cmd,
	"clock-calibration <-4095-4095>",
	"Set the clock calibration value\n" "Offset in PPB\n")
{
	int clkcal = atoi(argv[0]);
	struct gsm_bts_trx *trx = vty->index;
	struct femtol1_hdl *fl1h = trx_femtol1_hdl(trx);

	fl1h->clk_cal = clkcal;

	return CMD_SUCCESS;
}
#endif

DEFUN(cfg_trx_clksrc, cfg_trx_clksrc_cmd,
	"clock-source (tcxo|ocxo|ext|gps)",
	"Set the clock source value\n"
	"Use the TCXO\n"
	"Use the OCXO\n"
	"Use an external clock\n"
	"Use the GPS pps\n")
{
	struct gsm_bts_trx *trx = vty->index;
	struct femtol1_hdl *fl1h = trx_femtol1_hdl(trx);
	int rc;

	rc = get_string_value(femtobts_clksrc_names, argv[0]);
	if (rc < 0)
		return CMD_WARNING;

	fl1h->clk_src = rc;

	return CMD_SUCCESS;
}

/* runtime */

DEFUN(show_trx_clksrc, show_trx_clksrc_cmd,
	"show trx <0-0> clock-source",
	SHOW_TRX_STR "Display the clock source for this TRX")
{
	int trx_nr = atoi(argv[0]);
	struct gsm_bts_trx *trx = gsm_bts_trx_num(vty_bts, trx_nr);
	struct femtol1_hdl *fl1h;

	if (!trx)
		return CMD_WARNING;

	fl1h = trx_femtol1_hdl(trx);

	vty_out(vty, "TRX Clock Source: %s%s",
		get_value_string(femtobts_clksrc_names, fl1h->clk_src),
		VTY_NEWLINE);

	return CMD_SUCCESS;
}

DEFUN(show_dsp_trace_f, show_dsp_trace_f_cmd,
	"show trx <0-0> dsp-trace-flags",
	SHOW_TRX_STR "Display the current setting of the DSP trace flags")
{
	int trx_nr = atoi(argv[0]);
	struct gsm_bts_trx *trx = gsm_bts_trx_num(vty_bts, trx_nr);
	struct femtol1_hdl *fl1h;
	int i;

	if (!trx)
		return CMD_WARNING;

	fl1h = trx_femtol1_hdl(trx);

	vty_out(vty, "Femto L1 DSP trace flags:%s", VTY_NEWLINE);
	for (i = 0; i < ARRAY_SIZE(femtobts_tracef_names); i++) {
		const char *endis;

		if (femtobts_tracef_names[i].value == 0 &&
		    femtobts_tracef_names[i].str == NULL)
			break;

		if (fl1h->dsp_trace_f & femtobts_tracef_names[i].value)
			endis = "enabled";
		else
			endis = "disabled";

		vty_out(vty, "DSP Trace %-15s %s%s",
			femtobts_tracef_names[i].str, endis,
			VTY_NEWLINE);
	}

	return CMD_SUCCESS;

}

DEFUN(dsp_trace_f, dsp_trace_f_cmd, "HIDDEN", TRX_STR)
{
	int trx_nr = atoi(argv[0]);
	struct gsm_bts_trx *trx = gsm_bts_trx_num(vty_bts, trx_nr);
	struct femtol1_hdl *fl1h;
	unsigned int flag ;

	if (!trx) {
		vty_out(vty, "Cannot find TRX number %u%s",
			trx_nr, VTY_NEWLINE);
		return CMD_WARNING;
	}

	fl1h = trx_femtol1_hdl(trx);
	flag = get_string_value(femtobts_tracef_names, argv[1]);
	l1if_set_trace_flags(fl1h, fl1h->dsp_trace_f | flag);

	return CMD_SUCCESS;
}

DEFUN(no_dsp_trace_f, no_dsp_trace_f_cmd, "HIDDEN", NO_STR TRX_STR)
{
	int trx_nr = atoi(argv[0]);
	struct gsm_bts_trx *trx = gsm_bts_trx_num(vty_bts, trx_nr);
	struct femtol1_hdl *fl1h;
	unsigned int flag ;

	if (!trx) {
		vty_out(vty, "Cannot find TRX number %u%s",
			trx_nr, VTY_NEWLINE);
		return CMD_WARNING;
	}

	fl1h = trx_femtol1_hdl(trx);
	flag = get_string_value(femtobts_tracef_names, argv[1]);
	l1if_set_trace_flags(fl1h, fl1h->dsp_trace_f & ~flag);

	return CMD_SUCCESS;
}

DEFUN(show_sys_info, show_sys_info_cmd,
	"show trx <0-0> system-information",
	SHOW_TRX_STR "Display information about system\n")
{
	int trx_nr = atoi(argv[0]);
	struct gsm_bts_trx *trx = gsm_bts_trx_num(vty_bts, trx_nr);
	struct femtol1_hdl *fl1h;
	int i;

	if (!trx) {
		vty_out(vty, "Cannot find TRX number %u%s",
			trx_nr, VTY_NEWLINE);
		return CMD_WARNING;
	}
	fl1h = trx_femtol1_hdl(trx);

	vty_out(vty, "DSP Version: %u.%u.%u, FPGA Version: %u.%u.%u%s",
		fl1h->hw_info.dsp_version[0],
		fl1h->hw_info.dsp_version[1],
		fl1h->hw_info.dsp_version[2],
		fl1h->hw_info.fpga_version[0],
		fl1h->hw_info.fpga_version[1],
		fl1h->hw_info.fpga_version[2], VTY_NEWLINE);

	vty_out(vty, "GSM Band Support: ");
	for (i = 0; i < sizeof(fl1h->hw_info.band_support); i++) {
		if (fl1h->hw_info.band_support & (1 << i))
			vty_out(vty, "%s ",  gsm_band_name(1 << i));
	}
	vty_out(vty, "%s", VTY_NEWLINE);

	return CMD_SUCCESS;
}

DEFUN(activate_lchan, activate_lchan_cmd,
	"trx <0-0> <0-7> (activate|deactivate) <0-7>",
	TRX_STR
	"Timeslot number\n"
	"Activate Logical Channel\n"
	"Deactivate Logical Channel\n"
	"Logical Channel Number\n" )
{
	int trx_nr = atoi(argv[0]);
	int ts_nr = atoi(argv[1]);
	int lchan_nr = atoi(argv[3]);
	struct gsm_bts_trx *trx = gsm_bts_trx_num(vty_bts, trx_nr);
	struct gsm_bts_trx_ts *ts = &trx->ts[ts_nr];
	struct gsm_lchan *lchan = &ts->lchan[lchan_nr];

	if (!strcmp(argv[2], "activate"))
		lchan_activate(lchan);
	else
		lchan_deactivate(lchan);

	return CMD_SUCCESS;
}

DEFUN(set_tx_power, set_tx_power_cmd,
	"trx <0-0> tx-power <-110-23>",
	TRX_STR
	"Set transmit power (override BSC)\n"
	"Transmit power in dBm\n")
{
	int trx_nr = atoi(argv[0]);
	int power = atoi(argv[1]);
	struct gsm_bts_trx *trx = gsm_bts_trx_num(vty_bts, trx_nr);
	struct femtol1_hdl *fl1h = trx_femtol1_hdl(trx);

	l1if_set_txpower(fl1h, (float) power);

	return CMD_SUCCESS;
}

DEFUN(loopback, loopback_cmd,
	"trx <0-0> <0-7> loopback <0-1>",
	TRX_STR
	"Timeslot number\n"
	"Set TCH loopback\n"
	"Logical Channel Number\n")
{
	int trx_nr = atoi(argv[0]);
	int ts_nr = atoi(argv[1]);
	int lchan_nr = atoi(argv[2]);
	struct gsm_bts_trx *trx = gsm_bts_trx_num(vty_bts, trx_nr);
	struct gsm_bts_trx_ts *ts = &trx->ts[ts_nr];
	struct gsm_lchan *lchan = &ts->lchan[lchan_nr];

	lchan->loopback = 1;

	return CMD_SUCCESS;
}

DEFUN(no_loopback, no_loopback_cmd,
	"no trx <0-0> <0-7> loopback <0-1>",
	NO_STR TRX_STR
	"Timeslot number\n"
	"Set TCH loopback\n"
	"Logical Channel Number\n")
{
	int trx_nr = atoi(argv[0]);
	int ts_nr = atoi(argv[1]);
	int lchan_nr = atoi(argv[2]);
	struct gsm_bts_trx *trx = gsm_bts_trx_num(vty_bts, trx_nr);
	struct gsm_bts_trx_ts *ts = &trx->ts[ts_nr];
	struct gsm_lchan *lchan = &ts->lchan[lchan_nr];

	lchan->loopback = 0;

	return CMD_SUCCESS;
}


void bts_model_config_write_bts(struct vty *vty, struct gsm_bts *bts)
{
}

void bts_model_config_write_trx(struct vty *vty, struct gsm_bts_trx *trx)
{
	struct femtol1_hdl *fl1h = trx_femtol1_hdl(trx);

	vty_out(vty, "  clock-calibration %d%s", fl1h->clk_cal,
			VTY_NEWLINE);
	vty_out(vty, "  clock-source %s%s",
		get_value_string(femtobts_clksrc_names, fl1h->clk_src),
		VTY_NEWLINE);
}

int bts_model_vty_init(struct gsm_bts *bts)
{
	vty_bts = bts;

	/* runtime-patch the command strings with debug levels */
	dsp_trace_f_cmd.string = vty_cmd_string_from_valstr(femtobts_tracef_names,
						"trx <0-0> dsp-trace-flag (");
	no_dsp_trace_f_cmd.string = vty_cmd_string_from_valstr(femtobts_tracef_names,
						"no trx <0-0> dsp-trace-flag (");
	cfg_trx_gsmtap_sapi_cmd.string = vty_cmd_string_from_valstr(femtobts_l1sapi_names,
						"gsmtap-sapi (");
	cfg_trx_no_gsmtap_sapi_cmd.string = vty_cmd_string_from_valstr(femtobts_l1sapi_names,
						"no gsmtap-sapi (");

	install_element_ve(&show_dsp_trace_f_cmd);
	install_element_ve(&show_sys_info_cmd);
	install_element_ve(&show_trx_clksrc_cmd);
	install_element_ve(&dsp_trace_f_cmd);
	install_element_ve(&no_dsp_trace_f_cmd);

	install_element(ENABLE_NODE, &activate_lchan_cmd);
	install_element(ENABLE_NODE, &set_tx_power_cmd);

	install_element(ENABLE_NODE, &loopback_cmd);
	install_element(ENABLE_NODE, &no_loopback_cmd);

	install_element(TRX_NODE, &cfg_trx_clkcal_cmd);
	install_element(TRX_NODE, &cfg_trx_clkcal_def_cmd);
	install_element(TRX_NODE, &cfg_trx_clksrc_cmd);
	install_element(TRX_NODE, &cfg_trx_gsmtap_sapi_cmd);
	install_element(TRX_NODE, &cfg_trx_no_gsmtap_sapi_cmd);

	return 0;
}
