/* VTY interface for sysmoBTS */

/* (C) 2011 by Harald Welte <laforge@gnumonks.org>
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

#include "femtobts.h"
#include "l1_if.h"

#define SHOW_TRX_STR				\
	SHOW_STR				\
	"Show TRX specific information\n"	\
	"TRX number\n"

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

DEFUN(dsp_trace_f, dsp_trace_f_cmd, "HIDDEN", "HIDDEN")
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

DEFUN(no_dsp_trace_f, no_dsp_trace_f_cmd, "HIDDEN", "HIDDEN")
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
	for (i = 0; i < 32; i++) {
		if (fl1h->hw_info.band_support & (1 << i))
			vty_out(vty, "%s ",  gsm_band_name(1 << i));
	}
	vty_out(vty, "%s", VTY_NEWLINE);

	return CMD_SUCCESS;
}


int bts_model_vty_init(struct gsm_bts *bts)
{
	vty_bts = bts;

	/* runtime-patch the command strings with debug levels */
	dsp_trace_f_cmd.string = vty_cmd_string_from_valstr(femtobts_tracef_names,
						"trx <0-0> dsp-trace-flag (");
	no_dsp_trace_f_cmd.string = vty_cmd_string_from_valstr(femtobts_tracef_names,
						"no trx <0-0> dsp-trace-flag (");

	install_element_ve(&show_dsp_trace_f_cmd);
	install_element_ve(&show_sys_info_cmd);
	install_element_ve(&dsp_trace_f_cmd);
	install_element_ve(&no_dsp_trace_f_cmd);

	return 0;
}
