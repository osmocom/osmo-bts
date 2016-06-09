/* Copyright (C) 2015 by Yves Godin <support@nuranwireless.com>
 * 
 * Based on sysmoBTS:
 *     sysmobts_misc.c
 *     (C) 2012 by Harald Welte <laforge@gnumonks.org>
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
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <getopt.h>
#include <fcntl.h>
#include <limits.h>
#include <time.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <osmocom/core/talloc.h>
#include <osmocom/core/utils.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/application.h>
#include <osmocom/vty/telnet_interface.h>
#include <osmocom/vty/logging.h>

#include "btsconfig.h"
#include "lc15bts_misc.h"
#include "lc15bts_par.h"
#include "lc15bts_mgr.h"
#include "lc15bts_temp.h"

/*********************************************************************
 * Temperature handling
 *********************************************************************/

static const struct {
	const char *name;
	int has_max;
	enum lc15bts_temp_sensor sensor;
	enum lc15bts_par ee_par;
} temp_data[] = {
	{
		.name = "supply",
		.has_max = 1,
		.sensor = LC15BTS_TEMP_SUPPLY,
		.ee_par = LC15BTS_PAR_TEMP_SUPPLY_MAX,
	}, {
		.name = "soc",
		.has_max = 0,
		.sensor = LC15BTS_TEMP_SOC,
		.ee_par = LC15BTS_PAR_TEMP_SOC_MAX,
	}, {
		.name = "fpga",
		.has_max = 0,
		.sensor = LC15BTS_TEMP_FPGA,
		.ee_par = LC15BTS_PAR_TEMP_FPGA_MAX,

	}, {
		.name = "logrf",
		.has_max = 1,
		.sensor = LC15BTS_TEMP_LOGRF,
		.ee_par = LC15BTS_PAR_TEMP_LOGRF_MAX,
	}, {
		.name = "ocxo",
		.has_max = 1,
		.sensor = LC15BTS_TEMP_OCXO,
		.ee_par = LC15BTS_PAR_TEMP_OCXO_MAX,
	}, {
		.name = "tx0",
		.has_max = 0,
		.sensor = LC15BTS_TEMP_TX0,
		.ee_par = LC15BTS_PAR_TEMP_TX0_MAX,
	}, {
		.name = "tx1",
		.has_max = 0,
		.sensor = LC15BTS_TEMP_TX1,
		.ee_par = LC15BTS_PAR_TEMP_TX1_MAX,
	}, {
		.name = "pa0",
		.has_max = 1,
		.sensor = LC15BTS_TEMP_PA0,
		.ee_par = LC15BTS_PAR_TEMP_PA0_MAX,
	}, {
		.name = "pa1",
		.has_max = 1,
		.sensor = LC15BTS_TEMP_PA1,
		.ee_par = LC15BTS_PAR_TEMP_PA1_MAX,
	}
};

void lc15bts_check_temp(int no_rom_write)
{
	int temp_old[ARRAY_SIZE(temp_data)];
	int temp_cur[ARRAY_SIZE(temp_data)];
	int i, rc;

	for (i = 0; i < ARRAY_SIZE(temp_data); i++) {
		int ret;
		rc = lc15bts_par_get_int(temp_data[i].ee_par, &ret);
		temp_old[i] = ret * 1000;

		temp_cur[i] = lc15bts_temp_get(temp_data[i].sensor);
		if (temp_cur[i] < 0 && temp_cur[i] > -1000) {
			LOGP(DTEMP, LOGL_ERROR, "Error reading temperature (%d)\n", temp_data[i].sensor);
			continue;
		}
	
		LOGP(DTEMP, LOGL_DEBUG, "Current %s temperature: %d.%d C\n",
		     temp_data[i].name, temp_cur[i]/1000, temp_cur[i]%1000);

		if (temp_cur[i] > temp_old[i]) {
			LOGP(DTEMP, LOGL_NOTICE, "New maximum %s "
			     "temperature: %d.%d C\n", temp_data[i].name,
			     temp_cur[i]/1000, temp_old[i]%1000);

			if (!no_rom_write) {
				rc = lc15bts_par_set_int(temp_data[i].ee_par,
						  temp_cur[i]/1000);
				if (rc < 0)
					LOGP(DTEMP, LOGL_ERROR, "error writing new %s "
					     "max temp %d (%s)\n", temp_data[i].name,
					     rc, strerror(errno));
			}
		}
	}
}

/*********************************************************************
 * Hours handling
 *********************************************************************/
static time_t last_update;

int lc15bts_update_hours(int no_rom_write)
{
	time_t now = time(NULL);
	int rc, op_hrs;

	/* first time after start of manager program */
	if (last_update == 0) {
		last_update = now;

		rc = lc15bts_par_get_int(LC15BTS_PAR_HOURS, &op_hrs);
		if (rc < 0) {
			LOGP(DTEMP, LOGL_ERROR, "Unable to read "
			     "operational hours: %d (%s)\n", rc,
			     strerror(errno));
			return rc;
		}

		LOGP(DTEMP, LOGL_INFO, "Total hours of Operation: %u\n",
		     op_hrs);

		return 0;
	}

	if (now >= last_update + 3600) {
		rc = lc15bts_par_get_int(LC15BTS_PAR_HOURS, &op_hrs);
		if (rc < 0) {
			LOGP(DTEMP, LOGL_ERROR, "Unable to read "
			     "operational hours: %d (%s)\n", rc,
			     strerror(errno));
			return rc;
		}

		/* number of hours to increase */
		op_hrs += (now-last_update)/3600;

		LOGP(DTEMP, LOGL_INFO, "Total hours of Operation: %u\n",
		     op_hrs);

		if (!no_rom_write) {
			rc = lc15bts_par_set_int(LC15BTS_PAR_HOURS, op_hrs);
			if (rc < 0)
				return rc;
		}

		last_update = now;
	}

	return 0;
}

/*********************************************************************
 * Firmware reloading
 *********************************************************************/

static const char *fw_sysfs[_NUM_FW] = {
	[LC15BTS_FW_DSP0]	= "/sys/kernel/debug/remoteproc/remoteproc0/recovery",
	[LC15BTS_FW_DSP1]	= "/sys/kernel/debug/remoteproc/remoteproc0/recovery",
};

int lc15bts_firmware_reload(enum lc15bts_firmware_type type)
{
	int fd;
	int rc;

	switch (type) {
	case LC15BTS_FW_DSP0:
	case LC15BTS_FW_DSP1: 
		fd = open(fw_sysfs[type], O_WRONLY);
	        if (fd < 0) {
	                LOGP(DFW, LOGL_ERROR, "unable ot open firmware device %s: %s\n",
                    	fw_sysfs[type], strerror(errno));
	                close(fd);
        	        return fd;
		}
		rc = write(fd, "restart", 8); 
		if (rc < 8) {
                        LOGP(DFW, LOGL_ERROR, "short write during "
                             "fw write to %s\n", fw_sysfs[type]);
                        close(fd);
                        return -EIO;
                }
		close(fd);
	default: 
		return -EINVAL;
	}
	return 0;
}
