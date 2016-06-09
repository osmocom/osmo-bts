/* lc15bts - access to hardware related parameters */

/* Copyright (C) 2015 by Yves Godin <support@nuranwireless.com>
 * 
 * Based on sysmoBTS:
 *     sysmobts_par.c
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

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <limits.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <osmocom/core/utils.h>

#include "lc15bts_par.h"


#define FACTORY_ROM_PATH	"/mnt/rom/factory"
#define USER_ROM_PATH		"/mnt/rom/user"

const struct value_string lc15bts_par_names[_NUM_LC15BTS_PAR+1] = {
	{ LC15BTS_PAR_TEMP_SUPPLY_MAX,	"temp-supply-max" },
	{ LC15BTS_PAR_TEMP_SOC_MAX,	"temp-soc-max" },
	{ LC15BTS_PAR_TEMP_FPGA_MAX,	"temp-fpga-max" },
	{ LC15BTS_PAR_TEMP_LOGRF_MAX,	"temp-logrf-max" },
	{ LC15BTS_PAR_TEMP_OCXO_MAX,	"temp-ocxo-max" },
	{ LC15BTS_PAR_TEMP_TX0_MAX,	"temp-tx0-max" },
	{ LC15BTS_PAR_TEMP_TX1_MAX,	"temp-tx1-max" },
	{ LC15BTS_PAR_TEMP_PA0_MAX,	"temp-pa0-max" },
	{ LC15BTS_PAR_TEMP_PA1_MAX,	"temp-pa1-max" },
	{ LC15BTS_PAR_SERNR,		"serial-nr" },
	{ LC15BTS_PAR_HOURS, 		"hours-running" },
	{ LC15BTS_PAR_BOOTS, 		"boot-count" },
	{ LC15BTS_PAR_KEY, 		"key" },
	{ 0, NULL }
};

int lc15bts_par_is_int(enum lc15bts_par par)
{
	switch (par) {
	case LC15BTS_PAR_TEMP_SUPPLY_MAX:
        case LC15BTS_PAR_TEMP_SOC_MAX:
        case LC15BTS_PAR_TEMP_FPGA_MAX:
        case LC15BTS_PAR_TEMP_LOGRF_MAX:
        case LC15BTS_PAR_TEMP_OCXO_MAX:
        case LC15BTS_PAR_TEMP_TX0_MAX:
        case LC15BTS_PAR_TEMP_TX1_MAX:
        case LC15BTS_PAR_TEMP_PA0_MAX:
        case LC15BTS_PAR_TEMP_PA1_MAX:
	case LC15BTS_PAR_SERNR:
	case LC15BTS_PAR_HOURS:
	case LC15BTS_PAR_BOOTS:
		return 1;
	default:
		return 0;
	}
}

int lc15bts_par_get_int(enum lc15bts_par par, int *ret)
{
	char fpath[PATH_MAX];
	FILE *fp;
	int rc;

	if (par >= _NUM_LC15BTS_PAR)
		return -ENODEV;

	snprintf(fpath, sizeof(fpath)-1, "%s/%s", USER_ROM_PATH, get_value_string(lc15bts_par_names, par));
	fpath[sizeof(fpath)-1] = '\0';

	fp = fopen(fpath, "r");
	if (fp == NULL) {
		return -errno;
	}

	rc = fscanf(fp, "%d", ret);
	if (rc != 1) {
		fclose(fp);
		return -EIO;
	}
	fclose(fp);
	return 0;
}

int lc15bts_par_set_int(enum lc15bts_par par, int val)
{
	char fpath[PATH_MAX];
	FILE *fp;
	int rc;

	if (par >= _NUM_LC15BTS_PAR)
		return -ENODEV;

	snprintf(fpath, sizeof(fpath)-1, "%s/%s", USER_ROM_PATH, get_value_string(lc15bts_par_names, par));
	fpath[sizeof(fpath)-1] = '\0';

	fp = fopen(fpath, "w");
	if (fp == NULL) {
		return -errno;
	}

	rc = fprintf(fp, "%d", val);
	if (rc < 0) {
		fclose(fp);
		return -EIO;
	}
	fclose(fp);
	return 0;
}

int lc15bts_par_get_buf(enum lc15bts_par par, uint8_t *buf,
			 unsigned int size)
{	
	char fpath[PATH_MAX];
	FILE *fp;
	int rc;

	if (par >= _NUM_LC15BTS_PAR)
		return -ENODEV;

	snprintf(fpath, sizeof(fpath)-1, "%s/%s", USER_ROM_PATH, get_value_string(lc15bts_par_names, par));
	fpath[sizeof(fpath)-1] = '\0';

	fp = fopen(fpath, "rb");
	if (fp == NULL) {
		return -errno;
	}

	rc = fread(buf, 1, size, fp);
	
	fclose(fp);
	
	return rc;
}

int lc15bts_par_set_buf(enum lc15bts_par par, const uint8_t *buf,
			 unsigned int size)
{
        char fpath[PATH_MAX];
        FILE *fp;
        int rc;

        if (par >= _NUM_LC15BTS_PAR)
                return -ENODEV;

        snprintf(fpath, sizeof(fpath)-1, "%s/%s", USER_ROM_PATH, get_value_string(lc15bts_par_names, par));
        fpath[sizeof(fpath)-1] = '\0';

        fp = fopen(fpath, "wb");
        if (fp == NULL) {
                return -errno;
	}

        rc = fwrite(buf, 1, size, fp);

        fclose(fp);

        return rc;
}
