/* sysmobts - access to hardware related parameters */

/* (C) 2012 by Harald Welte <laforge@gnumonks.org>
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
#include <sys/types.h>
#include <sys/stat.h>

#include "sysmobts_eeprom.h"
#include "sysmobts_par.h"

#define EEPROM_PATH	"/sys/devices/platform/i2c_davinci.1/i2c-1/1-0050/eeprom"


static struct {
	int read;
	struct sysmobts_eeprom ee;
} g_ee;

static struct sysmobts_eeprom *get_eeprom(int update_rqd)
{
	if (update_rqd || g_ee.read == 0) {
		int fd, rc;

		fd = open(EEPROM_PATH, O_RDONLY);
		if (fd < 0)
			return NULL;

		rc = read(fd, &g_ee.ee, sizeof(g_ee.ee));

		close(fd);

		if (rc < sizeof(g_ee.ee))
			return NULL;

		g_ee.read = 1;
	}

	return &g_ee.ee;
}

static int set_eeprom(struct sysmobts_eeprom *ee)
{
	int fd, rc;

	memcpy(&g_ee.ee, ee, sizeof(*ee));

	fd = open(EEPROM_PATH, O_WRONLY);
	if (fd < 0)
		return fd;

	rc = write(fd, ee, sizeof(*ee));
	if (rc < sizeof(*ee)) {
		close(fd);
		return -EIO;
	}

	close(fd);

	return 0;
}

int sysmobts_par_get_int(enum sysmobts_par par, int *ret)
{
	struct sysmobts_eeprom *ee = get_eeprom(0);

	if (!ee)
		return -EIO;

	if (par >= _NUM_SYSMOBTS_PAR)
		return -ENODEV;

	switch (par) {
	case SYSMOBTS_PAR_CLK_FACTORY:
		*ret = ee->clk_cal_fact;
		break;
	case SYSMOBTS_PAR_TEMP_DIG_MAX:
		*ret = ee->temp1_max;
		break;
	case SYSMOBTS_PAR_TEMP_RF_MAX:
		*ret = ee->temp2_max;
		break;
	case SYSMOBTS_PAR_SERNR:
		*ret = ee->serial_nr;
		break;
	case SYSMOBTS_PAR_HOURS:
		*ret = ee->operational_hours;
		break;
	case SYSMOBTS_PAR_BOOTS:
		*ret = ee->boot_count;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int sysmobts_par_set_int(enum sysmobts_par par, int val)
{
	struct sysmobts_eeprom *ee = get_eeprom(1);

	if (!ee)
		return -EIO;

	if (par >= _NUM_SYSMOBTS_PAR)
		return -ENODEV;

	switch (par) {
	case SYSMOBTS_PAR_CLK_FACTORY:
		ee->clk_cal_fact = val;
		break;
	case SYSMOBTS_PAR_TEMP_DIG_MAX:
		ee->temp1_max = val;
		break;
	case SYSMOBTS_PAR_TEMP_RF_MAX:
		ee->temp2_max = val;
		break;
	case SYSMOBTS_PAR_SERNR:
		ee->serial_nr = val;
		break;
	case SYSMOBTS_PAR_HOURS:
		ee->operational_hours = val;
		break;
	case SYSMOBTS_PAR_BOOTS:
		ee->boot_count = val;
		break;
	default:
		return -EINVAL;
	}

	set_eeprom(ee);

	return 0;
}

int sysmobts_par_get_buf(enum sysmobts_par par, uint8_t *buf,
			 unsigned int size)
{
	uint8_t *ptr;
	unsigned int len;
	struct sysmobts_eeprom *ee = get_eeprom(0);

	if (!ee)
		return -EIO;

	if (par >= _NUM_SYSMOBTS_PAR)
		return -ENODEV;

	switch (par) {
	case SYSMOBTS_PAR_MAC:
		ptr = ee->eth_mac;
		len = sizeof(ee->eth_mac);
		break;
	case SYSMOBTS_PAR_KEY:
		ptr = ee->gpg_key;
		len = sizeof(ee->gpg_key);
		break;
	default:
		return -EINVAL;
	}

	if (size < len)
		len = size;
	memcpy(buf, ptr, len);

	return len;
}

int sysmobts_par_set_buf(enum sysmobts_par par, const uint8_t *buf,
			 unsigned int size)
{
	uint8_t *ptr;
	unsigned int len;
	struct sysmobts_eeprom *ee = get_eeprom(0);

	if (!ee)
		return -EIO;

	if (par >= _NUM_SYSMOBTS_PAR)
		return -ENODEV;

	switch (par) {
	case SYSMOBTS_PAR_MAC:
		ptr = ee->eth_mac;
		len = sizeof(ee->eth_mac);
		break;
	case SYSMOBTS_PAR_KEY:
		ptr = ee->gpg_key;
		len = sizeof(ee->gpg_key);
		break;
	default:
		return -EINVAL;
	}

	if (len < size)
		size = len;

	memcpy(ptr, buf, size);

	return len;
}
