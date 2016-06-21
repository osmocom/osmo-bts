/* Copyright (C) 2015 by Yves Godin <support@nuranwireless.com>
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
#include <fcntl.h>
#include <limits.h>

#include <osmocom/core/utils.h>

#include "lc15bts_temp.h"


static const char *temp_devs[_NUM_TEMP_SENSORS] = {
        [LC15BTS_TEMP_SUPPLY]	= "/sys/bus/i2c/devices/2-004d/hwmon/hwmon5/temp1_",
        [LC15BTS_TEMP_SOC]	= "/sys/class/hwmon/hwmon1/temp1_",
        [LC15BTS_TEMP_FPGA]	= "/sys/devices/0.iio_hwmon/temp1_",
        [LC15BTS_TEMP_MEMORY]	= "/sys/bus/i2c/devices/2-004c/hwmon/hwmon4/temp1_",
        [LC15BTS_TEMP_TX1]	= "/sys/devices/0.ncp15xh103_tx1/temp1_",
        [LC15BTS_TEMP_TX2]	= "/sys/devices/0.ncp15xh103_tx2/temp1_",
        [LC15BTS_TEMP_PA1]	= "/sys/bus/i2c/devices/2-004d/hwmon/hwmon5/temp2_",
        [LC15BTS_TEMP_PA2]	= "/sys/bus/i2c/devices/2-004c/hwmon/hwmon4/temp2_",
};

static const int temp_has_fault[_NUM_TEMP_SENSORS] = {
        [LC15BTS_TEMP_PA1]	= 1,
        [LC15BTS_TEMP_PA2]	= 1,
};

static const char *temp_type_str[_NUM_TEMP_TYPES] = {
	[LC15BTS_TEMP_INPUT] = "input",
	[LC15BTS_TEMP_LOWEST] = "lowest",
	[LC15BTS_TEMP_HIGHEST] = "highest",
	[LC15BTS_TEMP_FAULT] = "fault",
};

int lc15bts_temp_get(enum lc15bts_temp_sensor sensor,
		      enum lc15bts_temp_type type)
{
	char buf[PATH_MAX];
	char tempstr[8];
	char faultstr[8];
	int fd, rc;

	if (sensor < 0 || sensor >= _NUM_TEMP_SENSORS)
		return -EINVAL;

	if (type >= ARRAY_SIZE(temp_type_str))
		return -EINVAL;

	snprintf(buf, sizeof(buf)-1, "%s%s", temp_devs[sensor], temp_type_str[type]);
	buf[sizeof(buf)-1] = '\0';

	fd = open(buf, O_RDONLY);
	if (fd < 0)
		return fd;

	rc = read(fd, tempstr, sizeof(tempstr));
	tempstr[sizeof(tempstr)-1] = '\0';
	if (rc < 0) {
		close(fd);
		return rc;
	}
	if (rc == 0) {
		close(fd);
		return -EIO;
	}
	close(fd);

	// Check fault
	if (type == LC15BTS_TEMP_FAULT || !temp_has_fault[sensor])
		return atoi(tempstr);
		
        snprintf(buf, sizeof(buf)-1, "%s%s",  temp_devs[sensor], temp_type_str[LC15BTS_TEMP_FAULT]);
        buf[sizeof(buf)-1] = '\0';

        fd = open(buf, O_RDONLY);
        if (fd < 0)
                return fd;

        rc = read(fd, faultstr, sizeof(faultstr));
        tempstr[sizeof(faultstr)-1] = '\0';
        if (rc < 0) {
                close(fd);
                return rc;
        }
        if (rc == 0) {
                close(fd);
                return -EIO;
        }
        close(fd);

	if (atoi(faultstr))
		return -EIO;

	return atoi(tempstr);
}

