/* sx150x - minimal SX150x I2C GPIO expander driver
 * (C) 2015 by sysmocom - s.f.m.c. GmbH, Author: Harald Welte
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <limits.h>
#include <fcntl.h>
#include <getopt.h>

#include <sys/ioctl.h>
#include <sys/stat.h>

/* #include <linux/i2c-dev.h> */
#include "i2c-dev.h"

#include "sx150x.h"

enum sx150x_reg {
	SX150x_REG_DATA		= 0x00,
	SX150x_REG_DIRECTION	= 0x01,
	SX150x_REG_PULL_UP	= 0x02,
	SX150x_REG_PULL_DOWN	= 0x03,
	/* resered */
	SX150x_REG_IRQ_MASK	= 0x05,
	SX150x_REG_SENSE_HIGH	= 0x06,
	SX150x_REG_SENSE_LOW	= 0x07,
	SX150x_REG_IRQ_SOURCE	= 0x08,
	SX150x_REG_EVENT_STATUS	= 0x09,
	/* what about 0x0a - 0x0f? */
	SX150x_REG_PLD_MODE	= 0x10,
	SX150x_REG_PLD_TABLE0	= 0x11,
	SX150x_REG_PLD_TABLE1	= 0x12,
	SX150x_REG_PLD_TABLE2	= 0x13,
	SX150x_REG_PLD_TABLE3	= 0x13,
	SX150x_REG_PLD_TABLE4	= 0x13,
	SX150x_REG_ADVANCED	= 0xAB,
};

int sx150x_gpio_direction_set(int fd, int gpio, enum sx150x_direction dir)
{
	int rc;

	rc = i2c_smbus_read_byte_data(fd, SX150x_REG_DIRECTION);
	if (rc < 0)
		return rc;

	rc &= ~(1 << gpio);
	rc |= (dir << gpio);

	return i2c_smbus_write_byte_data(fd, SX150x_REG_DIRECTION, rc);
}

int sx150x_gpio_direction_get(int fd, int gpio)
{
	int rc;

	rc = i2c_smbus_read_byte_data(fd, SX150x_REG_DIRECTION);
	if (rc < 0)
		return rc;
	return (rc & (1 << gpio));
}

int sx150x_gpio_pull_up_set(int fd, int gpio, int on)
{
	int rc;

	rc = i2c_smbus_read_byte_data(fd, SX150x_REG_PULL_UP);
	if (rc < 0)
		return rc;

	if (on)
		rc |= (1 << gpio);
	else
		rc &= ~(1 << gpio);

	return i2c_smbus_write_byte_data(fd, SX150x_REG_PULL_UP, rc);
}

int sx150x_gpio_pull_down_set(int fd, int gpio, int on)
{
	int rc;

	rc = i2c_smbus_read_byte_data(fd, SX150x_REG_PULL_DOWN);
	if (rc < 0)
		return rc;

	if (on)
		rc |= (1 << gpio);
	else
		rc &= ~(1 << gpio);

	return i2c_smbus_write_byte_data(fd, SX150x_REG_PULL_DOWN, rc);
}

int sx150x_gpio_set(int fd, int gpio, int high)
{
	int rc;

	rc = i2c_smbus_read_byte_data(fd, SX150x_REG_DATA);
	if (rc < 0)
		return rc;

	if (high)
		rc |= (1 << gpio);
	else
		rc &= ~(1 << gpio);

	return i2c_smbus_write_byte_data(fd, SX150x_REG_DATA, rc);
}

int sx150x_gpio_get(int fd, int gpio)
{
	int rc;

	rc = i2c_smbus_read_byte_data(fd, SX150x_REG_DATA);
	if (rc < 0)
		return rc;
	return (rc & (1 << gpio));
}
