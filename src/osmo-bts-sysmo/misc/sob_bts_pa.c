#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <limits.h>
#include <unistd.h>
#include <fcntl.h>

#include "i2c-dev.h"
#include "sx150x.h"

#define SOB_BTS_NUM_PA	2

static int g_pa_gpio_offset;
static int g_i2c_fd;

int sob_bts_pa_enable(int pa_num, int enable)
{
	int level;

	if (pa_num >= SOB_BTS_NUM_PA)
		return -EINVAL;

	if (enable)
		level = 0;
	else
		level = 1;

	return sx150x_gpio_set(g_i2c_fd, g_pa_gpio_offset + pa_num, level);
}

int sob_bts_pa_init(int adapter_nr)
{
	char filename[PATH_MAX];
	int rc;

	snprintf(filename, sizeof(filename)-1, "/dev/i2c-%d", adapter_nr);
	rc = open(filename, O_RDWR);
	if (rc < 0) {
		fprintf(stderr, "Error opening the device: %d\n", rc);
		return rc;
	}

	g_i2c_fd = rc;

	rc = ioctl(g_i2c_fd, I2C_SLAVE, SX150x_ADDR_ADDR0);
	if (rc < 0) {
		fprintf(stderr, "Error setting slave addr: %d\n", rc);
		close(g_i2c_fd);
		return rc;
	}

	/* enable pull-up on GPIO7 */
	sx150x_gpio_pull_up_set(g_i2c_fd, 7, 1);

	if (sx150x_gpio_get(g_i2c_fd, 7) == 0) {
		/* daughter board installed: PAs connected to IO2+3 */
		g_pa_gpio_offset = 2;
	} else {
		/* PAs directly connected to 0+1 */
		g_pa_gpio_offset = 0;
	}

	/* set both as output */
	sx150x_gpio_direction_set(g_i2c_fd, g_pa_gpio_offset, SX150x_DIR_OUTPUT);
	sx150x_gpio_direction_set(g_i2c_fd, g_pa_gpio_offset+1, SX150x_DIR_OUTPUT);

	/* disable them as default */
	sob_bts_pa_enable(0, 0);
	sob_bts_pa_enable(1, 0);

	return 0;
}
