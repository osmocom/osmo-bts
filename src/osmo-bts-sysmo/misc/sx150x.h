#pragma once
#include <stdint.h>

#define SX150x_ADDR_ADDR0	0x20
#define SX150x_ADDR_ADDR1	0x21

enum sx150x_direction {
	SX150x_DIR_OUTPUT	= 0,
	SX150x_DIR_INPUT	= 1
};

int sx150x_gpio_pull_up_set(int fd, int gpio, int on);
int sx150x_gpio_pull_down_set(int fd, int gpio, int on);

int sx150x_gpio_direction_set(int fd, int gpio, enum sx150x_direction dir);
int sx150x_gpio_direction_get(int fd, int gpio);

int sx150x_gpio_set(int fd, int gpio, int high);
int sx150x_gpio_get(int fd, int gpio);
