#ifndef _SYSMOBTS_EEPROM_H
#define _SYSMOBTS_EEPROM_H

#include <stdint.h>

struct sysmobts_eeprom {		/* offset */
	uint8_t eth_mac[6];		/* 0-5 */
	uint8_t _pad0[10];		/* 6-15 */
	uint16_t clk_cal_fact;		/* 16-17 */
	uint8_t temp1_max;		/* 18 */
	uint8_t temp2_max;		/* 19 */
	uint32_t serial_nr;		/* 20-23 */
	uint32_t operational_hours;	/* 24-27 */
	uint32_t boot_count;		/* 28-31 */
	uint8_t _pad1[89];		/* 32-127 */
	uint8_t gpg_key[128];		/* 121-249 */
} __attribute__((packed));

#endif
