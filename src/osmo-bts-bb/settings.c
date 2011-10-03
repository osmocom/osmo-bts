/* Cell layout and configuration */

/* (C) 2011 by Andreas Eversberg <jolly@eversberg.eu>
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
#include <stdint.h>
#include <string.h>
#include "settings.h"

struct cell_layout {
	char *name;		/* name of layout */
	uint8_t slot_mask;	/* slots to be supported for BTS */
	char *desc;		/* description */
	uint8_t dl[8];		/* which baseband transmits on which slot */
	uint8_t ul[8];		/* which baseband receives on which slot */
};

static struct cell_layout cell_layouts[] = {
	{ "bcch", 0x01, "Send BCCH only, TX on slots 0..5.",
	 {1, 1, 1, 1, 1, 1, 0, 0},
	 {0, 0, 0, 0, 0, 0, 0, 0}},
	{ "4slots", 0xc3, "Use two phones, TX on all slots, RX on 4 slots",
	 {1, 1, 2, 2, 2, 2, 1, 1},
	 {1, 1, 0, 0, 2, 2, 0, 0}},
	{ "6slots", 0x3f, "Use two phones, TX on 6 slots, RX on 6 slots",
	 {1, 1, 2, 2, 2, 2, 0, 0},
	 {1, 1, 1, 1, 2, 2, 0, 0}},
	{ "8slots", 0xff, "Use three phones, TX and RX on all slots",
	 {1, 1, 3, 3, 2, 2, 3, 3},
	 {1, 1, 1, 1, 2, 2, 2, 2}},
	{ NULL, 0x00, NULL,
	 {0, 0, 0, 0, 0, 0, 0, 0},
	 {0, 0, 0, 0, 0, 0, 0, 0}},
};

int set_show_layout(int i)
{
	int s, p;
	uint8_t d_mask, u_mask;

	printf("Layout name: '%s'\n", cell_layouts[i].name);
	printf("Description: %s\n", cell_layouts[i].desc);
	printf("Slots used for BTS:");
	for (s = 0; s < 8; s++) {
		if ((cell_layouts[i].slot_mask & (1 << s)))
			printf(" %d", s);
	}
	printf("\n");
	for (p = 1; ; p++) {
		d_mask = 0;
		u_mask = 0;
		for (s = 0; s < 8; s++) {
			if (cell_layouts[i].dl[s] == p)
				d_mask |= (1 << s);
			if (cell_layouts[i].ul[s] == p)
				u_mask |= (1 << s);
		}
		if (d_mask == 0 && u_mask == 0)
			break;
		if (d_mask) {
			printf("Phone %d provides TX slots:", p);
			for (s = 0; s < 8; s++) {
				if ((d_mask & (1 << s)))
					printf(" %d", s);
			}
			printf("\n");
		}
		if (u_mask) {
			printf("Phone %d provides RX slots:", p);
			for (s = 0; s < 8; s++) {
				if ((u_mask & (1 << s)))
					printf(" %d", s);
			}
			printf("\n");
		}
	}

	return p - 1; /* number of phones */
}

int set_show_layouts(void)
{
	int i;

	printf("List of all layouts:\n");
	for (i = 0; cell_layouts[i].name; i++) {
		printf("\n");
		set_show_layout(i);
	}

	return 0;
}

int set_layout_by_name(const char *name)
{
	int i;

	for (i = 0; cell_layouts[i].name; i++) {
		if (!strcmp(cell_layouts[i].name, name))
			break;
	}

	if (!cell_layouts[i].name)
		return -1;

	return i;
}

int set_num_phones(int i)
{
	int s, p;
	uint8_t d_mask, u_mask;

	for (p = 1; ; p++) {
		d_mask = 0;
		u_mask = 0;
		for (s = 0; s < 8; s++) {
			if (cell_layouts[i].dl[s] == p)
				d_mask |= (1 << s);
			if (cell_layouts[i].ul[s] == p)
				u_mask |= (1 << s);
		}
		if (d_mask == 0 && u_mask == 0)
			break;
	}

	return p - 1; /* number of phones */
}

uint8_t set_get_tx_mask(int i, int p)
{
	uint8_t mask = 0;
	int s;

	for (s = 0; s < 8; s++) {
		if (cell_layouts[i].dl[s] == p)
			mask |= (1 << s);
	}

	return mask;
}

uint8_t set_get_rx_mask(int i, int p)
{
	uint8_t mask = 0;
	int s;

	for (s = 0; s < 8; s++) {
		if (cell_layouts[i].ul[s] == p)
			mask |= (1 << s);
	}

	return mask;
}

