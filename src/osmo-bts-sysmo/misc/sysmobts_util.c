/* sysmobts-util - access to hardware related parameters */

/* (C) 2012-2013 by Harald Welte <laforge@gnumonks.org>
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
#include <getopt.h>


#include "sysmobts_par.h"

enum act {
	ACT_GET,
	ACT_SET,
};

static enum act action;
static char *write_arg;

static void print_help()
{
	printf("sysmobts-util [-r | -w value] param_name\n");
}

static int parse_options(int argc, char **argv)
{
	int opt;

	while ((opt = getopt(argc, argv, "rw:h")) != -1) {
		switch (opt) {
		case 'r':
			action = ACT_GET;
			break;
		case 'w':
			action = ACT_SET;
			write_arg = optarg;
			break;
		case 'h':
			print_help();
			return -1;
			break;
		default:
			return -1;
		}
	}

	return 0;
}

int main(int argc, char **argv)
{
	const char *parname;
	enum sysmobts_par par;
	int rc, val;

	rc = parse_options(argc, argv);
	if (rc < 0)
		exit(2);

	if (optind >= argc) {
		fprintf(stderr, "You must specify the parameter name\n");
		exit(2);
	}
	parname = argv[optind];

	rc = get_string_value(sysmobts_par_names, parname);
	if (rc < 0) {
		fprintf(stderr, "`%s' is not a valid parameter\n", parname);
		exit(2);
	} else
		par = rc;

	switch (action) {
	case ACT_GET:
		rc = sysmobts_par_get_int(par, &val);
		if (rc < 0) {
			fprintf(stderr, "Error %d\n", rc);
			goto err;
		}
		printf("%d\n", val);
		break;
	case ACT_SET:
		rc = sysmobts_par_get_int(par, &val);
		if (rc < 0) {
			fprintf(stderr, "Error %d\n", rc);
			goto err;
		}
		if (val != 0xFFFF && val != 0xFF && val != 0xFFFFFFFF) {
			fprintf(stderr, "Parameter is already set!\r\n");
			goto err;
		}
		rc = sysmobts_par_set_int(par, atoi(write_arg));
		if (rc < 0) {
			fprintf(stderr, "Error %d\n", rc);
			goto err;
		}
		printf("Success setting %s=%d\n", parname,
			atoi(write_arg));
		break;
	default:
		fprintf(stderr, "Unsupported action\n");
		goto err;
	}

	exit(0);

err:
	exit(1);
}

