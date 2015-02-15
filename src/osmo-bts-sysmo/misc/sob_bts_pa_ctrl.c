#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "sob_bts_pa.h"

static void print_help(const char *cmdname)
{
	printf("%s <0-1> <on|off>\n", cmdname);
}

int main(int argc, char **argv)
{
	int rc;
	int pa_nr, on;

	if (argc < 3)
		goto err_param;

	pa_nr = atoi(argv[1]);
	if (pa_nr < 0 || pa_nr > 1)
		goto err_param;

	if (!strcmp(argv[2], "on"))
		on = 1;
	else if (!strcmp(argv[2], "off"))
		on = 0;
	else
		goto err_param;

	rc = sob_bts_pa_init(1);
	if (rc < 0)
		exit(1);

	printf("%sabling SOB-BTS PA #%u\n", on? "en" : "dis", pa_nr);
	rc = sob_bts_pa_enable(pa_nr, on);
	if (rc < 0) {
		fprintf(stderr, "Error during PA operation\n");
		exit(1);
	}

	exit(0);

err_param:
	print_help(argv[0]);
	exit(2);
}
