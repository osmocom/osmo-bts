#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

#define _GNU_SOURCE
#include <getopt.h>

#include <osmocom/core/talloc.h>
#include <osmocom/core/application.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/abis.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/oml.h>

static void print_usage(const char *prog_name)
{
	printf("Usage: %s [-h] [--features FOO,BAR,BAZ] dst_host site_id [trx_num]\n", prog_name);
}

static void print_help(const char *prog_name)
{
	print_usage(prog_name);
	printf("  -h --help			This text.\n");
	printf("  -f --features	FOO,BAR,BAZ	BTS features to issue on OML startup.\n"
	       "				The names correspond to BTS_FEAT_* constants\n"
	       "				as defined in osmocom/gsm/bts_features.h,\n"
	       "				e.g. '-f VAMOS'\n");
}

struct {
	char *dst_host;
	int site_id;
	int trx_num;
	char *features;
} cmdline = {
	.trx_num = 8,
};

void parse_cmdline(int argc, char **argv)
{
	while (1) {
		int option_index = 0, c;
		static struct option long_options[] = {
			{"help", 0, 0, 'h'},
			{"features", 1, 0, 'f'},
			{0}
		};

		c = getopt_long(argc, argv, "hf:", long_options, &option_index);
		if (c == -1)
			break;

		switch (c) {
		case 'h':
			print_help(argv[0]);
			exit(0);
		case 'f':
			cmdline.features = optarg;
			break;
		default:
			/* catch unknown options *as well as* missing arguments. */
			fprintf(stderr, "Error in command line options. Exiting.\n");
			exit(-1);
		}
	}

	if (optind + 2 > argc) {
		print_usage(argv[0]);
		exit(1);
	}

	cmdline.dst_host = argv[optind];
	cmdline.site_id = atoi(argv[optind + 1]);
	if (optind + 2 < argc)
		cmdline.trx_num = atoi(argv[optind + 2]);

	if (optind + 3 < argc) {
		print_usage(argv[0]);
		exit(1);
	}
}

void set_bts_features(struct bitvec *features, char *features_str)
{
	char *saveptr = NULL;
	char *token;

	if (!features_str)
		return;

	while ((token = strtok_r(features_str, ",", &saveptr))) {
		enum osmo_bts_features feat;
		features_str = NULL;

		feat = get_string_value(osmo_bts_features_names, token);

		if ((int)feat < 0) {
			fprintf(stderr, "Unknown BTS feature: '%s'\n", token);
			exit(-1);
		}

		osmo_bts_set_feature(features, feat);
	}
}

int main(int argc, char **argv)
{
	struct gsm_bts *bts;
	struct gsm_bts_trx *trx;
	struct e1inp_line *line;
	int i;

	parse_cmdline(argc, argv);

	tall_bts_ctx = talloc_named_const(NULL, 1, "OsmoBTS context");
	msgb_talloc_ctx_init(tall_bts_ctx, 10*1024);

	osmo_init_logging2(tall_bts_ctx, &bts_log_info);

	bts = gsm_bts_alloc(tall_bts_ctx, 0);
	if (!bts)
		exit(1);
	bts->ip_access.site_id = cmdline.site_id;
	bts->ip_access.bts_id = 0;

	/* Additional TRXs */
	for (i = 1; i < cmdline.trx_num; i++) {
		trx = gsm_bts_trx_alloc(bts, NULL);
		if (!trx)
			exit(1);
	}

	if (bts_init(bts) < 0)
		exit(1);

	set_bts_features(bts->features, cmdline.features);

	/* VAMOS: set up shadow TRXes */
	if (osmo_bts_has_feature(bts->features, BTS_FEAT_VAMOS)) {
		llist_for_each_entry(trx, &bts->trx_list, list) {
			/* Does this TRX already have a shadow TRX set up? */
			if (trx->vamos.shadow_trx)
				continue;
			/* Is this TRX itself a shadow TRX? */
			if (trx->vamos.primary_trx)
				continue;

			/* This is a primary TRX that should have a shadow TRX, and the shadow TRX still needs to be set
			 * up. */
			gsm_bts_trx_alloc(bts, trx);
		}
	}

	//btsb = bts_role_bts(bts);
	abis_init(bts);

	line = abis_open(bts, cmdline.dst_host, "OMLdummy");
	if (!line)
		exit(2);

	while (1) {
		osmo_select_main(0);
	}

	return EXIT_SUCCESS;
}
