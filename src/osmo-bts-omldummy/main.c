#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

#define _GNU_SOURCE
#include <getopt.h>

#include <osmocom/core/talloc.h>
#include <osmocom/core/application.h>
#include <osmocom/core/logging.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/abis.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/bts_sm.h>
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
			{ "debug", 1, 0, 'd' },
			{ "disable-color", 0, 0, 's' },
			{ "timestamp", 0, 0, 'T' },
			{ "log-level", 1, 0, 'e' },
			{0}
		};

		c = getopt_long(argc, argv, "hf:d:sTe:", long_options, &option_index);
		if (c == -1)
			break;

		switch (c) {
		case 'h':
			print_help(argv[0]);
			exit(0);
		case 'f':
			cmdline.features = optarg;
			break;
		case 's':
			log_set_use_color(osmo_stderr_target, 0);
			break;
		case 'd':
			log_parse_category_mask(osmo_stderr_target, optarg);
			break;
		case 'T':
			log_set_print_timestamp(osmo_stderr_target, 1);
			break;
		case 'e':
			log_set_log_level(osmo_stderr_target, atoi(optarg));
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
	struct bsc_oml_host *bsc_oml_host;
	int i;

	tall_bts_ctx = talloc_named_const(NULL, 1, "OsmoBTS context");
	msgb_talloc_ctx_init(tall_bts_ctx, 10*1024);

	osmo_init_logging2(tall_bts_ctx, &bts_log_info);

	parse_cmdline(argc, argv);

	g_bts_sm = gsm_bts_sm_alloc(tall_bts_ctx);
	if (!g_bts_sm)
		exit(1);

	bts = gsm_bts_alloc(g_bts_sm, 0);
	if (!bts)
		exit(1);

	bts->ip_access.site_id = cmdline.site_id;
	bts->ip_access.bts_id = 0;

	/* Additional TRXs */
	for (i = 1; i < cmdline.trx_num; i++) {
		trx = gsm_bts_trx_alloc(bts);
		if (!trx)
			exit(1);
	}

	if (bts_init(bts) < 0)
		exit(1);

	set_bts_features(bts->features, cmdline.features);

	/* VAMOS: allocate shadow timeslots for each TRX */
	if (osmo_bts_has_feature(bts->features, BTS_FEAT_VAMOS)) {
		llist_for_each_entry(trx, &bts->trx_list, list)
			gsm_bts_trx_init_shadow_ts(trx);
	}

	//btsb = bts_role_bts(bts);
	abis_init(bts);

	bsc_oml_host = talloc_zero(bts, struct bsc_oml_host);
	OSMO_ASSERT(bsc_oml_host);
	bsc_oml_host->addr = talloc_strdup(bsc_oml_host, cmdline.dst_host);
	OSMO_ASSERT(bsc_oml_host->addr);
	llist_add_tail(&bsc_oml_host->list, &bts->bsc_oml_hosts);
	if (abis_open(bts, "OMLdummy") != 0)
		exit(1);

	while (1) {
		osmo_select_main(0);
	}

	return EXIT_SUCCESS;
}
