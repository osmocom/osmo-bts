#define ULM(ber, ta, neg_rssi) \
	{ .ber10k = (ber), .ta_offs_256bits = (ta), .c_i = 1.0, .is_sub = 0, .inv_rssi = (neg_rssi) }

struct meas_testcase {
	const char *name;
	/* input data */
	const struct bts_ul_meas *ulm;
	unsigned int ulm_count;
	uint32_t final_fn;
	/* results */
	struct {
		int success;
		uint8_t rx_lev_full;
		uint8_t rx_qual_full;
		int16_t toa256_mean;
		int16_t toa256_min;
		int16_t toa256_max;
		uint16_t toa256_std_dev;
	} res;
};

static struct bts_ul_meas ulm1[] = {
	ULM(0, 0, 90),
	ULM(0, 256, 90),
	ULM(0, -256, 90),
};
static const struct meas_testcase mtc1 = {
	.name = "TOA256 Min-Max negative/positive",
	.ulm = ulm1,
	.ulm_count = ARRAY_SIZE(ulm1),
	.final_fn = 25,
	.res = {
		.success = 1,
		.rx_lev_full = 110-90,
		.rx_qual_full = 0,
		.toa256_mean = 0,
		.toa256_max = 256,
		.toa256_min = -256,
		.toa256_std_dev = 209,
	},
};

static struct bts_ul_meas ulm2[] = {
	ULM(0, 256, 90),
	ULM(0, 258, 90),
	ULM(0, 254, 90),
	ULM(0, 258, 90),
	ULM(0, 254, 90),
	ULM(0, 256, 90),
};
static const struct meas_testcase mtc2 = {
	.name = "TOA256 small jitter around 256",
	.ulm = ulm2,
	.ulm_count = ARRAY_SIZE(ulm2),
	.final_fn = 25,
	.res = {
		.success = 1,
		.rx_lev_full = 110-90,
		.rx_qual_full = 0,
		.toa256_mean = 256,
		.toa256_max = 258,
		.toa256_min = 254,
		.toa256_std_dev = 1,
	},
};

static struct bts_ul_meas ulm3[] = {
	ULM(0, 0, 90),
	ULM(0, 0, 80),
	ULM(0, 0, 80),
	ULM(0, 0, 100),
	ULM(0, 0, 100),
};
static const struct meas_testcase mtc3 = {
	.name = "RxLEv averaging",
	.ulm = ulm3,
	.ulm_count = ARRAY_SIZE(ulm3),
	.final_fn = 25,
	.res = {
		.success = 1,
		.rx_lev_full = 110-90,
		.rx_qual_full = 0,
		.toa256_mean = 0,
		.toa256_max = 0,
		.toa256_min = 0,
		.toa256_std_dev = 0,
	},
};

static struct bts_ul_meas ulm4[] = {};
static const struct meas_testcase mtc4 = {
	.name = "Empty measurements",
	.ulm = ulm4,
	.ulm_count = ARRAY_SIZE(ulm4),
	.final_fn = 25,
	.res = {
		.success = 0,
		.rx_lev_full = 0,
		.rx_qual_full = 0,
		.toa256_mean = 0,
		.toa256_max = 0,
		.toa256_min = 0,
		.toa256_std_dev = 0,
	},
};

static struct bts_ul_meas ulm5[] = {
	/* one 104 multiframe can at max contain 26 blocks (TCH/F),
	 * each of which can at maximum be 64 bits in advance (TA range) */
	ULM(0, 64*256, 90),
	ULM(0, 64*256, 90),
	ULM(0, 64*256, 90),
	ULM(0, 64*256, 90),
	ULM(0, 64*256, 90),
	ULM(0, 64*256, 90),
	ULM(0, 64*256, 90),
	ULM(0, 64*256, 90),
	ULM(0, 64*256, 90),
	ULM(0, 64*256, 90),
	ULM(0, 64*256, 90),
	ULM(0, 64*256, 90),
	ULM(0, 64*256, 90),
	ULM(0, 64*256, 90),
	ULM(0, 64*256, 90),
	ULM(0, 64*256, 90),
	ULM(0, 64*256, 90),
	ULM(0, 64*256, 90),
	ULM(0, 64*256, 90),
	ULM(0, 64*256, 90),
	ULM(0, 64*256, 90),
	ULM(0, 64*256, 90),
	ULM(0, 64*256, 90),
	ULM(0, 64*256, 90),
	ULM(0, 64*256, 90),
	ULM(0, 64*256, 90),
};
static const struct meas_testcase mtc5 = {
	.name = "TOA256 26 blocks with max TOA256",
	.ulm = ulm5,
	.ulm_count = ARRAY_SIZE(ulm5),
	.final_fn = 25,
	.res = {
		.success = 1,
		.rx_lev_full = 110-90,
		.rx_qual_full = 0,
		.toa256_mean = 64*256,
		.toa256_max = 64*256,
		.toa256_min = 64*256,
		.toa256_std_dev = 0,
	},
};
