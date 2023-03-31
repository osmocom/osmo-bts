static const int gsm_fr_bitclass[] = {
	-1, -1, -1, -1,  0,  0,  0,  0,  1,  2,  0,  0,  0,  1,  2,  2,
	 0,  0,  1,  2,  2,  0,  0,  1,  2,  2,  0,  1,  1,  2,  0,  1,
	 2,  2,  0,  1,  2,  1,  2,  2,  0,  0,  0,  0,  0,  0,  1,  1,
	 1,  1,  1,  0,  0,  0,  1,  1,  2,  1,  1,  2,  1,  1,  2,  1,
	 1,  2,  1,  1,  2,  1,  1,  2,  1,  1,  2,  1,  1,  2,  1,  1,
	 2,  1,  1,  2,  1,  1,  2,  1,  1,  2,  1,  1,  2,  1,  1,  2,
	 0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  0,  0,  0,  1,  1,
	 2,  1,  1,  2,  1,  1,  2,  1,  1,  2,  1,  1,  2,  1,  1,  2,
	 1,  1,  2,  1,  1,  2,  1,  1,  2,  1,  1,  2,  1,  1,  2,  1,
	 1,  2,  1,  1,  2,  1,  1,  2,  0,  0,  0,  0,  0,  0,  1,  1,
	 1,  1,  1,  0,  0,  0,  1,  1,  2,  1,  1,  2,  1,  1,  2,  1,
	 1,  2,  1,  1,  2,  1,  1,  2,  1,  1,  2,  1,  1,  2,  1,  1,
	 2,  1,  1,  2,  1,  1,  2,  1,  1,  2,  1,  1,  2,  1,  1,  2,
	 0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  0,  0,  0,  1,  1,
	 2,  1,  1,  2,  1,  1,  2,  1,  1,  2,  1,  1,  2,  1,  2,  2,
	 1,  2,  2,  1,  2,  2,  1,  2,  2,  1,  2,  2,  1,  2,  2,  1,
	 2,  2,  1,  2,  2,  1,  2,  2,
};

static const int gsm_efr_bitclass[] = {
	 1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  0,
	 0,  0,  0,  0,  1,  1,  1,  0,  1,  1,  1,  1,  1,  1,  1,  1,
	 1,  1,  1,  1,  1,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
	 1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,
	 2,  2,  2,  2,  2,  2,  0,  1,  1,  1,  1,  0,  0,  0,  0,  0,
	 1,  0,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
	 1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,  2,  2,
	 2,  2,  2,  2,  2,  2,  2,  2,  0,  1,  1,  1,  1,  0,  0,  0,
	 0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
	 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,
	 2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  1,  1,  1,
	 1,  1,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  1,  1,  1,  1,
	 1,  1,  2,  2,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,
	 2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  1,
	 1,  1,  1,  1,  2,  2,  1,  2,
};

static const int gsm_amr_12_2_bitclass[] = {
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	 0,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
	 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
	 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
	 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
	 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
	 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
	 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
	 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
	 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
	 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
	 1,  1,  1,  1, -1, -1, -1, -1,
};

