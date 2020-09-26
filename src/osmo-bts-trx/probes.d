provider osmo_bts_trx {
	probe ul_data_start(int, int, int); /* trx_nr, ts_nr, fn */
	probe ul_data_done(int, int, int); /* trx_nr, ts_nr, fn */

	probe dl_rts_start(int, int, int); /* trx_nr, ts_nr, fn */
	probe dl_rts_done(int, int, int); /* trx_nr, ts_nr, fn */
};
