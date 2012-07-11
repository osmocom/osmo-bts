#ifndef SYSMOBTS_LAYER_H
#define SYSMOBTS_LAYER_H

extern int initialize_layer1(uint32_t dsp_flags);
extern int print_system_info();
extern int activate_rf_frontend(int clock_source, int clock_cor);
extern int power_scan(int band, int arfcn, int duration, float *mean_rssi);
extern int follow_sch(int band, int arfcn, int calib, int reference, HANDLE *layer1);
extern int follow_bch(HANDLE layer1);
extern int find_bsic(void);
extern int set_tsc_from_bsic(HANDLE layer1, int bsic);
extern int set_clock_cor(int clock_corr, int calib, int source);
extern int rf_clock_info(HANDLE *layer1, int *clkErr, int *clkErrRes);
extern int mph_close(HANDLE layer1);
extern int wait_for_sync(HANDLE layer1, int cor, int calib, int source);
extern int follow_bcch(HANDLE layer1);
extern int wait_for_data(uint8_t *data, size_t *size);

#endif
