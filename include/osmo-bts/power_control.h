#pragma once

#include <stdint.h>
#include <stdbool.h>

/* MS/BS Power related measurement averaging algo */
enum gsm_power_ctrl_meas_avg_algo {
	GSM_PWR_CTRL_MEAS_AVG_ALGO_NONE			= 0x00,
	GSM_PWR_CTRL_MEAS_AVG_ALGO_UNWEIGHTED		= 0x01,
	GSM_PWR_CTRL_MEAS_AVG_ALGO_WEIGHTED		= 0x02,
	GSM_PWR_CTRL_MEAS_AVG_ALGO_MOD_MEDIAN		= 0x03,
	/* EWMA is an Osmocom specific algo */
	GSM_PWR_CTRL_MEAS_AVG_ALGO_OSMO_EWMA		= 0x04,
};

/* MS/BS Power related measurement parameters */
struct gsm_power_ctrl_meas_params {
	/* Thresholds (see 3GPP TS 45.008, section A.3.2.1) */
	uint8_t lower_thresh; /* lower (decreasing) direction */
	uint8_t upper_thresh; /* upper (increasing) direction */

	/* Threshold Comparators for lower (decreasing) direction */
	uint8_t lower_cmp_p; /* P1 for RxLev, P3 for RxQual */
	uint8_t lower_cmp_n; /* N1 for RxLev, N3 for RxQual */
	/* Threshold Comparators for upper (increasing) direction */
	uint8_t upper_cmp_p; /* P2 for RxLev, P4 for RxQual */
	uint8_t upper_cmp_n; /* N2 for RxLev, N4 for RxQual */

	/* Hreqave and Hreqt (see 3GPP TS 45.008, Annex A) */
	uint8_t h_reqave;
	uint8_t h_reqt;

	/* AVG algorithm and its specific parameters */
	enum gsm_power_ctrl_meas_avg_algo algo;
	union {
		/* Exponentially Weighted Moving Average */
		struct {
			/* Smoothing factor: higher the value - less smoothing */
			uint8_t alpha; /* 1 .. 99 (in %) */
		} ewma;
	};
};

/* MS/BS Power Control parameters */
struct gsm_power_ctrl_params {
	/* Minimum interval between power level changes */
	uint8_t ctrl_interval; /* 1 step is 2 SACCH periods */

	/* Power change step size (maximum) */
	uint8_t inc_step_size_db; /* increasing direction */
	uint8_t red_step_size_db; /* reducing direction */

	/* Measurement averaging parameters for RxLev & RxQual */
	struct gsm_power_ctrl_meas_params rxqual_meas;
	struct gsm_power_ctrl_meas_params rxlev_meas;

	/* Measurement averaging parameters for C/I, per chan type */
	struct gsm_power_ctrl_meas_params ci_fr_meas;
	struct gsm_power_ctrl_meas_params ci_hr_meas;
	struct gsm_power_ctrl_meas_params ci_amr_fr_meas;
	struct gsm_power_ctrl_meas_params ci_amr_hr_meas;
	struct gsm_power_ctrl_meas_params ci_sdcch_meas;
	struct gsm_power_ctrl_meas_params ci_gprs_meas;
};

/* Measurement pre-processing state */
struct gsm_power_ctrl_meas_proc_state {
	/* Number of measurements processed */
	unsigned int meas_num;
	/* Algorithm specific data */
	union {
		struct {
			/* Scaled up 100 times average value */
			int Avg100;
		} ewma;
	};
};

/* Default MS/BS Power Control parameters */
extern const struct gsm_power_ctrl_params power_ctrl_params_def;
void power_ctrl_params_def_reset(struct gsm_power_ctrl_params *params, bool is_bs_pwr);

struct gsm_lchan;
int lchan_ms_pwr_ctrl(struct gsm_lchan *lchan,
		      const uint8_t ms_power_lvl,
		      const int8_t ul_rssi_dbm,
		      const int16_t ul_lqual_cb);

int lchan_bs_pwr_ctrl(struct gsm_lchan *lchan,
		      const struct gsm48_meas_res *mr);
