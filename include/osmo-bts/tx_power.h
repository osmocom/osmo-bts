#pragma once

#include <stdint.h>
#include <osmocom/core/timer.h>

/* our unit is 'milli dB" or "milli dBm", i.e. 1/1000 of a dB(m) */
#define to_mdB(x)	(x * 1000)

/* PA calibration table */
struct pa_calibration {
	int delta_mdB[1024];		/* gain delta at given ARFCN */
	/* FIXME: thermal calibration */
};

/* representation of a RF power amplifier */
struct power_amp {
	/* nominal gain of the PA */
	int nominal_gain_mdB;
	/* table with calibrated actual gain for each ARFCN */
	struct pa_calibration calib;
};

typedef void (*ramp_compl_cb_t)(struct gsm_bts_trx *trx);

/* Transmit power related parameters of a transceiver */
struct trx_power_params {
	/* specified maximum output of TRX at full power, has to be
	 * initialized by BTS model at startup*/
	int trx_p_max_out_mdBm;

	/* intended current total system output power */
	int p_total_tgt_mdBm;

	/* actual current total system output power, filled in by tx_power code */
	int p_total_cur_mdBm;

	/* current temporary attenuation due to thermal management,
	 * set by thermal management code via control interface */
	int thermal_attenuation_mdB;

	/* external gain (+) or attenuation (-) added by the user, configured
	 * by the user via VTY */
	int user_gain_mdB;

	/* calibration table of internal PA */
	struct power_amp pa;

	/* calibration table of user PA */
	struct power_amp user_pa;

	/* power ramping related data */
	struct {
		/* maximum initial Pout including all PAs */
		int max_initial_pout_mdBm;
		/* temporary attenuation due to power ramping */
		int attenuation_mdB;
		unsigned int step_size_mdB;
		unsigned int step_interval_sec;
		struct osmo_timer_list step_timer;
		/* call-back called when target is reached */
		ramp_compl_cb_t compl_cb;
	} ramp;
};

int get_p_max_out_mdBm(const struct gsm_bts_trx *trx);

int get_p_nominal_mdBm(const struct gsm_bts_trx *trx);

int get_p_target_mdBm(const struct gsm_bts_trx *trx, uint8_t bs_power_red);
int get_p_target_mdBm_lchan(const struct gsm_lchan *lchan);

int get_p_actual_mdBm(const struct gsm_bts_trx *trx, int p_target_mdBm);

int get_p_trxout_target_mdBm(const struct gsm_bts_trx *trx, uint8_t bs_power_red);
int get_p_trxout_target_mdBm_lchan(const struct gsm_lchan *lchan);

int get_p_trxout_actual_mdBm(const struct gsm_bts_trx *trx, uint8_t bs_power_red);
int get_p_trxout_actual_mdBm_lchan(const struct gsm_lchan *lchan);

int power_ramp_start(struct gsm_bts_trx *trx, int p_total_tgt_mdBm, int bypass, ramp_compl_cb_t ramp_compl_cb);
void power_ramp_abort(struct gsm_bts_trx *trx);

void power_trx_change_compl(struct gsm_bts_trx *trx, int p_trxout_cur_mdBm);

int power_ramp_initial_power_mdBm(const struct gsm_bts_trx *trx);
