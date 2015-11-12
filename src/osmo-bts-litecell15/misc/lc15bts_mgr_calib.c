/* OCXO calibration control for Litecell 1.5 BTS management daemon */

/* Copyright (C) 2015 by Yves Godin <support@nuranwireless.com>
 * 
 * Based on sysmoBTS:
 *     sysmobts_mgr_calib.c
 *     (C) 2014,2015 by Holger Hans Peter Freyther
 *     (C) 2014 by Harald Welte for the IPA code from the oml router
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

#include "misc/lc15bts_mgr.h"
#include "misc/lc15bts_misc.h"
#include "misc/lc15bts_clock.h"
#include "osmo-bts/msg_utils.h"

#include <osmocom/core/logging.h>
#include <osmocom/core/select.h>

#include <osmocom/ctrl/control_cmd.h>

#include <osmocom/gsm/ipa.h>
#include <osmocom/gsm/protocol/ipaccess.h>

#include <osmocom/abis/abis.h>
#include <osmocom/abis/e1_input.h>
#include <osmocom/abis/ipa.h>

static void calib_adjust(struct lc15bts_mgr_instance *mgr);
static void calib_state_reset(struct lc15bts_mgr_instance *mgr, int reason);
static void calib_loop_run(void *_data);

enum calib_state {
	CALIB_INITIAL,
	CALIB_IN_PROGRESS,
};

enum calib_result {
        CALIB_FAIL_START,
        CALIB_FAIL_GPSFIX,
        CALIB_FAIL_CLKERR,
        CALIB_FAIL_OCXODAC,
        CALIB_SUCCESS,
};

static void calib_start(struct lc15bts_mgr_instance *mgr)
{
	int rc;

	rc = lc15bts_clock_err_open();
	if (rc != 0) {
		LOGP(DCALIB, LOGL_ERROR, "Failed to open clock error module %d\n", rc);
		calib_state_reset(mgr, CALIB_FAIL_CLKERR);
		return;
	}

	rc = lc15bts_clock_dac_open();
	if (rc != 0) {
		LOGP(DCALIB, LOGL_ERROR, "Failed to open OCXO dac module %d\n", rc);
		calib_state_reset(mgr, CALIB_FAIL_OCXODAC);
		return;
	}

	calib_adjust(mgr);
}

static void calib_adjust(struct lc15bts_mgr_instance *mgr)
{
	int rc;
	int fault;
	int error_ppt;
	int accuracy_ppq;
	int interval_sec;
	int dac_value;
	int new_dac_value;
	double dac_correction;

	rc = lc15bts_clock_err_get(&fault, &error_ppt, 
			&accuracy_ppq, &interval_sec);
	if (rc < 0) {
		LOGP(DCALIB, LOGL_ERROR, 
			"Failed to get clock error measurement %d\n", rc);
		calib_state_reset(mgr, CALIB_FAIL_CLKERR);
		return;
	}

	if (fault) {
		LOGP(DCALIB, LOGL_NOTICE, "GPS has no fix\n");
		calib_state_reset(mgr, CALIB_FAIL_GPSFIX);
		return;
	}

	rc = lc15bts_clock_dac_get(&dac_value);
	if (rc < 0) {
		LOGP(DCALIB, LOGL_ERROR, 
			"Failed to get OCXO dac value %d\n", rc);
		calib_state_reset(mgr, CALIB_FAIL_OCXODAC);
		return;
	}

	LOGP(DCALIB, LOGL_NOTICE,
		"Calibration ERR(%f PPB) ACC(%f PPB) INT(%d) DAC(%d)\n",
		error_ppt / 1000., accuracy_ppq / 1000000., interval_sec, dac_value);

	/* 1 unit of correction equal about 0.5 - 1 PPB correction */ 
	dac_correction = (int)(-error_ppt * 0.00056);
	new_dac_value = dac_value + dac_correction + 0.5;

	/* We have a fix, make sure the measured error is 
	meaningful (10 times the accuracy) */ 
	if ((new_dac_value != dac_value) && ((100l * abs(error_ppt)) > accuracy_ppq)) { 

		if (new_dac_value > 4095)
			dac_value = 4095;
		else if (new_dac_value < 0)
			dac_value = 0;
		else                     
			dac_value = new_dac_value;
	
		LOGP(DCALIB, LOGL_NOTICE,
			"Going to apply %d as new clock setting.\n",
			dac_value);
	
		rc = lc15bts_clock_dac_set(dac_value);
		if (rc < 0) {
			LOGP(DCALIB, LOGL_ERROR,
				"Failed to set OCXO dac value %d\n", rc);
			calib_state_reset(mgr, CALIB_FAIL_OCXODAC);
			return;
        	}
		rc = lc15bts_clock_err_reset();
		if (rc < 0) {
			LOGP(DCALIB, LOGL_ERROR,
				"Failed to set reset clock error module %d\n", rc);
				calib_state_reset(mgr, CALIB_FAIL_CLKERR);
			return;
		}
	} 

	/* Save the correction value in the DAC eeprom if the 
	frequency has been stable for 24 hours */
	else if (interval_sec >= (24 * 60 * 60)) {
		rc = lc15bts_clock_dac_save();
		if (rc < 0) {
                	LOGP(DCALIB, LOGL_ERROR,
                        	"Failed to save OCXO dac value %d\n", rc);
			calib_state_reset(mgr, CALIB_FAIL_OCXODAC);
		}
		rc = lc15bts_clock_err_reset();
		if (rc < 0) {
                	LOGP(DCALIB, LOGL_ERROR,
                        	"Failed to set reste clock error module %d\n", rc);
			calib_state_reset(mgr, CALIB_FAIL_CLKERR);
		}
	}

	calib_state_reset(mgr, CALIB_SUCCESS);
	return;
}

static void calib_close(struct lc15bts_mgr_instance *mgr)
{
	lc15bts_clock_err_close();
	lc15bts_clock_dac_close();
}

static void calib_state_reset(struct lc15bts_mgr_instance *mgr, int outcome)
{
	if (mgr->calib.calib_from_loop) {
		/*
		 * In case of success calibrate in two hours again
		 * and in case of a failure in some minutes.
		 *
		 * TODO NTQ: Select timeout based on last error and accuracy
		 */
		int timeout = 60;
		//int timeout = 2 * 60 * 60;
		//if (outcome != CALIB_SUCESS) }
		//	timeout = 5 * 60;
		//}

                mgr->calib.calib_timeout.data = mgr;
                mgr->calib.calib_timeout.cb = calib_loop_run;
                osmo_timer_schedule(&mgr->calib.calib_timeout, timeout, 0);
        }

        mgr->calib.state = CALIB_INITIAL;
	calib_close(mgr);
}

static int calib_run(struct lc15bts_mgr_instance *mgr, int from_loop)
{
	if (mgr->calib.state != CALIB_INITIAL) {
		LOGP(DCALIB, LOGL_ERROR, "Calib is already in progress.\n");
		return -1;
	}

	mgr->calib.calib_from_loop = from_loop;

	/* From now on everything will be handled from the failure */
	mgr->calib.state = CALIB_IN_PROGRESS;
	calib_start(mgr);
	return 0;
}

static void calib_loop_run(void *_data)
{
        int rc;
        struct lc15bts_mgr_instance *mgr = _data;

        LOGP(DCALIB, LOGL_NOTICE, "Going to calibrate the system.\n");
        rc = calib_run(mgr, 1);
        if (rc != 0) {
                calib_state_reset(mgr, CALIB_FAIL_START);
	}
}

int lc15bts_mgr_calib_run(struct lc15bts_mgr_instance *mgr)
{
        return calib_run(mgr, 0);
}

int lc15bts_mgr_calib_init(struct lc15bts_mgr_instance *mgr)
{
	mgr->calib.state = CALIB_INITIAL;
	mgr->calib.calib_timeout.data = mgr;
	mgr->calib.calib_timeout.cb = calib_loop_run;
	osmo_timer_schedule(&mgr->calib.calib_timeout, 0, 0);
        return 0;
}

