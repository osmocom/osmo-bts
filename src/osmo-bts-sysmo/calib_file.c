/* sysmocom femtobts L1 calibration file routines*/

/* (C) 2012 by Harald Welte <laforge@gnumonks.org>
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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <limits.h>

#include <osmocom/core/utils.h>

#include <osmo-bts/gsm_data.h>
#include <osmo-bts/logging.h>

#include <sysmocom/femtobts/superfemto.h>
#include <sysmocom/femtobts/gsml1const.h>

#include "l1_if.h"
#include "femtobts.h"

struct calib_file_desc {
	const char *fname;
	GsmL1_FreqBand_t band;
	int uplink;
	int rx;
};

static const struct calib_file_desc calib_files[] = {
	{
		.fname = "calib_rxu_850.cfg",
		.band = GsmL1_FreqBand_850,
		.uplink = 1,
		.rx = 1,
	}, {
		.fname = "calib_rxu_900.cfg",
		.band = GsmL1_FreqBand_900,
		.uplink = 1,
		.rx = 1,
	}, {
		.fname = "calib_rxu_1800.cfg",
		.band = GsmL1_FreqBand_1800,
		.uplink = 1,
		.rx = 1,
	}, {
		.fname = "calib_rxu_1900.cfg",
		.band = GsmL1_FreqBand_1900,
		.uplink = 1,
		.rx = 1,
	}, {
		.fname = "calib_rxd_850.cfg",
		.band = GsmL1_FreqBand_850,
		.uplink = 0,
		.rx = 1,
	}, {
		.fname = "calib_rxd_900.cfg",
		.band = GsmL1_FreqBand_900,
		.uplink = 0,
		.rx = 1,
	}, {
		.fname = "calib_rxd_1800.cfg",
		.band = GsmL1_FreqBand_1800,
		.uplink = 0,
		.rx = 1,
	}, {
		.fname = "calib_rxd_1900.cfg",
		.band = GsmL1_FreqBand_1900,
		.uplink = 0,
		.rx = 1,
	}, {
		.fname = "calib_tx_850.cfg",
		.band = GsmL1_FreqBand_850,
		.uplink = 0,
		.rx = 0,
	}, {
		.fname = "calib_tx_900.cfg",
		.band = GsmL1_FreqBand_900,
		.uplink = 0,
		.rx = 0,
	}, {
		.fname = "calib_tx_1800.cfg",
		.band = GsmL1_FreqBand_1800,
		.uplink = 0,
		.rx = 0,
	}, {
		.fname = "calib_tx_1900.cfg",
		.band = GsmL1_FreqBand_1900,
		.uplink = 0,
		.rx = 0,

	},
};

static const unsigned int arrsize_by_band[] = {
	[GsmL1_FreqBand_850] = 124,
	[GsmL1_FreqBand_900] = 194,
	[GsmL1_FreqBand_1800] = 374,
	[GsmL1_FreqBand_1900] = 299
};


static float read_float(FILE *in)
{
	float f;
	fscanf(in, "%f\n", &f);
	return f;
}

static int read_int(FILE *in)
{
	int i;
	fscanf(in, "%d\n", &i);
	return i;
}

int calib_file_read(const char *path, const struct calib_file_desc *desc,
		    SuperFemto_Prim_t *prim)
{
	FILE *in;
	char fname[PATH_MAX];
	int i;

	fname[0] = '\0';
	snprintf(fname, sizeof(fname)-1, "%s/%s", path, desc->fname);
	fname[sizeof(fname)-1] = '\0';

	in = fopen(fname, "r");
	if (!in)
		return -1;

#if SUPERFEMTO_API_VERSION >= SUPERFEMTO_API(2,4,0)
	if (desc->rx) {
		SuperFemto_SetRxCalibTblReq_t *rx = &prim->u.setRxCalibTblReq;
		memset(rx, 0, sizeof(*rx));

		prim->id = SuperFemto_PrimId_SetRxCalibTblReq;

		rx->freqBand = desc->band;
		rx->bUplink = desc->uplink;

		rx->fExtRxGain = read_float(in);
		rx->fRxMixGainCorr = read_float(in);

		for (i = 0; i < ARRAY_SIZE(rx->fRxLnaGainCorr); i++)
			rx->fRxLnaGainCorr[i] = read_float(in);

		for (i = 0; i < arrsize_by_band[desc->band]; i++)
			rx->fRxRollOffCorr[i] = read_float(in);

		if (desc->uplink) {
			rx->u8IqImbalMode = read_int(in);
			printf("%s: u8IqImbalMode=%d\n", desc->fname, rx->u8IqImbalMode);

			for (i = 0; i < ARRAY_SIZE(rx->u16IqImbalCorr); i++)
				rx->u16IqImbalCorr[i] = read_int(in);
		}
	} else {
		SuperFemto_SetTxCalibTblReq_t *tx = &prim->u.setTxCalibTblReq;
		memset(tx, 0, sizeof(*tx));

		prim->id = SuperFemto_PrimId_SetTxCalibTblReq;

		tx->freqBand = desc->band;

		for (i = 0; i < ARRAY_SIZE(tx->fTxGainGmsk); i++)
			tx->fTxGainGmsk[i] = read_float(in);

		tx->fTx8PskCorr = read_float(in);

		for (i = 0; i < ARRAY_SIZE(tx->fTxExtAttCorr); i++)
			tx->fTxExtAttCorr[i] = read_float(in);

		for (i = 0; i < arrsize_by_band[desc->band]; i++)
			tx->fTxRollOffCorr[i] = read_float(in);
	}
#else
#warning Format of calibration tables before API version 2.4.0 not supported
#endif
	fclose(in);

	return 0;
}

/* iteratively download the calibration data into the L1 */

static int calib_send_compl_cb(struct msgb *l1_msg, void *data);

/* send the calibration table for a single specified file */
static int calib_file_send(struct femtol1_hdl *fl1h,
			   const struct calib_file_desc *desc)
{
	struct msgb *msg;
	int rc;

	msg = sysp_msgb_alloc();

	rc = calib_file_read(fl1h->calib_path, desc, msgb_sysprim(msg));
	if (rc < 0) {
		msgb_free(msg);
		return rc;
	}

	return l1if_req_compl(fl1h, msg, 1, calib_send_compl_cb, fl1h->priv);
}

/* completion callback after every SetCalibTbl is confirmed */
static int calib_send_compl_cb(struct msgb *l1_msg, void *data)
{
	struct gsm_bts_trx *trx = data;
	struct femtol1_hdl *fl1h = trx_femtol1_hdl(trx);
	struct calib_send_state *st = &fl1h->st;

	LOGP(DL1C, LOGL_DEBUG, "L1 calibration table %s loaded\n",
		calib_files[st->last_file_idx].fname);

	st->last_file_idx++;

	if (st->last_file_idx < ARRAY_SIZE(calib_files))
		return calib_file_send(fl1h,
				       &calib_files[st->last_file_idx]);

	LOGP(DL1C, LOGL_INFO, "L1 calibration table loading complete!\n");

	return 0;
}


int calib_load(struct femtol1_hdl *fl1h)
{
#if SUPERFEMTO_API_VERSION < SUPERFEMTO_API(2,4,0)
	return -1;
#else
	return calib_file_send(fl1h, &calib_files[0]);
#endif
}


#if 0
int main(int argc, char **argv)
{
	SuperFemto_Prim_t p;
	int i;

	for (i = 0; i < ARRAY_SIZE(calib_files); i++) {
		memset(&p, 0, sizeof(p));
		calib_read_file(argv[1], &calib_files[i], &p);
	}
	exit(0);
}
#endif
