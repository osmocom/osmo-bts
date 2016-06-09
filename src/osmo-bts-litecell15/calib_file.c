/* NuRAN Wireless Litecell 1.5 BTS L1 calibration file routines*/

/* Copyright (C) 2015 by Yves Godin <support@nuranwireless.com>
 * Copyright (C) 2016 by Harald Welte <laforge@gnumonks.org>
 * 
 * Based on sysmoBTS:
 *     (C) 2012 by Harald Welte <laforge@gnumonks.org>
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
#include <errno.h>

#include <osmocom/core/utils.h>

#include <osmo-bts/gsm_data.h>
#include <osmo-bts/logging.h>

#include <nrw/litecell15/litecell15.h>
#include <nrw/litecell15/gsml1const.h>

#include "l1_if.h"
#include "lc15bts.h"
#include "utils.h"



struct calib_file_desc {
	const char *fname;
	int rx;
	int trx;
        int rxpath;
};

static const struct calib_file_desc calib_files[] = {
	{
		.fname = "calib_rx0a.conf",
		.rx = 1,
		.trx = 0,
		.rxpath = 0,
	}, {
		.fname = "calib_rx0b.conf",
		.rx = 1,
		.trx = 0,
		.rxpath = 1,
	}, {
		.fname = "calib_rx1a.conf",
		.rx = 1,
		.trx = 1,
		.rxpath = 0,
	}, {
		.fname = "calib_rx1b.conf",
		.rx = 1,
		.trx = 1,
		.rxpath = 1,
	}, {
		.fname = "calib_tx0.conf",
		.rx = 0,
		.trx = 0,
	}, {
		.fname = "calib_tx1.conf",
		.rx = 0,
		.trx = 1,
	},
};


static int calib_file_send(struct lc15l1_hdl *fl1h,
			   const struct calib_file_desc *desc);

/* determine next calibration file index based on supported bands */
static int get_next_calib_file_idx(struct lc15l1_hdl *fl1h, int last_idx)
{
	struct phy_link *plink = fl1h->phy_inst->phy_link;
        int i;

        for (i = last_idx+1; i < ARRAY_SIZE(calib_files); i++) {
                if (calib_files[i].trx == plink->num)
                        return i;
        }
        return -1;
}

static int calib_file_open(struct lc15l1_hdl *fl1h,
                           const struct calib_file_desc *desc)
{
	struct calib_send_state *st = &fl1h->st;
	char *calib_path = fl1h->phy_inst->u.lc15.calib_path;
        char fname[PATH_MAX];

	if (st->fp) {
		LOGP(DL1C, LOGL_NOTICE, "L1 calibration file was left opened !!\n");
		fclose(st->fp);
		st->fp = NULL;
	}

        fname[0] = '\0';
        snprintf(fname, sizeof(fname)-1, "%s/%s", calib_path, desc->fname);
        fname[sizeof(fname)-1] = '\0';

        st->fp = fopen(fname, "rb");
        if (!st->fp) {
                LOGP(DL1C, LOGL_ERROR,
                        "Failed to open '%s' for calibration data.\n", fname);
                return -1;
        }
	return 0;
}

static int calib_file_close(struct lc15l1_hdl *fl1h)
{
	struct calib_send_state *st = &fl1h->st;

	if (st->fp) {
		fclose(st->fp);
		st->fp = NULL;
	}
	return 0;
}

/* iteratively download the calibration data into the L1 */

static int calib_send_compl_cb(struct gsm_bts_trx *trx, struct msgb *l1_msg,
			       void *data);

/* send a chunk of calibration tabledata for a single specified file */
static int calib_file_send_next_chunk(struct lc15l1_hdl *fl1h)
{
	struct calib_send_state *st = &fl1h->st;
	Litecell15_Prim_t *prim;
	struct msgb *msg;
	size_t n;

	msg = sysp_msgb_alloc();
	prim = msgb_sysprim(msg);

	prim->id = Litecell15_PrimId_SetCalibTblReq;
	prim->u.setCalibTblReq.offset = (uint32_t)ftell(st->fp);
	n = fread(prim->u.setCalibTblReq.u8Data, 1, 
			sizeof(prim->u.setCalibTblReq.u8Data), st->fp);
	prim->u.setCalibTblReq.length = n;


	if (n == 0) {
		/* The table data has been completely sent and acknowledged */
                LOGP(DL1C, LOGL_NOTICE, "L1 calibration table %s loaded\n",
                        calib_files[st->last_file_idx].fname);

                calib_file_close(fl1h);

                msgb_free(msg);

                /* Send the next one if any */
                st->last_file_idx = get_next_calib_file_idx(fl1h, st->last_file_idx);
                if (st->last_file_idx >= 0) {
                        return calib_file_send(fl1h,
                                       &calib_files[st->last_file_idx]);
                }

                LOGP(DL1C, LOGL_INFO, "L1 calibration table loading complete!\n");
                return 0;
	}

	return l1if_req_compl(fl1h, msg, calib_send_compl_cb, NULL);
}

/* send the calibration table for a single specified file */
static int calib_file_send(struct lc15l1_hdl *fl1h,
			   const struct calib_file_desc *desc)
{
	struct calib_send_state *st = &fl1h->st;
	int rc;

	rc = calib_file_open(fl1h, desc);
	if (rc < 0) {
		/* still, we'd like to continue trying to load
                 * calibration for all other bands */
                st->last_file_idx = get_next_calib_file_idx(fl1h, st->last_file_idx);
                if (st->last_file_idx >= 0)
                        return calib_file_send(fl1h,
                                               &calib_files[st->last_file_idx]);

		LOGP(DL1C, LOGL_INFO, "L1 calibration table loading complete!\n");
                return 0;
	}

	return calib_file_send_next_chunk(fl1h);
}

/* completion callback after every SetCalibTbl is confirmed */
static int calib_send_compl_cb(struct gsm_bts_trx *trx, struct msgb *l1_msg,
				void *data)
{
	struct lc15l1_hdl *fl1h = trx_lc15l1_hdl(trx);
	struct calib_send_state *st = &fl1h->st;
	Litecell15_Prim_t *prim = msgb_sysprim(l1_msg);

	if (prim->u.setCalibTblCnf.status != GsmL1_Status_Success) {
		LOGP(DL1C, LOGL_ERROR, "L1 rejected calibration table\n");

		msgb_free(l1_msg);

		calib_file_close(fl1h);

		/* Skip this one and try the next one */
		st->last_file_idx = get_next_calib_file_idx(fl1h, st->last_file_idx);
	        if (st->last_file_idx >= 0) {
        	        return calib_file_send(fl1h, 
					&calib_files[st->last_file_idx]);
		}

		LOGP(DL1C, LOGL_INFO, "L1 calibration table loading complete!\n");
		return 0;
	}

	msgb_free(l1_msg);

	/* Keep sending the calibration file data */
	return calib_file_send_next_chunk(fl1h);
}

int calib_load(struct lc15l1_hdl *fl1h)
{
	int rc;
	struct calib_send_state *st = &fl1h->st;
	char *calib_path = fl1h->phy_inst->u.lc15.calib_path;

        if (!calib_path) {
                LOGP(DL1C, LOGL_ERROR, "Calibration file path not specified\n");
                return -1;
        }

	rc = get_next_calib_file_idx(fl1h, -1);
	if (rc < 0) {
		return -1;
	}
	st->last_file_idx = rc;

	return calib_file_send(fl1h, &calib_files[st->last_file_idx]);
}

