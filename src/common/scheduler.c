/* Scheduler for OsmoBTS-TRX */

/* (C) 2013 by Andreas Eversberg <jolly@eversberg.eu>
 * (C) 2015 by Alexander Chemeris <Alexander.Chemeris@fairwaves.co>
 * (C) 2015 by Harald Welte <laforge@gnumonks.org>
 * Contributions by sysmocom - s.f.m.c. GmbH
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
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <stdint.h>
#include <ctype.h>

#include <osmocom/core/msgb.h>
#include <osmocom/core/talloc.h>
#include <osmocom/core/bits.h>
#include <osmocom/core/utils.h>
#include <osmocom/core/rate_ctr.h>
#include <osmocom/core/stats.h>

#include <osmocom/gsm/protocol/gsm_08_58.h>
#include <osmocom/gsm/gsm0502.h>
#include <osmocom/gsm/a5.h>

#include <osmo-bts/gsm_data.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/rsl.h>
#include <osmo-bts/l1sap.h>
#include <osmo-bts/scheduler.h>
#include <osmo-bts/scheduler_backend.h>
#include <osmo-bts/bts.h>

extern void *tall_bts_ctx;

static int rts_data_fn(const struct l1sched_ts *l1ts, const struct trx_dl_burst_req *br);
static int rts_tchf_fn(const struct l1sched_ts *l1ts, const struct trx_dl_burst_req *br);
static int rts_tchh_fn(const struct l1sched_ts *l1ts, const struct trx_dl_burst_req *br);

/*! \brief Dummy Burst (TS 05.02 Chapter 5.2.6) */
const ubit_t _sched_dummy_burst[GSM_BURST_LEN] = {
	0,0,0,
	1,1,1,1,1,0,1,1,0,1,1,1,0,1,1,0,0,0,0,0,1,0,1,0,0,1,0,0,1,1,1,0,
	0,0,0,0,1,0,0,1,0,0,0,1,0,0,0,0,0,0,0,1,1,1,1,1,0,0,0,1,1,1,0,0,
	0,1,0,1,1,1,0,0,0,1,0,1,1,1,0,0,0,1,0,1,0,1,1,1,0,1,0,0,1,0,1,0,
	0,0,1,1,0,0,1,1,0,0,1,1,1,0,0,1,1,1,1,0,1,0,0,1,1,1,1,1,0,0,0,1,
	0,0,1,0,1,1,1,1,1,0,1,0,1,0,
	0,0,0,
};

/*! Training Sequences for Normal Burst (see 3GPP TS 45.002, section 5.2.3) */
const ubit_t _sched_train_seq_gmsk_nb[4][8][26] = {
	{ /* TSC set 1, table 5.2.3a */
		{ 0,0,1,0,0,1,0,1,1,1,0,0,0,0,1,0,0,0,1,0,0,1,0,1,1,1 },
		{ 0,0,1,0,1,1,0,1,1,1,0,1,1,1,1,0,0,0,1,0,1,1,0,1,1,1 },
		{ 0,1,0,0,0,0,1,1,1,0,1,1,1,0,1,0,0,1,0,0,0,0,1,1,1,0 },
		{ 0,1,0,0,0,1,1,1,1,0,1,1,0,1,0,0,0,1,0,0,0,1,1,1,1,0 },
		{ 0,0,0,1,1,0,1,0,1,1,1,0,0,1,0,0,0,0,0,1,1,0,1,0,1,1 },
		{ 0,1,0,0,1,1,1,0,1,0,1,1,0,0,0,0,0,1,0,0,1,1,1,0,1,0 },
		{ 1,0,1,0,0,1,1,1,1,1,0,1,1,0,0,0,1,0,1,0,0,1,1,1,1,1 },
		{ 1,1,1,0,1,1,1,1,0,0,0,1,0,0,1,0,1,1,1,0,1,1,1,1,0,0 },
	},
	{ /* TSC set 2, table 5.2.3b */
		{ 0,1,1,0,0,0,1,0,0,0,1,0,0,1,0,0,1,1,1,1,0,1,0,1,1,1 },
		{ 0,1,0,1,1,1,1,0,1,0,0,1,1,0,1,1,1,0,1,1,1,0,0,0,0,1 },
		{ 0,1,0,0,0,0,0,1,0,1,1,0,0,0,1,1,1,0,1,1,1,0,1,1,0,0 },
		{ 0,0,1,0,1,1,0,1,1,1,0,1,1,1,0,0,1,1,1,1,0,1,0,0,0,0 },
		{ 0,1,1,1,0,1,0,0,1,1,1,1,0,1,0,0,1,1,1,0,1,1,1,1,1,0 },
		{ 0,1,0,0,0,0,0,1,0,0,1,1,0,1,0,1,0,0,1,1,1,1,0,0,1,1 },
		{ 0,0,0,1,0,0,0,0,1,1,0,1,0,0,0,0,1,1,0,1,1,1,0,1,0,1 },
		{ 0,1,0,0,0,1,0,1,1,1,0,0,1,1,1,1,1,1,0,0,1,0,1,0,0,1 },
	},
	{ /* TSC set 3, table 5.2.3c */
		{ 1,1,0,0,0,0,1,0,0,1,0,0,0,1,1,1,1,0,1,0,1,0,0,0,1,0 },
		{ 0,0,1,0,1,1,1,1,1,0,0,0,1,0,0,1,0,1,0,0,0,0,1,0,0,0 },
		{ 1,1,0,0,1,0,0,0,1,1,1,1,1,0,1,1,1,0,1,0,1,1,0,1,1,0 },
		{ 0,0,1,1,0,0,0,0,1,0,1,0,0,1,1,0,0,0,0,0,1,0,1,1,0,0 },
		{ 0,0,0,1,1,1,1,0,1,0,1,1,1,0,1,0,0,0,0,1,0,0,0,1,1,0 },
		{ 1,1,0,0,1,1,1,1,0,1,0,1,0,1,1,1,1,0,0,1,0,0,0,0,0,0 },
		{ 1,0,1,1,1,0,0,1,1,0,1,0,1,1,1,1,1,1,0,0,0,1,0,0,0,0 },
		{ 1,1,1,0,0,1,0,1,1,1,1,0,1,1,1,0,0,0,0,0,1,0,0,1,0,0 },
	},
	{ /* TSC set 4, table 5.2.3d */
		{ 1,1,0,0,1,1,1,0,1,0,0,0,0,0,1,0,0,0,1,1,0,1,0,0,0,0 },
		{ 0,1,1,0,0,0,1,0,0,0,0,1,0,1,0,0,0,1,0,1,1,1,0,0,0,0 },
		{ 1,1,1,0,0,1,0,0,0,0,0,1,0,1,0,1,0,0,1,1,1,0,0,0,0,0 },
		{ 0,1,1,0,1,1,0,0,1,1,1,1,1,0,1,0,1,0,0,0,0,1,1,0,0,0 },
		{ 1,1,0,1,1,0,0,0,0,1,0,0,0,0,1,0,0,0,1,0,1,1,0,0,0,0 },
		{ 1,1,0,1,0,0,1,1,1,1,1,1,1,0,1,0,0,0,1,1,0,1,0,1,1,0 },
		{ 0,0,1,0,0,1,1,1,1,1,1,1,0,0,1,0,1,0,1,0,1,1,0,0,0,0 },
		{ 0,1,0,1,1,1,0,0,0,0,0,0,1,0,1,0,0,1,1,0,0,0,1,1,1,0 },
	},
};

const ubit_t _sched_train_seq_8psk_nb[8][78] = {
	{ 1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,0,0,1,1,1,1,0,0,1,0,0,
	  1,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,1,
	  1,1,0,0,1,1,1,1,1,1,1,0,0,1,1,1,1,0,0,1,0,0,1,0,0,1, },
	{ 1,1,1,1,1,1,0,0,1,1,1,1,0,0,1,0,0,1,1,1,1,0,0,1,0,0,
	  1,0,0,1,1,1,1,0,0,1,0,0,1,0,0,1,0,0,1,1,1,1,1,1,1,1,
	  1,1,0,0,1,1,1,1,0,0,1,0,0,1,1,1,1,0,0,1,0,0,1,0,0,1, },
	{ 1,1,1,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,1,0,0,1,0,0,
	  1,1,1,1,0,0,1,0,0,1,0,0,1,1,1,1,0,0,1,1,1,1,1,1,1,0,
	  0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,1,0,0,1,0,0,1,1,1,1, },
	{ 1,1,1,0,0,1,1,1,1,1,1,1,1,1,1,0,0,1,0,0,1,0,0,1,0,0,
	  1,1,1,1,0,0,1,0,0,1,1,1,1,0,0,1,1,1,1,1,1,1,1,1,1,0,
	  0,1,1,1,1,1,1,1,1,1,1,0,0,1,0,0,1,0,0,1,0,0,1,1,1,1, },
	{ 1,1,1,1,1,1,1,1,1,0,0,1,0,0,1,1,1,1,0,0,1,1,1,1,0,0,
	  1,0,0,1,0,0,1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,1,1,1,1,
	  1,1,1,1,1,0,0,1,0,0,1,1,1,1,0,0,1,1,1,1,0,0,1,0,0,1, },
	{ 1,1,1,0,0,1,1,1,1,1,1,1,0,0,1,0,0,1,0,0,1,1,1,1,0,0,
	  1,1,1,1,0,0,1,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
	  0,1,1,1,1,1,1,1,0,0,1,0,0,1,0,0,1,1,1,1,0,0,1,1,1,1, },
	{ 0,0,1,1,1,1,0,0,1,1,1,1,1,1,1,0,0,1,0,0,1,0,0,1,0,0,
	  1,0,0,1,1,1,1,0,0,1,0,0,1,1,1,1,1,1,1,1,1,1,0,0,1,1,
	  1,1,0,0,1,1,1,1,1,1,1,0,0,1,0,0,1,0,0,1,0,0,1,0,0,1, },
	{ 0,0,1,0,0,1,0,0,1,1,1,1,0,0,1,0,0,1,0,0,1,0,0,1,1,1,
	  1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,0,0,1,1,1,1,0,0,1,0,
	  0,1,0,0,1,1,1,1,0,0,1,0,0,1,0,0,1,0,0,1,1,1,1,1,1,1, },
};

/*! \brief SCH training sequence (TS 05.02 Chapter 5.2.5) */
const ubit_t _sched_train_seq_gmsk_sb[64] = {
	1,0,1,1,1,0,0,1,0,1,1,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,1,1,1,
	0,0,1,0,1,1,0,1,0,1,0,0,0,1,0,1,0,1,1,1,0,1,1,0,0,0,0,1,1,0,1,1,
};

/* Logical channel (TRXC_*) description */
const struct trx_chan_desc trx_chan_desc[_TRX_CHAN_MAX] = {
	[TRXC_IDLE] = {
		.name = "IDLE",
		.desc = "Idle channel",
	},
	[TRXC_FCCH] = {
		.name = "FCCH", /* 3GPP TS 05.02, section 3.3.2.1 */
		.desc = "Frequency correction channel",

		/* Tx only, frequency correction bursts */
		.dl_fn = tx_fcch_fn,
	},
	[TRXC_SCH] = {
		.name = "SCH", /* 3GPP TS 05.02, section 3.3.2.2 */
		.desc = "Synchronization channel",

		/* Tx only, synchronization bursts */
		.dl_fn = tx_sch_fn,
	},
	[TRXC_BCCH] = {
		.name = "BCCH", /* 3GPP TS 05.02, section 3.3.2.3 */
		.desc = "Broadcast control channel",
		.chan_nr = RSL_CHAN_BCCH,

		/* Tx only, xCCH convolutional coding (3GPP TS 05.03, section 4.4),
		 * regular interleaving (3GPP TS 05.02, clause 7, table 3):
		 * a L2 frame is interleaved over 4 consecutive bursts. */
		.rts_fn = rts_data_fn,
		.dl_fn = tx_data_fn,
	},
	[TRXC_RACH] = {
		.name = "RACH", /* 3GPP TS 05.02, section 3.3.3.1 */
		.desc = "Random access channel",
		.chan_nr = RSL_CHAN_RACH,

		/* Rx only, RACH convolutional coding (3GPP TS 05.03, section 4.6). */
		.ul_fn = rx_rach_fn,
	},
	[TRXC_CCCH] = {
		.name = "CCCH", /* 3GPP TS 05.02, section 3.3.3.1 */
		.desc = "Common control channel",
		.chan_nr = RSL_CHAN_PCH_AGCH,

		/* Tx only, xCCH convolutional coding (3GPP TS 05.03, section 4.4),
		 * regular interleaving (3GPP TS 05.02, clause 7, table 3):
		 * a L2 frame is interleaved over 4 consecutive bursts. */
		.rts_fn = rts_data_fn,
		.dl_fn = tx_data_fn,
	},
	[TRXC_TCHF] = {
		.name = "TCH/F", /* 3GPP TS 05.02, section 3.2 */
		.desc = "Full Rate traffic channel",
		.chan_nr = RSL_CHAN_Bm_ACCHs,
		.link_id = LID_DEDIC,

		/* Rx and Tx, multiple convolutional coding types (3GPP TS 05.03,
		 * chapter 3), block diagonal interleaving (3GPP TS 05.02, clause 7):
		 *
		 *   - a traffic frame is interleaved over 8 consecutive bursts
		 *     using the even numbered bits of the first 4 bursts
		 *     and odd numbered bits of the last 4 bursts;
		 *   - a FACCH/F frame 'steals' (replaces) one traffic frame,
		 *     interleaving is done in the same way. */
		.rts_fn = rts_tchf_fn,
		.dl_fn = tx_tchf_fn,
		.ul_fn = rx_tchf_fn,
	},
	[TRXC_TCHH_0] = {
		.name = "TCH/H(0)", /* 3GPP TS 05.02, section 3.2 */
		.desc = "Half Rate traffic channel (sub-channel 0)",
		.chan_nr = RSL_CHAN_Lm_ACCHs + (0 << 3),
		.link_id = LID_DEDIC,

		/* Rx and Tx, multiple convolutional coding types (3GPP TS 05.03,
		 * chapter 3), block diagonal interleaving (3GPP TS 05.02, clause 7):
		 *
		 *   - a traffic frame is interleaved over 4 consecutive bursts
		 *     using the even numbered bits of the first 2 bursts,
		 *     and odd numbered bits of the last 2 bursts;
		 *   - a FACCH/H frame 'steals' (replaces) two traffic frames,
		 *     interleaving is done over 6 consecutive bursts,
		 *     using the even numbered bits of the first 2 bursts,
		 *     all bits of the middle two 2 bursts,
		 *     and odd numbered bits of the last 2 bursts. */
		.rts_fn = rts_tchh_fn,
		.dl_fn = tx_tchh_fn,
		.ul_fn = rx_tchh_fn,
	},
	[TRXC_TCHH_1] = {
		.name = "TCH/H(1)", /* 3GPP TS 05.02, section 3.2 */
		.desc = "Half Rate traffic channel (sub-channel 1)",
		.chan_nr = RSL_CHAN_Lm_ACCHs + (1 << 3),
		.link_id = LID_DEDIC,

		/* Same as for TRXC_TCHH_0, see above. */
		.rts_fn = rts_tchh_fn,
		.dl_fn = tx_tchh_fn,
		.ul_fn = rx_tchh_fn,
	},
	[TRXC_SDCCH4_0] = {
		.name = "SDCCH/4(0)", /* 3GPP TS 05.02, section 3.3.4.1 */
		.desc = "Stand-alone dedicated control channel (sub-channel 0)",
		.chan_nr = RSL_CHAN_SDCCH4_ACCH + (0 << 3),
		.link_id = LID_DEDIC,

		/* Same as for TRXC_BCCH (xCCH), see above. */
		.rts_fn = rts_data_fn,
		.dl_fn = tx_data_fn,
		.ul_fn = rx_data_fn,
	},
	[TRXC_SDCCH4_1] = {
		.name = "SDCCH/4(1)", /* 3GPP TS 05.02, section 3.3.4.1 */
		.desc = "Stand-alone dedicated control channel (sub-channel 1)",
		.chan_nr = RSL_CHAN_SDCCH4_ACCH + (1 << 3),
		.link_id = LID_DEDIC,

		/* Same as for TRXC_BCCH (xCCH), see above. */
		.rts_fn = rts_data_fn,
		.dl_fn = tx_data_fn,
		.ul_fn = rx_data_fn,
	},
	[TRXC_SDCCH4_2] = {
		.name = "SDCCH/4(2)", /* 3GPP TS 05.02, section 3.3.4.1 */
		.desc = "Stand-alone dedicated control channel (sub-channel 2)",
		.chan_nr = RSL_CHAN_SDCCH4_ACCH + (2 << 3),
		.link_id = LID_DEDIC,

		/* Same as for TRXC_BCCH (xCCH), see above. */
		.rts_fn = rts_data_fn,
		.dl_fn = tx_data_fn,
		.ul_fn = rx_data_fn,
	},
	[TRXC_SDCCH4_3] = {
		.name = "SDCCH/4(3)", /* 3GPP TS 05.02, section 3.3.4.1 */
		.desc = "Stand-alone dedicated control channel (sub-channel 3)",
		.chan_nr = RSL_CHAN_SDCCH4_ACCH + (3 << 3),
		.link_id = LID_DEDIC,

		/* Same as for TRXC_BCCH (xCCH), see above. */
		.rts_fn = rts_data_fn,
		.dl_fn = tx_data_fn,
		.ul_fn = rx_data_fn,
	},
	[TRXC_SDCCH8_0] = {
		.name = "SDCCH/8(0)", /* 3GPP TS 05.02, section 3.3.4.1 */
		.desc = "Stand-alone dedicated control channel (sub-channel 0)",
		.chan_nr = RSL_CHAN_SDCCH8_ACCH + (0 << 3),
		.link_id = LID_DEDIC,

		/* Same as for TRXC_BCCH and TRXC_SDCCH4_* (xCCH), see above. */
		.rts_fn = rts_data_fn,
		.dl_fn = tx_data_fn,
		.ul_fn = rx_data_fn,
	},
	[TRXC_SDCCH8_1] = {
		.name = "SDCCH/8(1)", /* 3GPP TS 05.02, section 3.3.4.1 */
		.desc = "Stand-alone dedicated control channel (sub-channel 1)",
		.chan_nr = RSL_CHAN_SDCCH8_ACCH + (1 << 3),
		.link_id = LID_DEDIC,

		/* Same as for TRXC_BCCH and TRXC_SDCCH4_* (xCCH), see above. */
		.rts_fn = rts_data_fn,
		.dl_fn = tx_data_fn,
		.ul_fn = rx_data_fn,
	},
	[TRXC_SDCCH8_2] = {
		.name = "SDCCH/8(2)", /* 3GPP TS 05.02, section 3.3.4.1 */
		.desc = "Stand-alone dedicated control channel (sub-channel 2)",
		.chan_nr = RSL_CHAN_SDCCH8_ACCH + (2 << 3),
		.link_id = LID_DEDIC,

		/* Same as for TRXC_BCCH and TRXC_SDCCH4_* (xCCH), see above. */
		.rts_fn = rts_data_fn,
		.dl_fn = tx_data_fn,
		.ul_fn = rx_data_fn,
	},
	[TRXC_SDCCH8_3] = {
		.name = "SDCCH/8(3)", /* 3GPP TS 05.02, section 3.3.4.1 */
		.desc = "Stand-alone dedicated control channel (sub-channel 3)",
		.chan_nr = RSL_CHAN_SDCCH8_ACCH + (3 << 3),
		.link_id = LID_DEDIC,

		/* Same as for TRXC_BCCH and TRXC_SDCCH4_* (xCCH), see above. */
		.rts_fn = rts_data_fn,
		.dl_fn = tx_data_fn,
		.ul_fn = rx_data_fn,
	},
	[TRXC_SDCCH8_4] = {
		.name = "SDCCH/8(4)", /* 3GPP TS 05.02, section 3.3.4.1 */
		.desc = "Stand-alone dedicated control channel (sub-channel 4)",
		.chan_nr = RSL_CHAN_SDCCH8_ACCH + (4 << 3),
		.link_id = LID_DEDIC,

		/* Same as for TRXC_BCCH and TRXC_SDCCH4_* (xCCH), see above. */
		.rts_fn = rts_data_fn,
		.dl_fn = tx_data_fn,
		.ul_fn = rx_data_fn,
	},
	[TRXC_SDCCH8_5] = {
		.name = "SDCCH/8(5)", /* 3GPP TS 05.02, section 3.3.4.1 */
		.desc = "Stand-alone dedicated control channel (sub-channel 5)",
		.chan_nr = RSL_CHAN_SDCCH8_ACCH + (5 << 3),
		.link_id = LID_DEDIC,

		/* Same as for TRXC_BCCH and TRXC_SDCCH4_* (xCCH), see above. */
		.rts_fn = rts_data_fn,
		.dl_fn = tx_data_fn,
		.ul_fn = rx_data_fn,
	},
	[TRXC_SDCCH8_6] = {
		.name = "SDCCH/8(6)", /* 3GPP TS 05.02, section 3.3.4.1 */
		.desc = "Stand-alone dedicated control channel (sub-channel 6)",
		.chan_nr = RSL_CHAN_SDCCH8_ACCH + (6 << 3),
		.link_id = LID_DEDIC,

		/* Same as for TRXC_BCCH and TRXC_SDCCH4_* (xCCH), see above. */
		.rts_fn = rts_data_fn,
		.dl_fn = tx_data_fn,
		.ul_fn = rx_data_fn,
	},
	[TRXC_SDCCH8_7] = {
		.name = "SDCCH/8(7)", /* 3GPP TS 05.02, section 3.3.4.1 */
		.desc = "Stand-alone dedicated control channel (sub-channel 7)",
		.chan_nr = RSL_CHAN_SDCCH8_ACCH + (7 << 3),
		.link_id = LID_DEDIC,

		/* Same as for TRXC_BCCH and TRXC_SDCCH4_* (xCCH), see above. */
		.rts_fn = rts_data_fn,
		.dl_fn = tx_data_fn,
		.ul_fn = rx_data_fn,
	},
	[TRXC_SACCHTF] = {
		.name = "SACCH/TF", /* 3GPP TS 05.02, section 3.3.4.1 */
		.desc = "Slow TCH/F associated control channel",
		.chan_nr = RSL_CHAN_Bm_ACCHs,
		.link_id = LID_SACCH,

		/* Same as for TRXC_BCCH (xCCH), see above. */
		.rts_fn = rts_data_fn,
		.dl_fn = tx_data_fn,
		.ul_fn = rx_data_fn,
	},
	[TRXC_SACCHTH_0] = {
		.name = "SACCH/TH(0)", /* 3GPP TS 05.02, section 3.3.4.1 */
		.desc = "Slow TCH/H associated control channel (sub-channel 0)",
		.chan_nr = RSL_CHAN_Lm_ACCHs + (0 << 3),
		.link_id = LID_SACCH,

		/* Same as for TRXC_BCCH (xCCH), see above. */
		.rts_fn = rts_data_fn,
		.dl_fn = tx_data_fn,
		.ul_fn = rx_data_fn,
	},
	[TRXC_SACCHTH_1] = {
		.name = "SACCH/TH(1)", /* 3GPP TS 05.02, section 3.3.4.1 */
		.desc = "Slow TCH/H associated control channel (sub-channel 1)",
		.chan_nr = RSL_CHAN_Lm_ACCHs + (1 << 3),
		.link_id = LID_SACCH,

		/* Same as for TRXC_BCCH (xCCH), see above. */
		.rts_fn = rts_data_fn,
		.dl_fn = tx_data_fn,
		.ul_fn = rx_data_fn,
	},
	[TRXC_SACCH4_0] = {
		.name = "SACCH/4(0)", /* 3GPP TS 05.02, section 3.3.4.1 */
		.desc = "Slow SDCCH/4 associated control channel (sub-channel 0)",
		.chan_nr = RSL_CHAN_SDCCH4_ACCH + (0 << 3),
		.link_id = LID_SACCH,

		/* Same as for TRXC_BCCH and TRXC_SDCCH4_* (xCCH), see above. */
		.rts_fn = rts_data_fn,
		.dl_fn = tx_data_fn,
		.ul_fn = rx_data_fn,
	},
	[TRXC_SACCH4_1] = {
		.name = "SACCH/4(1)", /* 3GPP TS 05.02, section 3.3.4.1 */
		.desc = "Slow SDCCH/4 associated control channel (sub-channel 1)",
		.chan_nr = RSL_CHAN_SDCCH4_ACCH + (1 << 3),
		.link_id = LID_SACCH,

		/* Same as for TRXC_BCCH and TRXC_SDCCH4_* (xCCH), see above. */
		.rts_fn = rts_data_fn,
		.dl_fn = tx_data_fn,
		.ul_fn = rx_data_fn,
	},
	[TRXC_SACCH4_2] = {
		.name = "SACCH/4(2)", /* 3GPP TS 05.02, section 3.3.4.1 */
		.desc = "Slow SDCCH/4 associated control channel (sub-channel 2)",
		.chan_nr = RSL_CHAN_SDCCH4_ACCH + (2 << 3),
		.link_id = LID_SACCH,

		/* Same as for TRXC_BCCH and TRXC_SDCCH4_* (xCCH), see above. */
		.rts_fn = rts_data_fn,
		.dl_fn = tx_data_fn,
		.ul_fn = rx_data_fn,
	},
	[TRXC_SACCH4_3] = {
		.name = "SACCH/4(3)", /* 3GPP TS 05.02, section 3.3.4.1 */
		.desc = "Slow SDCCH/4 associated control channel (sub-channel 3)",
		.chan_nr = RSL_CHAN_SDCCH4_ACCH + (3 << 3),
		.link_id = LID_SACCH,

		/* Same as for TRXC_BCCH and TRXC_SDCCH4_* (xCCH), see above. */
		.rts_fn = rts_data_fn,
		.dl_fn = tx_data_fn,
		.ul_fn = rx_data_fn,
	},
	[TRXC_SACCH8_0] = {
		.name = "SACCH/8(0)", /* 3GPP TS 05.02, section 3.3.4.1 */
		.desc = "Slow SDCCH/8 associated control channel (sub-channel 0)",
		.chan_nr = RSL_CHAN_SDCCH8_ACCH + (0 << 3),
		.link_id = LID_SACCH,

		/* Same as for TRXC_BCCH and TRXC_SDCCH8_* (xCCH), see above. */
		.rts_fn = rts_data_fn,
		.dl_fn = tx_data_fn,
		.ul_fn = rx_data_fn,
	},
	[TRXC_SACCH8_1] = {
		.name = "SACCH/8(1)", /* 3GPP TS 05.02, section 3.3.4.1 */
		.desc = "Slow SDCCH/8 associated control channel (sub-channel 1)",
		.chan_nr = RSL_CHAN_SDCCH8_ACCH + (1 << 3),
		.link_id = LID_SACCH,

		/* Same as for TRXC_BCCH and TRXC_SDCCH8_* (xCCH), see above. */
		.rts_fn = rts_data_fn,
		.dl_fn = tx_data_fn,
		.ul_fn = rx_data_fn,
	},
	[TRXC_SACCH8_2] = {
		.name = "SACCH/8(2)", /* 3GPP TS 05.02, section 3.3.4.1 */
		.desc = "Slow SDCCH/8 associated control channel (sub-channel 2)",
		.chan_nr = RSL_CHAN_SDCCH8_ACCH + (2 << 3),
		.link_id = LID_SACCH,

		/* Same as for TRXC_BCCH and TRXC_SDCCH8_* (xCCH), see above. */
		.rts_fn = rts_data_fn,
		.dl_fn = tx_data_fn,
		.ul_fn = rx_data_fn,
	},
	[TRXC_SACCH8_3] = {
		.name = "SACCH/8(3)", /* 3GPP TS 05.02, section 3.3.4.1 */
		.desc = "Slow SDCCH/8 associated control channel (sub-channel 3)",
		.chan_nr = RSL_CHAN_SDCCH8_ACCH + (3 << 3),
		.link_id = LID_SACCH,

		/* Same as for TRXC_BCCH and TRXC_SDCCH8_* (xCCH), see above. */
		.rts_fn = rts_data_fn,
		.dl_fn = tx_data_fn,
		.ul_fn = rx_data_fn,
	},
	[TRXC_SACCH8_4] = {
		.name = "SACCH/8(4)", /* 3GPP TS 05.02, section 3.3.4.1 */
		.desc = "Slow SDCCH/8 associated control channel (sub-channel 4)",
		.chan_nr = RSL_CHAN_SDCCH8_ACCH + (4 << 3),
		.link_id = LID_SACCH,

		/* Same as for TRXC_BCCH and TRXC_SDCCH8_* (xCCH), see above. */
		.rts_fn = rts_data_fn,
		.dl_fn = tx_data_fn,
		.ul_fn = rx_data_fn,
	},
	[TRXC_SACCH8_5] = {
		.name = "SACCH/8(5)", /* 3GPP TS 05.02, section 3.3.4.1 */
		.desc = "Slow SDCCH/8 associated control channel (sub-channel 5)",
		.chan_nr = RSL_CHAN_SDCCH8_ACCH + (5 << 3),
		.link_id = LID_SACCH,

		/* Same as for TRXC_BCCH and TRXC_SDCCH8_* (xCCH), see above. */
		.rts_fn = rts_data_fn,
		.dl_fn = tx_data_fn,
		.ul_fn = rx_data_fn,
	},
	[TRXC_SACCH8_6] = {
		.name = "SACCH/8(6)", /* 3GPP TS 05.02, section 3.3.4.1 */
		.desc = "Slow SDCCH/8 associated control channel (sub-channel 6)",
		.chan_nr = RSL_CHAN_SDCCH8_ACCH + (6 << 3),
		.link_id = LID_SACCH,

		/* Same as for TRXC_BCCH and TRXC_SDCCH8_* (xCCH), see above. */
		.rts_fn = rts_data_fn,
		.dl_fn = tx_data_fn,
		.ul_fn = rx_data_fn,
	},
	[TRXC_SACCH8_7] = {
		.name = "SACCH/8(7)", /* 3GPP TS 05.02, section 3.3.4.1 */
		.desc = "Slow SDCCH/8 associated control channel (sub-channel 7)",
		.chan_nr = RSL_CHAN_SDCCH8_ACCH + (7 << 3),
		.link_id = LID_SACCH,

		/* Same as for TRXC_BCCH and TRXC_SDCCH8_* (xCCH), see above. */
		.rts_fn = rts_data_fn,
		.dl_fn = tx_data_fn,
		.ul_fn = rx_data_fn,
	},
	[TRXC_PDTCH] = {
		.name = "PDTCH", /* 3GPP TS 05.02, sections 3.2.4, 3.3.2.4 */
		.desc = "Packet data traffic & control channel",
		.chan_nr = RSL_CHAN_OSMO_PDCH,

		/* Rx and Tx, multiple coding schemes: CS-2..4 and MCS-1..9 (3GPP TS
		 * 05.03, chapter 5), regular interleaving as specified for xCCH. */
		.rts_fn = rts_data_fn,
		.dl_fn = tx_pdtch_fn,
		.ul_fn = rx_pdtch_fn,
	},
	[TRXC_PTCCH] = {
		.name = "PTCCH", /* 3GPP TS 05.02, section 3.3.4.2 */
		.desc = "Packet Timing advance control channel",
		.chan_nr = RSL_CHAN_OSMO_PDCH,

		/* On the Uplink, mobile stations transmit random Access Bursts
		 * to allow estimation of the timing advance for one MS in packet
		 * transfer mode. On Downlink, the network sends timing advance
		 * updates for several mobile stations. The coding scheme used
		 * for PTCCH/D messages is the same as for PDTCH CS-1. */
		.rts_fn = rts_data_fn,
		.dl_fn = tx_pdtch_fn,
		.ul_fn = rx_rach_fn,
	},
	[TRXC_CBCH] = {
		/* TODO: distinguish CBCH on SDCCH/4 and SDCCH/8 */
		.name = "CBCH", /* 3GPP TS 05.02, section 3.3.5 */
		.desc = "Cell Broadcast channel",
		.chan_nr = RSL_CHAN_OSMO_CBCH4,

		/* Tx only, same as for TRXC_BCCH (xCCH), see above. */
		.rts_fn = rts_data_fn,
		.dl_fn = tx_data_fn,
	},
};

enum {
	L1SCHED_TS_CTR_DL_LATE,
	L1SCHED_TS_CTR_DL_NOT_FOUND,
};

static const struct rate_ctr_desc l1sched_ts_ctr_desc[] = {
	[L1SCHED_TS_CTR_DL_LATE] =	{"l1sched_ts:dl_late", "Downlink frames arrived too late to submit to lower layers"},
	[L1SCHED_TS_CTR_DL_NOT_FOUND] =	{"l1sched_ts:dl_not_found", "Downlink frames not found while scheduling"},
};
static const struct rate_ctr_group_desc l1sched_ts_ctrg_desc = {
	"l1sched_ts",
	"L1 scheduler timeslot",
	OSMO_STATS_CLASS_GLOBAL,
	ARRAY_SIZE(l1sched_ts_ctr_desc),
	l1sched_ts_ctr_desc
};

/*
 * init / exit
 */

static void trx_sched_init_ts(struct gsm_bts_trx_ts *ts,
			      const unsigned int rate_ctr_idx)
{
	struct l1sched_ts *l1ts;
	unsigned int i;
	char name[128];

	l1ts = talloc_zero(ts->trx, struct l1sched_ts);
	OSMO_ASSERT(l1ts != NULL);

	/* Link both structures */
	ts->priv = l1ts;
	l1ts->ts = ts;

	l1ts->ctrs = rate_ctr_group_alloc(ts->trx,
					  &l1sched_ts_ctrg_desc,
					  rate_ctr_idx);
	snprintf(name, sizeof(name), "bts%u-trx%u-ts%u%s",
		 ts->trx->bts->nr, ts->trx->nr, ts->nr,
		 ts->vamos.is_shadow ? "-shadow" : "");
	rate_ctr_group_set_name(l1ts->ctrs, name);

	INIT_LLIST_HEAD(&l1ts->dl_prims);

	for (i = 0; i < ARRAY_SIZE(l1ts->chan_state); i++) {
		struct l1sched_chan_state *chan_state;
		chan_state = &l1ts->chan_state[i];
		chan_state->active = false;
	}
}

void trx_sched_init(struct gsm_bts_trx *trx)
{
	unsigned int tn;

	OSMO_ASSERT(trx != NULL);

	LOGPTRX(trx, DL1C, LOGL_DEBUG, "Init scheduler structures\n");

	/* Allocate shadow timeslots */
	gsm_bts_trx_init_shadow_ts(trx);

	for (tn = 0; tn < ARRAY_SIZE(trx->ts); tn++) {
		unsigned int rate_ctr_idx = trx->nr * 100 + tn;
		struct gsm_bts_trx_ts *ts = &trx->ts[tn];

		/* Init primary and shadow timeslots */
		trx_sched_init_ts(ts, rate_ctr_idx);
		trx_sched_init_ts(ts->vamos.peer, rate_ctr_idx + 10);
	}
}

static void trx_sched_clean_ts(struct gsm_bts_trx_ts *ts)
{
	struct l1sched_ts *l1ts = ts->priv;
	unsigned int i;

	msgb_queue_free(&l1ts->dl_prims);
	rate_ctr_group_free(l1ts->ctrs);
	l1ts->ctrs = NULL;

	/* clear lchan channel states */
	for (i = 0; i < ARRAY_SIZE(ts->lchan); i++)
		lchan_set_state(&ts->lchan[i], LCHAN_S_NONE);

	talloc_free(l1ts);
	ts->priv = NULL;
}

void trx_sched_clean(struct gsm_bts_trx *trx)
{
	unsigned int tn;

	LOGPTRX(trx, DL1C, LOGL_DEBUG, "Clean scheduler structures\n");

	for (tn = 0; tn < ARRAY_SIZE(trx->ts); tn++) {
		struct gsm_bts_trx_ts *ts = &trx->ts[tn];

		/* Clean primary and shadow timeslots */
		trx_sched_clean_ts(ts);
		trx_sched_clean_ts(ts->vamos.peer);
	}

	/* Free previously allocated shadow timeslots */
	gsm_bts_trx_free_shadow_ts(trx);
}

struct msgb *_sched_dequeue_prim(struct l1sched_ts *l1ts, const struct trx_dl_burst_req *br)
{
	struct msgb *msg, *msg2;
	uint32_t prim_fn, l1sap_fn;
	uint8_t chan_nr, link_id;

	/* get prim of current fn from queue */
	llist_for_each_entry_safe(msg, msg2, &l1ts->dl_prims, list) {
		struct osmo_phsap_prim *l1sap = msgb_l1sap_prim(msg);
		switch (l1sap->oph.primitive) {
		case PRIM_PH_DATA:
			chan_nr = l1sap->u.data.chan_nr;
			link_id = l1sap->u.data.link_id;
			l1sap_fn = l1sap->u.data.fn;
			break;
		case PRIM_TCH:
			chan_nr = l1sap->u.tch.chan_nr;
			link_id = 0;
			l1sap_fn = l1sap->u.tch.fn;
			break;
		default:
			LOGL1SB(DL1P, LOGL_ERROR, l1ts, br, "Prim has wrong type.\n");
			goto free_msg;
		}
		prim_fn = GSM_TDMA_FN_SUB(l1sap_fn, br->fn);
		if (prim_fn > 100) { /* l1sap_fn < fn */
			LOGL1SB(DL1P, LOGL_NOTICE, l1ts, br,
			     "Prim %u is out of range (%u vs exp %u), or channel %s with "
			     "type %s is already disabled. If this happens in "
			     "conjunction with PCU, increase 'rts-advance' by 5.\n",
			     prim_fn, l1sap_fn, br->fn,
			     get_lchan_by_chan_nr(l1ts->ts->trx, chan_nr)->name,
			     trx_chan_desc[br->chan].name);
			rate_ctr_inc2(l1ts->ctrs, L1SCHED_TS_CTR_DL_LATE);
			/* unlink and free message */
			llist_del(&msg->list);
			msgb_free(msg);
			continue;
		}
		if (prim_fn > 0) /* l1sap_fn > fn */
			break;

		/* l1sap_fn == fn */
		if ((chan_nr ^ (trx_chan_desc[br->chan].chan_nr | br->tn))
		 || ((link_id & 0xc0) ^ trx_chan_desc[br->chan].link_id)) {
			LOGL1SB(DL1P, LOGL_ERROR, l1ts, br, "Prim has wrong chan_nr=0x%02x link_id=%02x, "
				"expecting chan_nr=0x%02x link_id=%02x.\n", chan_nr, link_id,
				trx_chan_desc[br->chan].chan_nr | br->tn, trx_chan_desc[br->chan].link_id);
			goto free_msg;
		}

		/* unlink and return message */
		llist_del(&msg->list);
		return msg;
	}

	/* Queue was traversed with no candidate, no prim is available for current FN: */
	rate_ctr_inc2(l1ts->ctrs, L1SCHED_TS_CTR_DL_NOT_FOUND);
	return NULL;

free_msg:
	/* unlink and free message */
	llist_del(&msg->list);
	msgb_free(msg);
	return NULL;
}

int _sched_compose_ph_data_ind(struct l1sched_ts *l1ts, uint32_t fn,
			       enum trx_chan_type chan,
			       const uint8_t *data, size_t data_len,
			       uint16_t ber10k, float rssi,
			       int16_t ta_offs_256bits, int16_t link_qual_cb,
			       enum osmo_ph_pres_info_type presence_info)
{
	struct msgb *msg;
	struct osmo_phsap_prim *l1sap;
	uint8_t chan_nr = trx_chan_desc[chan].chan_nr | l1ts->ts->nr;

	/* VAMOS: use Osmocom specific channel number */
	if (l1ts->ts->vamos.is_shadow)
		chan_nr |= RSL_CHAN_OSMO_VAMOS_MASK;

	/* compose primitive */
	msg = l1sap_msgb_alloc(data_len);
	l1sap = msgb_l1sap_prim(msg);
	osmo_prim_init(&l1sap->oph, SAP_GSM_PH, PRIM_PH_DATA,
		PRIM_OP_INDICATION, msg);
	l1sap->u.data.chan_nr = chan_nr;
	l1sap->u.data.link_id = trx_chan_desc[chan].link_id;
	l1sap->u.data.fn = fn;
	l1sap->u.data.rssi = (int8_t) (rssi);
	l1sap->u.data.ber10k = ber10k;
	l1sap->u.data.ta_offs_256bits = ta_offs_256bits;
	l1sap->u.data.lqual_cb = link_qual_cb;
	l1sap->u.data.pdch_presence_info = presence_info;
	msg->l2h = msgb_put(msg, data_len);
	if (data_len)
		memcpy(msg->l2h, data, data_len);

	/* forward primitive */
	l1sap_up(l1ts->ts->trx, l1sap);

	return 0;
}

int _sched_compose_tch_ind(struct l1sched_ts *l1ts, uint32_t fn,
			   enum trx_chan_type chan,
			   const uint8_t *data, size_t data_len,
			   uint16_t ber10k, float rssi,
			   int16_t ta_offs_256bits, int16_t link_qual_cb,
			   uint8_t is_sub)
{
	struct msgb *msg;
	struct osmo_phsap_prim *l1sap;
	uint8_t chan_nr = trx_chan_desc[chan].chan_nr | l1ts->ts->nr;
	struct gsm_lchan *lchan = &l1ts->ts->lchan[l1sap_chan2ss(chan_nr)];

	/* VAMOS: use Osmocom specific channel number */
	if (l1ts->ts->vamos.is_shadow)
		chan_nr |= RSL_CHAN_OSMO_VAMOS_MASK;

	/* compose primitive */
	msg = l1sap_msgb_alloc(data_len);
	l1sap = msgb_l1sap_prim(msg);
	osmo_prim_init(&l1sap->oph, SAP_GSM_PH, PRIM_TCH,
		PRIM_OP_INDICATION, msg);
	l1sap->u.tch.chan_nr = chan_nr;
	l1sap->u.tch.fn = fn;
	l1sap->u.tch.rssi = (int8_t) (rssi);
	l1sap->u.tch.ber10k = ber10k;
	l1sap->u.tch.ta_offs_256bits = ta_offs_256bits;
	l1sap->u.tch.lqual_cb = link_qual_cb;
	l1sap->u.tch.is_sub = is_sub & 1;

	msg->l2h = msgb_put(msg, data_len);
	if (data_len)
		memcpy(msg->l2h, data, data_len);

	LOGL1S(DL1P, LOGL_DEBUG, l1ts, chan, l1sap->u.tch.fn, "%s Rx -> RTP: %s\n",
	       gsm_lchan_name(lchan), msgb_hexdump_l2(msg));
	/* forward primitive */
	l1sap_up(l1ts->ts->trx, l1sap);

	return 0;
}



/*
 * data request (from upper layer)
 */

int trx_sched_ph_data_req(struct gsm_bts_trx *trx, struct osmo_phsap_prim *l1sap)
{
	uint8_t tn = L1SAP_CHAN2TS(l1sap->u.data.chan_nr);
	struct l1sched_ts *l1ts = trx->ts[tn].priv;

	LOGL1S(DL1P, LOGL_DEBUG, l1ts, -1, l1sap->u.data.fn,
		"PH-DATA.req: chan_nr=0x%02x link_id=0x%02x\n",
		l1sap->u.data.chan_nr, l1sap->u.data.link_id);

	/* ignore empty frame */
	if (!l1sap->oph.msg->l2h || msgb_l2len(l1sap->oph.msg) == 0) {
		msgb_free(l1sap->oph.msg);
		return 0;
	}

	/* VAMOS: convert Osmocom specific channel number to a generic one */
	if (trx->ts[tn].vamos.is_shadow)
		l1sap->u.data.chan_nr &= ~RSL_CHAN_OSMO_VAMOS_MASK;

	msgb_enqueue(&l1ts->dl_prims, l1sap->oph.msg);

	return 0;
}

int trx_sched_tch_req(struct gsm_bts_trx *trx, struct osmo_phsap_prim *l1sap)
{
	uint8_t tn = L1SAP_CHAN2TS(l1sap->u.tch.chan_nr);
	struct l1sched_ts *l1ts = trx->ts[tn].priv;

	LOGL1S(DL1P, LOGL_DEBUG, l1ts, -1, l1sap->u.tch.fn,
	       "TCH.req: chan_nr=0x%02x\n", l1sap->u.tch.chan_nr);

	/* ignore empty frame */
	if (!msgb_l2len(l1sap->oph.msg)) {
		msgb_free(l1sap->oph.msg);
		return 0;
	}

	/* VAMOS: convert Osmocom specific channel number to a generic one */
	if (trx->ts[tn].vamos.is_shadow)
		l1sap->u.tch.chan_nr &= ~RSL_CHAN_OSMO_VAMOS_MASK;

	msgb_enqueue(&l1ts->dl_prims, l1sap->oph.msg);

	return 0;
}


/*
 * ready-to-send indication (to upper layer)
 */

/* RTS for data frame */
static int rts_data_fn(const struct l1sched_ts *l1ts, const struct trx_dl_burst_req *br)
{
	uint8_t chan_nr, link_id;
	struct msgb *msg;
	struct osmo_phsap_prim *l1sap;

	/* get data for RTS indication */
	chan_nr = trx_chan_desc[br->chan].chan_nr | br->tn;
	link_id = trx_chan_desc[br->chan].link_id;


	/* VAMOS: use Osmocom specific channel number */
	if (l1ts->ts->vamos.is_shadow)
		chan_nr |= RSL_CHAN_OSMO_VAMOS_MASK;

	/* For handover detection, there are cases where the SACCH should remain inactive until the first RACH
	 * indicating the TA is received. */
	if (L1SAP_IS_LINK_SACCH(link_id)
	    && !l1ts->chan_state[br->chan].lchan->want_dl_sacch_active)
		return 0;

	LOGL1SB(DL1P, LOGL_DEBUG, l1ts, br, "PH-RTS.ind: chan_nr=0x%02x link_id=0x%02x\n", chan_nr, link_id);

	/* generate prim */
	msg = l1sap_msgb_alloc(200);
	if (!msg)
		return -ENOMEM;
	l1sap = msgb_l1sap_prim(msg);
	osmo_prim_init(&l1sap->oph, SAP_GSM_PH, PRIM_PH_RTS,
	                                PRIM_OP_INDICATION, msg);
	l1sap->u.data.chan_nr = chan_nr;
	l1sap->u.data.link_id = link_id;
	l1sap->u.data.fn = br->fn;

	return l1sap_up(l1ts->ts->trx, l1sap);
}

static int rts_tch_common(const struct l1sched_ts *l1ts,
			  const struct trx_dl_burst_req *br,
			  bool facch)
{
	uint8_t chan_nr, link_id;
	struct msgb *msg;
	struct osmo_phsap_prim *l1sap;
	int rc = 0;

	/* get data for RTS indication */
	chan_nr = trx_chan_desc[br->chan].chan_nr | br->tn;
	link_id = trx_chan_desc[br->chan].link_id;


	/* VAMOS: use Osmocom specific channel number */
	if (l1ts->ts->vamos.is_shadow)
		chan_nr |= RSL_CHAN_OSMO_VAMOS_MASK;

	LOGL1SB(DL1P, LOGL_DEBUG, l1ts, br, "TCH RTS.ind: chan_nr=0x%02x\n", chan_nr);

	/* only send, if FACCH is selected */
	if (facch) {
		/* generate prim */
		msg = l1sap_msgb_alloc(200);
		if (!msg)
			return -ENOMEM;
		l1sap = msgb_l1sap_prim(msg);
		osmo_prim_init(&l1sap->oph, SAP_GSM_PH, PRIM_PH_RTS,
						PRIM_OP_INDICATION, msg);
		l1sap->u.data.chan_nr = chan_nr;
		l1sap->u.data.link_id = link_id;
		l1sap->u.data.fn = br->fn;

		rc = l1sap_up(l1ts->ts->trx, l1sap);
	}

	/* don't send, if TCH is in signalling only mode */
	if (l1ts->chan_state[br->chan].rsl_cmode != RSL_CMOD_SPD_SIGN) {
		/* generate prim */
		msg = l1sap_msgb_alloc(200);
		if (!msg)
			return -ENOMEM;
		l1sap = msgb_l1sap_prim(msg);
		osmo_prim_init(&l1sap->oph, SAP_GSM_PH, PRIM_TCH_RTS,
						PRIM_OP_INDICATION, msg);
		l1sap->u.tch.chan_nr = chan_nr;
		l1sap->u.tch.fn = br->fn;

		return l1sap_up(l1ts->ts->trx, l1sap);
	}

	return rc;
}

/* RTS for full rate traffic frame */
static int rts_tchf_fn(const struct l1sched_ts *l1ts, const struct trx_dl_burst_req *br)
{
	/* TCH/F may include FACCH on every 4th burst */
	return rts_tch_common(l1ts, br, true);
}

/* FACCH/H channel mapping for Downlink (see 3GPP TS 45.002, table 1).
 * This mapping is valid for both FACCH/H(0) and FACCH/H(1). */
const uint8_t sched_tchh_dl_facch_map[26] = {
	[4]  = 1, /* FACCH/H(0): B0(4,6,8,10,13,15) */
	[5]  = 1, /* FACCH/H(1): B0(5,7,9,11,14,16) */
	[13] = 1, /* FACCH/H(0): B1(13,15,17,19,21,23) */
	[14] = 1, /* FACCH/H(1): B1(14,16,18,20,22,24) */
	[21] = 1, /* FACCH/H(0): B2(21,23,0,2,4,6) */
	[22] = 1, /* FACCH/H(1): B2(22,24,1,3,5,7) */
};

/* RTS for half rate traffic frame */
static int rts_tchh_fn(const struct l1sched_ts *l1ts, const struct trx_dl_burst_req *br)
{
	return rts_tch_common(l1ts, br, sched_tchh_dl_facch_map[br->fn % 26]);
}

/* set multiframe scheduler to given pchan */
int trx_sched_set_pchan(struct gsm_bts_trx_ts *ts, enum gsm_phys_chan_config pchan)
{
	struct l1sched_ts *l1ts = ts->priv;
	int i = find_sched_mframe_idx(pchan, ts->nr);
	if (i < 0) {
		LOGP(DL1C, LOGL_NOTICE, "%s Failed to configure multiframe (pchan=0x%02x)\n",
		     gsm_ts_name(ts), pchan);
		return -ENOTSUP;
	}
	l1ts->mf_index = i;
	l1ts->mf_period = trx_sched_multiframes[i].period;
	l1ts->mf_frames = trx_sched_multiframes[i].frames;
	if (ts->vamos.peer != NULL) {
		l1ts = ts->vamos.peer->priv;
		l1ts->mf_index = i;
		l1ts->mf_period = trx_sched_multiframes[i].period;
		l1ts->mf_frames = trx_sched_multiframes[i].frames;
	}
	LOGP(DL1C, LOGL_NOTICE, "%s Configured multiframe with '%s'\n",
	     gsm_ts_name(ts), trx_sched_multiframes[i].name);
	return 0;
}

/* Remove all matching (by chan_nr & link_id) primitives from the given queue */
static void trx_sched_queue_filter(struct llist_head *q, uint8_t chan_nr, uint8_t link_id)
{
	struct msgb *msg, *_msg;

	llist_for_each_entry_safe(msg, _msg, q, list) {
		struct osmo_phsap_prim *l1sap = msgb_l1sap_prim(msg);
		switch (l1sap->oph.primitive) {
		case PRIM_PH_DATA:
			if (l1sap->u.data.chan_nr != chan_nr)
				continue;
			if (l1sap->u.data.link_id != link_id)
				continue;
			break;
		case PRIM_TCH:
			if (l1sap->u.tch.chan_nr != chan_nr)
				continue;
			if (link_id != 0x00)
				continue;
			break;
		default:
			/* Shall not happen */
			OSMO_ASSERT(0);
		}

		/* Unlink and free() */
		llist_del(&msg->list);
		talloc_free(msg);
	}
}

static void _trx_sched_set_lchan(struct gsm_lchan *lchan,
				 enum trx_chan_type chan,
				 bool active)
{
	struct l1sched_ts *l1ts = lchan->ts->priv;
	struct l1sched_chan_state *chan_state;

	OSMO_ASSERT(l1ts != NULL);
	chan_state = &l1ts->chan_state[chan];

	LOGPLCHAN(lchan, DL1C, LOGL_INFO, "%s %s\n",
		  (active) ? "Activating" : "Deactivating",
		  trx_chan_desc[chan].name);

	if (active) {
		/* Clean up everything */
		memset(chan_state, 0, sizeof(*chan_state));

		/* Bind to generic 'struct gsm_lchan' */
		chan_state->lchan = lchan;

		/* Allocate memory for Rx/Tx burst buffers.  Use the maximim size
		 * of 24 * (2 * 58) bytes, which is sufficient to store up to 24 GMSK
		 * modulated bursts for CSD or up to 8 8PSK modulated bursts for EGPRS. */
		const size_t buf_size = 24 * GSM_NBITS_NB_GMSK_PAYLOAD;
		if (trx_chan_desc[chan].dl_fn != NULL)
			chan_state->dl_bursts = talloc_zero_size(l1ts, buf_size);
		if (trx_chan_desc[chan].ul_fn != NULL) {
			chan_state->ul_bursts = talloc_zero_size(l1ts, buf_size);
			if (L1SAP_IS_LINK_SACCH(trx_chan_desc[chan].link_id))
				chan_state->ul_bursts_prev = talloc_zero_size(l1ts, buf_size);
		}
	} else {
		chan_state->ho_rach_detect = 0;

		/* Remove pending Tx prims belonging to this lchan */
		trx_sched_queue_filter(&l1ts->dl_prims,
				       trx_chan_desc[chan].chan_nr,
				       trx_chan_desc[chan].link_id);

		/* Release memory used by Rx/Tx burst buffers */
		TALLOC_FREE(chan_state->dl_bursts);
		TALLOC_FREE(chan_state->ul_bursts);
		TALLOC_FREE(chan_state->ul_bursts_prev);
	}

	chan_state->active = active;
}

/* setting all logical channels given attributes to active/inactive */
int trx_sched_set_lchan(struct gsm_lchan *lchan, uint8_t chan_nr, uint8_t link_id, bool active)
{
	struct l1sched_ts *l1ts = lchan->ts->priv;
	uint8_t tn = L1SAP_CHAN2TS(chan_nr);
	uint8_t ss = l1sap_chan2ss(chan_nr);
	bool found = false;

	if (!l1ts) {
		LOGPLCHAN(lchan, DL1C, LOGL_ERROR, "%s lchan with uninitialized scheduler structure\n",
			  (active) ? "Activating" : "Deactivating");
		return -EINVAL;
	}

	/* VAMOS: convert Osmocom specific channel number to a generic one,
	 * otherwise we won't match anything in trx_chan_desc[]. */
	if (lchan->ts->vamos.is_shadow)
		chan_nr &= ~RSL_CHAN_OSMO_VAMOS_MASK;

	/* look for all matching chan_nr/link_id */
	for (enum trx_chan_type chan = 0; chan < _TRX_CHAN_MAX; chan++) {
		if (trx_chan_desc[chan].chan_nr != (chan_nr & RSL_CHAN_NR_MASK))
			continue;
		if (trx_chan_desc[chan].link_id != link_id)
			continue;
		if (l1ts->chan_state[chan].active == active)
			continue;
		found = true;
		_trx_sched_set_lchan(lchan, chan, active);
	}

	/* disable handover detection (on deactivation) */
	if (!active)
		_sched_act_rach_det(lchan->ts->trx, tn, ss, 0);

	return found ? 0 : -EINVAL;
}

int trx_sched_set_ul_access(struct gsm_lchan *lchan, uint8_t chan_nr, bool active)
{
	struct l1sched_ts *l1ts = lchan->ts->priv;
	uint8_t tn = L1SAP_CHAN2TS(chan_nr);
	uint8_t ss = l1sap_chan2ss(chan_nr);
	int i;

	if (!l1ts) {
		LOGPLCHAN(lchan, DL1C, LOGL_ERROR, "%s UL access on lchan with uninitialized scheduler structure.\n",
			  (active) ? "Activating" : "Deactivating");
		return -EINVAL;
	}

	/* look for all matching chan_nr */
	for (i = 0; i < _TRX_CHAN_MAX; i++) {
		if (trx_chan_desc[i].chan_nr == (chan_nr & RSL_CHAN_NR_MASK)) {
			struct l1sched_chan_state *l1cs = &l1ts->chan_state[i];

			l1cs->ho_rach_detect = active;
		}
	}

	_sched_act_rach_det(lchan->ts->trx, tn, ss, active);

	return 0;
}

int trx_sched_set_bcch_ccch(struct gsm_lchan *lchan, bool active)
{
	struct l1sched_ts *l1ts = lchan->ts->priv;
	static const enum trx_chan_type chans[] = {
		TRXC_FCCH,
		TRXC_SCH,
		TRXC_BCCH,
		TRXC_RACH,
		TRXC_CCCH,
	};

	if (!l1ts)
		return -EINVAL;

	for (unsigned int i = 0; i < ARRAY_SIZE(chans); i++) {
		enum trx_chan_type chan = chans[i];

		if (l1ts->chan_state[chan].active == active)
			continue;
		_trx_sched_set_lchan(lchan, chan, active);
	}

	return 0;
}

/* setting all logical channels given attributes to active/inactive */
int trx_sched_set_mode(struct gsm_bts_trx_ts *ts, uint8_t chan_nr, uint8_t rsl_cmode,
	uint8_t tch_mode, int codecs, uint8_t codec0, uint8_t codec1,
	uint8_t codec2, uint8_t codec3, uint8_t initial_id, uint8_t handover)
{
	struct l1sched_ts *l1ts = ts->priv;
	uint8_t tn = L1SAP_CHAN2TS(chan_nr);
	uint8_t ss = l1sap_chan2ss(chan_nr);
	int i;
	int rc = -EINVAL;

	/* no mode for PDCH */
	if (ts->pchan == GSM_PCHAN_PDCH)
		return 0;

	/* VAMOS: convert Osmocom specific channel number to a generic one,
	 * otherwise we won't match anything in trx_chan_desc[]. */
	if (ts->vamos.is_shadow)
		chan_nr &= ~RSL_CHAN_OSMO_VAMOS_MASK;

	/* look for all matching chan_nr/link_id */
	for (i = 0; i < _TRX_CHAN_MAX; i++) {
		if (trx_chan_desc[i].chan_nr == (chan_nr & 0xf8)
		 && trx_chan_desc[i].link_id == 0x00) {
			struct l1sched_chan_state *chan_state = &l1ts->chan_state[i];

			LOGP(DL1C, LOGL_INFO,
			     "%s Set mode for %s (rsl_cmode=%u, tch_mode=%u, handover=%u)\n",
			     gsm_ts_name(ts), trx_chan_desc[i].name,
			     rsl_cmode, tch_mode, handover);

			chan_state->rsl_cmode = rsl_cmode;
			chan_state->tch_mode = tch_mode;
			chan_state->ho_rach_detect = handover;
			if (rsl_cmode == RSL_CMOD_SPD_SPEECH
			 && tch_mode == GSM48_CMODE_SPEECH_AMR) {
				chan_state->codecs = codecs;
				chan_state->codec[0] = codec0;
				chan_state->codec[1] = codec1;
				chan_state->codec[2] = codec2;
				chan_state->codec[3] = codec3;
				chan_state->ul_ft = initial_id;
				chan_state->dl_ft = initial_id;
				chan_state->ul_cmr = initial_id;
				chan_state->dl_cmr = initial_id;
				chan_state->lqual_cb_sum = 0;
				chan_state->lqual_cb_num = 0;
			}
			rc = 0;
		}
	}

	/* command rach detection
	 * always enable handover, even if state is still set (due to loss
	 * of transceiver link).
	 * disable handover, if state is still set, since we might not know
	 * the actual state of transceiver (due to loss of link) */
	_sched_act_rach_det(ts->trx, tn, ss, handover);

	return rc;
}

/* setting cipher on logical channels */
int trx_sched_set_cipher(struct gsm_lchan *lchan, uint8_t chan_nr, bool downlink)
{
	int algo = lchan->encr.alg_id - 1;
	int i, rc = -EINVAL;

	/* no cipher for PDCH */
	if (ts_pchan(lchan->ts) == GSM_PCHAN_PDCH)
		return 0;

	/* VAMOS: convert Osmocom specific channel number to a generic one,
	 * otherwise we won't match anything in trx_chan_desc[]. */
	if (lchan->ts->vamos.is_shadow)
		chan_nr &= ~RSL_CHAN_OSMO_VAMOS_MASK;

	/* no algorithm given means a5/0 */
	if (algo <= 0)
		algo = 0;
	else if (lchan->encr.key_len != 8 && lchan->encr.key_len != 16) {
		LOGPLCHAN(lchan, DL1C, LOGL_ERROR,
			  "Algo A5/%d not supported with given key_len=%u\n",
			  algo, lchan->encr.key_len);
		return -ENOTSUP;
	}

	/* look for all matching chan_nr */
	for (i = 0; i < _TRX_CHAN_MAX; i++) {
		if (trx_chan_desc[i].chan_nr == (chan_nr & RSL_CHAN_NR_MASK)) {
			struct l1sched_ts *l1ts = lchan->ts->priv;
			struct l1sched_chan_state *l1cs = &l1ts->chan_state[i];

			LOGPLCHAN(lchan, DL1C, LOGL_INFO, "Set A5/%d %s for %s\n",
				  algo, (downlink) ? "downlink" : "uplink",
				  trx_chan_desc[i].name);

			if (downlink) {
				l1cs->dl_encr_algo = algo;
				memcpy(l1cs->dl_encr_key, lchan->encr.key, lchan->encr.key_len);
				l1cs->dl_encr_key_len = lchan->encr.key_len;
			} else {
				l1cs->ul_encr_algo = algo;
				memcpy(l1cs->ul_encr_key, lchan->encr.key, lchan->encr.key_len);
				l1cs->ul_encr_key_len = lchan->encr.key_len;
			}
			rc = 0;
		}
	}

	return rc;
}

/* process ready-to-send */
int _sched_rts(const struct l1sched_ts *l1ts, uint32_t fn)
{
	const struct trx_sched_frame *frame;
	uint8_t offset, period, bid;
	trx_sched_rts_func *func;
	enum trx_chan_type chan;

	/* no multiframe set */
	if (!l1ts->mf_index)
		return 0;

	/* get frame from multiframe */
	period = l1ts->mf_period;
	offset = fn % period;
	frame = l1ts->mf_frames + offset;

	chan = frame->dl_chan;
	bid = frame->dl_bid;
	func = trx_chan_desc[frame->dl_chan].rts_fn;

	/* only on bid == 0 */
	if (bid != 0)
		return 0;

	/* no RTS function */
	if (!func)
		return 0;

	/* check if channel is active */
	if (!l1ts->chan_state[chan].active)
	 	return -EINVAL;

	/* There is no burst, just for logging */
	struct trx_dl_burst_req dbr = {
		.fn = fn,
		.tn = l1ts->ts->nr,
		.bid = bid,
		.chan = chan,
	};

	return func(l1ts, &dbr);
}

static void trx_sched_apply_att(const struct gsm_lchan *lchan,
				struct trx_dl_burst_req *br)
{
	const struct trx_chan_desc *desc = &trx_chan_desc[br->chan];

	/* Current BS power reduction value in dB */
	br->att = lchan->bs_power_ctrl.current;

	/* Temporary Overpower for SACCH/FACCH bursts */
	if (!lchan->top_acch_active)
		return;
	if ((lchan->top_acch_cap.sacch_enable && desc->link_id == LID_SACCH) ||
	    (lchan->top_acch_cap.facch_enable && br->flags & TRX_BR_F_FACCH)) {
		if (br->att > lchan->top_acch_cap.overpower_db)
			br->att -= lchan->top_acch_cap.overpower_db;
		else
			br->att = 0;
	}
}

/* process downlink burst */
void _sched_dl_burst(struct l1sched_ts *l1ts, struct trx_dl_burst_req *br)
{
	const struct l1sched_chan_state *l1cs;
	const struct trx_sched_frame *frame;
	uint8_t offset, period;
	trx_sched_dl_func *func;

	if (!l1ts->mf_index)
		return;

	/* get frame from multiframe */
	period = l1ts->mf_period;
	offset = br->fn % period;
	frame = l1ts->mf_frames + offset;

	br->chan = frame->dl_chan;
	br->bid = frame->dl_bid;
	func = trx_chan_desc[br->chan].dl_fn;

	l1cs = &l1ts->chan_state[br->chan];

	/* check if channel is active */
	if (!l1cs->active)
		return;

	/* Training Sequence Code and Set */
	br->tsc_set = l1ts->ts->tsc_set;
	br->tsc = l1ts->ts->tsc;

	/* get burst from function */
	if (func(l1ts, br) != 0)
		return;

	/* Modulation is indicated by func() */
	br->mod = l1cs->dl_mod_type;

	/* BS Power reduction (in dB) per logical channel */
	if (l1cs->lchan != NULL)
		trx_sched_apply_att(l1cs->lchan, br);

	/* encrypt */
	if (br->burst_len && l1cs->dl_encr_algo) {
		ubit_t ks[114];
		int i;

		osmo_a5(l1cs->dl_encr_algo, l1cs->dl_encr_key, br->fn, ks, NULL);
		for (i = 0; i < 57; i++) {
			br->burst[i +  3] ^= ks[i];
			br->burst[i + 88] ^= ks[i + 57];
		}
	}
}

static int trx_sched_calc_frame_loss(struct l1sched_ts *l1ts,
				     struct l1sched_chan_state *l1cs,
				     const struct trx_ul_burst_ind *bi)
{
	const struct trx_sched_frame *frame;
	uint32_t elapsed_fs;
	uint8_t offset, i;
	uint32_t fn_i;

	/**
	 * When a channel is just activated, the MS needs some time
	 * to synchronize and start burst transmission,
	 * so let's wait until the first UL burst...
	 */
	if (l1cs->proc_tdma_fs == 0)
		return 0;

	/* Not applicable for some logical channels */
	switch (bi->chan) {
	case TRXC_IDLE:
	case TRXC_RACH:
	case TRXC_PDTCH:
	case TRXC_PTCCH:
		return 0;
	default:
		/* No applicable if we are waiting for handover RACH */
		if (l1cs->ho_rach_detect)
			return 0;
	}

	/* How many frames elapsed since the last one? */
	elapsed_fs = GSM_TDMA_FN_SUB(bi->fn, l1cs->last_tdma_fn);
	if (elapsed_fs > l1ts->mf_period) { /* Too many! */
		LOGL1SB(DL1P, LOGL_ERROR, l1ts, bi,
			"Too many (>%u) contiguous TDMA frames=%u elapsed "
			"since the last processed fn=%u\n", l1ts->mf_period,
			elapsed_fs, l1cs->last_tdma_fn);
		/* FIXME: how should this affect the measurements? */
		return -EINVAL;
	}

	/**
	 * There are several TDMA frames between the last processed
	 * frame and currently received one. Let's walk through this
	 * path and count potentially lost frames, i.e. for which
	 * we didn't receive the corresponding UL bursts.
	 *
	 * Start counting from the last_fn + 1.
	 */
	for (i = 1; i < elapsed_fs; i++) {
		fn_i = GSM_TDMA_FN_SUM(l1cs->last_tdma_fn, i);
		offset = fn_i % l1ts->mf_period;
		frame = l1ts->mf_frames + offset;

		if (frame->ul_chan == bi->chan)
			l1cs->lost_tdma_fs++;
	}

	if (l1cs->lost_tdma_fs > 0) {
		LOGL1SB(DL1P, LOGL_NOTICE, l1ts, bi,
			"At least %u TDMA frames were lost since the last "
			"processed fn=%u\n", l1cs->lost_tdma_fs, l1cs->last_tdma_fn);

		/**
		 * HACK: substitute lost bursts by zero-filled ones
		 *
		 * Instead of doing this, it makes sense to use the
		 * amount of lost frames in measurement calculations.
		 */
		trx_sched_ul_func *func;

		/* Prepare dummy burst indication */
		struct trx_ul_burst_ind dbi = {
			.flags = TRX_BI_F_NOPE_IND,
			.burst_len = GSM_BURST_LEN,
			.burst = { 0 },
			.rssi = -128,
			.toa256 = 0,
			.chan = bi->chan,
			/* TDMA FN is set below */
			.tn = bi->tn,
		};

		for (i = 1; i < elapsed_fs; i++) {
			fn_i = GSM_TDMA_FN_SUM(l1cs->last_tdma_fn, i);
			offset = fn_i % l1ts->mf_period;
			frame = l1ts->mf_frames + offset;
			func = trx_chan_desc[frame->ul_chan].ul_fn;

			if (frame->ul_chan != bi->chan)
				continue;

			dbi.bid = frame->ul_bid;
			dbi.fn = fn_i;

			LOGL1SB(DL1P, LOGL_NOTICE, l1ts, &dbi,
				"Substituting lost burst with NOPE.ind\n");

			func(l1ts, &dbi);

			l1cs->lost_tdma_fs--;
		}
	}

	return 0;
}

/* Process a single noise measurement for an inactive timeslot. */
static void trx_sched_noise_meas(struct l1sched_chan_state *l1cs,
				 const struct trx_ul_burst_ind *bi)
{
	int *Avg = &l1cs->meas.interf_avg;

	/* EWMA (Exponentially Weighted Moving Average):
	*
	*   Avg[n] = a * Val[n] + (1 - a) * Avg[n - 1]
	*
	* Implemented using the '+=' operator:
	*
	*   Avg += a * Val - a * Avg
	*   Avg += a * (Val - Avg)
	*
	* We use constant 'a' = 0.5, what is equal to:
	*
	*   Avg += (Val - Avg) / 2
	*
	* We don't really need precisity here, so no scaling.
	*/

	*Avg += (bi->rssi - *Avg) / 2;
}

/* Process an Uplink burst indication */
int trx_sched_ul_burst(struct l1sched_ts *l1ts, struct trx_ul_burst_ind *bi)
{
	struct l1sched_chan_state *l1cs;
	const struct trx_sched_frame *frame;
	uint8_t offset, period;
	trx_sched_ul_func *func;

	/* VAMOS: redirect to the shadow timeslot */
	if (bi->flags & TRX_BI_F_SHADOW_IND)
		l1ts = l1ts->ts->vamos.peer->priv;

	if (!l1ts->mf_index)
		return -EINVAL;

	/* get frame from multiframe */
	period = l1ts->mf_period;
	offset = bi->fn % period;
	frame = l1ts->mf_frames + offset;

	bi->chan = frame->ul_chan;
	bi->bid = frame->ul_bid;
	l1cs = &l1ts->chan_state[bi->chan];
	func = trx_chan_desc[bi->chan].ul_fn;

	/* check if channel is active */
	if (!l1cs->active) {
		/* handle noise measurements on dedicated and idle channels */
		if (TRX_CHAN_IS_DEDIC(bi->chan) || bi->chan == TRXC_IDLE)
			trx_sched_noise_meas(l1cs, bi);
		return 0;
	}

	/* omit bursts which have no handler, like IDLE bursts */
	if (!func)
		return -EINVAL;

	/* calculate how many TDMA frames were potentially lost */
	trx_sched_calc_frame_loss(l1ts, l1cs, bi);

	/* update TDMA frame counters */
	l1cs->last_tdma_fn = bi->fn;
	l1cs->proc_tdma_fs++;

	/* handle NOPE indications */
	if (bi->flags & TRX_BI_F_NOPE_IND) {
		/* NOTE: Uplink burst handler must check bi->burst_len before
		 * accessing bi->burst to avoid uninitialized memory access. */
		return func(l1ts, bi);
	}

	/* decrypt */
	if (bi->burst_len && l1cs->ul_encr_algo) {
		ubit_t ks[114];
		int i;

		osmo_a5(l1cs->ul_encr_algo, l1cs->ul_encr_key, bi->fn, NULL, ks);
		for (i = 0; i < 57; i++) {
			if (ks[i])
				bi->burst[i + 3] = - bi->burst[i + 3];
			if (ks[i + 57])
				bi->burst[i + 88] = - bi->burst[i + 88];
		}
	}

	/* Invoke the logical channel handler */
	func(l1ts, bi);

	return 0;
}
