/* Scheduler for OsmoBTS-TRX */

/* (C) 2013 by Andreas Eversberg <jolly@eversberg.eu>
 * (C) 2015 by Alexander Chemeris <Alexander.Chemeris@fairwaves.co>
 * (C) 2015 by Harald Welte <laforge@gnumonks.org>
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

#include <osmocom/gsm/protocol/gsm_08_58.h>
#include <osmocom/gsm/a5.h>

#include <osmo-bts/gsm_data.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/rsl.h>
#include <osmo-bts/l1sap.h>
#include <osmo-bts/scheduler.h>
#include <osmo-bts/scheduler_backend.h>

extern void *tall_bts_ctx;

static int rts_data_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan);
static int rts_tchf_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan);
static int rts_tchh_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan);
/*! \brief Dummy Burst (TS 05.02 Chapter 5.2.6) */
static const ubit_t dummy_burst[GSM_BURST_LEN] = {
	0,0,0,
	1,1,1,1,1,0,1,1,0,1,1,1,0,1,1,0,0,0,0,0,1,0,1,0,0,1,0,0,1,1,1,0,
	0,0,0,0,1,0,0,1,0,0,0,1,0,0,0,0,0,0,0,1,1,1,1,1,0,0,0,1,1,1,0,0,
	0,1,0,1,1,1,0,0,0,1,0,1,1,1,0,0,0,1,0,1,0,1,1,1,0,1,0,0,1,0,1,0,
	0,0,1,1,0,0,1,1,0,0,1,1,1,0,0,1,1,1,1,0,1,0,0,1,1,1,1,1,0,0,0,1,
	0,0,1,0,1,1,1,1,1,0,1,0,1,0,
	0,0,0,
};

/*! \brief FCCH Burst (TS 05.02 Chapter 5.2.4) */
const ubit_t _sched_fcch_burst[GSM_BURST_LEN] = {
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
};

/*! \brief Training Sequences (TS 05.02 Chapter 5.2.3) */
const ubit_t _sched_tsc[8][26] = {
	{ 0,0,1,0,0,1,0,1,1,1,0,0,0,0,1,0,0,0,1,0,0,1,0,1,1,1, },
	{ 0,0,1,0,1,1,0,1,1,1,0,1,1,1,1,0,0,0,1,0,1,1,0,1,1,1, },
	{ 0,1,0,0,0,0,1,1,1,0,1,1,1,0,1,0,0,1,0,0,0,0,1,1,1,0, },
	{ 0,1,0,0,0,1,1,1,1,0,1,1,0,1,0,0,0,1,0,0,0,1,1,1,1,0, },
	{ 0,0,0,1,1,0,1,0,1,1,1,0,0,1,0,0,0,0,0,1,1,0,1,0,1,1, },
	{ 0,1,0,0,1,1,1,0,1,0,1,1,0,0,0,0,0,1,0,0,1,1,1,0,1,0, },
	{ 1,0,1,0,0,1,1,1,1,1,0,1,1,0,0,0,1,0,1,0,0,1,1,1,1,1, },
	{ 1,1,1,0,1,1,1,1,0,0,0,1,0,0,1,0,1,1,1,0,1,1,1,1,0,0, },
};

const ubit_t _sched_egprs_tsc[8][78] = {
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
const ubit_t _sched_sch_train[64] = {
	1,0,1,1,1,0,0,1,0,1,1,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,1,1,1,
	0,0,1,0,1,1,0,1,0,1,0,0,0,1,0,1,0,1,1,1,0,1,1,0,0,0,0,1,1,0,1,1,
};

/* Logical channel (TRXC_*) description */
const struct trx_chan_desc trx_chan_desc[_TRX_CHAN_MAX] = {
	[TRXC_IDLE] = {
		.name = "IDLE",
		.desc = "Idle channel",

		/* On C0, BTS needs to ensure discontinuous burst transmission.
		 * Therefore we need to send dummy bursts on IDLE slots. */
		.flags = TRX_CHAN_FLAG_AUTO_ACTIVE,
		.dl_fn = tx_idle_fn,
	},
	[TRXC_FCCH] = {
		.name = "FCCH", /* 3GPP TS 05.02, section 3.3.2.1 */
		.desc = "Frequency correction channel",

		/* Tx only, frequency correction bursts */
		.flags = TRX_CHAN_FLAG_AUTO_ACTIVE,
		.dl_fn = tx_fcch_fn,
	},
	[TRXC_SCH] = {
		.name = "SCH", /* 3GPP TS 05.02, section 3.3.2.2 */
		.desc = "Synchronization channel",

		/* Tx only, synchronization bursts */
		.flags = TRX_CHAN_FLAG_AUTO_ACTIVE,
		.dl_fn = tx_sch_fn,
	},
	[TRXC_BCCH] = {
		.name = "BCCH", /* 3GPP TS 05.02, section 3.3.2.3 */
		.desc = "Broadcast control channel",
		.chan_nr = RSL_CHAN_BCCH,

		/* Tx only, xCCH convolutional coding (3GPP TS 05.03, section 4.4),
		 * regular interleaving (3GPP TS 05.02, clause 7, table 3):
		 * a L2 frame is interleaved over 4 consecutive bursts. */
		.flags = TRX_CHAN_FLAG_AUTO_ACTIVE,
		.rts_fn = rts_data_fn,
		.dl_fn = tx_data_fn,
	},
	[TRXC_RACH] = {
		.name = "RACH", /* 3GPP TS 05.02, section 3.3.3.1 */
		.desc = "Random access channel",
		.chan_nr = RSL_CHAN_RACH,

		/* Rx only, RACH convolutional coding (3GPP TS 05.03, section 4.6). */
		.flags = TRX_CHAN_FLAG_AUTO_ACTIVE,
		.ul_fn = rx_rach_fn,
	},
	[TRXC_CCCH] = {
		.name = "CCCH", /* 3GPP TS 05.02, section 3.3.3.1 */
		.desc = "Common control channel",
		.chan_nr = RSL_CHAN_PCH_AGCH,

		/* Tx only, xCCH convolutional coding (3GPP TS 05.03, section 4.4),
		 * regular interleaving (3GPP TS 05.02, clause 7, table 3):
		 * a L2 frame is interleaved over 4 consecutive bursts. */
		.flags = TRX_CHAN_FLAG_AUTO_ACTIVE,
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
		 *   - a traffic frame is interleaved over 6 consecutive bursts
		 *     using the even numbered bits of the first 2 bursts,
		 *     all bits of the middle two 2 bursts,
		 *     and odd numbered bits of the last 2 bursts;
		 *   - a FACCH/H frame 'steals' (replaces) two traffic frames,
		 *     interleaving is done over 4 consecutive bursts,
		 *     the same as given for a TCH/FS. */
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
		 * 05.03, chapter 5), regular interleaving as specified for xCCH.
		 * NOTE: the burst buffer is three times bigger because the
		 * payload of EDGE bursts is three times longer. */
		.flags = TRX_CHAN_FLAG_PDCH,
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
		.flags = TRX_CHAN_FLAG_PDCH,
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
		.flags = TRX_CHAN_FLAG_AUTO_ACTIVE,
		.rts_fn = rts_data_fn,
		.dl_fn = tx_data_fn,
	},
};

/*
 * init / exit
 */

int trx_sched_init(struct l1sched_trx *l1t, struct gsm_bts_trx *trx)
{
	uint8_t tn;
	unsigned int i;

	if (!trx)
		return -EINVAL;

	l1t->trx = trx;

	LOGP(DL1C, LOGL_NOTICE, "Init scheduler for trx=%u\n", l1t->trx->nr);

	for (tn = 0; tn < ARRAY_SIZE(l1t->ts); tn++) {
		struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);

		l1ts->mf_index = 0;
		INIT_LLIST_HEAD(&l1ts->dl_prims);
		for (i = 0; i < ARRAY_SIZE(l1ts->chan_state); i++) {
			struct l1sched_chan_state *chan_state;
			chan_state = &l1ts->chan_state[i];
			chan_state->active = 0;
		}
	}

	return 0;
}

void trx_sched_exit(struct l1sched_trx *l1t)
{
	struct gsm_bts_trx_ts *ts;
	uint8_t tn;
	int i;

	LOGP(DL1C, LOGL_NOTICE, "Exit scheduler for trx=%u\n", l1t->trx->nr);

	for (tn = 0; tn < ARRAY_SIZE(l1t->ts); tn++) {
		struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);
		msgb_queue_flush(&l1ts->dl_prims);
		for (i = 0; i < _TRX_CHAN_MAX; i++) {
			struct l1sched_chan_state *chan_state;
			chan_state = &l1ts->chan_state[i];
			if (chan_state->dl_bursts) {
				talloc_free(chan_state->dl_bursts);
				chan_state->dl_bursts = NULL;
			}
			if (chan_state->ul_bursts) {
				talloc_free(chan_state->ul_bursts);
				chan_state->ul_bursts = NULL;
			}
		}
		/* clear lchan channel states */
		ts = &l1t->trx->ts[tn];
		for (i = 0; i < ARRAY_SIZE(ts->lchan); i++)
			lchan_set_state(&ts->lchan[i], LCHAN_S_NONE);
	}
}

/* close all logical channels and reset timeslots */
void trx_sched_reset(struct l1sched_trx *l1t)
{
	trx_sched_exit(l1t);
	trx_sched_init(l1t, l1t->trx);
}

struct msgb *_sched_dequeue_prim(struct l1sched_trx *l1t, int8_t tn, uint32_t fn,
				 enum trx_chan_type chan)
{
	struct msgb *msg, *msg2;
	struct osmo_phsap_prim *l1sap;
	uint32_t prim_fn;
	uint8_t chan_nr, link_id;
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);

	/* get prim of current fn from queue */
	llist_for_each_entry_safe(msg, msg2, &l1ts->dl_prims, list) {
		l1sap = msgb_l1sap_prim(msg);
		if (l1sap->oph.operation != PRIM_OP_REQUEST) {
wrong_type:
			LOGL1S(DL1P, LOGL_ERROR, l1t, tn, chan, fn, "Prim has wrong type.\n");
free_msg:
			/* unlink and free message */
			llist_del(&msg->list);
			msgb_free(msg);
			return NULL;
		}
		switch (l1sap->oph.primitive) {
		case PRIM_PH_DATA:
			chan_nr = l1sap->u.data.chan_nr;
			link_id = l1sap->u.data.link_id;
			prim_fn = ((l1sap->u.data.fn + GSM_HYPERFRAME - fn) % GSM_HYPERFRAME);
			break;
		case PRIM_TCH:
			chan_nr = l1sap->u.tch.chan_nr;
			link_id = 0;
			prim_fn = ((l1sap->u.tch.fn + GSM_HYPERFRAME - fn) % GSM_HYPERFRAME);
			break;
		default:
			goto wrong_type;
		}
		if (prim_fn > 100) {
			LOGL1S(DL1P, LOGL_NOTICE, l1t, tn, chan, fn,
			     "Prim %u is out of range (100), or channel %s with "
			     "type %s is already disabled. If this happens in "
			     "conjunction with PCU, increase 'rts-advance' by 5.\n",
			     prim_fn, get_lchan_by_chan_nr(l1t->trx, chan_nr)->name,
			     trx_chan_desc[chan].name);
			/* unlink and free message */
			llist_del(&msg->list);
			msgb_free(msg);
			continue;
		}
		if (prim_fn > 0)
			continue;

		goto found_msg;
	}

	return NULL;

found_msg:
	if ((chan_nr ^ (trx_chan_desc[chan].chan_nr | tn))
	 || ((link_id & 0xc0) ^ trx_chan_desc[chan].link_id)) {
		LOGL1S(DL1P, LOGL_ERROR, l1t, tn, chan, fn, "Prim has wrong chan_nr=0x%02x link_id=%02x, "
			"expecting chan_nr=0x%02x link_id=%02x.\n", chan_nr, link_id,
			trx_chan_desc[chan].chan_nr | tn, trx_chan_desc[chan].link_id);
		goto free_msg;
	}

	/* unlink and return message */
	llist_del(&msg->list);
	return msg;
}

int _sched_compose_ph_data_ind(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
			       enum trx_chan_type chan, uint8_t *l2,
			       uint8_t l2_len, float rssi,
			       int16_t ta_offs_256bits, int16_t link_qual_cb,
			       uint16_t ber10k,
			       enum osmo_ph_pres_info_type presence_info)
{
	struct msgb *msg;
	struct osmo_phsap_prim *l1sap;
	uint8_t chan_nr = trx_chan_desc[chan].chan_nr | tn;
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);

	/* compose primitive */
	msg = l1sap_msgb_alloc(l2_len);
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
	msg->l2h = msgb_put(msg, l2_len);
	if (l2_len)
		memcpy(msg->l2h, l2, l2_len);

	if (L1SAP_IS_LINK_SACCH(trx_chan_desc[chan].link_id))
		l1ts->chan_state[chan].lost_frames = 0;

	/* forward primitive */
	l1sap_up(l1t->trx, l1sap);

	return 0;
}

int _sched_compose_tch_ind(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
			   enum trx_chan_type chan, uint8_t *tch, uint8_t tch_len,
			   int16_t ta_offs_256bits, uint16_t ber10k, float rssi,
			   uint8_t is_sub)
{
	struct msgb *msg;
	struct osmo_phsap_prim *l1sap;
	struct gsm_bts_trx *trx = l1t->trx;
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);
	uint8_t chan_nr = trx_chan_desc[chan].chan_nr | tn;
	struct gsm_lchan *lchan = &trx->ts[L1SAP_CHAN2TS(chan_nr)].lchan[l1sap_chan2ss(chan_nr)];

	/* compose primitive */
	msg = l1sap_msgb_alloc(tch_len);
	l1sap = msgb_l1sap_prim(msg);
	osmo_prim_init(&l1sap->oph, SAP_GSM_PH, PRIM_TCH,
		PRIM_OP_INDICATION, msg);
	l1sap->u.tch.chan_nr = chan_nr;
	l1sap->u.tch.fn = fn;
	l1sap->u.tch.rssi = (int8_t) (rssi);
	l1sap->u.tch.ber10k = ber10k;
	l1sap->u.tch.ta_offs_256bits = ta_offs_256bits;
	l1sap->u.tch.is_sub = is_sub & 1;

	msg->l2h = msgb_put(msg, tch_len);
	if (tch_len)
		memcpy(msg->l2h, tch, tch_len);

	if (l1ts->chan_state[chan].lost_frames)
		l1ts->chan_state[chan].lost_frames--;

	LOGL1S(DL1P, LOGL_DEBUG, l1t, tn, -1, l1sap->u.data.fn,
	       "%s Rx -> RTP: %s\n",
	       gsm_lchan_name(lchan), osmo_hexdump(msgb_l2(msg), msgb_l2len(msg)));
	/* forward primitive */
	l1sap_up(l1t->trx, l1sap);

	return 0;
}



/*
 * data request (from upper layer)
 */

int trx_sched_ph_data_req(struct l1sched_trx *l1t, struct osmo_phsap_prim *l1sap)
{
	uint8_t tn = l1sap->u.data.chan_nr & 7;
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);

	LOGL1S(DL1P, LOGL_DEBUG, l1t, tn, -1, l1sap->u.data.fn,
		"PH-DATA.req: chan_nr=0x%02x link_id=0x%02x\n",
		l1sap->u.data.chan_nr, l1sap->u.data.link_id);

	OSMO_ASSERT(l1sap->oph.msg);

	/* ignore empty frame */
	if (!msgb_l2len(l1sap->oph.msg)) {
		msgb_free(l1sap->oph.msg);
		return 0;
	}

	msgb_enqueue(&l1ts->dl_prims, l1sap->oph.msg);

	return 0;
}

int trx_sched_tch_req(struct l1sched_trx *l1t, struct osmo_phsap_prim *l1sap)
{
	uint8_t tn = l1sap->u.tch.chan_nr & 7;
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);

	LOGL1S(DL1P, LOGL_DEBUG, l1t, tn, -1, l1sap->u.tch.fn, "TCH.req: chan_nr=0x%02x\n",
		l1sap->u.tch.chan_nr);

	OSMO_ASSERT(l1sap->oph.msg);

	/* ignore empty frame */
	if (!msgb_l2len(l1sap->oph.msg)) {
		msgb_free(l1sap->oph.msg);
		return 0;
	}

	msgb_enqueue(&l1ts->dl_prims, l1sap->oph.msg);

	return 0;
}


/* 
 * ready-to-send indication (to upper layer)
 */

/* RTS for data frame */
static int rts_data_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan)
{
	uint8_t chan_nr, link_id;
	struct msgb *msg;
	struct osmo_phsap_prim *l1sap;

	/* get data for RTS indication */
	chan_nr = trx_chan_desc[chan].chan_nr | tn;
	link_id = trx_chan_desc[chan].link_id;

	if (!chan_nr) {
		LOGL1S(DL1P, LOGL_FATAL, l1t, tn, chan, fn,
			"RTS func with non-existing chan_nr %d\n", chan_nr);
		return -ENODEV;
	}

	LOGL1S(DL1P, LOGL_DEBUG, l1t, tn, chan, fn,
		"PH-RTS.ind: chan_nr=0x%02x link_id=0x%02x\n", chan_nr, link_id);

	/* generate prim */
	msg = l1sap_msgb_alloc(200);
	if (!msg)
		return -ENOMEM;
	l1sap = msgb_l1sap_prim(msg);
	osmo_prim_init(&l1sap->oph, SAP_GSM_PH, PRIM_PH_RTS,
	                                PRIM_OP_INDICATION, msg);
	l1sap->u.data.chan_nr = chan_nr;
	l1sap->u.data.link_id = link_id;
	l1sap->u.data.fn = fn;

	return l1sap_up(l1t->trx, l1sap);
}

static int rts_tch_common(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan, int facch)
{
	uint8_t chan_nr, link_id;
	struct msgb *msg;
	struct osmo_phsap_prim *l1sap;
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);
	int rc = 0;

	/* get data for RTS indication */
	chan_nr = trx_chan_desc[chan].chan_nr | tn;
	link_id = trx_chan_desc[chan].link_id;

	if (!chan_nr) {
		LOGL1S(DL1P, LOGL_FATAL, l1t, tn, chan, fn,
			"RTS func with non-existing chan_nr %d\n", chan_nr);
		return -ENODEV;
	}

	LOGL1S(DL1P, LOGL_DEBUG, l1t, tn, chan, fn, "TCH RTS.ind: chan_nr=0x%02x\n", chan_nr);

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
		l1sap->u.data.fn = fn;

		rc = l1sap_up(l1t->trx, l1sap);
	}

	/* don't send, if TCH is in signalling only mode */
	if (l1ts->chan_state[chan].rsl_cmode != RSL_CMOD_SPD_SIGN) {
		/* generate prim */
		msg = l1sap_msgb_alloc(200);
		if (!msg)
			return -ENOMEM;
		l1sap = msgb_l1sap_prim(msg);
		osmo_prim_init(&l1sap->oph, SAP_GSM_PH, PRIM_TCH_RTS,
						PRIM_OP_INDICATION, msg);
		l1sap->u.tch.chan_nr = chan_nr;
		l1sap->u.tch.fn = fn;

		return l1sap_up(l1t->trx, l1sap);
	}

	return rc;
}

/* RTS for full rate traffic frame */
static int rts_tchf_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan)
{
	/* TCH/F may include FACCH on every 4th burst */
	return rts_tch_common(l1t, tn, fn, chan, 1);
}


/* RTS for half rate traffic frame */
static int rts_tchh_fn(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn,
	enum trx_chan_type chan)
{
	/* the FN 4/5, 13/14, 21/22 defines that FACCH may be included. */
	return rts_tch_common(l1t, tn, fn, chan, ((fn % 26) >> 2) & 1);
}

/* set multiframe scheduler to given pchan */
int trx_sched_set_pchan(struct l1sched_trx *l1t, uint8_t tn,
	enum gsm_phys_chan_config pchan)
{
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);
	int i;

	i = find_sched_mframe_idx(pchan, tn);
	if (i < 0) {
		LOGP(DL1C, LOGL_NOTICE, "Failed to configure multiframe "
			"trx=%d ts=%d\n", l1t->trx->nr, tn);
		return -ENOTSUP;
	}
	l1ts->mf_index = i;
	l1ts->mf_period = trx_sched_multiframes[i].period;
	l1ts->mf_frames = trx_sched_multiframes[i].frames;
	LOGP(DL1C, LOGL_NOTICE, "Configuring multiframe with %s trx=%d ts=%d\n",
		trx_sched_multiframes[i].name, l1t->trx->nr, tn);
	return 0;
}

/* setting all logical channels given attributes to active/inactive */
int trx_sched_set_lchan(struct l1sched_trx *l1t, uint8_t chan_nr, uint8_t link_id,
	int active)
{
	uint8_t tn = L1SAP_CHAN2TS(chan_nr);
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);
	uint8_t ss = l1sap_chan2ss(chan_nr);
	int i;
	int rc = -EINVAL;

	/* look for all matching chan_nr/link_id */
	for (i = 0; i < _TRX_CHAN_MAX; i++) {
		struct l1sched_chan_state *chan_state;
		chan_state = &l1ts->chan_state[i];
		/* Skip if pchan type does not match pdch flag.
		 * FIXME: Is it possible at all? Clarify if so. */
		if ((trx_sched_multiframes[l1ts->mf_index].pchan == GSM_PCHAN_PDCH)
		    && !(trx_chan_desc[i].flags & TRX_CHAN_FLAG_PDCH))
			continue;
		if (trx_chan_desc[i].chan_nr == (chan_nr & 0xf8)
		 && trx_chan_desc[i].link_id == link_id) {
			rc = 0;
			if (chan_state->active == active)
				continue;
			LOGP(DL1C, LOGL_NOTICE, "%s %s on trx=%d ts=%d\n",
				(active) ? "Activating" : "Deactivating",
				trx_chan_desc[i].name, l1t->trx->nr, tn);
			if (active)
				memset(chan_state, 0, sizeof(*chan_state));
			chan_state->active = active;
			/* free burst memory, to cleanly start with burst 0 */
			if (chan_state->dl_bursts) {
				talloc_free(chan_state->dl_bursts);
				chan_state->dl_bursts = NULL;
			}
			if (chan_state->ul_bursts) {
				talloc_free(chan_state->ul_bursts);
				chan_state->ul_bursts = NULL;
			}
			if (!active)
				chan_state->ho_rach_detect = 0;
		}
	}

	/* disable handover detection (on deactivation) */
	if (!active)
		_sched_act_rach_det(l1t, tn, ss, 0);

	return rc;
}

/* setting all logical channels given attributes to active/inactive */
int trx_sched_set_mode(struct l1sched_trx *l1t, uint8_t chan_nr, uint8_t rsl_cmode,
	uint8_t tch_mode, int codecs, uint8_t codec0, uint8_t codec1,
	uint8_t codec2, uint8_t codec3, uint8_t initial_id, uint8_t handover)
{
	uint8_t tn = L1SAP_CHAN2TS(chan_nr);
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);
	uint8_t ss = l1sap_chan2ss(chan_nr);
	int i;
	int rc = -EINVAL;
	struct l1sched_chan_state *chan_state;

	/* no mode for PDCH */
	if (trx_sched_multiframes[l1ts->mf_index].pchan == GSM_PCHAN_PDCH)
		return 0;

	/* look for all matching chan_nr/link_id */
	for (i = 0; i < _TRX_CHAN_MAX; i++) {
		if (trx_chan_desc[i].chan_nr == (chan_nr & 0xf8)
		 && trx_chan_desc[i].link_id == 0x00) {
			chan_state = &l1ts->chan_state[i];
			LOGP(DL1C, LOGL_NOTICE, "Set mode %u, %u, handover %u "
				"on %s of trx=%d ts=%d\n", rsl_cmode, tch_mode,
				handover, trx_chan_desc[i].name, l1t->trx->nr,
				tn);
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
				chan_state->ber_sum = 0;
				chan_state->ber_num = 0;
			}
			rc = 0;
		}
	}

	/* command rach detection
	 * always enable handover, even if state is still set (due to loss
	 * of transceiver link).
	 * disable handover, if state is still set, since we might not know
	 * the actual state of transceiver (due to loss of link) */
	_sched_act_rach_det(l1t, tn, ss, handover);

	return rc;
}

/* setting cipher on logical channels */
int trx_sched_set_cipher(struct l1sched_trx *l1t, uint8_t chan_nr, int downlink,
	int algo, uint8_t *key, int key_len)
{
	uint8_t tn = L1SAP_CHAN2TS(chan_nr);
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);
	int i;
	int rc = -EINVAL;
	struct l1sched_chan_state *chan_state;

	/* no cipher for PDCH */
	if (trx_sched_multiframes[l1ts->mf_index].pchan == GSM_PCHAN_PDCH)
		return 0;

	/* no algorithm given means a5/0 */
	if (algo <= 0)
		algo = 0;
	else if (key_len != 8) {
		LOGP(DL1C, LOGL_ERROR, "Algo A5/%d not supported with given "
			"key len=%d\n", algo, key_len);
		return -ENOTSUP;
	}

	/* look for all matching chan_nr */
	for (i = 0; i < _TRX_CHAN_MAX; i++) {
		/* skip if pchan type */
		if (trx_chan_desc[i].flags & TRX_CHAN_FLAG_PDCH)
			continue;
		if (trx_chan_desc[i].chan_nr == (chan_nr & 0xf8)) {
			chan_state = &l1ts->chan_state[i];
			LOGP(DL1C, LOGL_NOTICE, "Set a5/%d %s for %s on trx=%d "
				"ts=%d\n", algo,
				(downlink) ? "downlink" : "uplink",
				trx_chan_desc[i].name, l1t->trx->nr, tn);
			if (downlink) {
				chan_state->dl_encr_algo = algo;
				memcpy(chan_state->dl_encr_key, key, key_len);
				chan_state->dl_encr_key_len = key_len;
			} else {
				chan_state->ul_encr_algo = algo;
				memcpy(chan_state->ul_encr_key, key, key_len);
				chan_state->ul_encr_key_len = key_len;
			}
			rc = 0;
		}
	}

	return rc;
}

/* process ready-to-send */
int _sched_rts(struct l1sched_trx *l1t, uint8_t tn, uint32_t fn)
{
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);
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
	if (!TRX_CHAN_IS_ACTIVE(&l1ts->chan_state[chan], chan))
	 	return -EINVAL;

	return func(l1t, tn, fn, frame->dl_chan);
}

/* process downlink burst */
const ubit_t *_sched_dl_burst(struct l1sched_trx *l1t, uint8_t tn,
				uint32_t fn, uint16_t *nbits)
{
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, tn);
	struct l1sched_chan_state *l1cs;
	const struct trx_sched_frame *frame;
	uint8_t offset, period, bid;
	trx_sched_dl_func *func;
	enum trx_chan_type chan;
	ubit_t *bits = NULL;

	if (!l1ts->mf_index)
		goto no_data;

	/* get frame from multiframe */
	period = l1ts->mf_period;
	offset = fn % period;
	frame = l1ts->mf_frames + offset;

	chan = frame->dl_chan;
	bid = frame->dl_bid;
	func = trx_chan_desc[chan].dl_fn;

	l1cs = &l1ts->chan_state[chan];

	/* check if channel is active */
	if (!TRX_CHAN_IS_ACTIVE(l1cs, chan)) {
		if (nbits)
			*nbits = GSM_BURST_LEN;
		goto no_data;
	}

	/* get burst from function */
	bits = func(l1t, tn, fn, chan, bid, nbits);

	/* encrypt */
	if (bits && l1cs->dl_encr_algo) {
		ubit_t ks[114];
		int i;

		osmo_a5(l1cs->dl_encr_algo, l1cs->dl_encr_key, fn, ks, NULL);
		for (i = 0; i < 57; i++) {
			bits[i + 3] ^= ks[i];
			bits[i + 88] ^= ks[i + 57];
		}
	}

no_data:
	/* in case of C0, we need a dummy burst to maintain RF power */
	if (bits == NULL && l1t->trx == l1t->trx->bts->c0) {
#if 0
		if (chan != TRXC_IDLE) // hack
			LOGP(DL1C, LOGL_DEBUG, "No burst data for %s fn=%u ts=%u "
			     "burst=%d on C0, so filling with dummy burst\n",
			     trx_chan_desc[chan].name, fn, tn, bid);
#endif
		bits = (ubit_t *) dummy_burst;
	}

	return bits;
}

#define TDMA_FN_SUM(a, b) \
	((a + GSM_HYPERFRAME + b) % GSM_HYPERFRAME)

#define TDMA_FN_SUB(a, b) \
	((a + GSM_HYPERFRAME - b) % GSM_HYPERFRAME)

static int trx_sched_calc_frame_loss(struct l1sched_trx *l1t,
	struct l1sched_chan_state *l1cs, uint8_t tn, uint32_t fn)
{
	const struct trx_sched_frame *frame_head;
	const struct trx_sched_frame *frame;
	struct l1sched_ts *l1ts;
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

	/* Get current TDMA frame info */
	l1ts = l1sched_trx_get_ts(l1t, tn);
	offset = fn % l1ts->mf_period;
	frame_head = l1ts->mf_frames + offset;

	/* Not applicable for some logical channels */
	switch (frame_head->ul_chan) {
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
	elapsed_fs = TDMA_FN_SUB(fn, l1cs->last_tdma_fn);
	if (elapsed_fs > l1ts->mf_period) { /* Too many! */
		LOGL1S(DL1P, LOGL_ERROR, l1t, tn, frame_head->ul_chan, fn,
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
		fn_i = TDMA_FN_SUM(l1cs->last_tdma_fn, i);
		offset = fn_i % l1ts->mf_period;
		frame = l1ts->mf_frames + offset;

		if (frame->ul_chan == frame_head->ul_chan)
			l1cs->lost_tdma_fs++;
	}

	if (l1cs->lost_tdma_fs > 0) {
		LOGL1S(DL1P, LOGL_NOTICE, l1t, tn, frame_head->ul_chan, fn,
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
		struct trx_ul_burst_ind bi = {
			.flags = TRX_BI_F_NOPE_IND,
			.burst_len = GSM_BURST_LEN,
			.burst = { 0 },
			.rssi = -128,
			.toa256 = 0,
			/* TDMA FN is set below */
			.tn = tn,
		};

		for (i = 1; i < elapsed_fs; i++) {
			fn_i = TDMA_FN_SUM(l1cs->last_tdma_fn, i);
			offset = fn_i % l1ts->mf_period;
			frame = l1ts->mf_frames + offset;
			func = trx_chan_desc[frame->ul_chan].ul_fn;

			if (frame->ul_chan != frame_head->ul_chan)
				continue;

			LOGL1S(DL1P, LOGL_NOTICE, l1t, tn, frame->ul_chan, fn,
				"Substituting lost TDMA frame=%u by all-zero "
				"dummy burst\n", fn_i);

			bi.fn = fn_i;
			func(l1t, frame->ul_chan, frame->ul_bid, &bi);

			l1cs->lost_tdma_fs--;
		}
	}

	return 0;
}

/* Process an Uplink burst indication */
int trx_sched_ul_burst(struct l1sched_trx *l1t, struct trx_ul_burst_ind *bi)
{
	struct l1sched_ts *l1ts = l1sched_trx_get_ts(l1t, bi->tn);
	struct l1sched_chan_state *l1cs;
	const struct trx_sched_frame *frame;
	uint8_t offset, period, bid;
	trx_sched_ul_func *func;
	enum trx_chan_type chan;

	if (!l1ts->mf_index)
		return -EINVAL;

	/* get frame from multiframe */
	period = l1ts->mf_period;
	offset = bi->fn % period;
	frame = l1ts->mf_frames + offset;

	chan = frame->ul_chan;
	bid = frame->ul_bid;
	l1cs = &l1ts->chan_state[chan];
	func = trx_chan_desc[chan].ul_fn;

	/* TODO: handle noise measurements */
	if (chan == TRXC_IDLE && bi->flags & TRX_BI_F_NOPE_IND) {
		LOGL1S(DL1P, LOGL_DEBUG, l1t, bi->tn, chan, bi->fn,
		       "Rx noise measurement (%d)\n", bi->rssi);
		return -ENOTSUP;
	}

	/* check if channel is active */
	if (!TRX_CHAN_IS_ACTIVE(l1cs, chan))
		return -EINVAL;

	/* omit bursts which have no handler, like IDLE bursts */
	if (!func)
		return -EINVAL;

	/* calculate how many TDMA frames were potentially lost */
	trx_sched_calc_frame_loss(l1t, l1cs, bi->tn, bi->fn);

	/* update TDMA frame counters */
	l1cs->last_tdma_fn = bi->fn;
	l1cs->proc_tdma_fs++;

	/* handle NOPE indications */
	if (bi->flags & TRX_BI_F_NOPE_IND) {
		switch (chan) {
		case TRXC_PDTCH:
		case TRXC_PTCCH:
		case TRXC_RACH:
			/* For some logical channel types NOPE.ind is valueless. */
			return 0;
		default:
			/* NOTE: Uplink burst handler must check bi->burst_len before
			 * accessing bi->burst to avoid uninitialized memory access. */
			return func(l1t, chan, bid, bi);
		}
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
	func(l1t, chan, bid, bi);

	return 0;
}

struct l1sched_ts *l1sched_trx_get_ts(struct l1sched_trx *l1t, uint8_t tn)
{
	OSMO_ASSERT(tn < ARRAY_SIZE(l1t->ts));
	return &l1t->ts[tn];
}
