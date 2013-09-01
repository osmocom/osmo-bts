/* BTS support code common to all supported BTS models */

/* (C) 2011 by Andreas Eversberg <jolly@eversberg.eu>
 * (C) 2011 by Harald Welte <laforge@gnumonks.org>
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

#include <errno.h>
#include <unistd.h>
#include <stdio.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <osmocom/core/select.h>
#include <osmocom/core/timer.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/talloc.h>
#include <osmocom/gsm/protocol/gsm_12_21.h>
#include <osmocom/gsm/lapdm.h>
#include <osmocom/trau/osmo_ortp.h>

#include <osmo-bts/logging.h>
#include <osmo-bts/abis.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/bts_model.h>
#include <osmo-bts/rsl.h>
#include <osmo-bts/oml.h>
#include <osmo-bts/signal.h>


struct gsm_network bts_gsmnet = {
	.bts_list = { &bts_gsmnet.bts_list, &bts_gsmnet.bts_list },
	.num_bts = 0,
};

void *tall_bts_ctx;

/* Table 3.1 TS 04.08: Values of parameter S */
static const uint8_t tx_integer[] = {
	3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 16, 20, 25, 32, 50,
};

static const uint8_t s_values[][2] = {
	{ 55, 41 }, { 76, 52 }, { 109, 58 }, { 163, 86 }, { 217, 115 },
};

static int bts_signal_cbfn(unsigned int subsys, unsigned int signal,
			   void *hdlr_data, void *signal_data)
{
	if (subsys == SS_GLOBAL && signal == S_NEW_SYSINFO) {
		struct gsm_bts *bts = signal_data;

		bts_update_agch_max_queue_length(bts);
	}
	return 0;
}

int bts_init(struct gsm_bts *bts)
{
	struct gsm_bts_role_bts *btsb;
	struct gsm_bts_trx *trx;
	int rc;
	static int initialized = 0;

	/* add to list of BTSs */
	llist_add_tail(&bts->list, &bts_gsmnet.bts_list);

	bts->band = GSM_BAND_1800;

	bts->role = btsb = talloc_zero(bts, struct gsm_bts_role_bts);

	INIT_LLIST_HEAD(&btsb->agch_queue);
	btsb->agch_queue_length = 0;

	/* enable management with default levels,
	 * raise threshold to GSM_BTS_AGCH_QUEUE_THRESH_LEVEL_DISABLE to
	 * disable this feature.
	 */
	btsb->agch_queue_low_level = GSM_BTS_AGCH_QUEUE_LOW_LEVEL_DEFAULT;
	btsb->agch_queue_high_level = GSM_BTS_AGCH_QUEUE_HIGH_LEVEL_DEFAULT;
	btsb->agch_queue_thresh_level = GSM_BTS_AGCH_QUEUE_THRESH_LEVEL_DEFAULT;

	/* configurable via VTY */
	btsb->paging_state = paging_init(btsb, 200, 0);

	/* configurable via OML */
	btsb->load.ccch.load_ind_period = 112;
	load_timer_start(bts);
	btsb->rtp_jitter_buf_ms = 100;
	btsb->max_ta = 63;
	btsb->ny1 = 4;
	btsb->t3105_ms = 300;

	/* default RADIO_LINK_TIMEOUT */
	btsb->radio_link_timeout = 32;

	/* set BTS to dependency */
	oml_mo_state_init(&bts->mo, -1, NM_AVSTATE_DEPENDENCY);
	oml_mo_state_init(&bts->gprs.nse.mo, -1, NM_AVSTATE_DEPENDENCY);
	oml_mo_state_init(&bts->gprs.cell.mo, -1, NM_AVSTATE_DEPENDENCY);
	oml_mo_state_init(&bts->gprs.nsvc[0].mo, -1, NM_AVSTATE_DEPENDENCY);
	oml_mo_state_init(&bts->gprs.nsvc[1].mo, -1, NM_AVSTATE_DEPENDENCY);

	/* initialize bts data structure */
	llist_for_each_entry(trx, &bts->trx_list, list) {
		int i;
		for (i = 0; i < ARRAY_SIZE(trx->ts); i++) {
			struct gsm_bts_trx_ts *ts = &trx->ts[i];
			int k;

			for (k = 0; k < ARRAY_SIZE(ts->lchan); k++) {
				struct gsm_lchan *lchan = &ts->lchan[k];
				INIT_LLIST_HEAD(&lchan->dl_tch_queue);
			}
		}
	}

	osmo_rtp_init(tall_bts_ctx);

	rc = bts_model_init(bts);
	if (rc < 0) {
		llist_del(&bts->list);
		return rc;
	}

	bts_gsmnet.num_bts++;

	if (!initialized) {
		osmo_signal_register_handler(SS_GLOBAL, bts_signal_cbfn, NULL);
		initialized = 1;
	}

	return rc;
}

static void shutdown_timer_cb(void *data)
{
	fprintf(stderr, "Shutdown timer expired\n");
	exit(42);
}

static struct osmo_timer_list shutdown_timer = {
	.cb = &shutdown_timer_cb,
};

void bts_shutdown(struct gsm_bts *bts, const char *reason)
{
	struct gsm_bts_trx *trx;

	if (osmo_timer_pending(&shutdown_timer)) {
		LOGP(DOML, LOGL_NOTICE,
			"BTS is already being shutdown.\n");
		return;
	}

	LOGP(DOML, LOGL_NOTICE, "Shutting down BTS %u, Reason %s\n",
		bts->nr, reason);

	llist_for_each_entry(trx, &bts->trx_list, list) {
		bts_model_trx_deact_rf(trx);
		bts_model_trx_close(trx);
	}

	/* shedule a timer to make sure select loop logic can run again
	 * to dispatch any pending primitives */
	osmo_timer_schedule(&shutdown_timer, 3, 0);
}

/* main link is established, send status report */
int bts_link_estab(struct gsm_bts *bts)
{
	int i, j;

	LOGP(DSUM, LOGL_INFO, "Main link established, sending Status'.\n");

	/* BTS and SITE MGR are EAABLED, BTS is DEPENDENCY */
	oml_tx_state_changed(&bts->site_mgr.mo);
	oml_tx_state_changed(&bts->mo);

	/* those should all be in DEPENDENCY */
	oml_tx_state_changed(&bts->gprs.nse.mo);
	oml_tx_state_changed(&bts->gprs.cell.mo);
	oml_tx_state_changed(&bts->gprs.nsvc[0].mo);
	oml_tx_state_changed(&bts->gprs.nsvc[1].mo);

	/* All other objects start off-line until the BTS Model code says otherwise */
	for (i = 0; i < bts->num_trx; i++) {
		struct gsm_bts_trx *trx = gsm_bts_trx_num(bts, i);

		oml_tx_state_changed(&trx->mo);
		oml_tx_state_changed(&trx->bb_transc.mo);

		for (j = 0; j < ARRAY_SIZE(trx->ts); j++) {
			struct gsm_bts_trx_ts *ts = &trx->ts[j];

			oml_tx_state_changed(&ts->mo);
		}
	}

	return bts_model_oml_estab(bts);
}

/* RSL link is established, send status report */
int trx_link_estab(struct gsm_bts_trx *trx)
{
	struct e1inp_sign_link *link = trx->rsl_link;
	uint8_t radio_state = link ?  NM_OPSTATE_ENABLED : NM_OPSTATE_DISABLED;

	LOGP(DSUM, LOGL_INFO, "RSL link (TRX %02x) state changed to %s, sending Status'.\n",
		trx->nr, link ? "up" : "down");

	oml_mo_state_chg(&trx->mo, radio_state, NM_AVSTATE_OK);

	if (link)
		rsl_tx_rf_res(trx);

	return 0;
}

int lchan_init_lapdm(struct gsm_lchan *lchan)
{
	struct lapdm_channel *lc = &lchan->lapdm_ch;

	lapdm_channel_init(lc, LAPDM_MODE_BTS);
	lapdm_channel_set_flags(lc, LAPDM_ENT_F_POLLING_ONLY);
	lapdm_channel_set_l1(lc, NULL, lchan);
	lapdm_channel_set_l3(lc, lapdm_rll_tx_cb, lchan);

	return 0;
}

#define CCCH_RACH_RATIO_COMBINED256      (256*1/9)
#define CCCH_RACH_RATIO_SEPARATE256      (256*10/55)

int bts_agch_max_queue_length(int T, int bcch_conf)
{
	int S, ccch_rach_ratio256, i;
	int T_group = 0;
	int is_ccch_comb = 0;

	if (bcch_conf == RSL_BCCH_CCCH_CONF_1_C)
		is_ccch_comb = 1;

	/*
	 * The calculation is based on the ratio of the number RACH slots and
	 * CCCH blocks per time:
	 *   Lmax = (T + 2*S) / R_RACH * R_CCCH
	 * where
	 *   T3126_min = (T + 2*S) / R_RACH, as defined in GSM 04.08, 11.1.1
	 *   R_RACH is the RACH slot rate (e.g. RACHs per multiframe)
	 *   R_CCCH is the CCCH block rate (same time base like R_RACH)
	 *   S and T are defined in GSM 04.08, 3.3.1.1.2
	 * The ratio is mainly influenced by the downlink only channels
	 * (BCCH, FCCH, SCH, CBCH) that can not be used for CCCH.
	 * An estimation with an error of < 10% is used:
	 *   ~ 1/9 if CCCH is combined with SDCCH, and
	 *   ~ 1/5.5 otherwise.
	 */
	ccch_rach_ratio256 = is_ccch_comb ?
		CCCH_RACH_RATIO_COMBINED256 :
		CCCH_RACH_RATIO_SEPARATE256;

	for (i = 0; i < ARRAY_SIZE(tx_integer); i++) {
		if (tx_integer[i] == T) {
			T_group = i % 5;
			break;
		}
	}
	S = s_values[T_group][is_ccch_comb];

	return (T + 2 * S) * ccch_rach_ratio256 / 256;
}

void bts_update_agch_max_queue_length(struct gsm_bts *bts)
{
	struct gsm_bts_role_bts *btsb = bts_role_bts(bts);
	struct gsm48_system_information_type_3 *si3;
	int old_max_length = btsb->agch_max_queue_length;

	if (!(bts->si_valid & (1<<SYSINFO_TYPE_3)))
		return;

	si3 = GSM_BTS_SI(bts, SYSINFO_TYPE_3);

	btsb->agch_max_queue_length =
		bts_agch_max_queue_length(si3->rach_control.tx_integer,
					  si3->control_channel_desc.ccch_conf);

	if (btsb->agch_max_queue_length != old_max_length)
		LOGP(DRSL, LOGL_INFO, "Updated AGCH max queue length to %d\n",
		     btsb->agch_max_queue_length);
}

#define REQ_REFS_PER_IMM_ASS_REJ 4
static int store_imm_ass_rej_refs(struct gsm48_imm_ass_rej *rej,
				    struct gsm48_req_ref *req_refs,
				    uint8_t *wait_inds,
				    int count)
{
	switch (count) {
	case 0:
		/* TODO: Warning ? */
		return 0;
	default:
		count = 4;
		rej->req_ref4 = req_refs[3];
		rej->wait_ind4 = wait_inds[3];
		/* fall through */
	case 3:
		rej->req_ref3 = req_refs[2];
		rej->wait_ind3 = wait_inds[2];
		/* fall through */
	case 2:
		rej->req_ref2 = req_refs[1];
		rej->wait_ind2 = wait_inds[1];
		/* fall through */
	case 1:
		rej->req_ref1 = req_refs[0];
		rej->wait_ind1 = wait_inds[0];
		break;
	}

	switch (count) {
	case 1:
		rej->req_ref2 = req_refs[0];
		rej->wait_ind2 = wait_inds[0];
		/* fall through */
	case 2:
		rej->req_ref3 = req_refs[0];
		rej->wait_ind3 = wait_inds[0];
		/* fall through */
	case 3:
		rej->req_ref4 = req_refs[0];
		rej->wait_ind4 = wait_inds[0];
		/* fall through */
	default:
		break;
	}

	return count;
}

static int extract_imm_ass_rej_refs(struct gsm48_imm_ass_rej *rej,
				    struct gsm48_req_ref *req_refs,
				    uint8_t *wait_inds)
{
	int count = 0;
	req_refs[count] = rej->req_ref1;
	wait_inds[count] = rej->wait_ind1;
	count++;

	if (memcmp(&rej->req_ref1, &rej->req_ref2, sizeof(rej->req_ref2))) {
		req_refs[count] = rej->req_ref2;
		wait_inds[count] = rej->wait_ind2;
		count++;
	}

	if (memcmp(&rej->req_ref1, &rej->req_ref3, sizeof(rej->req_ref3)) &&
	    memcmp(&rej->req_ref2, &rej->req_ref3, sizeof(rej->req_ref3))) {
		req_refs[count] = rej->req_ref3;
		wait_inds[count] = rej->wait_ind3;
		count++;
	}

	if (memcmp(&rej->req_ref1, &rej->req_ref4, sizeof(rej->req_ref4)) &&
	    memcmp(&rej->req_ref2, &rej->req_ref4, sizeof(rej->req_ref4)) &&
	    memcmp(&rej->req_ref3, &rej->req_ref4, sizeof(rej->req_ref4))) {
		req_refs[count] = rej->req_ref4;
		wait_inds[count] = rej->wait_ind4;
		count++;
	}

	return count;
}

static int try_merge_imm_ass_rej(struct gsm48_imm_ass_rej *old_rej,
				 struct gsm48_imm_ass_rej *new_rej)
{
	struct gsm48_req_ref req_refs[2 * REQ_REFS_PER_IMM_ASS_REJ];
	uint8_t wait_inds[2 * REQ_REFS_PER_IMM_ASS_REJ];
	int count = 0;
	int stored = 0;

	if (new_rej->msg_type != GSM48_MT_RR_IMM_ASS_REJ)
		return 0;
	if (old_rej->msg_type != GSM48_MT_RR_IMM_ASS_REJ)
		return 0;

	/* GSM 08.58, 5.7
	 * -> The BTS may combine serveral IMM.ASS.REJ messages
	 * -> Identical request refs in one message may be squeezed
	 *
	 * GSM 04.08, 9.1.20.2
	 * -> Request ref and wait ind are duplicated to fill the message
	 */

	/* Extract all entries */
	count = extract_imm_ass_rej_refs(old_rej,
					 &req_refs[count], &wait_inds[count]);
	if (count == REQ_REFS_PER_IMM_ASS_REJ)
		return 0;

	count += extract_imm_ass_rej_refs(new_rej,
					  &req_refs[count], &wait_inds[count]);

	/* Store entries into old message */
	stored = store_imm_ass_rej_refs(old_rej,
					&req_refs[stored], &wait_inds[stored],
					count);
	count -= stored;
	if (count == 0)
		return 1;

	/* Store remaining entries into new message */
	stored += store_imm_ass_rej_refs(new_rej,
					 &req_refs[stored], &wait_inds[stored],
					 count);
	return 0;
}

int bts_agch_enqueue(struct gsm_bts *bts, struct msgb *msg)
{
	struct gsm_bts_role_bts *btsb = bts_role_bts(bts);
	int hard_limit = 1000;
	struct gsm48_imm_ass_rej *imm_ass_cmd = msgb_l3(msg);

	if (btsb->agch_queue_length > hard_limit) {
		LOGP(DSUM, LOGL_ERROR,
		     "AGCH: too many messages in queue, "
		     "refusing message type 0x%02x, length = %d/%d\n",
		     ((struct gsm48_imm_ass *)msgb_l3(msg))->msg_type,
		     btsb->agch_queue_length, btsb->agch_max_queue_length);

		btsb->agch_queue_rejected_msgs++;
		return -ENOMEM;
	}

	if (btsb->agch_queue_length > 0) {
		struct msgb *last_msg =
			llist_entry(btsb->agch_queue.prev, struct msgb, list);
		struct gsm48_imm_ass_rej *last_imm_ass_rej = msgb_l3(last_msg);

		if (try_merge_imm_ass_rej(last_imm_ass_rej, imm_ass_cmd)) {
			btsb->agch_queue_merged_msgs++;
			msgb_free(msg);
			return 0;
		}
	}

	msgb_enqueue(&btsb->agch_queue, msg);
	btsb->agch_queue_length++;

	return 0;
}

struct msgb *bts_agch_dequeue(struct gsm_bts *bts)
{
	struct gsm_bts_role_bts *btsb = bts_role_bts(bts);
	struct msgb *msg = msgb_dequeue(&btsb->agch_queue);
	if (!msg)
		return NULL;

	btsb->agch_queue_length--;
	return msg;
}

/*
 * Remove lower prio messages if the queue has grown too long.
 *
 * \return 0 iff the number of messages in the queue would fit into the AGCH
 *         reserved part of the CCCH.
 */
static void compact_agch_queue(struct gsm_bts *bts)
{
	struct gsm_bts_role_bts *btsb = bts_role_bts(bts);
	struct msgb *msg, *msg2;
	int max_len, slope, offs;
	int level_low = btsb->agch_queue_low_level;
	int level_high = btsb->agch_queue_high_level;
	int level_thres = btsb->agch_queue_thresh_level;

	max_len = btsb->agch_max_queue_length;

	if (max_len == 0)
		max_len = 1;

	if (btsb->agch_queue_length < max_len * level_thres / 100)
		return;

	/* p^
	 * 1+      /'''''
	 *  |     /
	 *  |    /
	 * 0+---/--+----+--> Q length
	 *    low high max_len
	 */

	offs = max_len * level_low / 100;
	if (level_high > level_low)
		slope = 0x10000 * 100 / (level_high - level_low);
	else
		slope = 0x10000 * max_len; /* p_drop >= 1 if len > offs */

	llist_for_each_entry_safe(msg, msg2, &btsb->agch_queue, list) {
		struct gsm48_imm_ass *imm_ass_cmd = msgb_l3(msg);
		int p_drop;

		if (imm_ass_cmd->msg_type != GSM48_MT_RR_IMM_ASS_REJ)
			return;

		/* IMMEDIATE ASSIGN REJECT */

		p_drop = (btsb->agch_queue_length - offs) * slope / max_len;

		if ((random() & 0xffff) >= p_drop)
			return;

		llist_del(&msg->list);
		btsb->agch_queue_length--;
		msgb_free(msg);

		btsb->agch_queue_dropped_msgs++;
	}
	return;
}

int bts_ccch_copy_msg(struct gsm_bts *bts, uint8_t *out_buf, struct gsm_time *gt,
		      int is_ag_res)
{
	struct msgb *msg = NULL;
	struct gsm_bts_role_bts *btsb = bts->role;
	int rc = 0;
	int is_empty = 1;

	/* Do queue house keeping.
	 * This needs to be done every time a CCCH message is requested, since
	 * the queue max length is calculated based on the CCCH block rate and
	 * PCH messages also reduce the drain of the AGCH queue.
	 */
	compact_agch_queue(bts);

	/* Check for paging messages first if this is PCH */
	if (!is_ag_res)
		rc = paging_gen_msg(btsb->paging_state, out_buf, gt, &is_empty);

	/* Check whether the block may be overwritten */
	if (!is_empty)
		return rc;

	msg = bts_agch_dequeue(bts);
	if (!msg)
		return rc;

	/* Copy AGCH message */
	memcpy(out_buf, msgb_l3(msg), msgb_l3len(msg));
	rc = msgb_l3len(msg);
	msgb_free(msg);

	if (is_ag_res)
		btsb->agch_queue_agch_msgs++;
	else
		btsb->agch_queue_pch_msgs++;

	return rc;
}

int bts_supports_cipher(struct gsm_bts_role_bts *bts, int rsl_cipher)
{
	int sup;

	if (rsl_cipher < 1 || rsl_cipher > 8)
		return -ENOTSUP;

	/* No encryption is always supported */
	if (rsl_cipher == 1)
		return 1;

	sup =  (1 << (rsl_cipher - 2)) & bts->support.ciphers;
	return sup > 0;
}

struct gsm_time *get_time(struct gsm_bts *bts)
{
	struct gsm_bts_role_bts *btsb = bts->role;

	return &btsb->gsm_time;
}

