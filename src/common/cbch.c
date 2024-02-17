/* Cell Broadcast routines */

/* (C) 2014,2018-2019 by Harald Welte <laforge@gnumonks.org>
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
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <errno.h>

#include <osmocom/core/utils.h>
#include <osmocom/core/linuxlist.h>
#include <osmocom/gsm/protocol/gsm_04_12.h>

#include <osmo-bts/bts.h>
#include <osmo-bts/cbch.h>
#include <osmo-bts/rsl.h>
#include <osmo-bts/logging.h>

/* internal representation of one SMS-CB message (e.g. in the pending queue */
struct smscb_msg {
	struct llist_head list;		/* list in smscb_state.queue */

	bool is_schedule;		/* is this a schedule message? */
	uint8_t msg[GSM412_MSG_LEN];	/* message buffer */
	uint8_t num_segs;		/* total number of segments */
};

/* determine if current queue length differs more than permitted hysteresis from target
 * queue length. If it does, send CBCH LOAD IND */
static void check_and_send_cbch_load(struct gsm_bts *bts, struct bts_smscb_state *bts_ss)
{
	int delta = bts_ss->queue_len - bts->smscb_queue_tgt_len;
	bool extended_cbch = false;

	if (bts_ss == &bts->smscb_extended)
		extended_cbch = true;

	if (abs(delta) < bts->smscb_queue_hyst)
		return;

	if (delta < 0) {
		/* Underrun */
		rsl_tx_cbch_load_indication(bts, extended_cbch, false, OSMO_MIN(15, -delta));
	} else {
		/* Overrun */
		rsl_tx_cbch_load_indication(bts, extended_cbch, true, OSMO_MIN(15, delta));
	}
}

/* determine SMSCB state by tb number */
static struct bts_smscb_state *bts_smscb_state(struct gsm_bts *bts, uint8_t tb)
{
	if (tb < 4)
		return &bts->smscb_basic;
	else if (tb < 8)
		return &bts->smscb_extended;
	else
		OSMO_ASSERT(0);
}

static const char *tb_to_chan_str(uint8_t tb)
{
	if (tb < 4)
		return "CBCH-BASIC";
	else
		return "CBCH-EXTENDED";
}

/* construct a SMSCB NULL block in the user-provided output buffer at 'out' */
static int get_smscb_null_block(uint8_t *out)
{
	struct gsm412_block_type *block_type = (struct gsm412_block_type *) out;

	block_type->spare = 0;
	block_type->lpd = 1;
	block_type->seq_nr = GSM412_SEQ_NULL_MSG;
	block_type->lb = 0;
	memset(out+1, GSM_MACBLOCK_PADDING, GSM412_BLOCK_LEN);

	return 0;
}

/* get the next block of the current CB message */
static int get_smscb_block(struct bts_smscb_state *bts_ss, uint8_t *out, uint8_t tb,
			   const struct gsm_time *g_time)
{
	int to_copy;
	struct gsm412_block_type *block_type;
	struct smscb_msg *msg = bts_ss->cur_msg;
	uint8_t block_nr = tb % 4;
	const char *chan_name = tb_to_chan_str(tb);

	if (!msg) {
		/* No message: Send NULL block */
		DEBUGPGT(DLSMS, g_time, "%s: No cur_msg; requesting NULL block\n", chan_name);
		return get_smscb_null_block(out);
	}
	OSMO_ASSERT(block_nr < 4);

	if (block_nr >= msg->num_segs) {
		/* Higher block number than this message has blocks: Send NULL block */
		DEBUGPGT(DLSMS, g_time, "%s: cur_msg has only %u blocks; requesting NULL block\n",
			 chan_name, msg->num_segs);
		return get_smscb_null_block(out);
	}

	block_type = (struct gsm412_block_type *) out++;

	/* LPD is always 01 */
	block_type->spare = 0;
	block_type->lpd = 1;

	/* determine how much data to copy */
	to_copy = GSM412_MSG_LEN - (block_nr * GSM412_BLOCK_LEN);
	if (to_copy > GSM412_BLOCK_LEN)
	to_copy = GSM412_BLOCK_LEN;
	OSMO_ASSERT(to_copy >= 0);

	/* copy data and increment index */
	memcpy(out, &msg->msg[block_nr * GSM412_BLOCK_LEN], to_copy);

	/* set + increment sequence number */
	if (block_nr == 0 && msg->is_schedule)
		block_type->seq_nr = 8;	/* first schedule block */
	else
		block_type->seq_nr = block_nr;

	/* determine if this is the last block */
	block_type->lb = (block_nr + 1 == msg->num_segs);
	if (block_type->lb) {
		if (msg != bts_ss->default_msg) {
			DEBUGPGT(DLSMS, g_time, "%s: deleting fully-transmitted message %p\n",
				 chan_name, msg);
			/* delete any fully-transmitted normal message (or superseded default) */
			talloc_free(bts_ss->cur_msg);
			bts_ss->cur_msg = NULL;
		} else {
			DEBUGPGT(DLSMS, g_time, "%s: keeping fully-transmitted default message %p\n",
				 chan_name, msg);
		}
	}

	return block_type->lb;
}

static const uint8_t last_block_rsl2um[4] = {
	[RSL_CB_CMD_LASTBLOCK_4]	= 4,
	[RSL_CB_CMD_LASTBLOCK_1]	= 1,
	[RSL_CB_CMD_LASTBLOCK_2]	= 2,
	[RSL_CB_CMD_LASTBLOCK_3]	= 3,
};

static const struct value_string rsl_cb_cmd_names[] = {
	{ RSL_CB_CMD_TYPE_NORMAL,	"NORMAL" },
	{ RSL_CB_CMD_TYPE_SCHEDULE,	"SCHEDULE" },
	{ RSL_CB_CMD_TYPE_DEFAULT,	"DEFAULT" },
	{ RSL_CB_CMD_TYPE_NULL,		"NULL" },
	{ 0, NULL }
};


/* incoming SMS broadcast command from RSL */
int bts_process_smscb_cmd(struct gsm_bts *bts, struct rsl_ie_cb_cmd_type cmd_type,
			  bool extended_cbch, uint8_t msg_len, const uint8_t *msg)
{
	struct smscb_msg *scm;
	struct bts_smscb_state *bts_ss;
	const char *chan_name;

	if (extended_cbch) {
		chan_name = tb_to_chan_str(4);
		bts_ss = &bts->smscb_extended;
	} else {
		chan_name = tb_to_chan_str(0);
		bts_ss = &bts->smscb_basic;
	}

	if (msg_len > sizeof(scm->msg)) {
		LOGP(DLSMS, LOGL_ERROR, "%s: Cannot process SMSCB of %u bytes (max %zu)\n",
		     chan_name, msg_len, sizeof(scm->msg));
		return -EINVAL;
	}

	scm = talloc_zero(bts, struct smscb_msg);
	if (!scm)
		return -1;

	/* initialize entire message with default padding */
	memset(scm->msg, GSM_MACBLOCK_PADDING, sizeof(scm->msg));

	if (cmd_type.command == RSL_CB_CMD_TYPE_SCHEDULE)
		scm->is_schedule = true;

	scm->num_segs = last_block_rsl2um[cmd_type.last_block&3];
	memcpy(scm->msg, msg, msg_len);

	LOGP(DLSMS, LOGL_INFO, "RSL SMSCB COMMAND (chan=%s, type=%s, num_blocks=%u)\n",
		chan_name, get_value_string(rsl_cb_cmd_names, cmd_type.command), scm->num_segs);

	switch (cmd_type.command) {
	case RSL_CB_CMD_TYPE_NORMAL:
	case RSL_CB_CMD_TYPE_SCHEDULE:
	case RSL_CB_CMD_TYPE_NULL:
		/* def_bcast is ignored as per Section 9.3.41 of 3GPP TS 48.058 */
		/* limit queue size and optionally send CBCH LOAD Information (overflow) via RSL */
		if (bts_ss->queue_len >= bts->smscb_queue_max_len) {
			LOGP(DLSMS, LOGL_NOTICE, "RSL SMSCB COMMAND (chan=%s, type=%s): OVERFLOW\n",
			     chan_name, get_value_string(rsl_cb_cmd_names, cmd_type.command));
			rate_ctr_inc2(bts_ss->ctrs, CBCH_CTR_RCVD_DROPPED);
			talloc_free(scm);
			break;
		}
		llist_add_tail(&scm->list, &bts_ss->queue);
		bts_ss->queue_len++;
		check_and_send_cbch_load(bts, bts_ss);
		rate_ctr_inc2(bts_ss->ctrs, CBCH_CTR_RCVD_QUEUED);
		break;
	case RSL_CB_CMD_TYPE_DEFAULT:
		/* clear the cur_msg pointer if it is the old default message */
		if (bts_ss->cur_msg && bts_ss->cur_msg == bts_ss->default_msg)
			bts_ss->cur_msg = NULL;
		talloc_free(bts_ss->default_msg);
		if (cmd_type.def_bcast == RSL_CB_CMD_DEFBCAST_NORMAL)
			/* def_bcast == 0: normal message */
			bts_ss->default_msg = scm;
		else {
			/* def_bcast == 1: NULL message */
			bts_ss->default_msg = NULL;
			talloc_free(scm);
		}
		break;
	default:
		talloc_free(scm);
		return -EINVAL;
	}

	return 0;
}

static struct smscb_msg *select_next_smscb(struct gsm_bts *bts, uint8_t tb)
{
	struct bts_smscb_state *bts_ss = bts_smscb_state(bts, tb);
	const char *chan_name = tb_to_chan_str(tb);
	struct smscb_msg *msg;

	msg = llist_first_entry_or_null(&bts_ss->queue, struct smscb_msg, list);
	if (msg) {
		llist_del(&msg->list);
		bts_ss->queue_len--;
		check_and_send_cbch_load(bts, bts_ss);
		DEBUGP(DLSMS, "%s: %s: Dequeued msg\n", __func__, chan_name);
		rate_ctr_inc2(bts_ss->ctrs, CBCH_CTR_SENT_SINGLE);
		return msg;
	}

	/* send CBCH LOAD Information (underflow) via RSL */
	check_and_send_cbch_load(bts, bts_ss);

	/* choose the default message, if any */
	msg = bts_ss->default_msg;
	if (msg) {
		DEBUGP(DLSMS, "%s: %s: Using default msg\n", __func__, chan_name);
		rate_ctr_inc2(bts_ss->ctrs, CBCH_CTR_SENT_DEFAULT);
		return msg;
	}

	DEBUGP(DLSMS, "%s: %s: No queued msg nor default\n", __func__, chan_name);
	rate_ctr_inc2(bts_ss->ctrs, CBCH_CTR_SENT_NULL);
	return NULL;
}

/* call-back from bts model specific code when it wants to obtain a CBCH
 * block for a given gsm_time.  outbuf must have 23 bytes of space. */
int bts_cbch_get(struct gsm_bts *bts, uint8_t *outbuf, struct gsm_time *g_time)
{
	uint32_t fn = gsm_gsmtime2fn(g_time);
	struct bts_smscb_state *bts_ss;
	/* According to 05.02 Section 6.5.4 */
	uint32_t tb = (fn / 51) % 8;
	int rc = 0;

	bts_ss = bts_smscb_state(bts, tb);

	/* The multiframes used for the basic cell broadcast channel
	 * shall be those in * which TB = 0,1,2 and 3. The multiframes
	 * used for the extended cell broadcast channel shall be those
	 * in which TB = 4, 5, 6 and 7 */

	/* The SMSCB header shall be sent in the multiframe in which TB
	 * = 0 for the basic, and TB = 4 for the extended cell
	 * broadcast channel. */

	switch (tb) {
	case 0:
	case 4:
		/* select a new SMSCB message */
		bts_ss->cur_msg = select_next_smscb(bts, tb);
		rc = get_smscb_block(bts_ss, outbuf, tb, g_time);
		break;
	case 1: case 2: case 3:
	case 5: case 6: case 7:
		rc = get_smscb_block(bts_ss, outbuf, tb, g_time);
		break;
	}

	return rc;
}

static void bts_smscb_state_reset(struct bts_smscb_state *bts_ss)
{
	struct smscb_msg *scm, *tmp;
	llist_for_each_entry_safe(scm, tmp, &bts_ss->queue, list) {
		llist_del(&scm->list);
		talloc_free(scm);
	}
	bts_ss->queue_len = 0;
	rate_ctr_group_reset(bts_ss->ctrs);
	/* avoid double-free of default_msg in case cur_msg == default_msg */
	if (bts_ss->cur_msg && bts_ss->cur_msg != bts_ss->default_msg)
		talloc_free(bts_ss->cur_msg);
	bts_ss->cur_msg = NULL;
	TALLOC_FREE(bts_ss->default_msg);
}

void bts_cbch_reset(struct gsm_bts *bts)
{
	bts_smscb_state_reset(&bts->smscb_basic);
	bts_smscb_state_reset(&bts->smscb_extended);
}
