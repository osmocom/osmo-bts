/* Paging message encoding + queue management */

/* (C) 2011-2012 by Harald Welte <laforge@gnumonks.org>
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

/* TODO:
	* eMLPP priprity
	* add P2/P3 rest octets
 */

#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <string.h>
#include <time.h>

#include <osmocom/core/talloc.h>
#include <osmocom/core/linuxlist.h>

#include <osmocom/gsm/protocol/gsm_04_08.h>
#include <osmocom/gsm/gsm0502.h>
#include <osmocom/gsm/gsm48.h>

#include <osmo-bts/bts.h>
#include <osmo-bts/rsl.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/paging.h>
#include <osmo-bts/signal.h>
#include <osmo-bts/pcu_if.h>

#define MAX_PAGING_BLOCKS_CCCH	9
#define MAX_BS_PA_MFRMS		9

enum paging_record_type {
	PAGING_RECORD_PAGING,
	PAGING_RECORD_IMM_ASS
};

struct paging_record {
	struct llist_head list;
	enum paging_record_type type;
	union {
		struct {
			time_t expiration_time;
			uint8_t chan_needed;
			uint8_t identity_lv[9];
		} paging;
		struct {
			uint8_t msg[GSM_MACBLOCK_LEN];
		} imm_ass;
	} u;
};

struct paging_state {
	struct gsm_bts *bts;

	/* parameters taken / interpreted from BCCH/CCCH configuration */
	struct gsm48_control_channel_descr chan_desc;

	/* configured otherwise */
	unsigned int paging_lifetime; /* in seconds */
	unsigned int num_paging_max;

	/* total number of currently active paging records in queue */
	unsigned int num_paging;
	struct llist_head paging_queue[MAX_PAGING_BLOCKS_CCCH*MAX_BS_PA_MFRMS];

	/* prioritization of cs pagings will automatically become
	 * active on congestions (queue almost full) */
	bool cs_priority_active;
};

/* The prioritization of cs pagings is controlled by a hysteresis. When the
 * fill state of the paging queue exceeds the upper fill level
 * THRESHOLD_CONGESTED [%], then PS pagings (immediate assignments) will be
 * dropped until fill state of the paging queue drops under the lower fill
 * level THRESHOLD_CLEAR [%]. */
#define THRESHOLD_CONGESTED 66 /* (percent of num_paging_max) */
#define THRESHOLD_CLEAR 50 /* (percent of num_paging_max) */

/* Check the queue fill status and decide if prioritization of CS pagings
 * must be turned on to flatten the negative effects of the congestion
 * situation on the CS domain. */
static void check_congestion(struct paging_state *ps)
{
	int pag_queue_len = paging_queue_length(ps);
	int pag_queue_max = paging_get_queue_max(ps);
	unsigned int treshold_upper = pag_queue_max * THRESHOLD_CONGESTED / 100;
	unsigned int treshold_lower = pag_queue_max * THRESHOLD_CLEAR / 100;

	if (pag_queue_len > treshold_upper && ps->cs_priority_active == false) {
		ps->cs_priority_active = true;
		rate_ctr_inc2(ps->bts->ctrs, BTS_CTR_PAGING_CONG);
	} else if (pag_queue_len < treshold_lower)
		ps->cs_priority_active = false;
}

unsigned int paging_get_lifetime(struct paging_state *ps)
{
	return ps->paging_lifetime;
}

unsigned int paging_get_queue_max(struct paging_state *ps)
{
	return ps->num_paging_max;
}

void paging_set_lifetime(struct paging_state *ps, unsigned int lifetime)
{
	ps->paging_lifetime = lifetime;
}

void paging_set_queue_max(struct paging_state *ps, unsigned int queue_max)
{
	ps->num_paging_max = queue_max;
}

static int tmsi_mi_to_uint(uint32_t *out, const uint8_t *tmsi_lv)
{
	if (tmsi_lv[0] < 5)
		return -EINVAL;
	if ((tmsi_lv[1] & 7) != GSM_MI_TYPE_TMSI)
		return -EINVAL;

	*out = *((uint32_t *)(tmsi_lv+2));

	return 0;
}

/* paging block numbers in a simple non-combined CCCH */
static const uint8_t block_by_tdma51[51] = {
	255, 255,		/* FCCH, SCH */
	255, 255, 255, 255,	/* BCCH */
	0, 0, 0, 0,		/* B0(6..9) */
	255, 255,		/* FCCH, SCH */
	1, 1, 1, 1,		/* B1(12..15) */
	2, 2, 2, 2,		/* B2(16..19) */
	255, 255,		/* FCCH, SCH */
	3, 3, 3, 3,		/* B3(22..25) */
	4, 4, 4, 4,		/* B3(26..29) */
	255, 255,		/* FCCH, SCH */
	5, 5, 5, 5,		/* B3(32..35) */
	6, 6, 6, 6,		/* B3(36..39) */
	255, 255,		/* FCCH, SCH */
	7, 7, 7, 7,		/* B3(42..45) */
	8, 8, 8, 8,		/* B3(46..49) */
	255,			/* empty */
};

/* get the paging block number _within_ current 51 multiframe */
static int get_pag_idx_n(struct paging_state *ps, struct gsm_time *gt)
{
	int blk_n = block_by_tdma51[gt->t3];
	int blk_idx;

	if (blk_n == 255)
		return -EINVAL;

	blk_idx = blk_n - ps->chan_desc.bs_ag_blks_res;
	if (blk_idx < 0)
		return -EINVAL;

	return blk_idx;
}

/* get paging block index over multiple 51 multiframes */
static int get_pag_subch_nr(struct paging_state *ps, struct gsm_time *gt)
{
	int pag_idx = get_pag_idx_n(ps, gt);
	unsigned int n_pag_blks_51 = gsm0502_get_n_pag_blocks(&ps->chan_desc);
	unsigned int mfrm_part;

	if (pag_idx < 0)
		return pag_idx;

	mfrm_part = ((gt->fn / 51) % (ps->chan_desc.bs_pa_mfrms+2)) * n_pag_blks_51;

	return pag_idx + mfrm_part;
}

int paging_buffer_space(struct paging_state *ps)
{
	if (ps->num_paging >= ps->num_paging_max)
		return 0;
	else
		return ps->num_paging_max - ps->num_paging;
}

/* Add an identity to the paging queue */
int paging_add_identity(struct paging_state *ps, uint8_t paging_group,
			const uint8_t *identity_lv, uint8_t chan_needed)
{
	struct llist_head *group_q = &ps->paging_queue[paging_group];
	int blocks = gsm48_number_of_paging_subchannels(&ps->chan_desc);
	struct paging_record *pr;

	check_congestion(ps);

	rate_ctr_inc2(ps->bts->ctrs, BTS_CTR_PAGING_RCVD);

	if (paging_group >= blocks) {
		LOGP(DPAG, LOGL_ERROR, "BSC Send PAGING for group %u, but number of paging "
			"sub-channels is only %u\n", paging_group, blocks);
		rate_ctr_inc2(ps->bts->ctrs, BTS_CTR_PAGING_DROP);
		return -EINVAL;
	}

	if (ps->num_paging >= ps->num_paging_max) {
		LOGP(DPAG, LOGL_NOTICE, "Dropping paging, queue full (%u)\n",
			ps->num_paging);
		rate_ctr_inc2(ps->bts->ctrs, BTS_CTR_PAGING_DROP);
		return -ENOSPC;
	}

	/* Check if we already have this identity */
	llist_for_each_entry(pr, group_q, list) {
		if (pr->type != PAGING_RECORD_PAGING)
			continue;
		if (identity_lv[0] == pr->u.paging.identity_lv[0] &&
		    !memcmp(identity_lv+1, pr->u.paging.identity_lv+1,
							identity_lv[0])) {
			LOGP(DPAG, LOGL_INFO, "Ignoring duplicate paging\n");
			pr->u.paging.expiration_time =
					time(NULL) + ps->paging_lifetime;
			return -EEXIST;
		}
	}

	pr = talloc_zero(ps, struct paging_record);
	if (!pr)
		return -ENOMEM;
	pr->type = PAGING_RECORD_PAGING;

	if (*identity_lv + 1 > sizeof(pr->u.paging.identity_lv)) {
		talloc_free(pr);
		return -E2BIG;
	}

	LOGP(DPAG, LOGL_INFO, "Add paging to queue (group=%u, queue_len=%u)\n",
		paging_group, ps->num_paging+1);

	pr->u.paging.expiration_time = time(NULL) + ps->paging_lifetime;
	pr->u.paging.chan_needed = chan_needed;
	memcpy(&pr->u.paging.identity_lv, identity_lv, identity_lv[0]+1);

	/* enqueue the new identity to the HEAD of the queue,
	 * to ensure it will be paged quickly at least once.  */
	llist_add(&pr->list, group_q);
	ps->num_paging++;

	return 0;
}

/* Add an IMM.ASS message to the paging queue */
int paging_add_imm_ass(struct paging_state *ps,
		       const uint8_t *data, uint8_t len)
{
	struct llist_head *group_q;
	struct paging_record *pr;
	uint16_t imsi, paging_group;

	check_congestion(ps);

	if (ps->cs_priority_active) {
		LOGP(DPAG, LOGL_NOTICE, "Dropping paging for PS, queue congested (%u)\n",
			ps->num_paging);
		rate_ctr_inc2(ps->bts->ctrs, BTS_CTR_PAGING_DROP_PS);
		return -ENOSPC;
	}

	if (len != GSM_MACBLOCK_LEN + 3) {
		LOGP(DPAG, LOGL_ERROR, "IMM.ASS invalid length %d\n", len);
		return -EINVAL;
	}
	len -= 3;

	imsi = 100 * ((*(data++)) - '0');
	imsi += 10 * ((*(data++)) - '0');
	imsi += (*(data++)) - '0';
	paging_group = gsm0502_calc_paging_group(&ps->chan_desc, imsi);

	group_q = &ps->paging_queue[paging_group];

	pr = talloc_zero(ps, struct paging_record);
	if (!pr)
		return -ENOMEM;
	pr->type = PAGING_RECORD_IMM_ASS;

	LOGP(DPAG, LOGL_INFO, "Add IMM.ASS to queue (group=%u)\n",
		paging_group);
	memcpy(pr->u.imm_ass.msg, data, GSM_MACBLOCK_LEN);

	/* enqueue the new message to the HEAD of the queue */
	llist_add(&pr->list, group_q);

	return 0;
}

#define L2_PLEN(len)	(((len - 1) << 2) | 0x01)

/* abstract representation of P1 rest octets; we only implement those parts we need for now */
struct p1_rest_octets {
	struct {
		bool present;
		uint8_t nln;
		uint8_t nln_status;
	} nln_pch;
	bool packet_page_ind[2];
	bool r8_present;
	struct {
		bool prio_ul_access;
		bool etws_present;
		struct {
			bool is_first;
			uint8_t page_nr;
			const uint8_t *page;
			size_t page_bytes;
		} etws;
	} r8;
};

/* 3GPP TS 44.018 10.5.2.23 append a segment/page of an ETWS primary notification to given bitvec */
static void append_etws_prim_notif(struct bitvec *bv, bool is_first, uint8_t page_nr,
				   const uint8_t *etws, ssize_t etws_len)
{
	OSMO_ASSERT(etws_len < 128/8);

	/* ETWS primary Notification struct
	 * 0 NNNN / 1 NNNN
	 * PNI n
	 * LEN nnnnnnn (at least 13 bits before paylod)
	 * number of bits (LEN; up to 128) */

	if (is_first)
		bitvec_set_bit(bv, 0);
	else
		bitvec_set_bit(bv, 1);
	bitvec_set_uint(bv, page_nr, 4); /* Segment Number / Total Number */
	bitvec_set_bit(bv, 0); /* PNI to distinguish different ETWS */
	bitvec_set_uint(bv, etws_len*8, 7); /* length of payload in number of bits */
	bitvec_set_bytes(bv, etws, etws_len);

	/* 17 bytes = 136bit - (11+13) = 112 bits = 14 bytes per PT1
	 *  => at least 4x PT1 RO for complete primary notification (56 bytes) */
}

/* 3GPP TS 44.018 10.5.2.23 append P1 Rest Octets to given bit-vector */
static void append_p1_rest_octets(struct bitvec *bv, const struct p1_rest_octets *p1ro)
{
	/* Paging 1 RO (at least 10 bits before ETWS struct) */
	if (p1ro->nln_pch.present) {
		bitvec_set_bit(bv, H);
		bitvec_set_uint(bv, p1ro->nln_pch.nln, 2);
		bitvec_set_uint(bv, p1ro->nln_pch.nln_status, 1);
	} else {
		bitvec_set_bit(bv, L);		/* no NLN */
	}
	bitvec_set_bit(bv, L);		/* no Priority1 */
	bitvec_set_bit(bv, L);		/* no Priority2 */
	bitvec_set_bit(bv, L);		/* no Group Call Info */
	if (p1ro->packet_page_ind[0])
		bitvec_set_bit(bv, H);		/* Packet Page Indication 1 */
	else
		bitvec_set_bit(bv, L);		/* Packet Page Indication 1 */
	if (p1ro->packet_page_ind[1])
		bitvec_set_bit(bv, H);		/* Packet Page Indication 2 */
	else
		bitvec_set_bit(bv, L);		/* Packet Page Indication 2 */

	bitvec_set_bit(bv, L);		/* No Release 6 additions */
	bitvec_set_bit(bv, L);		/* No Release 7 additions */

	if (p1ro->r8_present) {
		bitvec_set_bit(bv, H);		/* Release 8 */
		bitvec_set_bit(bv, p1ro->r8.prio_ul_access);	/* Priority Uplink Access */
		if (p1ro->r8.etws_present) {
			bitvec_set_bit(bv, 1);		/* ETWS present */
			append_etws_prim_notif(bv, p1ro->r8.etws.is_first, p1ro->r8.etws.page_nr,
					       p1ro->r8.etws.page, p1ro->r8.etws.page_bytes);
		} else
			bitvec_set_bit(bv, 0);
	}
}

static int fill_paging_type_1(uint8_t *out_buf, const uint8_t *identity1_lv,
				uint8_t chan1, const uint8_t *identity2_lv,
				uint8_t chan2, const struct p1_rest_octets *p1ro)
{
	struct gsm48_paging1 *pt1 = (struct gsm48_paging1 *) out_buf;
	unsigned int ro_len;
	uint8_t *cur;

	*pt1 = (struct gsm48_paging1) {
		.proto_discr = GSM48_PDISC_RR,
		.msg_type = GSM48_MT_RR_PAG_REQ_1,
		.pag_mode = GSM48_PM_NORMAL,
		.cneed1 = chan1 & 3,
		.cneed2 = chan2 & 3,
	};

	cur = lv_put(pt1->data, identity1_lv[0], identity1_lv+1);
	if (identity2_lv)
		cur = tlv_put(cur, GSM48_IE_MOBILE_ID, identity2_lv[0], identity2_lv+1);

	pt1->l2_plen = L2_PLEN(cur - out_buf);

	/* Pad remaining octets with constant '2B'O */
	ro_len = GSM_MACBLOCK_LEN - (cur - out_buf);
	memset(cur, GSM_MACBLOCK_PADDING, ro_len);

	/* Optional P1 Rest Octets */
	if (p1ro) {
		struct bitvec bv = {
			.data_len = ro_len,
			.data = cur,
		};

		append_p1_rest_octets(&bv, p1ro);
	}

	return GSM_MACBLOCK_LEN;
}

static int fill_paging_type_2(uint8_t *out_buf, const uint8_t *tmsi1_lv,
				uint8_t cneed1, const uint8_t *tmsi2_lv,
				uint8_t cneed2, const uint8_t *identity3_lv)
{
	struct gsm48_paging2 *pt2 = (struct gsm48_paging2 *) out_buf;
	uint32_t tmsi;
	uint8_t *cur;
	int rc;

	memset(out_buf, 0, sizeof(*pt2));

	pt2->proto_discr = GSM48_PDISC_RR;
	pt2->msg_type = GSM48_MT_RR_PAG_REQ_2;
	pt2->pag_mode = GSM48_PM_NORMAL;
	pt2->cneed1 = cneed1;
	pt2->cneed2 = cneed2;
	rc = tmsi_mi_to_uint(&tmsi, tmsi1_lv);
	if (rc == 0)
		pt2->tmsi1 = tmsi;
	rc = tmsi_mi_to_uint(&tmsi, tmsi2_lv);
	if (rc == 0)
		pt2->tmsi2 = tmsi;
	cur = out_buf + sizeof(*pt2);

	if (identity3_lv)
		cur = tlv_put(pt2->data, GSM48_IE_MOBILE_ID, identity3_lv[0], identity3_lv+1);

	pt2->l2_plen = L2_PLEN(cur - out_buf);

	return cur - out_buf;
}

static int fill_paging_type_3(uint8_t *out_buf, const uint8_t *tmsi1_lv, uint8_t cneed1,
				const uint8_t *tmsi2_lv, uint8_t cneed2,
				const uint8_t *tmsi3_lv, uint8_t cneed3,
				const uint8_t *tmsi4_lv, uint8_t cneed4)
{
	struct gsm48_paging3 *pt3 = (struct gsm48_paging3 *) out_buf;
	uint32_t tmsi;
	int rc;

	memset(out_buf, 0, sizeof(*pt3));

	pt3->proto_discr = GSM48_PDISC_RR;
	pt3->msg_type = GSM48_MT_RR_PAG_REQ_3;
	pt3->pag_mode = GSM48_PM_NORMAL;
	pt3->cneed1 = cneed1;
	pt3->cneed2 = cneed2;
	rc = tmsi_mi_to_uint(&tmsi, tmsi1_lv);
	if (rc == 0)
		pt3->tmsi1 = tmsi;
	rc = tmsi_mi_to_uint(&tmsi, tmsi2_lv);
	if (rc == 0)
		pt3->tmsi2 = tmsi;
	rc = tmsi_mi_to_uint(&tmsi, tmsi3_lv);
	if (rc == 0)
		pt3->tmsi3 = tmsi;
	rc = tmsi_mi_to_uint(&tmsi, tmsi4_lv);
	if (rc == 0)
		pt3->tmsi4 = tmsi;

	/* The structure definition in libosmocore is wrong. It includes as last
	 * byte some invalid definition of chneed3/chneed4, so we must do this by hand
	 * here and cannot rely on sizeof(*pt3) */
	out_buf[20] = (0x23 & ~0xf8) | 0x80 | (cneed3 & 3) << 5 | (cneed4 & 3) << 3;

	return 21;
}

static const uint8_t empty_id_lv[] = { 0x01, 0xF0 };

static struct paging_record *dequeue_pr(struct llist_head *group_q)
{
	struct paging_record *pr;

	pr = llist_entry(group_q->next, struct paging_record, list);
	llist_del(&pr->list);

	return pr;
}

static int pr_is_imsi(struct paging_record *pr)
{
	if ((pr->u.paging.identity_lv[1] & 7) == GSM_MI_TYPE_IMSI)
		return 1;
	else
		return 0;
}

static void sort_pr_tmsi_imsi(struct paging_record *pr[], unsigned int n)
{
	int i, j;
	struct paging_record *t;

	if (n < 2)
		return;

	/* simple bubble sort */
	for (i = n-2; i >= 0; i--) {
		for (j=0; j<=i ; j++) {
			if (pr_is_imsi(pr[j]) > pr_is_imsi(pr[j+1])) {
				t = pr[j];
				pr[j] = pr[j+1];
				pr[j+1] = t;
			}
		}
	}
}

static void build_p1_rest_octets(struct p1_rest_octets *p1ro, struct gsm_bts *bts)
{
	memset(p1ro, 0, sizeof(*p1ro));
	p1ro->nln_pch.present = false;
	p1ro->packet_page_ind[0] = false;
	p1ro->packet_page_ind[1] = false;
	p1ro->r8_present = true;
	p1ro->r8.prio_ul_access = false;
	p1ro->r8.etws_present = true;
	unsigned int offset = bts->etws.page_size * bts->etws.next_page;

	if (bts->etws.next_page == 0) {
		p1ro->r8.etws.is_first = true;
		p1ro->r8.etws.page_nr = bts->etws.num_pages;
	} else {
		p1ro->r8.etws.is_first = false;
		p1ro->r8.etws.page_nr = bts->etws.next_page + 1;
	}
	p1ro->r8.etws.page = bts->etws.prim_notif + offset;
	/* last page may be smaller than first pages */
	if (bts->etws.next_page < bts->etws.num_pages-1)
		p1ro->r8.etws.page_bytes = bts->etws.page_size;
	else
		p1ro->r8.etws.page_bytes = bts->etws.prim_notif_len - offset;
	bts->etws.next_page = (bts->etws.next_page + 1) % bts->etws.num_pages;
}

/* generate paging message for given gsm time */
int paging_gen_msg(struct paging_state *ps, uint8_t *out_buf, struct gsm_time *gt,
		   int *is_empty)
{
	struct llist_head *group_q;
	struct gsm_bts *bts = ps->bts;
	int group;
	int len;

	/* This will have no effect on behavior of this function, we just need
	 * need to check the congestion status of the queue from time to time. */
	check_congestion(ps);

	*is_empty = 0;
	bts->load.ccch.pch_total += 1;

	group = get_pag_subch_nr(ps, gt);
	if (group < 0) {
		LOGP(DPAG, LOGL_ERROR,
		     "Paging called for GSM wrong time: FN %d/%d/%d/%d.\n",
		     gt->fn, gt->t1, gt->t2, gt->t3);
		return -1;
	}

	group_q = &ps->paging_queue[group];

	if (ps->bts->etws.prim_notif) {
		struct p1_rest_octets p1ro;
		build_p1_rest_octets(&p1ro, bts);
		len = fill_paging_type_1(out_buf, empty_id_lv, 0, NULL, 0, &p1ro);
	} else if (llist_empty(group_q)) {
		/* There is nobody to be paged, send Type1 with two empty ID */
		//DEBUGP(DPAG, "Tx PAGING TYPE 1 (empty)\n");
		len = fill_paging_type_1(out_buf, empty_id_lv, 0,
					 NULL, 0, NULL);
		*is_empty = 1;
	} else {
		struct paging_record *pr[4];
		unsigned int num_pr = 0, imm_ass = 0;
		time_t now = time(NULL);
		unsigned int i, num_imsi = 0;

		bts->load.ccch.pch_used += 1;

		/* get (if we have) up to four paging records */
		for (i = 0; i < ARRAY_SIZE(pr); i++) {
			if (llist_empty(group_q))
				break;
			pr[i] = dequeue_pr(group_q);

			/* check for IMM.ASS */
			if (pr[i]->type == PAGING_RECORD_IMM_ASS) {
				imm_ass = 1;
				break;
			}

			num_pr++;

			/* count how many IMSIs are among them */
			if (pr_is_imsi(pr[i]))
				num_imsi++;
		}

		/* if we have an IMMEDIATE ASSIGNMENT */
		if (imm_ass) {
			/* re-add paging records */
			for (i = 0; i < num_pr; i++)
				llist_add(&pr[i]->list, group_q);

			/* get message and free record */
			memcpy(out_buf, pr[num_pr]->u.imm_ass.msg,
							GSM_MACBLOCK_LEN);
			pcu_tx_pch_data_cnf(gt->fn, pr[num_pr]->u.imm_ass.msg,
							GSM_MACBLOCK_LEN);
			talloc_free(pr[num_pr]);
			return GSM_MACBLOCK_LEN;
		}

		/* make sure the TMSIs are ahead of the IMSIs in the array */
		sort_pr_tmsi_imsi(pr, num_pr);

		if (num_pr == 4 && num_imsi == 0) {
			/* No IMSI: easy case, can use TYPE 3 */
			DEBUGP(DPAG, "Tx PAGING TYPE 3 (4 TMSI)\n");
			len = fill_paging_type_3(out_buf,
						 pr[0]->u.paging.identity_lv,
						 pr[0]->u.paging.chan_needed,
						 pr[1]->u.paging.identity_lv,
						 pr[1]->u.paging.chan_needed,
						 pr[2]->u.paging.identity_lv,
						 pr[2]->u.paging.chan_needed,
						 pr[3]->u.paging.identity_lv,
						 pr[3]->u.paging.chan_needed);
		} else if (num_pr >= 3 && num_imsi <= 1) {
			/* 3 or 4, of which only up to 1 is IMSI */
			DEBUGP(DPAG, "Tx PAGING TYPE 2 (2 TMSI,1 xMSI)\n");
			len = fill_paging_type_2(out_buf,
						 pr[0]->u.paging.identity_lv,
						 pr[0]->u.paging.chan_needed,
						 pr[1]->u.paging.identity_lv,
						 pr[1]->u.paging.chan_needed,
						 pr[2]->u.paging.identity_lv);
			if (num_pr == 4) {
				/* re-add #4 for next time */
				llist_add(&pr[3]->list, group_q);
				pr[3] = NULL;
			}
		} else if (num_pr == 1) {
			DEBUGP(DPAG, "Tx PAGING TYPE 1 (1 xMSI,1 empty)\n");
			len = fill_paging_type_1(out_buf,
						 pr[0]->u.paging.identity_lv,
						 pr[0]->u.paging.chan_needed,
						 NULL, 0, NULL);
		} else {
			/* 2 (any type) or
			 * 3 or 4, of which only 2 will be sent */
			DEBUGP(DPAG, "Tx PAGING TYPE 1 (2 xMSI)\n");
			len = fill_paging_type_1(out_buf,
						 pr[0]->u.paging.identity_lv,
						 pr[0]->u.paging.chan_needed,
						 pr[1]->u.paging.identity_lv,
						 pr[1]->u.paging.chan_needed, NULL);
			if (num_pr >= 3) {
				/* re-add #4 for next time */
				llist_add(&pr[2]->list, group_q);
				pr[2] = NULL;
			}
			if (num_pr == 4) {
				/* re-add #4 for next time */
				llist_add(&pr[3]->list, group_q);
				pr[3] = NULL;
			}
		}

		for (i = 0; i < num_pr; i++) {
			/* skip those that we might have re-added above */
			if (pr[i] == NULL)
				continue;
			rate_ctr_inc2(bts->ctrs, BTS_CTR_PAGING_SENT);
			/* check if we can expire the paging record,
			 * or if we need to re-queue it */
			if (pr[i]->u.paging.expiration_time <= now) {
				talloc_free(pr[i]);
				ps->num_paging--;
				LOGP(DPAG, LOGL_INFO, "Removed paging record, queue_len=%u\n",
					ps->num_paging);
			} else
				llist_add_tail(&pr[i]->list, group_q);
		}
	}
	memset(out_buf+len, 0x2B, GSM_MACBLOCK_LEN-len);
	return len;
}

int paging_si_update(struct paging_state *ps, struct gsm48_control_channel_descr *chan_desc)
{
	LOGP(DPAG, LOGL_INFO, "Paging SI update\n");

	ps->chan_desc = *chan_desc;

	/* FIXME: do we need to re-sort the old paging_records? */

	return 0;
}

static int paging_signal_cbfn(unsigned int subsys, unsigned int signal, void *hdlr_data,
				void *signal_data)
{
	if (subsys == SS_GLOBAL && signal == S_NEW_SYSINFO) {
		struct gsm_bts *bts = signal_data;
		struct paging_state *ps = bts->paging_state;
		struct gsm48_system_information_type_3 *si3 = (void *) bts->si_buf[SYSINFO_TYPE_3];

		paging_si_update(ps, &si3->control_channel_desc);
	}
	return 0;
}

static int initialized = 0;

struct paging_state *paging_init(struct gsm_bts *bts,
				 unsigned int num_paging_max,
				 unsigned int paging_lifetime)
{
	struct paging_state *ps;
	unsigned int i;

	ps  = talloc_zero(bts, struct paging_state);
	if (!ps)
		return NULL;

	ps->bts = bts;
	ps->paging_lifetime = paging_lifetime;
	ps->num_paging_max = num_paging_max;
	ps->cs_priority_active = false;

	for (i = 0; i < ARRAY_SIZE(ps->paging_queue); i++)
		INIT_LLIST_HEAD(&ps->paging_queue[i]);

	if (!initialized) {
		osmo_signal_register_handler(SS_GLOBAL, paging_signal_cbfn, NULL);
		initialized = 1;
	}
	return ps;
}

void paging_config(struct paging_state *ps,
		  unsigned int num_paging_max,
		  unsigned int paging_lifetime)
{
	ps->num_paging_max = num_paging_max;
	ps->paging_lifetime = paging_lifetime;
}

void paging_reset(struct paging_state *ps)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ps->paging_queue); i++) {
		struct llist_head *queue = &ps->paging_queue[i];
		struct paging_record *pr, *pr2;
		llist_for_each_entry_safe(pr, pr2, queue, list) {
			llist_del(&pr->list);
			talloc_free(pr);
			ps->num_paging--;
		}
	}

	if (ps->num_paging != 0)
		LOGP(DPAG, LOGL_NOTICE, "num_paging != 0 after flushing all records?!?\n");

	ps->num_paging = 0;
}

/**
 * \brief Helper for the unit tests
 */
int paging_group_queue_empty(struct paging_state *ps, uint8_t grp)
{
	if (grp >= ARRAY_SIZE(ps->paging_queue))
		return 1;
	return llist_empty(&ps->paging_queue[grp]);
}

int paging_queue_length(struct paging_state *ps)
{
	return ps->num_paging;
}
