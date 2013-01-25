/* testing the paging code */

/* (C) 2011 by Holger Hans Peter Freyther
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
#include <osmocom/core/talloc.h>

#include <osmo-bts/bts.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/paging.h>
#include <osmo-bts/gsm_data.h>

#include <unistd.h>

static struct gsm_bts *bts;
static struct gsm_bts_role_bts *btsb;
int pcu_direct = 0;

static const uint8_t static_ilv[] = {
	0x08, 0x59, 0x51, 0x30, 0x99, 0x00, 0x00, 0x00, 0x19
};

#define ASSERT_TRUE(rc) \
	if (!(rc)) { \
		printf("Assert failed in %s:%d.\n",  \
		       __FILE__, __LINE__);          \
		abort();			     \
	}

static void test_paging_smoke(void)
{
	int rc;
	uint8_t out_buf[GSM_MACBLOCK_LEN];
	struct gsm_time g_time;
	printf("Testing that paging messages expire.\n");

	/* add paging entry */
	rc = paging_add_identity(btsb->paging_state, 0, static_ilv, 0);
	ASSERT_TRUE(rc == 0);
	ASSERT_TRUE(paging_queue_length(btsb->paging_state) == 1);

	/* generate messages */
	g_time.fn = 0;
	g_time.t1 = 0;
	g_time.t2 = 0;
	g_time.t3 = 6;
	rc = paging_gen_msg(btsb->paging_state, out_buf, &g_time);
	ASSERT_TRUE(rc == 13);

	ASSERT_TRUE(paging_group_queue_empty(btsb->paging_state, 0));
	ASSERT_TRUE(paging_queue_length(btsb->paging_state) == 0);

	/*
	 * TODO: test all the cases of different amount tmsi/imsi and check
	 * if we fill the slots in a optimal way.
	 */
}

static void test_paging_sleep(void)
{
	int rc;
	uint8_t out_buf[GSM_MACBLOCK_LEN];
	struct gsm_time g_time;
	printf("Testing that paging messages expire with sleep.\n");

	/* add paging entry */
	rc = paging_add_identity(btsb->paging_state, 0, static_ilv, 0);
	ASSERT_TRUE(rc == 0);
	ASSERT_TRUE(paging_queue_length(btsb->paging_state) == 1);

	/* sleep */
	sleep(1);

	/* generate messages */
	g_time.fn = 0;
	g_time.t1 = 0;
	g_time.t2 = 0;
	g_time.t3 = 6;
	rc = paging_gen_msg(btsb->paging_state, out_buf, &g_time);
	ASSERT_TRUE(rc == 13);

	ASSERT_TRUE(paging_group_queue_empty(btsb->paging_state, 0));
	ASSERT_TRUE(paging_queue_length(btsb->paging_state) == 0);
}

int main(int argc, char **argv)
{
	void *tall_msgb_ctx;

	tall_bts_ctx = talloc_named_const(NULL, 1, "OsmoBTS context");
	tall_msgb_ctx = talloc_named_const(tall_bts_ctx, 1, "msgb");
	msgb_set_talloc_ctx(tall_msgb_ctx);

	bts_log_init(NULL);

	bts = gsm_bts_alloc(tall_bts_ctx);
	if (bts_init(bts) < 0) {
		fprintf(stderr, "unable to to open bts\n");
		exit(1);
	}

	btsb = bts_role_bts(bts);
	test_paging_smoke();
	test_paging_sleep();
	printf("Success\n");

	return 0;
}

