/* (C) 2012 by Holger Hans Peter Freyther
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

#include <osmo-bts/bts.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/paging.h>
#include <osmo-bts/gsm_data.h>

#include <osmocom/core/talloc.h>

#include <errno.h>
#include <unistd.h>

static struct gsm_bts *bts;
static struct gsm_bts_role_bts *btsb;
int pcu_direct = 0;

#define ASSERT_TRUE(rc) \
	if (!(rc)) { \
		printf("Assert failed in %s:%d.\n",  \
		       __FILE__, __LINE__);          \
		abort();			     \
	}

static void test_cipher_parsing(void)
{
	int i;

	btsb->support.ciphers = 0;

	/* always support A5/0 */
	ASSERT_TRUE(bts_supports_cipher(btsb, 0x0) == -ENOTSUP);
	ASSERT_TRUE(bts_supports_cipher(btsb, 0x1) == 1); /* A5/0 */
	for (i = 2; i <= 8; ++i) {
		ASSERT_TRUE(bts_supports_cipher(btsb, i) == 0);
	}

	/* checking default A5/1 to A5/3 support */
	btsb->support.ciphers = CIPHER_A5(1) | CIPHER_A5(2) | CIPHER_A5(3);
	ASSERT_TRUE(bts_supports_cipher(btsb, 0x0) == -ENOTSUP);
	ASSERT_TRUE(bts_supports_cipher(btsb, 0x1) == 1); /* A5/0 */
	ASSERT_TRUE(bts_supports_cipher(btsb, 0x2) == 1); /* A5/1 */
	ASSERT_TRUE(bts_supports_cipher(btsb, 0x3) == 1); /* A5/2 */
	ASSERT_TRUE(bts_supports_cipher(btsb, 0x4) == 1); /* A5/3 */
	ASSERT_TRUE(bts_supports_cipher(btsb, 0x5) == 0); /* A5/4 */
	ASSERT_TRUE(bts_supports_cipher(btsb, 0x6) == 0); /* A5/5 */
	ASSERT_TRUE(bts_supports_cipher(btsb, 0x7) == 0); /* A5/6 */
	ASSERT_TRUE(bts_supports_cipher(btsb, 0x8) == 0); /* A5/7 */
	ASSERT_TRUE(bts_supports_cipher(btsb, 0x9) == -ENOTSUP);
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
	test_cipher_parsing();
	printf("Success\n");

	return 0;
}

/* stub to link */
const uint8_t abis_mac[6] = { 0,1,2,3,4,5 };
const char *software_version = "0815";

int bts_model_chg_adm_state(struct gsm_bts *bts, struct gsm_abis_mo *mo,
			    void *obj, uint8_t adm_state)
{ return 0; }
int bts_model_init(struct gsm_bts *bts)
{ return 0; }
int bts_model_apply_oml(struct gsm_bts *bts, struct msgb *msg,
			struct tlv_parsed *new_attr, void *obj)
{ return 0; }
int bts_model_rsl_chan_rel(struct gsm_lchan *lchan)
{ return 0;}

int bts_model_rsl_deact_sacch(struct gsm_lchan *lchan)
{ return 0; }

int bts_model_trx_deact_rf(struct gsm_bts_trx *trx)
{ return 0; }
int bts_model_trx_close(struct gsm_bts_trx *trx)
{ return 0; }
int bts_model_check_oml(struct gsm_bts *bts, uint8_t msg_type,
			struct tlv_parsed *old_attr, struct tlv_parsed *new_attr,
			void *obj)
{ return 0; }
int bts_model_opstart(struct gsm_bts *bts, struct gsm_abis_mo *mo,
		      void *obj)
{ return 0; }
int bts_model_rsl_chan_act(struct gsm_lchan *lchan, struct tlv_parsed *tp)
{ return 0; }
int bts_model_rsl_mode_modify(struct gsm_lchan *lchan)
{ return 0; }
void bts_model_rtp_rx_cb(struct osmo_rtp_socket *rs, const uint8_t *rtp_pl,
			 unsigned int rtp_pl_len) {}

int l1if_pdch_req(struct gsm_bts_trx_ts *ts, int is_ptcch, uint32_t fn,
        uint16_t arfcn, uint8_t block_nr, uint8_t *data, uint8_t len)
{ return 0; }

uint32_t trx_get_hlayer1(struct gsm_bts_trx *trx)
{ return 0; }
