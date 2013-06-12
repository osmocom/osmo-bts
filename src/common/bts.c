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


struct gsm_network bts_gsmnet = {
	.bts_list = { &bts_gsmnet.bts_list, &bts_gsmnet.bts_list },
	.num_bts = 0,
};

void *tall_bts_ctx;

int bts_init(struct gsm_bts *bts)
{
	struct gsm_bts_role_bts *btsb;
	struct gsm_bts_trx *trx;
	int rc;

	/* add to list of BTSs */
	llist_add_tail(&bts->list, &bts_gsmnet.bts_list);

	bts->band = GSM_BAND_1800;

	bts->role = btsb = talloc_zero(bts, struct gsm_bts_role_bts);

	INIT_LLIST_HEAD(&btsb->agch_queue);

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

int bts_agch_enqueue(struct gsm_bts *bts, struct msgb *msg)
{
	struct gsm_bts_role_bts *btsb = bts_role_bts(bts);

	/* FIXME: implement max queue length */
	msgb_enqueue(&btsb->agch_queue, msg);

	return 0;
}

struct msgb *bts_agch_dequeue(struct gsm_bts *bts)
{
	struct gsm_bts_role_bts *btsb = bts_role_bts(bts);

	return msgb_dequeue(&btsb->agch_queue);
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
