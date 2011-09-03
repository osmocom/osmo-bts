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


void *tall_bts_ctx;

int bts_init(struct gsm_bts *bts)
{
	struct gsm_bts_role_bts *btsb;
	struct gsm_bts_trx *trx;

	bts->role = btsb = talloc_zero(bts, struct gsm_bts_role_bts);

	INIT_LLIST_HEAD(&btsb->agch_queue);

	/* FIXME: make those parameters configurable */
	btsb->paging_state = paging_init(btsb, 200, 0);

	/* set BTS to dependency */
	oml_mo_state_chg(&bts->mo, -1, NM_AVSTATE_DEPENDENCY);

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

	return bts_model_init(bts);
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

	LOGP(DOML, LOGL_NOTICE, "Shutting down BTS %u, Reason %s\n",
		bts->nr, reason);

	llist_for_each_entry(trx, &bts->trx_list, list)
		bts_model_trx_deact_rf(trx);

	/* shedule a timer to make sure select loop logic can run again
	 * to dispatch any pending primitives */
	osmo_timer_schedule(&shutdown_timer, 3, 0);
}

#if 0
struct osmobts_lchan *lchan_by_channelnr(struct osmobts_trx *trx,
	uint8_t channelnr)
{
	uint8_t ts = channelnr & ~RSL_CHAN_NR_MASK;
	uint8_t type = channelnr & RSL_CHAN_NR_MASK;
	uint8_t sub = 0;
	struct osmobts_slot *slot = trx->slot[ts];

	if ((type & 0xf0) == RSL_CHAN_Lm_ACCHs) {
		sub = (channelnr >> 3) & 1;
		type = RSL_CHAN_Lm_ACCHs;
	} else
	if ((type & 0xe0) == RSL_CHAN_SDCCH4_ACCH) {
		sub = (channelnr >> 3) & 3;
		type = RSL_CHAN_SDCCH4_ACCH;
	} else
	if ((type & 0xc0) == RSL_CHAN_SDCCH8_ACCH) {
		sub = (channelnr >> 3) & 7;
		type = RSL_CHAN_SDCCH8_ACCH;
	} else
	if (type == RSL_CHAN_BCCH
	 || type == RSL_CHAN_RACH
	 || type == RSL_CHAN_PCH_AGCH) {
	 	if (!slot->has_bcch) {
	 		LOGP(DSUM, LOGL_NOTICE, "Slot %d has no BCCH\n", ts);
	 		return NULL;
		}
		return slot->lchan[0];
	} else {
	 	LOGP(DSUM, LOGL_NOTICE, "Unknown channel type 0x%02x\n", type);
	 	return NULL;
	}

	if (slot->acch_type == type)
			return slot->lchan[sub];
	}
 	LOGP(DSUM, LOGL_NOTICE, "No slot found with channel type 0x%02x\n", type);
	return NULL;

}

struct osmocom_bts *create_bts(uint8_t num_trx, char *id)
{
	struct osmocom_bts *bts;
	struct osmobts_trx *trx;
	int i, j;

	LOGP(DSUM, LOGL_INFO, "Creating BTS\n");
	bts = talloc_zero(l23_ctx, struct osmocom_bts);
	if (!bts)
		return NULL;
	bts->link.bts = bts;
	bts->id = id;
	for (i = 0; i < num_trx; i++) {
		LOGP(DSUM, LOGL_INFO, "Creating TRX %d\n", i);
		trx = talloc_zero(l23_ctx, struct osmobts_trx);
		if (!trx)
			return NULL;
		trx->bts = bts;
		trx->trx_nr = i;
		INIT_LLIST_HEAD(&trx->ms_list);
		for (j = 0; j < 8; j++) {
			trx->slot[j].trx = trx;
			trx->slot[j].slot_nr = j;
			trx->slot[j].chan_comb = 0xff; /* not set */
		}
		trx->link.trx = trx;
		bts->trx[i] = trx;
	}
	bts->num_trx = num_trx;

	return bts;
}

int create_ms(struct osmobts_trx *trx, int maskc, uint8_t *maskv_tx,
	uint8_t *maskv_rx)
{
	struct osmobts_ms *tx_ms, *rx_ms;
	int i, j;
	int rc;
	static char sock_path[] = "/tmp/osmocom_l2.1";

	for (i = 0; i < maskc; i++) {
		/* create MS */
		tx_ms = talloc_zero(l23_ctx, struct osmobts_ms);
		printf("%p\n", tx_ms);
		if (!tx_ms)
			return -ENOMEM;
		tx_ms->trx = trx;
		rc = layer2_open(&tx_ms->ms, sock_path);
		strcpy(tx_ms->ms.name, strchr(sock_path, '.') + 1);
		sock_path[strlen(sock_path) - 1]++;
		if (rc < 0) {
			talloc_free(tx_ms);
			return rc;
		}
		llist_add_tail(&tx_ms->entry, &trx->ms_list);
		rx_ms = talloc_zero(l23_ctx, struct osmobts_ms);
		rx_ms->trx = trx;
		if (!rx_ms)
			return -ENOMEM;
		rc = layer2_open(&rx_ms->ms, sock_path);
		strcpy(rx_ms->ms.name, strchr(sock_path, '.') + 1);
		sock_path[strlen(sock_path) - 1]++;
		if (rc < 0) {
			talloc_free(rx_ms);
			return rc;
		}
		llist_add_tail(&rx_ms->entry, &trx->ms_list);
		/* assign to SLOT */
		for (j = 0; j < 8; j++) {
			if ((maskv_tx[i] & (1 << j)))
				trx->slot[j].tx_ms = tx_ms;
			if ((maskv_rx[i] & (1 << j)))
				trx->slot[j].rx_ms = rx_ms;
		}
	}

	return i;
}

void destroy_lchan(struct osmobts_lchan *lchan)
{
	LOGP(DSUM, LOGL_INFO, "Destroying logical channel. (trx=%d ts=%d ss=%d)\n", lchan->slot->trx->trx_nr, lchan->slot->slot_nr, lchan->lchan_nr);
	lapdm_exit(&lchan->l2_entity.lapdm_acch);
	lapdm_exit(&lchan->l2_entity.lapdm_acch);
	if (lchan->rtp.socket_created)
		rtp_close_socket(&lchan->rtp);
	talloc_free(lchan);
}

/* create and destroy lchan */
void bts_setup_slot(struct osmobts_slot *slot, uint8_t comb)
{
	int i, ii;
	struct osmobts_lchan *lchan;
	uint8_t cbits;

	if (slot->chan_comb == comb)
		return;

	/* destroy old */
	for (i = 0; i < 8; i++) {
		if (slot->lchan[i])
			destroy_lchan(slot->lchan[i]);
		slot->lchan[i] = NULL;
	}
	switch(comb) {
	case 0xff:
		return;
	case NM_CHANC_TCHFull:
		cbits = 0x01;
		ii = 1;
		break;
	case NM_CHANC_TCHHalf:
		cbits = 0x02;
		ii = 2;
		break;
	case NM_CHANC_BCCHComb:
		cbits = 0x04;
		ii = 4;
		break;
	case NM_CHANC_SDCCH:
		cbits = 0x08;
		ii = 8;
		break;
	default:
		cbits = 0x10;
		ii = 0;
	}
	
	if (!slot->tx_ms) {
		LOGP(DSUM, LOGL_ERROR, "Slot is not available\n");
		return;
	}

	for (i = 0; i < ii; i++) {
		LOGP(DSUM, LOGL_INFO, "Creating logical channel. (trx=%d ts=%d ss=%d)\n", slot->trx->trx_nr, slot->slot_nr, i);
		lchan = talloc_zero(l23_ctx, struct osmobts_lchan);
		lchan->slot = slot;
		lchan->lchan_nr = i;
		lchan->chan_nr = ((cbits | i) << 3) | slot->slot_nr;
		lapdm_init(&lchan->l2_entity.lapdm_dcch, &lchan->l2_entity, &slot->tx_ms->ms);
		lapdm_init(&lchan->l2_entity.lapdm_acch, &lchan->l2_entity, &slot->tx_ms->ms);
		lapdm_set_bts();
		osmol2_register_handler(&lchan->l2_entity, &rsl_tx_rll);
		slot->lchan[i] = lchan;
	}

}

static void destroy_ms(struct osmobts_ms *ms)
{
	layer2_close(&ms->ms);
	llist_del(&ms->entry);
	talloc_free(ms);
}

void destroy_bts(struct osmocom_bts *bts)
{
	int i;
	struct osmobts_ms *ms;

	for (i = 0; i < bts->num_trx; i++) {
		abis_close(&bts->trx[i]->link);
		while(!llist_empty(&bts->trx[i]->ms_list)) {
			ms = llist_entry(bts->trx[i]->ms_list.next,
				struct osmobts_ms, entry);
			destroy_ms(ms);
		}
		if (osmo_timer_pending(&bts->trx[i]->si.timer))
			osmo_timer_del(&bts->trx[i]->si.timer);
		talloc_free(bts->trx[i]);
		bts->trx[i] = NULL;
	}
	abis_close(&bts->link);
	talloc_free(bts);
}
#endif

/* main link is established, send status report */
int bts_link_estab(struct gsm_bts *bts)
{
	int i, j;

	LOGP(DSUM, LOGL_INFO, "Main link established, sending Status'.\n");

	/* BTS and SITE MGR are EAABLED, BTS is DEPENDENCY */
	oml_tx_state_changed(&bts->site_mgr.mo);
	oml_tx_state_changed(&bts->mo);

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

	return 0;
}

/* RSL link is established, send status report */
int trx_link_estab(struct gsm_bts_trx *trx)
{
	struct ipabis_link *link = (struct ipabis_link *) trx->rsl_link;
	uint8_t radio_state = (link->state == LINK_STATE_CONNECT) ?  NM_OPSTATE_ENABLED : NM_OPSTATE_DISABLED;

	LOGP(DSUM, LOGL_INFO, "RSL link (TRX %02x) state changed to %s, sending Status'.\n",
		trx->nr, (link->state == LINK_STATE_CONNECT) ? "up" : "down");

	oml_mo_state_chg(&trx->mo, radio_state, NM_AVSTATE_OK);

	if (link->state == LINK_STATE_CONNECT)
		rsl_tx_rf_res(trx);

	return 0;
}

void bts_new_si(void *arg)
{
	struct osmobts_trx *trx = arg;

#if 0
	if (osmo_timer_pending(&trx->si.timer))
		return;

	i = 0;
	while(i < BTS_SI_NUM) {
		if ((trx->si.flags[i] & BTS_SI_NEW))
			break;
		i++;
	}
	if (i == BTS_SI_NUM)
		return;
	if ((trx->si.flags[i] & BTS_SI_USE))
		LOGP(DSUM, LOGL_INFO, "Setting SYSTEM INFORMATION %s.\n", bts_si_name[i]);
	else
		LOGP(DSUM, LOGL_INFO, "Removing SYSTEM INFORMATION %s.\n", bts_si_name[i]);
	trx->si.flags[i] &= ~BTS_SI_NEW;
	/* distribute */
	printf("TODO: send SI update to L1\n");
	/* delay until next SI */
	trx->si.timer.cb = bts_new_si;
	trx->si.timer.data = trx;
	osmo_timer_schedule(&trx->si.timer, 0, 200000);
#endif
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
	struct gsm_bts_role_bts *btsb = bts_role_bts(bts);;

	/* FIXME: implement max queue length */
	llist_add_tail(&msg->list, &btsb->agch_queue);

	return 0;
}

struct msgb *bts_agch_dequeue(struct gsm_bts *bts)
{
	struct gsm_bts_role_bts *btsb = bts_role_bts(bts);;
	struct msgb *msg;

	if (llist_empty(&btsb->agch_queue))
		return NULL;

	msg = llist_entry(btsb->agch_queue.next, struct msgb, list);
	llist_del(&msg->list);

	return msg;
}
