/* (C) 2011 by Andreas Eversberg <jolly@eversberg.eu>
 *
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#include <errno.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <osmocom/core/select.h>
#include <osmocom/core/timer.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/talloc.h>
#include <osmocom/gsm/protocol/gsm_12_21.h>
#include <osmo-bts/logging.h>
//#include <osmocom/bb/common/osmocom_data.h>
//#include <osmocom/bb/common/l1l2_interface.h>
#include <osmo-bts/abis.h>
#include <osmo-bts/rtp.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/rsl.h>
#include <osmo-bts/oml.h>

extern void *l23_ctx;

extern uint8_t cr_loc2rem_cmd;
extern uint8_t cr_loc2rem_resp;
extern uint8_t cr_rem2loc_cmd;
extern uint8_t cr_rem2loc_resp;

BTS_SI_NAME;

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
#endif

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

/* main link is established, send status report */
int bts_link_estab(struct osmocom_bts *bts)
{
	int i, j;
	uint8_t radio_state;

	LOGP(DSUM, LOGL_INFO, "Main link established, sending Status'.\n");

	/* site-manager status */
	oml_tx_state_changed(&bts->link, NM_OPSTATE_ENABLED, NM_AVSTATE_OK,
		NM_OC_SITE_MANAGER, 0xff, 0xff, 0xff);
	/* bts status */
	oml_tx_state_changed(&bts->link, NM_OPSTATE_ENABLED, NM_AVSTATE_OK,
		NM_OC_BTS, 0, 0xff, 0xff);
	/* trx */
	for (i = 0; i < bts->num_trx; i++) {
		radio_state = (bts->trx[i]->link.state == LINK_STATE_CONNECT) ?  NM_OPSTATE_ENABLED : NM_OPSTATE_DISABLED;
		oml_tx_state_changed(&bts->link, radio_state,
			NM_AVSTATE_OK, NM_OC_RADIO_CARRIER, 0,
			bts->trx[i]->trx_nr, 0xff);
		oml_tx_sw_act_rep(&bts->link, NM_OC_RADIO_CARRIER, 0, bts->trx[i]->trx_nr, 0xff);
		oml_tx_state_changed(&bts->link, NM_OPSTATE_ENABLED,
			NM_AVSTATE_OK, NM_OC_BASEB_TRANSC, 0,
			bts->trx[i]->trx_nr, 0xff);
		oml_tx_sw_act_rep(&bts->link, NM_OC_BASEB_TRANSC, 0, bts->trx[i]->trx_nr, 0xff);
		/* channel */
		for (j = 0; j < 8; j++) {
			if (bts->trx[i]->slot[j].tx_ms)
				oml_tx_state_changed(&bts->link,
					NM_OPSTATE_DISABLED, NM_AVSTATE_DEPENDENCY,
					NM_OC_CHANNEL, 0, bts->trx[i]->trx_nr,
					bts->trx[i]->slot[j].slot_nr);
			else
				oml_tx_state_changed(&bts->link,
					NM_OPSTATE_DISABLED,
					NM_AVSTATE_NOT_INSTALLED,
					NM_OC_CHANNEL, 0, bts->trx[i]->trx_nr,
					bts->trx[i]->slot[j].slot_nr);
		}
	}

	return 0;
}

/* RSL link is established, send status report */
int trx_link_estab(struct osmobts_trx *trx)
{
	uint8_t radio_state = (trx->link.state == LINK_STATE_CONNECT) ?  NM_OPSTATE_ENABLED : NM_OPSTATE_DISABLED;

	LOGP(DSUM, LOGL_INFO, "RSL link (TRX %02x) state changed to %s, sending Status'.\n", trx->trx_nr, (trx->link.state == LINK_STATE_CONNECT) ? "up" : "down");

	oml_tx_state_changed(&trx->bts->link, radio_state, NM_AVSTATE_OK, NM_OC_RADIO_CARRIER, 0, trx->trx_nr, 0xff);

	if (trx->link.state == LINK_STATE_CONNECT)
		rsl_tx_rf_res(trx);

	return 0;
}

void bts_new_si(void *arg)
{
	struct osmobts_trx *trx = arg;
	int i;

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
}

/* handle bts instance (including MS instances) */
int work_bts(struct osmocom_bts *bts)
{
	int work = 0, w;

	do {
		w = 0;
//		w |= xxx_dequeue(ms);
		if (w)
			work = 1;
	} while (w);
	return work;
}
#if 0
int create_chan(struct osmocom_bts *slot, uint8_t type)
{
	welcher type?:
}
#endif

