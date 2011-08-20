/* Interface to layer 1 of baseband */

/* (C) 2011 by Andreas Eversberg <jolly@eversberg.eu>
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

#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>

#include <arpa/inet.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/un.h>

#include <osmocom/core/talloc.h>
#include <osmocom/core/utils.h>
#include <osmocom/core/select.h>
#include <osmocom/core/timer.h>
#include <osmocom/core/write_queue.h>
#include <osmocom/gsm/gsm_utils.h>
#include <osmocom/gsm/lapdm.h>

#include <osmo-bts/logging.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/oml.h>
#include <osmo-bts/gsm_data.h>
#include <osmo-bts/signal.h>

#include "l1ctl.h"
#include "l1_if.h"
#include "oml.h"

extern int tx_only;

static struct msgb *osmo_l1if_alloc(uint8_t msg_type)
{
	struct l1if_hdr *l1h;
	struct msgb *msg = msgb_alloc_headroom(256, 16, "osmo_l1if");

	if (!msg) {
		LOGP(DL1C, LOGL_ERROR, "Failed to allocate memory.\n");
		return NULL;
	}

	msg->l1h = msgb_put(msg, sizeof(*l1h));
	l1h = (struct l1if_hdr *) msg->l1h;
	l1h->msg_type = msg_type;
	
	return msg;
}
/* l1 confirms setup */
static int l1if_setup_conf(struct gsm_bts_trx *trx)
{

	trx_init_complete(trx, 0);

	LOGP(DL1C, LOGL_INFO, "L1 Setup confirm\n");
	/* signal availability */
	oml_mo_state_chg(&trx->mo, NM_OPSTATE_ENABLED, NM_AVSTATE_OK);
	oml_mo_tx_sw_act_rep(&trx->mo);
	oml_mo_state_chg(&trx->bb_transc.mo, NM_OPSTATE_ENABLED, NM_AVSTATE_OK);
	oml_mo_tx_sw_act_rep(&trx->bb_transc.mo);

	return 0;
}

/* sending parameters to init L1's TRX */
int l1if_setup(struct gsm_bts_trx *trx)
{
	uint16_t arfcn = trx->arfcn;
	uint8_t bsic = trx->bts->bsic;

	LOGP(DL1C, LOGL_INFO, "Setup TRX: arfcn=%d bsic=%d/%d\n",
		arfcn, bsic / 8, bsic & 7);
	
	l1if_setup_conf(trx);

	return 0;
}

static int l1if_new_si(struct gsm_bts_trx *trx, enum osmo_sysinfo_type osmo_si)
{
	char *name = "";

	switch (osmo_si) {
	case SYSINFO_TYPE_1:
		name = "1";
		break;
	case SYSINFO_TYPE_2:
		name = "2";
		break;
	case SYSINFO_TYPE_2bis:
		name = "2bis";
		break;
	case SYSINFO_TYPE_2ter:
		name = "2ter";
		break;
	case SYSINFO_TYPE_3:
		name = "3";
		break;
	case SYSINFO_TYPE_4:
		name = "4";
		break;
	case SYSINFO_TYPE_5:
		name = "5";
		break;
	case SYSINFO_TYPE_5bis:
		name = "5bis";
		break;
	case SYSINFO_TYPE_5ter:
		name = "5ter";
		break;
	case SYSINFO_TYPE_6:
		name = "6";
		break;
	case SYSINFO_TYPE_7:
	case SYSINFO_TYPE_8:
	case SYSINFO_TYPE_9:
	case SYSINFO_TYPE_10:
	case SYSINFO_TYPE_13:
	case SYSINFO_TYPE_16:
	case SYSINFO_TYPE_17:
	case SYSINFO_TYPE_18:
	case SYSINFO_TYPE_19:
	case SYSINFO_TYPE_20:
	case SYSINFO_TYPE_2quater:
	case SYSINFO_TYPE_EMO:
	case SYSINFO_TYPE_MEAS_INFO:
	default:
		LOGP(DL1C, LOGL_INFO, "Sysinfo (osmo_si %d) not supported.\n",
			osmo_si);
		return -ENOTSUP;
	}

	LOGP(DL1C, LOGL_INFO, "Providing Sysinfo %s to L1\n",
		name);

	return 0;
}

static int l1if_signal_cbfn(unsigned int subsys, unsigned int signal, void *hdlr_data,
				void *signal_data)
{
	if (subsys == SS_GLOBAL && signal == S_NEW_SYSINFO) {
		struct osmo_signal_new_si *new_si = signal_data;
		struct gsm_bts_trx *trx = new_si->trx;

		return l1if_new_si(trx, new_si->osmo_si);
	}
	return 0;
}

static int l1if_reset_cnf(struct osmo_l1ctl *l1ctl, struct msgb *msg)
{
	enum baseband_role bb_role = l1ctl->bb_role;
	struct osmo_l1_if *l1if = l1ctl->l1_if;
	struct gsm_bts_trx *trx = l1if->trx;
	int on = 1; // FIXME: handle failure (wrong firmware)

	if (bb_role == BASEBAND_TX) {
		LOGP(DL1C, LOGL_INFO, "Reset of TX baseband complete\n");
		l1if->reset_cnf_tx = 1;
	}
	if (bb_role == BASEBAND_RX) {
		LOGP(DL1C, LOGL_INFO, "Reset of RX baseband complete\n");
		l1if->reset_cnf_rx = 1;
	}
	if (!l1if->reset_cnf_tx || (!l1if->reset_cnf_rx && !tx_only)) {
		LOGP(DL1C, LOGL_INFO, "Waiting for other baseband\n");
		return 0;
	}

	if (on) {
		int i;
		/* signal availability */
		oml_mo_state_chg(&trx->mo, NM_OPSTATE_DISABLED, NM_AVSTATE_OK);
		oml_mo_tx_sw_act_rep(&trx->mo);
		oml_mo_state_chg(&trx->bb_transc.mo, -1, NM_AVSTATE_OK);
		oml_mo_tx_sw_act_rep(&trx->bb_transc.mo);

		for (i = 0; i < ARRAY_SIZE(trx->ts); i++)
			oml_mo_state_chg(&trx->ts[i].mo, NM_OPSTATE_DISABLED, NM_AVSTATE_DEPENDENCY);
	} else {
		oml_mo_state_chg(&trx->mo, NM_OPSTATE_DISABLED, NM_AVSTATE_OFF_LINE);
		oml_mo_state_chg(&trx->bb_transc.mo, NM_OPSTATE_DISABLED, NM_AVSTATE_OFF_LINE);
	}

	return 0;
}

int l1if_reset(struct gsm_bts_trx *trx)
{
	struct osmo_l1_if *l1if = trx_l1_if(trx);
	struct msgb *msg;
	struct l1ctl_reset *res;
	uint8_t type = L1CTL_RES_T_FULL;

	msg = osmo_l1if_alloc(L1IF_RESET_REQ);
	if (!msg)
		return -1;

	LOGP(DL1C, LOGL_INFO, "Tx Reset Req (%u) of TX baseband\n", type);
	res = (struct l1ctl_reset *) msgb_put(msg, sizeof(*res));
	res->type = type;

	l1ctl_send(&l1if->l1ctl_tx, msg);

	if (tx_only)
		return 0;

	msg = osmo_l1if_alloc(L1IF_RESET_REQ);
	if (!msg)
		return -1;

	LOGP(DL1C, LOGL_INFO, "Tx Reset Req (%u) of RX baseband\n", type);
	res = (struct l1ctl_reset *) msgb_put(msg, sizeof(*res));
	res->type = type;

	l1ctl_send(&l1if->l1ctl_rx, msg);

	return 0;
}

/* Receive incoming data from L1 using L1CTL format */
int l1if_recv(struct osmo_l1ctl *l1ctl, struct msgb *msg)
{
	int rc = -EINVAL;
	struct l1if_hdr *l1h;
	struct l1ctl_info_dl *dl;

	if (msgb_l2len(msg) < sizeof(*dl)) {
		LOGP(DL1C, LOGL_ERROR, "Short Layer2 message: %u\n",
			msgb_l2len(msg));
		msgb_free(msg);
		return -1;
	}

	l1h = (struct l1if_hdr *) msg->l1h;
	msg->l1h = l1h->data;

	switch (l1h->msg_type) {
	case L1IF_RESET_IND:
	case L1IF_RESET_CNF:
		rc = l1if_reset_cnf(l1ctl, msg);
		msgb_free(msg);
		break;
	default:
		LOGP(DL1C, LOGL_ERROR, "Unknown MSG: %u\n", l1h->msg_type);
		msgb_free(msg);
		break;
	}

	return rc;
}

int l1if_open(struct gsm_bts_trx *trx, const char *socket_path)
{
	struct osmo_l1_if *l1if;
	char pathname[128];
	int rc;

	osmo_signal_register_handler(SS_GLOBAL, l1if_signal_cbfn, NULL);

	l1if = talloc_zero(tall_bts_ctx, struct osmo_l1_if);
	if (!l1if)
		return -ENOMEM;
	l1if->trx = trx;
	l1if->l1ctl_tx.bb_role = BASEBAND_TX;
	l1if->l1ctl_tx.l1_if = l1if;
	sprintf(pathname, "%s.tx", socket_path);
	LOGP(DL1C, LOGL_INFO, "Open connection to TX baseband.\n");
	rc = l1socket_open(&l1if->l1ctl_tx, pathname);
	if (rc) {
		talloc_free(l1if);
		return rc;
	}
	if (!tx_only) {
		l1if->l1ctl_rx.bb_role = BASEBAND_RX;
		l1if->l1ctl_rx.l1_if = l1if;
		sprintf(pathname, "%s.rx", socket_path);
		LOGP(DL1C, LOGL_INFO, "Open connection to RX baseband.\n");
		rc = l1socket_open(&l1if->l1ctl_rx, pathname);
		if (rc) {
			l1socket_close(&l1if->l1ctl_tx);
			talloc_free(l1if);
			return rc;
		}
	}
	trx->role_bts.l1h = l1if;
	trx->nominal_power = 23;

	return 0;
}

int l1if_close(struct gsm_bts_trx *trx)
{
	struct osmo_l1_if *l1if = trx_l1_if(trx);

	l1socket_close(&l1if->l1ctl_tx);
	l1socket_close(&l1if->l1ctl_rx);
	talloc_free(l1if);
	osmo_signal_unregister_handler(SS_GLOBAL, l1if_signal_cbfn, NULL);
	return 0;
}

