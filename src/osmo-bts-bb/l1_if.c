/* Interface to layer 1 of baseband */

/* (C) 2010 by Holger Hans Peter Freyther
 * (C) 2010 by Harald Welte <laforge@gnumonks.org>
 * (C) 2011 by Andreas Eversberg <jolly@eversberg.eu>
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

#include "l1_if.h"
#include "oml.h"

uint16_t ref_arfcn;
extern int tx_only;

/*
 * socket communication with baseband
 */

#include "../../osmocom-bb/include/l1ctl_proto.h"

#define GSM_L2_LENGTH 256
#define GSM_L2_HEADROOM 32

static int l1if_recv(struct osmo_l1l2_if *l1l2if, struct msgb *msg);
static int layer2_close(struct osmo_l1l2_if *l1l2if);

static int layer2_read(struct osmo_fd *fd)
{
	struct msgb *msg;
	u_int16_t len;
	int rc;

	msg = msgb_alloc_headroom(GSM_L2_LENGTH+GSM_L2_HEADROOM, GSM_L2_HEADROOM, "Layer2");
	if (!msg) {
		LOGP(DL1C, LOGL_ERROR, "Failed to allocate msg.\n");
		return -ENOMEM;
	}

	rc = read(fd->fd, &len, sizeof(len));
	if (rc < sizeof(len)) {
		fprintf(stderr, "Layer2 socket failed\n");
		msgb_free(msg);
		if (rc >= 0)
			rc = -EIO;
		layer2_close((struct osmo_l1l2_if *) fd->data);
		return rc;
	}

	len = ntohs(len);
	if (len > GSM_L2_LENGTH) {
		LOGP(DL1C, LOGL_ERROR, "Length is too big: %u\n", len);
		msgb_free(msg);
		return -EINVAL;
	}


	msg->l1h = msgb_put(msg, len);
	rc = read(fd->fd, msg->l1h, msgb_l1len(msg));
	if (rc != msgb_l1len(msg)) {
		LOGP(DL1C, LOGL_ERROR, "Can not read data: len=%d rc=%d "
		     "errno=%d\n", len, rc, errno);
		msgb_free(msg);
		return rc;
	}

	l1if_recv((struct osmo_l1l2_if *) fd->data, msg);

	return 0;
}

static int layer2_write(struct osmo_fd *fd, struct msgb *msg)
{
	int rc;

	if (fd->fd <= 0)
		return -EINVAL;

	rc = write(fd->fd, msg->data, msg->len);
	if (rc != msg->len) {
		LOGP(DL1C, LOGL_ERROR, "Failed to write data: rc: %d\n", rc);
		return rc;
	}

	return 0;
}

static int layer2_open(struct osmo_l1l2_if *l1l2if, const char *socket_path)
{
	int rc;
	struct sockaddr_un local;

	l1l2if->l2_wq.bfd.fd = socket(AF_UNIX, SOCK_STREAM, 0);
	if (l1l2if->l2_wq.bfd.fd < 0) {
		fprintf(stderr, "Failed to create unix domain socket.\n");
		return l1l2if->l2_wq.bfd.fd;
	}

	local.sun_family = AF_UNIX;
	strncpy(local.sun_path, socket_path, sizeof(local.sun_path));
	local.sun_path[sizeof(local.sun_path) - 1] = '\0';

	rc = connect(l1l2if->l2_wq.bfd.fd, (struct sockaddr *) &local,
		     sizeof(local));
	if (rc < 0) {
		fprintf(stderr, "Failed to connect to '%s': %s\n", local.sun_path,
			strerror(errno));
		fprintf(stderr, "Please run osmocon for this socket.\n");
		close(l1l2if->l2_wq.bfd.fd);
		return rc;
	}

	osmo_wqueue_init(&l1l2if->l2_wq, 100);
	l1l2if->l2_wq.bfd.data = l1l2if;
	l1l2if->l2_wq.bfd.when = BSC_FD_READ;
	l1l2if->l2_wq.read_cb = layer2_read;
	l1l2if->l2_wq.write_cb = layer2_write;

	rc = osmo_fd_register(&l1l2if->l2_wq.bfd);
	if (rc != 0) {
		fprintf(stderr, "Failed to register fd.\n");
		close(l1l2if->l2_wq.bfd.fd);
		return rc;
	}

	return 0;
}

static int layer2_close(struct osmo_l1l2_if *l1l2if)
{
	if (l1l2if->l2_wq.bfd.fd <= 0)
		return -EINVAL;

	close(l1l2if->l2_wq.bfd.fd);
	l1l2if->l2_wq.bfd.fd = -1;
	osmo_fd_unregister(&l1l2if->l2_wq.bfd);

	return 0;
}

static int osmo_send_l1(struct osmo_l1l2_if *l1l2if, struct msgb *msg)
{
	uint16_t *len;

	DEBUGP(DL1C, "Sending: '%s'\n", osmo_hexdump(msg->data, msg->len));

	if (msg->l1h != msg->data)
		LOGP(DL1C, LOGL_ERROR, "Message L1 header != Message Data\n");
	
	/* prepend 16bit length before sending */
	len = (uint16_t *) msgb_push(msg, sizeof(*len));
	*len = htons(msg->len - sizeof(*len));

	if (osmo_wqueue_enqueue(&l1l2if->l2_wq, msg) != 0) {
		LOGP(DL1C, LOGL_ERROR, "Failed to enqueue msg.\n");
		msgb_free(msg);
		return -1;
	}

	return 0;
}

/*
 * messages to and from layer 1
 */

static struct msgb *osmo_l1_alloc(uint8_t msg_type)
{
	struct l1ctl_hdr *l1h;
	struct msgb *msg = msgb_alloc_headroom(256, 4, "osmo_l1");

	if (!msg) {
		LOGP(DL1C, LOGL_ERROR, "Failed to allocate memory.\n");
		return NULL;
	}

	msg->l1h = msgb_put(msg, sizeof(*l1h));
	l1h = (struct l1ctl_hdr *) msg->l1h;
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
	
// Sylvain: put your code here: init trx, then call on a confirm:
	l1if_setup_conf(trx);

	return 0;
}

static int l1if_new_si(struct gsm_bts_trx *trx, enum osmo_sysinfo_type osmo_si)
{
	char *name = "";

	switch (osmo_si) {
	case SYSINFO_TYPE_1:
		name = "1";
// Sylvain: put your code here
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
// FIXME: prepare bursts and send
// Sylvain: put your code here

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

static int l1if_reset_cnf(struct bbl1_hdl *bbl1h, struct msgb *msg, enum baseband_role bb_role)
{
	struct gsm_bts_trx *trx = bbl1h->trx;
	int on = 1; // FIXME: handle failure (wrong firmware)

	if (bb_role == BASEBAND_TX) {
		LOGP(DL1C, LOGL_INFO, "Reset of TX baseband complete\n");
		bbl1h->reset_cnf_tx = 1;
	}
	if (bb_role == BASEBAND_RX) {
		LOGP(DL1C, LOGL_INFO, "Reset of RX baseband complete\n");
		bbl1h->reset_cnf_rx = 1;
	}
	if (!bbl1h->reset_cnf_tx || (!bbl1h->reset_cnf_rx && !tx_only)) {
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
	struct bbl1_hdl *bbl1h = trx_bbl1_hdl(trx);
	struct msgb *msg;
	struct l1ctl_reset *res;
	uint8_t type = L1CTL_RES_T_FULL;

	msg = osmo_l1_alloc(L1CTL_RESET_REQ);
	if (!msg)
		return -1;

	LOGP(DL1C, LOGL_INFO, "Tx Reset Req (%u) of TX baseband\n", type);
	res = (struct l1ctl_reset *) msgb_put(msg, sizeof(*res));
	res->type = type;

	osmo_send_l1(&bbl1h->l1l2if_tx, msg);

	if (tx_only)
		return 0;

	msg = osmo_l1_alloc(L1CTL_RESET_REQ);
	if (!msg)
		return -1;

	LOGP(DL1C, LOGL_INFO, "Tx Reset Req (%u) of RX baseband\n", type);
	res = (struct l1ctl_reset *) msgb_put(msg, sizeof(*res));
	res->type = type;

	osmo_send_l1(&bbl1h->l1l2if_rx, msg);

	return 0;
}

/* Receive incoming data from L1 using L1CTL format */
static int l1if_recv(struct osmo_l1l2_if *l1l2if, struct msgb *msg)
{
	struct bbl1_hdl *bbl1h = l1l2if->bbl1h;
	int rc = 0;
	struct l1ctl_hdr *l1h;
	struct l1ctl_info_dl *dl;

	if (msgb_l2len(msg) < sizeof(*dl)) {
		LOGP(DL1C, LOGL_ERROR, "Short Layer2 message: %u\n",
			msgb_l2len(msg));
		msgb_free(msg);
		return -1;
	}

	l1h = (struct l1ctl_hdr *) msg->l1h;

	/* move the l1 header pointer to point _BEHIND_ l1ctl_hdr,
	   as the l1ctl header is of no interest to subsequent code */
	msg->l1h = l1h->data;

	switch (l1h->msg_type) {
	case L1CTL_RESET_IND:
	case L1CTL_RESET_CONF:
		rc = l1if_reset_cnf(bbl1h, msg, l1l2if->bb_role);
		msgb_free(msg);
		break;
#if 0
	case L1CTL_FBSB_CONF:
		rc = rx_l1_fbsb_conf(ms, msg);
		msgb_free(msg);
		break;
	case L1CTL_DATA_IND:
		rc = rx_ph_data_ind(ms, msg);
		break;
	case L1CTL_DATA_CONF:
		rc = rx_ph_data_conf(ms, msg);
		break;
	case L1CTL_PM_CONF:
		rc = rx_l1_pm_conf(ms, msg);
		if (l1h->flags & L1CTL_F_DONE)
			osmo_signal_dispatch(SS_L1CTL, S_L1CTL_PM_DONE, ms);
		msgb_free(msg);
		break;
	case L1CTL_RACH_CONF:
		rc = rx_l1_rach_conf(ms, msg);
		break;
	case L1CTL_CCCH_MODE_CONF:
		rc = rx_l1_ccch_mode_conf(ms, msg);
		msgb_free(msg);
		break;
	case L1CTL_TCH_MODE_CONF:
		rc = rx_l1_tch_mode_conf(ms, msg);
		msgb_free(msg);
		break;
	case L1CTL_SIM_CONF:
		rc = rx_l1_sim_conf(ms, msg);
		break;
	case L1CTL_NEIGH_PM_IND:
		rc = rx_l1_neigh_pm_ind(ms, msg);
		msgb_free(msg);
		break;
	case L1CTL_TRAFFIC_IND:
		rc = rx_l1_traffic_ind(ms, msg);
		break;
	case L1CTL_TRAFFIC_CONF:
		msgb_free(msg);
		break;
#endif
	default:
		LOGP(DL1C, LOGL_ERROR, "Unknown MSG: %u\n", l1h->msg_type);
		msgb_free(msg);
		break;
	}

	return rc;
}

int l1if_open(struct gsm_bts_trx *trx, const char *socket_path)
{
	struct bbl1_hdl *bbl1h;
	char pathname[128];
	int rc;

	osmo_signal_register_handler(SS_GLOBAL, l1if_signal_cbfn, NULL);

	bbl1h = talloc_zero(tall_bts_ctx, struct bbl1_hdl);
	if (!bbl1h)
		return -ENOMEM;
	bbl1h->trx = trx;
	bbl1h->l1l2if_tx.bb_role = BASEBAND_TX;
	bbl1h->l1l2if_tx.bbl1h = bbl1h;
	sprintf(pathname, "%s.tx", socket_path);
	LOGP(DL1C, LOGL_INFO, "Open connection to TX baseband.\n");
	rc = layer2_open(&bbl1h->l1l2if_tx, pathname);
	if (rc) {
		talloc_free(bbl1h);
		return rc;
	}
	if (!tx_only) {
		bbl1h->l1l2if_rx.bb_role = BASEBAND_RX;
		bbl1h->l1l2if_rx.bbl1h = bbl1h;
		sprintf(pathname, "%s.rx", socket_path);
		LOGP(DL1C, LOGL_INFO, "Open connection to RX baseband.\n");
		rc = layer2_open(&bbl1h->l1l2if_rx, pathname);
		if (rc) {
			layer2_close(&bbl1h->l1l2if_tx);
			talloc_free(bbl1h);
			return rc;
		}
	}
	trx->role_bts.l1h = bbl1h;
	trx->nominal_power = 23;

	return 0;
}

int l1if_close(struct gsm_bts_trx *trx)
{
	struct bbl1_hdl *bbl1h = trx_bbl1_hdl(trx);

	layer2_close(&bbl1h->l1l2if_tx);
	layer2_close(&bbl1h->l1l2if_rx);
	talloc_free(bbl1h);
	osmo_signal_unregister_handler(SS_GLOBAL, l1if_signal_cbfn, NULL);
	return 0;
}
