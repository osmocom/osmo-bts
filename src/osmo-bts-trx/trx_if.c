/*
 * OpenBTS-style TRX interface/protocol handling
 *
 * This file contains the BTS-side implementation of the OpenBTS-style
 * UDP TRX protocol.  It manages the clock, control + burst-data UDP
 * sockets and their respective protocol encoding/parsing.
 *
 * Copyright (C) 2013  Andreas Eversberg <jolly@eversberg.eu>
 * Copyright (C) 2016-2017  Harald Welte <laforge@gnumonks.org>
 * Copyright (C) 2019  Vadim Yanitskiy <axilirator@gmail.com>
 * Copyright (C) 2021  sysmocom - s.m.f.c. GmbH <info@sysmocom.de>
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

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>

#include <netinet/in.h>

#include <osmocom/core/select.h>
#include <osmocom/core/socket.h>
#include <osmocom/core/timer.h>
#include <osmocom/core/talloc.h>
#include <osmocom/core/bits.h>
#include <osmocom/core/fsm.h>

#include <osmo-bts/phy_link.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/scheduler.h>

#include "l1_if.h"
#include "trx_if.h"
#include "trx_provision_fsm.h"

#include "btsconfig.h"

#ifdef HAVE_SYSTEMTAP
/* include the generated probes header and put markers in code */
#include "probes.h"
#define TRACE(probe) probe
#define TRACE_ENABLED(probe) probe ## _ENABLED()
#else
/* Wrap the probe to allow it to be removed when no systemtap available */
#define TRACE(probe)
#define TRACE_ENABLED(probe) (0)
#endif /* HAVE_SYSTEMTAP */

/*
 * socket helper functions
 */

/*! convenience wrapper to open socket + fill in osmo_fd */
static int trx_udp_open(void *priv, struct osmo_fd *ofd, const char *host_local,
			uint16_t port_local, const char *host_remote, uint16_t port_remote,
			int (*cb)(struct osmo_fd *fd, unsigned int what))
{
	int rc;

	/* Init */
	ofd->fd = -1;
	ofd->cb = cb;
	ofd->data = priv;

	/* Listen / Binds + Connect */
	rc = osmo_sock_init2_ofd(ofd, AF_UNSPEC, SOCK_DGRAM, IPPROTO_UDP, host_local, port_local,
				host_remote, port_remote, OSMO_SOCK_F_BIND | OSMO_SOCK_F_CONNECT);
	if (rc < 0)
		return rc;

	return 0;
}

/* close socket + unregister osmo_fd */
static void trx_udp_close(struct osmo_fd *ofd)
{
	if (ofd->fd >= 0) {
		osmo_fd_unregister(ofd);
		close(ofd->fd);
		ofd->fd = -1;
	}
}


/*
 * TRX clock socket
 */

/* get clock from clock socket */
static int trx_clk_read_cb(struct osmo_fd *ofd, unsigned int what)
{
	struct phy_link *plink = ofd->data;
	struct phy_instance *pinst = phy_instance_by_num(plink, 0);
	char buf[TRXC_MSG_BUF_SIZE];
	ssize_t len;
	uint32_t fn;

	OSMO_ASSERT(pinst);

	len = recv(ofd->fd, buf, sizeof(buf) - 1, 0);
	if (len <= 0) {
		strerror_r(errno, (char *)buf, sizeof(buf));
		LOGPPHI(pinst, DTRX, LOGL_ERROR,
			"recv() failed on TRXD with rc=%zd (%s)\n", len, buf);
		return len;
	}
	buf[len] = '\0';

	if (!!strncmp(buf, "IND CLOCK ", 10)) {
		LOGPPHI(pinst, DTRX, LOGL_NOTICE,
			"Unknown message on clock port: %s\n", buf);
		return 0;
	}

	if (sscanf(buf, "IND CLOCK %u", &fn) != 1) {
		LOGPPHI(pinst, DTRX, LOGL_ERROR, "Unable to parse '%s'\n", buf);
		return 0;
	}

	LOGPPHI(pinst, DTRX, LOGL_INFO, "Clock indication: fn=%u\n", fn);

	if (fn >= GSM_TDMA_HYPERFRAME) {
		fn %= GSM_TDMA_HYPERFRAME;
		LOGPPHI(pinst, DTRX, LOGL_ERROR, "Indicated clock's FN is not "
			"wrapping correctly, correcting to fn=%u\n", fn);
	}

	if (!plink->u.osmotrx.powered) {
		LOGPPHI(pinst, DTRX, LOGL_NOTICE, "Ignoring CLOCK IND %u, TRX not yet powered on\n", fn);
		return 0;
	}
	/* inform core TRX clock handling code that a FN has been received */
	trx_sched_clock(pinst->trx->bts, fn);

	return 0;
}


/*
 * TRX ctrl socket
 */

/* send first ctrl message and start timer */
static void trx_ctrl_send(struct trx_l1h *l1h)
{
	struct trx_ctrl_msg *tcm;
	char buf[TRXC_MSG_BUF_SIZE];
	int len;
	ssize_t snd_len;

	/* get first command */
	if (llist_empty(&l1h->trx_ctrl_list))
		return;
	tcm = llist_entry(l1h->trx_ctrl_list.next, struct trx_ctrl_msg, list);

	len = snprintf(buf, sizeof(buf), "CMD %s%s%s", tcm->cmd, tcm->params_len ? " ":"", tcm->params);
	OSMO_ASSERT(len < sizeof(buf));

	LOGPPHI(l1h->phy_inst, DTRX, LOGL_DEBUG, "Sending control '%s'\n", buf);
	/* send command */
	snd_len = send(l1h->trx_ofd_ctrl.fd, buf, len+1, 0);
	if (snd_len <= 0) {
		strerror_r(errno, (char *)buf, sizeof(buf));
		LOGPPHI(l1h->phy_inst, DTRX, LOGL_ERROR,
			"send() failed on TRXC with rc=%zd (%s)\n", snd_len, buf);
	}

	/* start timer */
	osmo_timer_schedule(&l1h->trx_ctrl_timer, 2, 0);
}

/* send first ctrl message and start timer */
static void trx_ctrl_timer_cb(void *data)
{
	struct trx_l1h *l1h = data;
	struct trx_ctrl_msg *tcm = NULL;

	/* get first command */
	OSMO_ASSERT(!llist_empty(&l1h->trx_ctrl_list));
	tcm = llist_entry(l1h->trx_ctrl_list.next, struct trx_ctrl_msg, list);

	LOGPPHI(l1h->phy_inst, DTRX, LOGL_NOTICE, "No satisfactory response from transceiver(CMD %s%s%s)\n",
		tcm->cmd, tcm->params_len ? " ":"", tcm->params);

	trx_ctrl_send(l1h);
}

void trx_if_init(struct trx_l1h *l1h)
{
	l1h->trx_ctrl_timer.cb = trx_ctrl_timer_cb;
	l1h->trx_ctrl_timer.data = l1h;

	/* initialize ctrl queue */
	INIT_LLIST_HEAD(&l1h->trx_ctrl_list);

	l1h->trx_ofd_ctrl.fd = -1;
	l1h->trx_ofd_data.fd = -1;
}

/*! Send a new TRX control command.
 *  \param[inout] l1h TRX Layer1 handle to which to send command
 *  \param[in] critical
 *  \param[in] cb callback function to be called when valid response is
 *  		  received. Type of cb depends on type of message.
 *  \param[in] cmd zero-terminated string containing command
 *  \param[in] fmt Format string (+ variable list of arguments)
 *  \returns 0 on success; negative on error
 *
 *  The new command will be added to the end of the control command
 *  queue.
 */
static int trx_ctrl_cmd_cb(struct trx_l1h *l1h, int critical, void *cb, const char *cmd,
	const char *fmt, ...)
{
	struct trx_ctrl_msg *tcm;
	struct trx_ctrl_msg *prev = NULL;
	va_list ap;

	/* create message */
	tcm = talloc_zero(tall_bts_ctx, struct trx_ctrl_msg);
	if (!tcm)
		return -ENOMEM;
	snprintf(tcm->cmd, sizeof(tcm->cmd)-1, "%s", cmd);
	tcm->cmd[sizeof(tcm->cmd)-1] = '\0';
	tcm->cmd_len = strlen(tcm->cmd);
	if (fmt && fmt[0]) {
		va_start(ap, fmt);
		vsnprintf(tcm->params, sizeof(tcm->params) - 1, fmt, ap);
		va_end(ap);
		tcm->params[sizeof(tcm->params)-1] = '\0';
		tcm->params_len = strlen(tcm->params);
	} else {
		tcm->params[0] ='\0';
		tcm->params_len = 0;
	}
	tcm->critical = critical;
	tcm->cb = cb;

	/* Avoid adding consecutive duplicate messages, eg: two consecutive POWEROFF */
	if (!llist_empty(&l1h->trx_ctrl_list))
		prev = llist_entry(l1h->trx_ctrl_list.prev, struct trx_ctrl_msg, list);
	if (prev != NULL && !strcmp(tcm->cmd, prev->cmd)
			 && !strcmp(tcm->params, prev->params)) {
		LOGPPHI(l1h->phy_inst, DTRX, LOGL_DEBUG,
			"Not sending duplicate command '%s'\n", tcm->cmd);
		talloc_free(tcm);
		return 0;
	}

	LOGPPHI(l1h->phy_inst, DTRX, LOGL_INFO, "Enqueuing TRX control command 'CMD %s%s%s'\n",
		tcm->cmd, tcm->params_len ? " " : "", tcm->params);
	llist_add_tail(&tcm->list, &l1h->trx_ctrl_list);

	/* send message, if we didn't already have pending messages */
	if (prev == NULL)
		trx_ctrl_send(l1h);

	return 0;
}
#define trx_ctrl_cmd(l1h, critical, cmd, fmt, ...) trx_ctrl_cmd_cb(l1h, critical, NULL, cmd, fmt, ##__VA_ARGS__)

/*! Send "POWEROFF" command to TRX */
int trx_if_cmd_poweroff(struct trx_l1h *l1h, trx_if_cmd_poweronoff_cb *cb)
{
	return trx_ctrl_cmd_cb(l1h, 1, cb, "POWEROFF", "");
}

/*! Send "POWERON" command to TRX */
int trx_if_cmd_poweron(struct trx_l1h *l1h, trx_if_cmd_poweronoff_cb *cb)
{
	return trx_ctrl_cmd_cb(l1h, 1, cb, "POWERON", "");
}

/*! Send "SETFORMAT" command to TRX: change TRXD PDU version */
int trx_if_cmd_setformat(struct trx_l1h *l1h, uint8_t ver, trx_if_cmd_generic_cb *cb)
{
	LOGPPHI(l1h->phy_inst, DTRX, LOGL_INFO,
		"Requesting TRXD PDU version %u\n", ver);

	return trx_ctrl_cmd_cb(l1h, 0, cb, "SETFORMAT", "%u", ver);
}

/*! Send "SETTSC" command to TRX */
int trx_if_cmd_settsc(struct trx_l1h *l1h, uint8_t tsc, trx_if_cmd_generic_cb *cb)
{
	return trx_ctrl_cmd_cb(l1h, 1, cb, "SETTSC", "%d", tsc);
}

/*! Send "SETBSIC" command to TRX */
int trx_if_cmd_setbsic(struct trx_l1h *l1h, uint8_t bsic, trx_if_cmd_generic_cb *cb)
{
	return trx_ctrl_cmd_cb(l1h, 1, cb, "SETBSIC", "%d", bsic);
}

/*! Send "SETRXGAIN" command to TRX */
int trx_if_cmd_setrxgain(struct trx_l1h *l1h, int db)
{
	return trx_ctrl_cmd(l1h, 0, "SETRXGAIN", "%d", db);
}

/*! Send "NOMTXPOWER" command to TRX */
int trx_if_cmd_getnompower(struct trx_l1h *l1h, trx_if_cmd_getnompower_cb *cb)
{
	return trx_ctrl_cmd_cb(l1h, 1, cb, "NOMTXPOWER", "");
}

/*! Send "SETPOWER" command to TRX */
int trx_if_cmd_setpower_att(struct trx_l1h *l1h, int power_att_db, trx_if_cmd_setpower_att_cb *cb)
{
	return trx_ctrl_cmd_cb(l1h, 0, cb, "SETPOWER", "%d", power_att_db);
}

/*! Send "SETMAXDLY" command to TRX, i.e. maximum delay for RACH bursts */
int trx_if_cmd_setmaxdly(struct trx_l1h *l1h, int dly)
{
	return trx_ctrl_cmd(l1h, 0, "SETMAXDLY", "%d", dly);
}

/*! Send "SETMAXDLYNB" command to TRX, i.e. maximum delay for normal bursts */
int trx_if_cmd_setmaxdlynb(struct trx_l1h *l1h, int dly)
{
	return trx_ctrl_cmd(l1h, 0, "SETMAXDLYNB", "%d", dly);
}

/*! Send "SETSLOT" command to TRX: Configure Channel Combination and TSC for TS */
int trx_if_cmd_setslot(struct trx_l1h *l1h, uint8_t tn,
		       trx_if_cmd_setslot_cb *cb)
{
	const struct trx_config *cfg = &l1h->config;
	const struct phy_instance *pinst = l1h->phy_inst;

	if (cfg->setslot[tn].tsc_valid && cfg->setslot[tn].tsc_val != BTS_TSC(pinst->trx->bts)) {
		/* PHY is instructed to use a custom TSC */
		return trx_ctrl_cmd_cb(l1h, 1, cb, "SETSLOT", "%u %u C%u/S%u",
				       tn, cfg->setslot[tn].slottype,
				       cfg->setslot[tn].tsc_val,
				       cfg->setslot[tn].tsc_set);
       } else { /* PHY is instructed to use the default TSC from 'SETTSC' */
		return trx_ctrl_cmd_cb(l1h, 1, cb, "SETSLOT", "%u %u",
				       tn, cfg->setslot[tn].slottype);
	}
}

/*! Send "RXTUNE" command to TRX: Tune Receiver to given ARFCN */
int trx_if_cmd_rxtune(struct trx_l1h *l1h, uint16_t arfcn, trx_if_cmd_generic_cb *cb)
{
	struct phy_instance *pinst = l1h->phy_inst;
	uint16_t freq10;

	if (pinst->trx->bts->band == GSM_BAND_1900)
		arfcn |= ARFCN_PCS;

	freq10 = gsm_arfcn2freq10(arfcn, 1); /* RX = uplink */
	if (freq10 == 0xffff) {
		LOGPPHI(pinst, DTRX, LOGL_ERROR, "Arfcn %d not defined.\n",
			arfcn & ~ARFCN_FLAG_MASK);
		return -ENOTSUP;
	}

	return trx_ctrl_cmd_cb(l1h, 1, cb, "RXTUNE", "%d", freq10 * 100);
}

/*! Send "TXTUNE" command to TRX: Tune Transmitter to given ARFCN */
int trx_if_cmd_txtune(struct trx_l1h *l1h, uint16_t arfcn, trx_if_cmd_generic_cb *cb)
{
	struct phy_instance *pinst = l1h->phy_inst;
	uint16_t freq10;

	if (pinst->trx->bts->band == GSM_BAND_1900)
		arfcn |= ARFCN_PCS;

	freq10 = gsm_arfcn2freq10(arfcn, 0); /* TX = downlink */
	if (freq10 == 0xffff) {
		LOGPPHI(pinst, DTRX, LOGL_ERROR, "Arfcn %d not defined.\n",
			arfcn & ~ARFCN_FLAG_MASK);
		return -ENOTSUP;
	}

	return trx_ctrl_cmd_cb(l1h, 1, cb, "TXTUNE", "%d", freq10 * 100);
}

/*! Send "HANDOVER" command to TRX: Enable handover RACH Detection on timeslot/sub-slot */
int trx_if_cmd_handover(struct trx_l1h *l1h, uint8_t tn, uint8_t ss)
{
	return trx_ctrl_cmd(l1h, 1, "HANDOVER", "%d %d", tn, ss);
}

/*! Send "NOHANDOVER" command to TRX: Disable handover RACH Detection on timeslot/sub-slot */
int trx_if_cmd_nohandover(struct trx_l1h *l1h, uint8_t tn, uint8_t ss)
{
	return trx_ctrl_cmd(l1h, 1, "NOHANDOVER", "%d %d", tn, ss);
}

/*! Send "RFMUTE" command to TRX: Mute or Unmute RF transmission */
int trx_if_cmd_rfmute(struct trx_l1h *l1h, bool mute)
{
	return trx_ctrl_cmd(l1h, 0, "RFMUTE", mute ? "1" : "0");
}

struct trx_ctrl_rsp {
	char cmd[50];
	char params[100];
	int status;
	void *cb;
};

static int parse_rsp(const char *buf_in, size_t len_in, struct trx_ctrl_rsp *rsp)
{
	size_t nlen, plen;
	char *p, *k;

	if (strncmp(buf_in, "RSP ", 4))
		goto parse_err;

	/* Get the RSP cmd name */
	if (!(p = strchr(buf_in + 4, ' ')))
		goto parse_err;

	/* Calculate length of the name part */
	nlen = p - (buf_in + 4);

	if (nlen >= sizeof(rsp->cmd)) {
		LOGP(DTRX, LOGL_ERROR, "TRXC command name part is too long: "
		     "%zu >= %zu\n", nlen, sizeof(rsp->cmd));
		goto parse_err;
	}

	memcpy(&rsp->cmd[0], buf_in + 4, nlen);
	rsp->cmd[nlen] = '\0';

	/* Now comes the status code of the response */
	p++;
	if (sscanf(p, "%d", &rsp->status) != 1)
		goto parse_err;

	/* Now copy back the parameters */
	k = strchr(p, ' ');
	if (k)
		k++;
	else
		k = p + strlen(p);

	/* Calculate length of the parameters part */
	plen = strlen(k);

	if (plen >= sizeof(rsp->params)) {
		LOGP(DTRX, LOGL_ERROR, "TRXC command parameters part is too long: "
		     "%zu >= %zu\n", plen, sizeof(rsp->params));
		goto parse_err;
	}

	memcpy(&rsp->params[0], k, plen);
	rsp->params[plen] = '\0';

	return 0;

parse_err:
	LOGP(DTRX, LOGL_NOTICE, "Unknown TRXC message: %s\n", buf_in);
	return -1;
}

static bool cmd_matches_rsp(struct trx_ctrl_msg *tcm, struct trx_ctrl_rsp *rsp)
{
	if (strcmp(tcm->cmd, rsp->cmd))
		return false;

	/* For SETSLOT we also need to check if it's the response for the
	   specific timeslot. For other commands such as SETRXGAIN, it is
	   expected that they can return different values */
	if (strcmp(tcm->cmd, "SETSLOT") == 0 && strcmp(tcm->params, rsp->params))
		return false;
	else if (strcmp(tcm->cmd, "SETFORMAT") == 0 && strcmp(tcm->params, rsp->params))
		return false;

	return true;
}

static int trx_ctrl_rx_rsp_poweron(struct trx_l1h *l1h, struct trx_ctrl_rsp *rsp)
{
	trx_if_cmd_poweronoff_cb *cb = (trx_if_cmd_poweronoff_cb*) rsp->cb;

	if (rsp->status != 0)
		LOGPPHI(l1h->phy_inst, DTRX, LOGL_NOTICE,
			"transceiver rejected POWERON command (%d), re-trying in a few seconds\n",
			rsp->status);

	if (cb)
		cb(l1h, true, rsp->status);

	/* If TRX fails, try again after 5 sec */
	return rsp->status == 0 ? 0 : 5;
}

static int trx_ctrl_rx_rsp_poweroff(struct trx_l1h *l1h, struct trx_ctrl_rsp *rsp)
{
	trx_if_cmd_poweronoff_cb *cb = (trx_if_cmd_poweronoff_cb*) rsp->cb;

	if (rsp->status == 0) {
		if (cb)
			cb(l1h, false, rsp->status);
		return 0;
	}

	return -EINVAL;
}

static int trx_ctrl_rx_rsp_setslot(struct trx_l1h *l1h, struct trx_ctrl_rsp *rsp)
{
	trx_if_cmd_setslot_cb *cb = (trx_if_cmd_setslot_cb*) rsp->cb;
	struct phy_instance *pinst = l1h->phy_inst;
	unsigned int tn, ts_type;

	if (rsp->status)
		LOGPPHI(pinst, DTRX, LOGL_ERROR, "transceiver SETSLOT failed with status %d\n",
			rsp->status);

	/* Since message was already validated against CMD we sent, we know format
	 * of params is: "<TN> <TS_TYPE>" */
	if (sscanf(rsp->params, "%u %u", &tn, &ts_type) < 2) {
		LOGPPHI(pinst, DTRX, LOGL_ERROR, "transceiver SETSLOT unable to parse params\n");
		return -EINVAL;
	}

	if (cb)
		cb(l1h, tn, ts_type, rsp->status);

	return rsp->status == 0 ? 0 : -EINVAL;
}

/* TRXD PDU format negotiation handler.
 *
 * If the transceiver does not support the format negotiation, it would
 * reject SETFORMAT with 'RSP ERR 1'. If the requested version is not
 * supported by the transceiver, status code of the response message
 * should indicate a preferred (basically, the latest) version.
 */
static int trx_ctrl_rx_rsp_setformat(struct trx_l1h *l1h,
				     struct trx_ctrl_rsp *rsp)
{
	trx_if_cmd_generic_cb *cb;

	/* Old transceivers reject 'SETFORMAT' with 'RSP ERR 1' */
	if (strcmp(rsp->cmd, "SETFORMAT") != 0) {
		LOGPPHI(l1h->phy_inst, DTRX, LOGL_NOTICE,
			"Transceiver rejected the format negotiation command, "
			"using legacy TRXD PDU version (0)\n");
		if (rsp->cb) {
			cb = (trx_if_cmd_generic_cb*) rsp->cb;
			cb(l1h, 0);
		}
		return 0;
	}

	/* Status shall indicate a proper version supported by the transceiver */
	if (rsp->status < 0 || rsp->status > l1h->config.trxd_pdu_ver_req) {
		LOGPPHI(l1h->phy_inst, DTRX, LOGL_ERROR,
			"Transceiver indicated an out of range "
			"PDU version %d (requested %u)\n",
			rsp->status, l1h->config.trxd_pdu_ver_req);
		return -EINVAL;
	}

	if (rsp->cb) {
		cb = (trx_if_cmd_generic_cb*) rsp->cb;
		cb(l1h, rsp->status);
	}

	return 0;
}

static int trx_ctrl_rx_rsp_nomtxpower(struct trx_l1h *l1h, struct trx_ctrl_rsp *rsp)
{
	trx_if_cmd_getnompower_cb *cb = (trx_if_cmd_getnompower_cb*) rsp->cb;
	struct phy_instance *pinst = l1h->phy_inst;
	int nominal_power;

	if (rsp->status)
		LOGPPHI(pinst, DTRX, LOGL_ERROR, "transceiver NOMTXPOWER failed "
			"with status %d. If your transceiver doesn't support this "
			"command, then please set the nominal transmit power manually "
			"through VTY cmd 'nominal-tx-power'.\n",
			rsp->status);
	if (cb) {
		sscanf(rsp->params, "%d", &nominal_power);
		cb(l1h, nominal_power, rsp->status);
	}
	return 0;
}

static int trx_ctrl_rx_rsp_setpower(struct trx_l1h *l1h, struct trx_ctrl_rsp *rsp)
{
	trx_if_cmd_setpower_att_cb *cb = (trx_if_cmd_setpower_att_cb*) rsp->cb;
	struct phy_instance *pinst = l1h->phy_inst;
	int power_att;

	if (rsp->status)
		LOGPPHI(pinst, DTRX, LOGL_ERROR, "transceiver SETPOWER failed with status %d\n",
			rsp->status);
	if (cb) {
		sscanf(rsp->params, "%d", &power_att);
		cb(l1h, power_att, rsp->status);
	}
	return 0;
}

/* -EINVAL: unrecoverable error, exit BTS
 * N > 0: try sending originating command again after N seconds
 * 0: Done with response, get originating command out from send queue
 */
static int trx_ctrl_rx_rsp(struct trx_l1h *l1h,
			   struct trx_ctrl_rsp *rsp,
			   struct trx_ctrl_msg *tcm)
{
	trx_if_cmd_generic_cb *cb;

	if (strcmp(rsp->cmd, "POWERON") == 0) {
		return trx_ctrl_rx_rsp_poweron(l1h, rsp);
	} else if (strcmp(rsp->cmd, "POWEROFF") == 0) {
		return trx_ctrl_rx_rsp_poweroff(l1h, rsp);
	} else if (strcmp(rsp->cmd, "SETSLOT") == 0) {
		return trx_ctrl_rx_rsp_setslot(l1h, rsp);
	/* We may get 'RSP ERR 1' if 'SETFORMAT' is not supported,
	 * so that's why we should use tcm instead of rsp. */
	} else if (strcmp(tcm->cmd, "SETFORMAT") == 0) {
		return trx_ctrl_rx_rsp_setformat(l1h, rsp);
	} else if (strcmp(tcm->cmd, "NOMTXPOWER") == 0) {
		return trx_ctrl_rx_rsp_nomtxpower(l1h, rsp);
	} else if (strcmp(tcm->cmd, "SETPOWER") == 0) {
		return trx_ctrl_rx_rsp_setpower(l1h, rsp);
	}

	/* Generic callback if available */
	if (rsp->cb) {
		cb = (trx_if_cmd_generic_cb*) rsp->cb;
		cb(l1h, rsp->status);
	}

	if (rsp->status) {
		LOGPPHI(l1h->phy_inst, DTRX, tcm->critical ? LOGL_FATAL : LOGL_NOTICE,
			"transceiver rejected TRX command with response: '%s%s%s %d'\n",
			rsp->cmd, rsp->params[0] != '\0' ? " ":"",
			rsp->params, rsp->status);
		if (tcm->critical)
			return -EINVAL;
	}
	return 0;
}

/*! Get + parse response from TRX ctrl socket */
static int trx_ctrl_read_cb(struct osmo_fd *ofd, unsigned int what)
{
	struct trx_l1h *l1h = ofd->data;
	struct phy_instance *pinst = l1h->phy_inst;
	char buf[TRXC_MSG_BUF_SIZE];
	struct trx_ctrl_rsp rsp;
	int len, rc;
	struct trx_ctrl_msg *tcm;

	len = recv(ofd->fd, buf, sizeof(buf) - 1, 0);
	if (len <= 0)
		return len;
	buf[len] = '\0';

	if (parse_rsp(buf, len, &rsp) < 0)
		return 0;

	LOGPPHI(l1h->phy_inst, DTRX, LOGL_INFO, "Response message: '%s'\n", buf);

	/* abort timer and send next message, if any */
	if (osmo_timer_pending(&l1h->trx_ctrl_timer))
		osmo_timer_del(&l1h->trx_ctrl_timer);

	/* get command for response message */
	if (llist_empty(&l1h->trx_ctrl_list)) {
		/* RSP from a retransmission, skip it */
		if (l1h->last_acked && cmd_matches_rsp(l1h->last_acked, &rsp)) {
			LOGPPHI(l1h->phy_inst, DTRX, LOGL_NOTICE, "Discarding duplicated RSP "
				"from old CMD '%s'\n", buf);
			return 0;
		}
		LOGPPHI(l1h->phy_inst, DTRX, LOGL_NOTICE, "Response message without command\n");
		return -EINVAL;
	}
	tcm = llist_entry(l1h->trx_ctrl_list.next, struct trx_ctrl_msg,
		list);

	/* check if response matches command */
	if (!cmd_matches_rsp(tcm, &rsp)) {
		/* RSP from a retransmission, skip it */
		if (l1h->last_acked && cmd_matches_rsp(l1h->last_acked, &rsp)) {
			LOGPPHI(l1h->phy_inst, DTRX, LOGL_NOTICE, "Discarding duplicated RSP "
				"from old CMD '%s'\n", buf);
			return 0;
		}
		LOGPPHI(l1h->phy_inst, DTRX, (tcm->critical) ? LOGL_FATAL : LOGL_NOTICE,
			"Response message '%s' does not match command "
			"message 'CMD %s%s%s'\n",
			buf, tcm->cmd, tcm->params_len ? " ":"", tcm->params);

		/* We may get 'RSP ERR 1' for non-critical commands */
		if (tcm->critical)
			goto rsp_error;
	}

	rsp.cb = tcm->cb;

	/* Remove command from list, save it to last_acked and remove previous
	 * last_acked. Do it before calling callback to avoid user freeing tcm
	 * pointer if flushing/closing the iface. */
	llist_del(&tcm->list);
	talloc_free(l1h->last_acked);
	l1h->last_acked = tcm;

	/* check for response code */
	rc = trx_ctrl_rx_rsp(l1h, &rsp, tcm);
	if (rc == -EINVAL)
		goto rsp_error;

	/* re-schedule last cmd in rc seconds time */
	if (rc > 0) {
		if (!llist_empty(&l1h->trx_ctrl_list))
			osmo_timer_schedule(&l1h->trx_ctrl_timer, rc, 0);
		return 0;
	}

	trx_ctrl_send(l1h);

	return 0;

rsp_error:
	bts_shutdown(pinst->trx->bts, "TRX-CTRL-MSG: CRITICAL");
	/* keep tcm list, so process is stopped */
	return -EIO;
}


/*
 * TRX burst data socket
 */

/* Uplink TRXDv0 header length: TDMA TN + FN + RSSI + ToA256 */
#define TRX_UL_V0HDR_LEN	(1 + 4 + 1 + 2)
/* Uplink TRXDv1 header length: additional MTS + C/I */
#define TRX_UL_V1HDR_LEN	(TRX_UL_V0HDR_LEN + 1 + 2)
/* Uplink TRXDv2 header length: TDMA TN + TRXN + MTS + RSSI + ToA256 + C/I */
#define TRX_UL_V2HDR_LEN	(1 + 1 + 1 + 1 + 2 + 2)

/* Minimum Uplink TRXD header length for all PDU versions */
static const uint8_t trx_data_rx_hdr_len[] = {
	TRX_UL_V0HDR_LEN, /* TRXDv0 */
	TRX_UL_V1HDR_LEN, /* TRXDv1 */
	TRX_UL_V2HDR_LEN, /* TRXDv2 */
};

static const uint8_t trx_data_mod_val[] = {
	[TRX_MOD_T_GMSK]	= 0x00, /* .00xx... */
	[TRX_MOD_T_8PSK]	= 0x20, /* .010x... */
	[TRX_MOD_T_AQPSK]	= 0x60, /* .11xx... */
};

/* Header dissector for TRXDv0 (and part of TRXDv1) */
static inline void trx_data_handle_hdr_v0_part(struct trx_ul_burst_ind *bi,
					       const uint8_t *buf)
{
	bi->tn = buf[0] & 0b111;
	bi->fn = osmo_load32be(buf + 1);
	bi->rssi = -(int8_t)buf[5];
	bi->toa256 = (int16_t) osmo_load16be(buf + 6);
}

/* TRXD header dissector for version 0x00 */
static int trx_data_handle_hdr_v0(struct phy_instance *phy_inst,
				  struct trx_ul_burst_ind *bi,
				  const uint8_t *buf, size_t buf_len)
{
	/* Parse TRXDv0 specific header part */
	trx_data_handle_hdr_v0_part(bi, buf);
	buf_len -= TRX_UL_V0HDR_LEN;

	/* Guess modulation and burst length by the rest octets.
	 * NOTE: a legacy transceiver may append two garbage bytes. */
	switch (buf_len) {
	case EGPRS_BURST_LEN + 2:
	case EGPRS_BURST_LEN:
		bi->mod = TRX_MOD_T_8PSK;
		break;
	case GSM_BURST_LEN + 2:
	case GSM_BURST_LEN:
		bi->mod = TRX_MOD_T_GMSK;
		break;
	default:
		LOGPPHI(phy_inst, DTRX, LOGL_NOTICE,
			"Rx TRXD PDU with odd burst length %zu\n", buf_len);
		return -EINVAL;
	}

	return TRX_UL_V0HDR_LEN;
}

/* Parser for MTS (Modulation and Training Sequence) */
static inline int trx_data_parse_mts(struct phy_instance *phy_inst,
				     struct trx_ul_burst_ind *bi,
				     const uint8_t mts)
{
	if (mts & (1 << 7)) {
		bi->flags |= TRX_BI_F_NOPE_IND;
		return 0;
	}

	/* | 7 6 5 4 3 2 1 0 | Bitmask / description
	 * | . 0 0 X X . . . | GMSK, 4 TSC sets (0..3)
	 * | . 0 1 0 X . . . | 8-PSK, 2 TSC sets (0..1) */
	if ((mts >> 5) == 0x00) {
		bi->mod = TRX_MOD_T_GMSK;
		bi->tsc_set = (mts >> 3) & 0x03;
	} else if ((mts >> 4) == 0x02) {
		bi->mod = TRX_MOD_T_8PSK;
		bi->tsc_set = (mts >> 3) & 0x01;
	} else {
		LOGPPHI(phy_inst, DTRX, LOGL_ERROR,
			"Rx TRXD PDU with unknown or not supported "
			"modulation (MTS=0x%02x)\n", mts);
		return -ENOTSUP;
	}

	/* Training Sequence Code */
	bi->tsc = mts & 0x07;

	bi->flags |= (TRX_BI_F_MOD_TYPE | TRX_BI_F_TS_INFO);

	return 0;
}

/* TRXD header dissector for version 0x01 */
static int trx_data_handle_hdr_v1(struct phy_instance *phy_inst,
				  struct trx_ul_burst_ind *bi,
				  const uint8_t *buf, size_t buf_len)
{
	int rc;

	/* Parse TRXDv0 specific header part */
	trx_data_handle_hdr_v0_part(bi, buf);
	buf += TRX_UL_V0HDR_LEN;

	/* MTS (Modulation and Training Sequence) */
	rc = trx_data_parse_mts(phy_inst, bi, buf[0]);
	if (OSMO_UNLIKELY(rc < 0))
		return rc;

	/* C/I: Carrier-to-Interference ratio (in centiBels) */
	bi->ci_cb = (int16_t) osmo_load16be(buf + 1);
	bi->flags |= TRX_BI_F_CI_CB;

	return TRX_UL_V1HDR_LEN;
}

/* TRXD header dissector for version 0x01 */
static int trx_data_handle_pdu_v2(struct phy_instance *phy_inst,
				  struct trx_ul_burst_ind *bi,
				  const uint8_t *buf, size_t buf_len)
{
	int rc;

	/* TDMA timeslot number (other bits are RFU) */
	bi->tn = buf[0] & 0x07;

	if (buf[1] & (1 << 7)) /* BATCH.ind */
		bi->flags |= TRX_BI_F_BATCH_IND;
	if (buf[1] & (1 << 6)) /* VAMOS.ind */
		bi->flags |= TRX_BI_F_SHADOW_IND;

	/* TRX (RF channel) number */
	bi->trx_num = buf[1] & 0x3f;
	bi->flags |= TRX_BI_F_TRX_NUM;

	/* MTS (Modulation and Training Sequence) */
	rc = trx_data_parse_mts(phy_inst, bi, buf[2]);
	if (OSMO_UNLIKELY(rc < 0))
		return rc;

	bi->rssi = -(int8_t)buf[3];
	bi->toa256 = (int16_t) osmo_load16be(&buf[4]);
	bi->ci_cb = (int16_t) osmo_load16be(&buf[6]);
	bi->flags |= TRX_BI_F_CI_CB;

	/* TDMA frame number is absent in batched PDUs */
	if (bi->_num_pdus == 0) {
		if (OSMO_UNLIKELY(buf_len < sizeof(bi->fn) + TRX_UL_V2HDR_LEN)) {
			LOGPPHI(phy_inst, DTRX, LOGL_ERROR,
				"Rx malformed TRXDv2 PDU: not enough bytes "
				"to parse TDMA frame number\n");
			return -EINVAL;
		}

		bi->fn = osmo_load32be(buf + TRX_UL_V2HDR_LEN);
		return TRX_UL_V2HDR_LEN + sizeof(bi->fn);
	}

	return TRX_UL_V2HDR_LEN;
}

/* TRXD burst handler (version independent) */
static int trx_data_handle_burst(struct trx_ul_burst_ind *bi,
				 const uint8_t *buf, size_t buf_len)
{
	size_t i;

	/* NOPE.ind contains no burst */
	if (bi->flags & TRX_BI_F_NOPE_IND) {
		bi->burst_len = 0;
		return 0;
	}

	/* Modulation types defined in 3GPP TS 45.002 */
	static const size_t bl[] = {
		[TRX_MOD_T_GMSK] = 148, /* 1 bit per symbol */
		[TRX_MOD_T_8PSK] = 444, /* 3 bits per symbol */
	};

	bi->burst_len = bl[bi->mod];
	if (OSMO_UNLIKELY(buf_len < bi->burst_len))
		return -EINVAL;

	/* Convert unsigned soft-bits [254..0] to soft-bits [-127..127] */
	for (i = 0; i < bi->burst_len; i++) {
		if (buf[i] == 255)
			bi->burst[i] = -127;
		else
			bi->burst[i] = 127 - buf[i];
	}

	return 0;
}

static const char *trx_data_desc_msg(const struct trx_ul_burst_ind *bi)
{
	struct osmo_strbuf sb;
	static char buf[256];

	/* Modulation types defined in 3GPP TS 45.002 */
	static const char *mod_names[] = {
		[TRX_MOD_T_GMSK] = "GMSK",
		[TRX_MOD_T_8PSK] = "8-PSK",
	};

	/* Initialize the string buffer */
	sb = (struct osmo_strbuf) { .buf = buf, .len = sizeof(buf) };

	/* Common TDMA parameters */
	OSMO_STRBUF_PRINTF(sb, "tn=%u fn=%u", bi->tn, bi->fn);

	/* TRX (RF channel number) */
	if (bi->flags & TRX_BI_F_TRX_NUM)
		OSMO_STRBUF_PRINTF(sb, " trx_num=%u", bi->trx_num);

	/* RSSI and ToA256 */
	OSMO_STRBUF_PRINTF(sb, " rssi=%d toa256=%d", bi->rssi, bi->toa256);

	/* C/I: Carrier-to-Interference ratio (in centiBels) */
	if (bi->flags & TRX_BI_F_CI_CB)
		OSMO_STRBUF_PRINTF(sb, " C/I=%d cB", bi->ci_cb);

	/* Nothing else to print for NOPE.ind */
	if (bi->flags & TRX_BI_F_NOPE_IND)
		return buf;

	/* Modulation and TSC set */
	if (bi->flags & TRX_BI_F_MOD_TYPE)
		OSMO_STRBUF_PRINTF(sb, " mod=%s", mod_names[bi->mod]);

	/* Training Sequence Code */
	if (bi->flags & TRX_BI_F_TS_INFO)
		OSMO_STRBUF_PRINTF(sb, " set=%u tsc=%u", bi->tsc_set, bi->tsc);

	/* Burst length */
	OSMO_STRBUF_PRINTF(sb, " burst_len=%zu", bi->burst_len);

	return buf;
}

/* TRXD buffer used by Rx/Tx handlers */
static uint8_t trx_data_buf[TRXD_MSG_BUF_SIZE];

/* Parse TRXD message from transceiver, compose an UL burst indication. */
static int trx_data_read_cb(struct osmo_fd *ofd, unsigned int what)
{
	const uint8_t *buf = &trx_data_buf[0];
	struct trx_l1h *l1h = ofd->data;
	struct trx_ul_burst_ind bi;
	ssize_t hdr_len, buf_len;
	uint8_t pdu_ver;

	buf_len = recv(ofd->fd, trx_data_buf, sizeof(trx_data_buf), 0);
	if (OSMO_UNLIKELY(buf_len <= 0)) {
		strerror_r(errno, (char *) trx_data_buf, sizeof(trx_data_buf));
		LOGPPHI(l1h->phy_inst, DTRX, LOGL_ERROR,
			"recv() failed on TRXD with rc=%zd (%s)\n",
			buf_len, trx_data_buf);
		return buf_len;
	}

	/* Parse PDU version first */
	pdu_ver = buf[0] >> 4;

	/* Make sure that PDU version matches our expectations */
	if (OSMO_UNLIKELY(pdu_ver != l1h->config.trxd_pdu_ver_use)) {
		LOGPPHI(l1h->phy_inst, DTRX, LOGL_ERROR,
			"Rx TRXD PDU with unexpected version %u (expected %u)\n",
			pdu_ver, l1h->config.trxd_pdu_ver_use);
		return -EIO;
	}

	/* We're about to parse the first PDU */
	bi._num_pdus = 0;

	/* Starting from TRXDv2, there can be batched PDUs */
	do {
		/* (Re)initialize the flags */
		bi.flags = 0x00;

		/* Make sure that we have enough bytes to parse the header */
		if (OSMO_UNLIKELY(buf_len < trx_data_rx_hdr_len[pdu_ver])) {
			LOGPPHI(l1h->phy_inst, DTRX, LOGL_ERROR,
				"Rx malformed TRXDv%u PDU: len=%zd < expected %u\n",
				pdu_ver, buf_len, trx_data_rx_hdr_len[pdu_ver]);
			return -EINVAL;
		}

		/* Parse header depending on the PDU version */
		switch (pdu_ver) {
		case 0: /* TRXDv0 */
			hdr_len = trx_data_handle_hdr_v0(l1h->phy_inst, &bi, buf, buf_len);
			break;
		case 1: /* TRXDv1 */
			hdr_len = trx_data_handle_hdr_v1(l1h->phy_inst, &bi, buf, buf_len);
			break;
		case 2: /* TRXDv2 */
			hdr_len = trx_data_handle_pdu_v2(l1h->phy_inst, &bi, buf, buf_len);
			break;
		default:
			/* Shall not happen */
			OSMO_ASSERT(0);
		}

		/* Header parsing error */
		if (OSMO_UNLIKELY(hdr_len < 0))
			return hdr_len;

		if (OSMO_UNLIKELY(bi.fn >= GSM_TDMA_HYPERFRAME)) {
			LOGPPHI(l1h->phy_inst, DTRX, LOGL_ERROR,
				"Rx malformed TRXDv%u PDU: illegal TDMA fn=%u\n",
				pdu_ver, bi.fn);
			return -EINVAL;
		}

		/* We're done with the header now */
		buf_len -= hdr_len;
		buf += hdr_len;

		/* Calculate burst length and parse it (if present) */
		if (OSMO_UNLIKELY(trx_data_handle_burst(&bi, buf, buf_len) != 0)) {
			LOGPPHI(l1h->phy_inst, DTRX, LOGL_ERROR,
				"Rx malformed TRXDv%u PDU: odd burst length=%zd\n",
				pdu_ver, buf_len);
			return -EINVAL;
		}

		/* We're done with the burst bits now */
		buf_len -= bi.burst_len;
		buf += bi.burst_len;

		/* Print header & burst info */
		LOGPPHI(l1h->phy_inst, DTRX, LOGL_DEBUG, "Rx %s (pdu_ver=%u): %s\n",
			(bi.flags & TRX_BI_F_NOPE_IND) ? "NOPE.ind" : "UL burst",
			pdu_ver, trx_data_desc_msg(&bi));

		/* Number of processed PDUs */
		bi._num_pdus++;

		/* feed received burst into scheduler code */
		TRACE(OSMO_BTS_TRX_UL_DATA_START(l1h->phy_inst->trx->nr, bi.tn, bi.fn));
		trx_sched_route_burst_ind(l1h->phy_inst->trx, &bi);
		TRACE(OSMO_BTS_TRX_UL_DATA_DONE(l1h->phy_inst->trx->nr, bi.tn, bi.fn));
	} while (bi.flags & TRX_BI_F_BATCH_IND);

	return 0;
}

/*! Send burst data for given FN/timeslot to TRX
 *  \param[inout] l1h TRX Layer1 handle referring to TX
 *  \param[in] br Downlink burst request structure
 *  \returns 0 on success; negative on error */
int trx_if_send_burst(struct trx_l1h *l1h, const struct trx_dl_burst_req *br)
{
	uint8_t pdu_ver = l1h->config.trxd_pdu_ver_use;
	static uint8_t *buf = &trx_data_buf[0];
	static uint8_t *last_pdu = NULL;
	static unsigned int pdu_num = 0;
	ssize_t snd_len, buf_len;

	/* Make sure that the PHY is powered on */
	if (OSMO_UNLIKELY(!trx_if_powered(l1h))) {
		LOGPPHI(l1h->phy_inst, DTRX, LOGL_ERROR,
			"Ignoring Tx data, transceiver is powered off\n");
		return -ENODEV;
	}

	/* Burst batching breaker */
	if (br == NULL) {
		if (pdu_num > 0)
			goto sendall;
		return -ENOMSG;
	}

	/* Pointer to the last encoded PDU */
	last_pdu = &buf[0];

	switch (pdu_ver) {
	/* Both versions have the same PDU format */
	case 0: /* TRXDv0 */
	case 1: /* TRXDv1 */
		buf[0] = ((pdu_ver & 0x0f) << 4) | br->tn;
		osmo_store32be(br->fn, buf + 1);
		buf[5] = br->att;
		buf += 6;
		break;
	case 2: /* TRXDv2 */
		buf[0] = br->tn;
		/* BATCH.ind will be unset in the last PDU */
		buf[1] = (br->trx_num & 0x3f) | (1 << 7);
		buf[2] = trx_data_mod_val[br->mod]
		       | (br->tsc_set << 3)
		       | (br->tsc & 0x07);
		buf[3] = br->att;
		buf[4] = (uint8_t) br->scpir;
		buf[5] = buf[6] = buf[7] = 0x00; /* Spare */
		/* Some fields are not present in batched PDUs */
		if (pdu_num == 0) {
			buf[0] |= (pdu_ver & 0x0f) << 4;
			osmo_store32be(br->fn, buf + 8);
			buf += 4;
		}
		buf += 8;
		break;
	default:
		/* Shall not happen */
		OSMO_ASSERT(0);
	}

	/* copy ubits {0,1} */
	memcpy(buf, br->burst, br->burst_len);
	buf += br->burst_len;

	/* One more PDU in the buffer */
	pdu_num++;

	/* TRXDv2: wait for the batching breaker */
	if (pdu_ver >= 2)
		return 0;

sendall:
	LOGPPHI(l1h->phy_inst, DTRX, LOGL_DEBUG,
		"Tx TRXDv%u datagram with %u PDU(s)\n",
		pdu_ver, pdu_num);

	/* TRXDv2: unset BATCH.ind in the last PDU */
	if (pdu_ver >= 2)
		last_pdu[1] &= ~(1 << 7);

	buf_len = buf - &trx_data_buf[0];
	buf = &trx_data_buf[0];
	pdu_num = 0;

	snd_len = send(l1h->trx_ofd_data.fd, trx_data_buf, buf_len, 0);
	if (OSMO_UNLIKELY(snd_len <= 0)) {
		strerror_r(errno, (char *) trx_data_buf, sizeof(trx_data_buf));
		LOGPPHI(l1h->phy_inst, DTRX, LOGL_ERROR,
			"send() failed on TRXD with rc=%zd (%s)\n",
			snd_len, trx_data_buf);
		return -2;
	}

	return 0;
}


/*
 * open/close
 */

/*! flush (delete) all pending control messages */
void trx_if_flush(struct trx_l1h *l1h)
{
	struct trx_ctrl_msg *tcm;

	/* free ctrl message list */
	while (!llist_empty(&l1h->trx_ctrl_list)) {
		tcm = llist_entry(l1h->trx_ctrl_list.next, struct trx_ctrl_msg,
			list);
		llist_del(&tcm->list);
		talloc_free(tcm);
	}
	talloc_free(l1h->last_acked);
	l1h->last_acked = NULL;

	/* Tx queue is now empty, so there's no point in keeping the retrans timer armed: */
	if (osmo_timer_pending(&l1h->trx_ctrl_timer))
		osmo_timer_del(&l1h->trx_ctrl_timer);
}

/*! close the TRX for given handle (data + control socket) */
void trx_if_close(struct trx_l1h *l1h)
{
	LOGPPHI(l1h->phy_inst, DTRX, LOGL_NOTICE, "Closing TRXC/TRXD connections\n");

	trx_if_flush(l1h);

	/* close sockets */
	trx_udp_close(&l1h->trx_ofd_ctrl);
	trx_udp_close(&l1h->trx_ofd_data);
}

/*! compute UDP port number used for TRX protocol */
static uint16_t compute_port(struct phy_instance *pinst, int remote, int is_data)
{
	struct phy_link *plink = pinst->phy_link;
	uint16_t inc = 1;

	if (is_data)
		inc = 2;

	if (remote)
		return plink->u.osmotrx.base_port_remote + (pinst->num << 1) + inc;
	else
		return plink->u.osmotrx.base_port_local + (pinst->num << 1) + inc;
}

/*! open a TRX interface. creates control + data sockets */
static int trx_if_open(struct trx_l1h *l1h)
{
	struct phy_instance *pinst = l1h->phy_inst;
	struct phy_link *plink = pinst->phy_link;
	int rc;

	LOGPPHI(pinst, DTRX, LOGL_NOTICE, "Opening TRXC/TRXD connections\n");

	/* open sockets */
	rc = trx_udp_open(l1h, &l1h->trx_ofd_ctrl,
			  plink->u.osmotrx.local_ip,
			  compute_port(pinst, 0, 0),
			  plink->u.osmotrx.remote_ip,
			  compute_port(pinst, 1, 0), trx_ctrl_read_cb);
	if (rc < 0)
		return rc;
	rc = trx_udp_open(l1h, &l1h->trx_ofd_data,
			  plink->u.osmotrx.local_ip,
			  compute_port(pinst, 0, 1),
			  plink->u.osmotrx.remote_ip,
			  compute_port(pinst, 1, 1), trx_data_read_cb);
	if (rc < 0)
		return rc;

	return 0;
}

/*! close the control + burst data sockets for one phy_instance */
static void trx_phy_inst_close(struct phy_instance *pinst)
{
	struct trx_l1h *l1h = pinst->u.osmotrx.hdl;

	trx_if_close(l1h);
	if (pinst->trx)
		trx_sched_clean(pinst->trx);
}

/*! open the control + burst data sockets for one phy_instance */
static int trx_phy_inst_open(struct phy_instance *pinst)
{
	struct trx_l1h *l1h;
	int rc;

	l1h = pinst->u.osmotrx.hdl;
	if (!l1h)
		return -EINVAL;

	/* PHY instance may be not associated with a TRX instance */
	if (pinst->trx != NULL)
		trx_sched_init(pinst->trx);

	rc = trx_if_open(l1h);
	if (rc < 0) {
		LOGPPHI(l1h->phy_inst, DL1C, LOGL_FATAL, "Cannot open TRX interface\n");
		trx_phy_inst_close(pinst);
		return -EIO;
	}

	return 0;
}

/*! open the PHY link using TRX protocol */
int bts_model_phy_link_open(struct phy_link *plink)
{
	struct phy_instance *pinst;
	int rc;

	phy_link_state_set(plink, PHY_LINK_CONNECTING);

	/* open the shared/common clock socket */
	rc = trx_udp_open(plink, &plink->u.osmotrx.trx_ofd_clk,
			  plink->u.osmotrx.local_ip,
			  plink->u.osmotrx.base_port_local,
			  plink->u.osmotrx.remote_ip,
			  plink->u.osmotrx.base_port_remote,
			  trx_clk_read_cb);
	if (rc < 0) {
		phy_link_state_set(plink, PHY_LINK_SHUTDOWN);
		return -1;
	}

	/* open the individual instances with their ctrl+data sockets */
	llist_for_each_entry(pinst, &plink->instances, list) {
		struct trx_l1h *l1h = pinst->u.osmotrx.hdl;
		if (trx_phy_inst_open(pinst) < 0)
			goto cleanup;
		osmo_fsm_inst_dispatch(l1h->provision_fi, TRX_PROV_EV_OPEN, NULL);
	}

	return 0;

cleanup:
	phy_link_state_set(plink, PHY_LINK_SHUTDOWN);
	llist_for_each_entry(pinst, &plink->instances, list) {
		if (pinst->u.osmotrx.hdl) {
			trx_if_close(pinst->u.osmotrx.hdl);
			pinst->u.osmotrx.hdl = NULL;
		}
	}
	trx_udp_close(&plink->u.osmotrx.trx_ofd_clk);
	return -1;
}

/*! close the PHY link using TRX protocol */
int bts_model_phy_link_close(struct phy_link *plink)
{
	bool clock_stopped = false;
	struct phy_instance *pinst;
	llist_for_each_entry(pinst, &plink->instances, list) {
		if (!clock_stopped) {
			clock_stopped = true;
			trx_sched_clock_stopped(pinst->trx->bts);
		}
		trx_phy_inst_close(pinst);
	}
	trx_udp_close(&plink->u.osmotrx.trx_ofd_clk);
	phy_link_state_set(plink, PHY_LINK_SHUTDOWN);
	return 0;
}

/*! determine if the TRX for given handle is powered up */
int trx_if_powered(struct trx_l1h *l1h)
{
	struct phy_instance *pinst = l1h->phy_inst;
	struct phy_link *plink = pinst->phy_link;
	return plink->u.osmotrx.powered;
}
