/* Main program for Sysmocom BTS */

/* (C) 2011-2013 by Harald Welte <laforge@gnumonks.org>
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
#include <stdlib.h>
#include <errno.h>
#include <getopt.h>
#include <limits.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sched.h>

#include <netinet/in.h>
#include <arpa/inet.h>

#include <osmocom/core/talloc.h>
#include <osmocom/core/application.h>
#include <osmocom/core/socket.h>
#include <osmocom/vty/telnet_interface.h>
#include <osmocom/vty/logging.h>
#include <osmocom/gsm/protocol/ipaccess.h>
#include <osmocom/gsm/abis_nm.h>

#include <osmo-bts/gsm_data.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/abis.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/vty.h>
#include <osmo-bts/bts_model.h>
#include <osmo-bts/pcu_if.h>
#include <osmo-bts/oml.h>

#include "misc/sysmobts_mgr.h"

#define SYSMOBTS_RF_LOCK_PATH	"/var/lock/bts_rf_lock"

#include "utils.h"
#include "eeprom.h"
#include "l1_if.h"
#include "hw_misc.h"

/* FIXME: read from real hardware */
const uint8_t abis_mac[6] = { 0,1,2,3,4,5 };
int pcu_direct = 0;

static const char *config_file = "osmo-bts.cfg";
static int daemonize = 0;
static unsigned int dsp_trace = 0x71c00020;
static int rt_prio = -1;

int bts_model_init(struct gsm_bts *bts)
{
	struct femtol1_hdl *fl1h;
	int rc;

	fl1h = l1if_open(bts->c0);
	if (!fl1h) {
		LOGP(DL1C, LOGL_FATAL, "Cannot open L1 Interface\n");
		return -EIO;
	}
	fl1h->dsp_trace_f = dsp_trace;

	bts->c0->role_bts.l1h = fl1h;

	rc = sysmobts_get_nominal_power(bts->c0);
	if (rc < 0) {
		LOGP(DL1C, LOGL_NOTICE, "Cannot determine nominal "
		     "transmit power. Assuming 23dBm.\n");
		rc = 23;
	}
	bts->c0->nominal_power = rc;

	bts_model_vty_init(bts);

	return 0;
}

int bts_model_oml_estab(struct gsm_bts *bts)
{
	struct femtol1_hdl *fl1h = bts->c0->role_bts.l1h;

	l1if_reset(fl1h);

	return 0;
}

/* Set the clock calibration to the value
 * read from the eeprom.
 */
void clk_cal_use_eeprom(struct gsm_bts *bts)
{
	int rc;
	struct femtol1_hdl *hdl;
	eeprom_RfClockCal_t rf_clk;

	hdl = bts->c0->role_bts.l1h;

	if (!hdl || !hdl->clk_use_eeprom)
		return;

	rc = eeprom_ReadRfClockCal(&rf_clk);
	if (rc != EEPROM_SUCCESS) {
		LOGP(DL1C, LOGL_ERROR, "Failed to read from EEPROM.\n");
		return;
	}

	hdl->clk_cal = rf_clk.iClkCor;
	LOGP(DL1C, LOGL_NOTICE,
		"Read clock calibration(%d) from EEPROM.\n", hdl->clk_cal);
}

void bts_update_status(enum bts_global_status which, int on)
{
	static uint64_t states = 0;
	uint64_t old_states = states;
	int led_rf_active_on;

	if (on)
		states |= (1ULL << which);
	else
		states &= ~(1ULL << which);

	led_rf_active_on =
		(states & (1ULL << BTS_STATUS_RF_ACTIVE)) &&
		!(states & (1ULL << BTS_STATUS_RF_MUTE));

	LOGP(DL1C, LOGL_INFO,
	     "Set global status #%d to %d (%04llx -> %04llx), LEDs: ACT %d\n",
	     which, on,
	     (long long)old_states, (long long)states,
	     led_rf_active_on);

	sysmobts_led_set(LED_RF_ACTIVE, led_rf_active_on);
}

static void print_help()
{
	printf( "Some useful options:\n"
		"  -h	--help		this text\n"
		"  -d	--debug MASK	Enable debugging (e.g. -d DRSL:DOML:DLAPDM)\n"
		"  -D	--daemonize	For the process into a background daemon\n"
		"  -c	--config-file 	Specify the filename of the config file\n"
		"  -s	--disable-color	Don't use colors in stderr log output\n"
		"  -T	--timestamp	Prefix every log line with a timestamp\n"
		"  -V	--version	Print version information and exit\n"
		"  -e 	--log-level	Set a global log-level\n"
		"  -p	--dsp-trace	Set DSP trace flags\n"
		"  -r	--realtime PRIO	Use SCHED_RR with the specified priority\n"
		"  -w	--hw-version	Print the targeted HW Version\n"
		"  -M	--pcu-direct	Force PCU to access message queue for "
			"PDCH dchannel directly\n"
		);
}

static void print_hwversion()
{
#ifdef HW_SYSMOBTS_V1
	printf("sysmobts was compiled for hw version 1.\n");
#else
	printf("sysmobts was compiled for hw version 2.\n");
#endif
}

/* FIXME: finally get some option parsing code into libosmocore */
static void handle_options(int argc, char **argv)
{
	while (1) {
		int option_idx = 0, c;
		static const struct option long_options[] = {
			/* FIXME: all those are generic Osmocom app options */
			{ "help", 0, 0, 'h' },
			{ "debug", 1, 0, 'd' },
			{ "daemonize", 0, 0, 'D' },
			{ "config-file", 1, 0, 'c' },
			{ "disable-color", 0, 0, 's' },
			{ "timestamp", 0, 0, 'T' },
			{ "version", 0, 0, 'V' },
			{ "log-level", 1, 0, 'e' },
			{ "dsp-trace", 1, 0, 'p' },
			{ "hw-version", 0, 0, 'w' },
			{ "pcu-direct", 0, 0, 'M' },
			{ "realtime", 1, 0, 'r' },
			{ 0, 0, 0, 0 }
		};

		c = getopt_long(argc, argv, "hc:d:Dc:sTVe:p:w:Mr:",
				long_options, &option_idx);
		if (c == -1)
			break;

		switch (c) {
		case 'h':
			print_help();
			exit(0);
			break;
		case 's':
			log_set_use_color(osmo_stderr_target, 0);
			break;
		case 'd':
			log_parse_category_mask(osmo_stderr_target, optarg);
			break;
		case 'D':
			daemonize = 1;
			break;
		case 'c':
			config_file = strdup(optarg);
			break;
		case 'T':
			log_set_print_timestamp(osmo_stderr_target, 1);
			break;
		case 'M':
			pcu_direct = 1;
			break;
		case 'V':
			print_version(1);
			exit(0);
			break;
		case 'e':
			log_set_log_level(osmo_stderr_target, atoi(optarg));
			break;
		case 'p':
			dsp_trace = strtoul(optarg, NULL, 16);
			break;
		case 'w':
			print_hwversion();
			exit(0);
			break;
		case 'r':
			rt_prio = atoi(optarg);
			break;
		default:
			break;
		}
	}
}

static struct gsm_bts *bts;

static void signal_handler(int signal)
{
	fprintf(stderr, "signal %u received\n", signal);

	switch (signal) {
	case SIGINT:
		//osmo_signal_dispatch(SS_GLOBAL, S_GLOBAL_SHUTDOWN, NULL);
		bts_shutdown(bts, "SIGINT");
		unlink(SOCKET_PATH);
		break;
	case SIGABRT:
	case SIGUSR1:
	case SIGUSR2:
		talloc_report_full(tall_bts_ctx, stderr);
		break;
	default:
		break;
	}
}

static int write_pid_file(char *procname)
{
	FILE *outf;
	char tmp[PATH_MAX+1];

	snprintf(tmp, sizeof(tmp)-1, "/var/run/%s.pid", procname);
	tmp[PATH_MAX-1] = '\0';

	outf = fopen(tmp, "w");
	if (!outf)
		return -1;

	fprintf(outf, "%d\n", getpid());

	fclose(outf);

	return 0;
}

#define oml_tlv_parse(dec, buf, len)	\
	tlv_parse(dec, &abis_nm_att_tlvdef, buf, len, 0, 0)

static int send_oml_fom_ack_nack(int fd_unix, struct msgb *old_msg,
				 uint8_t cause, int is_manuf)
{
	struct abis_om_hdr *old_om = msgb_l2(old_msg);
	struct abis_om_fom_hdr *old_foh = msgb_l3(old_msg);
	struct msgb *msg;
	struct abis_om_fom_hdr *foh;
	struct abis_om_hdr *om;
	int rc;

	msg = oml_msgb_alloc();
	if (!msg)
		return -ENOMEM;

	msg->l3h = msgb_push(msg, sizeof(*foh));
	foh = (struct abis_om_fom_hdr *) msg->l3h;
	memcpy(foh, old_foh, sizeof(*foh));

	if (is_manuf)
		add_manufacturer_id_label(msg, OSMOCOM_MANUF_ID);

	msg->l2h = msgb_push(msg, sizeof(*om));
	om = (struct abis_om_hdr *) msg->l2h;
	memcpy(om, old_om, sizeof(*om));

	/* alter message type */
	if (cause) {
		LOGP(DOML, LOGL_ERROR, "Sending FOM NACK with cause %s.\n",
		     abis_nm_nack_cause_name(cause));
		foh->msg_type += 2; /* nack */
		msgb_tv_put(msg, NM_ATT_NACK_CAUSES, cause);
	} else {
		LOGP(DOML, LOGL_DEBUG, "Sending FOM ACK.\n");
		foh->msg_type++; /* ack */
	}

	prepend_oml_ipa_header(msg);

	rc = send(fd_unix, msg->data, msg->len, 0);
	if (rc < 0 || rc != msg->len) {
		LOGP(DTEMP, LOGL_ERROR,
		     "send error %s during ACK/NACK message send\n",
		     strerror(errno));
		close(fd_unix);
		msgb_free(msg);
		return -1;
	}

	msgb_free(msg);
	return rc;
}

static void update_transmiter_power(struct gsm_bts_trx *trx)
{
	struct femtol1_hdl *fl1h = trx_femtol1_hdl(trx);

	if (fl1h->hLayer1)
		l1if_set_txpower(fl1h, sysmobts_get_power_trx(trx));
}

static int take_reduce_power(struct msgb *msg)
{
	int recv_reduce_power;
	struct tlv_parsed tlv_out;
	struct gsm_bts_trx *trx = bts->c0;
	int rc, abis_oml_hdr_len;

	abis_oml_hdr_len = sizeof(struct abis_om_hdr);
	abis_oml_hdr_len += sizeof(struct abis_om_fom_hdr);
	abis_oml_hdr_len += sizeof(osmocom_magic) + 1;

	rc = oml_tlv_parse(&tlv_out, msg->data + abis_oml_hdr_len,
			   msg->len - abis_oml_hdr_len);

	if (rc < 0) {
		msgb_free(msg);
		return -1;
	}

	if (TLVP_PRESENT(&tlv_out, NM_ATT_O_REDUCEPOWER))
		recv_reduce_power = *TLVP_VAL(&tlv_out,
					      NM_ATT_O_REDUCEPOWER);
	else
		return -1;

	trx->power_reduce = recv_reduce_power;

	update_transmiter_power(trx);

	return 0;
}

static int read_sock(struct osmo_fd *fd, unsigned int what)
{
	struct msgb *msg;
	struct gsm_abis_mo *mo;
	struct abis_om_fom_hdr *fom;
	int rc;

	msg = oml_msgb_alloc();
	if (msg == NULL) {
		LOGP(DL1C, LOGL_ERROR,
		     "Failed to allocate oml msgb.\n");
		return -1;
	}

	rc = recv(fd->fd, msg->tail, msg->data_len, 0);
	if (rc <= 0) {
		close(fd->fd);
		osmo_fd_unregister(fd);
		fd->fd = -1;
		goto err;
	}

	msgb_put(msg, rc);

	if (check_oml_msg(msg) < 0) {
		LOGP(DL1C, LOGL_ERROR, "Malformed receive message\n");
		goto err;
	}

	mo = &bts->mo;
	msg->trx = mo->bts->c0;
	fom = (struct abis_om_fom_hdr *) msg->l3h;

	switch (fom->msg_type) {
	case NM_MT_SET_RADIO_ATTR:
		rc = take_reduce_power(msg);
		if (rc < 0) {
			rc = send_oml_fom_ack_nack(fd->fd, msg,
						   NM_NACK_INCORR_STRUCT, 1);
		} else {
			rc = send_oml_fom_ack_nack(fd->fd, msg, 0, 1);
		}
		msgb_free(msg);
		break;
	case NM_MT_FAILURE_EVENT_REP:
		rc = abis_oml_sendmsg(msg);
		break;
	default:
		LOGP(DL1C, LOGL_ERROR, "Unknown Fom message type %d\n",
		     fom->msg_type);
		goto err;
	}

	if (rc < 0)
		goto err;

	return rc;
err:
	msgb_free(msg);
	return -1;
}

static int accept_unix_sock(struct osmo_fd *fd, unsigned int what)
{
	int sfd = fd->fd, cl;
	struct osmo_fd *read_fd = (struct osmo_fd *)fd->data;

	if (read_fd->fd > -1) {
		close(read_fd->fd);
		osmo_fd_unregister(read_fd);
		read_fd->fd = -1;
	}

	cl = accept(sfd, NULL, NULL);
	if (cl < 0) {
		LOGP(DL1C, LOGL_ERROR, "Failed to accept. errno: %s.\n",
		     strerror(errno));
		return -1;
	}

	read_fd->fd = cl;
	if (osmo_fd_register(read_fd) != 0) {
		LOGP(DL1C, LOGL_ERROR, "Register the read file desc.\n");
		close(cl);
		read_fd->fd = -1;
		return -1;
	}

	return 0;
}

static int oml_sock_unix_init(struct osmo_fd *accept, struct osmo_fd *read)
{
	int rc;

	accept->cb = accept_unix_sock;
	read->cb = read_sock;
	read->when = BSC_FD_READ;
	read->fd = -1;
	accept->data = read;

	rc = osmo_sock_unix_init_ofd(accept, SOCK_SEQPACKET, 0, SOCKET_PATH,
				     OSMO_SOCK_F_BIND | OSMO_SOCK_F_NONBLOCK);
	return rc;
}

int main(int argc, char **argv)
{
	struct stat st;
	struct sched_param param;
	struct gsm_bts_role_bts *btsb;
	struct e1inp_line *line;
	void *tall_msgb_ctx;
	struct osmo_fd accept_fd, read_fd;
	int rc;

	tall_bts_ctx = talloc_named_const(NULL, 1, "OsmoBTS context");
	tall_msgb_ctx = talloc_named_const(tall_bts_ctx, 1, "msgb");
	msgb_set_talloc_ctx(tall_msgb_ctx);

	bts_log_init(NULL);

	vty_init(&bts_vty_info);
	bts_vty_init(&bts_log_info);

	handle_options(argc, argv);

	/* enable realtime priority for us */
	if (rt_prio != -1) {
		memset(&param, 0, sizeof(param));
		param.sched_priority = rt_prio;
		rc = sched_setscheduler(getpid(), SCHED_RR, &param);
		if (rc != 0) {
			fprintf(stderr, "Setting SCHED_RR priority(%d) failed: %s\n",
				param.sched_priority, strerror(errno));
			exit(1);
		}
	}

	bts = gsm_bts_alloc(tall_bts_ctx);
	if (bts_init(bts) < 0) {
		fprintf(stderr, "unable to open bts\n");
		exit(1);
	}
	btsb = bts_role_bts(bts);
	btsb->support.ciphers = CIPHER_A5(1) | CIPHER_A5(2) | CIPHER_A5(3);

	abis_init(bts);

	rc = vty_read_config_file(config_file, NULL);
	if (rc < 0) {
		fprintf(stderr, "Failed to parse the config file: '%s'\n",
			config_file);
		exit(1);
	}

	clk_cal_use_eeprom(bts);

	if (stat(SYSMOBTS_RF_LOCK_PATH, &st) == 0) {
		LOGP(DL1C, LOGL_NOTICE, "Not starting BTS due to RF_LOCK file present\n");
		exit(23);
	}
	write_pid_file("osmo-bts");

	rc = telnet_init(tall_bts_ctx, NULL, 4241);
	if (rc < 0) {
		fprintf(stderr, "Error initializing telnet\n");
		exit(1);
	}

	if (pcu_sock_init()) {
		fprintf(stderr, "PCU L1 socket failed\n");
		exit(1);
	}

	signal(SIGINT, &signal_handler);
	//signal(SIGABRT, &signal_handler);
	signal(SIGUSR1, &signal_handler);
	signal(SIGUSR2, &signal_handler);
	osmo_init_ignore_signals();

	if (!btsb->bsc_oml_host) {
		fprintf(stderr, "Cannot start BTS without knowing BSC OML IP\n");
		exit(1);
	}

	line = abis_open(bts, btsb->bsc_oml_host, "sysmoBTS");
	if (!line) {
		fprintf(stderr, "unable to connect to BSC\n");
		exit(1);
	}

	rc = oml_sock_unix_init(&accept_fd, &read_fd);
	if (rc < 0) {
		perror("Error creating socket domain creation");
		exit(1);
	}

	if (daemonize) {
		rc = osmo_daemonize();
		if (rc < 0) {
			perror("Error during daemonize");
			exit(1);
		}
	}

	while (1) {
		log_reset_context();
		osmo_select_main(0);
	}
}
