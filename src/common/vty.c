/* OsmoBTS VTY interface */

/* (C) 2011 by Harald Welte <laforge@gnumonks.org>
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

#include "btsconfig.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <ctype.h>

#include <osmocom/core/talloc.h>
#include <osmocom/gsm/abis_nm.h>
#include <osmocom/vty/vty.h>
#include <osmocom/vty/command.h>
#include <osmocom/vty/logging.h>
#include <osmocom/vty/misc.h>
#include <osmocom/core/gsmtap.h>

#include <osmocom/trau/osmo_ortp.h>


#include <osmo-bts/logging.h>
#include <osmo-bts/gsm_data.h>
#include <osmo-bts/abis.h>
#include <osmo-bts/bts.h>
#include <osmo-bts/rsl.h>
#include <osmo-bts/oml.h>
#include <osmo-bts/signal.h>
#include <osmo-bts/bts_model.h>
#include <osmo-bts/measurement.h>
#include <osmo-bts/vty.h>
#include <osmo-bts/l1sap.h>

enum node_type bts_vty_go_parent(struct vty *vty)
{
	switch (vty->node) {
	case TRX_NODE:
		vty->node = BTS_NODE;
		{
			struct gsm_bts_trx *trx = vty->index;
			vty->index = trx->bts;
		}
		break;
	default:
		vty->node = CONFIG_NODE;
	}
	return vty->node;
}

int bts_vty_is_config_node(struct vty *vty, int node)
{
	switch (node) {
	case TRX_NODE:
	case BTS_NODE:
		return 1;
	default:
		return 0;
	}
}

gDEFUN(ournode_exit, ournode_exit_cmd, "exit",
	"Exit current node, go down to provious node")
{
	switch (vty->node) {
	case TRX_NODE:
		vty->node = BTS_NODE;
		{
			struct gsm_bts_trx *trx = vty->index;
			vty->index = trx->bts;
		}
		break;
	default:
		break;
	}
	return CMD_SUCCESS;
}

gDEFUN(ournode_end, ournode_end_cmd, "end",
	"End current mode and change to enable mode")
{
	switch (vty->node) {
	default:
		vty_config_unlock(vty);
		vty->node = ENABLE_NODE;
		vty->index = NULL;
		vty->index_sub = NULL;
		break;
	}
	return CMD_SUCCESS;
}

static const char osmobts_copyright[] =
	"Copyright (C) 2010, 2011 by Harald Welte, Andreas Eversberg and On-Waves\r\n"
	"License AGPLv3+: GNU AGPL version 3 or later <http://gnu.org/licenses/agpl-3.0.html>\r\n"
	"This is free software: you are free to change and redistribute it.\r\n"
	 "There is NO WARRANTY, to the extent permitted by law.\r\n";

struct vty_app_info bts_vty_info = {
	.name		= "OsmoBTS",
	.version	= PACKAGE_VERSION,
	.copyright	= osmobts_copyright,
	.go_parent_cb	= bts_vty_go_parent,
	.is_config_node	= bts_vty_is_config_node,
};

extern struct gsm_network bts_gsmnet;

struct gsm_network *gsmnet_from_vty(struct vty *v)
{
	return &bts_gsmnet;
}

struct gsm_bts *gsm_bts_num(struct gsm_network *net, int num)
{
	struct gsm_bts *bts;

	if (num >= net->num_bts)
		return NULL;

	llist_for_each_entry(bts, &net->bts_list, list) {
		if (bts->nr == num)
			return bts;
	}

	return NULL;
}

static struct cmd_node bts_node = {
	BTS_NODE,
	"%s(bts)#",
	1,
};

static struct cmd_node trx_node = {
	TRX_NODE,
	"%s(trx)#",
	1,
};

static char cfg_bts_trx_cmd_string[16];
DEFUN(cfg_bts_trx, cfg_bts_trx_cmd,
	cfg_bts_trx_cmd_string,
	"Select a TRX to configure\n" "TRX number\n")
{
	int trx_nr = atoi(argv[0]);
	struct gsm_bts *bts = vty->index;
	struct gsm_bts_trx *trx;

	trx = gsm_bts_trx_num(bts, trx_nr);
	if (!trx) {
		vty_out(vty, "Unknown TRX %u%s", trx_nr, VTY_NEWLINE);
		return CMD_WARNING;
	}

	vty->index = trx;
	vty->index_sub = &trx->description;
	vty->node = TRX_NODE;

	return CMD_SUCCESS;
}

/* FIXME: move to libosmocore ? */
static char buf_casecnvt[256];
char *osmo_str_tolower(const char *in)
{
	int len, i;

	if (!in)
		return NULL;

	len = strlen(in);
	if (len > sizeof(buf_casecnvt))
		len = sizeof(buf_casecnvt);

	for (i = 0; i < len; i++) {
		buf_casecnvt[i] = tolower(in[i]);
		if (in[i] == '\0')
			break;
	}
	if (i < sizeof(buf_casecnvt))
		buf_casecnvt[i] = '\0';

	/* just to make sure we're always zero-terminated */
	buf_casecnvt[sizeof(buf_casecnvt)-1] = '\0';

	return buf_casecnvt;
}

static void config_write_bts_single(struct vty *vty, struct gsm_bts *bts)
{
	struct gsm_bts_role_bts *btsb = bts_role_bts(bts);
	struct gsm_bts_trx *trx;
	int i;

	vty_out(vty, "bts %u%s", bts->nr, VTY_NEWLINE);
	if (bts->description)
		vty_out(vty, " description %s%s", bts->description, VTY_NEWLINE);
	vty_out(vty, " band %s%s", gsm_band_name(bts->band), VTY_NEWLINE);
	vty_out(vty, " ipa unit-id %u %u%s",
		bts->ip_access.site_id, bts->ip_access.bts_id, VTY_NEWLINE);
	vty_out(vty, " oml remote-ip %s%s", btsb->bsc_oml_host, VTY_NEWLINE);
	vty_out(vty, " rtp jitter-buffer %u%s", btsb->rtp_jitter_buf_ms,
		VTY_NEWLINE);
	vty_out(vty, " paging queue-size %u%s", paging_get_queue_max(btsb->paging_state),
		VTY_NEWLINE);
	vty_out(vty, " paging lifetime %u%s", paging_get_lifetime(btsb->paging_state),
		VTY_NEWLINE);
	if (btsb->agch_queue_thresh_level != GSM_BTS_AGCH_QUEUE_THRESH_LEVEL_DEFAULT
		 || btsb->agch_queue_low_level != GSM_BTS_AGCH_QUEUE_LOW_LEVEL_DEFAULT
		 || btsb->agch_queue_high_level != GSM_BTS_AGCH_QUEUE_HIGH_LEVEL_DEFAULT)
		vty_out(vty, " agch-queue-mgmt threshold %d low %d high %d%s",
			btsb->agch_queue_thresh_level, btsb->agch_queue_low_level,
			btsb->agch_queue_high_level, VTY_NEWLINE);

	for (i = 0; i < 32; i++) {
		if (gsmtap_sapi_mask & (1 << i)) {
			const char *name = get_value_string(gsmtap_sapi_names, i);
			vty_out(vty, " gsmtap-sapi %s%s", osmo_str_tolower(name), VTY_NEWLINE);
		}
	}
	if (gsmtap_sapi_acch) {
		const char *name = get_value_string(gsmtap_sapi_names, GSMTAP_CHANNEL_ACCH);
		vty_out(vty, " gsmtap-sapi %s%s", osmo_str_tolower(name), VTY_NEWLINE);
	}

	bts_model_config_write_bts(vty, bts);

	llist_for_each_entry(trx, &bts->trx_list, list) {
		vty_out(vty, " trx %u%s", trx->nr, VTY_NEWLINE);
		bts_model_config_write_trx(vty, trx);
	}
}

static int config_write_bts(struct vty *vty)
{
	struct gsm_network *net = gsmnet_from_vty(vty);
	struct gsm_bts *bts;

	llist_for_each_entry(bts, &net->bts_list, list)
		config_write_bts_single(vty, bts);

	return CMD_SUCCESS;
}

static int config_write_dummy(struct vty *vty)
{
	return CMD_SUCCESS;
}

/* per-BTS configuration */
DEFUN(cfg_bts,
      cfg_bts_cmd,
      "bts BTS_NR",
      "Select a BTS to configure\n"
	"BTS Number\n")
{
	struct gsm_network *gsmnet = gsmnet_from_vty(vty);
	int bts_nr = atoi(argv[0]);
	struct gsm_bts *bts;

	if (bts_nr >= gsmnet->num_bts) {
		vty_out(vty, "%% Unknown BTS number %u (num %u)%s",
			bts_nr, gsmnet->num_bts, VTY_NEWLINE);
		return CMD_WARNING;
	} else
		bts = gsm_bts_num(gsmnet, bts_nr);

	vty->index = bts;
	vty->index_sub = &bts->description;
	vty->node = BTS_NODE;

	return CMD_SUCCESS;
}

#warning merge with OpenBSC?
DEFUN(cfg_bts_unit_id,
      cfg_bts_unit_id_cmd,
      "ipa unit-id <0-65534> <0-255>",
      "ip.access RSL commands\n"
      "Set the Unit ID of this BTS\n"
      "Site ID\n" "Unit ID\n")
{
	struct gsm_bts *bts = vty->index;
	int site_id = atoi(argv[0]);
	int bts_id = atoi(argv[1]);

	bts->ip_access.site_id = site_id;
	bts->ip_access.bts_id = bts_id;

	return CMD_SUCCESS;
}

DEFUN(cfg_bts_band,
      cfg_bts_band_cmd,
      "band (450|GSM450|480|GSM480|750|GSM750|810|GSM810|850|GSM850|900|GSM900|1800|DCS1800|1900|PCS1900)",
      "Set the frequency band of this BTS\n"
      "Alias for GSM450\n450Mhz\n"
      "Alias for GSM480\n480Mhz\n"
      "Alias for GSM750\n750Mhz\n"
      "Alias for GSM810\n810Mhz\n"
      "Alias for GSM850\n850Mhz\n"
      "Alias for GSM900\n900Mhz\n"
      "Alias for DCS1800\n1800Mhz\n"
      "Alias for PCS1900\n1900Mhz\n")
{
	struct gsm_bts *bts = vty->index;
	int band = gsm_band_parse(argv[0]);

	if (band < 0) {
		vty_out(vty, "%% BAND %d is not a valid GSM band%s",
			band, VTY_NEWLINE);
		return CMD_WARNING;
	}

	bts->band = band;

	return CMD_SUCCESS;
}

DEFUN(cfg_bts_oml_ip,
      cfg_bts_oml_ip_cmd,
      "oml remote-ip A.B.C.D",
      "OML Parameters\n" "OML IP Address\n" "OML IP Address\n")
{
	struct gsm_bts *bts = vty->index;
	struct gsm_bts_role_bts *btsb = bts_role_bts(bts);

	if (btsb->bsc_oml_host)
		talloc_free(btsb->bsc_oml_host);

	btsb->bsc_oml_host = talloc_strdup(btsb, argv[0]);

	return CMD_SUCCESS;
}

#define RTP_STR "RTP parameters\n"

DEFUN_HIDDEN(cfg_bts_rtp_bind_ip,
      cfg_bts_rtp_bind_ip_cmd,
      "rtp bind-ip A.B.C.D",
      RTP_STR "RTP local bind IP Address\n" "RTP local bind IP Address\n")
{
	vty_out(vty, "%% rtp bind-ip is now deprecated%s", VTY_NEWLINE);

	return CMD_WARNING;
}

DEFUN(cfg_bts_rtp_jitbuf,
	cfg_bts_rtp_jitbuf_cmd,
	"rtp jitter-buffer <0-10000>",
	RTP_STR "RTP jitter buffer\n" "jitter buffer in ms\n")
{
	struct gsm_bts *bts = vty->index;
	struct gsm_bts_role_bts *btsb = bts_role_bts(bts);

	btsb->rtp_jitter_buf_ms = atoi(argv[0]);

	return CMD_SUCCESS;
}

#define PAG_STR "Paging related parameters\n"

DEFUN(cfg_bts_paging_queue_size,
	cfg_bts_paging_queue_size_cmd,
	"paging queue-size <1-1024>",
	PAG_STR "Maximum length of BTS-internal paging queue\n"
	        "Maximum length of BTS-internal paging queue\n")
{
	struct gsm_bts *bts = vty->index;
	struct gsm_bts_role_bts *btsb = bts_role_bts(bts);

	paging_set_queue_max(btsb->paging_state, atoi(argv[0]));

	return CMD_SUCCESS;
}

DEFUN(cfg_bts_paging_lifetime,
	cfg_bts_paging_lifetime_cmd,
	"paging lifetime <0-60>",
	PAG_STR "Maximum lifetime of a paging record\n"
	        "Maximum lifetime of a paging record (secods)\n")
{
	struct gsm_bts *bts = vty->index;
	struct gsm_bts_role_bts *btsb = bts_role_bts(bts);

	paging_set_lifetime(btsb->paging_state, atoi(argv[0]));

	return CMD_SUCCESS;
}

#define AGCH_QUEUE_STR "AGCH queue mgmt\n"

DEFUN(cfg_bts_agch_queue_mgmt_params,
	cfg_bts_agch_queue_mgmt_params_cmd,
	"agch-queue-mgmt threshold <0-100> low <0-100> high <0-100000>",
	AGCH_QUEUE_STR
	"Threshold to start cleanup\nin %% of the maximum queue length\n"
	"Low water mark for cleanup\nin %% of the maximum queue length\n"
	"High water mark for cleanup\nin %% of the maximum queue length\n")
{
	struct gsm_bts *bts = vty->index;
	struct gsm_bts_role_bts *btsb = bts_role_bts(bts);

	btsb->agch_queue_thresh_level = atoi(argv[0]);
	btsb->agch_queue_low_level = atoi(argv[1]);
	btsb->agch_queue_high_level = atoi(argv[2]);

	return CMD_SUCCESS;
}

DEFUN(cfg_bts_agch_queue_mgmt_default,
	cfg_bts_agch_queue_mgmt_default_cmd,
	"agch-queue-mgmt default",
	AGCH_QUEUE_STR
	"Reset clean parameters to default values\n")
{
	struct gsm_bts *bts = vty->index;
	struct gsm_bts_role_bts *btsb = bts_role_bts(bts);

	btsb->agch_queue_thresh_level = GSM_BTS_AGCH_QUEUE_THRESH_LEVEL_DEFAULT;
	btsb->agch_queue_low_level = GSM_BTS_AGCH_QUEUE_LOW_LEVEL_DEFAULT;
	btsb->agch_queue_high_level = GSM_BTS_AGCH_QUEUE_HIGH_LEVEL_DEFAULT;

	return CMD_SUCCESS;
}


/* ======================================================================
 * SHOW
 * ======================================================================*/

static void net_dump_nmstate(struct vty *vty, struct gsm_nm_state *nms)
{
	vty_out(vty,"Oper '%s', Admin %u, Avail '%s'%s",
		abis_nm_opstate_name(nms->operational), nms->administrative,
		abis_nm_avail_name(nms->availability), VTY_NEWLINE);
}

static void bts_dump_vty(struct vty *vty, struct gsm_bts *bts)
{
	struct gsm_bts_role_bts *btsb = bts->role;

	vty_out(vty, "BTS %u is of %s type in band %s, has CI %u LAC %u, "
		"BSIC %u, TSC %u and %u TRX%s",
		bts->nr, "FIXME", gsm_band_name(bts->band),
		bts->cell_identity,
		bts->location_area_code, bts->bsic, bts->tsc,
		bts->num_trx, VTY_NEWLINE);
	vty_out(vty, "  Description: %s%s",
		bts->description ? bts->description : "(null)", VTY_NEWLINE);
	vty_out(vty, "  Unit ID: %u/%u/0, OML Stream ID 0x%02x%s",
			bts->ip_access.site_id, bts->ip_access.bts_id,
			bts->oml_tei, VTY_NEWLINE);
	vty_out(vty, "  NM State: ");
	net_dump_nmstate(vty, &bts->mo.nm_state);
	vty_out(vty, "  Site Mgr NM State: ");
	net_dump_nmstate(vty, &bts->site_mgr.mo.nm_state);
	vty_out(vty, "  Paging: Queue size %u, occupied %u, lifetime %us%s",
		paging_get_queue_max(btsb->paging_state), paging_queue_length(btsb->paging_state),
		paging_get_lifetime(btsb->paging_state), VTY_NEWLINE);
	vty_out(vty, "  AGCH: Queue limit %u, occupied %d, "
		"dropped %llu, merged %llu, rejected %llu, "
		"ag-res %llu, non-res %llu%s",
		btsb->agch_max_queue_length, btsb->agch_queue_length,
		btsb->agch_queue_dropped_msgs, btsb->agch_queue_merged_msgs,
		btsb->agch_queue_rejected_msgs, btsb->agch_queue_agch_msgs,
		btsb->agch_queue_pch_msgs,
		VTY_NEWLINE);
#if 0
	vty_out(vty, "  Paging: %u pending requests, %u free slots%s",
		paging_pending_requests_nr(bts),
		bts->paging.available_slots, VTY_NEWLINE);
	if (is_ipaccess_bts(bts)) {
		vty_out(vty, "  OML Link state: %s.%s",
			bts->oml_link ? "connected" : "disconnected", VTY_NEWLINE);
	} else {
		vty_out(vty, "  E1 Signalling Link:%s", VTY_NEWLINE);
		e1isl_dump_vty(vty, bts->oml_link);
	}

	/* FIXME: chan_desc */
	memset(&pl, 0, sizeof(pl));
	bts_chan_load(&pl, bts);
	vty_out(vty, "  Current Channel Load:%s", VTY_NEWLINE);
	dump_pchan_load_vty(vty, "    ", &pl);
#endif
}


DEFUN(show_bts, show_bts_cmd, "show bts <0-255>",
	SHOW_STR "Display information about a BTS\n"
		"BTS number")
{
	struct gsm_network *net = gsmnet_from_vty(vty);
	int bts_nr;

	if (argc != 0) {
		/* use the BTS number that the user has specified */
		bts_nr = atoi(argv[0]);
		if (bts_nr >= net->num_bts) {
			vty_out(vty, "%% can't find BTS '%s'%s", argv[0],
				VTY_NEWLINE);
			return CMD_WARNING;
		}
		bts_dump_vty(vty, gsm_bts_num(net, bts_nr));
		return CMD_SUCCESS;
	}
	/* print all BTS's */
	for (bts_nr = 0; bts_nr < net->num_bts; bts_nr++)
		bts_dump_vty(vty, gsm_bts_num(net, bts_nr));

	return CMD_SUCCESS;
}

static struct gsm_lchan *resolve_lchan(struct gsm_network *net,
					const char **argv, int idx)
{
	int bts_nr = atoi(argv[idx+0]);
	int trx_nr = atoi(argv[idx+1]);
	int ts_nr = atoi(argv[idx+2]);
	int lchan_nr = atoi(argv[idx+3]);
	struct gsm_bts *bts;
	struct gsm_bts_trx *trx;
	struct gsm_bts_trx_ts *ts;

	bts = gsm_bts_num(net, bts_nr);
	if (!bts)
		return NULL;

	trx = gsm_bts_trx_num(bts, trx_nr);
	if (!trx)
		return NULL;

	if (ts_nr >= ARRAY_SIZE(trx->ts))
		return NULL;
	ts = &trx->ts[ts_nr];

	if (lchan_nr >= ARRAY_SIZE(ts->lchan))
		return NULL;

	return &ts->lchan[lchan_nr];
}

#define BTS_T_T_L_STR			\
	"BTS related commands\n"	\
	"BTS number\n"			\
	"TRX related commands\n"	\
	"TRX number\n"			\
	"timeslot related commands\n"	\
	"timeslot number\n"		\
	"logical channel commands\n"	\
	"logical channel number\n"

DEFUN(cfg_trx_gsmtap_sapi, cfg_trx_gsmtap_sapi_cmd,
	"HIDDEN", "HIDDEN")
{
	int sapi;

	sapi = get_string_value(gsmtap_sapi_names, argv[0]);

	if (sapi == GSMTAP_CHANNEL_ACCH)
		gsmtap_sapi_acch = 1;
	else
		gsmtap_sapi_mask |= (1 << sapi);

	return CMD_SUCCESS;
}

DEFUN(cfg_trx_no_gsmtap_sapi, cfg_trx_no_gsmtap_sapi_cmd,
	"HIDDEN", "HIDDEN")
{
	int sapi;

	sapi = get_string_value(gsmtap_sapi_names, argv[0]);

	if (sapi == GSMTAP_CHANNEL_ACCH)
		gsmtap_sapi_acch = 0;
	else
		gsmtap_sapi_mask &= ~(1 << sapi);

	return CMD_SUCCESS;
}

DEFUN(bts_t_t_l_jitter_buf,
	bts_t_t_l_jitter_buf_cmd,
	"bts <0-0> trx <0-0> ts <0-7> lchan <0-1> rtp jitter-buffer <0-10000>",
	BTS_T_T_L_STR "RTP settings\n"
	"Jitter buffer\n" "Size of jitter buffer in (ms)\n")
{
	struct gsm_network *net = gsmnet_from_vty(vty);
	struct gsm_lchan *lchan;
	int jitbuf_ms = atoi(argv[4]);

	lchan = resolve_lchan(net, argv, 0);
	if (!lchan) {
		vty_out(vty, "%% can't find BTS%s", VTY_NEWLINE);
		return CMD_WARNING;
	}
	if (!lchan->abis_ip.rtp_socket) {
		vty_out(vty, "%% this channel has no active RTP stream%s",
			VTY_NEWLINE);
		return CMD_WARNING;
	}
	osmo_rtp_socket_set_param(lchan->abis_ip.rtp_socket,
				  OSMO_RTP_P_JITBUF, jitbuf_ms);

	return CMD_SUCCESS;
}

DEFUN(bts_t_t_l_loopback,
	bts_t_t_l_loopback_cmd,
	"bts <0-0> trx <0-0> ts <0-7> lchan <0-1> loopback",
	BTS_T_T_L_STR "Set loopback\n")
{
	struct gsm_network *net = gsmnet_from_vty(vty);
	struct gsm_lchan *lchan;

	lchan = resolve_lchan(net, argv, 0);
	if (!lchan) {
		vty_out(vty, "%% can't find BTS%s", VTY_NEWLINE);
		return CMD_WARNING;
	}
	lchan->loopback = 1;

	return CMD_SUCCESS;
}

DEFUN(no_bts_t_t_l_loopback,
	no_bts_t_t_l_loopback_cmd,
	"no bts <0-0> trx <0-0> ts <0-7> lchan <0-1> loopback",
	NO_STR BTS_T_T_L_STR "Set loopback\n")
{
	struct gsm_network *net = gsmnet_from_vty(vty);
	struct gsm_lchan *lchan;

	lchan = resolve_lchan(net, argv, 0);
	if (!lchan) {
		vty_out(vty, "%% can't find BTS%s", VTY_NEWLINE);
		return CMD_WARNING;
	}
	lchan->loopback = 0;

	return CMD_SUCCESS;
}

int bts_vty_init(struct gsm_bts *bts, int trx_num, const struct log_info *cat)
{
	cfg_trx_gsmtap_sapi_cmd.string = vty_cmd_string_from_valstr(bts, gsmtap_sapi_names,
						"gsmtap-sapi (",
						"|",")", VTY_DO_LOWER);
	cfg_trx_gsmtap_sapi_cmd.doc = vty_cmd_string_from_valstr(bts, gsmtap_sapi_names,
						"GSMTAP SAPI\n",
						"\n", "", 0);

	cfg_trx_no_gsmtap_sapi_cmd.string = vty_cmd_string_from_valstr(bts, gsmtap_sapi_names,
						"no gsmtap-sapi (",
						"|",")", VTY_DO_LOWER);
	cfg_trx_no_gsmtap_sapi_cmd.doc = vty_cmd_string_from_valstr(bts, gsmtap_sapi_names,
						NO_STR "GSMTAP SAPI\n",
						"\n", "", 0);

	install_element_ve(&show_bts_cmd);

	logging_vty_add_cmds(cat);

	install_node(&bts_node, config_write_bts);
	install_element(CONFIG_NODE, &cfg_bts_cmd);
	install_default(BTS_NODE);
	install_element(BTS_NODE, &cfg_bts_unit_id_cmd);
	install_element(BTS_NODE, &cfg_bts_oml_ip_cmd);
	install_element(BTS_NODE, &cfg_bts_rtp_bind_ip_cmd);
	install_element(BTS_NODE, &cfg_bts_rtp_jitbuf_cmd);
	install_element(BTS_NODE, &cfg_bts_band_cmd);
	install_element(BTS_NODE, &cfg_description_cmd);
	install_element(BTS_NODE, &cfg_no_description_cmd);
	install_element(BTS_NODE, &cfg_bts_paging_queue_size_cmd);
	install_element(BTS_NODE, &cfg_bts_paging_lifetime_cmd);
	install_element(BTS_NODE, &cfg_bts_agch_queue_mgmt_default_cmd);
	install_element(BTS_NODE, &cfg_bts_agch_queue_mgmt_params_cmd);

	install_element(BTS_NODE, &cfg_trx_gsmtap_sapi_cmd);
	install_element(BTS_NODE, &cfg_trx_no_gsmtap_sapi_cmd);

	/* add and link to TRX config node */
	sprintf(cfg_bts_trx_cmd_string, "trx <0-%d>", trx_num - 1);
	install_element(BTS_NODE, &cfg_bts_trx_cmd);
	install_node(&trx_node, config_write_dummy);
	install_default(TRX_NODE);

	install_element(ENABLE_NODE, &bts_t_t_l_jitter_buf_cmd);
	install_element(ENABLE_NODE, &bts_t_t_l_loopback_cmd);
	install_element(ENABLE_NODE, &no_bts_t_t_l_loopback_cmd);

	return 0;
}
