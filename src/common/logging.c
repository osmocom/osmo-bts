/* libosmocore logging support */

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
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */


#include <errno.h>

#include <osmocom/core/logging.h>
#include <osmocom/core/application.h>
#include <osmocom/core/utils.h>

#include <osmo-bts/bts.h>
#include <osmo-bts/logging.h>

static struct log_info_cat bts_log_info_cat[] = {
	[DRSL] = {
		.name = "DRSL",
		.description = "A-bis Radio Siganlling Link (RSL)",
		.color = "\033[1;35m",
		.enabled = 1, .loglevel = LOGL_NOTICE,
	},
	[DOML] =	{
		.name = "DOML",
		.description = "A-bis Network Management / O&M (NM/OML)",
		.color = "\033[1;36m",
		.enabled = 1, .loglevel = LOGL_NOTICE,
	},
	[DRLL] = {
		.name = "DRLL",
		.description = "A-bis Radio Link Layer (RLL)",
		.color = "\033[1;31m",
		.enabled = 1, .loglevel = LOGL_NOTICE,
	},
	[DRR] = {
		.name = "DRR",
		.description = "Layer3 Radio Resource (RR)",
		.color = "\033[1;34m",
		.enabled = 1, .loglevel = LOGL_NOTICE,
	},
	[DMEAS] = {
		.name = "DMEAS",
		.description = "Radio Measurement Processing",
		.enabled = 1, .loglevel = LOGL_NOTICE,
	},
	[DPAG]	= {
		.name = "DPAG",
		.description = "Paging Subsystem",
		.color = "\033[1;38m",
		.enabled = 1, .loglevel = LOGL_NOTICE,
	},
	[DL1C] = {
		.name = "DL1C",
		.description = "Layer 1 Control (MPH)",
		.loglevel = LOGL_NOTICE,
		.enabled = 1,
	},
	[DL1P] = {
		.name = "DL1P",
		.description = "Layer 1 Primitives (PH)",
		.loglevel = LOGL_NOTICE,
		.enabled = 0,
	},
	[DDSP] = {
		.name = "DDSP",
		.description = "DSP Trace Messages",
		.loglevel = LOGL_NOTICE,
		.enabled = 1,
	},
	[DPCU] = {
		.name = "DPCU",
		.description = "PCU interface",
		.loglevel = LOGL_NOTICE,
		.enabled = 1,
	},
	[DHO] = {
		.name = "DHO",
		.description = "Handover",
		.color = "\033[0;37m",
		.enabled = 1, .loglevel = LOGL_NOTICE,
	},
	[DTRX] = {
		.name = "DTRX",
		.description = "TRX interface",
		.color = "\033[1;33m",
		.enabled = 1, .loglevel = LOGL_NOTICE,
	},
	[DLOOP] = {
		.name = "DLOOP",
		.description = "Control loops",
		.color = "\033[0;94m",
		.enabled = 1, .loglevel = LOGL_NOTICE,
	},
	[DABIS] = {
		.name = "DABIS",
		.description = "A-bis Intput Subsystem",
		.enabled = 1, .loglevel = LOGL_NOTICE,
	},
	[DRTP] = {
		.name = "DRTP",
		.description = "Realtime Transfer Protocol",
		.loglevel = LOGL_NOTICE,
		.enabled = 1,
	},
	[DOSMUX] = {
		.name = "DOSMUX",
		.description = "Osmux (Osmocom RTP multiplexing)",
		.loglevel = LOGL_NOTICE,
		.enabled = 1,
	},
	[DASCI] = {
		.name = "DASCI",
		.description = "ASCI (Advanced Speech Call Items: VGCS/VBS)",
		.loglevel = LOGL_NOTICE,
		.enabled = 1,
	},

};

static int osmo_bts_filter_fn(const struct log_context *ctx, struct log_target *tgt)
{
	uint8_t *sapi = ctx->ctx[LOG_CTX_L1_SAPI];
	uint16_t *sapi_mask = tgt->filter_data[LOG_FLT_L1_SAPI];

	if ((tgt->filter_map & (1 << LOG_FLT_L1_SAPI)) != 0
	    && sapi_mask && sapi
	    && (*sapi_mask & (1 << *sapi)) != 0)
		return 1;

	return 0;
}

const struct log_info bts_log_info = {
	.filter_fn = osmo_bts_filter_fn,
	.cat = bts_log_info_cat,
	.num_cat = ARRAY_SIZE(bts_log_info_cat),
};
