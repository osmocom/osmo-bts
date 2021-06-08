#pragma once

#include <osmocom/core/msgb.h>
#include <osmo-bts/pcuif_proto.h>

struct gsm_bts;

int down_osmo(struct gsm_bts *bts, struct msgb *msg);

int abis_osmo_pcu_tx_container(struct gsm_bts *bts, const struct gsm_pcu_if_container *container);
