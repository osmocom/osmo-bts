#ifndef _ABIS_H
#define _ABIS_H

#include <osmocom/core/select.h>
#include <osmocom/core/timer.h>

#include <osmo-bts/gsm_data.h>

enum abis_link_fsm_event {
	ABIS_LINK_EV_SIGN_LINK_OML_UP,
	ABIS_LINK_EV_SIGN_LINK_DOWN,
	ABIS_LINK_EV_VTY_RM_ADDR, /* data: struct bsc_oml_host* being removed */
};

void abis_init(struct gsm_bts *bts);
int abis_open(struct gsm_bts *bts, char *model_name);



int abis_oml_sendmsg(struct msgb *msg);
int abis_bts_rsl_sendmsg(struct msgb *msg);

uint32_t get_signlink_remote_ip(struct e1inp_sign_link *link);

#endif /* _ABIS_H */
