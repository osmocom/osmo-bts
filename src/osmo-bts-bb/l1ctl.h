#ifndef _BB_L1CTL_H
#define _BB_L1CTL_H

#include <osmocom/core/write_queue.h>

enum baseband_role {
	BASEBAND_TX,
	BASEBAND_RX,
};

struct osmo_l1ctl {

	int bb_role;
	struct osmo_l1_if *l1_if;
	struct osmo_wqueue l2_wq;
};

int l1ctl_send(struct osmo_l1ctl *l1ctl, struct msgb *msg);
int l1socket_open(struct osmo_l1ctl *l1ctl, const char *socket_path);
int l1socket_close(struct osmo_l1ctl *l1ctl);

/*
 * L1CTL interface
 */

enum {
	_L1CTL_NONE		= 0,
	L1CTL_FBSB_REQ,
	L1CTL_FBSB_CONF,
	L1CTL_DATA_IND,
	L1CTL_RACH_REQ,
	L1CTL_DM_EST_REQ,
	L1CTL_DATA_REQ,
	L1CTL_RESET_IND,
	L1CTL_PM_REQ,		/* power measurement */
	L1CTL_PM_CONF,		/* power measurement */
	L1CTL_ECHO_REQ,
	L1CTL_ECHO_CONF,
	L1CTL_RACH_CONF,
	L1CTL_RESET_REQ,
	L1CTL_RESET_CONF,
	L1CTL_DATA_CONF,
	L1CTL_CCCH_MODE_REQ,
	L1CTL_CCCH_MODE_CONF,
	L1CTL_DM_REL_REQ,
	L1CTL_PARAM_REQ,
	L1CTL_DM_FREQ_REQ,
	L1CTL_CRYPTO_REQ,
	L1CTL_SIM_REQ,
	L1CTL_SIM_CONF,
	L1CTL_TCH_MODE_REQ,
	L1CTL_TCH_MODE_CONF,
	L1CTL_NEIGH_PM_REQ,
	L1CTL_NEIGH_PM_IND,
	L1CTL_TRAFFIC_REQ,
	L1CTL_TRAFFIC_CONF,
	L1CTL_TRAFFIC_IND,
};
/* there are no more messages in a sequence */
#define L1CTL_F_DONE	0x01

struct l1ctl_hdr {
	uint8_t msg_type;
	uint8_t flags;
	uint8_t padding[2];
	uint8_t data[0];
} __attribute__((packed));

struct l1ctl_info_dl {
	/* GSM 08.58 channel number (9.3.1) */
	uint8_t chan_nr;
	/* GSM 08.58 link identifier (9.3.2) */
	uint8_t link_id;
	/* the ARFCN and the band. FIXME: what about MAIO? */
	uint16_t band_arfcn;

	uint32_t frame_nr;

	uint8_t rx_level;	/* 0 .. 63 in typical GSM notation (dBm+110) */
	uint8_t snr;		/* Signal/Noise Ration (dB) */
	uint8_t num_biterr;
	uint8_t fire_crc;

	uint8_t payload[0];
} __attribute__((packed));

struct l1ctl_info_ul {
	/* GSM 08.58 channel number (9.3.1) */
	uint8_t chan_nr;
	/* GSM 08.58 link identifier (9.3.2) */
	uint8_t link_id;
	uint8_t padding[2];

	uint8_t payload[0];
} __attribute__((packed));

enum l1ctl_reset_type {
	L1CTL_RES_T_BOOT,	/* only _IND */
	L1CTL_RES_T_FULL,
	L1CTL_RES_T_SCHED,
};

/* argument to L1CTL_RESET_REQ and L1CTL_RESET_IND */
struct l1ctl_reset {
	uint8_t type;
	uint8_t pad[3];
} __attribute__((packed));

#endif /* _BB_L1CTL_H */
