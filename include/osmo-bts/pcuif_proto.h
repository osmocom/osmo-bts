#ifndef _PCUIF_PROTO_H
#define _PCUIF_PROTO_H

#include <osmocom/gsm/l1sap.h>
#include <arpa/inet.h>
#include <osmocom/gsm/protocol/gsm_23_003.h>

#define PCU_SOCK_DEFAULT	"/tmp/pcu_bts"

#define PCU_IF_VERSION		0x0d
#define TXT_MAX_LEN	128

/* msg_type */
#define PCU_IF_MSG_DATA_REQ	0x00	/* send data to given channel */
#define PCU_IF_MSG_DATA_IND	0x02	/* receive data from given channel */
#define PCU_IF_MSG_SUSP_REQ	0x03	/* BTS forwards GPRS SUSP REQ to PCU */
#define PCU_IF_MSG_APP_INFO_REQ	0x04	/* BTS asks PCU to transmit APP INFO via PACCH */
#define PCU_IF_MSG_RTS_REQ	0x10	/* ready to send request */
#define PCU_IF_MSG_DATA_CNF_2	0x11	/* confirm (using message id) */
#define PCU_IF_MSG_RACH_IND	0x22	/* receive RACH */
#define PCU_IF_MSG_INFO_IND	0x32	/* retrieve BTS info */
#define PCU_IF_MSG_ACT_REQ	0x40	/* activate/deactivate PDCH */
#define PCU_IF_MSG_TIME_IND	0x52	/* GSM time indication */
#define PCU_IF_MSG_INTERF_IND	0x53	/* interference report */
#define PCU_IF_MSG_PAG_REQ	0x60	/* paging request */
#define PCU_IF_MSG_TXT_IND	0x70	/* Text indication for BTS */
#define PCU_IF_MSG_CONTAINER	0x80	/* Transparent container message */

/* sapi */
#define PCU_IF_SAPI_RACH	0x01	/* channel request on CCCH */
#define PCU_IF_SAPI_BCCH	0x04	/* SI on BCCH */
#define PCU_IF_SAPI_PDTCH	0x05	/* packet data/control/ccch block */
#define PCU_IF_SAPI_PRACH	0x06	/* packet random access channel */
#define PCU_IF_SAPI_PTCCH	0x07	/* packet TA control channel */
#define PCU_IF_SAPI_PCH_2	0x08	/* assignment on PCH (confirmed using message id) */
#define PCU_IF_SAPI_AGCH_2	0x09	/* assignment on AGCH (confirmed using message id) */

/* flags */
#define PCU_IF_FLAG_ACTIVE	(1 << 0)/* BTS is active */
#define PCU_IF_FLAG_DIRECT_PHY	(1 << 1)/* access PHY directly via dedicated hardware support */
#define PCU_IF_FLAG_CS1		(1 << 16)
#define PCU_IF_FLAG_CS2		(1 << 17)
#define PCU_IF_FLAG_CS3		(1 << 18)
#define PCU_IF_FLAG_CS4		(1 << 19)
#define PCU_IF_FLAG_MCS1	(1 << 20)
#define PCU_IF_FLAG_MCS2	(1 << 21)
#define PCU_IF_FLAG_MCS3	(1 << 22)
#define PCU_IF_FLAG_MCS4	(1 << 23)
#define PCU_IF_FLAG_MCS5	(1 << 24)
#define PCU_IF_FLAG_MCS6	(1 << 25)
#define PCU_IF_FLAG_MCS7	(1 << 26)
#define PCU_IF_FLAG_MCS8	(1 << 27)
#define PCU_IF_FLAG_MCS9	(1 << 28)

/* NSVC address type */
#define PCU_IF_ADDR_TYPE_UNSPEC	0x00	/* No address - empty entry */
#define PCU_IF_ADDR_TYPE_IPV4	0x04	/* IPv4 address */
#define PCU_IF_ADDR_TYPE_IPV6	0x29	/* IPv6 address */

/* BTS model */
enum gsm_pcuif_bts_model {
	PCU_IF_BTS_MODEL_UNSPEC,
	PCU_IF_BTS_MODEL_LC15,
	PCU_IF_BTS_MODEL_OC2G,
	PCU_IF_BTS_MODEL_OCTPHY,
	PCU_IF_BTS_MODEL_SYSMO,
	PCU_IF_BTS_MODEL_TRX,
	PCU_IF_BTS_MODEL_RBS,
};

#define PCU_IF_NUM_NSVC 2
#define PCU_IF_NUM_TRX 8

enum gsm_pcu_if_text_type {
	PCU_VERSION,
	PCU_OML_ALERT,
};

struct gsm_pcu_if_txt_ind {
	uint8_t		type; /* gsm_pcu_if_text_type */
	char		text[TXT_MAX_LEN]; /* Text to be transmitted to BTS */
} __attribute__ ((packed));

struct gsm_pcu_if_data {
	uint8_t		sapi;
	uint8_t		len;
	uint8_t		data[162];
	uint32_t	fn;
	uint16_t	arfcn;
	uint8_t		trx_nr;
	uint8_t		ts_nr;
	uint8_t		block_nr;
	int8_t		rssi;
	uint16_t	ber10k;		/* !< \brief BER in units of 0.01% */
	int16_t		ta_offs_qbits;	/* !< \brief Burst TA Offset in quarter bits */
	int16_t		lqual_cb;	/* !< \brief Link quality in centiBel */
} __attribute__ ((packed));

/* data confirmation with message id (instead of raw mac block) */
struct gsm_pcu_if_data_cnf {
	uint8_t		sapi;
	uint32_t	msg_id;
} __attribute__ ((packed));

struct gsm_pcu_if_rts_req {
	uint8_t		sapi;
	uint8_t		spare[3];
	uint32_t	fn;
	uint16_t	arfcn;
	uint8_t		trx_nr;
	uint8_t		ts_nr;
	uint8_t		block_nr;
} __attribute__ ((packed));

struct gsm_pcu_if_rach_ind {
	uint8_t		sapi;
	uint16_t	ra;
	int16_t		qta;
	uint32_t	fn;
	uint16_t	arfcn;
	uint8_t		is_11bit;
	uint8_t		burst_type;
	uint8_t		trx_nr;
	uint8_t		ts_nr;
} __attribute__ ((packed));

struct gsm_pcu_if_info_trx_ts {
	uint8_t		tsc;
	uint8_t		hopping;
	uint8_t		hsn;
	uint8_t		maio;
	uint8_t		ma_bit_len;
	uint8_t		ma[8];
} __attribute__ ((packed));

struct gsm_pcu_if_info_trx {
	uint16_t	arfcn;
	uint8_t		pdch_mask;		/* PDCH timeslot mask */
	uint8_t		spare;
	uint32_t	hlayer1;
	struct gsm_pcu_if_info_trx_ts ts[8];
} __attribute__ ((packed));

struct gsm_pcu_if_info_ind {
	uint32_t	version;
	uint32_t	flags;
	struct gsm_pcu_if_info_trx trx[PCU_IF_NUM_TRX];	/* TRX infos per BTS */
	uint8_t		bsic;
	/* RAI */
	uint16_t	mcc, mnc;
	uint8_t		mnc_3_digits;
	uint16_t	lac, rac;
	/* NSE */
	uint16_t	nsei;
	uint8_t		nse_timer[7];
	uint8_t		cell_timer[11];
	/* cell */
	uint16_t	cell_id;
	uint16_t	repeat_time;
	uint8_t		repeat_count;
	uint16_t	bvci;
	uint8_t		t3142;
	uint8_t		t3169;
	uint8_t		t3191;
	uint8_t		t3193_10ms;
	uint8_t		t3195;
	uint8_t		n3101;
	uint8_t		n3103;
	uint8_t		n3105;
	uint8_t		cv_countdown;
	uint16_t	dl_tbf_ext;
	uint16_t	ul_tbf_ext;
	uint8_t		initial_cs;
	uint8_t		initial_mcs;
	/* NSVC */
	uint16_t	nsvci[PCU_IF_NUM_NSVC];
	uint16_t	local_port[PCU_IF_NUM_NSVC];
	uint16_t	remote_port[PCU_IF_NUM_NSVC];
	uint8_t		address_type[PCU_IF_NUM_NSVC];
	union {
		struct in_addr v4;
		struct in6_addr v6;
	} remote_ip[PCU_IF_NUM_NSVC];
	uint8_t		bts_model; /* enum gsm_pcuif_bts_model */
} __attribute__ ((packed));

struct gsm_pcu_if_act_req {
	uint8_t		activate;
	uint8_t		trx_nr;
	uint8_t		ts_nr;
	uint8_t		spare;
} __attribute__ ((packed));

struct gsm_pcu_if_time_ind {
	uint32_t	fn;
} __attribute__ ((packed));

struct gsm_pcu_if_pag_req {
	uint8_t		sapi;
	uint8_t		chan_needed;
	uint8_t		identity_lv[9];
} __attribute__ ((packed));

/* BTS tells PCU to [once] send given application data via PACCH to all UE with active TBF */
struct gsm_pcu_if_app_info_req {
	uint8_t		application_type; /* 4bit field, see TS 44.060 11.2.47 */
	uint8_t		len;		  /* length of data */
	uint8_t		data[162];	  /* random size choice; ETWS needs 56 bytes */
} __attribute__ ((packed));

/* BTS tells PCU about a GPRS SUSPENSION REQUEST received on DCCH */
struct gsm_pcu_if_susp_req {
	uint32_t	tlli;
	uint8_t		ra_id[6];
	uint8_t		cause;
} __attribute__ ((packed));

/* Interference measurements on PDCH timeslots */
struct gsm_pcu_if_interf_ind {
	uint8_t		trx_nr;
	uint8_t		spare[3];
	uint32_t	fn;
	uint8_t		interf[8];
} __attribute__ ((packed));

/* Contains messages transmitted BSC<->PCU, potentially forwarded by BTS via IPA/PCU */
struct gsm_pcu_if_container {
	uint8_t		msg_type;
	uint8_t 	spare;
	uint16_t	length; /* network byte order */
	uint8_t		data[0];
} __attribute__ ((packed));

/* Struct to send a (confirmed) IMMEDIATE ASSIGNMENT message via PCH. The struct is sent as a data request
 * (data_req) under SAPI PCU_IF_SAPI_PCH_2. */
struct gsm_pcu_if_pch {
	/* message id as reference for confirmation */
	uint32_t msg_id;
	/* IMSI (to derive paging group) */
	char imsi[OSMO_IMSI_BUF_SIZE];
	/* GSM mac-block (with immediate assignment message) */
	uint8_t data[GSM_MACBLOCK_LEN];
	/* Set to true in case the receiving end must send a confirmation
	 * when the MAC block (data) has been sent. */
	bool confirm;
} __attribute__((packed));

/* Struct to send a (confirmed) IMMEDIATE ASSIGNMENT message via AGCH. The struct is sent as a data request
 * (data_req) under SAPI PCU_IF_SAPI_AGCH_2. */
struct gsm_pcu_if_agch {
	/* message id as reference for confirmation */
	uint32_t msg_id;
	/* GSM mac-block (with immediate assignment message) */
	uint8_t data[GSM_MACBLOCK_LEN];
	/* Set to true in case the receiving end must send a confirmation
	 * when the MAC block (data) has been sent. */
	bool confirm;
} __attribute__((packed));

/* reserved BTS number to indicate that the PCUIF INDICATION is not targeted to a
 * specific BTS. (commonly used with TXT indications to transfer the PCU version number) */
#define PCU_IF_BTS_NR_BCAST 0xff

struct gsm_pcu_if {
	/* context based information */
	uint8_t		msg_type;	/* message type */
	uint8_t		bts_nr;		/* bts number */
	uint8_t		spare[2];

	union {
		struct gsm_pcu_if_data		data_req;
		struct gsm_pcu_if_data_cnf	data_cnf2;
		struct gsm_pcu_if_data		data_ind;
		struct gsm_pcu_if_susp_req	susp_req;
		struct gsm_pcu_if_rts_req	rts_req;
		struct gsm_pcu_if_rach_ind	rach_ind;
		struct gsm_pcu_if_txt_ind	txt_ind;
		struct gsm_pcu_if_info_ind	info_ind;
		struct gsm_pcu_if_act_req	act_req;
		struct gsm_pcu_if_time_ind	time_ind;
		struct gsm_pcu_if_pag_req	pag_req;
		struct gsm_pcu_if_app_info_req	app_info_req;
		struct gsm_pcu_if_interf_ind	interf_ind;
		struct gsm_pcu_if_container	container;
	} u;
} __attribute__ ((packed));

#endif /* _PCUIF_PROTO_H */
