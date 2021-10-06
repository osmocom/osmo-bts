#ifndef _GSM_DATA_H
#define _GSM_DATA_H

#include <stdbool.h>
#include <stdint.h>

#include <osmocom/core/timer.h>
#include <osmocom/core/bitvec.h>
#include <osmocom/core/statistics.h>
#include <osmocom/core/utils.h>
#include <osmocom/core/linuxlist.h>
#include <osmocom/core/tdef.h>
#include <osmocom/gsm/gsm23003.h>
#include <osmocom/gsm/gsm0502.h>
#include <osmocom/gsm/gsm_utils.h>
#include <osmocom/gsm/tlv.h>
#include <osmocom/gsm/rxlev_stat.h>
#include <osmocom/gsm/sysinfo.h>
#include <osmocom/gsm/bts_features.h>
#include <osmocom/gsm/gsm48_rest_octets.h>
#include <osmocom/gsm/protocol/gsm_04_08.h>
#include <osmocom/gsm/protocol/gsm_08_58.h>
#include <osmocom/gsm/protocol/gsm_12_21.h>

#include <osmocom/abis/e1_input.h>

#include <osmo-bts/paging.h>
#include <osmo-bts/tx_power.h>
#include <osmo-bts/oml.h>
#include <osmo-bts/lchan.h>

#define GSM_FR_BITS	260
#define GSM_EFR_BITS	244

#define GSM_FR_BYTES	33	/* TS 101318 Chapter 5.1: 260 bits + 4bit sig */
#define GSM_HR_BYTES	14	/* TS 101318 Chapter 5.2: 112 bits, no sig */
#define GSM_EFR_BYTES	31	/* TS 101318 Chapter 5.3: 244 bits + 4bit sig */

#define GSM_BTS_AGCH_QUEUE_THRESH_LEVEL_DEFAULT 41
#define GSM_BTS_AGCH_QUEUE_THRESH_LEVEL_DISABLE 999999
#define GSM_BTS_AGCH_QUEUE_LOW_LEVEL_DEFAULT 41
#define GSM_BTS_AGCH_QUEUE_HIGH_LEVEL_DEFAULT 91

struct gsm_network {
	struct llist_head bts_list;
	unsigned int num_bts;
	struct osmo_plmn_id plmn;
	struct pcu_sock_state *pcu_state;
};

/* 16 is the max. number of SI2quater messages according to 3GPP TS 44.018 Table 10.5.2.33b.1:
   4-bit index is used (2#1111 = 10#15) */
#define SI2Q_MAX_NUM 16
/* length in bits (for single SI2quater message) */
#define SI2Q_MAX_LEN 160
#define SI2Q_MIN_LEN 18

/* lchans 0..3 are SDCCH in combined channel configuration,
   use 4 as magic number for BCCH hack - see osmo-bts-../oml.c:opstart_compl() */
#define CCCH_LCHAN 4

#define TRX_NR_TS	8
#define TS_MAX_LCHAN	8

#define MAX_VERSION_LENGTH 64

enum gsm_bts_trx_ts_flags {
	TS_F_PDCH_ACTIVE =		0x1000,
	TS_F_PDCH_ACT_PENDING =		0x2000,
	TS_F_PDCH_DEACT_PENDING =	0x4000,
	TS_F_PDCH_PENDING_MASK =	0x6000 /*<
			TS_F_PDCH_ACT_PENDING | TS_F_PDCH_DEACT_PENDING */
};

/* One Timeslot in a TRX */
struct gsm_bts_trx_ts {
	struct gsm_bts_trx *trx;
	/* number of this timeslot at the TRX */
	uint8_t nr;

	enum gsm_phys_chan_config pchan;

	struct {
		enum gsm_phys_chan_config pchan_is;
		enum gsm_phys_chan_config pchan_want;
		struct msgb *pending_chan_activ;
	} dyn;

	unsigned int flags;
	struct gsm_abis_mo mo;

	/* Training Sequence Code (range 0..7) */
	uint8_t tsc_oml; /* configured via OML */
	uint8_t tsc; /* currently in use */
	/* Training Sequence Set (range 0..3) */
	uint8_t tsc_set;

	/* Actual BCCH carrier power reduction */
	uint8_t c0_power_red_db;

	/* Frequency hopping parameters (configured via OML) */
	struct {
		bool enabled;
		uint8_t maio;
		uint8_t hsn;
		uint16_t arfcn_list[64];
		uint8_t arfcn_num;
	} hopping;

	/* Transceiver "cache" for frequency hopping */
	const struct gsm_bts_trx *fh_trx_list[64];

	/* Implementation specific structure(s) */
	void *priv;

	/* VAMOS specific fields */
	struct {
		/* NULL if BTS_FEAT_VAMOS is not set */
		struct gsm_bts_trx_ts *peer;
		bool is_shadow;
	} vamos;

	struct gsm_lchan lchan[TS_MAX_LCHAN];
};

enum gprs_rlc_par {
	RLC_T3142,
	RLC_T3169,
	RLC_T3191,
	RLC_T3193,
	RLC_T3195,
	RLC_N3101,
	RLC_N3103,
	RLC_N3105,
	CV_COUNTDOWN,
	T_DL_TBF_EXT,	/* ms */
	T_UL_TBF_EXT,	/* ms */
	_NUM_RLC_PAR
};

enum gprs_cs {
	GPRS_CS1,
	GPRS_CS2,
	GPRS_CS3,
	GPRS_CS4,
	GPRS_MCS1,
	GPRS_MCS2,
	GPRS_MCS3,
	GPRS_MCS4,
	GPRS_MCS5,
	GPRS_MCS6,
	GPRS_MCS7,
	GPRS_MCS8,
	GPRS_MCS9,
	_NUM_GRPS_CS
};

/* The amount of time within which a sudden disconnect of a newly established
 * OML connection will cause a special warning to be logged. */
#define OSMO_BTS_OML_CONN_EARLY_DISCONNECT 10	 /* in seconds */

extern struct osmo_tdef_group bts_tdef_groups[];
extern struct osmo_tdef bts_T_defs[];
extern struct osmo_tdef abis_T_defs[];

extern const struct value_string gsm_pchant_names[13];
extern const struct value_string gsm_pchant_descs[13];
const char *gsm_pchan_name(enum gsm_phys_chan_config c);
const char *gsm_lchant_name(enum gsm_chan_t c);
char *gsm_ts_name(const struct gsm_bts_trx_ts *ts);
char *gsm_ts_and_pchan_name(const struct gsm_bts_trx_ts *ts);

#define GSM_TS_NAME_FMT \
	"bts=%u,trx=%u,ts=%u" "%s"
#define GSM_TS_NAME_ARGS(ts) \
	(ts)->trx->bts->nr, (ts)->trx->nr, (ts)->nr, \
	(ts)->vamos.is_shadow ? ",shadow" : ""

#define BSIC2BCC(bsic) ((bsic) & 0x07)
#define BTS_TSC(bts) BSIC2BCC((bts)->bsic)

struct gsm_lchan *rsl_lchan_lookup(struct gsm_bts_trx *trx, uint8_t chan_nr,
				   int *rc);

enum gsm_phys_chan_config ts_pchan(const struct gsm_bts_trx_ts *ts);
uint8_t ts_subslots(const struct gsm_bts_trx_ts *ts);
bool ts_is_tch(const struct gsm_bts_trx_ts *ts);

int conf_lchans_as_pchan(struct gsm_bts_trx_ts *ts,
			 enum gsm_phys_chan_config pchan);

/* cipher code */
#define CIPHER_A5(x) (1 << (x-1))

bool ts_is_pdch(const struct gsm_bts_trx_ts *ts);

void gsm_ts_release(struct gsm_bts_trx_ts *ts);

#endif /* _GSM_DATA_H */
