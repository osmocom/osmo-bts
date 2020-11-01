#ifndef L1SAP_H
#define L1SAP_H

#include <osmocom/gsm/protocol/gsm_04_08.h>
#include <osmocom/gsm/protocol/gsm_08_58.h>

/* lchan link ID */
#define LID_SACCH 0x40
#define LID_DEDIC 0x00

/* timeslot and subslot from chan_nr */
#define L1SAP_CHAN2TS(chan_nr) (chan_nr & 7)
#define L1SAP_CHAN2SS_TCHH(chan_nr) ((chan_nr >> 3) & 1)
#define L1SAP_CHAN2SS_SDCCH4(chan_nr) ((chan_nr >> 3) & 3)
#define L1SAP_CHAN2SS_SDCCH8(chan_nr) ((chan_nr >> 3) & 7)
#define L1SAP_CHAN2SS_BCCH(chan_nr) (CCCH_LCHAN)

/* logical channel from chan_nr + link_id */
#define L1SAP_IS_LINK_SACCH(link_id) \
	((link_id & 0xC0) == LID_SACCH)
#define L1SAP_IS_CHAN_TCHF(chan_nr) \
	((chan_nr & 0xf8) == RSL_CHAN_Bm_ACCHs)
#define L1SAP_IS_CHAN_TCHH(chan_nr) \
	((chan_nr & 0xf0) == RSL_CHAN_Lm_ACCHs)
#define L1SAP_IS_CHAN_SDCCH4(chan_nr) \
	((chan_nr & 0xe0) == RSL_CHAN_SDCCH4_ACCH)
#define L1SAP_IS_CHAN_SDCCH8(chan_nr) \
	((chan_nr & 0xc0) == RSL_CHAN_SDCCH8_ACCH)
#define L1SAP_IS_CHAN_BCCH(chan_nr) \
	((chan_nr & 0xf8) == RSL_CHAN_BCCH)
#define L1SAP_IS_CHAN_RACH(chan_nr) \
	((chan_nr & 0xf8) == RSL_CHAN_RACH)
#define L1SAP_IS_CHAN_AGCH_PCH(chan_nr) \
	((chan_nr & 0xf8) == RSL_CHAN_PCH_AGCH)
#define L1SAP_IS_CHAN_PDCH(chan_nr) \
	((chan_nr & 0xf8) == RSL_CHAN_OSMO_PDCH)
#define L1SAP_IS_CHAN_CBCH(chan_nr) \
	((chan_nr & 0xf8) == RSL_CHAN_OSMO_CBCH4) \
	|| ((chan_nr & 0xf8) == RSL_CHAN_OSMO_CBCH8)

/* rach type from ra */
#define L1SAP_IS_PACKET_RACH(ra) ((ra & 0xf0) == 0x70 && (ra & 0x0f) != 0x0f)

/* CCCH block from frame number */
unsigned int l1sap_fn2ccch_block(uint32_t fn);

/* PTCH layout from frame number */
#define L1SAP_FN2MACBLOCK(fn) ((fn % 52) / 4)
#define L1SAP_FN2PTCCHBLOCK(fn) ((fn / 104) & 3)

/* Calculate PTCCH occurrence, See also 3GPP TS 05.02, Clause 7, Table 6 of 9 */
#define L1SAP_IS_PTCCH(fn) (((fn % 52) == 12) || ((fn % 52) == 38))


static const uint8_t fill_frame[GSM_MACBLOCK_LEN] = {
        0x03, 0x03, 0x01, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B,
        0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B,
        0x2B, 0x2B, 0x2B
};

/* subslot from any chan_nr */
static inline uint8_t l1sap_chan2ss(uint8_t chan_nr)
{
	if (L1SAP_IS_CHAN_BCCH(chan_nr))
                return L1SAP_CHAN2SS_BCCH(chan_nr);
	if (L1SAP_IS_CHAN_SDCCH8(chan_nr))
		return L1SAP_CHAN2SS_SDCCH8(chan_nr);
	if (L1SAP_IS_CHAN_SDCCH4(chan_nr))
		return L1SAP_CHAN2SS_SDCCH4(chan_nr);
	if (L1SAP_IS_CHAN_TCHH(chan_nr))
		return L1SAP_CHAN2SS_TCHH(chan_nr);
	return 0;
}

struct gsm_lchan *get_lchan_by_chan_nr(struct gsm_bts_trx *trx,
				       unsigned int chan_nr);

/* allocate a msgb containing a osmo_phsap_prim + optional l2 data */
struct msgb *l1sap_msgb_alloc(unsigned int l2_len);

/* any L1 prim received from bts model */
int l1sap_up(struct gsm_bts_trx *trx, struct osmo_phsap_prim *l1sap);

/* pcu (socket interface) sends us a data request primitive */
int l1sap_pdch_req(struct gsm_bts_trx_ts *ts, int is_ptcch, uint32_t fn,
	uint16_t arfcn, uint8_t block_nr, const uint8_t *data, uint8_t len);

/* call-back function for incoming RTP */
void l1sap_rtp_rx_cb(struct osmo_rtp_socket *rs, const uint8_t *rtp_pl,
		     unsigned int rtp_pl_len, uint16_t seq_number,
		     uint32_t timestamp, bool marker);

/* channel control */
int l1sap_chan_act(struct gsm_bts_trx *trx, uint8_t chan_nr, struct tlv_parsed *tp);
int l1sap_chan_rel(struct gsm_bts_trx *trx, uint8_t chan_nr);
int l1sap_chan_deact_sacch(struct gsm_bts_trx *trx, uint8_t chan_nr);
int l1sap_chan_modify(struct gsm_bts_trx *trx, uint8_t chan_nr);

enum l1sap_common_sapi {
	L1SAP_COMMON_SAPI_UNKNOWN,
	/* alphabetic order */
	L1SAP_COMMON_SAPI_AGCH,
	L1SAP_COMMON_SAPI_BCCH,
	L1SAP_COMMON_SAPI_CBCH,
	L1SAP_COMMON_SAPI_FACCH_F,
	L1SAP_COMMON_SAPI_FACCH_H,
	L1SAP_COMMON_SAPI_FCCH,
	L1SAP_COMMON_SAPI_IDLE,
	L1SAP_COMMON_SAPI_NCH,
	L1SAP_COMMON_SAPI_PACCH,
	L1SAP_COMMON_SAPI_PAGCH,
	L1SAP_COMMON_SAPI_PBCCH,
	L1SAP_COMMON_SAPI_PCH,
	L1SAP_COMMON_SAPI_PDTCH,
	L1SAP_COMMON_SAPI_PNCH,
	L1SAP_COMMON_SAPI_PPCH,
	L1SAP_COMMON_SAPI_PRACH,
	L1SAP_COMMON_SAPI_PTCCH,
	L1SAP_COMMON_SAPI_RACH,
	L1SAP_COMMON_SAPI_SACCH,
	L1SAP_COMMON_SAPI_SCH,
	L1SAP_COMMON_SAPI_SDCCH,
	L1SAP_COMMON_SAPI_TCH_F,
	L1SAP_COMMON_SAPI_TCH_H,
};

extern uint16_t l1sap_log_ctx_sapi;
extern const struct value_string l1sap_common_sapi_names[];

extern const struct value_string gsmtap_sapi_names[];
extern struct gsmtap_inst *gsmtap;
extern uint32_t gsmtap_sapi_mask;
extern uint8_t gsmtap_sapi_acch;

int add_l1sap_header(struct gsm_bts_trx *trx, struct msgb *rmsg,
		     struct gsm_lchan *lchan, uint8_t chan_nr, uint32_t fn,
		     uint16_t ber10k, int16_t lqual_cb, int8_t rssi,
		     int16_t ta_offs, uint8_t is_sub);

#define msgb_l1sap_prim(msg) ((struct osmo_phsap_prim *)(msg)->l1h)

int bts_check_for_first_ciphrd(struct gsm_lchan *lchan,
				uint8_t *data, int len);

int is_ccch_for_agch(struct gsm_bts_trx *trx, uint32_t fn);

void repeated_dl_facch_active_decision(struct gsm_lchan *lchan,
				       const uint8_t *l3, size_t l3_len);

#endif /* L1SAP_H */
