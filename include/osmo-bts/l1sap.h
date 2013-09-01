#ifndef L1SAP_H
#define L1SAP_H

/* timeslot and subslot from chan_nr */
#define L1SAP_CHAN2TS(chan_nr) (chan_nr & 7)
#define L1SAP_CHAN2SS_TCHH(chan_nr) ((chan_nr >> 3) & 1)
#define L1SAP_CHAN2SS_SDCCH4(chan_nr) ((chan_nr >> 3) & 3)
#define L1SAP_CHAN2SS_SDCCH8(chan_nr) ((chan_nr >> 3) & 7)

/* logical channel from chan_nr + link_id */
#define L1SAP_IS_LINK_SACCH(link_id) ((link_id & 0xC0) == 0x40)
#define L1SAP_IS_CHAN_TCHF(chan_nr) ((chan_nr & 0xf8) == 0x08)
#define L1SAP_IS_CHAN_TCHH(chan_nr) ((chan_nr & 0xf0) == 0x10)
#define L1SAP_IS_CHAN_SDCCH4(chan_nr) ((chan_nr & 0xe0) == 0x20)
#define L1SAP_IS_CHAN_SDCCH8(chan_nr) ((chan_nr & 0xc0) == 0x40)
#define L1SAP_IS_CHAN_BCCH(chan_nr) ((chan_nr & 0xf8) == 0x80)
#define L1SAP_IS_CHAN_RACH(chan_nr) ((chan_nr & 0xf8) == 0x88)
#define L1SAP_IS_CHAN_AGCH_PCH(chan_nr) ((chan_nr & 0xf8) == 0x90)

/* rach type from ra */
#define L1SAP_IS_PACKET_RACH(ra) ((ra & 0xf0) == 0x70)

/* CCCH block from frame number */
#define L1SAP_FN2CCCHBLOCK(fn) ((fn % 51) / 5 - 1)

/* PTCH layout from frame number */
#define L1SAP_FN2MACBLOCK(fn) ((fn % 52) / 4)
#define L1SAP_FN2PTCCHBLOCK(fn) ((fn / 52) & 7)
#define L1SAP_IS_PTCCH(fn) ((fn % 52) == 12)

/* subslot from any chan_nr */
static inline uint8_t l1sap_chan2ss(uint8_t chan_nr)
{
	if (L1SAP_IS_CHAN_SDCCH8(chan_nr))
		return L1SAP_CHAN2SS_SDCCH8(chan_nr);
	if (L1SAP_IS_CHAN_SDCCH4(chan_nr))
		return L1SAP_CHAN2SS_SDCCH4(chan_nr);
	if (L1SAP_IS_CHAN_TCHH(chan_nr))
		return L1SAP_CHAN2SS_TCHH(chan_nr);
	return 0;
}


/* allocate a msgb containing a osmo_phsap_prim + optional l2 data */
struct msgb *l1sap_msgb_alloc(unsigned int l2_len);

/* any L1 prim received from bts model */
int l1sap_up(struct gsm_bts_trx *trx, struct osmo_phsap_prim *l1sap);

/* pcu (socket interface) sends us a data request primitive */
int l1sap_pdch_req(struct gsm_bts_trx_ts *ts, int is_ptcch, uint32_t fn,
	uint16_t arfcn, uint8_t block_nr, uint8_t *data, uint8_t len);

/* call-back function for incoming RTP */
void l1sap_rtp_rx_cb(struct osmo_rtp_socket *rs, const uint8_t *rtp_pl,
	unsigned int rtp_pl_len);

/* channel control */
int l1sap_chan_act(struct gsm_bts_trx *trx, uint8_t chan_nr, struct tlv_parsed *tp);
int l1sap_chan_rel(struct gsm_bts_trx *trx, uint8_t chan_nr);
int l1sap_chan_deact_sacch(struct gsm_bts_trx *trx, uint8_t chan_nr);
int l1sap_chan_modify(struct gsm_bts_trx *trx, uint8_t chan_nr);

extern const struct value_string gsmtap_sapi_names[];
extern struct gsmtap_inst *gsmtap;
extern uint32_t gsmtap_sapi_mask;
extern uint8_t gsmtap_sapi_acch;

#define msgb_l1sap_prim(msg) ((struct osmo_phsap_prim *)(msg)->l1h)

#endif /* L1SAP_H */
