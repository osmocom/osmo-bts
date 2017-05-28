#ifndef _FEMTO_L1_H
#define _FEMTO_L1_H

#include <osmocom/core/select.h>
#include <osmocom/core/write_queue.h>
#include <osmocom/core/gsmtap_util.h>
#include <osmocom/core/timer.h>
#include <osmocom/gsm/gsm_utils.h>

#include <sysmocom/femtobts/gsml1prim.h>

enum {
	MQ_SYS_READ,
	MQ_L1_READ,
#ifndef HW_SYSMOBTS_V1
	MQ_TCH_READ,
	MQ_PDTCH_READ,
#endif
	_NUM_MQ_READ
};

enum {
	MQ_SYS_WRITE,
	MQ_L1_WRITE,
#ifndef HW_SYSMOBTS_V1
	MQ_TCH_WRITE,
	MQ_PDTCH_WRITE,
#endif
	_NUM_MQ_WRITE
};

struct calib_send_state {
	const char *path;
	int last_file_idx;
};

enum {
	FIXUP_UNITILIAZED,
	FIXUP_NEEDED,
	FIXUP_NOT_NEEDED,
};

struct femtol1_hdl {
	struct gsm_time gsm_time;
	uint32_t hLayer1;			/* handle to the L1 instance in the DSP */
	uint32_t dsp_trace_f;
	uint8_t clk_use_eeprom;
	int clk_cal;
	int ul_power_target;
	uint8_t clk_src;
	float min_qual_rach;
	float min_qual_norm;
	char *calib_path;
	struct llist_head wlc_list;

	struct gsmtap_inst *gsmtap;
	uint32_t gsmtap_sapi_mask;

	void *priv;			/* user reference */

	struct osmo_timer_list alive_timer;
	unsigned int alive_prim_cnt;

	struct osmo_fd read_ofd[_NUM_MQ_READ];	/* osmo file descriptors */
	struct osmo_wqueue write_q[_NUM_MQ_WRITE];

	struct {
		/* from DSP/FPGA after L1 Init */
		uint8_t dsp_version[3];
		uint8_t fpga_version[3];
		uint32_t band_support;	/* bitmask of GSM_BAND_* */
		uint8_t ver_major;
		uint8_t ver_minor;
		/* from EEPROM */
		uint16_t model_nr;
		uint16_t model_flags;
		uint8_t trx_nr;
	} hw_info;

	int fixup_needed;
	bool rtp_hr_jumble_needed;

	struct calib_send_state st;

	uint8_t last_rf_mute[8];
};

#define L1_VER_SHIFT(x,y,z) ((x << 16) | (y << 8) | (z))

static inline uint32_t l1if_dsp_ver(struct femtol1_hdl *fl1h)
{
        const uint8_t *v = fl1h->hw_info.dsp_version;
        return L1_VER_SHIFT(v[0], v[1], v[2]);
}

static inline uint32_t l1if_fpga_ver(struct femtol1_hdl *fl1h)
{
        const uint8_t *v = fl1h->hw_info.fpga_version;
        return L1_VER_SHIFT(v[0], v[1], v[2]);
}

#define msgb_l1prim(msg)	((GsmL1_Prim_t *)(msg)->l1h)
#define msgb_sysprim(msg)	((SuperFemto_Prim_t *)(msg)->l1h)

typedef int l1if_compl_cb(struct gsm_bts_trx *trx, struct msgb *l1_msg, void *data);

/* send a request primitive to the L1 and schedule completion call-back */
int l1if_req_compl(struct femtol1_hdl *fl1h, struct msgb *msg,
		   l1if_compl_cb *cb, void *cb_data);
int l1if_gsm_req_compl(struct femtol1_hdl *fl1h, struct msgb *msg,
		l1if_compl_cb *cb, void *cb_data);

struct femtol1_hdl *l1if_open(void *priv);
int l1if_close(struct femtol1_hdl *hdl);
int l1if_reset(struct femtol1_hdl *hdl);
int l1if_activate_rf(struct femtol1_hdl *hdl, int on);
int l1if_set_trace_flags(struct femtol1_hdl *hdl, uint32_t flags);
int l1if_set_txpower(struct femtol1_hdl *fl1h, float tx_power);
int l1if_mute_rf(struct femtol1_hdl *hdl, uint8_t mute[8], l1if_compl_cb *cb);

struct msgb *l1p_msgb_alloc(void);
struct msgb *sysp_msgb_alloc(void);

uint32_t l1if_lchan_to_hLayer(struct gsm_lchan *lchan);
struct gsm_lchan *l1if_hLayer_to_lchan(struct gsm_bts_trx *trx, uint32_t hLayer);

/* tch.c */
int l1if_tch_rx(struct gsm_lchan *lchan, struct msgb *l1p_msg);
int l1if_tch_fill(struct gsm_lchan *lchan, uint8_t *l1_buffer);
struct msgb *gen_empty_tch_msg(struct gsm_lchan *lchan);

/* ciphering */
int l1if_set_ciphering(struct femtol1_hdl *fl1h,
			  struct gsm_lchan *lchan,
			  int dir_downlink);

/* calibration loading */
int calib_load(struct femtol1_hdl *fl1h);

/* on-line re-calibration */
int l1if_rf_clock_info_reset(struct femtol1_hdl *fl1h);
int l1if_rf_clock_info_correct(struct femtol1_hdl *fl1h);

/* public helpers for test */
int bts_check_for_ciph_cmd(struct femtol1_hdl *fl1h,
			      struct msgb *msg, struct gsm_lchan *lchan);
void bts_check_for_first_ciphrd(struct femtol1_hdl *fl1h,
				GsmL1_MsgUnitParam_t *msgUnitParam,
				struct gsm_lchan *lchan);
inline int l1if_ms_pwr_ctrl(struct gsm_lchan *lchan, const int uplink_target,
			const uint8_t ms_power, const float rxLevel);
#endif /* _FEMTO_L1_H */
