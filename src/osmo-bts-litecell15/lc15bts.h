#ifndef LC15BTS_H
#define LC15BTS_H

#include <stdlib.h>
#include <osmocom/core/utils.h>

#include <nrw/litecell15/litecell15.h>
#include <nrw/litecell15/gsml1const.h>

/*
 * Depending on the firmware version either GsmL1_Prim_t or Litecell15_Prim_t
 * is the bigger struct. For earlier firmware versions the GsmL1_Prim_t was the
 * bigger struct.
 */
#define LC15BTS_PRIM_SIZE \
	(OSMO_MAX(sizeof(Litecell15_Prim_t), sizeof(GsmL1_Prim_t)) + 128)

enum l1prim_type {
	L1P_T_INVALID, /* this must be 0 to detect uninitialized elements */
	L1P_T_REQ,
	L1P_T_CONF,
	L1P_T_IND,
};


enum lc15_diversity_mode{
	LC15_DIVERSITY_SISO_A = 0,
	LC15_DIVERSITY_SISO_B,
	LC15_DIVERSITY_MRC,
};

enum lc15_pedestal_mode{
	LC15_PEDESTAL_OFF = 0,
	LC15_PEDESTAL_ON,
};

enum lc15_led_control_mode{
	LC15_LED_CONTROL_BTS = 0,
	LC15_LED_CONTROL_EXT,
};

enum lc15_auto_pwr_adjust_mode{
	LC15_TX_PWR_ADJ_NONE = 0,
	LC15_TX_PWR_ADJ_AUTO,
};

enum l1prim_type lc15bts_get_l1prim_type(GsmL1_PrimId_t id);
const struct value_string lc15bts_l1prim_names[GsmL1_PrimId_NUM+1];
GsmL1_PrimId_t lc15bts_get_l1prim_conf(GsmL1_PrimId_t id);

enum l1prim_type lc15bts_get_sysprim_type(Litecell15_PrimId_t id);
const struct value_string lc15bts_sysprim_names[Litecell15_PrimId_NUM+1];
Litecell15_PrimId_t lc15bts_get_sysprim_conf(Litecell15_PrimId_t id);

const struct value_string lc15bts_l1sapi_names[GsmL1_Sapi_NUM+1];
const struct value_string lc15bts_l1status_names[GSML1_STATUS_NUM+1];

const struct value_string lc15bts_tracef_names[29];
const struct value_string lc15bts_tracef_docs[29];

const struct value_string lc15bts_tch_pl_names[15];

const struct value_string lc15bts_clksrc_names[10];

const struct value_string lc15bts_dir_names[6];

enum pdch_cs {
	PDCH_CS_1,
	PDCH_CS_2,
	PDCH_CS_3,
	PDCH_CS_4,
	PDCH_MCS_1,
	PDCH_MCS_2,
	PDCH_MCS_3,
	PDCH_MCS_4,
	PDCH_MCS_5,
	PDCH_MCS_6,
	PDCH_MCS_7,
	PDCH_MCS_8,
	PDCH_MCS_9,
	_NUM_PDCH_CS
};

const uint8_t pdch_msu_size[_NUM_PDCH_CS];

/* LC15 default parameters */
#define LC15_BTS_MAX_CELL_SIZE_DEFAULT	166	/* 166 qbits is default  value */
#define LC15_BTS_DIVERSITY_MODE_DEFAULT	0	/* SISO-A is default mode */
#define LC15_BTS_PEDESTAL_MODE_DEFAULT	0	/* Unused TS is off by default */
#define LC15_BTS_LED_CTRL_MODE_DEFAULT	0	/* LED is controlled by BTS by default */
#define LC15_BTS_DSP_ALIVE_TMR_DEFAULT	5	/* Default DSP alive timer is 5 seconds  */
#define LC15_BTS_TX_PWR_ADJ_DEFAULT	0	/* Default Tx power auto adjustment is none */
#define LC15_BTS_TX_RED_PWR_8PSK_DEFAULT	0	/* Default 8-PSK maximum power level is 0 dB */
#define LC15_BTS_RTP_DRIFT_THRES_DEFAULT	0	/* Default RTP drift threshold is 0 ms (disabled) */
#define LC15_BTS_TX_C0_IDLE_RED_PWR_DEFAULT	0	/* Default C0 idle slot reduction power level is 0 dB */

#endif /* LC15BTS_H */
