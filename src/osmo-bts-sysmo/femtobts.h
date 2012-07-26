#ifndef FEMTOBTS_H
#define FEMTOBTS_H

#include <stdlib.h>
#include <osmocom/core/utils.h>

#include <sysmocom/femtobts/superfemto.h>
#include <sysmocom/femtobts/gsml1const.h>

#ifdef L1_HAS_RTP_MODE
/* This is temporarily disabled, as AMR has some bugs in RTP mode */
//#define USE_L1_RTP_MODE		/* Tell L1 to use RTP mode */
#endif

/* older header files don't have this */
#ifndef SUPERFEMTO_API
#define SUPERFEMTO_API(x,y,z)	((x << 16) + (y << 8) + z)
#endif

#ifndef SUPERFEMTO_API_VERSION
#define SUPERFEMTO_API_VERSION SUPERFEMTO_API(2,2,0)
#endif

/*
 * Depending on the firmware version either GsmL1_Prim_t or SuperFemto_Prim_t
 * is the bigger struct. For earlier firmware versions the GsmL1_Prim_t was the
 * bigger struct.
 */
#define SYSMOBTS_PRIM_SIZE \
	(OSMO_MAX(sizeof(SuperFemto_Prim_t), sizeof(GsmL1_Prim_t)) + 128)

enum l1prim_type {
	L1P_T_REQ,
	L1P_T_CONF,
	L1P_T_IND,
};

const enum l1prim_type femtobts_l1prim_type[GsmL1_PrimId_NUM];
const struct value_string femtobts_l1prim_names[GsmL1_PrimId_NUM+1];
const GsmL1_PrimId_t femtobts_l1prim_req2conf[GsmL1_PrimId_NUM];

const enum l1prim_type femtobts_sysprim_type[SuperFemto_PrimId_NUM];
const struct value_string femtobts_sysprim_names[SuperFemto_PrimId_NUM+1];
const SuperFemto_PrimId_t femtobts_sysprim_req2conf[SuperFemto_PrimId_NUM];

const struct value_string femtobts_l1sapi_names[GsmL1_Sapi_NUM+1];
const struct value_string femtobts_l1status_names[GSML1_STATUS_NUM+1];

const struct value_string femtobts_tracef_names[29];
const struct value_string femtobts_tracef_docs[29];

const struct value_string femtobts_tch_pl_names[15];

const struct value_string femtobts_clksrc_names[10];

const struct value_string femtobts_dir_names[6];

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

#endif /* FEMTOBTS_H */
