#ifndef FEMTOBTS_H
#define FEMTOBTS_H

#include <stdlib.h>
#include <osmocom/core/utils.h>

#include <sysmocom/femtobts/femtobts.h>
#include <sysmocom/femtobts/gsml1const.h>

enum l1prim_type {
	L1P_T_REQ,
	L1P_T_CONF,
	L1P_T_IND,
};


const enum l1prim_type femtobts_l1prim_type[GsmL1_PrimId_NUM];
const struct value_string femtobts_l1prim_names[GsmL1_PrimId_NUM+1];
const GsmL1_PrimId_t femtobts_l1prim_req2conf[GsmL1_PrimId_NUM];

const enum l1prim_type femtobts_sysprim_type[FemtoBts_PrimId_NUM];
const struct value_string femtobts_sysprim_names[FemtoBts_PrimId_NUM+1];
const FemtoBts_PrimId_t femtobts_sysprim_req2conf[FemtoBts_PrimId_NUM];

const struct value_string femtobts_l1sapi_names[GsmL1_Sapi_NUM+1];
const struct value_string femtobts_l1status_names[GSML1_STATUS_NUM+1];

const struct value_string femtobts_tracef_names[29];

const struct value_string femtobts_tch_pl_names[];

#endif /* FEMTOBTS_H */
