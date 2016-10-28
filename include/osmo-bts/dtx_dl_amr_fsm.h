#pragma once

#include <osmocom/core/fsm.h>
#include <osmocom/core/utils.h>
#include <osmocom/core/logging.h>

/* DTX DL AMR FSM */

#define X(s) (1 << (s))

enum dtx_dl_amr_fsm_states {
	ST_VOICE,
	ST_SID_F1,
	ST_SID_F2,
	ST_F1_INH,
	ST_U_INH,
	ST_SID_U,
	ST_ONSET_V,
	ST_ONSET_F,
	ST_FACCH_V,
	ST_FACCH,
};

enum dtx_dl_amr_fsm_events {
	E_VOICE,
	E_ONSET,
	E_FACCH,
	E_COMPL,
	E_INHIB,
	E_SID_F,
	E_SID_U,
};

extern const struct value_string dtx_dl_amr_fsm_event_names[];
extern struct osmo_fsm dtx_dl_amr_fsm;
