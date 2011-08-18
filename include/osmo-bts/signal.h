#ifndef OSMO_BTS_SIGNAL_H
#define OSMO_BTS_SIGNAL_H

#include <osmocom/core/signal.h>

enum sig_subsys {
	SS_GLOBAL,
};

enum signals_global {
	S_NEW_SYSINFO,
};

struct osmo_signal_new_si {
	struct gsm_bts_trx *trx;
	enum osmo_sysinfo_type osmo_si;
};

#endif
