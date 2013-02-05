#ifndef _LOGGING_H
#define _LOGGING_H

#define DEBUG
#include <osmocom/core/logging.h>

enum {
	DRSL,
	DOML,
	DRLL,
	DRR,
	DMEAS,
	DPAG,
	DL1C,
	DL1P,
	DDSP,
	DPCU,
	DHO,
	DTRX,
	DABIS,
	DRTP,
	DSUM,
};

extern const struct log_info bts_log_info;

int bts_log_init(const char *category_mask);

#endif /* _LOGGING_H */
