#ifndef _LOGGING_H
#define _LOGGING_H

#define DEBUG
#include <osmocore/logging.h>

enum {
	DRSL,
	DOML,
	DRLL,
	DRR,
	DMM,
	DCC,
	DSMS,
	DMEAS,
	DPAG,
	DLAPDM,
	DL1C,
	DSAP,
	DABIS,
	DRTP,
};

extern const struct log_info log_info;

#endif /* _LOGGING_H */
