#ifndef _SYSMOBTS_MGR_H
#define _SYSMOBTS_MGR_H

enum {
	DTEMP,
	DFW,
	DFIND,
};

enum {
	SYSMO_MGR_DISCONNECTED = 0,
	SYSMO_MGR_CONNECTED,
};

#define SOCKET_PATH		"/var/run/bts_oml"

#endif
