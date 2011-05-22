#ifndef _ABIS_H
#define _ABIS_H

#include <osmocom/core/select.h>
#include <osmocom/core/timer.h>

#define IPA_TCP_PORT_OML	3002
#define IPA_TCP_PORT_RSL	3003

#define	OML_RETRY_TIMER		5
#define	OML_PING_TIMER		20

struct ipabis_head {
	u_int16_t len;	/* network byte order */
	u_int8_t proto;
	u_int8_t data[0];
} __attribute__ ((packed));

enum ipabis_proto {
	IPA_PROTO_RSL		= 0x00,
	IPA_PROTO_IPACCESS	= 0xfe,
	IPA_PROTO_SCCP		= 0xfd,
	IPA_PROTO_OML		= 0xff,
};

enum ipabis_msgtype {
	IPA_MSGT_PING		= 0x00,
	IPA_MSGT_PONG		= 0x01,
	IPA_MSGT_ID_GET		= 0x04,
	IPA_MSGT_ID_RESP	= 0x05,
	IPA_MSGT_ID_ACK		= 0x06,
};

enum ipabis_id_tags {
	IPA_IDTAG_SERNR		= 0x00,
	IPA_IDTAG_UNITNAME	= 0x01,
	IPA_IDTAG_LOCATION1	= 0x02,
	IPA_IDTAG_LOCATION2	= 0x03,
	IPA_IDTAG_EQUIPVERS	= 0x04,
	IPA_IDTAG_SWVERSION	= 0x05,
	IPA_IDTAG_IPADDR	= 0x06,
	IPA_IDTAG_MACADDR	= 0x07,
	IPA_IDTAG_UNIT		= 0x08,
};

struct ipabis_link {
	int state;
	struct osmocom_bts	*bts;	/* set, if OML link */
	struct osmobts_trx	*trx;	/* set, if RSL link */
	struct osmo_fd		bfd;
	struct osmo_timer_list	timer;
	struct msgb		*rx_msg;
	struct llist_head	tx_queue;
	int			ping, pong, id_resp;
	uint32_t		ip;
};

enum {
	LINK_STATE_IDLE = 0,
	LINK_STATE_RETRYING,
	LINK_STATE_CONNECTING,
	LINK_STATE_CONNECT,
};

int abis_tx(struct ipabis_link *link, struct msgb *msg);
struct msgb *abis_msgb_alloc(int headroom);
void abis_push_ipa(struct msgb *msg, uint8_t proto);
int abis_open(struct ipabis_link *link, uint32_t ip);
void abis_close(struct ipabis_link *link);

#endif /* _ABIS_H */
