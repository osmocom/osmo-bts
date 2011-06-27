#ifndef _ABIS_H
#define _ABIS_H

#include <osmocom/core/select.h>
#include <osmocom/core/timer.h>
#include <osmocom/gsm/protocol/ipaccess.h>

#include <osmo-bts/gsm_data.h>

#define	OML_RETRY_TIMER		5
#define	OML_PING_TIMER		20

struct ipabis_link {
	int state;
	struct gsm_bts		*bts;	/* set, if OML link */
	struct gsm_bts_trx	*trx;	/* set, if RSL link */
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


int abis_oml_sendmsg(struct msgb *msg);
int abis_rsl_sendmsg(struct msgb *msg);

#endif /* _ABIS_H */
