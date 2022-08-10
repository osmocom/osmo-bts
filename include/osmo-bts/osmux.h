#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <osmocom/core/select.h>
#include <osmocom/netif/osmux.h>

struct gsm_bts;
struct gsm_lchan;

enum osmux_usage {
	OSMUX_USAGE_OFF = 0,
	OSMUX_USAGE_ON = 1,
	OSMUX_USAGE_ONLY = 2,
};

struct osmux_state {
	enum osmux_usage use;
	char *local_addr;
	uint16_t local_port;
	struct osmo_fd fd;
	uint8_t batch_factor;
	unsigned int batch_size;
	bool dummy_padding;
	struct llist_head osmux_handle_list;
};

/* Contains a "struct osmux_in_handle" towards a specific peer (remote IPaddr+port) */
struct osmux_handle {
	struct llist_head head;
	struct gsm_bts *bts;
	struct osmux_in_handle *in;
	struct osmo_sockaddr rem_addr;
	int refcnt;
};

int bts_osmux_init(struct gsm_bts *bts);
void bts_osmux_release(struct gsm_bts *bts);
int bts_osmux_open(struct gsm_bts *bts);

int lchan_osmux_init(struct gsm_lchan *lchan, uint8_t rtp_payload);
void lchan_osmux_release(struct gsm_lchan *lchan);
int lchan_osmux_connect(struct gsm_lchan *lchan);
bool lchan_osmux_connected(const struct gsm_lchan *lchan);
int lchan_osmux_send_frame(struct gsm_lchan *lchan, const uint8_t *payload,
			   unsigned int payload_len, unsigned int duration, bool marker);

int lchan_osmux_skipped_frame(struct gsm_lchan *lchan, unsigned int duration);
