#ifndef _BTS_H
#define _BTS_H

#include <osmo-bts/gsm_data.h>

extern void *tall_bts_ctx;

int bts_init(struct gsm_bts *bts);
void bts_shutdown(struct gsm_bts *bts, const char *reason);

struct gsm_bts *create_bts(uint8_t num_trx, char *id);
int create_ms(struct gsm_bts_trx *trx, int maskc, uint8_t *maskv_tx,
	uint8_t *maskv_rx);
void destroy_bts(struct gsm_bts *bts);
int work_bts(struct gsm_bts *bts);
int bts_link_estab(struct gsm_bts *bts);
int trx_link_estab(struct gsm_bts_trx *trx);
void bts_new_si(void *arg);
void bts_setup_slot(struct gsm_bts_trx_ts *slot, uint8_t comb);

int bts_agch_enqueue(struct gsm_bts *bts, struct msgb *msg);
struct msgb *bts_agch_dequeue(struct gsm_bts *bts);

uint8_t *bts_sysinfo_get(struct gsm_bts *bts, struct gsm_time *g_time);
uint8_t *lchan_sacch_get(struct gsm_lchan *lchan, struct gsm_time *g_time);
int lchan_init_lapdm(struct gsm_lchan *lchan);

void load_timer_start(struct gsm_bts *bts);

#endif /* _BTS_H */

