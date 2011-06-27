#ifndef OSMO_BTS_PAGING_H
#define OSMO_BTS_PAGING_H

#include <stdint.h>
#include <osmocom/gsm/gsm_utils.h>
#include <osmocom/gsm/protocol/gsm_04_08.h>

struct paging_state;

/* initialize paging code */
struct paging_state *paging_init(void *ctx, unsigned int num_paging_max,
				 unsigned int paging_lifetime);

void paging_reset(struct paging_state *ps);

/* update with new SYSTEM INFORMATION parameters */
int paging_si_update(struct paging_state *ps, struct gsm48_control_channel_descr *chan_desc);

/* Add an identity to the paging queue */
int paging_add_identity(struct paging_state *ps, uint8_t paging_group,
			const uint8_t *identity_lv, uint8_t chan_needed);

/* generate paging message for given gsm time */
int paging_gen_msg(struct paging_state *ps, uint8_t *out_buf, struct gsm_time *gt);

#endif
