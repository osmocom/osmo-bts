#ifndef OSMO_BTS_PAGING_H
#define OSMO_BTS_PAGING_H

#include <stdint.h>
#include <osmocom/gsm/gsm_utils.h>
#include <osmocom/gsm/protocol/gsm_04_08.h>

struct paging_state;
struct gsm_bts;
struct asci_notification;

/* abstract representation of P1 rest octets; we only implement those parts we need for now */
struct p1_rest_octets {
	struct {
		bool present;
		uint8_t nln;
		uint8_t nln_status;
	} nln_pch;
	bool packet_page_ind[2];
	bool r8_present;
	struct {
		bool prio_ul_access;
		bool etws_present;
		struct {
			bool is_first;
			uint8_t page_nr;
			const uint8_t *page;
			size_t page_bytes;
		} etws;
	} r8;
};

/* abstract representation of P2 rest octets; we only implement those parts we need for now */
struct p2_rest_octets {
	struct {
		bool present;
		uint8_t cn3;
	} cneed;
	struct {
		bool present;
		uint8_t nln;
		uint8_t nln_status;
	} nln_pch;
};

/* abstract representation of P3 rest octets; we only implement those parts we need for now */
struct p3_rest_octets {
	struct {
		bool present;
		uint8_t cn3;
		uint8_t cn4;
	} cneed;
	struct {
		bool present;
		uint8_t nln;
		uint8_t nln_status;
	} nln_pch;
};

/* initialize paging code */
struct paging_state *paging_init(struct gsm_bts *bts,
				 unsigned int num_paging_max,
				 unsigned int paging_lifetime);

/* (re) configure paging code */
void paging_config(struct paging_state *ps,
		  unsigned int num_paging_max,
		  unsigned int paging_lifetime);

void paging_reset(struct paging_state *ps);

/* The max number of paging entries */
unsigned int paging_get_queue_max(struct paging_state *ps);
void paging_set_queue_max(struct paging_state *ps, unsigned int queue_max);

/* The lifetime of a paging entry */
unsigned int paging_get_lifetime(struct paging_state *ps);
void paging_set_lifetime(struct paging_state *ps, unsigned int lifetime);

/* update with new SYSTEM INFORMATION parameters */
int paging_si_update(struct paging_state *ps, struct gsm48_control_channel_descr *chan_desc);

/* Add an identity to the paging queue */
int paging_add_identity(struct paging_state *ps, uint8_t paging_group,
			const uint8_t *identity_lv, uint8_t chan_needed);

/* Add a ready formatted MAC block message to the paging queue, this can be an IMMEDIATE ASSIGNMENT, or a
 * PAGING COMMAND (from the PCU) */
int paging_add_macblock(struct paging_state *ps, uint32_t msg_id, const char *imsi, bool confirm, const uint8_t *macblock);

/* Paging rest octests */
void append_p1_rest_octets(struct bitvec *bv, const struct p1_rest_octets *p1ro,
			   const struct asci_notification *notif);
void append_p2_rest_octets(struct bitvec *bv, const struct p2_rest_octets *p2ro);
void append_p3_rest_octets(struct bitvec *bv, const struct p3_rest_octets *p3ro);

/* generate paging message for given gsm time */
int paging_gen_msg(struct paging_state *ps, uint8_t *out_buf, struct gsm_time *gt,
		   int *is_empty);


/* inspection methods below */
int paging_group_queue_empty(struct paging_state *ps, uint8_t group);
int paging_queue_length(struct paging_state *ps);
int paging_buffer_space(struct paging_state *ps);

#endif
