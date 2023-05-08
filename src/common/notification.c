#include <errno.h>

#include <osmocom/core/bitvec.h>
#include <osmocom/core/linuxlist.h>
#include <osmocom/gsm/protocol/gsm_08_58.h>

#include <osmo-bts/bts.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/notification.h>

static struct asci_notification *bts_asci_notification_find(struct gsm_bts *bts, const uint8_t *group_call_ref)
{
	struct asci_notification *n;
	llist_for_each_entry(n, &bts->asci.notifications, list) {
		if (!memcmp(n->group_call_ref, group_call_ref, sizeof(n->group_call_ref)))
			return n;
	}
	return NULL;
}

int bts_asci_notification_add(struct gsm_bts *bts, const uint8_t *group_call_ref, const uint8_t *chan_desc,
			      uint8_t chan_desc_len, const struct rsl_ie_nch_drx_info *nch_drx_info)
{
	struct asci_notification *n;

	if (bts_asci_notification_find(bts, group_call_ref))
		return -EEXIST;

	n = talloc_zero(bts, struct asci_notification);
	if (!n)
		return -ENOMEM;

	memcpy(n->group_call_ref, group_call_ref, sizeof(n->group_call_ref));
	if (chan_desc && chan_desc_len) {
		n->chan_desc.present = true;
		n->chan_desc.len = chan_desc_len;
		memcpy(&n->chan_desc.value, chan_desc, chan_desc_len);
	}
	if (nch_drx_info) {
		n->nch_drx_info.present = true;
		n->nch_drx_info.value = *nch_drx_info;
	}

	LOGP(DASCI, LOGL_INFO, "Added ASCI Notification for group call reference %s\n",
	     osmo_hexdump_nospc(n->group_call_ref, ARRAY_SIZE(n->group_call_ref)));

	/* add at beginning of "queue" to make sure a new call is notified first */
	llist_add(&n->list, &bts->asci.notifications);

	return 0;
}

int bts_asci_notification_del(struct gsm_bts *bts, const uint8_t *group_call_ref)
{
	struct asci_notification *n = bts_asci_notification_find(bts, group_call_ref);
	if (!n)
		return -ENODEV;

	LOGP(DASCI, LOGL_INFO, "Deleting ASCI Notification for group call reference %s\n",
	     osmo_hexdump_nospc(n->group_call_ref, ARRAY_SIZE(n->group_call_ref)));

	llist_del(&n->list);
	talloc_free(n);

	return 0;
}

int bts_asci_notification_reset(struct gsm_bts *bts)
{
	struct asci_notification *n, *n2;

	LOGP(DASCI, LOGL_INFO, "Deleting all %u ASCI Notifications of BTS\n",
	     llist_count(&bts->asci.notifications));

	llist_for_each_entry_safe(n, n2, &bts->asci.notifications, list) {
		llist_del(&n->list);
		talloc_free(n);
	}
	return 0;
}

const struct asci_notification *bts_asci_notification_get_next(struct gsm_bts *bts)
{
	struct asci_notification *n;

	n = llist_first_entry_or_null(&bts->asci.notifications, struct asci_notification, list);
	if (!n)
		return NULL;

	/* move to end of list to iterate over them */
	llist_del(&n->list);
	llist_add_tail(&n->list, &bts->asci.notifications);

	return n;
}


/*! append a "Group Call Information" CSN.1 structure to the caller-provided bit-vector.
 *  \param[out] bv caller-provided output bit-vector
 *  \param[in] gcr 5-byte group call reference
 *  \param[in] ch_desc optional group channel description (may be NULL)
 *  \param[in] ch_desc_len length of group channel description (in bytes) */
void append_group_call_information(struct bitvec *bv, const uint8_t *gcr, const uint8_t *ch_desc, uint8_t ch_desc_len)
{
	/* spec reference: TS 44.018 Section 9.1.21a */

	/* <Group Call Reference : bit(36)> */
	struct bitvec *gcr_bv = bitvec_alloc(5*8, NULL);
	OSMO_ASSERT(gcr_bv);
	bitvec_unpack(gcr_bv, gcr);
	for (unsigned int i = 0; i < 36; i++)
		bitvec_set_bit(bv, bitvec_get_bit_pos(gcr_bv, i));

	/* Group Channel Description */
	if (ch_desc && ch_desc_len) {
		struct bitvec *chd_bv = bitvec_alloc(ch_desc_len*8, NULL);
		OSMO_ASSERT(chd_bv);
		bitvec_set_bit(bv, 1);
		/* <Channel Description : bit(24)> */
		for (unsigned int i = 0; i < ch_desc_len * 8; i++)
			bitvec_set_bit(bv, bitvec_get_bit_pos(chd_bv, i));
		bitvec_free(chd_bv);
	} else {
		bitvec_set_bit(bv, 0);
	}

	bitvec_free(gcr_bv);
}

#if 0
foo()
{
	/* {0 I 1 < NLN(NCH) : bit (2) >} */
	if (nln_present) {
		bitvec_set_bit(bv, 1);
		bitvec_set_uint(bv, nln_nch, 2);
	} else {
		bitvec_set_bit(bv, 0);
	}

	/* <list of Group Call NCH information> := 0 | 1 < Group Call information > < List of Group Call NCH information > } ; */
	bitvec_set_bit(bv, 1);
	append_group_call_information(bv, FIXME);
	bitvec_set_bit(bv, 0);	/* only one element in the list */

	/* TODO: Additions in Release 6 */
	/* TODO: Additions in Release 7 */
}
#endif
