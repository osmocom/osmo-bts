/* Paging Rest Octets / ETWS support code */
/*
 * (C) 2014 by Harald Welte <laforge@gnumonks.org>
 *
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#include <osmocom/core/bits.h>
#include <osmocom/core/bitvec.h>

/*! \brief construct the Paging 1 Rest Octets
 * \param[in] bitvec Bit Vector (destionation)
 * \param[in] seg_nr segment number 0...n
 * \param[in] num_segs total number of segments
 * \param[in] payload binary payload for this segment
 * \param[in] len_of_seg length of the segment
 *
 * Put together the P1 Rest Octets according to 10.5.2.23
 */
int construct_p1_rest_octets(struct bitvec *bv, int etws_will_follow)
{

	/* no NLN */
	bitvec_set_bit(bv, L);
	/* no Priority 1 */
	bitvec_set_bit(bv, L);
	/* no Priority 2 */
	bitvec_set_bit(bv, L);
	/* Packet Page Indication 1 */
	bitvec_set_bit(bv, L);
	/* Packet Page Indication 2 */
	bitvec_set_bit(bv, L);

	/* No Rel6 extensions */
	bitvec_set_bit(bv, L);

	/* No Rel7 extensions */
	bitvec_set_bit(bv, L);

	/* Rel8 extensions */
	bitvec_set_bit(bv, H);

	/* priority_uplink_access bit: Use RACH */
	bitvec_set_bit(bv, L);

	if (etws_will_follow) {
		/* We do have ETWS Primary Notification */
		bitvec_set_bit(bv, ONE);
	} else {
		bitvec_set_bit(bv, ZERO);
	}

	return 0;
}

/*! \brief construct a ETWS Primary Notification struct
 * \param[in] bitvec Bit Vector (destionation)
 * \param[in] pni Primary Notification Identifier
 * \param[in] seg_nr Segment Number 0...n
 * \param[in] num_segs Total Number of Segments
 * \param[in] payload Binary Payload for this Segment
 * \param[in] num_payload_bits Length of the Segment
 */
int construct_etws_prim_notif(struct bitvec *bv, uint8_t pni,
			      uint8_t seg_nr, uint8_t num_segs,
			      const uint8_t *payload,
			      uint8_t num_payload_bits)
{
	uint8_t payload_ubits[num_payload_bits];

	/* Put together a "ETWS Primary Notification struct" as per TS
	 * 44.018 */

	if (seg_nr == 0) {
		bitvec_set_bit(bv, ZERO);
		bitvec_set_uint(bv, num_segs, 4);
	} else {
		bitvec_set_bit(bv, ONE);
		/* seg_nr is coounting 1..n, not 0..n */
		bitvec_set_uint(bv, seg_nr + 1, 4);
	}
	bitvec_set_bit(bv, pni ? ONE : ZERO);
	bitvec_set_uint(bv, num_payload_bits, 7);

	/* expand packed payload bits to unpacked bits and set them in
	 * the bit vector */
	osmo_pbit2ubit(payload_ubits, payload, num_payload_bits);
	bitvec_set_bits(bv, (enum bit_value *) payload_ubits, num_payload_bits);

	return 0;
}
