/*
 * Here we implement the function that prepares the SDES subpacket
 * for subsequent RTCP output.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <arpa/inet.h>	/* for network byte order functions */

#include <osmocom/core/talloc.h>

#include <themwi/rtp/endp.h>
#include <themwi/rtp/rtcp_defs.h>

int twrtp_endp_set_sdes(struct twrtp_endp *endp, const char *cname,
			const char *name, const char *email, const char *phone,
			const char *loc, const char *tool, const char *note)
{
	uint16_t len_str, len_padded, len_with_hdr, len;
	struct rtcp_sr_rr_hdr *hdr;
	uint8_t *dp;

	if (!cname)
		return -EINVAL;
	len_str = strlen(cname) + 2;
	if (name)
		len_str += strlen(name) + 2;
	if (email)
		len_str += strlen(email) + 2;
	if (phone)
		len_str += strlen(phone) + 2;
	if (loc)
		len_str += strlen(loc) + 2;
	if (tool)
		len_str += strlen(tool) + 2;
	if (note)
		len_str += strlen(note) + 2;
	len_padded = (len_str + 4) & ~3;
	len_with_hdr = len_padded + sizeof(struct rtcp_sr_rr_hdr);

	if (endp->sdes_buf)
		talloc_free(endp->sdes_buf);
	endp->sdes_buf = talloc_size(endp, len_with_hdr);
	if (!endp->sdes_buf)
		return -ENOMEM;

	hdr = (struct rtcp_sr_rr_hdr *) endp->sdes_buf;
	hdr->v_p_rc = 0x81;
	hdr->pt = RTCP_PT_SDES;
	hdr->len = htons(len_with_hdr / 4 - 1);
	hdr->ssrc = htonl(endp->tx.ssrc);
	dp = endp->sdes_buf + sizeof(struct rtcp_sr_rr_hdr);
	*dp++ = SDES_ITEM_CNAME;
	*dp++ = len = strlen(cname);
	memcpy(dp, cname, len);
	dp += len;
	if (name) {
		*dp++ = SDES_ITEM_NAME;
		*dp++ = len = strlen(name);
		memcpy(dp, name, len);
		dp += len;
	}
	if (email) {
		*dp++ = SDES_ITEM_EMAIL;
		*dp++ = len = strlen(email);
		memcpy(dp, email, len);
		dp += len;
	}
	if (phone) {
		*dp++ = SDES_ITEM_PHONE;
		*dp++ = len = strlen(phone);
		memcpy(dp, phone, len);
		dp += len;
	}
	if (loc) {
		*dp++ = SDES_ITEM_LOC;
		*dp++ = len = strlen(loc);
		memcpy(dp, loc, len);
		dp += len;
	}
	if (tool) {
		*dp++ = SDES_ITEM_TOOL;
		*dp++ = len = strlen(tool);
		memcpy(dp, tool, len);
		dp += len;
	}
	if (note) {
		*dp++ = SDES_ITEM_NOTE;
		*dp++ = len = strlen(note);
		memcpy(dp, note, len);
		dp += len;
	}
	memset(dp, 0, len_padded - len_str);

	endp->sdes_len = len_with_hdr;
	return 0;
}
