#include <errno.h>

#include <osmocom/core/msgb.h>
#include <osmocom/trau/rtp.h>

#include <osmo-bts/gsm_data.h>

/* input of an uplink received GSM codec frame
 * this is called by the BTS L1 code after a uplink voice frame was
 * received and has to be transmitted towards the TRAU */
int lchan_codec_up(struct gsm_lchan *lchan, struct msgb *msg)
{
	struct rtp_socket *rs = lchan->abis_ip.rtp_socket;

	if (!rs)
		return -ENODEV;

	return rtp_socket_send(rs, msg);
}
