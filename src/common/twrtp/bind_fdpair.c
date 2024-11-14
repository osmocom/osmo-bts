/*
 * Here we implement the function that creates and binds a pair of
 * UDP sockets for RTP & RTCP, given the bind IP address in string form
 * and the even port number in integer form.
 *
 * The current implementation supports only IPv4; however, given that
 * the API takes a string for the IP address, it should be possible
 * to extend this function to support IPv6 if and when such support
 * becomes necessary.
 */

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>

#include <themwi/rtp/fdpair.h>

int twrtp_bind_fdpair(const char *ip, uint16_t port, int *fd_rtp, int *fd_rtcp)
{
	struct sockaddr_in sin;
	int rc;

	sin.sin_family = AF_INET;
	rc = inet_aton(ip, &sin.sin_addr);
	if (!rc)
		return -EINVAL;

	/* do RTP socket first */
	rc = socket(AF_INET, SOCK_DGRAM, 0);
	if (rc < 0)
		return -errno;
	*fd_rtp = rc;
	sin.sin_port = htons(port);
	rc = bind(*fd_rtp, (struct sockaddr *) &sin, sizeof sin);
	if (rc < 0) {
		rc = -errno;
		close(*fd_rtp);
		return rc;
	}

	/* now do RTCP */
	rc = socket(AF_INET, SOCK_DGRAM, 0);
	if (rc < 0) {
		rc = -errno;
		close(*fd_rtp);
		return rc;
	}
	*fd_rtcp = rc;
	sin.sin_port = htons(port + 1);
	rc = bind(*fd_rtcp, (struct sockaddr *) &sin, sizeof sin);
	if (rc < 0) {
		rc = -errno;
		close(*fd_rtp);
		close(*fd_rtcp);
		return rc;
	}

	return 0;
}
