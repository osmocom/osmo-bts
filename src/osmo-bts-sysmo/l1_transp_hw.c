/* Interface handler for Sysmocom L1 (real hardware) */

/* (C) 2011 by Harald Welte <laforge@gnumonks.org>
 *
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>

#include <sys/types.h>
#include <sys/stat.h>

#include <osmocom/core/talloc.h>
#include <osmocom/core/utils.h>
#include <osmocom/core/select.h>
#include <osmocom/core/write_queue.h>
#include <osmocom/gsm/gsm_utils.h>

#include <osmo-bts/logging.h>
#include <osmo-bts/gsm_data.h>

#include <sysmocom/femtobts/superfemto.h>
#include <sysmocom/femtobts/gsml1prim.h>
#include <sysmocom/femtobts/gsml1const.h>
#include <sysmocom/femtobts/gsml1types.h>

#include "femtobts.h"
#include "l1_if.h"
#include "l1_transp.h"


#ifdef HW_VERSION_1
#define DEV_SYS_DSP2ARM_NAME	"/dev/msgq/femtobts_dsp2arm"
#define DEV_SYS_ARM2DSP_NAME	"/dev/msgq/femtobts_arm2dsp"
#define DEV_L1_DSP2ARM_NAME	"/dev/msgq/gsml1_dsp2arm"
#define DEV_L1_ARM2DSP_NAME	"/dev/msgq/gsml1_arm2dsp"
#else
#define DEV_SYS_DSP2ARM_NAME	"/dev/msgq/superfemto_dsp2arm"
#define DEV_SYS_ARM2DSP_NAME	"/dev/msgq/superfemto_arm2dsp"
#define DEV_L1_DSP2ARM_NAME	"/dev/msgq/gsml1_sig_dsp2arm"
#define DEV_L1_ARM2DSP_NAME	"/dev/msgq/gsml1_sig_arm2dsp"
#endif

#define DEV_TCH_DSP2ARM_NAME	"/dev/msgq/gsml1_tch_dsp2arm"
#define DEV_TCH_ARM2DSP_NAME	"/dev/msgq/gsml1_tch_arm2dsp"
#define DEV_PDTCH_DSP2ARM_NAME	"/dev/msgq/gsml1_pdtch_dsp2arm"
#define DEV_PDTCH_ARM2DSP_NAME	"/dev/msgq/gsml1_pdtch_arm2dsp"

static const char *rd_devnames[] = {
	[MQ_SYS_READ]	= DEV_SYS_DSP2ARM_NAME,
	[MQ_L1_READ]	= DEV_L1_DSP2ARM_NAME,
	[MQ_TCH_READ]	= DEV_TCH_DSP2ARM_NAME,
	[MQ_PDTCH_READ]	= DEV_PDTCH_DSP2ARM_NAME,
};

static const char *wr_devnames[] = {
	[MQ_SYS_WRITE]	= DEV_SYS_ARM2DSP_NAME,
	[MQ_L1_WRITE]	= DEV_L1_ARM2DSP_NAME,
	[MQ_TCH_WRITE]	= DEV_TCH_ARM2DSP_NAME,
	[MQ_PDTCH_WRITE]= DEV_PDTCH_ARM2DSP_NAME,
};

/* callback when there's something to read from the l1 msg_queue */
static int l1if_fd_cb(struct osmo_fd *ofd, unsigned int what)
{
	//struct msgb *msg = l1p_msgb_alloc();
	struct msgb *msg = msgb_alloc_headroom(2048, 128, "1l_fd");
	struct femtol1_hdl *fl1h = ofd->data;
	int rc;

	msg->l1h = msg->data;
	rc = read(ofd->fd, msg->l1h, sizeof(GsmL1_Prim_t));
	if (rc < 0) {
		if (rc != -1) 
			LOGP(DL1C, LOGL_ERROR, "error reading from L1 msg_queue: %s\n",
				strerror(errno));
		msgb_free(msg);
		return rc;
	}
	msgb_put(msg, rc);

	switch (ofd->priv_nr) {
	case MQ_SYS_WRITE:
		return l1if_handle_sysprim(fl1h, msg);
	case MQ_L1_WRITE:
	case MQ_TCH_WRITE:
	case MQ_PDTCH_WRITE:
		return l1if_handle_l1prim(fl1h, msg);
	}
};

/* callback when we can write to one of the l1 msg_queue devices */
static int l1fd_write_cb(struct osmo_fd *ofd, struct msgb *msg)
{
	int rc;

	rc = write(ofd->fd, msg->l1h, msgb_l1len(msg));
	if (rc < 0) {
		LOGP(DL1C, LOGL_ERROR, "error writing to L1 msg_queue: %s\n",
			strerror(errno));
		return rc;
	} else if (rc < msg->len) {
		LOGP(DL1C, LOGL_ERROR, "short write to L1 msg_queue: "
			"%u < %u\n", rc, msg->len);
		return -EIO;
	}

	return 0;
}

int l1if_transport_open(struct femtol1_hdl *hdl)
{
	int rc, i;

	/* Step 1: Open all msg_queue file descriptors */
	for (i = 0; i < ARRAY_SIZE(hdl->read_ofd); i++) {
		struct osmo_fd *ofd = &hdl->read_ofd[i];

		rc = open(rd_devnames[i], O_RDONLY);
		if (rc < 0) {
			LOGP(DL1C, LOGL_FATAL, "unable to open msg_queue: %s\n",
				strerror(errno));
			goto out_free;
		}
		ofd->fd = rc;
		ofd->priv_nr = i;
		ofd->data = hdl;
		ofd->cb = l1if_fd_cb;
		ofd->when = BSC_FD_READ;
		rc = osmo_fd_register(ofd);
		if (rc < 0) {
			close(ofd->fd);
			ofd->fd = -1;
			goto out_free;
		}
	}
	for (i = 0; i < ARRAY_SIZE(hdl->write_q); i++) {
		struct osmo_wqueue *wq = &hdl->write_q[i];
		struct osmo_fd *ofd = &hdl->write_q[i].bfd;

		rc = open(wr_devnames[i], O_WRONLY);
		if (rc < 0) {
			LOGP(DL1C, LOGL_FATAL, "unable to open msg_queue: %s\n",
				strerror(errno));
			goto out_read;
		}

		osmo_wqueue_init(wq, 10);
		wq->write_cb = l1fd_write_cb;

		ofd->fd = rc;
		ofd->priv_nr = i;
		ofd->data = hdl;
		ofd->when = BSC_FD_WRITE;
		rc = osmo_fd_register(ofd);
		if (rc < 0) {
			close(ofd->fd);
			ofd->fd = -1;
			goto out_read;
		}

	}
	return 0;

out_read:
	for (i = 0; i < ARRAY_SIZE(hdl->read_ofd); i++) {
		close(hdl->read_ofd[i].fd);
		osmo_fd_unregister(&hdl->read_ofd[i]);
	}
out_free:
	talloc_free(hdl);
	return rc;
}

int l1if_transport_close(struct femtol1_hdl *hdl)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(hdl->read_ofd); i++) {
		struct osmo_fd *ofd = &hdl->read_ofd[i];

		osmo_fd_unregister(ofd);
		close(ofd->fd);
		ofd->fd = -1;
	}

	for (i = 0; i < ARRAY_SIZE(hdl->write_q); i++) {
		struct osmo_fd *ofd = &hdl->write_q[i].bfd;

		osmo_fd_unregister(ofd);
		close(ofd->fd);
		ofd->fd = -1;
	}
	return 0;
}
