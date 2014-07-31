/* (C) 2014 by s.f.m.c. GmbH
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

#include "sysmobts_misc.h"
#include "sysmobts_par.h"
#include "sysmobts_mgr.h"
#include "btsconfig.h"

#include <osmocom/core/logging.h>
#include <osmocom/core/msgb.h>

#include <errno.h>
#include <unistd.h>

#ifdef BUILD_SBTS2050
#include <sysmocom/femtobts/sbts2050_header.h>

#define SERIAL_ALLOC_SIZE	300
#define SIZE_HEADER_RSP		5
#define SIZE_HEADER_CMD		4


/**********************************************************************
 *	Functions read/write from serial interface
 *********************************************************************/
static int hand_serial_read(int fd, struct msgb *msg, int numbytes)
{
	int rc, bread = 0;

	if (numbytes > msgb_tailroom(msg))
		return -ENOSPC;

	while (bread < numbytes) {
		rc = read(fd, msg->tail, numbytes - bread);
		if (rc < 0)
			return -1;
		if (rc == 0)
			break;

		bread += rc;
		msgb_put(msg, rc);
	}

	return bread;
}

static int hand_serial_write(int fd, struct msgb *msg)
{
	int rc, bwritten = 0;

	while (msg->len > 0) {
		rc = write(fd, msg->data, msg->len);
		if (rc <= 0)
			return -1;

		msgb_pull(msg, rc);
		bwritten += rc;
	}

	return bwritten;
}

/**********************************************************************
 *	Functions request information to Microcontroller
 *********************************************************************/
static void add_parity(cmdpkt_t *command)
{
	int n;
	uint8_t parity = 0x00;
	for (n = 0; n < SIZE_HEADER_CMD+command->u8Len; n++)
		parity ^= ((uint8_t *)command)[n];

	command->cmd.raw[command->u8Len] = parity;
}

static struct msgb *sbts2050_ucinfo_get(struct uc *ucontrol, struct ucinfo info)
{
	int num, rc;
	cmdpkt_t *command;
	rsppkt_t *response;
	struct msgb *msg;
	fd_set fdread;
	struct timeval tout = {
		.tv_sec = 10,
	};

	switch (info.id) {
	case SBTS2050_TEMP_RQT:
		num = sizeof(command->cmd.tempGet);
		break;
	case SBTS2050_PWR_RQT:
		num = sizeof(command->cmd.pwrSetState);
		break;
	case SBTS2050_PWR_STATUS:
		num = sizeof(command->cmd.pwrGetStatus);
		break;
	default:
		return NULL;
	}
	num = num + SIZE_HEADER_CMD+1;

	msg = msgb_alloc(SERIAL_ALLOC_SIZE, "Message Microcontroller");
	if (msg == NULL) {
		LOGP(DTEMP, LOGL_ERROR, "Error creating msg\n");
		return NULL;
	}
	command = (cmdpkt_t *) msgb_put(msg, num);

	command->u16Magic = 0xCAFE;
	switch (info.id) {
	case SBTS2050_TEMP_RQT:
		command->u8Id = info.id;
		command->u8Len = sizeof(command->cmd.tempGet);
		break;
	case SBTS2050_PWR_RQT:
		command->u8Id = info.id;
		command->u8Len = sizeof(command->cmd.pwrSetState);
		command->cmd.pwrSetState.u1MasterEn = !!info.master;
		command->cmd.pwrSetState.u1SlaveEn  = !!info.slave;
		command->cmd.pwrSetState.u1PwrAmpEn = !!info.pa;
		break;
	case SBTS2050_PWR_STATUS:
		command->u8Id     = info.id;
		command->u8Len    = sizeof(command->cmd.pwrGetStatus);
		break;
	default:
		goto err;
	}

	add_parity(command);

	if (hand_serial_write(ucontrol->fd, msg) < 0)
		goto err;

	msgb_reset(msg);

	FD_ZERO(&fdread);
	FD_SET(ucontrol->fd, &fdread);

	num = SIZE_HEADER_RSP;
	while (1) {
		rc = select(ucontrol->fd+1, &fdread, NULL, NULL, &tout);
		if (rc > 0) {
			if (hand_serial_read(ucontrol->fd, msg, num) < 0)
				goto err;

			response = (rsppkt_t *)msg->data;

			if (response->u8Id != info.id || msg->len <= 0 ||
			    response->i8Error != RQT_SUCCESS)
				goto err;

			if (msg->len == SIZE_HEADER_RSP + response->u8Len + 1)
				break;

			num = response->u8Len + 1;
		} else
			goto err;
	}

	return msg;

err:
	msgb_free(msg);
	return NULL;
}

/**********************************************************************
 *	Get power status function
 *********************************************************************/
int sbts2050_uc_status(struct uc *ucontrol, enum sbts2050_status_rqt status)
{
	struct msgb *msg;
	struct ucinfo info = {
		.id = SBTS2050_PWR_STATUS,
	};
	rsppkt_t *response;
	int val_status;

	msg = sbts2050_ucinfo_get(ucontrol, info);

	if (msg == NULL) {
		LOGP(DTEMP, LOGL_ERROR, "Error switching off some unit");
		return -1;
	}

	response = (rsppkt_t *)msg->data;

	switch (status) {
	case SBTS2050_STATUS_MASTER:
		val_status = response->rsp.pwrGetStatus.u1MasterEn;
		break;
	case SBTS2050_STATUS_SLAVE:
		val_status = response->rsp.pwrGetStatus.u1SlaveEn;
		break;
	case SBTS2050_STATUS_PA:
		val_status = response->rsp.pwrGetStatus.u1PwrAmpEn;
		break;
	default:
		msgb_free(msg);
		return -1;
	}
	msgb_free(msg);
	return val_status;
}

/**********************************************************************
 *	Uc Power Switching handling
 *********************************************************************/
void sbts2050_uc_power(struct uc *ucontrol, int pmaster, int pslave, int ppa)
{
	struct msgb *msg;
	struct ucinfo info = {
		.id = 0x00,
		.master = pmaster,
		.slave = pslave,
		.pa = ppa
	};

	msg = sbts2050_ucinfo_get(ucontrol, info);

	if (msg == NULL) {
		LOGP(DTEMP, LOGL_ERROR, "Error switching off some unit");
		return;
	}

	LOGP(DTEMP, LOGL_DEBUG, "Switch off/on success:\n"
				"MASTER %s\n"
				"SLAVE %s\n"
				"PA %s\n",
				pmaster ? "ON" : "OFF",
				pslave ? "ON" : "OFF",
				ppa ? "ON" : "OFF");

	msgb_free(msg);
}

/**********************************************************************
 *	Uc temperature handling
 *********************************************************************/
void sbts2050_uc_check_temp(struct uc *ucontrol, int *temp_pa, int *temp_board)
{
	rsppkt_t *response;
	struct msgb *msg;
	struct ucinfo info = {
		.id = SBTS2050_TEMP_RQT,
	};

	msg = sbts2050_ucinfo_get(ucontrol, info);

	if (msg == NULL) {
		LOGP(DTEMP, LOGL_ERROR, "Error reading temperature\n");
		return;
	}

	response = (rsppkt_t *)msg->data;

	*temp_board = response->rsp.tempGet.i8BrdTemp;
	*temp_pa = response->rsp.tempGet.i8PaTemp;

	LOGP(DTEMP, LOGL_DEBUG, "Temperature Board: %+3d C\n"
				"Tempeture PA: %+3d C\n",
				 response->rsp.tempGet.i8BrdTemp,
				 response->rsp.tempGet.i8PaTemp);
	msgb_free(msg);
}
#endif
