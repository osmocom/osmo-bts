/* (C) 2012 by Harald Welte <laforge@gnumonks.org>
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
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <getopt.h>
#include <fcntl.h>
#include <limits.h>
#include <time.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <arpa/inet.h>

#include <osmocom/core/talloc.h>
#include <osmocom/core/utils.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/socket.h>
#include <osmocom/core/application.h>
#include <osmocom/vty/telnet_interface.h>
#include <osmocom/vty/logging.h>
#include <osmocom/gsm/abis_nm.h>
#include <osmocom/gsm/protocol/ipaccess.h>

#include <osmo-bts/bts.h>
#include <osmo-bts/oml.h>

#include "utils.h"
#include "btsconfig.h"
#include "sysmobts_misc.h"
#include "sysmobts_par.h"
#include "sysmobts_mgr.h"

#ifdef BUILD_SBTS2050
#include <sysmocom/femtobts/sbts2050_header.h>
#endif

#define SERIAL_ALLOC_SIZE	300
#define SIZE_HEADER_RSP		5
#define SIZE_HEADER_CMD		4
#define OM_ALLOC_SIZE		1024
#define OM_HEADROOM_SIZE	128

#ifdef BUILD_SBTS2050
static void add_sw_descr(struct msgb *msg)
{
	char file_version[255];
	char file_id[255];

	strncpy(file_id, "sysmomgr", strlen("sysmomgr"));
	file_id[sizeof(file_id) - 1] = '\0';
	strncpy(file_version, PACKAGE_VERSION, strlen(PACKAGE_VERSION));
	file_version[sizeof(file_version) - 1] = '\0';
	msgb_v_put(msg, NM_ATT_SW_DESCR);
	msgb_tl16v_put(msg, NM_ATT_FILE_ID, strlen(file_id),
		       (uint8_t *)file_id);
	msgb_tl16v_put(msg, NM_ATT_FILE_VERSION, strlen(file_version),
		       (uint8_t *)file_version);
}

static void add_probable_cause(struct msgb *msg)
{
	msgb_tv_put(msg, NM_ATT_PROB_CAUSE, NM_PCAUSE_T_MANUF);
	msgb_v_put(msg, 0);
	msgb_v_put(msg, 0);
}

static void add_oml_hdr_msg(struct msgb *msg, uint8_t msg_type,
			    uint8_t obj_class, uint8_t bts_nr,
			    uint8_t trx_nr, uint8_t ts_nr, int is_manuf)
{
	struct abis_om_fom_hdr *foh;
	struct abis_om_hdr *omh;

	msg->l3h = msgb_push(msg, sizeof(*foh));
	foh = (struct abis_om_fom_hdr *) msg->l3h;

	foh->msg_type = msg_type;
	foh->obj_class = obj_class;
	foh->obj_inst.bts_nr = bts_nr;
	foh->obj_inst.trx_nr = trx_nr;
	foh->obj_inst.ts_nr = ts_nr;

	if (is_manuf)
		oml_add_manufacturer_id_label(msg, MANUF_ID_OSMO);

	msg->l2h = msgb_push(msg, sizeof(*omh));
	omh = (struct abis_om_hdr *) msg->l2h;

	if (is_manuf)
		omh->mdisc = ABIS_OM_MDISC_MANUF;
	else
		omh->mdisc = ABIS_OM_MDISC_FOM;
	omh->placement = ABIS_OM_PLACEMENT_ONLY;
	omh->sequence = 0;
	omh->length = msgb_l3len(msg);
}

int send_omlfailure(int fd_unix, enum sbts2050_alert_lvl alert,
		   enum sbts2050_temp_sensor sensor,
		   struct sbts2050_config_info *add_info, int trx_nr)
{
	int rc;
	struct msgb *msg;
	const char *buf, *nsensor;

	msg = msgb_alloc_headroom(OM_ALLOC_SIZE, OM_HEADROOM_SIZE, "OML");
	if (msg == NULL) {
		LOGP(DTEMP, LOGL_ERROR, "Failed to allocate oml msgb\n");
		return -1;
	}

	add_oml_hdr_msg(msg, NM_MT_FAILURE_EVENT_REP, 0, 0, trx_nr, 255, 0);

	msgb_tv_put(msg, NM_ATT_EVENT_TYPE, NM_EVT_ENV_FAIL);

	switch (alert) {
	case SBTS2050_WARN_ALERT:
		msgb_tv_put(msg, NM_ATT_SEVERITY, NM_SEVER_WARNING);
		break;
	case SBTS2050_SEVERE_ALERT:
		msgb_tv_put(msg, NM_ATT_SEVERITY, NM_SEVER_CRITICAL);
		break;
	default:
		LOGP(DTEMP, LOGL_ERROR, "Unknown attr severity type %d\n",
		     alert);
		goto err;
	}

	add_probable_cause(msg);

	add_sw_descr(msg);

	switch (sensor) {
	case SBTS2050_TEMP_BOARD:
		buf = "Unusual temperature on the Board";
		nsensor = "Board";
		break;
	case SBTS2050_TEMP_PA:
		buf = "Unusual temperature on the PA";
		nsensor = "PA";
		break;
	default:
		LOGP(DTEMP, LOGL_ERROR, "Unknown sensor type %d\n", sensor);
		goto err;
	}
	strncpy(add_info->name_sensor, nsensor, sizeof(add_info->name_sensor));
	add_info->name_sensor[sizeof(add_info->name_sensor) - 1] = '\0';

	msgb_tl16v_put(msg, NM_ATT_ADD_TEXT, strlen(buf), (const uint8_t *)buf);

	msgb_tl16v_put(msg, NM_ATT_ADD_INFO,
		       sizeof(struct sbts2050_config_info),
		       (const uint8_t *)add_info);

	prepend_oml_ipa_header(msg);

	rc = send(fd_unix, msg->data, msg->len, 0);
	if (rc < 0 || rc != msg->len) {
		LOGP(DTEMP, LOGL_ERROR,
		     "send error %s during send the Failure Event Report msg\n",
		     strerror(errno));
		close(fd_unix);
		msgb_free(msg);
		return SYSMO_MGR_DISCONNECTED;
	}

	msgb_free(msg);
	return SYSMO_MGR_CONNECTED;
err:
	msgb_free(msg);
	return -1;
}

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
#endif

/**********************************************************************
 *	Get power status function
 *********************************************************************/
int sbts2050_uc_status(struct uc *ucontrol, enum sbts2050_status_rqt status)
{
#ifdef BUILD_SBTS2050
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
#else
	return -1;
#endif
}

/**********************************************************************
 *	Uc Power Switching handling
 *********************************************************************/
void sbts2050_uc_power(struct uc *ucontrol, int pmaster, int pslave, int ppa)
{
#ifdef BUILD_SBTS2050
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
#endif
}

/**********************************************************************
 *	Uc temperature handling
 *********************************************************************/
void sbts2050_uc_check_temp(struct uc *ucontrol, int *temp_pa, int *temp_board)
{
#ifdef BUILD_SBTS2050
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
#endif
}

/*********************************************************************
 * Temperature handling
 *********************************************************************/

#define TEMP_PATH	"/sys/class/hwmon/hwmon0/device/temp%u_%s"

static const char *temp_type_str[_NUM_TEMP_TYPES] = {
	[SYSMOBTS_TEMP_INPUT] = "input",
	[SYSMOBTS_TEMP_LOWEST] = "lowest",
	[SYSMOBTS_TEMP_HIGHEST] = "highest",
};

int sysmobts_temp_get(enum sysmobts_temp_sensor sensor,
		      enum sysmobts_temp_type type)
{
	char buf[PATH_MAX];
	char tempstr[8];
	int fd, rc;

	if (sensor < SYSMOBTS_TEMP_DIGITAL ||
	    sensor > SYSMOBTS_TEMP_RF)
		return -EINVAL;

	if (type >= ARRAY_SIZE(temp_type_str))
		return -EINVAL;

	snprintf(buf, sizeof(buf)-1, TEMP_PATH, sensor, temp_type_str[type]);
	buf[sizeof(buf)-1] = '\0';

	fd = open(buf, O_RDONLY);
	if (fd < 0)
		return fd;

	rc = read(fd, tempstr, sizeof(tempstr));
	tempstr[sizeof(tempstr)-1] = '\0';
	if (rc < 0) {
		close(fd);
		return rc;
	}
	if (rc == 0) {
		close(fd);
		return -EIO;
	}

	close(fd);

	return atoi(tempstr);
}

static const struct {
	const char *name;
	enum sysmobts_temp_sensor sensor;
	enum sysmobts_par ee_par;
} temp_data[] = {
	{
		.name = "digital",
		.sensor = SYSMOBTS_TEMP_DIGITAL,
		.ee_par = SYSMOBTS_PAR_TEMP_DIG_MAX,
	}, {
		.name = "rf",
		.sensor = SYSMOBTS_TEMP_RF,
		.ee_par = SYSMOBTS_PAR_TEMP_RF_MAX,
	}
};

void sysmobts_check_temp(int no_eeprom_write)
{
	int temp_old[ARRAY_SIZE(temp_data)];
	int temp_hi[ARRAY_SIZE(temp_data)];
	int temp_cur[ARRAY_SIZE(temp_data)];
	int i, rc;

	for (i = 0; i < ARRAY_SIZE(temp_data); i++) {
		int ret;
		rc = sysmobts_par_get_int(temp_data[i].ee_par, &ret);
		temp_old[i] = ret * 1000;
		temp_hi[i] = sysmobts_temp_get(temp_data[i].sensor,
						SYSMOBTS_TEMP_HIGHEST);
		temp_cur[i] = sysmobts_temp_get(temp_data[i].sensor,
						SYSMOBTS_TEMP_INPUT);

		if ((temp_cur[i] < 0 && temp_cur[i] > -1000) ||
		    (temp_hi[i] < 0 && temp_hi[i] > -1000)) {
			LOGP(DTEMP, LOGL_ERROR, "Error reading temperature\n");
			return;
		}

		LOGP(DTEMP, LOGL_DEBUG, "Current %s temperature: %d.%d C\n",
		     temp_data[i].name, temp_cur[i]/1000, temp_cur[i]%1000);

		if (temp_hi[i] > temp_old[i]) {
			LOGP(DTEMP, LOGL_NOTICE, "New maximum %s "
			     "temperature: %d.%d C\n", temp_data[i].name,
			     temp_hi[i]/1000, temp_hi[i]%1000);

			if (!no_eeprom_write) {
				rc = sysmobts_par_set_int(SYSMOBTS_PAR_TEMP_DIG_MAX,
						  temp_hi[0]/1000);
				if (rc < 0)
					LOGP(DTEMP, LOGL_ERROR, "error writing new %s "
					     "max temp %d (%s)\n", temp_data[i].name,
					     rc, strerror(errno));
			}
		}
	}
}

/*********************************************************************
 * Hours handling
 *********************************************************************/
static time_t last_update;

int sysmobts_update_hours(int no_eeprom_write)
{
	time_t now = time(NULL);
	int rc, op_hrs;

	/* first time after start of manager program */
	if (last_update == 0) {
		last_update = now;

		rc = sysmobts_par_get_int(SYSMOBTS_PAR_HOURS, &op_hrs);
		if (rc < 0) {
			LOGP(DTEMP, LOGL_ERROR, "Unable to read "
			     "operational hours: %d (%s)\n", rc,
			     strerror(errno));
			return rc;
		}

		LOGP(DTEMP, LOGL_INFO, "Total hours of Operation: %u\n",
		     op_hrs);

		return 0;
	}

	if (now >= last_update + 3600) {
		rc = sysmobts_par_get_int(SYSMOBTS_PAR_HOURS, &op_hrs);
		if (rc < 0) {
			LOGP(DTEMP, LOGL_ERROR, "Unable to read "
			     "operational hours: %d (%s)\n", rc,
			     strerror(errno));
			return rc;
		}

		/* number of hours to increase */
		op_hrs += (now-last_update)/3600;

		LOGP(DTEMP, LOGL_INFO, "Total hours of Operation: %u\n",
		     op_hrs);

		if (!no_eeprom_write) {
			rc = sysmobts_par_set_int(SYSMOBTS_PAR_HOURS, op_hrs);
			if (rc < 0)
				return rc;
		}

		last_update = now;
	}

	return 0;
}

/*********************************************************************
 * Firmware reloading
 *********************************************************************/

#define SYSMOBTS_FW_PATH	"/lib/firmware"

static const char *fw_names[_NUM_FW] = {
	[SYSMOBTS_FW_FPGA]	= "sysmobts-v2.bit",
	[SYSMOBTS_FW_DSP]	= "sysmobts-v2.out",
};
static const char *fw_devs[_NUM_FW] = {
	[SYSMOBTS_FW_FPGA]	= "/dev/fpgadl_par0",
	[SYSMOBTS_FW_DSP]	= "/dev/dspdl_dm644x_0",
};

int sysmobts_firmware_reload(enum sysmobts_firmware_type type)
{
	char name[PATH_MAX];
	uint8_t buf[1024];
	int fd_in, fd_out, rc;

	if (type >= _NUM_FW)
		return -EINVAL;

	snprintf(name, sizeof(name)-1, "%s/%s",
		 SYSMOBTS_FW_PATH, fw_names[type]);
	name[sizeof(name)-1] = '\0';

	fd_in = open(name, O_RDONLY);
	if (fd_in < 0) {
		LOGP(DFW, LOGL_ERROR, "unable ot open firmware file %s: %s\n",
		     name, strerror(errno));
		return fd_in;
	}

	fd_out = open(fw_devs[type], O_WRONLY);
	if (fd_out < 0) {
		LOGP(DFW, LOGL_ERROR, "unable ot open firmware device %s: %s\n",
		     fw_devs[type], strerror(errno));
		close(fd_in);
		return fd_out;
	}

	while ((rc = read(fd_in, buf, sizeof(buf)))) {
		int written;

		if (rc < 0) {
			LOGP(DFW, LOGL_ERROR, "error %d during read "
			     "from %s: %s\n", rc, name, strerror(errno));
			close(fd_in);
			close(fd_out);
			return -EIO;
		}

		written	= write(fd_out, buf, rc);
		if (written < rc) {
			LOGP(DFW, LOGL_ERROR, "short write during "
			     "fw write to %s\n", fw_devs[type]);
			close(fd_in);
			close(fd_out);
			return -EIO;
		}
	}

	close(fd_in);
	close(fd_out);

	return 0;
}
