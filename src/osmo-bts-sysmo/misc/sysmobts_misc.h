#ifndef _SYSMOBTS_MISC_H
#define _SYSMOBTS_MISC_H

enum sysmobts_temp_sensor {
	SYSMOBTS_TEMP_DIGITAL = 1,
	SYSMOBTS_TEMP_RF = 2,
};

enum sysmobts_temp_type {
	SYSMOBTS_TEMP_INPUT,
	SYSMOBTS_TEMP_LOWEST,
	SYSMOBTS_TEMP_HIGHEST,
	_NUM_TEMP_TYPES
};

enum sbts2050_status_rqt {
	SBTS2050_STATUS_MASTER,
	SBTS2050_STATUS_SLAVE,
	SBTS2050_STATUS_PA
};

struct uc {
	int id;
	int fd;
	const char *path;
};

struct ucinfo {
	uint16_t id;
	int master;
	int slave;
	int pa;
};

enum sbts2050_alert_lvl {
	SBTS2050_WARN_ALERT,
	SBTS2050_SEVERE_ALERT
};

enum sbts2050_temp_sensor {
	SBTS2050_TEMP_BOARD,
	SBTS2050_TEMP_PA
};

struct sbts2050_config_info {
	char name_sensor[8];
	int temp_max_pa_warn_limit;
	int temp_min_pa_warn_limit;
	int temp_max_pa_severe_limit;
	int temp_min_pa_severe_limit;
	int temp_max_board_warn_limit;
	int temp_min_board_warn_limit;
	int temp_max_board_severe_limit;
	int temp_min_board_severe_limit;
	int reduce_max_power;
	int slave_power_act;
	int master_power_act;
	int pa_power_act;
	int temp_pa_cur;
	int temp_board_cur;
};

int sysmobts_temp_get(enum sysmobts_temp_sensor sensor,
		      enum sysmobts_temp_type type);

void sysmobts_check_temp(int no_eeprom_write);

void sbts2050_uc_check_temp(struct uc *ucontrol, int *temp_pa, int *temp_board);

void sbts2050_uc_power(struct uc *ucontrol, int pmaster, int pslave, int ppa);

int sbts2050_uc_status(struct uc *ucontrol, enum sbts2050_status_rqt status);

int send_omlfailure(int fd_unix, enum sbts2050_alert_lvl alert,
		   enum sbts2050_temp_sensor sensor,
		   struct sbts2050_config_info *add_info, int trx_nr);

int sysmobts_update_hours(int no_epprom_write);

enum sysmobts_firmware_type {
	SYSMOBTS_FW_FPGA,
	SYSMOBTS_FW_DSP,
	_NUM_FW
};

int sysmobts_firmware_reload(enum sysmobts_firmware_type type);

#endif
