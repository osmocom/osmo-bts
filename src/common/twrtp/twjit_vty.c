/*
 * Themyscira Wireless jitter buffer implementation: vty configuration.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include <osmocom/core/utils.h>
#include <osmocom/vty/vty.h>
#include <osmocom/vty/command.h>

#include <themwi/rtp/twjit.h>

int twrtp_jibuf_config_write(struct vty *vty,
			     const struct twrtp_jibuf_config *conf,
			     const char *name, const char *prefix)
{
	vty_out(vty, "%s%s%s", prefix, name ? : "twjit", VTY_NEWLINE);
	vty_out(vty, "%s buffer-depth %u %u%s", prefix, conf->bd_start,
		conf->bd_hiwat, VTY_NEWLINE);
	vty_out(vty, "%s thinning-interval %u%s", prefix, conf->thinning_int,
		VTY_NEWLINE);
	vty_out(vty, "%s max-future-sec %u%s", prefix, conf->max_future_sec,
		VTY_NEWLINE);

	if (conf->start_min_delta) {
		vty_out(vty, "%s start-min-delta %u%s", prefix,
			conf->start_min_delta, VTY_NEWLINE);
	}
	if (conf->start_max_delta) {
		vty_out(vty, "%s start-max-delta %u%s", prefix,
			conf->start_max_delta, VTY_NEWLINE);
	}

	return CMD_SUCCESS;
}

DEFUN(cfg_buffer_depth, cfg_buffer_depth_cmd,
      "buffer-depth <2-65535> <2-65535>",
      "Buffer depth configuration\n"
      "Minimum fill required to start flow\n"
      "High water mark fill level\n")
{
	struct twrtp_jibuf_config *conf = vty->index;
	unsigned bd_start = atoi(argv[0]);
	unsigned bd_hiwat = atoi(argv[1]);

	if (bd_hiwat < bd_start) {
		vty_out(vty, "%% Error: high water mark cannot be less than starting level%s",
			VTY_NEWLINE);
		return CMD_WARNING;
	}

	conf->bd_start = bd_start;
	conf->bd_hiwat = bd_hiwat;

	return CMD_SUCCESS;
}

DEFUN(cfg_thinning, cfg_thinning_cmd,
      "thinning-interval <2-65535>",
      "Standing queue thinning configuration\n"
      "Drop every Nth packet\n")
{
	struct twrtp_jibuf_config *conf = vty->index;
	conf->thinning_int = atoi(argv[0]);
	return CMD_SUCCESS;
}

DEFUN(cfg_max_future, cfg_max_future_cmd,
      "max-future-sec <1-65535>",
      "Guard against time traveler packets\n"
      "Maximum permissible number of seconds into the future\n")
{
	struct twrtp_jibuf_config *conf = vty->index;
	conf->max_future_sec = atoi(argv[0]);
	return CMD_SUCCESS;
}

DEFUN(cfg_start_min_delta, cfg_start_min_delta_cmd,
      "start-min-delta <1-65535>",
      "Minimum required delta in time-of-arrival to start flow\n"
      "Time delta value in ms\n")
{
	struct twrtp_jibuf_config *conf = vty->index;
	conf->start_min_delta = atoi(argv[0]);
	return CMD_SUCCESS;
}

DEFUN(cfg_no_start_min_delta, cfg_no_start_min_delta_cmd,
      "no start-min-delta",
      NO_STR "Minimum required delta in time-of-arrival to start flow\n")
{
	struct twrtp_jibuf_config *conf = vty->index;
	conf->start_min_delta = 0;
	return CMD_SUCCESS;
}

DEFUN(cfg_start_max_delta, cfg_start_max_delta_cmd,
      "start-max-delta <1-65535>",
      "Maximum permitted gap in time-of-arrival in starting state\n"
      "Time delta value in ms\n")
{
	struct twrtp_jibuf_config *conf = vty->index;
	conf->start_max_delta = atoi(argv[0]);
	return CMD_SUCCESS;
}

DEFUN(cfg_no_start_max_delta, cfg_no_start_max_delta_cmd,
      "no start-max-delta",
      NO_STR "Maximum permitted gap in time-of-arrival in starting state\n")
{
	struct twrtp_jibuf_config *conf = vty->index;
	conf->start_max_delta = 0;
	return CMD_SUCCESS;
}

void twrtp_jibuf_vty_init(int twjit_node)
{
	install_lib_element(twjit_node, &cfg_buffer_depth_cmd);
	install_lib_element(twjit_node, &cfg_thinning_cmd);
	install_lib_element(twjit_node, &cfg_max_future_cmd);
	install_lib_element(twjit_node, &cfg_start_min_delta_cmd);
	install_lib_element(twjit_node, &cfg_no_start_min_delta_cmd);
	install_lib_element(twjit_node, &cfg_start_max_delta_cmd);
	install_lib_element(twjit_node, &cfg_no_start_max_delta_cmd);
}
