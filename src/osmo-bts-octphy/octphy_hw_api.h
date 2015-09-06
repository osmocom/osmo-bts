#pragma once

#include <stdint.h>
#include "l1_if.h"

int octphy_hw_get_pcb_info(struct octphy_hdl *fl1h);
int octphy_hw_get_rf_port_info(struct octphy_hdl *fl1h, uint32_t index);
int octphy_hw_get_rf_port_stats(struct octphy_hdl *fl1h, uint32_t index);
int octphy_hw_get_rf_ant_rx_config(struct octphy_hdl *fl1h, uint32_t port_idx,
				   uint32_t ant_idx);
int octphy_hw_get_rf_ant_tx_config(struct octphy_hdl *fl1h, uint32_t port_idx,
				   uint32_t ant_idx);
int octphy_hw_get_clock_sync_info(struct octphy_hdl *fl1h);
int octphy_hw_get_clock_sync_stats(struct octphy_hdl *fl1h);
