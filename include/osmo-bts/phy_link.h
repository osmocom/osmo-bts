#pragma once

#include <stdint.h>
#include <osmocom/core/linuxlist.h>

#include <osmo-bts/scheduler.h>

#include <linux/if_packet.h>

struct gsm_bts_trx;

enum phy_link_type {
	PHY_LINK_T_NONE,
	PHY_LINK_T_SYSMOBTS,
	PHY_LINK_T_OSMOTRX,
};

enum phy_link_state {
	PHY_LINK_SHUTDOWN,
	PHY_LINK_CONNECTING,
	PHY_LINK_CONNECTED,
};

/* A PHY link represents the connection to a given PHYsical layer
 * implementation.  That PHY link contains 1...N PHY instances, one for
 * each TRX */
struct phy_link {
	struct llist_head list;
	int num;
	enum phy_link_type type;
	enum phy_link_state state;
	struct llist_head instances;
	char *description;
	union {
		struct {
		} sysmobts;
		struct {
			char *transceiver_ip;
			uint16_t base_port_local;
			uint16_t base_port_remote;
			struct osmo_fd trx_ofd_clk;

			uint32_t clock_advance;
			uint32_t rts_advance;

			int	rxgain_valid;
			int	rxgain;
			int	rxgain_sent;

			int	power_valid;
			int	power;
			int	power_oml;
			int	power_sent;
		} osmotrx;
		struct {
			/* MAC address of the PHY */
			struct sockaddr_ll phy_addr;
			/* Network device name */
			char *netdev_name;

			/* configuration */
			uint32_t rf_port_index;
			uint32_t rx_gain_db;
			uint32_t tx_atten_db;
			/* arfcn used by TRX with id 0 */
			uint16_t center_arfcn;
			struct octphy_hdl *hdl;
		} octphy;
	} u;
};

struct phy_instance {
	/* liked inside phy_link.linstances */
	struct llist_head list;
	int num;
	char *description;

	/* pointer to the PHY link to which we belong */
	struct phy_link *phy_link;

	/* back-pointer to the TRX to which we're associated */
	struct gsm_bts_trx *trx;

	union {
		struct {
			/* configuration */
			uint8_t clk_use_eeprom;
			uint32_t dsp_trace_f;
			int clk_cal;
			uint8_t clk_src;
			char *calib_path;

			struct femtol1_hdl *hdl;
		} sysmobts;
		struct {
			struct trx_l1h *hdl;
		} osmotrx;
		struct {
			/* logical transceiver number within one PHY */
			uint32_t trx_id;
		} octphy;
		struct {
			/* configuration */
			uint32_t dsp_trace_f;
			char *calib_path;
			int minTxPower;
			int maxTxPower;
			struct lc15l1_hdl *hdl;
		} lc15;
	} u;
};

struct phy_link *phy_link_by_num(int num);
struct phy_link *phy_link_create(void *ctx, int num);
void phy_link_destroy(struct phy_link *plink);
void phy_link_state_set(struct phy_link *plink, enum phy_link_state state);
int phy_links_open(void);

struct phy_instance *phy_instance_by_num(struct phy_link *plink, int num);
struct phy_instance *phy_instance_create(struct phy_link *plink, int num);
void phy_instance_link_to_trx(struct phy_instance *pinst, struct gsm_bts_trx *trx);
void phy_instance_destroy(struct phy_instance *pinst);
const char *phy_instance_name(struct phy_instance *pinst);

void phy_user_statechg_notif(struct phy_instance *pinst, enum phy_link_state link_state);

static inline struct phy_instance *trx_phy_instance(struct gsm_bts_trx *trx)
{
	OSMO_ASSERT(trx);
	return trx->role_bts.l1h;
}

int bts_model_phy_link_open(struct phy_link *plink);
