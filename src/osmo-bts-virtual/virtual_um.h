#pragma once

#include <osmocom/core/select.h>
#include <osmocom/core/msgb.h>

struct virt_um_inst {
	void *priv;
	struct osmo_fd ofd;
	void (*recv_cb)(struct virt_um_inst *vui, struct msgb *msg);
};

struct virt_um_inst *virt_um_init(void *ctx, const char *group, uint16_t port,
				  const char *netdev, void *priv,
				  void (*recv_cb)(struct virt_um_inst *vui, struct msgb *msg));

void virt_um_destroy(struct virt_um_inst *vui);

int virt_um_write_msg(struct virt_um_inst *vui, struct msgb *msg);
