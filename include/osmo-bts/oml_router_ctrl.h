#ifndef OML_ROUTER_CTRL_H
#define OML_ROUTER_CTRL_H

enum osmo_omlrctrl_msgtype {
	OSMO_ORC_MSGT_REGISTER_REQ	= 1,
	OSMO_ORC_MSGT_REGISTER_ACK	= 2,
	OSMO_ORC_MSGT_REGISTER_NACK	= 3,

	OSMO_ORC_MSGT_ROUTE_ADD_REQ	= 4,
	OSMO_ORC_MSGT_ROUTE_ADD_ACK	= 5,
	OSMO_ORC_MSGT_ROUTE_ADD_NACK	= 6,

	OSMO_ORC_MSGT_ROUTE_DEL_REQ	= 7,
	OSMO_ORC_MSGT_ROUTE_DEL_ACK	= 8,
	OSMO_ORC_MSGT_ROUTE_DEL_NACK	= 9,
};

struct osmo_omlrctrl_hdr {
	uint8_t version;
	uint8_t msg_type;
	uint16_t data_len;
	uint8_t data[0];
} __attribute__((packed));

struct osmo_omlrctrl_register_req {
	uint8_t name_len;
	char name[0];
} __attribute__((packed));

#endif
