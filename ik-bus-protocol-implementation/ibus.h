#ifndef _I_BUS_H_
#define _I_BUS_H_

/* TX_IDx1 LENx1 RX_IDx1 CMDx1 DATAx32 CHK_SUMx1 */
#define IBUS_MAX_DLEN 35
#define IBUS_MAX_FRAME_SIZE 37
#define SOL_IBUS 200
#define IBUS_ID_ALL 0xff

enum {
	IBUS_FILTER = 1,    /* set 0 .. n filter(s) */
	IBUS_ERR_FILTER,    /* set filter for error frames       */
	IBUS_RECV_OWN_MSGS, /* receive my own msgs (default:off) */
	IBUS_ONLY_DATA,     /* Only data part in stren */
};

/*
 * struct sockaddr_ibu - the sockaddr structure for IBUS sockets
 * @ibus_family:  address family number AF_IBUS.
 * @ifindex: CAN network interface index.
 * @can_addr:    protocol specific address information
 */
struct sockaddr_ibus {
	unsigned short int ibus_family;
	int ifindex;
} ibus_addr;

/*
 * struct ibus_filter - filter for receive frames
 * @id_rx: identifier of receiver, 0xff - all
 * @id_tx: identifier of sender, 0xff - all
 */
struct ibus_filter {
	u_int8_t id_rx;
	u_int8_t id_tx;
};

struct ibus_head {
	u_int8_t rx_id;
	u_int8_t len;
	u_int8_t tx_id;
	u_int8_t cmd;
};

struct ibus_frame {
	struct ibus_head *head;
	u_int8_t *data;
};

#endif /* _I_BUS_H_ */
