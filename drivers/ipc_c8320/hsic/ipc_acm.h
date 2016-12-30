#ifndef _IPC_ACM_H
#define _IPC_ACM_H
#include "../../usb/class/mdm8320_acm.h"
/*
struct acm_rb {
	struct list_head	list;
	int			size;
	unsigned char		*base;
	dma_addr_t		dma;
};
*/
enum acm_state{
	ACM_NONE,
	ACM_PROBE ,
	ACM_REMOVE,
	ACM_SUSPEND,
	ACM_RESUME,
	ACM_RESETRESUME,
	ACM_MAX,
};

struct acm_ch {
	struct list_head ch_list;

	char name[32];
	unsigned int cid;
	
	void *priv;

	int (*rx_cb)(void * priv, struct acm_rb *buf);	
	void (*wr_cb)(void*);
	
	int (*open)(struct acm_ch *ch);
	void (*close)(struct acm_ch *ch);
	int (*is_connected)(struct acm_ch *ch);
	int (*is_opened)(struct acm_ch *ch);
	int (*write_avail)(struct acm_ch *ch);
	int (*get_write_size)(struct acm_ch *ch);
	int (*read_done)(struct acm_ch *ch, struct acm_rb *buf);
	int (*notify)(void *priv, enum acm_state type);

	int (*write)(struct acm_ch *ch, const unsigned char *data, int len);
	unsigned state;
};

struct acm_ch* acm_register(unsigned int cid, void (**low_notify)(struct acm_ch *),  int (*rxcb)(void * priv, struct acm_rb *buf), void(*wrcb)(void*), int (*notify)(void * priv,enum acm_state type), void *priv);
#endif
