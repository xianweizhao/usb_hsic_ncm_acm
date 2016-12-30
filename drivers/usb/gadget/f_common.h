/*
  *   f_common.h  /drivers/usb/gadget/f_common.h         add by cool_zhao 2012-01-30
  *
  *
  */
#ifndef _F_COMMON_H
#define _F_COMMON_H

#include <linux/blkdev.h>
#include <linux/completion.h>
#include <linux/dcache.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fcntl.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/kref.h>
#include <linux/kthread.h>
#include <linux/limits.h>
#include <linux/rwsem.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/freezer.h>
#include <linux/utsname.h>
#include <linux/interrupt.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/composite.h>

#include "gadget_chips.h"
#define MAX_W_BUF       16

struct common_dev;
struct common_descs {
	struct usb_endpoint_descriptor	*in;
	struct usb_endpoint_descriptor	*out;
};

struct common_buf {
	unsigned		buf_size;
	int 			data_len;
	int 			packet_xmit;
	char			*buf_buf;
	char			*buf_get;
	char			*buf_put;
	bool             used;
};
struct common_req{
	int           used;
	char         *buf;
};
/*-------------------------------------------------------------------------*/
struct common_operations {
	int (*open)(struct common_dev *com);
	int (*close)(struct common_dev *com);
	int (*read)(struct common_dev *com, char *buf, int len);
	int (*write)(struct common_dev *com, char *buf, int len);
	unsigned (*buf_data_avail)(struct common_dev *com, int rw);
	unsigned (*buf_space_avail)(struct common_dev *com, int rw);
};
struct common_dev {
	struct usb_function	function;
	struct usb_gadget	*gadget;	/* Copy of cdev->gadget */
	int com_enabled;
	char * dev_name;                     //common dev name     but function name is  dev_name add port_number---add by cool_zhao 
	
	spinlock_t   lock;
	int port_num;                  //one  class common device number ,example ps has three devices ,port_num is 0,1,2,---add by cool_zhao

	int interface_number ;       //this is interface number of configure;C6320  max surport  7 
	int dev_number;              //commondev group index;
	
	/* port is managed by gserial_{connect,disconnect} */
	struct usb_ep			*in;
	struct usb_ep			*out;
	
	int 	bulk_in_enabled;
	int 	bulk_out_enabled;

	struct list_head	read_pool;
	int read_started;
	int read_allocated;
	struct list_head	read_queue;
	unsigned		n_read;


	struct list_head	write_pool;
	int write_started;
	int write_allocated;
	
	wait_queue_head_t  write_wait;
	wait_queue_head_t   close_wait;
	wait_queue_head_t   read_wait;	

	struct common_descs		fs;
	struct common_descs		hs;

	unsigned int			bulk_out_maxpacket;
	unsigned long       		len;

	struct common_buf            write_buf[MAX_W_BUF];
	unsigned int 				 wbuf_index;
	unsigned int				 xmit_index;
	struct common_buf 		read_buf;
	spinlock_t    buf_lock_read;
	spinlock_t    buf_lock_write;
	
	struct tasklet_struct	push;
		
	bool openclose;
	bool push_enabled;
	int open_count;
	struct common_operations *ops;
	int read_block_flag;
	struct usb_common_driver *dev_driver;   //commdev  driver
	struct list_head  list;  				//this list is added into driver list_head
	int used;
	int channel_id;

};
struct usb_common_driver{	
	struct list_head	list;
	char *name;
	int  (*probe) (struct common_dev *com);       //bind
	int  (*remove)(struct common_dev *com);      //unbind
	int  (*connect) (struct common_dev *com);	//enum complete
	int  (*disconnect)(struct common_dev *com);   //usb disconnect 
	struct list_head	dev_list;
	
	#ifdef COMMON_POWER	
	int (*suspend) (*probe)(common_dev * com);		//usb suspend
	int (*resume)(*resume)(common_dev * com);		//usb resume 
	#endif

};

static inline int usb_common_dev_open(struct common_dev *com)
{
	return com->ops->open(com);
		
}

static inline int usb_common_dev_close(struct common_dev *com)
{
	return com->ops->close(com);
}

static inline int usb_common_dev_read(struct common_dev *com, char *buf, int len)
{
	return com->ops->read(com,buf,len);
}

static inline int usb_common_dev_write(struct common_dev *com, char *buf, int len)
{
	return com->ops->write(com,buf,len);
}

static inline unsigned usb_common_readbuf_data_avail(struct common_dev *com)
{
	return com->ops->buf_data_avail(com,0);
}

static inline unsigned usb_common_readbuf_space_avail(struct common_dev *com)
{
	return com->ops->buf_space_avail(com,0);
}

static inline unsigned usb_common_writebuf_data_avail(struct common_dev *com)
{
	return com->ops->buf_data_avail(com,1);
}

static inline unsigned usb_common_writebuf_space_avail(struct common_dev *com)
{
	return com->ops->buf_space_avail(com,1);
}

extern int register_common_driver(struct usb_common_driver * driver);
//extern int register_common_android_usb_function(char *name, int (*ipc_probe)(void *, int i ));
#endif 
