#ifndef _C8320_USB_H
#define _C8320_USB_H

#include <linux/wakelock.h>

#define MAX_DEV  3
#define USB_NW   2
#define USB_NR   16
#define USB_MAX_WRITE    1024 /* one time transfer 20*max_packet */
#define USB_MAX_READ     10


struct usbdev_wb {
	unsigned char *buf;
	dma_addr_t dmah;
	int len;
	int use;
	struct urb		*urb;
	struct usb_dev	*instance;
};

struct usbdev_rb {
	struct list_head	list;
	int			size;
	int  		index;
	unsigned char		*base;
	dma_addr_t		dma;
	struct usb_dev	*instance;
};

struct usb_dev {
	const char* cdev_name;
	struct cdev *cdev;
	struct usb_device *udev;	
	struct usb_interface *interface;

	struct usbdev_wb wb[USB_NW];
	struct usbdev_wb  *delayed_wb;

	unsigned long read_urbs_free;
	struct urb *read_urbs[USB_NR];
	struct usbdev_rb rb[USB_NR];
	int rx_buflimit;

	struct list_head filled_read_bufs;

	int rx_endpoint;
	int tx_endpoint;

	int writesize;
	int readsize;
	int transmitting;
	
	wait_queue_head_t read_wq;

	struct tasklet_struct urb_task;                 /* rx processing */
	spinlock_t write_lock;
	spinlock_t read_lock;
	struct mutex mutex;
#ifdef CONFIG_HAS_WAKELOCK
	struct wake_lock 	pm_lock;
#endif
	int susp_count;

	int connected;
	int opened;
	int major;
	int minor;

};


#endif
