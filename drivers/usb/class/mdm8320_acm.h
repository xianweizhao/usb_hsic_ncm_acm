/*
 *
 * Includes for smm6260-acm.h
 *
 * Mainly take from usbnet's cdc-ether part
 *
 */

/*
 * CMSPAR, some architectures can't have space and mark parity.
 */

#ifndef MDM8320_H

#define MDM8320_H

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/usb/cdc.h>
#include <linux/list.h>
#include <linux/pm_runtime.h>
#include <linux/suspend.h>
#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#endif

#ifndef CMSPAR
#define CMSPAR			0
#endif
#define ACM_INTERFACES		4

/*
 * Requests.
 */

#define USB_RT_ACM		(USB_TYPE_CLASS | USB_RECIP_INTERFACE)


/*
 * Output control lines.
 */

#define ACM_CTRL_DTR		0x01
#define ACM_CTRL_RTS		0x02
/*
 * Input control lines and line errors.
 */

#define ACM_CTRL_DCD		0x01
#define ACM_CTRL_DSR		0x02
#define ACM_CTRL_BRK		0x04
#define ACM_CTRL_RI		0x08

#define ACM_CTRL_FRAMING	0x10
#define ACM_CTRL_PARITY		0x20
#define ACM_CTRL_OVERRUN	0x40
/*
 * Internal driver structures.
 */
/*
 * The only reason to have several buffers is to accomodate assumptions
 * in line disciplines. They ask for empty space amount, receive our URB size,
 * and proceed to issue several 1-character writes, assuming they will fit.
 * The very first write takes a complete URB. Fortunately, this only happens
 * when processing onlcr, so we only need 2 buffers. These values must be
 * powers of 2.
 */
#define ACM_NW  16
#define ACM_NR  16

struct acm_wb {
	unsigned char *buf;
	dma_addr_t dmah;
	int len;
	int use;
	struct urb		*urb;
	struct acm		*instance;
};

struct acm_rb {
	struct list_head	list;
	int			size;
	unsigned char		*base;
	dma_addr_t		dma;
};


struct acm_rb ;
struct acm_ru {
	struct list_head	list;
	struct acm_rb		*buffer;
	struct urb		*urb;
	struct acm		*instance;
};

struct acm {
	struct acm *parent;	
	struct usb_device *dev;				/* the corresponding usb device */
	struct usb_interface *control;			/* control interface */
	struct usb_interface *data;			/* data interface */
	unsigned int cid;
	int count;
	//struct tty_port port;			 	/* our tty port data */
	struct urb *ctrlurb;				/* urbs */
	u8 *ctrl_buffer;				/* buffers of urbs */
	dma_addr_t ctrl_dma;				/* dma handles of buffers */
	u8 *country_codes;				/* country codes from device */
	unsigned int country_code_size;			/* size of this buffer */
	unsigned int country_rel_date;			/* release date of version */
	struct acm_wb wb[ACM_NW];
	struct acm_ru ru[ACM_NR];
	struct acm_rb rb[ACM_NR];
	int rx_buflimit;
	int rx_endpoint;
	int tx_endpoint;
	int ctrl_endpoint;	
	spinlock_t read_lock;
	struct list_head spare_read_urbs;
	struct list_head spare_read_bufs;
	struct list_head filled_read_bufs;
	int bufs_filled;
	int write_used;					/* number of non-empty write buffers */
	int processing;
	int transmitting;
	spinlock_t write_lock;
	struct mutex mutex;
	struct usb_cdc_line_coding line;		/* bits, stop, parity */
	struct work_struct work;			/* work queue entry for line discipline waking up */
	wait_queue_head_t drain_wait;			/* close processing */
	struct tasklet_struct urb_task;                 /* rx processing */
	spinlock_t throttle_lock;			/* synchronize throtteling and read callback */
	unsigned int ctrlin;				/* input control lines (DCD, DSR, RI, break, overruns) */
	unsigned int ctrlout;				/* output control lines (DTR, RTS) */
	unsigned int writesize;				/* max packet size for the output bulk endpoint */
	unsigned int readsize,ctrlsize;			/* buffer sizes for freeing */
	unsigned int minor;				/* acm minor number */
	unsigned char throttle;				/* throttled by tty layer */
	unsigned char clocal;				/* termios CLOCAL */
	unsigned int ctrl_caps;				/* control capabilities from the class specific header */
	unsigned int susp_count;			/* number of suspended interfaces */
	unsigned int combined_interfaces:1;		/* control and data collapsed */
	unsigned int is_int_ep:1;			/* interrupt endpoints contrary to spec used */
	u8 bInterval;
	struct acm_wb *delayed_wb;			/* write queued for a device about to be woken */

	unsigned int  dpm_suspending;
	unsigned int  skip_hostwakeup;
	unsigned int  resume_debug;
	unsigned int  usb_connected;
	struct work_struct post_resume_work;
#ifdef CONFIG_HAS_WAKELOCK
	struct wake_lock 	pm_lock;
	struct wake_lock 	dormancy_lock;
	long 			wake_time;
#endif	
};


#define CDC_DATA_INTERFACE_TYPE	0x0a

/* constants describing various quirks and errors */
#define NO_UNION_NORMAL			1
#define SINGLE_RX_URB			2
#define NO_CAP_LINE			4
#define NOT_A_MODEM			8
#endif
