/*
 * f_common.c -- drivers/usb/gadget/f_common.c    add by cool_zhao
 *
 *
 *
 */


/* #define VERBOSE_DEBUG */
/* #define DUMP_MSGS */

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

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/composite.h>

#include <mach/ipc.h>
#include <linux/moduleparam.h>

#include "gadget_chips.h"
#include "f_common.h"


/*------------------------------------------------------------------------*/

#define BUF_STATE_EMPTY 	0
#define BUF_STATE_FULL  	1
#define QUEUE_SIZE  16
#define USB_MAX_TRANSFER_SIZE   512
#define MAX_TRANSFER_SIZE   QUEUE_SIZE*USB_MAX_TRANSFER_SIZE*2
#define COMMON_CLOSE_TIMEOUT        15

#define MAX_DEV                 6 
#define MAX_REQ_USED			32*4

static struct list_head  com_driver_head;
static struct common_dev * com_dev[MAX_DEV];
static int write_number = 0;
module_param_named(write_number, write_number, int, S_IRUGO | S_IWUSR | S_IWGRP);
static int write_number_commplete = 0 ;
module_param_named(write_number_commplete, write_number_commplete, int, S_IRUGO | S_IWUSR | S_IWGRP);

//#define COMMON_DEBUG   printk("in function: %s--------\n",__func__)
#define COMMON_DEBUG  do{;}while(0) 
//static const char common_string_interface[] = "COMMON";

static void common_out_complete(struct usb_ep *ep, struct usb_request *req);
static void common_in_complete(struct usb_ep *ep, struct usb_request *req);
struct common_req common_req_list[MAX_REQ_USED];

static struct usb_interface_descriptor
common_intf_desc = {
	.bLength =		sizeof common_intf_desc,
	.bDescriptorType =	USB_DT_INTERFACE,

	.bNumEndpoints =	2,		
	.bInterfaceClass =	0xFF,
	.bInterfaceSubClass =	0xFF,	
	.bInterfaceProtocol =	0xFF,	
};

static struct usb_endpoint_descriptor common_fs_bulk_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor common_fs_bulk_out_desc  = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *common_fs_function[]  = {
	(struct usb_descriptor_header *) &common_intf_desc,
	(struct usb_descriptor_header *) &common_fs_bulk_in_desc,
	(struct usb_descriptor_header *) &common_fs_bulk_out_desc,
	NULL,
};
/*
 * USB 2.0 devices need to expose both high speed and full speed
 * descriptors, unless they only run at full speed.
 *
 * That means alternate endpoint descriptors (bigger packets)
 * and a "device qualifier" ... plus more construction options
 * for the configuration descriptor.
 */
static struct usb_endpoint_descriptor
common_hs_bulk_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize 	=	__constant_cpu_to_le16(512),
	.bInterval =		0,	/* NAK every 0 uframe */
	/* wMaxPacketSize set by autoconfiguration */
};

static struct usb_endpoint_descriptor
common_hs_bulk_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize 	=	__constant_cpu_to_le16(512),
	.bInterval =		0,	/* NAK every 0 uframe */
	/* wMaxPacketSize set by autoconfiguration */
};

static struct usb_descriptor_header *common_hs_function[] = {
	(struct usb_descriptor_header *) &common_intf_desc,
	(struct usb_descriptor_header *) &common_hs_bulk_in_desc,
	(struct usb_descriptor_header *) &common_hs_bulk_out_desc,
	NULL,
};


#if 0
/* Maxpacket and other transfer characteristics vary by speed. */
static struct usb_endpoint_descriptor *
common_ep_desc(struct usb_gadget *g, struct usb_endpoint_descriptor *fs,
		struct usb_endpoint_descriptor *hs)
{
	if (gadget_is_dualspeed(g) && g->speed == USB_SPEED_HIGH)
		return hs;
	return fs;
}
#endif


/* Static strings, in UTF-8 (for simplicity we use only ASCII characters) */
static struct usb_string		common_strings[] = {
	[0].s = "Gerneric common ssssssssssssssssssss",
	{} /* end of list */
};

static struct usb_gadget_strings	common_stringtab = {
	.language	= 0x0409,		/* en-us */
	.strings	= common_strings,
};

/* There is only one interface. */




/*
 * gs_alloc_req
 *
 * Allocate a usb_request and its buffer.  Returns a pointer to the
 * usb_request or NULL if there is an error.
 */

static int common_buf_alloc(struct common_buf *cb, unsigned size)
{
	if(cb->buf_buf != NULL&& cb->buf_size == size)
	{
		goto complete;
	}
	cb->buf_buf = kmalloc(size, GFP_KERNEL);
	if (cb->buf_buf == NULL)
		return -ENOMEM;
complete:
	cb->buf_size = size;
	cb->data_len = 0;
	cb->packet_xmit = 0;
	cb->buf_put = cb->buf_buf;
	cb->buf_get = cb->buf_buf;
	cb->used = false;

	return 0;
}

static void common_buf_free(struct common_buf *cb)
{
	if(cb->buf_buf)
		kfree(cb->buf_buf);
	cb->buf_buf = NULL;
	cb->buf_size = 0;
}
static unsigned common_buf_data_avail(struct common_buf *cb)
{
	return (cb->buf_size + cb->buf_put - cb->buf_get) % cb->buf_size;
}

/*
 * gs_buf_space_avail
 *
 * Return the number of bytes of space available in the circular
 * buffer.
 */
static unsigned common_buf_space_avail(struct common_buf *cb)
{
	return (cb->buf_size + cb->buf_get - cb->buf_put - 1) % cb->buf_size;
}


static unsigned common_buf_put(struct common_buf *cb, const char *buf, unsigned count)
{
	unsigned len;

	len  = common_buf_space_avail(cb);
	if (count > len)
		count = len;

	if (count == 0)
		return 0;

	len = cb->buf_buf + cb->buf_size - cb->buf_put;
	if (count > len) {
		memcpy(cb->buf_put, buf, len);
		memcpy(cb->buf_buf, buf+len, count - len);
		cb->buf_put = cb->buf_buf + count - len;
	} else {
		memcpy(cb->buf_put, buf, count);
		if (count < len)
			cb->buf_put += count;
		else /* count == len */
			cb->buf_put = cb->buf_buf;
	}

	return count;
}

/*
 * gs_buf_get
 *
 * Get data from the circular buffer and copy to the given buffer.
 * Restrict to the amount of data available.
 *
 * Return the number of bytes copied.
 */
static unsigned
common_buf_get(struct common_buf *cb, char *buf, unsigned count)
{
	unsigned len;

	len = common_buf_data_avail(cb);
	if (count > len)
		count = len;

	if (count == 0)
		return 0;

	len = cb->buf_buf + cb->buf_size -cb->buf_get;
	if (count > len) {
		memcpy(buf, cb->buf_get, len);
		memcpy(buf+len, cb->buf_buf, count - len);
		cb->buf_get = cb->buf_buf + count - len;
	} else {
		memcpy(buf, cb->buf_get, count);
		if (count < len)
			cb->buf_get += count;
		else /* count == len */
			cb->buf_get = cb->buf_buf;
	}

	return count;
}

static unsigned common_send_packet(struct common_dev *com, char *packet, unsigned size)
{
	unsigned len;
	unsigned long flags;

	len = common_buf_data_avail(&com->write_buf[0]);
	if (len < size)
		size = len;
	if (size != 0){
		spin_lock_irqsave(&com->buf_lock_write,flags);
		size = common_buf_get(&com->write_buf[0], packet, size);
		spin_unlock_irqrestore(&com->buf_lock_write,flags);
	}
	return size;
}
static struct usb_request *
common_alloc_req(struct usb_ep *ep, unsigned len, gfp_t kmalloc_flags)
{
	struct usb_request *req;
//	struct common_dev *com = ep->driver_data;
	int i = 0;

	req = usb_ep_alloc_request(ep, kmalloc_flags);

	if (req != NULL) {
		req->length = len;
#if 0
		if (((com->channel_id >= IPC_CHANNEL_PS1_ID)&&(com->channel_id <= IPC_CHANNEL_PS6_ID)) 
			|| (com->channel_id == IPC_CHANNEL_ARMLOG_ID)){
			return req;
		}
#endif
		for(i = 0; i < MAX_REQ_USED; i++)
		{
			if(common_req_list[i].used == 0)
				break;
		}
		if(i == MAX_REQ_USED){
			printk("in function %s ,use pool req is failed\n",__func__);
			return NULL;
		}
		common_req_list[i].buf = kmalloc(len, kmalloc_flags);
		req->buf = common_req_list[i].buf;	
		if (req->buf == NULL) {
			usb_ep_free_request(ep, req);
			return NULL;
		}
		common_req_list[i].used = 1;
	}

	return req;
}

void common_free_req(struct usb_ep *ep, struct usb_request *req)
{
//	struct common_dev *com = ep->driver_data;
	int i = 0;
#if 0
	if (((com->channel_id >= IPC_CHANNEL_PS1_ID)&&(com->channel_id <= IPC_CHANNEL_PS6_ID)) 
			|| (com->channel_id == IPC_CHANNEL_ARMLOG_ID)){
			usb_ep_free_request(ep, req);
			return;
	}
	kfree(req->buf);
#endif
    if(req->buf)
    {
    	for(i = 0; i < MAX_REQ_USED; i++)
		{
			if(common_req_list[i].buf == req->buf)
				break;
		}

		if(i == MAX_REQ_USED){
			//kfree(req->buf);
		}else{			
    		common_req_list[i].used = 0;
		}
    	req->buf = NULL;
    }
	usb_ep_free_request(ep, req);
}
static void common_free_requests(struct usb_ep *ep, struct list_head *head,
							 int *allocated)
{
	struct usb_request	*req;

	while (!list_empty(head)) {
		req = list_entry(head->next, struct usb_request, list);
		list_del(&req->list);
		common_free_req(ep, req);
		if (allocated)
			(*allocated)--;
	}
}

static int common_alloc_requests(struct usb_ep *ep, struct list_head *head,
		void (*fn)(struct usb_ep *, struct usb_request *),
		int *allocated)
{
	int			i;
	struct usb_request	*req;
	int n = allocated ? QUEUE_SIZE - *allocated : QUEUE_SIZE;

	/* Pre-allocate up to QUEUE_SIZE transfers, but if we can't
	 * do quite that many this time, don't fail ... we just won't
	 * be as speedy as we might otherwise be.
	 */
	 COMMON_DEBUG;
	printk("n is %d\n",n);
	for (i = 0; i < n; i++) {
		req = common_alloc_req(ep, ep->maxpacket, GFP_ATOMIC);
		if (!req)
			return list_empty(head) ? -ENOMEM : 0;
		req->complete = fn;
		list_add_tail(&req->list, head);
		if (allocated)
			(*allocated)++;
	}
	return 0;
}
static int common_start_tx(struct common_dev *com)

{
	struct list_head	*pool = &com->write_pool;
	struct usb_ep		*in = com->in;
	int			status = 0;
	unsigned long flags = 0;

	spin_lock_irqsave(&com->lock,flags);
	while (!list_empty(pool)) {
		struct usb_request	*req;
		int			len;

		if (com->write_started >= QUEUE_SIZE)
			break;

		req = list_entry(pool->next, struct usb_request, list);
		
		len = common_send_packet(com, req->buf, in->maxpacket);
		if (len == 0) {
			wake_up_interruptible(&com->write_wait);
			break;
		}		
		req->length = len;
		list_del(&req->list);
			
		spin_unlock_irqrestore(&com->lock,flags);
		status = usb_ep_queue(in, req, GFP_ATOMIC);
		spin_lock_irqsave(&com->lock,flags);

		if (status) {
			pr_debug("%s: %s %s err %d\n",
					__func__, "queue", in->name, status);
			list_add(&req->list, pool);
			break;
		}

		com->write_started++;

		/* abort immediately after disconnect */
		if(!com->com_enabled)
			break;
	}
	spin_unlock_irqrestore(&com->lock,flags);
		
	return status;
}

static int common_push_buffer(struct common_dev *com,int len)

{
	struct common_buf    *buffer = NULL;
//	int 	i = 0;
	int temp = 0;	
	unsigned long flags = 0;
	COMMON_DEBUG;
	if(len >= MAX_TRANSFER_SIZE){
		printk("len is > max transfer size\n");
		len = MAX_TRANSFER_SIZE - 6;
	}
#if 0
	
	for(i = 0; i < MAX_W_BUF; i++){
		if(!com->write_buf[i].used)
		{
			break;
		}
	}
	
	if(i == MAX_W_BUF){
		printk("com dev write buf is not enough\n");
		i = 0;
	}
#endif
	spin_lock_irqsave(&com->lock,flags);
	buffer = &com->write_buf[com->wbuf_index];
	com->wbuf_index++ ;
	if(com->wbuf_index >= MAX_W_BUF){
		com->wbuf_index = com->wbuf_index - MAX_W_BUF;
	}
	//printk("writ index is %d xmix is %d\n",com->wbuf_index,com->xmit_index);
	if(com->wbuf_index == com->xmit_index){
		printk("com dev write buf is not enough writ index is %d xmix is %d\n",com->wbuf_index,com->xmit_index);
		com->xmit_index++ ;
		if(com->xmit_index >= MAX_W_BUF){
			com->xmit_index = com->xmit_index - MAX_W_BUF;
		}
	
	}
	spin_unlock_irqrestore(&com->lock,flags);
	
	if ((com->channel_id >= IPC_CHANNEL_PS1_ID)&&(com->channel_id <= IPC_CHANNEL_PS6_ID))
	{

		buffer->buf_buf[0] = 0xaa;
		buffer->buf_buf[1] = 0xaa;
		memcpy(buffer->buf_buf+2, &len, 2);
		temp = ipc_read(com->channel_id,buffer->buf_buf+4,len);
		if(temp != len){
			printk("read ipc not complete\n");
		}
		buffer->buf_buf[len+4] = 0xbb;
		buffer->buf_buf[len+5] = 0xbb;
		buffer->data_len = temp + 6;
//		buffer->used = true;
		
		
	}else if(com->channel_id == IPC_CHANNEL_ARMLOG_ID){
		temp = ipc_read(com->channel_id,buffer->buf_buf,len);
		if(temp != len){
			printk("read ipc not complete\n");
		}
		buffer->data_len = temp;
//		buffer->used = true;
	}

	return temp;
}

static int common_start_ps_tx(struct common_dev *com)

{
	struct list_head	*pool = &com->write_pool;
	struct usb_ep		*in = com->in;
	struct common_buf    *buffer = NULL;
	int 	i = 0;
	int     loop = 0;
	int			status = 0;
	int      temp = 0;
	int      length = 0;
	unsigned long flags = 0;
    COMMON_DEBUG;
	spin_lock_irqsave(&com->lock,flags);

	if(com->xmit_index >=com->wbuf_index){
		loop = com->wbuf_index + MAX_W_BUF - com->xmit_index ;
	}else{
		loop = com->wbuf_index - com->xmit_index;
	}
	//printk("loop is %d  wbuf is %d xmit is %d", loop,com->wbuf_index,com->xmit_index);
	for(i = 0 ; i < loop; i++){
		/*
		if(!com->write_buf[i].used){
			continue;
		}
		*/
		buffer = &com->write_buf[com->xmit_index];
		length = buffer->data_len;
		//printk("---data len is %d\n",length);
		
		if(!list_empty(pool)&&length) {
			struct usb_request	*req;
			//printk("------- while com->write_started is %d\n",com->write_started);
					
			if (com->write_started >= QUEUE_SIZE)
				break;

			req = list_entry(pool->next, struct usb_request, list);
#if 0
			if (length > in->maxpacket){
				temp = in->maxpacket;
			}else{
				temp = length;
			}
#endif
//			req->buf = buffer->buf_get ;
			req->buf = buffer->buf_buf;		
//			req->length = temp;
			req->length = length;
			req->context = buffer;
			
			list_del(&req->list);
				
			spin_unlock_irqrestore(&com->lock,flags);
			
			status = usb_ep_queue(in, req, GFP_ATOMIC);
			spin_lock_irqsave(&com->lock,flags);
			
			if (status) {
				printk("%s: %s %s err %d\n",
						__func__, "queue", in->name, status);
				list_add(&req->list, pool);
				break;
			}
			length = length - temp;
//			buffer->buf_get = buffer->buf_get + temp;
//			buffer->packet_xmit++;

			com->write_started++;
#if 0
			if((com->channel_id == IPC_CHANNEL_PS1_ID) || (com->channel_id == IPC_CHANNEL_PS2_ID)){
				write_number++;
			}
			if(com->channel_id == IPC_CHANNEL_ARMLOG_ID) 
			{
				write_number++;
			}
#endif
			

			/* abort immediately after disconnect */
			if(!com->com_enabled)
				break;
			com->xmit_index++ ;
			if(com->xmit_index >= MAX_W_BUF){
				com->xmit_index = com->xmit_index - MAX_W_BUF;
			}	
		}
//		buffer->data_len = length;
		
	
		if(list_empty(pool) || status){
			break;
		}
	}
	spin_unlock_irqrestore(&com->lock,flags);
			
	return status;
}

static unsigned common_start_rx(struct common_dev *com)

{
	struct list_head	*pool = &com->read_pool;
	struct usb_ep		*out = com->out;
	unsigned long flags = 0;
	COMMON_DEBUG;
	spin_lock_irqsave(&com->lock,flags);
	while (!list_empty(pool)) {
		struct usb_request	*req;
		int			status;
	
		if(!com->open_count)
			break;
		if (com->read_started >= QUEUE_SIZE)
			break;

		req = list_entry(pool->next, struct usb_request, list);
		list_del(&req->list);
		req->length = out->maxpacket;

		/* drop lock while we call out; the controller driver
		 * may need to call us back (e.g. for disconnect)
		 */
		spin_unlock_irqrestore(&com->lock,flags);
		status = usb_ep_queue(out, req, GFP_ATOMIC);

		spin_lock_irqsave(&com->lock,flags);
		if (status) {
			pr_debug("%s: %s %s err %d\n",
					__func__, "queue", out->name, status);
			list_add(&req->list, pool);
			break;
		}
		com->read_started++;
		if(!com->com_enabled)
			break;

		/* abort immediately after disconnect *///here we need do something 
		
	}
	spin_unlock_irqrestore(&com->lock,flags);
	return com->read_started;
}

static void common_rx_push(unsigned long _com)
{
	struct common_dev* com  = (void *)_com;
	struct list_head	*queue = &com->read_queue;
	bool			disconnect = false;
	unsigned long flags = 0;
	unsigned long flags_buf = 0;
	COMMON_DEBUG;

	/* hand any queued data to the tty */
	spin_lock_irqsave(&com->lock,flags);
	
	while (!list_empty(queue)) {
		struct usb_request	*req;

		req = list_first_entry(queue, struct usb_request, list);

		/* leave data queued if tty was rx throttled */
		
		switch (req->status) {
		case -ESHUTDOWN:
			disconnect = true;
			printk("%d: ---shutdown\n", com->port_num);
			break;

		default:
			/* presumably a transient fault */
/*
			printk("%d: unexpected RX status %d\n",
					com->port_num, req->status);
*/
			/* FALLTHROUGH */
		case 0:
//			printk("%d: RX status %d\n",com->port_num, req->status);
			/* normal completion */

			/* push data to (open) tty */
			break;

		}

		if (req->actual) {
			char		*packet = req->buf;
			unsigned	size = req->actual;


			//printk("roll recevice char is %s\n",packet);
			spin_lock_irqsave(&com->buf_lock_read,flags_buf);
			common_buf_put(&com->read_buf, req->buf, req->actual);
			spin_unlock_irqrestore(&com->buf_lock_read,flags_buf);

			memset(packet ,0,size);

		}
		if(com->com_enabled == 0){

			list_del(&req->list);
			common_free_req(com->out, req);
		}else{
		
			list_move(&req->list, &com->read_pool);
		}
		com->read_started--;
	}
	
//	com->push_enabled = false;
	spin_unlock_irqrestore(&com->lock,flags);
	/* If we're still connected, refill the USB RX queue. */
	if(!disconnect&&com->open_count){
		wake_up_interruptible(&com->read_wait);	
		//printk("wake up read_wait\n");
		if(com->used == 1){
			common_start_rx(com);
		}
	}


}

static int common_start_io(struct common_dev *com)
{
	int status = 0;
	int i = 0;
	struct list_head	*pool = &com->write_pool;
	struct list_head	*temp = pool;
	COMMON_DEBUG;
	status = common_alloc_requests(com->out, &com->read_pool, common_out_complete,
		&com->read_allocated);
	if (status){
		printk("alloc failed status is %d\n",status);
		return status;
	}

	status = common_alloc_requests(com->in, &com->write_pool,
			common_in_complete, &com->write_allocated);
	if (status) {
		common_free_requests(com->out, &com->read_pool, &com->read_allocated);
		printk("--alloc failed status is %d\n",status);
		return status;
	}
	if (((com->channel_id >= IPC_CHANNEL_PS1_ID)&&(com->channel_id <= IPC_CHANNEL_PS6_ID)) 
			|| (com->channel_id == IPC_CHANNEL_ARMLOG_ID)){
			struct usb_request	*req;
			temp = temp->next;
		//	for(k = 0; k < QUEUE_SIZE; k++){
			
			while((temp != pool)&&(temp != NULL)){
				req = list_entry(temp, struct usb_request, list);
				for(i = 0; i < MAX_REQ_USED; i++)
				{
					if(common_req_list[i].buf == req->buf){
						req->buf = NULL;
						common_req_list[i].used = 0;
						break;
					}
				}
				temp = temp->next;
			}
	}

	status = common_start_rx(com);

	if(status < QUEUE_SIZE)
	{
		if (status == 0){
			printk("error--,queue size is zero \n");
			return -1;
		}
		else{
			printk("waring--,queue size is not full and size is %d\n",status);
		}
	}

	COMMON_DEBUG;
	return 0;
}

static inline struct common_dev *func_to_common(struct usb_function *f)
{
	return container_of(f, struct common_dev, function);
}
static int common_writes_finished(struct common_dev *com)
{
	int cond = 0;
	
	/* return true on disconnect or empty buffer */
	spin_lock_irq(&com->lock);
	if((com->channel_id == IPC_CHANNEL_PS1_ID) || (com->channel_id == IPC_CHANNEL_PS2_ID) 
							|| (com->channel_id == IPC_CHANNEL_ARMLOG_ID)){
		if(com->wbuf_index == com->xmit_index){
			cond = 1;
		}
	}else{
		cond =  !common_buf_data_avail(&com->write_buf[0]);
	}
	spin_unlock_irq(&com->lock);

	return cond;
}

/* Completion handlers. These always run in_irq. */

static void common_in_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct common_dev *com = ep->driver_data;
	struct common_buf *buffer = req->context;
	int disconnect = 0;
	
	
	COMMON_DEBUG;
	if (req->status || req->actual != req->length)
		printk("%s --> %d, %u/%u\n", __func__,
		    req->status, req->actual, req->length);
	
	spin_lock(&com->lock);
	list_add(&req->list, &com->write_pool);
#if 0
	if((com->channel_id == IPC_CHANNEL_PS1_ID) || (com->channel_id == IPC_CHANNEL_PS2_ID) 
							|| (com->channel_id == IPC_CHANNEL_ARMLOG_ID)){
		req->buf = NULL;
	}
#endif

	com->write_started--;
   // printk("----in complete started is %d\n",com->write_started);
#if 0
	if(buffer->packet_xmit > 0)
	buffer->packet_xmit--;

	if((!buffer->packet_xmit)&&(!buffer->data_len))
	{
		buffer->used = false;
		buffer->buf_get = buffer->buf_buf;
	}
#endif
	switch (req->status) {
	default:
		/* presumably a transient fault */
		printk("%s: unexpected %s status %d\n",
				__func__, ep->name, req->status);
		/* FALL THROUGH */
	case 0:
		/* normal completion */
		spin_unlock(&com->lock);
		if((com->channel_id == IPC_CHANNEL_PS1_ID) || (com->channel_id == IPC_CHANNEL_PS2_ID) 
			|| (com->channel_id == IPC_CHANNEL_ARMLOG_ID)){
			//write_number_commplete++;
			
			//common_start_ps_tx(com);
		}else{
			common_start_tx(com);
		}
		spin_lock(&com->lock);
		//printk("ok\n");
		break;

	case -ESHUTDOWN:
		/* disconnect */
		printk("%s: %s shutdown\n", __func__, ep->name);
		disconnect = 1;
		break;
	}

	spin_unlock(&com->lock);

}

static void common_out_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct common_dev *com = ep->driver_data;

	COMMON_DEBUG;

	smp_wmb();
	spin_lock(&com->lock);
	list_add_tail(&req->list, &com->read_queue);
/*
	if (com->push_enabled == true){
		spin_unlock(&com->lock);
		return;
	}
*/

	
	tasklet_schedule(&com->push);
	//com->push_enabled = true;

	spin_unlock(&com->lock);
		
}

static int common_setup(struct usb_function *f,
		     const struct usb_ctrlrequest *ctrl)
{
	//struct roll_dev		*roll = func_to_roll(f);
	COMMON_DEBUG;
	return 0;
}

/*-------------------------------------------------------------------------*/

static int common_enable_endpoint(struct usb_ep *ep)
		
{
	int	rc;
	COMMON_DEBUG;
	ep->maxburst = 0;

	rc = usb_ep_enable(ep);
	if (rc){
		printk("can't enable %s, result %d\n", ep->name, rc);
	}
	return rc;
}


/* Reset interface setting and re-init endpoint state (toggle etc). */
static int set_interface(struct common_dev *new_com)
{
	int i, rc = 0;

	COMMON_DEBUG;
	i = 0;
//disable old roll
	if(new_com->com_enabled){
		if(new_com&&new_com->dev_driver&&new_com->dev_driver->disconnect){
			rc= new_com->dev_driver->disconnect(new_com);
			if(rc){
				printk("disconnect failed\n");
			}
		}
		//printk("---------free requeset set interface\n");
		
		common_free_requests(new_com->out,&new_com->read_pool,&new_com->read_allocated);
		common_free_requests(new_com->out,&new_com->read_queue,&new_com->read_allocated);
		common_free_requests(new_com->in,&new_com->write_pool,&new_com->write_allocated);
	
		#if 0
		common_free_requests(new_com->out,&new_com->read_pool,NULL);
		common_free_requests(new_com->out,&new_com->read_queue,NULL);
		common_free_requests(new_com->in,&new_com->write_pool,NULL);
			#endif
		if (new_com->bulk_in_enabled) {
			usb_ep_disable(new_com->in);
			new_com->bulk_in_enabled = 0;
		}
		if (new_com->bulk_out_enabled) {
			usb_ep_disable(new_com->out);
			new_com->bulk_out_enabled = 0;
		}
		new_com->com_enabled = 0;
	}
//enable new roll	
	
/*
	d = common_ep_desc(new_com->gadget,
			new_com->fs.in, new_com->hs.in);
	*/
	rc = config_ep_by_speed(new_com->gadget, &(new_com->function), new_com->in);
	if (rc){
		printk("config_ep_by_speed failed in functions: %s",__func__);
		return -1;
	}
	
	rc = common_enable_endpoint(new_com->in);
	if (rc){
		printk("enabled ep failed in functions: %s",__func__);
		return -1;
	}
	new_com->bulk_in_enabled = 1;
	new_com->in->driver_data = new_com;
/*
	d = common_ep_desc(new_com->gadget,
			new_com->fs.out, new_com->hs.out);
	*/
	rc = config_ep_by_speed(new_com->gadget, &(new_com->function), new_com->out);
	if (rc){
		printk("config_ep_by_speed failed in functions: %s",__func__);
		return -1;
	}
	rc = common_enable_endpoint(new_com->out);
	if (rc){
		printk("enabled ep failed in functions: %s",__func__);
		return -1;
	}		
	new_com->bulk_out_enabled = 1;
	new_com->out->driver_data = new_com;
	new_com->bulk_out_maxpacket = usb_endpoint_maxp(new_com->out->desc);//d->wMaxPacketSize;
	
	new_com->com_enabled = 1;
	/* Allocate the requests */
	if(new_com&&new_com->dev_driver&&new_com->dev_driver->connect){
		rc= new_com->dev_driver->connect(new_com);
		if(rc){
			printk("connect failed\n");
		}
	}

	return rc;
}


/****************************** ALT CONFIGS ******************************/

static int common_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
	struct common_dev *com = func_to_common(f);
	COMMON_DEBUG;
	//printk("in function:%s ,intf is %d , alt is %d\n",__func__,intf, alt);
	//roll->interface_number = intf;
	
	set_interface(com);
	
	return 0;
}

static void common_disable(struct usb_function *f)
{
	struct common_dev *com = func_to_common(f);
	int ret = 0;
	COMMON_DEBUG;
	if(com&&com->dev_driver&&com->dev_driver->disconnect){
		ret = com->dev_driver->disconnect(com);
	}
	//printk("---------free requeset\n");
	common_free_requests(com->out,&com->read_pool,&com->read_allocated);
	common_free_requests(com->out,&com->read_queue,&com->read_allocated);
	common_free_requests(com->in,&com->write_pool,&com->write_allocated);
	#if 0
	
	common_free_requests(com->out,&com->read_pool,NULL);
	common_free_requests(com->out,&com->read_queue,NULL);
	common_free_requests(com->in,&com->write_pool,NULL);
	#endif
	if (com->bulk_in_enabled) {
		usb_ep_disable(com->in);
		com->in->driver_data = NULL;
		com->bulk_in_enabled = 0;
	}
	if (com->bulk_out_enabled) {
		usb_ep_disable(com->out);
		com->out->driver_data = NULL;
		com->bulk_out_enabled = 0;
	}

	com->com_enabled = 0;

}

/*-------------------------------------------------------------------------*/

static void common_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct common_dev		*com = func_to_common(f);
	int ret = 0;
	COMMON_DEBUG;
	if(com&&com->dev_driver&&com->dev_driver->remove){
		ret = com->dev_driver->remove(com);
	}
	if (gadget_is_dualspeed(c->cdev->gadget))
		usb_free_descriptors(f->hs_descriptors);
	usb_free_descriptors(f->descriptors);
	com->used = 0;
	
	//kfree(func_to_common(f));	
	
}

static int common_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct common_dev		*com =func_to_common(f);
	struct usb_gadget	*gadget = c->cdev->gadget;
	struct usb_ep		*ep;
	int			i;
	//int 			rc = 0;

	COMMON_DEBUG;
	com->gadget = gadget;

	/* New interface */
	i = usb_interface_id(c, f);
	if (i < 0)
		return i;
	common_intf_desc.bInterfaceNumber = i;
	com->interface_number = i;

	/* Find all the endpoints we will use */
	ep = usb_ep_autoconfig(gadget, &common_fs_bulk_in_desc);
	if (!ep)
		goto autoconf_fail;
	//ep->driver_data = roll;	/* claim the endpoint */
	
	ep->driver_data = cdev;	
	com->in = ep;

	ep = usb_ep_autoconfig(gadget, &common_fs_bulk_out_desc);
	if (!ep)
		goto autoconf_fail;
	//ep->driver_data = roll;	/* claim the endpoint */
	ep->driver_data = cdev;	
	com->out = ep;

	/* Copy descriptors */
	f->descriptors = usb_copy_descriptors(common_fs_function);
	if (unlikely(!f->descriptors))
		return -ENOMEM;
/*	
	com->fs.in = usb_find_endpoint(common_fs_function,
			f->descriptors, &common_fs_bulk_in_desc);
	com->fs.out = usb_find_endpoint(common_fs_function,
			f->descriptors, &common_fs_bulk_out_desc);
	*/
	
	if (gadget_is_dualspeed(gadget)) {
		/* Assume endpoint addresses are the same for both speeds */
		common_hs_bulk_in_desc.bEndpointAddress =
			common_fs_bulk_in_desc.bEndpointAddress;
		common_hs_bulk_out_desc.bEndpointAddress =
			common_fs_bulk_out_desc.bEndpointAddress;
		f->hs_descriptors = usb_copy_descriptors(common_hs_function);
		if (unlikely(!f->hs_descriptors)) {
			usb_free_descriptors(f->descriptors);
			return -ENOMEM;
		}
		/*
		com->hs.in = usb_find_endpoint(common_hs_function,
				f->hs_descriptors, &common_hs_bulk_in_desc);
		com->hs.out = usb_find_endpoint(common_hs_function,
				f->hs_descriptors, &common_hs_bulk_out_desc);
		*/
	}

	return 0;

autoconf_fail:
	printk("unable to autoconfigure all endpoints\n");
	return -ENOTSUPP;
}
void common_suspend(struct usb_function *f)
{
		
	COMMON_DEBUG;
	#ifdef COMMON_POWER
	struct common_dev		*com =func_to_common(f);
	int ret = 0;
	if(com&&com->dev_driver&&com->dev_driver->suspend){
		ret = com->dev_driver->suspend(com);
	}
	#endif
	//return ret;
}
void common_resume(struct usb_function *f)
{	
	
	COMMON_DEBUG;
	#ifdef COMMON_POWER
	int ret = 0;
	struct common_dev		*com =func_to_common(f);
	if(com&&com->dev_driver&&com->dev_driver->resume){
		ret = com->dev_driver->resume(com);
	}
	#endif
	//return ret;
}


/****************************** ADD FUNCTION ******************************/

static struct usb_gadget_strings *common_strings_array[] = {
	&common_stringtab,
	NULL,
};

static int usb_ipc_read(struct common_dev* com, char *buf, int len)
{
	//int ret;
	int num;
	COMMON_DEBUG;
	if(buf ==NULL){
		printk("buffer invalue cool_zhao\n");
		return -1;
	}
	if (com->com_enabled ==0)
	{
		printk("usb_common is not  connected  cool_zhao\n");
		return -1;
	}

	wait_event_interruptible(com->read_wait, (common_buf_data_avail(&com->read_buf))!= 0);

	num = common_buf_data_avail(&com->read_buf);

	//printk("in function:%s -----number is %d\n",__func__,num);
	num = min(num,len);
	common_buf_get(&com->read_buf, buf,num);
	return num;
}
static int usb_ipc_write(struct common_dev *com, char *buf,int len)
{
	//int ret;
	int num;
	COMMON_DEBUG;
	//printk("in func %s len is %d\n",__func__, len);
	
	if (com->com_enabled ==0){
		printk("usb_common is connected  cool_zhao\n");
		return -1;
	}
	if(buf == NULL){
		
		num = common_push_buffer(com, len);
		common_start_ps_tx(com);
		
	}else{
		num = common_buf_space_avail(&com->write_buf[0]);
		if(num < len)
		{
			printk("has no enough space\n");
		}


		num = min(num, len);
		num = common_buf_put(&com->write_buf[0], buf,num);
		common_start_tx(com);
	}
	return num;
}
static int usb_ipc_close(struct common_dev *com)
{
	int i = 0;
	COMMON_DEBUG;
	
	com->open_count--;
	
	if (com->open_count)
	{
		printk("com dev is opened not only times and count is %d\n",com->open_count);
		return 0;
	}
	com->openclose = true;
	if (common_buf_data_avail(&com->write_buf[0]) > 0 ) {
		//spin_unlock_irq(&com->lock);
		if(com->com_enabled == 1){
			wait_event_interruptible_timeout(com->write_wait,
					     common_writes_finished(com),
					     COMMON_CLOSE_TIMEOUT * HZ);
		}
		//spin_lock_irq(&com->lock);
		//gser = port->port_usb;
	}
	for(i = 0; i < MAX_W_BUF; i++){
		common_buf_free(&com->write_buf[i]);
	}
	common_buf_free(&com->read_buf);
	com->openclose = false;
	//wake_up_interruptible(&com->close_wait);
	return 0;
}
int usb_ipc_open(struct common_dev *com)
{
	int  rc = 0;
	//int flags = 0;
	int i = 0;
	if (com->com_enabled == 0)
	{
		printk(" usb is not connected open failed\n");
		return -1;
	}
	COMMON_DEBUG;
	if (com->open_count)
	{
		com->open_count++;
		printk("usb_ipc_open is opened and count is %d\n",com->open_count);
		return 0;
	}
	
	com->openclose = true;
	
	//spin_lock_irqsave(&com->lock, flags);

	rc =common_buf_alloc(&com->read_buf, MAX_TRANSFER_SIZE);
	if (rc){
		printk("malloc buf failed \n");
		com->openclose = false;
		return rc;
	}
	
	for(i = 0; i < MAX_W_BUF; i++){
		rc =common_buf_alloc(&com->write_buf[i], MAX_TRANSFER_SIZE);
		if (rc){
			
			printk("malloc buf\n");
			goto fail1;
		};
	}
	//spin_unlock_irqrestore(&com->lock, flags);
	com->open_count = 1;
	com->openclose = false;
	com->xmit_index = 0;
	com->wbuf_index = 0;
	tasklet_init(&com->push, common_rx_push, (unsigned long) com);
	com->push_enabled = false;
	rc = common_start_io(com);
	return rc;
fail1:
	common_buf_free(&com->read_buf);
	com->openclose = false;
	return rc;
}

static unsigned usb_ipc_data_avail(struct common_dev *com, int rw)
{
	if(rw == 0)
	{
		return common_buf_data_avail(&com->read_buf);
	}else{
		return common_buf_data_avail(&com->write_buf[0]);
	}
}

static unsigned usb_ipc_space_avail(struct common_dev *com, int rw)
{
	if(rw == 0)
	{
		return common_buf_space_avail(&com->read_buf);
	}else{
		return common_buf_space_avail(&com->write_buf[0]);
	
	}
}
static struct common_operations com_ops ={
	.open = usb_ipc_open,
	.close = usb_ipc_close,
	.read = usb_ipc_read,
	.write = usb_ipc_write,
	.buf_data_avail = usb_ipc_data_avail,
	.buf_space_avail = usb_ipc_space_avail,
};

#if 0
static int common_setup(struct usb_gadget *g, unsigned count)
{
	;
}
static int common_clean(struct usb_gadget *g, unsigned count)
{
	wait_event(port->close_wait, gs_closed(port));
}
#endif 
static int common_malloc(int max)
{
	int i = 0;
	for(i = 0; i < max; i++)
	{
		com_dev[i] = kzalloc(sizeof *com_dev[i], GFP_KERNEL);
		if (com_dev[i] == NULL){
			printk("----------malloc memory is failed but not do with defaut---------\n");
		}
	}
	return 0;
}
static const char * tmp_name = NULL;

static int common_bind_config(struct usb_configuration *c, char * name,
			   int port_num )
{
	struct common_dev *com;
	struct list_head *p;
	struct usb_common_driver * com_driver = NULL;
	int rc = 0;
	int status;
	int i;
	
	COMMON_DEBUG;
	//com = kzalloc(sizeof *com, GFP_KERNEL);
	
	for (i=0; i < MAX_DEV; i++){
		//char tmp_name[15];
		//sprintf(tmp_name, "%s%u", name, port_num);
		if(com_dev[i]->function.name == NULL){
			continue;
		}

		if (com_dev[i]->used == 1){
			continue;
		}

		tmp_name  =kasprintf(GFP_KERNEL,"%s%u",name,port_num);

		if(strcmp(tmp_name, com_dev[i]->function.name) == 0)
		{
			printk("----tmp_name is %s \n",tmp_name);
			com = com_dev[i];
			com_dev[i]->used =1;
			printk("----------bind config i is %d\n",i);
			goto next;
		}
	}
	for ( i=0; i < MAX_DEV; i++){
		if (com_dev[i]->used == 0)
		{
			printk("----------bind config --used-- i is %d\n",i);
			com_dev[i]->used =1;
			break;
		}
	}
	if (i == MAX_DEV){
		printk("------------bind failed ----\n");
		return -1;
	}
	com = com_dev[i];
	
	if (common_strings[0].id == 0) {
		status = usb_string_id(c->cdev);
		if (status < 0)
			return status;
		common_strings[0].s = kasprintf(GFP_KERNEL,"Gerneric common %s",name);
		common_strings[0].id = status;
	}
	com->com_enabled = 0; //init use by set_interface;
	com->dev_name = name;
	com->port_num =port_num;
	com->open_count = 0;
	com->function.name       =kasprintf(GFP_KERNEL,"%s%u",name,port_num);
	com->function.strings     = common_strings_array;
	com->function.bind        = common_bind;
	com->function.unbind      = common_unbind;
	com->function.setup       = common_setup;
	com->function.set_alt     = common_set_alt;
	com->function.disable     = common_disable;
	com->function.suspend    = common_suspend;
	com->function.resume      = common_resume;
	
	
	com->len = USB_MAX_TRANSFER_SIZE; //USB_MAX_BUFFER_SIZE when malloc buffer

	/*
	 * Our caller holds a reference to common structure so we
	 * don't have to be worry about it being freed until we return
	 * from this function.  So instead of incrementing counter now
	 * and decrement in error recovery we increment it only when
	 * call to usb_add_function() was successful.
	 */
	spin_lock_init(&com->lock);
	spin_lock_init(&com->buf_lock_read);  //for read buffer lock add by  cool_zhao 20130125
	spin_lock_init(&com->buf_lock_write);  //for read buffer lock add by  cool_zhao 20130125

	INIT_LIST_HEAD(&com->read_pool);
	INIT_LIST_HEAD(&com->read_queue);
	INIT_LIST_HEAD(&com->write_pool);
	
	init_waitqueue_head(&com->close_wait);
	init_waitqueue_head(&com->read_wait);
	init_waitqueue_head(&com->write_wait);
next:
	rc = usb_add_function(c, &com->function);
	
	if (unlikely(rc)){
		printk("\n\n-----------in function %s usb_add_function failed\n",__func__);
		//kfree(com);
		com->used = 0;
		return rc;
	}
	com->ops = &com_ops;

	if(!list_empty(&com_driver_head)){
		list_for_each(p, &com_driver_head) {
			com_driver = list_entry(p, struct usb_common_driver, list);
			if(strcmp(com_driver->name, com->dev_name)==0){
				rc = com_driver->probe(com);
				if (rc)
				{
					printk("in IPC driver probe failed\n");
				}
				list_add(&com->list,&com_driver->dev_list);
				com->dev_driver = com_driver;
				break;
				
			}
		}
	}
	return rc;
}

int register_common_driver(struct usb_common_driver * _driver)
{
	int ret = 0;
	int i = 0;
	//int matched = 0;
	struct list_head *p;
	struct common_dev * com = NULL;
	struct usb_common_driver * com_driver = NULL;
	if(!list_empty(&com_driver_head)){
		list_for_each(p, &com_driver_head) {
			com_driver = list_entry(p, struct usb_common_driver, list);
			if(com_driver == _driver)
			{
				printk("This driver has registered\n");
				return 0;
			}
			if (strcmp(com_driver->name,_driver->name)== 0)
			{
				printk("the driver name has defined in other registered driver cool_zhao\n");
				return -1;
			}
		}
	}

	list_add_tail(&_driver->list,&com_driver_head);
	INIT_LIST_HEAD(&_driver->dev_list);
	com = com_dev[0];
	for(i=0; i< MAX_DEV;i++)
	{
		com = com_dev[i];
		if (com == NULL){
			continue;
		}
		if (com->used == 0){
			continue;
		}
		//mach though device name and driver name
		if(strcmp(_driver->name, com->dev_name)==0){
			ret = _driver->probe(com);
			if (ret){
				printk("probe is failed\n");
				return ret;
			}
			list_add(&com->list,&_driver->dev_list);
			com->dev_driver = _driver;
		}
	}
	
	return ret;	
	
}

int unregister_common_driver(struct usb_common_driver * driver)
{
	//TODO;
	#if 0

	int ret = 0;
	int i = 0;
	int matched = 0;
	struct list_head *p;
	struct common_dev *com = NULL;
	struct usb_common_driver *com_driver = NULL;

		list_for_each(p, &com_driver_head) {
		com_driver = list_entry(p, struct usb_common_driver, list);
		if(com_driver == driver)
		{
			printk("This driver has registered\n");
		
			matched = 1;
			break;
		}
		if (strcmp(com_driver->name,driver->name)== 0)
		{
			printk("the driver name has defined in other registered driver cool_zhao\n");
			return -1;
		}
	}
	#endif
	return 0;
}

static __init int common_dev_init(void)
{
	INIT_LIST_HEAD(&com_driver_head);
	common_malloc(MAX_DEV);
	return 0;
}
#if 0
static __init void common_dev_exit(void)
{
	return;
}
#endif


//sys_initcall(common_dev_init);
module_init(common_dev_init);
//module_exit(common_dev_exit);



