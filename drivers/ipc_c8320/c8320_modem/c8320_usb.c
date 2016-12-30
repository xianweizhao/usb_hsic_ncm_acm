/*
 * c8320_usb.c
 */

#undef DEBUG
#undef VERBOSE_DEBUG

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/serial.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/cdev.h>
#include <linux/usb/cdc.h>
#include <linux/list.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <asm/byteorder.h>
#include <asm/unaligned.h>

#include "c8320_usb.h"
#ifdef CONFIG_C8320_HSIC_NEW
#include "../modem_v2/c8320_modem_ctl.h"
#endif

static struct cdev usbdev;
static struct class *c8320_usbclass;
static int major;
static struct usb_dev *usbdev_table[MAX_DEV];



/*
 * Write buffer management.
 * All of these assume proper locks taken by the caller.
 */

static int c8320_usb_alloc_wb(struct usb_dev *dev)
{
	int i, wbn;
	struct usbdev_wb *wb;

	wbn = 0;
	i = 0;
	for (;;) {
		wb = &dev->wb[wbn];
		if (!wb->use) {
			wb->use = 1;
			return wbn;
		}
		wbn = (wbn + 1) % USB_NW;
		if (++i >= USB_NW)
			return -1;
	}
}

/*
 * Finish write. Caller must hold acm->write_lock
 */
static void c8320_usb_write_done(struct usb_dev *dev, struct usbdev_wb *wb)
{
	wb->use = 0;
	dev->transmitting--;
	usb_autopm_put_interface_async(dev->interface);
}

/*
 * Poke write.
 *
 * the caller is responsible for locking
 */

static int c8320_usb_start_wb(struct usb_dev *dev, struct usbdev_wb *wb)
{
	int ret;

	dev->transmitting++;

	wb->urb->transfer_buffer = wb->buf;
	wb->urb->transfer_dma = wb->dmah;
	wb->urb->transfer_buffer_length = wb->len;
	wb->urb->dev = dev->udev;

	ret = usb_submit_urb(wb->urb, GFP_ATOMIC);
	if (ret < 0) {
		dev_err(&dev->interface->dev,
			"%s - usb_submit_urb(write bulk) failed: %d\n",
			__func__, ret);
		c8320_usb_write_done(dev, wb);
	}
	
	return ret;
}

static int c8320_usb_write_start(struct usb_dev *dev, int index)
{
	unsigned long flags;
	struct usbdev_wb *wb = &dev->wb[index];
	int ret;

	spin_lock_irqsave(&dev->write_lock, flags);
	
	if (!dev->udev) {
		wb->use = 0;
		spin_unlock_irqrestore(&dev->write_lock, flags);
		return -ENODEV;
	}
	
	usb_autopm_get_interface_async(dev->interface);
	if (dev->susp_count) {
		if (!dev->delayed_wb)
			dev->delayed_wb = wb;
		else
			usb_autopm_put_interface_async(dev->interface);
		spin_unlock_irqrestore(&dev->write_lock, flags);
		return 0;	/* A white lie */
	}
	usb_mark_last_busy(dev->udev);

	ret = c8320_usb_start_wb(dev, wb);
	
	spin_unlock_irqrestore(&dev->write_lock, flags);

	return ret;
}

static int c8320_usb_submit_read_urb(struct usb_dev *dev, int index,
				gfp_t mem_flags)
{
	int res;

	if (!test_and_clear_bit(index, &dev->read_urbs_free))
		return 0;

	dev_vdbg(&dev->interface->dev, "%s - urb %d\n", __func__, index);

	res = usb_submit_urb(dev->read_urbs[index], mem_flags);
	if (res) {
		if (res != -EPERM) {
			dev_err(&dev->interface->dev,
					"%s - usb_submit_urb failed: %d\n",
					__func__, res);
		}
		set_bit(index, &dev->read_urbs_free);
		return res;
	}

	return 0;
}

static int c8320_usb_submit_read_urbs(struct usb_dev *dev, gfp_t mem_flags)
{
	int res;
	int i;

	for (i = 0; i < dev->rx_buflimit; ++i) {
		res = c8320_usb_submit_read_urb(dev, i, mem_flags);
		if (res)
			return res;
	}

	return 0;
}

static void c8320_usb_read_bulk_callback(struct urb *urb)
{
	struct usbdev_rb *rb = urb->context;
	struct usb_dev *dev = rb->instance;
	unsigned long flags;

	printk("%s: status is %d, actual length is %d\n", __func__, 
				urb->status, urb->actual_length);
	
	if (!dev->udev) {
		dev_dbg(&dev->interface->dev, "%s - disconnected\n", __func__);
		printk("%s: failed\n", __func__);
	}
	usb_mark_last_busy(dev->udev);

	if (urb->status) {	
		set_bit(rb->index, &dev->read_urbs_free);
		return;
	}	
	
	/* throttle device if requested by tty */
	spin_lock_irqsave(&dev->read_lock, flags);	
	list_add(&rb->list, &dev->filled_read_bufs);
	spin_unlock_irqrestore(&dev->read_lock, flags);
	wake_up(&dev->read_wq);
}

/* data interface wrote those outgoing bytes */
static void c8320_usb_write_bulk_callback(struct urb *urb)
{
	struct usbdev_wb *wb = urb->context;
	struct usb_dev *dev = wb->instance;
	unsigned long flags;

	if (urb->status	|| (urb->actual_length != urb->transfer_buffer_length)) {
		dev_vdbg(&dev->interface->dev, "%s - len %d/%d, status %d\n",
			__func__,
			urb->actual_length,
			urb->transfer_buffer_length,
			urb->status);
	}

	printk("%s: status is %d, actual length is %d, bffer_length is %d\n",
			__func__, urb->status, urb->actual_length, urb->transfer_buffer_length);

	spin_lock_irqsave(&dev->write_lock, flags);
	c8320_usb_write_done(dev, wb);
	spin_unlock_irqrestore(&dev->write_lock, flags);
}

/*
 * USB probe and disconnect routines.
 */

/* Little helpers: write/read buffers free */
static void c8320_usb_write_buffers_free(struct usb_dev *dev)
{
	int i;
	struct usbdev_wb *wb;
	struct usb_device *usb_dev = interface_to_usbdev(dev->interface);

	for (i = 0; i < USB_NW; i++){
		wb = &dev->wb[i];
		usb_free_coherent(usb_dev, dev->writesize, wb->buf, wb->dmah);
	}
}

static void c8320_usb_read_buffers_free(struct usb_dev *cboot)
{
	struct usb_device *usb_dev = interface_to_usbdev(cboot->interface);
	int i;

	for (i = 0; i < cboot->rx_buflimit; i++)
		usb_free_coherent(usb_dev, cboot->readsize,
			  cboot->rb[i].base, cboot->rb[i].dma);
}

/* Little helper: write buffers allocate */
static int c8320_usb_write_buffers_alloc(struct usb_dev *dev)
{
	int i;
	struct usbdev_wb *wb;

	for (wb = &dev->wb[0], i = 0; i < USB_NW; i++, wb++) {
		wb->buf = usb_alloc_coherent(dev->udev, dev->writesize, GFP_KERNEL, &wb->dmah);
		if (!wb->buf) {
			printk("%s: alloc wb buf failed\n", __func__);
			while (i != 0) {
				--i;
				--wb;
				usb_free_coherent(dev->udev, dev->writesize,
				    wb->buf, wb->dmah);
			}
			return -ENOMEM;
		}
	}
	
	return 0;
}

static void c8320_usb_kill_filled_urb(struct usb_dev *dev)
{	
	unsigned long flags = 0;
	struct usbdev_rb *rb = NULL;
	struct list_head *read_bufs = &dev->filled_read_bufs;
	
	while (!list_empty(&dev->filled_read_bufs)) {
		spin_lock_irqsave(&dev->read_lock, flags);
		rb = list_first_entry(read_bufs, struct usbdev_rb, list);
		list_del(&rb->list);
		spin_unlock_irqrestore(&dev->read_lock, flags);
		
		set_bit(rb->index, &dev->read_urbs_free);
	}
}

static int c8320_usb_open(struct inode *inode, struct file *file)
{
	dev_t devno = inode->i_rdev;
	int minor = MINOR(devno);
	struct usb_dev *dev = usbdev_table[minor];	

	if (dev->opened) {
		printk("%s: device is already opened\n", __func__);
		return -EBUSY;
	}
	dev->opened = 1;
	
	if (!dev->connected) {
		printk("%s: device is not ready\n", __func__);
		return -ENODEV;
	}
	file->private_data = dev;
	c8320_usb_submit_read_urbs(dev, GFP_KERNEL);
	
	return 0;
}

static ssize_t c8320_usb_read(struct file *fp, char __user *buf, 
					size_t count, loff_t *pos)
{
	struct usb_dev *dev = fp->private_data;
	struct usbdev_rb *rb = NULL;
	struct urb *urb = NULL;
	struct list_head *read_bufs = NULL;
	unsigned long flags = 0;
	int length = 0;
	int ret = 0;

	if (!dev->connected) {
		return -ENODEV;
	}
	printk("%s\n", __func__);
	
	if (list_empty(&dev->filled_read_bufs)) {
		ret = wait_event_interruptible(dev->read_wq,
				(!list_empty(&dev->filled_read_bufs) || !dev->connected));
	}
	
	if (!dev->connected) {
		return -ENODEV;
	}

	/* get index */
	read_bufs = &dev->filled_read_bufs;
	
	spin_lock_irqsave(&dev->read_lock, flags);
	rb = list_first_entry(read_bufs, struct usbdev_rb, list);
	list_del(&rb->list);
	spin_unlock_irqrestore(&dev->read_lock, flags);

	/* get urb */
	urb = dev->read_urbs[rb->index];
	length = urb->actual_length;
	length = (length > count) ? count : length;
	
	ret = copy_to_user(buf, rb->base, length);
	
	set_bit(rb->index, &dev->read_urbs_free);
	c8320_usb_submit_read_urb(dev, rb->index, GFP_KERNEL);
	
	printk("%s: length is %d\n", __func__, length);
	
	return length - ret;
}

static ssize_t c8320_usb_write( struct file * file, const char __user * buf,
					size_t count, loff_t *ppos )
{
	struct usb_dev *dev = file->private_data;
	struct usbdev_wb *wb;
	int index = 0, ret = 0;
	
	printk("%s: before write count is %d\n", __func__, count);
	
	index = c8320_usb_alloc_wb(dev);
	if (index < 0) {
		printk("%s: c8320_usb_alloc_wb failed\n", __func__);
		return 0;
	}
	wb = &dev->wb[index];
	
	count = (count > dev->writesize) ? dev->writesize : count;

	ret = copy_from_user(wb->buf, buf, count);
	wb->len = count;

	ret = c8320_usb_write_start(dev, index);
	if (ret < 0){
		printk("%s: write failed \n", __func__);
		return ret;
	}
	printk("%s: after write count is %d\n", __func__, count);
	
	return count;
}

static int c8320_usb_release(struct inode *inode, struct file *filp)	
{
	dev_t devno = inode->i_rdev;
	int minor = MINOR(devno);
	struct usb_dev *dev = usbdev_table[minor];
	
	dev->opened = 0;
	c8320_usb_kill_filled_urb(dev);
	filp->private_data = NULL;	
#ifdef CONFIG_C8320_HSIC_NEW
	c8320_modem_ctl_ex.phone_state = STATE_LOADER_DONE;
#endif
	return 0;
}

static const struct file_operations dev_fops = {
	.owner			= THIS_MODULE,
	.read			= c8320_usb_read,
	.write			= c8320_usb_write,
	.open			= c8320_usb_open,
	.release		= c8320_usb_release,
};

static int c8320_usb_probe(struct usb_interface *interface,
		     const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(interface);
	struct usb_host_interface *iface_desc = NULL;
	struct usb_endpoint_descriptor *endpoint;
	struct usb_endpoint_descriptor *bulk_in_endpoint = NULL;
	struct usb_endpoint_descriptor *bulk_out_endpoint = NULL;
	struct usb_dev *dev = NULL;
	int i = 0;
	int temp = 0;

	printk("===%s start===\n", __func__);
	
	iface_desc = interface->cur_altsetting;
	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
		endpoint = &iface_desc->endpoint[i].desc;

		if (usb_endpoint_is_bulk_in(endpoint)) {
			/* we found a bulk in endpoint */
			printk("found bulk in endpoint, on endpoint %d\n", i);
			bulk_in_endpoint = endpoint;
		}

		if (usb_endpoint_is_bulk_out(endpoint)) {
			/* we found a bulk out endpoint */
			printk("found bulk out endpoint, on endpoint %d\n", i);
			bulk_out_endpoint = endpoint;
		}
	}

	if ((le16_to_cpu(udev->descriptor.idVendor) == 0x4050) &&
		(le16_to_cpu(udev->descriptor.idProduct) == 0x0040)) {
		dev = usbdev_table[0];
	}
	
	if (!bulk_in_endpoint || !bulk_out_endpoint) {
		printk("%s: endpoint is not complete\n", __func__);
		return 0;
	}


	dev->interface = interface;
	dev->udev = udev;
	mutex_init(&dev->mutex);
	spin_lock_init(&dev->write_lock);
	spin_lock_init(&dev->read_lock);
	init_waitqueue_head(&dev->read_wq);

	dev->rx_endpoint = usb_rcvbulkpipe(udev, bulk_in_endpoint->bEndpointAddress);
	dev->tx_endpoint = usb_sndbulkpipe(udev, bulk_out_endpoint->bEndpointAddress);
	dev->writesize = usb_endpoint_maxp(bulk_out_endpoint) * USB_MAX_WRITE;
	dev->readsize = usb_endpoint_maxp(bulk_in_endpoint) * USB_MAX_READ;
	
	if (c8320_usb_write_buffers_alloc(dev) < 0) {
		dev_err(&interface->dev, "out of memory (write buffer alloc)\n");
		goto alloc_fail1;
	}
	dev->rx_buflimit = 0;

	/* init read urbs */
	for (i = 0; i < USB_NR; i++) {
		struct usbdev_rb *rb = &(dev->rb[i]);
		struct urb *urb;

		rb->base = usb_alloc_coherent(dev->udev, dev->readsize, GFP_KERNEL, &rb->dma);
		if (!rb->base) {
			dev_err(&interface->dev, "out of memory "
					"(read bufs usb_alloc_coherent)\n");
			goto alloc_fail2;
		}
		rb->index = i;
		rb->instance = dev;

		urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!urb) {
			usb_free_coherent(dev->udev, dev->readsize, dev->rb[i].base, 
							dev->rb[i].dma);
			dev_err(&interface->dev,
				"out of memory (read urbs usb_alloc_urb)\n");
			goto alloc_fail2;
		}
		urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		urb->transfer_dma = rb->dma;
		usb_fill_bulk_urb(urb, dev->udev,
					  dev->rx_endpoint,
					  rb->base,
					  dev->readsize,
					  c8320_usb_read_bulk_callback, rb);

		dev->read_urbs[i] = urb;
		dev->rx_buflimit++;
		__set_bit(i, &dev->read_urbs_free);
	}

	/* init write urbs */
	for (i = 0; i < USB_NW; i++,temp++) {
		struct usbdev_wb *snd = &(dev->wb[i]);

		snd->urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!snd->urb) {
			dev_err(&interface->dev,
				"out of memory (write urbs usb_alloc_urb)\n");
			goto alloc_fail3;
		}

		usb_fill_bulk_urb(snd->urb, udev,
				dev->tx_endpoint,
				NULL, 
				dev->writesize, 
				c8320_usb_write_bulk_callback, snd);
		
		snd->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		snd->instance = dev;
	}

	device_create(c8320_usbclass, &interface->dev, MKDEV(dev->major, dev->minor), 
				NULL, dev->cdev_name);

	usb_set_intfdata(interface, dev);
	dev->connected = 1;

	printk("===%s end===\n", __func__);
	
	return 0;
	
alloc_fail3:
	for (i = 0; i < temp; i++)
		usb_free_urb(dev->wb[i].urb);
	
alloc_fail2:
	for (i = 0; i < dev->rx_buflimit; i++)
		usb_free_urb(dev->read_urbs[i]);
	c8320_usb_read_buffers_free(dev);
	c8320_usb_write_buffers_free(dev);
	
alloc_fail1:
	printk("%s: alloc write buf failed\n",__func__);
	return -ENOMEM;
}

static void stop_data_traffic(struct usb_dev *dev)
{
	int i;

	dev_dbg(&dev->interface->dev, "%s\n", __func__);

	for (i = 0; i < USB_NW; i++)
		usb_kill_urb(dev->wb[i].urb);
	for (i = 0; i < dev->rx_buflimit; i++)
		usb_kill_urb(dev->read_urbs[i]);
}

static void c8320_usb_disconnect(struct usb_interface *intf)
{
	int i;
	struct usb_dev *dev = usb_get_intfdata(intf);

	dev_dbg(&intf->dev, "%s\n", __func__);

	if (!dev) {
		printk("sibling interface is already cleaning up\n");
	}

	mutex_lock(&dev->mutex);	

	usb_set_intfdata(dev->interface, NULL);	
	dev->connected = 0;
	mutex_unlock(&dev->mutex);
	wake_up(&dev->read_wq);
	device_destroy(c8320_usbclass, MKDEV(dev->major, dev->minor));
	c8320_usb_kill_filled_urb(dev);
	
	stop_data_traffic(dev);

	for (i = 0; i < USB_NW; i++)
		usb_free_urb(dev->wb[i].urb);
	for (i = 0; i < dev->rx_buflimit; i++)
		usb_free_urb(dev->read_urbs[i]);

	c8320_usb_write_buffers_free(dev);	
	c8320_usb_read_buffers_free(dev);		
}

#ifdef CONFIG_PM
static int c8320_usb_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct usb_dev *cboot = usb_get_intfdata(intf);
	int cnt;

	if (PMSG_IS_AUTO(message)) {
		int b;

		spin_lock_irq(&cboot->write_lock);
		b = cboot->transmitting;
		spin_unlock_irq(&cboot->write_lock);
		if (b)
			return -EBUSY;
	}

	spin_lock_irq(&cboot->read_lock);
	spin_lock(&cboot->write_lock);
	cnt = cboot->susp_count++;
	spin_unlock(&cboot->write_lock);
	spin_unlock_irq(&cboot->read_lock);

	if (cnt)
		return 0;

	stop_data_traffic(cboot);

	return 0;
}

static int c8320_usb_resume(struct usb_interface *intf)
{
	struct usb_dev *dev = usb_get_intfdata(intf);
	struct usbdev_wb *wb;
	int rv = 0;
	int cnt;

	spin_lock_irq(&dev->read_lock);
	dev->susp_count -= 1;
	cnt = dev->susp_count;
	spin_unlock_irq(&dev->read_lock);

	if (cnt)
		return 0;

	spin_lock_irq(&dev->write_lock);
	if (dev->delayed_wb) {
		wb = dev->delayed_wb;
		dev->delayed_wb = NULL;
		spin_unlock_irq(&dev->write_lock);
		c8320_usb_start_wb(dev, wb);
	} else {
		spin_unlock_irq(&dev->write_lock);
	}

		/*
		 * delayed error checking because we must
		 * do the write path at all cost
		 */
	if (rv < 0)
		goto err_out;

	rv = c8320_usb_submit_read_urbs(dev, GFP_NOIO);

err_out:
	return rv;
}

static int c8320_usb_reset_resume(struct usb_interface *intf)
{
	return c8320_usb_resume(intf);
}

#endif /* CONFIG_PM */

/*
 * USB driver structure.
 */

static const struct usb_device_id c8320_ids[] = {
	/* Support for Owen devices */
	{ USB_DEVICE(0x4050, 0x0040), }, /* c8320 */
	{ }
};
MODULE_DEVICE_TABLE(usb, acm_ids);

static struct usb_driver c8320_usb_driver = {
	.name =		"c8320_usb",
	.probe =	c8320_usb_probe,
	.disconnect =	c8320_usb_disconnect,
#ifdef CONFIG_PM
	.suspend =	c8320_usb_suspend,
	.resume =	c8320_usb_resume,
	.reset_resume =	c8320_usb_reset_resume,
	.supports_autosuspend = 1,
#endif
	.id_table =	c8320_ids,
};

static int __init c8320_usb_init(void)
{
	int ret = 0, n = 0;
	dev_t devno;
	
	ret = alloc_chrdev_region(&devno, 0, MAX_DEV, "c8320_usb");
	if (ret < 0) {
		printk("%s: Alloc c8320_usb devno failed\n", __func__);
		return ret;
	} else {
		major = MAJOR(devno);
	}
	
	cdev_init(&usbdev, &dev_fops);
	//usbdev.owner = THIS_MODULE;
	ret = cdev_add(&usbdev, devno, MAX_DEV);	

	c8320_usbclass = class_create(THIS_MODULE, "c8320_usb");
	if (IS_ERR(c8320_usbclass)) {
		printk("%s: create c8320 usb class failed\n", __func__);
		return PTR_ERR(c8320_usbclass);
	}

	for (n = 0; n < MAX_DEV; n++) {
		usbdev_table[n] = kzalloc(sizeof(struct usb_dev), GFP_KERNEL);
		if (!usbdev_table[n]) {
			printk("%s: kzalloc usbdev_table[%d] memory failed\n", __func__, n);
			return -ENOMEM;
		}
		if (n == 0) {
			usbdev_table[n]->cdev_name = kasprintf(GFP_KERNEL,"c8320_usb");
		}

		usbdev_table[n]->major = major;
		usbdev_table[n]->minor = n;
		usbdev_table[n]->connected = 0;
		usbdev_table[n]->cdev = &usbdev;
		INIT_LIST_HEAD(&usbdev_table[n]->filled_read_bufs);
	}
	
	ret = usb_register(&c8320_usb_driver);
	if (ret < 0) {
		printk("%s: usb_register failed\n", __func__);
		return ret;
	}

	return 0;
}

static void __exit c8320_usb_exit(void)
{
	int n = 0;
	
	usb_deregister(&c8320_usb_driver);
	unregister_chrdev_region(MKDEV(major, 0), MAX_DEV);
	for (n = 0; n < MAX_DEV; n++) {
		kfree(usbdev_table[n]);
	}
	class_destroy(c8320_usbclass);
}

module_init(c8320_usb_init);
module_exit(c8320_usb_exit);
MODULE_LICENSE("GPL");
