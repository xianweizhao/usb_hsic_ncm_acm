# Makefile of modem_if

EXTRA_CFLAGS += -Idrivers/ipc_c8320/modem_v2

obj-y += sipc5_modem.o sipc5_io_device.o
obj-y += modem_net_flowcontrol_device.o modem_utils.o

obj-$(CONFIG_LTE_MODEM_C8320) += modem_modemctl_device_c8320.o

obj-$(CONFIG_LINK_DEVICE_HSIC) += modem_link_device_hsic_ncm.o link_usb_cdc_ncm.o
