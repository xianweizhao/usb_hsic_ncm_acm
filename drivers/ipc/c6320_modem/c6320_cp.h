/* Copyright(C) 2005-2013,CYIT. Co., Ltd. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 */

#include <linux/platform_device.h>
#include <mach/ipc.h>

#define IOC_CP_STOP 		 _IOW('C', 0, int)
#define IOC_CP_START 		 _IOW('C', 1, int)
#define IOC_CP_POWON 		 _IOW('C', 2, int)
#define IOC_CP_POWOFF 		 _IOW('C', 3, int)
#define IOC_CP_LOOPBACK  	 _IOW('C', 4, int)
#define IOC_CP_DHI_SM  		 _IOW('C', 5, int)
#define IOC_CP_ZSP_IMEM  	 _IOW('C', 6, int)
#define IOC_CP_ZSP_DMEM  	 _IOW('C', 7, int)
#define IOC_CP_DDR_MEM  	 _IOW('C', 8, int)
#define IOC_CP_CEVA_SM  	 _IOW('C', 9, int)
#define IOC_CP_INITRAM		 _IOW('C', 10, int)
#define IOC_AP_DDR_MEM	 	 _IOW('C', 11, int)
#define IOC_IPC_MEM			 _IOW('C', 12, int)

/* dump cp and zsp mem to file:  */
#define DHI_SM_FILE		"/data/corelog/DHI_MEM.bin"
#define ZSP_IMEM_FILE	"/data/corelog/ZSP_IMEM.bin"
#define ZSP_DMEM_FILE	"/data/corelog/ZSP_DMEM.bin"
#define CP_DDR_FILE		"/data/corelog/CPDDR.bin"
#define CEVA_SM_FILE	"/data/corelog/CEVA_SM.bin"
#define INITRAM_FILE	"/data/corelog/INITRAM.bin"
#define AP_DDR_FILE		"/data/corelog/APDDR.bin"
#define IPC_MEM_FILE	"/data/corelog/IPC_MEM.bin"

#define CP_DHI_SM_PADDR    (0x21000000)
#define CP_DHI_SM_SIZE        (0x1800)

#define CP_ZSP_IMEM_PADDR   (0x23040000)
#define CP_ZSP_IMEM_SIZE       (0x36000)

#define CP_ZSP_DMEM_PADDR   (0x23000000)
#define CP_ZSP_DMEM_SIZE       (0x2E000)

#define CP_CEVA_SM_PADDR   (0x28801000)
#define CP_CEVA_SM_SIZE       (0xc00)

#define CP_INITRAM_PADDR	(0x27000000)
#define CP_INITRAM_SIZE		(0X1000)

#define AP_DDR_PADDR		(0x40000000)
#define AP_DDR_SIZE			(0x40000000)

#define RESET_CP	(1)
#define LINUX_PANIC	(2)
#define AT_TIMEOUT	(3)


enum pmic_req {
	pmic_sim1_vol_ctl_req = 1,
	pmic_sim2_vol_ctl_req,
	pmic_battery_vol_get_req,
};


static struct ipc_channel ipc_channel_devctl =
{
    .name = IPC_CHANNEL_DEVCTL_NAME,
    .type = IPC_TYPE_PACKET,
    .id = IPC_CHANNEL_DEVCTL_ID,
    .c2a_buf_len = CHANNEL_BUF_2KB,
    .a2c_buf_len = CHANNEL_BUF_1KB,
    .wake_flag = CH_WAKEUP_ENABLE,
};

static struct ipc_channel ipc_channel_devctl_event =
{
	.name = IPC_CHANNEL_DEVCTL_EVENT_NAME,
	.type = IPC_TYPE_EVENT,
	.id = IPC_CHANNEL_DEVCTL_EVENT_ID,
	.wake_flag = CH_WAKEUP_ENABLE,
};

static struct ipc_channel ipc_channel_pmic_req =
{
    .name = IPC_CHANNEL_PMIC_REQ_NAME,
    .type = IPC_TYPE_EVENT,
    .id = IPC_CHANNEL_PMIC_REQ_ID,
    .wake_flag = CH_WAKEUP_ENABLE,
};

static struct ipc_channel ipc_channel_pmic_cnf =
{
    .name = IPC_CHANNEL_PMIC_CNF_NAME,
    .type = IPC_TYPE_EVENT,
    .id = IPC_CHANNEL_PMIC_CNF_ID,
    .wake_flag = CH_WAKEUP_ENABLE,
};

struct T_C2A_STATUS_NOTIFY{
      int type;
      int len;
      unsigned char data[200];
};

struct T_A2C_DEVCTL{
      int type;
      int len;
      unsigned long data;
};