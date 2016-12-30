/* 
*
* Copyright(C) 2005-2013,CYIT. Co., Ltd. All rights reserved.
* File name:  ipc.h
* Author:  
* Description:  ipc header
* Version: V1.0
* History:
*     <author>          <time>           <version>          <desc>    
*
*
*/


#ifndef	_IPC_H
#define	_IPC_H

typedef struct ipc_channel{
	char name[64]; /* channel name, include '\0' char */
	unsigned type;
	unsigned id;
	unsigned wake_flag;
	unsigned c2a_buf_len;
	unsigned a2c_buf_len;
	unsigned reserve;
}ipc_ch_t;


typedef enum {
	CPU_WORK_STAT=0,
	CPU_SLEEP_STAT=1
}cpu_stat_t;


typedef enum{
	W_AVAIL_INT=1,
	R_AVAIL_INT=2,
	RW_AVAIL_INT=3
} ch_int_t;

/*interrupt handle function event parameter*/
typedef enum{
	W_AVAIL = 1,
	R_AVAIL = 2,
	RW_AVIL =3
} ch_event_type;


typedef enum{
	IPC_TYPE_STREAM = 0,
	IPC_TYPE_PACKET = 1,
	IPC_TYPE_EVENT = 2,
	IPC_TYPE_MAX
} ch_type;


typedef enum{
	CH_WAKEUP_DISENABLE = 0,
	CH_WAKEUP_ENABLE
} ch_wake_t;


typedef enum{
    IPC_CHANNEL_PMIC_REQ_ID = 1,
	IPC_CHANNEL_PMIC_CNF_ID,
	IPC_CHANNEL_AT1_ID = 11,
	IPC_CHANNEL_AT2_ID,
	IPC_CHANNEL_CS_ID,
	IPC_CHANNEL_VT_ID,
	IPC_CHANNEL_VM_ID,
	IPC_CHANNEL_PS1_ID,
	IPC_CHANNEL_PS2_ID,
	IPC_CHANNEL_PS3_ID,
	IPC_CHANNEL_PS4_ID,
	IPC_CHANNEL_PS5_ID,
	IPC_CHANNEL_PS6_ID,
	IPC_CHANNEL_DEVCTL_ID,
	IPC_CHANNEL_DEVCTL_EVENT_ID,
	IPC_CHANNEL_NVM_ID,
	IPC_CHANNEL_ARMLOG_ID,
	IPC_CHANNEL_ZSPLOG_ID,
	IPC_CHANNEL_ID_MAX
} ch_id_t;

#define	IPC_CHANNEL_PMIC_REQ_NAME              "IPC_PMIC_REQ"
#define	IPC_CHANNEL_PMIC_CNF_NAME              "IPC_PMIC_CNF"
#define	IPC_CHANNEL_AT1_NAME	               "IPC_AT1"
#define	IPC_CHANNEL_AT2_NAME                   "IPC_AT2"
#define	IPC_CHANNEL_CS_NAME                    "IPC_CS"
#define	IPC_CHANNEL_VT_NAME                    "IPC_VT"
#define	IPC_CHANNEL_VM_NAME                    "IPC_VM"
#define	IPC_CHANNEL_PS1_NAME				"IPC_PS1"
#define	IPC_CHANNEL_PS2_NAME				"IPC_PS2"
#define	IPC_CHANNEL_PS3_NAME				"IPC_PS3"
#define	IPC_CHANNEL_PS4_NAME				"IPC_PS4"
#define	IPC_CHANNEL_PS5_NAME				"IPC_PS5"
#define	IPC_CHANNEL_PS6_NAME				"IPC_PS6"
#define	IPC_CHANNEL_DEVCTL_NAME			"IPC_DEVCTL"
#define	IPC_CHANNEL_DEVCTL_EVENT_NAME	"IPC_DEVCTL_EVENT"
#define	IPC_CHANNEL_NVM_NAME			"IPC_NVM"
#define	IPC_CHANNEL_ARMLOG_NAME			"IPC_ARMLOG"
#define	IPC_CHANNEL_ZSPLOG_NAME			"IPC_ZSPLOG"

typedef enum{
        CHANNEL_BUF_512B = 512,
	    CHANNEL_BUF_1KB = 1024,
	    CHANNEL_BUF_2KB = 2*1024,
	    CHANNEL_BUF_4KB = 4*1024,
	    CHANNEL_BUF_8KB = 8*1024,
	    CHANNEL_BUF_16KB = 16*1024,
	    CHANNEL_BUF_32KB = 32 *1024,
	    CHANNEL_BUF_64KB = 64*1024,
	    CHANNEL_BUF_128KB = 128*1024,
	    CHANNEL_BUF_256KB = 256*1024,
	    CHANNEL_BUF_512KB = 512*1024,
	    CHANNEL_BUF_1MB = 1024*1024,
	    CHANNEL_BUF_2MB = 2*1024*1024
}  ch_buflen_t;


int hacm_create_ch(struct ipc_channel *ch);
int hacm_read_avail(int id);
int hacm_write_avail(int id);
int hacm_read(int id, void *data, unsigned len);
int hacm_write(int id, const void *data, unsigned len);

int hacm_read_to_user(int id, void __user *data, unsigned len);

int hacm_write_from_user(int id, const void __user *data, unsigned len);
int hacm_register_inthandle(int id, void *priv, long (*handle)(int id, void *priv, int event));
int hacm_enable_interrupt(int id, unsigned flag);
int hacm_disable_interrupt(int id, unsigned flag);
int hacm_find_ch(char *name);
int hacm_set_localcpu_status(unsigned status);
int hacm_get_remotecpu_status(void);
int hacm_reset(void);
int hacm_destroy_ch(int id);
int hacm_reset_ch(int id);
int hacm_init(void);
void hacm_genint_to_other(void);
void hacm_genint_to_self(void);

#define ipc_create_ch      hacm_create_ch
#define ipc_destroy_ch    hacm_destroy_ch
#define ipc_read_avail     hacm_read_avail
#define ipc_write_avail    hacm_write_avail
#define ipc_read              hacm_read
#define ipc_write             hacm_write
#define ipc_read_to_user         hacm_read_to_user
#define ipc_write_from_user    hacm_write_from_user
#define ipc_register_inthandle  hacm_register_inthandle
#define ipc_enable_interrupt    hacm_enable_interrupt
#define ipc_disable_interrupt    hacm_disable_interrupt
#define ipc_find_ch                  hacm_find_ch
#define ipc_set_localcpu_status         hacm_set_localcpu_status
#define ipc_get_remotecpu_status    hacm_get_remotecpu_status
#define ipc_reset     hacm_reset
#define ipc_reset_ch     hacm_reset_ch
#define ipc_init        hacm_init
#define ipc_genint_to_other    hacm_genint_to_other
#define ipc_genint_to_other    hacm_genint_to_other

#endif /* _IPC_H */

