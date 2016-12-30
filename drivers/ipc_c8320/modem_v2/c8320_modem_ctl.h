/* Does modem ctl structure will use state ? or status defined below ?*/
#ifndef _C8320_MODEM_CTL_H
#define _C8320_MODEM_CTL_H
enum modem_state {
	STATE_OFFLINE, // serial load and before
	STATE_BOOTING, // enum usb 2 usb loader
	STATE_LOADER_DONE, //usb loader compete 
	STATE_ONLINE, // normal
	STATE_NORMAL_SLEEP, /* usb sleep need connect*/
	STATE_RESUME, /* cp resume */
	STATE_CRASH, // bulescreen	
	STATE_RESET_CP,
};

enum com_state {
	COM_NONE,
	COM_ONLINE,
	COM_RECONNECT,
	COM_CRASH,
};
#define SLAVE_DEBUG_TEST  1<<0
#define CPSWITCH_DEBUG_TEST  1<<1
#define ONLY_SLEEP_ONE_TIME  1<<2
#define FORBID_SWITCH_USBDEV  	1<<3
struct c8320_modem_ctl_extern {
	enum	modem_state phone_state;
	enum	com_state com_state;
	int 	debugmask;
	bool    cmd_send;
	bool    force_hsic_poweroff;
};
extern struct c8320_modem_ctl_extern c8320_modem_ctl_ex;
extern void sc3_set_phy2dev(void);
#endif
