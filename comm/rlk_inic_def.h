#ifndef __RLK_INIC_DEF_H__
#define __RLK_INIC_DEF_H__

#include <linux/kernel.h>
#include <linux/compiler.h>
#include <linux/module.h>
//#include <linux/config.h>
//#include <linux/autoconf.h>//peter : for FC6
#include <generated/autoconf.h>//peter : for FC6
#include <linux/version.h>
#include <linux/byteorder/generic.h>
#include <linux/unistd.h>
#include <linux/ctype.h>
#include <linux/wireless.h>
#include <linux/notifier.h>
#include <linux/list.h>
#include <linux/if_bridge.h>
#include <linux/netdevice.h>
#include <linux/kfifo.h>
#include <net/iw_handler.h>
#include "crc.h"

#define INIC_INF_TYPE_PCI 0
#define INIC_INF_TYPE_MII 1
#define INIC_INF_TYPE_USB 2


#ifndef SET_MODULE_OWNER
#define SET_MODULE_OWNER(dev) do {} while(0)
#endif

#ifndef INT32
	#define INT32        int
#endif

#ifndef UINT32
	#define UINT32       unsigned int
#endif

#define INIC_INFNAME			"wl"
#define FIRMWARE_PATH "iNIC/" CHIP_NAME INTERFACE "/"

#if (CONFIG_INF_TYPE == INIC_INF_TYPE_MII)
	#ifdef PHASE_LOAD_CODE
		#define PHASE_FIRMWARE			FIRMWARE_PATH "iNIC.bin"
	#endif
#endif

#define AP_FIRMWARE				FIRMWARE_PATH "iNIC_ap.bin"
#define AP_PROFILE				FIRMWARE_PATH "iNIC_ap.dat"
#define AP_CONCURRENT_PROFILE 	FIRMWARE_PATH "iNIC_ap1.dat"


#define STA_FIRMWARE			FIRMWARE_PATH "iNIC_sta.bin"
#define STA_PROFILE				FIRMWARE_PATH "iNIC_sta.dat"
#define STA_CONCURRENT_PROFILE 	FIRMWARE_PATH "iNIC_sta1.dat"

#define fATE_LOAD_EEPROM		0x0C43
#define EEPROM_SIZE				0x200
#define EEPROM_BIN 				FIRMWARE_PATH "iNIC_e2p.bin"

#define CONCURRENT_EEPROM_BIN	FIRMWARE_PATH "iNIC_e2p1.bin"


#if defined(MULTIPLE_CARD_SUPPORT) || defined(CONFIG_CONCURRENT_INIC_SUPPORT)

#define STA_CARD_INFO			FIRMWARE_PATH "iNIC_sta_card.dat"
#define AP_CARD_INFO			FIRMWARE_PATH "iNIC_ap_card.dat"
// MC: Multple Cards
#define MAX_NUM_OF_MULTIPLE_CARD		32

// record whether the card in the card list is used in the card file
extern u8  MC_CardUsed[MAX_NUM_OF_MULTIPLE_CARD];
// record used card irq in the card list
extern s32 MC_CardIrq[MAX_NUM_OF_MULTIPLE_CARD];


#endif // MULTIPLE_CARD_SUPPORT //


/* hardware minimum and maximum for a single frame's data payload */
#define RLK_INIC_MIN_MTU		64	/* TODO: allow lower, but pad */
#define RLK_INIC_MAX_MTU		1536

#define MAC_ADDR_LEN         6
#define MBSS_MAX_DEV_NUM    32  /* for OS possible ra%d name    */
#define WDS_MAX_DEV_NUM     32  /* for OS possible wds%d name   */
#define APCLI_MAX_DEV_NUM   32  /* for OS possible apcli%d name */
#ifdef MESH_SUPPORT
#define MESH_MAX_DEV_NUM    32  /* for OS possible mesh%d name */
#endif // MESH_SUPPORT //
#define MAX_MBSSID_NUM       8
#define MAX_WDS_NUM          4
#define MAX_APCLI_NUM        1
#ifdef MESH_SUPPORT
#define MAX_MESH_NUM         1
#endif // MESH_SUPPORT //
#define FIRST_MBSSID         1
#define FIRST_WDSID          0
#define FIRST_APCLIID        0
#ifdef MESH_SUPPORT
#define FIRST_MESHID         0
#endif // MESH_SUPPORT //
#define MIN_NET_DEVICE_FOR_MBSSID       0x00 /* defined in iNIC firmware */
#define MIN_NET_DEVICE_FOR_WDS          0x10 /* defined in iNIC firmware */
#define MIN_NET_DEVICE_FOR_APCLI        0x20 /* defined in iNIC firmware */
#ifdef MESH_SUPPORT
#define MIN_NET_DEVICE_FOR_MESH         0x30 /* defined in iNIC firmware */
#endif // MESH_SUPPORT //

#define IFNAMSIZ            16
// !! Be careful to not interfere with enum netdev_priv_flags definition in linux/include/linux/netdevice.h !!
#define INT_MAIN        0x0
#define INT_MBSSID      1<<29
#define INT_WDS         1<<30
#define INT_APCLI       1<<31
// Mesh support isn't supported
//#ifdef MESH_SUPPORT
//#define INT_MESH        0x500
//#endif // MESH_SUPPORT //

#define MAX_INI_BUFFER_SIZE			4096
#define MAX_PARAM_BUFFER_SIZE		(2048) 
#define ETH_MAC_ADDR_STR_LEN 17  		   // in format of xx:xx:xx:xx:xx:xx



#define MAX_LEN_OF_RACFG_QUEUE 		32
#define MAX_FEEDBACK_LEN			1024
#define MAX_PROFILE_SIZE            (8*MAX_FEEDBACK_LEN)
#define MAX_FILE_NAME_SIZE			256
#define MAX_EXT_EEPROM_SIZE			1024

#define NETIF_IS_UP(x)              (x->flags & IFF_UP)

#if (CONFIG_INF_TYPE!=INIC_INF_TYPE_MII)
#define GET_PARENT(x)  (((VIRTUAL_ADAPTER *)netdev_priv(x))->RtmpDev)
#else
#define GET_PARENT(x)  (((iNIC_PRIVATE *)netdev_priv(((VIRTUAL_ADAPTER *)netdev_priv(x))->RtmpDev))->RaCfgObj.MBSSID[0].MSSIDDev)
#endif

#ifdef RLK_INIC_NDEBUG
#define ASSERT(expr)
#else
#define ASSERT(expr) \
        if(unlikely(!(expr))) {                                   \
        printk(KERN_ERR "Assertion failed! %s,%s,%s,line=%d\n", \
        #expr,__FILE__,__FUNCTION__,__LINE__);          \
        }
#endif


#define SK_RECEIVE_QUEUE sk_receive_queue
#define PCI_SET_CONSISTENT_DMA_MASK pci_set_consistent_dma_mask
#define SK_SOCKET sk_socket

#ifndef __iomem
#define __iomem
#endif


#define DEV_SHORTCUT(ndev)  ((ndev)->dev.parent)

#define DEV_GET_BY_NAME(name) dev_get_by_name(&init_net, name);

#ifndef SET_NETDEV_DEV
#define SET_NETDEV_DEV(ndev, pdev)  (DEV_SHORTCUT(ndev) = (pdev)) //do { } while (0)
#endif

#ifndef SEEK_SET
#define SEEK_SET 0
#endif

#ifndef SEEK_END
#define SEEK_END 2
#endif

//#define DBG

#ifdef DBG
#define DBGPRINT(fmt, args...)      printk(fmt, ## args)
#else
#define DBGPRINT(fmt, args...)
#endif


#ifndef TRUE
#define TRUE              (1)
#define FALSE             (0)
#endif


#define MEM_ALLOC_FLAG      (GFP_ATOMIC) //(GFP_DMA | GFP_ATOMIC)

#define MAGIC_NUMBER			0x18142880

#ifndef TRUE
#define FALSE 0
#define TRUE (!FALSE)
#endif


typedef spinlock_t RTMP_SPIN_LOCK;

#define RTMP_SEM_LOCK(__lock)                   \
{                                               \
    spin_lock_bh((spinlock_t *)(__lock));       \
}

#define RTMP_SEM_UNLOCK(__lock)                 \
{                                               \
    spin_unlock_bh((spinlock_t *)(__lock));     \
}
#define RTMP_SEM_INIT(__lock)            \
{                                               \
    spin_lock_init((spinlock_t *)(__lock));     \
}

typedef void (*PRaCfgFunc)(uintptr_t);
typedef void (*pHndlFunc)(void*);

typedef struct _hndl_task
{
	pHndlFunc func;
    void * arg;
} HndlTask;


typedef struct m_file_handle
 {
 	char name[MAX_FILE_NAME_SIZE];

 #if defined(MULTIPLE_CARD_SUPPORT) || defined(CONFIG_CONCURRENT_INIC_SUPPORT)
 	/* read name form card file */
 	char read_name[MAX_FILE_NAME_SIZE];
 #endif // MULTIPLE_CARD_SUPPORT || CONFIG_CONCURRENT_INIC_SUPPORT
 	int  seq;
    u32  crc;
    CRC_HEADER hdr;
    unsigned char hdr_src[CRC_HEADER_LEN];
    char * fw_data;
    size_t size;
    loff_t r_off;
 } FWHandle;

typedef unsigned char   boolean;

typedef struct _MULTISSID_STRUCT
{
	unsigned char       Bssid[MAC_ADDR_LEN];
	struct net_device   *MSSIDDev;	
	u16                 vlan_id;  
	u16                 flags;
	u8					bVLAN_tag;
} MULTISSID_STRUCT, *PMULTISSID_STRUCT;

typedef struct _VIRTUAL_ADAPTER
{
	struct net_device       *RtmpDev;
	struct net_device       *VirtualDev;
	struct net_device_stats     net_stats;
} VIRTUAL_ADAPTER, PVIRTUAL_ADAPTER;


#define MAX_NOTIFY_DROP_NUMBER	5

//#define TEST_BOOT_RECOVER 1

#ifdef RETRY_PKT_SEND
#define RetryTimeOut 60    // default 50ms, better set timeout >= 50ms
#define FwRetryCnt	10		 // retry counts for fw upload
#define InbandRetryCnt	3	// retry counts for inband

typedef struct _RETRY_PKT_INFO
{
	int Seq;
	int BootType;
	int length;
	int retry;
	u8 buffer[MAX_FEEDBACK_LEN];
} RETRY_PKT_INFO;

#endif

typedef struct __RACFG_OBJECT
{
	struct net_device           *MainDev;
	struct task_struct          *config_thread_task;
	struct task_struct          *backlog_thread_task;
	unsigned char               opmode;						// 0: STA, 1: AP
	boolean                     bBridge;					// use built-in bridge or not ?
    boolean                     bChkSumOff;
	unsigned char               BssidNum;
	unsigned char               bWds;						// enable wds interface not
	unsigned char               bApcli;						// enable apclient interface
#ifdef MESH_SUPPORT
	unsigned char               bMesh;						// enable mesh interface
#endif // MESH_SUPPORT //
	MULTISSID_STRUCT            MBSSID[MAX_MBSSID_NUM];
	MULTISSID_STRUCT            WDS[MAX_WDS_NUM];
	MULTISSID_STRUCT            APCLI[MAX_APCLI_NUM];
#ifdef MESH_SUPPORT
	MULTISSID_STRUCT            MESH[MAX_MESH_NUM];
#endif // MESH_SUPPORT //
	boolean                     flg_is_open;
	boolean                     flg_mbss_init;
	boolean                     flg_wds_init;
	boolean                     flg_apcli_init;
#ifdef MESH_SUPPORT
	boolean                     flg_mesh_init;
#endif // MESH_SUPPORT //
	boolean                     wds_close_all;
	boolean                     mbss_close_all;
	boolean                     apcli_close_all;
#ifdef MESH_SUPPORT
	boolean                     mesh_close_all;
#endif // MESH_SUPPORT //
	boolean                     bRestartiNIC;
	boolean						bGetMac;
	struct timer_list           heartBeat;					// timer to check the iNIC heart beat
	RTMP_SPIN_LOCK              timerLock;					// for heartBeat timer

#ifdef RETRY_PKT_SEND
	struct timer_list           uploadTimer;					// timer to check the iNIC upload process
	RETRY_PKT_INFO 		RPKTInfo;
#endif

	int                         wait_completed;
	RTMP_SPIN_LOCK              waitLock;
	DECLARE_KFIFO(task_fifo, HndlTask, MAX_LEN_OF_RACFG_QUEUE);
	wait_queue_head_t			taskQH;
	DECLARE_KFIFO(backlog_fifo, HndlTask, MAX_LEN_OF_RACFG_QUEUE);
	wait_queue_head_t			backlogQH;
	DECLARE_KFIFO(wait_fifo, HndlTask, MAX_LEN_OF_RACFG_QUEUE);
	wait_queue_head_t           waitQH;

	u8                          packet[1536];
	u8                          wsc_updated_profile[MAX_PROFILE_SIZE];
	FWHandle                  	firmware, profile, ext_eeprom;
	boolean               		bGetExtEEPROMSize;			 // for upload profile
	boolean               		bExtEEPROM;					 // enable external eeprom not
	unsigned int 				ExtEEPROMSize;				 // for external eeprom
	char                        test_pool[MAX_FEEDBACK_LEN];
	char                        cmp_buf[MAX_FEEDBACK_LEN];
	char                        upload_buf[MAX_FEEDBACK_LEN];
	mm_segment_t                orgfs;
	boolean                     bDebugShow;
	unsigned char               HeartBeatCount;
	//unsigned char               RebootCount;
	struct notifier_block       net_dev_notifier;
#ifdef TEST_BOOT_RECOVER
    boolean                     cfg_simulated;
    boolean                     firm_simulated;
#endif
    int                         extraProfileOffset;
#if defined(MULTIPLE_CARD_SUPPORT) || defined(CONFIG_CONCURRENT_INIC_SUPPORT)
	s8					InterfaceNumber;
	u8					bRadioOff;
#endif 

#ifdef MULTIPLE_CARD_SUPPORT
	s8					MC_RowID;
	u8					MC_MAC[20];
#endif // MULTIPLE_CARD_SUPPORT //

#if (CONFIG_INF_TYPE==INIC_INF_TYPE_MII)
	struct sk_buff *eapol_skb;	
	u16 eapol_command_seq;
	u16 eapol_seq;
	boolean               		bLocalAdminAddr;			// for locally administered address
#ifdef PHASE_LOAD_CODE
	boolean						bLoadPhase;					//0:initail; 1:load 
	u8							dropNotifyCount;					//drop notify
#endif
#endif
    boolean                     bWpaSupplicantUp;
    int         fw_upload_counter;
    
#ifdef WOWLAN_SUPPORT
	struct notifier_block		pm_wow_notifier;
	unsigned long				pm_wow_state;
	boolean                     bWoWlanUp;
	struct timer_list           WowInbandSignalTimer;					// timer to check the WOWLAN heart beat
	RTMP_SPIN_LOCK              WowInbandSignalTimerLock;				// for WowInbandSignalTimer timer
	u8							WowInbandInterval;						// WOW inband timer interval					
#endif // WOWLAN_SUPPORT //     
    
} RACFG_OBJECT;

#ifdef CONFIG_CONCURRENT_INIC_SUPPORT

#define CONCURRENT_CARD_NUM 2

typedef struct __CONCURRENT_OBJECT
{
u8 			CardCount;
char 		Mac[CONCURRENT_CARD_NUM][64];
FWHandle	Profile[CONCURRENT_CARD_NUM];
FWHandle	ExtEeprom[CONCURRENT_CARD_NUM];
int			extraProfileOffset2;
}CONCURRENT_OBJECT;

#endif // CONFIG_CONCURRENT_INIC_SUPPORT //

#if (CONFIG_INF_TYPE==INIC_INF_TYPE_MII)
typedef struct inic_private
{
	struct net_device   *dev;
	struct net_device   *master;
//	spinlock_t          lock;
	u32                 msg_enable;
	struct net_device_stats     net_stats;

	RACFG_OBJECT	    RaCfgObj;
	struct iw_statistics iw_stats;
#ifdef NM_SUPPORT
    int                 (*hardware_reset)(int op);
#endif

} iNIC_PRIVATE;
#endif


struct vlan_tic
{
#ifdef __BIG_ENDIAN
	u16     priority : 3;
	u16     cfi      : 1;
	u16     vid      : 12;
#else
	u16     vid      : 12;
	u16     cfi      : 1;
	u16     priority : 3;
#endif
} __attribute__((packed));

#define GET_SKB_HARD_HEADER_LEN(_x)  (_x->cb[0])
#define SET_SKB_HARD_HEADER_LEN(_x, _y)  (_x->cb[0] = _y)


typedef struct pid * THREAD_PID;
#define THREAD_PID_INIT_VALUE NULL
#define	GET_PID(_v)	find_get_pid(_v)
#define GET_PID_NUMBER(_v) pid_nr((_v))
#define CHECK_PID_LEGALITY(_pid) if (pid_nr((_pid)) >= 0)
#define KILL_THREAD_PID(_A, _B, _C) kill_pid((_A), (_B), (_C))

#ifdef CONFIG_CONCURRENT_INIC_SUPPORT
extern CONCURRENT_OBJECT ConcurrentObj;
#endif // CONFIG_CONCURRENT_INIC_SUPPORT //

#if (CONFIG_INF_TYPE == INIC_INF_TYPE_MII)
extern iNIC_PRIVATE *gAdapter[CONCURRENT_CARD_NUM];
#endif


#define RLK_STRLEN(target_len, source_len) (target_len)
#define RLK_STRCPY(target_src, source_src) strncpy(target_src, source_src, RLK_STRLEN(sizeof(target_src), sizeof(source_src)))


#endif /* __RLK_INIC_DEF_H__ */

