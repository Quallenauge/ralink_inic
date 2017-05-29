#include "../comm/rlk_inic.h"
#include <linux/gpio.h>

/* These identify the driver base version and may not be removed. */
static char version[] =
		KERN_INFO DRV_NAME ": 802.11n WLAN MII driver v" DRV_VERSION " (" DRV_RELDATE ")\n";

MODULE_AUTHOR("XiKun Wu <xikun_wu@ralinktech.com.tw>");
MODULE_DESCRIPTION("Ralink iNIC 802.11n WLAN MII driver");

static int debug = -1;
 char *mode = "ap";		// ap mode
 char *mac = "";			// default 00:00:00:00:00:00
static int bridge = 1;	// enable built-in bridge
static int csumoff = 0;	// Enable Checksum Offload over IPv4(TCP, UDP)
 char *miimaster = "";	// MII master device name
 int syncmiimac = 1;// Sync mac address with MII, instead of iNIC, if no mac specified
 int max_fw_upload = 5;	// Max Firmware upload attempt
static int reset_gpio = -1;	// Reset GPIO struct

#ifdef CONFIG_CONCURRENT_INIC_SUPPORT
	char *mac2 = "";			// default 00:00:00:00:00:00
#endif // CONFIG_CONCURRENT_INIC_SUPPORT //

#ifdef DBG
	char *root = "";
#endif

#ifdef DBG
module_param (root, charp, 0);
MODULE_PARM_DESC(root, DRV_NAME ": firmware and profile path offset");
#endif
module_param(debug, int, 0);
module_param(bridge, int, 0);
module_param(csumoff, int, 0);
module_param(mode, charp, 0);
module_param(mac, charp, 0);

#ifdef CONFIG_CONCURRENT_INIC_SUPPORT
module_param(mac2, charp, 0);
#endif // CONFIG_CONCURRENT_INIC_SUPPORT //

module_param(max_fw_upload, int, 0);
module_param(miimaster, charp, 0);
module_param(syncmiimac, int, 0);
module_param(reset_gpio, int, 0644);

MODULE_PARM_DESC(debug, DRV_NAME ": bitmapped message enable number");
MODULE_PARM_DESC(mode, DRV_NAME ": iNIC operation mode: AP(default) or STA");
MODULE_PARM_DESC(mac, DRV_NAME ": iNIC mac addr");

#ifdef CONFIG_CONCURRENT_INIC_SUPPORT
MODULE_PARM_DESC(mac2, DRV_NAME ": iNIC concurrent mac addr");
#endif // CONFIG_CONCURRENT_INIC_SUPPORT //

MODULE_PARM_DESC(max_fw_upload, DRV_NAME ": Max Firmware upload attempt");
MODULE_PARM_DESC(bridge, DRV_NAME ": on/off iNIC built-in bridge engine");
MODULE_PARM_DESC(csumoff,
		DRV_NAME ": on/off checksum offloading over IPv4 <TCP, UDP>");
MODULE_PARM_DESC(miimaster, DRV_NAME ": MII master device name");

#ifdef CONFIG_CONCURRENT_INIC_SUPPORT
MODULE_PARM_DESC(syncmiimac,
		DRV_NAME ": sync MAC with MII master, This value is forced to 0 in dual concurrent mode");
#else
MODULE_PARM_DESC (syncmiimac, DRV_NAME ": sync MAC with MII master");
#endif // CONFIG_CONCURRENT_INIC_SUPPORT //

static int mii_open(struct net_device *dev);
static int mii_close(struct net_device *dev);
static int mii_send_packet(struct sk_buff *skb, struct net_device *dev);
static struct net_device_stats *mii_get_stats(struct net_device *netdev);

/*
 * Indicates, that the profile and firmware was
 * uploaded to the iNIC. Then the iNIC has
 * booted successfully.
 */
extern int iNIC_initialized;
/*
 * Indicates, that the packet type was temporary
 */
int packet_temporary_disabled=0;

static int mii_hardware_reset(int op) {
	DBGPRINT("-----> %s\n", __FUNCTION__);
	gpio_set_value(reset_gpio, op);
	return op;
}

extern int SendFragmentPackets(iNIC_PRIVATE *pAd, unsigned short cmd_type,
		unsigned short cmd_id, unsigned short cmd_seq, unsigned short dev_type,
		unsigned short dev_id, unsigned char *msg, int total);

iNIC_PRIVATE *gAdapter[CONCURRENT_CARD_NUM];

static const struct net_device_ops Netdev_Ops[CONCURRENT_CARD_NUM] =
{
		{
				.ndo_open = mii_open,
				.ndo_stop = mii_close,
				.ndo_start_xmit = mii_send_packet,
				.ndo_do_ioctl = rlk_inic_ioctl,
				.ndo_get_stats = mii_get_stats
		},
		{
				.ndo_open = mii_open,
				.ndo_stop = mii_close,
				.ndo_start_xmit = mii_send_packet,
				.ndo_do_ioctl = rlk_inic_ioctl,
				.ndo_get_stats = mii_get_stats
		}
};
#if defined EXPERIMENTAL_BRIDGE
#if defined(CONFIG_BRIDGE) || defined (CONFIG_BRIDGE_MODULE)
static br_should_route_hook_t *org_br_should_route_hook;

static int my_br_handle_frame(struct sk_buff *skb){
	iNIC_PRIVATE *pAd = gAdapter[0];
	br_should_route_hook_t *rhook;
#ifdef CONFIG_CONCURRENT_INIC_SUPPORT
	DispatchAdapter(&pAd, skb);
#endif // CONFIG_CONCURRENT_INIC_SUPPORT //

	ASSERT(skb);

	if ((skb->protocol == 0xFFFF) && pAd) {

		if (racfg_frame_handle(pAd, skb)) {
			return RX_HANDLER_CONSUMED;
		}
		return RX_HANDLER_PASS;
	} else {
		// check vlan and in-band, then remove vlan tag
		if ((((htons(skb->protocol) >> 8) & 0xff) == 0x81)
				&& (*((u16 *) (&skb->data[2])) == htons(0xFFFF)) && pAd) {
			// Remove VLAN Tag 4 bytes
			skb->data -= ETH_HLEN;
			memmove(&skb->data[4], &skb->data[0], 12);

			// skip ETH_HLEN + VLAN Tag 4 bytes
			skb->data += (4 + ETH_HLEN);
			skb->len -= (4 + ETH_HLEN);

			// reset protocol to in-band frame
			skb->protocol = htons(0xFFFF);

#ifdef CONFIG_CONCURRENT_INIC_SUPPORT

			DispatchAdapter(&pAd, skb);
			if (pAd == NULL) {
				printk("Warnning: DEFINE_BR_HANDLE_FRAME pAd is NULL\n");
				return RX_HANDLER_PASS;
			}

#endif // CONFIG_CONCURRENT_INIC_SUPPORT //

			if (racfg_frame_handle(pAd, skb)) {
				return RX_HANDLER_CONSUMED;
			}
			return RX_HANDLER_PASS;
		}
		rhook = rcu_dereference(org_br_should_route_hook);
		if (rhook) {
			return (*rhook)(skb);
		}
	}
	return RX_HANDLER_PASS;
}
#endif
#endif

/*
 *      Receive an in-band command from the device layer.
 */

static int in_band_rcv(struct sk_buff *skb, struct net_device *dev, \
                    struct packet_type *pt, struct net_device *orig_dev) {
	iNIC_PRIVATE *pAd = gAdapter[0];
	struct net_device_stats *stats = &pAd->net_stats;
#ifdef CONFIG_CONCURRENT_INIC_SUPPORT
	DispatchAdapter(&pAd, skb);
#endif // CONFIG_CONCURRENT_INIC_SUPPORT //
//	DBGPRINT("skb: skb->pkt_type %x, pAd %x, skb->dev %x, pAd->dev %x\n", skb->pkt_type, pAd, skb->dev, pAd->master);

	if (skb->pkt_type == PACKET_OUTGOING || pAd == NULL
			|| pAd->master != skb->dev) {
		dev_kfree_skb(skb);
		return 1;
	}

	if (racfg_frame_handle(pAd, skb)) {
		/* no need to call dev_kfree_skb(), racfg_frame_handle() already does it itself */
		return 0;
	}

	skb = skb_share_check(skb, GFP_ATOMIC);
	if (skb == NULL)
		return -1;

	rcu_read_lock();
	skb->dev = pAd->dev;
	stats->rx_packets++;
	stats->rx_bytes += skb->len;
	netif_rx(skb);
	rcu_read_unlock();

	return 0;
}

static struct packet_type in_band_packet_type = { .type = __constant_htons(
		ETH_P_ALL), .func = in_band_rcv, };

/* netdev event handler */
static int ralink_netdev_event(struct notifier_block *this, unsigned long event,
							 void *ptr)
{
	struct net_device *netdev = netdev_notifier_info_to_dev(ptr);
	struct net_device *master;

	//printk("Got netdev (dev: %s) event: 0%02lx\n", netdev->name, event);
	if (iNIC_initialized && !strcmp(netdev->name,miimaster)){
		if (event == NETDEV_REGISTER){
			master = DEV_GET_BY_NAME(miimaster);
			if (master == NULL) {
				printk("Master device (%s) isn't accessible. Ignoring event.\n",miimaster);
				goto done;
			}else{
				dev_put(master);
			}
			printk("Master device (%s) has turned on. Setup internal references to new master at 0x%p\n",miimaster, master);
			// Set new master device
			SET_NETDEV_DEV(gAdapter[0]->dev, DEV_SHORTCUT(master));
			gAdapter[0]->master = master;
#ifdef CONFIG_CONCURRENT_INIC_SUPPORT
			SET_NETDEV_DEV(gAdapter[1]->dev, DEV_SHORTCUT(master));
			gAdapter[1]->master = master;
#endif
		}else if (event == NETDEV_GOING_DOWN){
			netdev_info(gAdapter[0]->dev, "Stopping heart beat timer and closing device.\n");
			RaCfgDelHeartBeatTimer(gAdapter[0]);
			dev_close(gAdapter[0]->dev);
#ifdef CONFIG_CONCURRENT_INIC_SUPPORT
			netdev_info(gAdapter[1]->dev, "Stopping heart beat timer and closing device.\n");
			RaCfgDelHeartBeatTimer(gAdapter[1]);
			dev_close(gAdapter[1]->dev);
#endif
			printk("Remove inband packet type from master device (%s)\n", miimaster);
			dev_remove_pack(&in_band_packet_type);
		}else if (event == NETDEV_UP){
			printk("Register inband packet type to master device (%s)\n", miimaster);
			in_band_packet_type.dev = gAdapter[0]->master; /* hook only on mii master device */
			dev_add_pack(&in_band_packet_type);

			dev_open(gAdapter[0]->dev);
			RaCfgAddHeartBeatTimer(gAdapter[0]);
#ifdef CONFIG_CONCURRENT_INIC_SUPPORT
			dev_open(gAdapter[1]->dev);
			RaCfgAddHeartBeatTimer(gAdapter[1]);
#endif
		}
	}
done:
	return NOTIFY_DONE;
}

static struct notifier_block ralink_netdev_notifier = {
	.notifier_call = ralink_netdev_event
};

//#define DEBUG_HOOK 1
#ifdef DEBUG_HOOK
static int sniff_arp(struct sk_buff *skb, struct net_device *dev, \
                    struct packet_type *pt, struct net_device *orig_dev) {
	if (skb->pkt_type == PACKET_OUTGOING) {
		dev_kfree_skb(skb);
		return NET_RX_SUCCESS;
	}

	if (gAdapter[0] && skb->protocol != ETH_P_RACFG) {
		hex_dump("sniff", skb->data, 32);
	}
	//dev_kfree_skb(skb);
	return NET_RX_SUCCESS;
}
static struct packet_type arp_packet_type = { .type = __constant_htons(0x0806),
		.func = sniff_arp, };
#endif

void racfg_inband_hook_init(iNIC_PRIVATE *pAd) {
#if defined EXPERIMENTAL_BRIDGE
#if defined(CONFIG_BRIDGE) || defined (CONFIG_BRIDGE_MODULE)
	org_br_should_route_hook = rcu_dereference(br_should_route_hook);
	if (org_br_should_route_hook) {
		printk("Org bridge hook = %p\n", br_should_route_hook);
		RCU_INIT_POINTER(br_should_route_hook,
				(br_should_route_hook_t *)my_br_handle_frame);
		printk("Change bridge hook = %p\n", br_should_route_hook);
	} else {
		printk(
				"Warning! Bridge module not init yet. Please modprobe bridge at first if you want to use bridge.\n");
	}
#endif
#endif
	in_band_packet_type.dev = pAd->master; /* hook only on mii master device */
	dev_add_pack(&in_band_packet_type);
#ifdef DEBUG_HOOK
	dev_add_pack(&arp_packet_type);
#endif
}

void racfg_inband_hook_cleanup(void) {
#ifdef CONFIG_CONCURRENT_INIC_SUPPORT
	if (ConcurrentObj.CardCount > 0)
		return;
#endif // CONFIG_CONCURRENT_INIC_SUPPORT //

	dev_remove_pack(&in_band_packet_type);
#ifdef DEBUG_HOOK
	dev_remove_pack(&arp_packet_type);
#endif
#if defined EXPERIMENTAL_BRIDGE
#if defined(CONFIG_BRIDGE) || defined (CONFIG_BRIDGE_MODULE)
	if (rcu_dereference(org_br_should_route_hook)) {
		RCU_INIT_POINTER(br_should_route_hook, org_br_should_route_hook);
		printk("Restore bridge hook = %p\n", br_should_route_hook);
	}
#endif
#endif
}

static int mii_open(struct net_device *dev) {
	iNIC_PRIVATE *pAd = netdev_priv(dev);

	if (!pAd->master) {
		printk("ERROR! MII master unregistered, please register it at first\n");
		return -ENODEV;
	} else if (!NETIF_IS_UP(pAd->master)) {
		printk("ERROR! MII master not open, please open (%s) at first\n",
				pAd->master->name);
		return -ENODEV;
	}

	printk("iNIC Open %s\n", dev->name);
	if(pAd->RaCfgObj.InterfaceNumber == 0 && iNIC_initialized == 0){
		mii_hardware_reset(0);
		msleep(1000);
		mii_hardware_reset(1);
	}// to start racfg_frame_handle()
	RaCfgInterfaceOpen(pAd);

	netif_carrier_on(dev);
	netif_start_queue(dev);

//#ifndef NM_SUPPORT
	RaCfgSetUp(pAd, dev);
//#endif

#ifdef CONFIG_CONCURRENT_INIC_SUPPORT
	ConcurrentObj.CardCount++;
#endif // CONFIG_CONCURRENT_INIC_SUPPORT //
	return 0;
}

static int mii_close(struct net_device *dev) {
	iNIC_PRIVATE *pAd = netdev_priv(dev);
	if (netif_msg_ifdown(pAd))
		DBGPRINT("%s: disabling interface\n", dev->name);

#ifdef WOWLAN_SUPPORT 	
	if(pAd->RaCfgObj.bWoWlanUp == TRUE && pAd->RaCfgObj.pm_wow_state == WOW_CPU_DOWN )
	{
		DBGPRINT("CPU is in Sleep mode & WoW enabled, mii_close do nothing ....\n");
		return 0;
	}
#endif // WOWLAN_SUPPORT //

	SetRadioOn(pAd, 0);

	netif_stop_queue(dev);
	netif_carrier_off(dev);

	pAd->RaCfgObj.bExtEEPROM = FALSE;
	DBGPRINT("iNIC Closed\n");
//#ifndef NM_SUPPORT
	RaCfgStateReset(pAd);
//#endif
	RaCfgInterfaceClose(pAd);

#ifdef CONFIG_CONCURRENT_INIC_SUPPORT
	ConcurrentObj.CardCount--;
#endif // CONFIG_CONCURRENT_INIC_SUPPORT //
	return 0;
}

int rlk_inic_mii_xmit(struct sk_buff *skb, struct net_device *dev) {
	iNIC_PRIVATE *pAd = netdev_priv(dev);

	if (skb->data[12] == 0xff && skb->data[13] == 0xff) {
		if (syncmiimac == 1) {
			if (pAd->RaCfgObj.bLocalAdminAddr && pAd->RaCfgObj.bGetMac)
				skb->data[0] = (skb->data[0] & 0xFE) | 0x82; // turn on local admin address bit
			else
				skb->data[6] |= 0x01;
		}
	}
#ifdef INBAND_DEBUG
	if (pAd->RaCfgObj.bGetMac)
	{
		hex_dump("send", skb->data, 64);
	}
#endif

	if (!pAd->master) {
		printk("No MII master device, name:%s\n", dev->name);
		dev_kfree_skb(skb);
		return NETDEV_TX_OK;
	}
	skb->dev = pAd->master;

	skb_reset_network_header(skb); // keep away from buggy warning in /net/core/dev.c

#if 0
	if (skb->data[12] != 0xff || skb->data[13] != 0xff)
	hex_dump("dev_queue_xmit", skb->data, 32);
#endif 
	return dev_queue_xmit(skb);
}

static int mii_send_packet(struct sk_buff *skb, struct net_device *dev) {
	int dev_id = 0;
	iNIC_PRIVATE *pAd = netdev_priv(dev);
	struct net_device_stats *stats = &pAd->net_stats;

	ASSERT(skb);

	if (RaCfgDropRemoteInBandFrame(skb)) {
		return 0;
	}
	if (pAd->RaCfgObj.bBridge == 1) {
		// MII_SLAVE_STANDALONE => enable for isolated VLAN
#ifdef MII_SLAVE_STANDALONE
		stats->tx_packets++;
		stats->tx_bytes += skb->len;
		return rlk_inic_start_xmit(skb, dev);
#else

		// built in bridge enabled,all packets sent by main device,
		// slave should drop all the packets(may flooded by bridge) 
		// 1. not WPA supplicant => drop
		if (!pAd->RaCfgObj.bWpaSupplicantUp)
		{
			dev_kfree_skb(skb);
			return NETDEV_TX_OK;
		}
		// 2. broadcast => drop
		if (skb->data[6] & 0x01)
		{
			dev_kfree_skb(skb);
			return NETDEV_TX_OK;
		}

		// 3. non-EAPOL packet => drop
		if (skb->protocol != ntohs(0x888e))
		{
			dev_kfree_skb(skb);
			return NETDEV_TX_OK;
		}

		//for mii non-sync, tunnel the eapol frame by inband command.
		if(syncmiimac==0) {
			SendFragmentPackets(pAd, RACFG_CMD_TYPE_TUNNEL, 0, 0, 0, 0, &skb->data[0], skb->len);
			dev_kfree_skb(skb);
			return NETDEV_TX_OK;
		}

		//printk("unicast (protocol:0x%04x) send to mii slave %s->xmit()\n", skb->protocol, dev->name);
		stats->tx_packets++;
		stats->tx_bytes += skb->len;
		return rlk_inic_start_xmit(skb, dev);
#endif

	} else {
		skb = Insert_Vlan_Tag(pAd, skb, dev_id, SOURCE_MBSS);
		if (skb) {
			stats->tx_packets++;
			stats->tx_bytes += skb->len;
			return rlk_inic_start_xmit(skb, dev);
		} else {
			return NETDEV_TX_OK;
		}
	}
} /* End of rlk_inic_send_packet */

static struct net_device_stats *mii_get_stats(struct net_device *netdev) {
	iNIC_PRIVATE *pAd = netdev_priv(netdev);

	return &pAd->net_stats;
}

static int __init rlk_inic_init(void) {
	int i, rc = 0;
	char name[8];
	iNIC_PRIVATE *pAd;
	struct net_device *dev, *device, *master;

#ifdef CONFIG_CONCURRENT_INIC_SUPPORT
	struct net_device *dev2 = NULL;
	iNIC_PRIVATE *pAd2 = NULL;

	if (syncmiimac != 0) {
		syncmiimac = 0;
		printk("Warnning!! syncmiimac is foreced to 0 in dual concurrent mode.\n");
	}
#endif // CONFIG_CONCURRENT_INIC_SUPPORT //

	master = DEV_GET_BY_NAME(miimaster);
	if (master == NULL) {
		printk("ERROR! Please assign miimaster=[MII master device name] "
				"at module insertion.\n");
		return -ENODEV;
	}
	dev_put(master);

	if(-1 == reset_gpio){
		printk("ERROR! Please assign reset_gpio=[reset GPIO num] "
				"at module insertion.\n");
		return -ENODEV;
	} else if (!gpio_is_valid(reset_gpio)) {
		printk(KERN_ERR "ERROR! gpio number invalid\n");
		return -EINVAL;
	}

	rc = gpio_request(reset_gpio, "reset");

	if (rc) {
		printk(KERN_ERR "ERROR! gpio_request failed for %u, err=%d\n", reset_gpio,	rc);
		return rc;
	}
	rc = gpio_direction_output(reset_gpio, 1);
	if (rc) {
			printk(KERN_ERR "ERROR! set reset gpio failed for %u, err=%d\n", reset_gpio, rc);
			return rc;
		}

#ifdef MODULE
	printk("%s", version);
#endif

	dev = alloc_etherdev(sizeof(iNIC_PRIVATE));
	if (!dev) {
		printk("Can't alloc net device!\n");
		return -ENOMEM;
	}
	SET_MODULE_OWNER(dev);

	SET_NETDEV_DEV(dev, DEV_SHORTCUT(master));

	pAd = netdev_priv(dev);

	gAdapter[0] = pAd;

	pAd->dev = dev;
	pAd->master = master;

	DBGPRINT("%s: Master at 0x%lx, "
			"%02x:%02x:%02x:%02x:%02x:%02x\n", pAd->master->name, pAd->master->base_addr,
			pAd->master->dev_addr[0], pAd->master->dev_addr[1], pAd->master->dev_addr[2],
			pAd->master->dev_addr[3], pAd->master->dev_addr[4], pAd->master->dev_addr[5]);

#ifdef NM_SUPPORT
	pAd->hardware_reset = mii_hardware_reset;
#endif
	dev->netdev_ops = &Netdev_Ops[0];

	for (i = 0; i < 32; i++) {
#ifdef CONFIG_CONCURRENT_INIC_SUPPORT
		snprintf(dev->name, sizeof(dev->name), "%s00%d", INIC_INFNAME, i);
#else
		snprintf(name, sizeof(name), "%s%d", INIC_INFNAME, i);
#endif // CONFIG_CONCURRENT_INIC_SUPPORT //

		device = DEV_GET_BY_NAME(name);
		if (device == NULL)
			break;
		dev_put(device);
	}
	if (i == 32) {
		printk(KERN_ERR "No available dev name\n");
		rc = -ENODEV;
		goto err_out_free;
	}

#ifdef CONFIG_CONCURRENT_INIC_SUPPORT
	snprintf(dev->name, sizeof(dev->name), "%s00%d", INIC_INFNAME, i);
#else
	snprintf(dev->name, sizeof(dev->name), "%s%d", INIC_INFNAME, i);
#endif // CONFIG_CONCURRENT_INIC_SUPPORT //

	// Be sure to init racfg before register_netdev,Otherwise 
	// network manager cannot identify ra0 as wireless device

	memset(&pAd->RaCfgObj, 0, sizeof(RACFG_OBJECT));

#ifdef CONFIG_CONCURRENT_INIC_SUPPORT
	pAd->RaCfgObj.InterfaceNumber = 0;

	ConcurrentCardInfoRead();
	/* 
	 * The first priority of MAC address is read from file. 
	 * The second priority of MAC address is read form command line.
	 */
	if (strlen(ConcurrentObj.Mac[0]))
		mac = ConcurrentObj.Mac[0];

	if (strlen(ConcurrentObj.Mac[1]))
		mac2 = ConcurrentObj.Mac[1];
#endif // CONFIG_CONCURRENT_INIC_SUPPORT //

	RaCfgInit(pAd, dev, mac, mode, bridge, csumoff);

	rc = register_netdev(dev);

	if(rc)
		goto err_out_free;
	DBGPRINT("%s: Ralink iNIC at 0x%lx, "
			"%02x:%02x:%02x:%02x:%02x:%02x\n", dev->name, dev->base_addr,
			dev->dev_addr[0], dev->dev_addr[1], dev->dev_addr[2],
			dev->dev_addr[3], dev->dev_addr[4], dev->dev_addr[5]);

	dev->priv_flags = INT_MAIN;

#ifndef MII_SLAVE_STANDALONE
	if (bridge == 0)
	{
		printk("WARNING! Built-in bridge shouldn't be disabled "
				"in iNIC MII driver, [bridge] parameter ignored...\n");
		bridge = 1;
	}
#endif

#ifdef CONFIG_CONCURRENT_INIC_SUPPORT
	dev2 = alloc_etherdev(sizeof(iNIC_PRIVATE));
	if (!dev2) {
		printk("Can't alloc net device!\n");
		rc = -ENOMEM;
		goto err_out_free;
	}
	SET_MODULE_OWNER(dev2);

	SET_NETDEV_DEV(dev2, DEV_SHORTCUT(master));

	pAd2 = netdev_priv(dev2);

	gAdapter[1] = pAd2;

	pAd2->dev = dev2;
	pAd2->master = master;
#ifdef NM_SUPPORT
	pAd2->hardware_reset = NULL;
#endif
	dev2->netdev_ops = &Netdev_Ops[1];
	for (i = 0; i < 32; i++) {
		snprintf(name, sizeof(name), "%s01%d", INIC_INFNAME, i);

		device = DEV_GET_BY_NAME(name);
		if (device == NULL)
			break;
		dev_put(device);
	}
	if (i == 32) {
		printk(KERN_ERR "No available dev name\n");
		rc = -ENODEV;
		goto err_out_free;
	}

	snprintf(dev2->name, sizeof(dev2->name), "%s01%d", INIC_INFNAME, i);

	// Be sure to init racfg before register_netdev,Otherwise 
	// network manager cannot identify ra0 as wireless device

	memset(&pAd2->RaCfgObj, 0, sizeof(RACFG_OBJECT));
	pAd2->RaCfgObj.InterfaceNumber = 1;

	RaCfgInit(pAd2, dev2, mac2, mode, bridge, csumoff);

	rc = register_netdev(dev2);
	if(rc)
		goto err_out_free;

	DBGPRINT("%s: Ralink iNIC at 0x%lx, "
			"%02x:%02x:%02x:%02x:%02x:%02x\n", dev2->name, dev2->base_addr,
			dev2->dev_addr[0], dev2->dev_addr[1], dev2->dev_addr[2],
			dev2->dev_addr[3], dev2->dev_addr[4], dev2->dev_addr[5]);

	dev2->priv_flags = INT_MAIN;

#endif // CONFIG_CONCURRENT_INIC_SUPPORT //

	register_netdevice_notifier(&ralink_netdev_notifier);
	return rc;

	err_out_free:
	free_netdev(dev);
	gpio_free(reset_gpio);
	return rc;
}
module_init(rlk_inic_init);

static void __exit rlk_inic_exit(void) {
	iNIC_PRIVATE *pAd;
	struct net_device *dev;

#ifdef CONFIG_CONCURRENT_INIC_SUPPORT

	pAd = gAdapter[1];
	dev = pAd->dev;

	if (!dev)
		BUG();
	printk("unregister netdev %s\n", dev->name);

	if (pAd->RaCfgObj.opmode == 0) {
#if IW_HANDLER_VERSION < 7
		dev->get_wireless_stats = NULL;
#endif
		dev->wireless_handlers = NULL;
	}

	unregister_netdev(dev);
#ifdef NM_SUPPORT 
	RaCfgStateReset(pAd);
#endif

	RaCfgExit(pAd);
	free_netdev(dev);
#endif // CONFIG_CONCURRENT_INIC_SUPPORT //

	pAd = gAdapter[0];
	dev = pAd->dev;

	if (!dev)
		BUG();
	printk("unregister netdev %s\n", dev->name);

	if (pAd->RaCfgObj.opmode == 0) {
#if IW_HANDLER_VERSION < 7
		dev->get_wireless_stats = NULL;
#endif
		dev->wireless_handlers = NULL;
	}

	unregister_netdev(dev);
#ifdef NM_SUPPORT 
	RaCfgStateReset(pAd);
#endif

	RaCfgExit(pAd);
	unregister_netdevice_notifier(&ralink_netdev_notifier);
	free_netdev(dev);
	gpio_free(reset_gpio);
}

module_exit(rlk_inic_exit);

MODULE_LICENSE("GPL");
