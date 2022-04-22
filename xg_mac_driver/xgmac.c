#include <linux/module.h>
#include <linux/device.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/ethtool.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/phy.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>

#define DRIVER_NAME "xgmac"


#ifdef __BIG_ENDIAN
#define xgmac_readl		ioread32be
#define xgmac_writel	iowrite32be
#else
#define xgmac_readl		ioread32
#define xgmac_writel	iowrite32
#endif

struct net_local {

	struct net_device *net_dev;
	void __iomem *base_addr;

	spinlock_t reset_lock;
	struct sk_buff *deferred_skb;

	struct phy_device *phy_dev;
	struct device_node *phy_node;

	struct mii_bus *mii_bus;
};

#define XGMAC_DMA_MM2S_DMACR_OFFSET 	0x00000000
#define XGMAC_DMA_MM2S_DMASR_OFFSET 	0x00000004
#define XGMAC_DMA_MM2S_SA_OFFSET 		0x00000018
#define XGMAC_DMA_MM2S_SA_MSB_OFFSET	0x0000001c
#define XGMAC_DMA_MM2S_LENGTH_OFFSET	0x00000028
#define XGMAC_DMA_S2MM_DMACR_OFFSET 	0x00000030
#define XGMAC_DMA_S2MM_DMASR_OFFSET 	0x00000034
#define XGMAC_DMA_S2MM_DA_OFFSET 		0x00000048
#define XGMAC_DMA_S2MM_DA_MSB_OFFSET	0x0000004c
#define XGMAC_DMA_S2MM_LENGTH_OFFSET	0x00000058

#define XGMAC_DMA_DMACR_IOC_IRQEN_MASK	(1u << 12)
#define XGMAC_DMA_DMACR_DLY_IRQEN_MASK	(1u << 13)
#define XGMAC_DMA_DMACR_ERR_IRQEN_MASK	(1u << 14)

static void xgmac_enable_interrupts(struct net_local *local)
{
	u32 reg_data;

	reg_data = xgmac_readl(local->base_addr + XGMAC_DMA_MM2S_DMACR_OFFSET);
	xgmac_writel(reg_data | XGMAC_DMA_DMACR_IOC_IRQEN_MASK, local->base_addr + XGMAC_DMA_MM2S_DMACR_OFFSET);
	reg_data = xgmac_readl(local->base_addr + XGMAC_DMA_S2MM_DMACR_OFFSET);
	xgmac_writel(reg_data | XGMAC_DMA_DMACR_IOC_IRQEN_MASK, local->base_addr + XGMAC_DMA_S2MM_DMACR_OFFSET);
}

static void xgmac_disable_interrupts(struct net_local *local)
{
	u32 reg_data;

	reg_data = xgmac_readl(local->base_addr + XGMAC_DMA_MM2S_DMACR_OFFSET);
	xgmac_writel(reg_data & ~XGMAC_DMA_DMACR_IOC_IRQEN_MASK, local->base_addr + XGMAC_DMA_MM2S_DMACR_OFFSET);
	reg_data = xgmac_readl(local->base_addr + XGMAC_DMA_S2MM_DMACR_OFFSET);
	xgmac_writel(reg_data & ~XGMAC_DMA_DMACR_IOC_IRQEN_MASK, local->base_addr + XGMAC_DMA_S2MM_DMACR_OFFSET);
}


static int xgmac_open(struct net_device* dev)
{
    struct net_local* local = netdev_priv(dev);
    int rc = 0;

    xgmac_disable_interrupts(local);
    xgmac_enable_interrupts(local);

	return rc;
}

static int xgmac_close(struct net_device* dev)
{
	struct net_local* local = netdev_priv(dev);

	netif_stop_queue(dev);
	xgmac_disable_interrupts(local);
	free_irq(dev->irq, dev);

	return 0;
}

static netdev_tx_t xgmac_send(struct sk_buff* skb, struct net_device* dev)
{
	struct net_local* local = netdev_priv(dev);
	struct sk_buff* new_skb;

	return NETDEV_TX_OK;
}


static const struct net_device_ops xgmac_netdev_ops;

static int xgmac_probe(struct platform_device* pdev)
{
	struct net_local* local = NULL;
	struct net_device* net_dev = NULL;
	struct device* dev = &pdev->dev;
	struct resource* res = NULL;
	int rc = 0;

	net_dev = alloc_etherdev(sizeof(struct net_local));
	if( !net_dev )
		return -ENOMEM;
	
	dev_set_drvdata(dev, net_dev);
	SET_NETDEV_DEV(net_dev, dev);

	local = netdev_priv(net_dev);
	local->net_dev = net_dev;

	rc = platform_get_irq(pdev, 0);
	if( rc < 0 )
		goto error;
	
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	local->base_addr = devm_ioremap_resource(dev, res);
	if( IS_ERR(local->base_addr) ) {
		rc = PTR_ERR(local->base_addr);
		goto error;
	}

	net_dev->mem_start = res->start;
	net_dev->mem_end = res->end;

	eth_hw_addr_random(net_dev);

	net_dev->netdev_ops = &xgmac_netdev_ops;
	net_dev->flags &= ~IFF_MULTICAST;

	rc = register_netdev(net_dev);
	if( rc ) {
		dev_err(dev, "Failed to register xgmac network device - %d\n", rc);
		goto error;
	}

	dev_info(dev, "xg_mac driver at 0x%08lx (%p - %p), irq=%d\n", (unsigned long)net_dev->mem_start, local->base_addr, net_dev->irq);

	return 0;
error:
	return rc;
}

static xgmac_remove(struct platform_device* pdev)
{
	struct net_device* net_dev = platform_get_drvdata(pdev);
	struct net_local* local = netdev_priv(net_dev);

	unregister_netdev(net_dev);
	free_netdev(net_dev);

	return 0;
}

static const struct net_device_ops xgmac_netdev_ops = {
	.ndo_open		= xgmac_open,
	.ndo_stop		= xgmac_close,
	.ndo_start_xmit	= xgmac_send,
};

static const struct of_device_id xgmac_of_match[] = {
	{ .compatible = "fixstars,xg_mac", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, xgmac_of_match);

static struct platform_driver xgmac_of_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = xgmac_of_match,
	},
	.probe	= xgmac_probe,
	.remove	= xgmac_remove,
};

module_platform_driver(xgmac_of_driver);

MODULE_DESCRIPTION("Fixstars 10GMAC Ethernet Driver");
MODULE_LICENSE("GPL");