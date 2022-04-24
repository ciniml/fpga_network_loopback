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

	struct net_device *netdev;
	struct platform_device* pdev;

	void __iomem *base_addr;

	spinlock_t tx_lock;

	struct sk_buff *tx_skb;
	phys_addr_t tx_skb_pa;
	struct sk_buff *next_tx_skb;

	struct sk_buff *rx_skb;
	phys_addr_t rx_skb_pa;
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

#define XGMAC_DMA_DMACR_RS_MASK			(1u << 0)
#define XGMAC_DMA_DMACR_RESET_MASK		(1u << 2)
#define XGMAC_DMA_DMACR_IOC_IRQEN_MASK	(1u << 12)
#define XGMAC_DMA_DMACR_DLY_IRQEN_MASK	(1u << 13)
#define XGMAC_DMA_DMACR_ERR_IRQEN_MASK	(1u << 14)

#define XGMAC_DMA_DMASR_HALTED_MASK		(1u << 0)
#define XGMAC_DMA_DMASR_IDLE_MASK		(1u << 1)
#define XGMAC_DMA_DMASR_IOC_IRQ_MASK	(1u << 12)
#define XGMAC_DMA_DMASR_DLY_IRQ_MASK	(1u << 13)
#define XGMAC_DMA_DMASR_ERR_IRQ_MASK	(1u << 14)


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


static int xgmac_send_skb(struct net_device* netdev, struct sk_buff* skb)
{
	struct net_local* local = netdev_priv(netdev);
	u32 reg_data;
	phys_addr_t pa;
	size_t size;

	dev_info(&netdev->dev, "send_skb netdev=%px, data=%px, len=%lu, tx_skb=%px\n", netdev, skb->data, skb->len, local->tx_skb);

	size = skb->len;
	if (size > ETH_FRAME_LEN)
		size = ETH_FRAME_LEN;

	reg_data = xgmac_readl(local->base_addr + XGMAC_DMA_MM2S_DMASR_OFFSET);
	if( (reg_data & (XGMAC_DMA_DMASR_HALTED_MASK | XGMAC_DMA_DMASR_IDLE_MASK)) == 0 || local->tx_skb != NULL ) {
		/* TX DMA is busy. */
		dev_err(&netdev->dev, "TX DMA is busy dmasr=%08lx tx_skb=%px\n", reg_data, local->tx_skb);
		return -EBUSY;
	}
	
	/* Setup DMA */
	local->tx_skb_pa = dma_map_single(&local->pdev->dev, skb->data, size, DMA_TO_DEVICE);
	if( !local->tx_skb_pa ) {
		/* Failed to map the buffer */
		dev_err(&netdev->dev, "Failed to map skb. addr=%px, len=%u\n", skb->data, size);
		return -ENOMEM;
	}

	reg_data = xgmac_readl(local->base_addr + XGMAC_DMA_MM2S_DMACR_OFFSET);
	xgmac_writel(reg_data | XGMAC_DMA_DMACR_RS_MASK, local->base_addr + XGMAC_DMA_MM2S_DMACR_OFFSET);
	xgmac_writel(local->tx_skb_pa & 0xffffffff, local->base_addr + XGMAC_DMA_MM2S_SA_OFFSET);
	xgmac_writel(local->tx_skb_pa >> 32, local->base_addr + XGMAC_DMA_MM2S_SA_MSB_OFFSET);

	/* Store current SKB */
	local->tx_skb = skb;

	/* Start DMA */
	xgmac_writel(size, local->base_addr + XGMAC_DMA_MM2S_LENGTH_OFFSET);

	return 0;
}

static void xgmac_tx_handler(struct net_device* netdev)
{
	struct net_local* local = netdev_priv(netdev);
	struct device* dev = &netdev->dev;

	dev_info(&netdev->dev, "tx_handler netdev=%px, local=%px, tx_skb=%px\n", netdev, local, local->tx_skb);

	if( local->tx_skb ) {
		/* Release the last tx skb. */
		dma_unmap_single(&local->pdev->dev, local->tx_skb_pa, local->tx_skb->len, DMA_TO_DEVICE);
		dev_consume_skb_irq(local->tx_skb);
		local->tx_skb = NULL;
		local->tx_skb_pa = 0;
	}

	netdev->stats.tx_packets++;	/* Update TX statistics */

	if( !local->next_tx_skb )	/* Check if there is pending skb to send. */
		return;
	
	if( xgmac_send_skb(netdev, local->next_tx_skb) )	/* Send the pending skb*/
		return;

	netdev->stats.tx_bytes += local->next_tx_skb->len;
	/* consume the processed skb. */
	local->next_tx_skb = NULL;

	/* wakeup queue */
	netif_trans_update(netdev);
	netif_wake_queue(netdev);
}

static int xgmac_setup_rx(struct net_device* netdev)
{
	struct device* dev = &netdev->dev;
	struct net_local* local = netdev_priv(netdev);
	struct sk_buff* skb;
	size_t bytes_to_receive;
	u32 reg_data;

	bytes_to_receive = ETH_FRAME_LEN + ETH_FCS_LEN;
	skb = netdev_alloc_skb(netdev, (bytes_to_receive + 3) & ~3 );
	if( !skb ) {
		/* failed to allocate memory */
		netdev->stats.rx_dropped++;
		dev_err(dev, "Failed to allocate receive buffer.\n");
		return -ENOMEM;
	}

	/* Setup DMA */
	local->rx_skb_pa = dma_map_single(&local->pdev->dev, skb->data, skb->len, DMA_FROM_DEVICE);
	if( !local->rx_skb_pa ) {
		/* Failed to map the buffer */
		dev_err(dev, "Failed to map rx skb. addr=%px, len=%u\n", skb->data, skb->len);
		dev_kfree_skb_irq(skb);
		return -ENOMEM;
	}
	
	reg_data = xgmac_readl(local->base_addr + XGMAC_DMA_S2MM_DMACR_OFFSET);
	xgmac_writel(reg_data | XGMAC_DMA_DMACR_RS_MASK, local->base_addr + XGMAC_DMA_S2MM_DMACR_OFFSET);
	xgmac_writel(local->rx_skb_pa & 0xffffffff, local->base_addr + XGMAC_DMA_S2MM_DA_OFFSET);
	xgmac_writel(local->rx_skb_pa >> 32, local->base_addr + XGMAC_DMA_S2MM_DA_MSB_OFFSET);

	/* Store current SKB */
	local->rx_skb = skb;

	/* Start DMA */
	xgmac_writel(bytes_to_receive, local->base_addr + XGMAC_DMA_S2MM_LENGTH_OFFSET);

	return 0;
}

static void xgmac_rx_handler(struct net_device* netdev)
{
	struct device* dev = &netdev->dev;
	struct net_local* local = netdev_priv(netdev);
	
	size_t bytes_received;

	if( local->rx_skb ) {
		/* Receive complete. */
		bytes_received = xgmac_readl(local->base_addr + XGMAC_DMA_S2MM_LENGTH_OFFSET);
		dma_unmap_single(&local->pdev->dev, local->rx_skb_pa, local->rx_skb->len, DMA_FROM_DEVICE);
		local->rx_skb_pa = 0;

		skb_put(local->rx_skb, bytes_received);
		local->rx_skb->protocol = eth_type_trans(local->rx_skb, netdev);
		skb_checksum_none_assert(local->rx_skb);

		netdev->stats.rx_packets++;
		netdev->stats.rx_bytes += bytes_received;
		if( !skb_defer_rx_timestamp(local->rx_skb) )
			netif_rx(local->rx_skb);
		
		dev_info(&netdev->dev, "received %lu bytes\n", bytes_received);

		local->rx_skb = NULL;
	}

	if( xgmac_setup_rx(netdev) ) {
		/* Failed to setup RX DMA */
		dev_err(dev, "Failed to seup RX DMA.\n");
	}
}

static irqreturn_t xgmac_interrupt(int irq, void *dev_id)
{
	struct net_device *netdev = dev_id;
	struct net_local *local = netdev_priv(netdev);
	
	if( xgmac_readl(local->base_addr + XGMAC_DMA_S2MM_DMASR_OFFSET) & XGMAC_DMA_DMASR_IOC_IRQ_MASK ) {
		/* RX DMA has completed. Clear the IOC flag */
		xgmac_writel(XGMAC_DMA_DMASR_IOC_IRQ_MASK, local->base_addr + XGMAC_DMA_S2MM_DMASR_OFFSET);
		xgmac_rx_handler(netdev);
	}

	if( xgmac_readl(local->base_addr + XGMAC_DMA_MM2S_DMASR_OFFSET) & XGMAC_DMA_DMASR_IOC_IRQ_MASK ) {
		/* TX DMA has completed. Clear the IOC flag */
		xgmac_writel(XGMAC_DMA_DMASR_IOC_IRQ_MASK, local->base_addr + XGMAC_DMA_MM2S_DMASR_OFFSET);
		xgmac_tx_handler(netdev);
	}
	
	return IRQ_HANDLED;
}


static int xgmac_open(struct net_device* netdev)
{
	struct device* dev = &netdev->dev;
    struct net_local* local = netdev_priv(netdev);
    int rc = 0;

    xgmac_disable_interrupts(local);

	rc = request_irq(netdev->irq, xgmac_interrupt, 0, netdev->name, netdev);
	if (rc) {
		dev_err(dev, "Could not allocate interrupt %d\n", netdev->irq);
		return rc;
	}

	/* Setup RX DMA */
	if( rc = xgmac_setup_rx(netdev) ) {
		dev_err(dev, "Failed to setup RX DMA\n");
		free_irq(netdev->irq, netdev);
		netdev->irq = 0;
		return rc;
	}

	xgmac_enable_interrupts(local);
	netif_start_queue(netdev);

	return rc;
}

static int xgmac_close(struct net_device* netdev)
{
	struct net_local* local = netdev_priv(netdev);

	netif_stop_queue(netdev);
	xgmac_disable_interrupts(local);
	if( netdev->irq ) {
		free_irq(netdev->irq, netdev);
		netdev->irq = 0;
	}

	return 0;
}


static netdev_tx_t xgmac_send(struct sk_buff* skb, struct net_device* netdev)
{
	struct net_local* local = netdev_priv(netdev);
	unsigned int len;
	unsigned long flags;

	len = skb->len;

	dev_info(&netdev->dev, "xgmac_send netdev=%px, local=%px, skb=%px\n", netdev, local, skb);

	spin_lock_irqsave(&local->tx_lock, flags);
	if (xgmac_send_skb(netdev, skb) != 0) {
		netif_stop_queue(netdev);
		local->next_tx_skb = skb;
		skb_tx_timestamp(skb);
		spin_unlock_irqrestore(&local->tx_lock, flags);
		return NETDEV_TX_OK;
	}
	spin_unlock_irqrestore(&local->tx_lock, flags);

	skb_tx_timestamp(skb);
	netdev->stats.tx_bytes += len;
	/* dev_consume_skb_any(new_skb); */ /* The skb must be consumed after DMA transfer completes. */

	return NETDEV_TX_OK;
}


static const struct net_device_ops xgmac_netdev_ops;

static int xgmac_probe(struct platform_device* pdev)
{
	struct net_local* local = NULL;
	struct net_device* netdev = NULL;
	struct device* dev = &pdev->dev;
	struct resource* res = NULL;
	int rc = 0;

	dev_info(dev, "Probing xg_mac.\n");

	netdev = alloc_etherdev(sizeof(struct net_local));
	if( !netdev )
		return -ENOMEM;
	
	dev_set_drvdata(dev, netdev);
	SET_NETDEV_DEV(netdev, dev);

	local = netdev_priv(netdev);
	dev_info(dev, "local=%px\n", local);
	local->netdev = netdev;
	local->pdev = pdev;
	local->tx_skb = NULL;
	local->next_tx_skb = NULL;
	local->rx_skb = NULL;

	
	/* Configure DMA */
	if( !dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64)) ) {
		dev_info(dev, "Use 64bit DMA\n");
	} else if( !dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32)) ) {
		dev_info(dev, "Use 32bit DMA\n");
	} else {
		dev_err(dev, "No DMA configuration available. \n");
		rc = -1;
		goto error;
	}

	/* IRQ */
	rc = platform_get_irq(pdev, 0);
	if( rc < 0 )
		goto error;
	netdev->irq = rc;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	local->base_addr = devm_ioremap_resource(dev, res);
	if( IS_ERR(local->base_addr) ) {
		rc = PTR_ERR(local->base_addr);
		goto error;
	}

	netdev->mem_start = res->start;
	netdev->mem_end = res->end;

	/* Reset DMA */
	xgmac_writel(XGMAC_DMA_DMACR_RESET_MASK, local->base_addr + XGMAC_DMA_MM2S_DMACR_OFFSET);
	xgmac_writel(XGMAC_DMA_DMACR_RESET_MASK, local->base_addr + XGMAC_DMA_S2MM_DMACR_OFFSET);

	eth_hw_addr_random(netdev);

	netdev->netdev_ops = &xgmac_netdev_ops;
	netdev->flags &= ~IFF_MULTICAST;

	rc = register_netdev(netdev);
	if( rc ) {
		dev_err(dev, "Failed to register xgmac network device - %d\n", rc);
		goto error;
	}

	dev_info(&netdev->dev, "xg_mac driver at 0x%08lx (%px), irq=%d\n", (unsigned long)netdev->mem_start, local->base_addr, (int)netdev->irq);

	return 0;
error:
	if( netdev )
		free_netdev(netdev);
	return rc;
}

static int xgmac_remove(struct platform_device* pdev)
{
	struct device* dev = &pdev->dev;
	struct net_device* netdev = platform_get_drvdata(pdev);
	struct net_local* local = netdev_priv(netdev);

	dev_info(dev, "Removing xg_mac\n");

	unregister_netdev(netdev);
	free_netdev(netdev);

	return 0;
}

static const struct net_device_ops xgmac_netdev_ops = {
	.ndo_open		= xgmac_open,
	.ndo_stop		= xgmac_close,
	.ndo_start_xmit	= xgmac_send,
};

static const struct of_device_id xgmac_of_match[] = {
	{ .compatible = "fugafuga,xg_mac", },
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

MODULE_DESCRIPTION("10GMAC Ethernet Driver");
MODULE_LICENSE("GPL");