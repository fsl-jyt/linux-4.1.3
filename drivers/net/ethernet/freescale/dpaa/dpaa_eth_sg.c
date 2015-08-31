/* Copyright 2012 - 2015 Freescale Semiconductor Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *	 notice, this list of conditions and the following disclaimer in the
 *	 documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *	 names of its contributors may be used to endorse or promote products
 *	 derived from this software without specific prior written permission.
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/skbuff.h>
#include <linux/highmem.h>
#include <soc/fsl/bman.h>

#include "dpaa_eth.h"
#include "dpaa_eth_common.h"

/* Convenience macros for storing/retrieving the skb back-pointers.
 *
 * NB: @off is an offset from a (struct sk_buff **) pointer!
 */
#define DPA_WRITE_SKB_PTR(skb, skbh, addr, off) \
	{ \
		skbh = (struct sk_buff **)addr; \
		*(skbh + (off)) = skb; \
	}
#define DPA_READ_SKB_PTR(skb, skbh, addr, off) \
	{ \
		skbh = (struct sk_buff **)addr; \
		skb = *(skbh + (off)); \
	}

static int _dpa_bp_add_8_bufs(const struct dpa_bp *dpa_bp)
{
	struct bm_buffer bmb[8];
	void *new_buf;
	dma_addr_t addr;
	u8 i;
	struct device *dev = dpa_bp->dev;
	struct sk_buff *skb, **skbh;

	for (i = 0; i < 8; i++) {
		/* We'll prepend the skb back-pointer; can't use the DPA
		 * priv space, because FMan will overwrite it (from offset 0)
		 * if it ends up being the second, third, etc. fragment
		 * in a S/G frame.
		 *
		 * We only need enough space to store a pointer, but allocate
		 * an entire cacheline for performance reasons.
		 */
		new_buf = netdev_alloc_frag(SMP_CACHE_BYTES + DPA_BP_RAW_SIZE);
		if (unlikely(!new_buf))
			goto netdev_alloc_failed;
		new_buf = PTR_ALIGN(new_buf + SMP_CACHE_BYTES, SMP_CACHE_BYTES);

		skb = build_skb(new_buf, DPA_SKB_SIZE(dpa_bp->size) +
			SKB_DATA_ALIGN(sizeof(struct skb_shared_info)));
		if (unlikely(!skb)) {
			put_page(virt_to_head_page(new_buf));
			goto build_skb_failed;
		}
		DPA_WRITE_SKB_PTR(skb, skbh, new_buf, -1);

		addr = dma_map_single(dev, new_buf,
				      dpa_bp->size, DMA_BIDIRECTIONAL);
		if (unlikely(dma_mapping_error(dev, addr)))
			goto dma_map_failed;

		bm_buffer_set64(&bmb[i], addr);
	}

release_bufs:
	/* Release the buffers. In case bman is busy, keep trying
	 * until successful. bman_release() is guaranteed to succeed
	 * in a reasonable amount of time
	 */
	while (unlikely(bman_release(dpa_bp->pool, bmb, i, 0)))
		cpu_relax();
	return i;

dma_map_failed:
	kfree_skb(skb);

build_skb_failed:
netdev_alloc_failed:
	net_err_ratelimited("dpa_bp_add_8_bufs() failed\n");
	WARN_ONCE(1, "Memory allocation failure on Rx\n");

	bm_buffer_set64(&bmb[i], 0);
	/* Avoid releasing a completely null buffer; bman_release() requires
	 * at least one buffer.
	 */
	if (likely(i))
		goto release_bufs;

	return 0;
}

/* Cold path wrapper over _dpa_bp_add_8_bufs(). */
static void dpa_bp_add_8_bufs(const struct dpa_bp *dpa_bp, int cpu)
{
	int *count_ptr = per_cpu_ptr(dpa_bp->percpu_count, cpu);
	*count_ptr += _dpa_bp_add_8_bufs(dpa_bp);
}

int dpa_bp_priv_seed(struct dpa_bp *dpa_bp)
{
	int i;

	/* Give each CPU an allotment of "config_count" buffers */
	for_each_possible_cpu(i) {
		int j;

		/* Although we access another CPU's counters here
		 * we do it at boot time so it is safe
		 */
		for (j = 0; j < dpa_bp->config_count; j += 8)
			dpa_bp_add_8_bufs(dpa_bp, i);
	}
	return 0;
}

/* Add buffers/(pages) for Rx processing whenever bpool count falls below
 * REFILL_THRESHOLD.
 */
int dpaa_eth_refill_bpools(struct dpa_bp *dpa_bp, int *countptr)
{
	int count = *countptr;
	int new_bufs;

	if (unlikely(count < FSL_DPAA_ETH_REFILL_THRESHOLD)) {
		do {
			new_bufs = _dpa_bp_add_8_bufs(dpa_bp);
			if (unlikely(!new_bufs)) {
				/* Avoid looping forever if we've temporarily
				 * run out of memory. We'll try again at the
				 * next NAPI cycle.
				 */
				break;
			}
			count += new_bufs;
		} while (count < FSL_DPAA_ETH_MAX_BUF_COUNT);

		*countptr = count;
		if (unlikely(count < FSL_DPAA_ETH_MAX_BUF_COUNT))
			return -ENOMEM;
	}

	return 0;
}

/* Cleanup function for outgoing frame descriptors that were built on Tx path,
 * either contiguous frames or scatter/gather ones.
 * Skb freeing is not handled here.
 *
 * This function may be called on error paths in the Tx function, so guard
 * against cases when not all fd relevant fields were filled in.
 *
 * Return the skb backpointer, since for S/G frames the buffer containing it
 * gets freed here.
 */
struct sk_buff *_dpa_cleanup_tx_fd(const struct dpa_priv_s *priv,
				   const struct qm_fd *fd)
{
	struct dpa_bp *dpa_bp = priv->dpa_bp;
	dma_addr_t addr = qm_fd_addr(fd);
	struct sk_buff **skbh;
	struct sk_buff *skb = NULL;
	const enum dma_data_direction dma_dir = DMA_TO_DEVICE;

	/* retrieve skb back pointer */
	DPA_READ_SKB_PTR(skb, skbh, phys_to_virt(addr), 0);
	dma_unmap_single(dpa_bp->dev, addr,
			 skb_tail_pointer(skb) - (u8 *)skbh, dma_dir);
	return skb;
}

/* Build a linear skb around the received buffer.
 * We are guaranteed there is enough room at the end of the data buffer to
 * accommodate the shared info area of the skb.
 */
static struct sk_buff *contig_fd_to_skb(const struct dpa_priv_s *priv,
					const struct qm_fd *fd)
{
	struct sk_buff *skb = NULL, **skbh;
	ssize_t fd_off = dpa_fd_offset(fd);
	dma_addr_t addr = qm_fd_addr(fd);
	void *vaddr;

	vaddr = phys_to_virt(addr);
	DPA_ERR_ON(!IS_ALIGNED((unsigned long)vaddr, SMP_CACHE_BYTES));

	/* Retrieve the skb and adjust data and tail pointers, to make sure
	 * forwarded skbs will have enough space on Tx if extra headers
	 * are added.
	 */
	DPA_READ_SKB_PTR(skb, skbh, vaddr, -1);

	DPA_ERR_ON(fd_off != priv->rx_headroom);
	skb_reserve(skb, fd_off);
	skb_put(skb, dpa_fd_length(fd));

	skb->ip_summed = CHECKSUM_NONE;

	return skb;
}

void _dpa_rx(struct net_device *net_dev,
	     struct qman_portal *portal,
	     const struct dpa_priv_s *priv,
	     struct dpa_percpu_priv_s *percpu_priv,
	     const struct qm_fd *fd,
	     u32 fqid,
	     int *count_ptr)
{
	struct dpa_bp *dpa_bp;
	struct sk_buff *skb;
	dma_addr_t addr = qm_fd_addr(fd);
	u32 fd_status = fd->status;
	unsigned int skb_len;
	struct rtnl_link_stats64 *percpu_stats = &percpu_priv->stats;

	if (unlikely(fd_status & FM_FD_STAT_RX_ERRORS) != 0) {
		if (net_ratelimit())
			netif_warn(priv, hw, net_dev, "FD status = 0x%08x\n",
				   fd_status & FM_FD_STAT_RX_ERRORS);

		percpu_stats->rx_errors++;
		goto _release_frame;
	}

	dpa_bp = priv->dpa_bp;
	DPA_ERR_ON(dpa_bp != dpa_bpid2pool(fd->bpid));

	/* prefetch the first 64 bytes of the frame */
	dma_unmap_single(dpa_bp->dev, addr, dpa_bp->size, DMA_BIDIRECTIONAL);
	prefetch(phys_to_virt(addr) + dpa_fd_offset(fd));

	/* The only FD type that we may receive is contig */
	DPA_ERR_ON((fd->format != qm_fd_contig));

	skb = contig_fd_to_skb(priv, fd);

	/* Account for the contig buffer
	 * having been removed from the pool.
	 */
	(*count_ptr)--;
	skb->protocol = eth_type_trans(skb, net_dev);

	/* IP Reassembled frames are allowed to be larger than MTU */
	if (unlikely(dpa_check_rx_mtu(skb, net_dev->mtu) &&
		     !(fd_status & FM_FD_IPR))) {
		percpu_stats->rx_dropped++;
		goto drop_bad_frame;
	}

	skb_len = skb->len;

	if (unlikely(netif_receive_skb(skb) == NET_RX_DROP))
		goto packet_dropped;

	percpu_stats->rx_packets++;
	percpu_stats->rx_bytes += skb_len;

packet_dropped:
	return;

drop_bad_frame:
	dev_kfree_skb(skb);
	return;

_release_frame:
	dpa_fd_release(net_dev, fd);
}

static int skb_to_contig_fd(struct dpa_priv_s *priv,
			    struct sk_buff *skb, struct qm_fd *fd,
			    int *count_ptr, int *offset)
{
	struct sk_buff **skbh;
	dma_addr_t addr;
	struct dpa_bp *dpa_bp = priv->dpa_bp;
	struct net_device *net_dev = priv->net_dev;
	int err;
	enum dma_data_direction dma_dir;
	unsigned char *buffer_start;

	{
		/* We are guaranteed to have at least tx_headroom bytes
		 * available, so just use that for offset.
		 */
		fd->bpid = 0xff;
		buffer_start = skb->data - priv->tx_headroom;
		fd->offset = priv->tx_headroom;
		dma_dir = DMA_TO_DEVICE;

		DPA_WRITE_SKB_PTR(skb, skbh, buffer_start, 0);
	}

	/* Enable L3/L4 hardware checksum computation.
	 *
	 * We must do this before dma_map_single(DMA_TO_DEVICE), because we may
	 * need to write into the skb.
	 */
	err = dpa_enable_tx_csum(priv, skb, fd,
				 ((char *)skbh) + DPA_TX_PRIV_DATA_SIZE);
	if (unlikely(err < 0)) {
		if (net_ratelimit())
			netif_err(priv, tx_err, net_dev, "HW csum error: %d\n",
				  err);
		return err;
	}

	/* Fill in the rest of the FD fields */
	fd->format = qm_fd_contig;
	fd->length20 = skb->len;
	fd->cmd |= FM_FD_CMD_FCO;

	/* Map the entire buffer size that may be seen by FMan, but no more */
	addr = dma_map_single(dpa_bp->dev, skbh,
			      skb_tail_pointer(skb) - buffer_start, dma_dir);
	if (unlikely(dma_mapping_error(dpa_bp->dev, addr))) {
		if (net_ratelimit())
			netif_err(priv, tx_err, net_dev, "dma_map_single() failed\n");
		return -EINVAL;
	}
	fd->addr_hi = (u8)upper_32_bits(addr);
	fd->addr_lo = lower_32_bits(addr);

	return 0;
}

int dpa_tx(struct sk_buff *skb, struct net_device *net_dev)
{
	struct dpa_priv_s *priv;
	struct qm_fd fd;
	struct dpa_percpu_priv_s *percpu_priv;
	struct rtnl_link_stats64 *percpu_stats;
	int err = 0;
	const int queue_mapping = dpa_get_queue_mapping(skb);
	int *countptr, offset = 0;

	priv = netdev_priv(net_dev);
	/* Non-migratable context, safe to use raw_cpu_ptr */
	percpu_priv = raw_cpu_ptr(priv->percpu_priv);
	percpu_stats = &percpu_priv->stats;
	countptr = raw_cpu_ptr(priv->dpa_bp->percpu_count);

	clear_fd(&fd);

	/* We're going to store the skb backpointer at the beginning
	 * of the data buffer, so we need a privately owned skb
	 *
	 * We've made sure skb is not shared in dev->priv_flags,
	 * we need to verify the skb head is not cloned
	 */
	if (skb_cow_head(skb, priv->tx_headroom))
		goto enomem;

	DPA_ERR_ON(skb_is_nonlinear(skb));

	/* Finally, create a contig FD from this skb */
	err = skb_to_contig_fd(priv, skb, &fd, countptr, &offset);
	if (unlikely(err < 0))
		goto skb_to_fd_failed;

	if (likely(dpa_xmit(priv, percpu_stats, queue_mapping, &fd) == 0))
		return NETDEV_TX_OK;

	/* dpa_xmit failed */
	if (fd.bpid != 0xff) {
		(*countptr)--;
		dpa_fd_release(net_dev, &fd);
		percpu_stats->tx_errors++;
		return NETDEV_TX_OK;
	}
	_dpa_cleanup_tx_fd(priv, &fd);
skb_to_fd_failed:
enomem:
	percpu_stats->tx_errors++;
	dev_kfree_skb(skb);
	return NETDEV_TX_OK;
}
