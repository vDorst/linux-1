// SPDX-License-Identifier: GPL-2.0-only
/*
 * Based on stmmac selftest
 */

#include <linux/bitrev.h>
#include <linux/completion.h>
#include <linux/crc32.h>
#include <linux/ethtool.h>
#include <linux/ip.h>
#include <linux/phy.h>
#include <linux/udp.h>
#include <net/pkt_cls.h>
#include <net/tcp.h>
#include <net/udp.h>
#include <net/tc_act/tc_gact.h>

#include "mtk_eth_soc.h"

struct mtk_packet_attrs {
	int vlan;
	int vlan_id_in;
	int vlan_id_out;
	unsigned char *src;
	unsigned char *dst;
	u32 ip_src;
	u32 ip_dst;
	int tcp;
	int sport;
	int dport;
	u32 exp_hash;
	int dont_wait;
	int timeout;
	int size;
	int max_size;
	int remove_sa;
	u8 id;
	int sarc;
	u16 queue_mapping;
};

struct mtk_test_priv {
	struct mtk_packet_attrs *packet;
	struct packet_type pt;
	struct completion comp;
	int double_vlan;
	int vlan_id;
	int ok;
};

struct stmmachdr {
	__be32 version;
	__be64 magic;
	u8 id;
} __packed;

#define STMMAC_TEST_PKT_SIZE (sizeof(struct ethhdr) + sizeof(struct iphdr) + \
			      sizeof(struct stmmachdr))
#define STMMAC_TEST_PKT_MAGIC	0xdeadcafecafedeadULL
#define STMMAC_LB_TIMEOUT	msecs_to_jiffies(200)

static int mtk_test_loopback_validate(struct sk_buff *skb,
				      struct net_device *ndev,
				      struct packet_type *pt,
				      struct net_device *orig_ndev)
{
	struct mtk_test_priv *tpriv = pt->af_packet_priv;
	unsigned char *src = tpriv->packet->src;
	unsigned char *dst = tpriv->packet->dst;
	struct stmmachdr *shdr;
	struct ethhdr *ehdr;
	struct udphdr *uhdr;
	struct tcphdr *thdr;
	struct iphdr *ihdr;

	skb = skb_unshare(skb, GFP_ATOMIC);
	if (!skb)
		goto out;

	if (skb_linearize(skb))
		goto out;
	if (skb_headlen(skb) < (STMMAC_TEST_PKT_SIZE - ETH_HLEN))
		goto out;

	ehdr = (struct ethhdr *)skb_mac_header(skb);
	if (dst) {
		if (!ether_addr_equal_unaligned(ehdr->h_dest, dst))
			goto out;
	}
	if (tpriv->packet->sarc) {
		if (!ether_addr_equal_unaligned(ehdr->h_source, ehdr->h_dest))
			goto out;
	} else if (src) {
		if (!ether_addr_equal_unaligned(ehdr->h_source, src))
			goto out;
	}

	ihdr = ip_hdr(skb);
	if (tpriv->double_vlan)
		ihdr = (struct iphdr *)(skb_network_header(skb) + 4);

	if (tpriv->packet->tcp) {
		if (ihdr->protocol != IPPROTO_TCP)
			goto out;

		thdr = (struct tcphdr *)((u8 *)ihdr + 4 * ihdr->ihl);
		if (thdr->dest != htons(tpriv->packet->dport))
			goto out;

		shdr = (struct stmmachdr *)((u8 *)thdr + sizeof(*thdr));
	} else {
		if (ihdr->protocol != IPPROTO_UDP)
			goto out;

		uhdr = (struct udphdr *)((u8 *)ihdr + 4 * ihdr->ihl);
		if (uhdr->dest != htons(tpriv->packet->dport))
			goto out;

		shdr = (struct stmmachdr *)((u8 *)uhdr + sizeof(*uhdr));
	}

	if (shdr->magic != cpu_to_be64(STMMAC_TEST_PKT_MAGIC))
		goto out;
	if (tpriv->packet->exp_hash && !skb->hash)
		goto out;
	if (tpriv->packet->id != shdr->id)
		goto out;

	tpriv->ok = true;
	complete(&tpriv->comp);
out:
	kfree_skb(skb);
	return 0;
}
static u8 mtk_test_next_id;
#define STMMAC_TEST_PKT_SIZE (sizeof(struct ethhdr) + sizeof(struct iphdr) + \
			      sizeof(struct stmmachdr))
static struct sk_buff *mtk_test_get_udp_skb(struct mtk_mac *mac,
					    struct mtk_packet_attrs *attr)
{
	struct mtk_eth *eth = mac->hw;
	struct sk_buff *skb = NULL;
	struct udphdr *uhdr = NULL;
	struct tcphdr *thdr = NULL;
	struct stmmachdr *shdr;
	struct ethhdr *ehdr;
	struct iphdr *ihdr;
	int iplen, size;

	size = attr->size + STMMAC_TEST_PKT_SIZE;
	if (attr->vlan) {
		size += 4;
		if (attr->vlan > 1)
			size += 4;
	}

	if (attr->tcp)
		size += sizeof(struct tcphdr);
	else
		size += sizeof(struct udphdr);

	if (attr->max_size && (attr->max_size > size))
		size = attr->max_size;

	skb = netdev_alloc_skb(eth->netdev[mac->id], size);
	if (!skb)
		return NULL;

	prefetchw(skb->data);

	if (attr->vlan > 1)
		ehdr = skb_push(skb, ETH_HLEN + 8);
	else if (attr->vlan)
		ehdr = skb_push(skb, ETH_HLEN + 4);
	else if (attr->remove_sa)
		ehdr = skb_push(skb, ETH_HLEN - 6);
	else
		ehdr = skb_push(skb, ETH_HLEN);
	skb_reset_mac_header(skb);

	skb_set_network_header(skb, skb->len);
	ihdr = skb_put(skb, sizeof(*ihdr));

	skb_set_transport_header(skb, skb->len);
	if (attr->tcp)
		thdr = skb_put(skb, sizeof(*thdr));
	else
		uhdr = skb_put(skb, sizeof(*uhdr));

	if (!attr->remove_sa)
		eth_zero_addr(ehdr->h_source);
	eth_zero_addr(ehdr->h_dest);
	if (attr->src && !attr->remove_sa)
		ether_addr_copy(ehdr->h_source, attr->src);
	if (attr->dst)
		ether_addr_copy(ehdr->h_dest, attr->dst);

	if (!attr->remove_sa) {
		ehdr->h_proto = htons(ETH_P_IP);
	} else {
		__be16 *ptr = (__be16 *)ehdr;

		/* HACK */
		ptr[3] = htons(ETH_P_IP);
	}

	if (attr->vlan) {
		__be16 *tag, *proto;

		if (!attr->remove_sa) {
			tag = (void *)ehdr + ETH_HLEN;
			proto = (void *)ehdr + (2 * ETH_ALEN);
		} else {
			tag = (void *)ehdr + ETH_HLEN - 6;
			proto = (void *)ehdr + ETH_ALEN;
		}

		proto[0] = htons(ETH_P_8021Q);
		tag[0] = htons(attr->vlan_id_out);
		tag[1] = htons(ETH_P_IP);
		if (attr->vlan > 1) {
			proto[0] = htons(ETH_P_8021AD);
			tag[1] = htons(ETH_P_8021Q);
			tag[2] = htons(attr->vlan_id_in);
			tag[3] = htons(ETH_P_IP);
		}
	}

	if (attr->tcp) {
		thdr->source = htons(attr->sport);
		thdr->dest = htons(attr->dport);
		thdr->doff = sizeof(struct tcphdr) / 4;
		thdr->check = 0;
	} else {
		uhdr->source = htons(attr->sport);
		uhdr->dest = htons(attr->dport);
		uhdr->len = htons(sizeof(*shdr) + sizeof(*uhdr) + attr->size);
		if (attr->max_size)
			uhdr->len = htons(attr->max_size -
					  (sizeof(*ihdr) + sizeof(*ehdr)));
		uhdr->check = 0;
	}

	ihdr->ihl = 5;
	ihdr->ttl = 32;
	ihdr->version = 4;
	if (attr->tcp)
		ihdr->protocol = IPPROTO_TCP;
	else
		ihdr->protocol = IPPROTO_UDP;
	iplen = sizeof(*ihdr) + sizeof(*shdr) + attr->size;
	if (attr->tcp)
		iplen += sizeof(*thdr);
	else
		iplen += sizeof(*uhdr);

	if (attr->max_size)
		iplen = attr->max_size - sizeof(*ehdr);

	ihdr->tot_len = htons(iplen);
	ihdr->frag_off = 0;
	ihdr->saddr = htonl(attr->ip_src);
	ihdr->daddr = htonl(attr->ip_dst);
	ihdr->tos = 0;
	ihdr->id = 0;
	ip_send_check(ihdr);

	shdr = skb_put(skb, sizeof(*shdr));
	shdr->version = 0;
	shdr->magic = cpu_to_be64(STMMAC_TEST_PKT_MAGIC);
	attr->id = mtk_test_next_id;
	shdr->id = mtk_test_next_id++;

	if (attr->size)
		skb_put(skb, attr->size);
	if (attr->max_size && (attr->max_size > skb->len))
		skb_put(skb, attr->max_size - skb->len);

	skb->csum = 0;
	skb->ip_summed = CHECKSUM_PARTIAL;
	if (attr->tcp) {
		thdr->check = ~tcp_v4_check(skb->len, ihdr->saddr, ihdr->daddr, 0);
		skb->csum_start = skb_transport_header(skb) - skb->head;
		skb->csum_offset = offsetof(struct tcphdr, check);
	} else {
		udp4_hwcsum(skb, ihdr->saddr, ihdr->daddr);
	}

	skb->protocol = htons(ETH_P_IP);
	skb->pkt_type = PACKET_HOST;
	skb->dev = eth->netdev[mac->id];

	return skb;
}

static int __mtk_test_loopback(struct mtk_mac *mac,
			       struct mtk_packet_attrs *attr)
{
	struct mtk_eth *eth = mac->hw;
	struct mtk_test_priv *tpriv;
	struct sk_buff *skb = NULL;
	int ret = 0;

	tpriv = kzalloc(sizeof(*tpriv), GFP_KERNEL);
	if (!tpriv)
		return -ENOMEM;

	tpriv->ok = false;
	init_completion(&tpriv->comp);

	tpriv->pt.type = htons(ETH_P_IP);
	tpriv->pt.func = mtk_test_loopback_validate;
	tpriv->pt.dev = eth->netdev[mac->id];
	tpriv->pt.af_packet_priv = tpriv;
	tpriv->packet = attr;

	if (!attr->dont_wait)
		dev_add_pack(&tpriv->pt);

	skb = mtk_test_get_udp_skb(mac, attr);
	if (!skb) {
		ret = -ENOMEM;
		goto cleanup;
	}

	skb_set_queue_mapping(skb, attr->queue_mapping);
	ret = dev_queue_xmit(skb);
	if (ret)
		goto cleanup;

	if (attr->dont_wait)
		goto cleanup;

	if (!attr->timeout)
		attr->timeout = STMMAC_LB_TIMEOUT;

	wait_for_completion_timeout(&tpriv->comp, attr->timeout);
	ret = tpriv->ok ? 0 : -ETIMEDOUT;

cleanup:
	if (!attr->dont_wait)
		dev_remove_pack(&tpriv->pt);
	kfree(tpriv);
	return ret;
}

static int mtk_test_mac_loopback(struct mtk_mac *mac)
{
	struct mtk_packet_attrs attr = { };
	struct mtk_eth *eth = mac->hw;
	struct net_device *ndev = eth->netdev[mac->id];

	attr.dst = ndev->dev_addr;
	return __mtk_test_loopback(mac, &attr);
}

static int mtk_test_phy_loopback(struct mtk_mac *mac)
{
	struct mtk_packet_attrs attr = { };
	struct mtk_eth *eth = mac->hw;
	struct net_device *ndev = eth->netdev[mac->id];
	int ret;

	if (!ndev->phydev)
		return -EBUSY;

	ret = phy_loopback(ndev->phydev, true);
	if (ret)
		return ret;

	attr.dst = ndev->dev_addr;
	ret = __mtk_test_loopback(mac, &attr);

	phy_loopback(ndev->phydev, false);
	return ret;
}

#define MTK_LOOPBACK_NONE	0
#define MTK_LOOPBACK_MAC	1
#define MTK_LOOPBACK_PHY	2

static const struct mtk_test {
	char name[ETH_GSTRING_LEN];
	int lb;
	int (*fn)(struct mtk_mac *mac);
} mtk_selftests[] = {
	{
		.name = "MAC Loopback               ",
		.lb = MTK_LOOPBACK_MAC,
		.fn = mtk_test_mac_loopback,
	}, {
		.name = "PHY Loopback               ",
		.lb = MTK_LOOPBACK_NONE, /* Test will handle it */
		.fn = mtk_test_phy_loopback,
	},
};

void mtk_selftest_get_strings(struct net_device *dev, u8 *data)
{
	u8 *p = data;
	int i;

	for (i = 0; i < mtk_selftest_get_count(dev); i++) {
		snprintf(p, ETH_GSTRING_LEN, "%2d. %s", i + 1,
			 mtk_selftests[i].name);
		p += ETH_GSTRING_LEN;
	}
}

int mtk_selftest_get_count(struct net_device *dev)
{
	return ARRAY_SIZE(mtk_selftests);
}

void mtk_selftest_run(struct net_device *dev,
		      struct ethtool_test *etest, u64 *buf)
{
	struct mtk_mac *mac = netdev_priv(dev);
	// struct mtk_eth *eth = mac->hw;
	int count = mtk_selftest_get_count(dev);
	int carrier = netif_carrier_ok(dev);
	int i, ret;

	memset(buf, 0, sizeof(*buf) * count);
	mtk_test_next_id = 0;

	if (etest->flags != ETH_TEST_FL_OFFLINE) {
		netdev_err(dev, "Only offline tests are supported\n");
		etest->flags |= ETH_TEST_FL_FAILED;
		return;
	} else if (!carrier) {
		netdev_err(dev, "You need valid Link to execute tests\n");
		etest->flags |= ETH_TEST_FL_FAILED;
		return;
	}

	/* We don't want extra traffic */
	netif_carrier_off(dev);

	/* Wait for queues drain */
	msleep(200);

	for (i = 0; i < count; i++) {
		ret = 0;

		switch (mtk_selftests[i].lb) {
		case MTK_LOOPBACK_NONE:
			break;
		default:
			ret = -EOPNOTSUPP;
			break;
		}

		/*
		 * First tests will always be MAC / PHY loobpack. If any of
		 * them is not supported we abort earlier.
		 */
		if (ret) {
			netdev_err(dev, "Loopback is not supported\n");
			etest->flags |= ETH_TEST_FL_FAILED;
			break;
		}

		ret = mtk_selftests[i].fn(mac);
		if (ret && (ret != -EOPNOTSUPP))
			etest->flags |= ETH_TEST_FL_FAILED;
		buf[i] = ret;

		switch (mtk_selftests[i].lb) {
		case MTK_LOOPBACK_PHY:
			ret = -EOPNOTSUPP;
			if (dev->phydev)
				ret = phy_loopback(dev->phydev, false);
			if (!ret)
				break;
			/* Fallthrough */
			break;
		default:
			break;
		}
	}

	/* Restart everything */
	if (carrier)
		netif_carrier_on(dev);
}