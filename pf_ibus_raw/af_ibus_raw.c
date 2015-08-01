/*
 * af_ibus_raw.c - implements raw sockets of I/K bus protocol family.
 *
 * This file is derived from /net/can/af_can.c, /net/can/raw.c
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307.
 * 
 */

#include <linux/module.h>
#include <linux/stddef.h>
#include <linux/init.h>
#include <linux/kmod.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/rcupdate.h>
#include <linux/uaccess.h>
#include <linux/net.h>
#include <linux/netdevice.h>
#include <linux/socket.h>
#include <linux/if_ether.h>
#include <linux/if_arp.h>
#include <linux/skbuff.h>
#include <net/net_namespace.h>
#include <net/sock.h>
#include <linux/uio.h>
#include <linux/ibus.h>

static __initconst const char banner[] =
	KERN_INFO "ibus_raw: raw protocol for I/K communication bus\n";

MODULE_ALIAS_NETPROTO(PF_IBUS);
MODULE_DESCRIPTION("raw protocol I/K bus");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Vladimir Korol <vovabox@mail.ru>");

/* per device receive filters linked at dev->ml_priv */
static struct dev_rcv_lists {
	struct hlist_head rx;
	int remove_on_zero_entries;
	int entries;
} ibus_rx_alldev_list;

struct receiver {
	struct hlist_node list;
	struct rcu_head rcu;
	u_int8_t id_tx;
	u_int8_t id_rx;
	unsigned long matches;
	void (*func)(struct sk_buff *, void *);
	void *data;
	char *ident;
};

static struct kmem_cache *rcv_cache __read_mostly;
static DEFINE_SPINLOCK(ibus_rcvlists_lock);

struct ibus_sock {
	struct sock sk;
	int bound;
	int ifindex;
	struct notifier_block notifier;
	int count;                  /* number of active filters */
	struct ibus_filter dfilter; /* default/single filter */
	struct ibus_filter *filter; /* pointer to filter(s) */
	void *chk_skb;              /* to prevent double receiving */

};

static void ibus_disable_filters(struct net_device *, struct sock *,
			      struct ibus_filter *, int);

static int ibus_sk_notifier(struct notifier_block *nb, unsigned long msg,
			void *ptr)
{
	struct net_device *dev = netdev_notifier_info_to_dev(ptr);
	struct ibus_sock *ibo = container_of(nb, struct ibus_sock, notifier);
	struct sock *sk = &ibo->sk;

	if (!net_eq(dev_net(dev), &init_net))
		return NOTIFY_DONE;

	if (dev->type != ARPHRD_IBUS)
		return NOTIFY_DONE;

	if (ibo->ifindex != dev->ifindex)
		return NOTIFY_DONE;

	switch (msg) {

	case NETDEV_UNREGISTER:
		lock_sock(sk);
		/* remove current filters & unregister */
		if (ibo->bound)
			ibus_disable_filters(dev, sk, ibo->filter, ibo->count);

		if (ibo->count > 1)
			kfree(ibo->filter);

		ibo->ifindex = 0;
		ibo->bound   = 0;
		ibo->count   = 0;
		release_sock(sk);

		sk->sk_err = ENODEV;
		if (!sock_flag(sk, SOCK_DEAD))
			sk->sk_error_report(sk);
		break;

	case NETDEV_DOWN:
		sk->sk_err = ENETDOWN;
		if (!sock_flag(sk, SOCK_DEAD))
			sk->sk_error_report(sk);
		break;
	}

	return NOTIFY_DONE;
}


static int ibus_proto_init(struct sock *sk)
{
	struct ibus_sock *ibo = (struct ibus_sock *)sk;

	ibo->bound            = 0;
	ibo->ifindex          = 0;

	/* set default filter to single entry dfilter */
	ibo->dfilter.id_tx      = IBUS_ID_ALL;
	ibo->dfilter.id_rx      = IBUS_ID_ALL;
	ibo->filter           = &ibo->dfilter;
	ibo->count            = 1;

	/* set notifier */
	ibo->notifier.notifier_call = ibus_sk_notifier;
	register_netdevice_notifier(&ibo->notifier);

	ibo->chk_skb = NULL;

	return 0;
}

static struct proto ibus_proto __read_mostly = {
	.name       = "IBUS",
	.owner      = THIS_MODULE,
	.obj_size   = sizeof(struct ibus_sock),
	.init       = ibus_proto_init,
};
/* Maybe handler to transfer data from packet to socket */
static void ibus_sk_rcv(struct sk_buff *oskb, void *data)
{
	struct sock *sk = (struct sock *)data;
	struct sockaddr_ibus *addr;
	struct sk_buff *skb;
	unsigned int *pflags;

	/* check the received tx sock reference */
	if (oskb->sk == sk)
		return;

	/* clone the given skb to be able to enqueue it into the rcv queue */
	skb = skb_clone(oskb, GFP_ATOMIC);
	if (!skb)
		return;

	/*
	 *  Put the datagram to the queue so that raw_recvmsg() can
	 *  get it from there.  We need to pass the interface index to
	 *  raw_recvmsg().  We pass a whole struct sockaddr_ibus in skb->cb
	 *  containing the interface index.
	 */
	BUILD_BUG_ON(sizeof(skb->cb) < sizeof(struct sockaddr_ibus));
	addr = (struct sockaddr_ibus *)skb->cb;
	memset(addr, 0, sizeof(*addr));
	addr->ibus_family  = AF_IBUS;
	addr->ifindex = skb->dev->ifindex;

	/* add IBUS specific message flags for ibus_recvmsg() */
	BUILD_BUG_ON(sizeof(skb->cb) <= (sizeof(struct sockaddr_ibus) +
					 sizeof(unsigned int)));
	pflags = (unsigned int *)(&((struct sockaddr_ibus *)skb->cb)[1]);
	*pflags = 0;
	if (oskb->sk)
		*pflags |= MSG_DONTROUTE;
	if (oskb->sk == sk)
		*pflags |= MSG_CONFIRM;

	if (sock_queue_rcv_skb(sk, skb) < 0)
		kfree_skb(skb);
}

static struct dev_rcv_lists *find_dev_rcv_lists(struct net_device *dev)
{
	if (!dev) {
		return &ibus_rx_alldev_list;
	}
	else {
		return (struct dev_rcv_lists *)dev->ml_priv;
	}

}

static void ibus_rx_delete_receiver(struct rcu_head *rp)
{
	struct receiver *r = container_of(rp, struct receiver, rcu);

	kmem_cache_free(rcv_cache, r);
}

static int ibus_rx_register(struct net_device *dev, u_int8_t id_tx,
		    u_int8_t id_rx, void (*func)(struct sk_buff *, void *),
		    void *data, char *ident)
{
	struct receiver *r;
	struct dev_rcv_lists *d;
	int err = 0;

	/* insert new receiver  (dev, rx_id, tx_id) -> (func,data) */

	if (dev && dev->type != ARPHRD_IBUS)
		return -ENODEV;

	r = kmem_cache_alloc(rcv_cache, GFP_KERNEL);
	if (!r)
		return -ENOMEM;

	spin_lock(&ibus_rcvlists_lock);

	d = find_dev_rcv_lists(dev);
	if (d) {
		r->id_tx   = id_tx;
		r->id_rx   = id_rx;
		r->matches = 0;
		r->func    = func;
		r->data    = data;
		r->ident   = ident;

		hlist_add_head_rcu(&r->list, &d->rx);
		d->entries++;
	} else {
		kmem_cache_free(rcv_cache, r);
		err = -ENODEV;
	}

	spin_unlock(&ibus_rcvlists_lock);

	return err;
}

static void ibus_rx_unregister(struct net_device *dev, u_int8_t id_tx,
		       u_int8_t id_rx, void (*func)(struct sk_buff *, void *),
		       void *data)
{
	struct receiver *r = NULL;
	struct dev_rcv_lists *d;

	if (dev && dev->type != ARPHRD_IBUS)
		return;

	spin_lock(&ibus_rcvlists_lock);

	d = find_dev_rcv_lists(dev);
	if (!d) {
		printk(KERN_ERR "IBUS_RAW: receive list not found for "
		       "dev %s, id %03X, mask %03X\n",
		       (dev ? dev->name : "any"), id_tx, id_rx);
		goto out;
	}

	/*
	 * Search the receiver list for the item to delete.  This should
	 * exist, since no receiver may be unregistered that hasn't
	 * been registered before.
	 */

	hlist_for_each_entry_rcu(r, &d->rx, list) {
		if (r->id_tx == id_tx && r->id_rx == id_rx &&
		    r->func == func && r->data == data)
			break;
	}

	if (!r) {
		printk(KERN_WARNING "IBUS_RAW: receive list entry not found for "
		     "dev %s, id %03X, mask %03X\n", 
		     (dev ? dev->name : "any"), id_tx, id_rx);
		goto out;
	}

	hlist_del_rcu(&r->list);
	d->entries--;

	/* remove device structure requested by NETDEV_UNREGISTER */
	if (d->remove_on_zero_entries && !d->entries) {
		kfree(d);
		dev->ml_priv = NULL;
	}

 out:
	spin_unlock(&ibus_rcvlists_lock);

	/* schedule the receiver item for deletion */
	if (r)
		call_rcu(&r->rcu, ibus_rx_delete_receiver);
}

static int ibus_enable_filters(struct net_device *dev, struct sock *sk,
			      struct ibus_filter *filter, int count)
{
	int err = 0;
	int i;

	for (i = 0; i < count; i++) {
		err = ibus_rx_register(dev, filter[i].id_tx,
				      filter[i].id_rx,
				      ibus_sk_rcv, sk, "ibus");
		if (err) {
			/* clean up successfully registered filters */
			while (--i >= 0)
				ibus_rx_unregister(dev, filter[i].id_tx,
						  filter[i].id_rx,
						  ibus_sk_rcv, sk);
			break;
		}
	}

	return err;
}

static void ibus_disable_filters(struct net_device *dev, struct sock *sk,
			      struct ibus_filter *filter, int count)
{
	int i;

	for (i = 0; i < count; i++)
		ibus_rx_unregister(dev, filter[i].id_tx, filter[i].id_rx,
				  ibus_sk_rcv, sk);
}

static int ibus_release(struct socket *sock)
{
	struct sock *sk = sock->sk;
	struct ibus_sock *ibo;

	if (!sk)
		return 0;

	ibo = (struct ibus_sock *)sk;

	unregister_netdevice_notifier(&ibo->notifier);

	lock_sock(sk);

	/* remove current filters & unregister */
	if (ibo->bound) {
		if (ibo->ifindex) {
			struct net_device *dev;
			dev = dev_get_by_index(&init_net, ibo->ifindex);
			if (dev) {
				ibus_disable_filters(dev, sk, ibo->filter, ibo->count);
				dev_put(dev);
			}
		} else
			ibus_disable_filters(NULL, sk, ibo->filter, ibo->count);
	}

	if (ibo->count > 1)
		kfree(ibo->filter);

	ibo->ifindex = 0;
	ibo->bound   = 0;
	ibo->count   = 0;

	sock_orphan(sk);
	sock->sk = NULL;

	release_sock(sk);
	sock_put(sk);

	return 0;
}

static int ibus_bind(struct socket *sock, struct sockaddr *uaddr, int len)
{
	struct sockaddr_ibus *addr = (struct sockaddr_ibus *)uaddr;
	struct sock *sk = sock->sk;
	struct ibus_sock *ibo = (struct ibus_sock *)sk;
	int ifindex;
	int err = 0;
	int notify_enetdown = 0;

	if (len < sizeof(*addr))
		return -EINVAL;

	lock_sock(sk);

	if (ibo->bound && addr->ifindex == ibo->ifindex)
		goto out;

	if (addr->ifindex) {
		struct net_device *dev;

		dev = dev_get_by_index(&init_net, addr->ifindex);
		if (!dev) {
			err = -ENODEV;
			goto out;
		}
		if (dev->type != ARPHRD_IBUS) {
			dev_put(dev);
			err = -ENODEV;
			goto out;
		}
		if (!(dev->flags & IFF_UP))
			notify_enetdown = 1;

		ifindex = dev->ifindex;

		/* filters set by default/setsockopt */
		err = ibus_enable_filters(dev, sk, ibo->filter, ibo->count);
		dev_put(dev);
	} else {
		ifindex = 0;

		/* filters set by default/setsockopt */
		err = ibus_enable_filters(NULL, sk, ibo->filter, ibo->count);
	}

	if (!err) {
		if (ibo->bound) {
			/* unregister old filters */
			if (ibo->ifindex) {
				struct net_device *dev;

				dev = dev_get_by_index(&init_net, ibo->ifindex);
				if (dev) {
					ibus_disable_filters(dev, sk, ibo->filter, ibo->count);
					dev_put(dev);
				}
			} else
				ibus_disable_filters(NULL, sk, ibo->filter, ibo->count);
		}
		ibo->ifindex = ifindex;
		ibo->bound = 1;
	}

 out:
	release_sock(sk);

	if (notify_enetdown) {
		sk->sk_err = ENETDOWN;
		if (!sock_flag(sk, SOCK_DEAD))
			sk->sk_error_report(sk);
	}

	return err;
}

static int ibus_getname(struct socket *sock, struct sockaddr *uaddr,
		       int *len, int peer)
{
	struct sockaddr_ibus *addr = (struct sockaddr_ibus *)uaddr;
	struct sock *sk = sock->sk;
	struct ibus_sock *ibo = (struct ibus_sock *)sk;

	if (peer)
		return -EOPNOTSUPP;

	memset(addr, 0, sizeof(*addr));
	addr->ibus_family  = AF_IBUS;
	addr->ifindex = ibo->ifindex;

	*len = sizeof(*addr);

	return 0;
}

int ibus_ioctl(struct socket *sock, unsigned int cmd, unsigned long arg)
{
	struct sock *sk = sock->sk;

	switch (cmd) {

	case SIOCGSTAMP:
		return sock_get_timestamp(sk, (struct timeval __user *)arg);

	default:
		return -ENOIOCTLCMD;
	}
}

static int ibus_setsockopt(struct socket *sock, int level, int optname,
			  char __user *optval, unsigned int optlen)
{
	struct sock *sk = sock->sk;
	struct ibus_sock *ibo = (struct ibus_sock *)sk;
	struct ibus_filter *filter = NULL;  /* dyn. alloc'ed filters */
	struct ibus_filter sfilter;         /* single filter */
	struct net_device *dev = NULL;

	int count = 0;
	int err = 0;

	if (level != SOL_IBUS)
		return -EINVAL;

	switch (optname) {

	case IBUS_FILTER:
		if (optlen % sizeof(struct ibus_filter) != 0)
			return -EINVAL;

		count = optlen / sizeof(struct ibus_filter);

		if (count > 1) {
			/* filter does not fit into dfilter => alloc space */
			filter = memdup_user(optval, optlen);
			if (IS_ERR(filter))
				return PTR_ERR(filter);

		} else if (count == 1) {
			if (copy_from_user(&sfilter, optval, sizeof(sfilter)))
				return -EFAULT;
		}

		lock_sock(sk);

		if (ibo->bound && ibo->ifindex)
			dev = dev_get_by_index(&init_net, ibo->ifindex);

		if (ibo->bound) {
			/* (try to) register the new filters */
			if (count == 1)
				err = ibus_enable_filters(dev, sk, &sfilter, 1);
			else
				err = ibus_enable_filters(dev, sk, filter, count);
			if (err) {
				if (count > 1)
					kfree(filter);
				goto out_fil;
			}

			/* remove old filter registrations */
			ibus_disable_filters(dev, sk, ibo->filter, ibo->count);
		}

		/* remove old filter space */
		if (ibo->count > 1)
			kfree(ibo->filter);

		/* link new filters to the socket */
		if (count == 1) {
			/* copy filter data for single filter */
			ibo->dfilter = sfilter;
			filter = &ibo->dfilter;
		}
		ibo->filter = filter;
		ibo->count  = count;

 out_fil:
		if (dev)
			dev_put(dev);

		release_sock(sk);

		break;

	case IBUS_ERR_FILTER:

		break;

	case IBUS_RECV_OWN_MSGS:

		break;

	case IBUS_ONLY_DATA:

		break;

	default:
		return -ENOPROTOOPT;
	}
	return err;
}

static int ibus_getsockopt(struct socket *sock, int level, int optname,
			  char __user *optval, int __user *optlen)
{
	struct sock *sk = sock->sk;
	struct ibus_sock *ibo = (struct ibus_sock *)sk;
	int len;
	int err = 0;

	if (level != SOL_IBUS)
		return -EINVAL;
	if (get_user(len, optlen))
		return -EFAULT;
	if (len < 0)
		return -EINVAL;

	switch (optname) {

	case IBUS_FILTER:
		lock_sock(sk);
		if (ibo->count > 0) {
			int fsize = ibo->count * sizeof(struct ibus_filter);
			if (len > fsize)
				len = fsize;
			if (copy_to_user(optval, ibo->filter, len))
				err = -EFAULT;
		} else
			len = 0;
		release_sock(sk);

		if (!err)
			err = put_user(len, optlen);
		return err;

	case IBUS_ERR_FILTER:

		break;

	case IBUS_RECV_OWN_MSGS:

		break;

	case IBUS_ONLY_DATA:

		break;

	default:
		return -ENOPROTOOPT;
	}

	return 0;
}

static int ibus_sendmsg(struct kiocb *iocb, struct socket *sock,
		       struct msghdr *msg, size_t size)
{
	struct sock *sk = sock->sk;
	struct ibus_sock *ibo = (struct ibus_sock *)sk;
	struct sk_buff *skb;
	struct net_device *dev;
	int frame_len;
	int ifindex;
	int err;

	if (msg->msg_name) {
		DECLARE_SOCKADDR(struct sockaddr_ibus *, addr, msg->msg_name);

		if (msg->msg_namelen < sizeof(*addr))
			return -EINVAL;

		if (addr->ibus_family != AF_IBUS)
			return -EINVAL;

		ifindex = addr->ifindex;
	} else
		ifindex = ibo->ifindex;

	if (unlikely((size > IBUS_MAX_DLEN + 2) && (size < 4)))
		return -EINVAL;

	dev = dev_get_by_index(&init_net, ifindex);
	if (unlikely(!dev))
		return -ENXIO;

	if (unlikely(dev->type != ARPHRD_IBUS))
		return -EPERM;

	skb = sock_alloc_send_skb(sk, size,
				  msg->msg_flags & MSG_DONTWAIT, &err);
	if (!skb)
		goto put_dev;

	err = memcpy_fromiovec(skb_put(skb, size), msg->msg_iov, size);
	if (err < 0)
		goto free_skb;

	sock_tx_timestamp(sk, &skb_shinfo(skb)->tx_flags);

	skb->dev = dev;
	frame_len = (int)skb->data[1];

	/* frame must be with or without check sum */
	if (unlikely((skb->len < frame_len + 1) ||
			   (skb->len > frame_len + 2) ||
			   (frame_len > IBUS_MAX_DLEN))) {
		err = -EINVAL;
		goto free_skb;
	}

	if (unlikely(skb->len > skb->dev->mtu )) {
		err = -EMSGSIZE;
		goto free_skb;
	}

	if (unlikely(!(skb->dev->flags & IFF_UP))) {
		err = -ENETDOWN;
		goto free_skb;
	}

	skb->sk  = sk;
	skb->priority = sk->sk_priority;
	skb_reset_network_header(skb);
	skb_reset_transport_header(skb);
	skb->protocol = htons(ETH_P_IBUS);
	skb->pkt_type = PACKET_HOST;

	/* send to netdevice */
	err = dev_queue_xmit(skb);
	if (err > 0) {
		err = net_xmit_errno(err);
		goto free_skb;
	}

	dev_put(dev);
	return size;

free_skb:
	kfree_skb(skb);

put_dev:
	dev_put(dev);
	return err;
}

static int ibus_recvmsg(struct kiocb *iocb, struct socket *sock,
		       struct msghdr *msg, size_t size, int flags)
{
	struct sock *sk = sock->sk;
	struct sk_buff *skb;
	int err = 0;
	int noblock;

	noblock =  flags & MSG_DONTWAIT;
	flags   &= ~MSG_DONTWAIT;

	skb = skb_recv_datagram(sk, flags, noblock, &err);
	if (!skb)
		return err;

	if (size < skb->len)
		msg->msg_flags |= MSG_TRUNC;
	else
		size = skb->len;

	err = memcpy_toiovec(msg->msg_iov, skb->data, size);
	if (err < 0) {
		skb_free_datagram(sk, skb);
		return err;
	}

	sock_recv_ts_and_drops(msg, sk, skb);

	if (msg->msg_name) {
		__sockaddr_check_size(sizeof(struct sockaddr_ibus));
		msg->msg_namelen = sizeof(struct sockaddr_ibus);
		memcpy(msg->msg_name, skb->cb, msg->msg_namelen);
	}

	/* assign the flags that have been recorded in ibus_rcv1() */
	BUILD_BUG_ON(sizeof(skb->cb) <= (sizeof(struct sockaddr_ibus) +
				 sizeof(unsigned int)));

	/* return pointer after struct sockaddr_ibus */
	msg->msg_flags |= *((unsigned int *) (&((struct sockaddr_ibus *)skb->cb)[1]));

	skb_free_datagram(sk, skb);

	return size;
}

static const struct proto_ops ibus_proto_ops = {
	.family        = PF_IBUS,
	.release       = ibus_release,
	.bind          = ibus_bind,
	.connect       = sock_no_connect,
	.socketpair    = sock_no_socketpair,
	.accept        = sock_no_accept,
	.getname       = ibus_getname,
	.poll          = datagram_poll,
	.ioctl         = ibus_ioctl,
	.listen        = sock_no_listen,
	.shutdown      = sock_no_shutdown,
	.setsockopt    = ibus_setsockopt,
	.getsockopt    = ibus_getsockopt,
	.sendmsg       = ibus_sendmsg,
	.recvmsg       = ibus_recvmsg,
	.mmap          = sock_no_mmap,
	.sendpage      = sock_no_sendpage,
};

static void ibus_sock_destruct(struct sock *sk)
{
	skb_queue_purge(&sk->sk_receive_queue);
}

static int ibus_create(struct net *net, struct socket *sock, int protocol,
		      int kern)
{
	struct sock *sk;
	int err = 0;

	sock->state = SS_UNCONNECTED;

	if (!net_eq(net, &init_net))
		return -EAFNOSUPPORT;

	if (sock->type != SOCK_RAW)
		return -EPROTOTYPE;

	sock->ops = &ibus_proto_ops;

	sk = sk_alloc(net, PF_IBUS, GFP_KERNEL, &ibus_proto);
	if (!sk)
		return -ENOMEM;

	sock_init_data(sock, sk);

	sk->sk_destruct = ibus_sock_destruct;

	if (sk->sk_prot->init)
		err = sk->sk_prot->init(sk);

	if (err) {
		/* release sk on errors */
		sock_orphan(sk);
		sock_put(sk);
	}

	return err;
}

//----------------------------------------------------------------------
static const struct net_proto_family ibus_family_ops = {
	.family = PF_IBUS,
	.create = ibus_create,
	.owner  = THIS_MODULE,
};

static inline void deliver(struct sk_buff *skb, struct receiver *r)
{
	r->func(skb, r->data);
	r->matches++;
}

static int ibus_rcv_filter(struct dev_rcv_lists *d, struct sk_buff *skb)
{
	struct receiver *r;
	int matches = 0;
	u_int8_t rx_id = (u_int8_t)skb->data[2];
	u_int8_t tx_id = (u_int8_t)skb->data[0];

	if (d->entries == 0)
		return 0;

	/* check for rx_id tx_id entries */
	hlist_for_each_entry_rcu(r, &d->rx, list) {
		if (((0xff == r->id_rx) || (rx_id == r->id_rx)) && 
			((0xff == r->id_tx) || (tx_id == r->id_tx))) {
			deliver(skb, r);
			matches++;
		}
	}
	return matches;
}

static void ibus_receive(struct sk_buff *skb, struct net_device *dev)
{
	struct dev_rcv_lists *d;
	int matches;

	rcu_read_lock();

	/* deliver the packet to sockets listening on all devices */
	matches = ibus_rcv_filter(&ibus_rx_alldev_list, skb);

	/* find receive list for this device */
	d = find_dev_rcv_lists(dev);
	if (d)
		matches += ibus_rcv_filter(d, skb);

	rcu_read_unlock();

	/* consume the skbuff allocated by the netdevice driver */
	consume_skb(skb);
};

static int ibus_rcv(struct sk_buff *skb, struct net_device *dev,
		   struct packet_type *pt, struct net_device *orig_dev)
{

	if (unlikely(!net_eq(dev_net(dev), &init_net)))
		goto drop;

	if (WARN_ONCE(dev->type != ARPHRD_IBUS ||
		      skb->len < 5 ||
		      skb->len > IBUS_MAX_DLEN + 2,
		      "PF_IBUS: dropped non conform IBUS skbuf: "
		      "dev type %d, len %d, datalen %d\n",
		      dev->type, skb->len, skb->data[1]))
		goto drop;
	ibus_receive(skb, dev);
	return NET_RX_SUCCESS;

drop:
	kfree_skb(skb);
	return NET_RX_DROP;
};


static struct packet_type ibus_packet __read_mostly = {
	.type = cpu_to_be16(ETH_P_IBUS),
	.func = ibus_rcv,
};

/*
 * ibus notifier to create/remove IBUS netdevice specific structs
 */
static int ibus_notifier(struct notifier_block *nb, unsigned long msg,
			void *ptr)
{
	struct net_device *dev = netdev_notifier_info_to_dev(ptr);
	struct dev_rcv_lists *d;

	if (!net_eq(dev_net(dev), &init_net))
		return NOTIFY_DONE;

	if (dev->type != ARPHRD_IBUS)
		return NOTIFY_DONE;

	switch (msg) {

	case NETDEV_REGISTER:

		/* create new dev_rcv_lists for this device */
		d = kzalloc(sizeof(*d), GFP_KERNEL);
		if (!d)
			return NOTIFY_DONE;
		BUG_ON(dev->ml_priv);
		dev->ml_priv = d;

		break;

	case NETDEV_UNREGISTER:
		spin_lock(&ibus_rcvlists_lock);

		d = dev->ml_priv;
		if (d) {
			if (d->entries)
				d->remove_on_zero_entries = 1;
			else {
				kfree(d);
				dev->ml_priv = NULL;
			}
		} else
			pr_err("ibus_raw: notifier: receive list not found for dev "
			       "%s\n", dev->name);

		spin_unlock(&ibus_rcvlists_lock);

		break;
	}

	return NOTIFY_DONE;
}

/* notifier block for netdevice event */
static struct notifier_block ibus_netdev_notifier __read_mostly = {
	.notifier_call = ibus_notifier,
};

static int __init ibus_init(void)
{
	int err = 0;

	memset(&ibus_rx_alldev_list, 0, sizeof(ibus_rx_alldev_list));

	rcv_cache = kmem_cache_create("ibus_receiver", sizeof(struct receiver),
				      0, 0, NULL);
	if (!rcv_cache)
		return -ENOMEM;

	/* protocol register */
	err = proto_register(&ibus_proto, 0);
	if (err != 0) {
		kmem_cache_destroy(rcv_cache);
		printk(KERN_ERR "ibus: registration protocol failed\n");
		return err;
	}

	sock_register(&ibus_family_ops);
	register_netdevice_notifier(&ibus_netdev_notifier);
	dev_add_pack(&ibus_packet);

	printk(KERN_INFO "module loaded\n");
	return 0;
}

static void __exit ibus_exit(void)
{
	struct net_device *dev;

	/* protocol unregister */
	dev_remove_pack(&ibus_packet);
	unregister_netdevice_notifier(&ibus_netdev_notifier);
	sock_unregister(PF_IBUS);
	proto_unregister(&ibus_proto);

	/* remove created dev_rcv_lists from still registered ibus devices */
	rcu_read_lock();
	for_each_netdev_rcu(&init_net, dev) {
		if (dev->type == ARPHRD_IBUS && dev->ml_priv) {

			struct dev_rcv_lists *d = dev->ml_priv;

			BUG_ON(d->entries);
			kfree(d);
			dev->ml_priv = NULL;
		}
	}
	rcu_read_unlock();

	rcu_barrier(); /* Wait for completion of call_rcu()'s */

	kmem_cache_destroy(rcv_cache);

	printk(KERN_INFO "module unloaded\n");
}

module_init(ibus_init);
module_exit(ibus_exit);
