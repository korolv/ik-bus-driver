/*
 * slibus.c - serial communication bus interface driver (using tty line 
 * discipline). Information and Body bus (I and K bus) are network to 
 * interconnect control units in BMW vehicles.
 *
 * This file is derived from drivers/net/can/slcan.c
 * slcan.c Author: Oliver Hartkopp <socketcan@hartkopp.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307. You can also get it
 * at http://www.gnu.org/licenses/gpl.html
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 * Idea:       Oliver Hartkopp <oliver.hartkopp@volkswagen.de>
 * Copyright:  (c) 2015 Vladimir Korol
 * Author:     Vladimir Korol <vovabox@mail.ru>
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/uaccess.h>
#include <linux/bitops.h>
#include <linux/string.h>
#include <linux/tty.h>
#include <linux/errno.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/rtnetlink.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/hrtimer.h>
#include <linux/ibus.h>

static __initdata const char banner[] =
	KERN_INFO "slibus: I/K serial communications bus driver\n";

MODULE_DESCRIPTION("serial line I/K bus interface");

#define SLIBUS_MAGIC           0x53CA
#define IBUS_BUADRATE           9600
#define IBUS_RX_BUFFER_SIZE     512
#define IBUS_TX_BUFFER_SIZE     512
#define IBUS_TX_TRY             1
#define IBUS_GP_CHAR_TIMEOUT    21
#define IBUS_TX_CHAR_TIMEOUT    51
#define IBUS_RX_CHAR_TIMEOUT    47

static int maxdev = 10;    /* MAX number of SLLIN channels;
				   This can be overridden with
				   insmod slibus.ko maxdev=nnn	*/
static int ibus_tx_try = IBUS_TX_TRY; /* default number of try retransmit */
module_param(maxdev, int, 0);
MODULE_PARM_DESC(maxdev, "Maximum number of I/K bus interfaces");
module_param(ibus_tx_try, int, 0);
MODULE_PARM_DESC(ibus_tx_try, "Baudrate of LIN interface");

struct slibus {
	int         magic;

	/* Various fields. */
	struct tty_struct   *tty;      /* ptr to TTY structure	     */
	struct net_device   *dev;      /* easy for intr handling    */
	spinlock_t          lock;
	struct work_struct  tx_work;   /* Flushes transmit buffer   */

	/* These are pointers to the malloc()ed frame buffers. */
	uint8_t     rbuff[IBUS_RX_BUFFER_SIZE]; /* receiver buffer */
	int         rcount;         /* received chars counter    */
	uint8_t     *rhead;         /* pointer to receiving frame */
	uint8_t     xbuff[IBUS_TX_BUFFER_SIZE]; /* transmitter buffer */
	uint8_t     *xhead;         /* pointer to next XMIT byte */
	int         xleft;          /* bytes left in XMIT queue  */
	int         xtry;           /* number of try retransmit */

	unsigned long       flags;     /* Flag values/ mode etc     */
	
	struct timespec     fstart;
#define SLF_INUSE         0     /* Channel in use */
#define SLF_ERROR         1     /* Parity, etc. error */
#define SLF_RECEIVING     2     /* Receive state */
#define SLF_CHK_TX        3     /* Acknowledgement */
#define SLF_BUS_BUSY      4     /* Guard period */
	
	struct hrtimer      rx_timer;          /* RX timeout timer */
	struct hrtimer      tx_timer;          /* RX timeout timer */
	struct hrtimer      gp_timer;          /* Guard period timer */
	unsigned long       gp_timer_timeout;  /* Guard period timer value */

	wait_queue_head_t   tx_wq;             /* Waiting release ibus */
};

static struct net_device **slibus_devs;
static u64 ibus_char_period;

/* Start timer for different timeout */
static inline void sli_timer_start(struct hrtimer *timer, 
				unsigned long chars)
{
	ktime_t ktimeout = ns_to_ktime(ibus_char_period * chars);
	hrtimer_start(timer, ktime_add(ktime_get(), ktimeout),
				HRTIMER_MODE_ABS);
}

/* Transmission acknowledgement */
static void sli_chk_transmission(struct slibus *sl)
{
	if (!memcmp(sl->rhead, sl->xbuff, sl->rcount)) {
	/* OK, positive acknowledgment */
		netif_wake_queue(sl->dev);
		hrtimer_cancel(&sl->tx_timer);
		clear_bit(SLF_CHK_TX, &sl->flags);
	}
}

/* Send one completely frame to the network layer */
static void sli_bump(struct slibus *sl)
{
	struct net_device *dev = sl->dev;
	struct sk_buff *skb;
	int count;

	sli_timer_start(&sl->gp_timer, IBUS_GP_CHAR_TIMEOUT);

	if (test_bit(SLF_CHK_TX, &sl->flags))
		sli_chk_transmission(sl);

	clear_bit(SLF_ERROR, &sl->flags);

	count = sl->rcount;
	
	skb = dev_alloc_skb(count);
	if (skb == NULL) {
		printk(KERN_WARNING "%s: memory squeeze, dropping packet.\n", dev->name);
		dev->stats.rx_dropped++;
		return;
	}
	skb->dev = dev;
	skb->protocol = htons(ETH_P_IBUS);
	skb->pkt_type = PACKET_BROADCAST;
	skb->ip_summed = CHECKSUM_UNNECESSARY;
	memcpy(skb_put(skb, count), sl->rhead, count);
	netif_rx_ni(skb);

	dev->stats.rx_packets++;
	dev->stats.rx_bytes += sl->rhead[1] - 2;
}
/* Configure port with right settings */
static int sltty_configure_port(struct tty_struct *tty, unsigned speed)
{
	struct ktermios old_termios;
	int cflag;
	int ret = 0;

	down_write(&tty->termios_rwsem);

	old_termios = tty->termios;

	cflag = CS8 | CREAD | CLOCAL | HUPCL | PARENB;// | CRTSCTS;
	cflag &= ~(CBAUD | CIBAUD);
	cflag |= BOTHER;
	tty->termios.c_cflag = cflag;
	tty->termios.c_oflag = 0;
	tty->termios.c_lflag = 0;

	/* Enable interrupt when UART-Break or Framing error received */
	tty->termios.c_iflag = BRKINT | INPCK;

	tty_encode_baud_rate(tty, speed, speed);

	if (tty->ops->set_termios)
		tty->ops->set_termios(tty, &old_termios);

	up_write(&tty->termios_rwsem);

	return ret;
}
/* Calculate and set check sum of the frame */
static int slibus_err_checksum(uint8_t *buffer)
{
	unsigned i;
	unsigned len;
	uint8_t checksum;
	
	if(buffer == NULL)
		return -1;

	checksum = 0;
	len = buffer[1] + 1;

	for(i = 0; i < len; i++)
		checksum ^= buffer[i];
	
	if (checksum == buffer[len])
		return 0;
	else {
		buffer[len] = checksum;
		return 1;
	}
}

/* Write out any remaining transmit buffer. Scheduled when tty is writable */
static void slibus_transmit(struct work_struct *work)
{
	struct slibus *sl = container_of(work, struct slibus, tx_work);
	int actual;
	spin_lock_bh(&sl->lock);
	/* First make sure we're connected. */
	if (!sl->tty || sl->magic != SLIBUS_MAGIC || !netif_running(sl->dev)) {
		spin_unlock_bh(&sl->lock);
		return;
	}
	spin_unlock_bh(&sl->lock);

	if (sl->xleft <= 0) {
		/* Now serial buffer is almost free & we can start
		 * transmission of another packet after acknowledgement
		 * or timeout */
		clear_bit(TTY_DO_WRITE_WAKEUP, &sl->tty->flags);
		set_bit(SLF_CHK_TX, &sl->flags);
		return;
	}
	wait_event_killable(sl->tx_wq, !test_bit(SLF_BUS_BUSY, &sl->flags));
	spin_lock_bh(&sl->lock);
	actual = sl->tty->ops->write(sl->tty, sl->xhead, sl->xleft);
	sl->xleft -= actual;
	sl->xhead += actual;
	spin_unlock_bh(&sl->lock);
}
/*
 * Called by the driver when there's room for more data.  If we have
 * more packets to send, we send them here.
 */
static void slibus_write_wakeup(struct tty_struct *tty)
{
	struct slibus *sl = tty->disc_data;

	schedule_work(&sl->tx_work);
}

static void slibus_start_tx(struct slibus *sl)
{
	sl->xhead = sl->xbuff;
	sl->xleft = sl->xbuff[1] + 2;
	sli_timer_start(&sl->tx_timer, sl->xleft + IBUS_TX_CHAR_TIMEOUT);
	set_bit(TTY_DO_WRITE_WAKEUP, &sl->tty->flags);
	schedule_work(&sl->tx_work);
}

/**
 * sli_xmit() -- Send a frame to a TTY queue.
 *
 * @skb: Pointer to Socket buffer to be sent.
 * @dev: Network device where @skb will be sent.
 */
static netdev_tx_t sli_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct slibus *sl = netdev_priv(dev);

	if (unlikely((skb->len < 4) || (skb->len > IBUS_TX_BUFFER_SIZE -1)))
		goto out;

	spin_lock(&sl->lock);
	if (!netif_running(dev)) {
		spin_unlock(&sl->lock);
		printk(KERN_WARNING "%s: xmit: iface is down\n", dev->name);
		goto out;
	}
	if (sl->tty == NULL) {
		spin_unlock(&sl->lock);
		goto out;
	}

	netif_stop_queue(sl->dev);
	memcpy(sl->xbuff, skb->data, skb->len);
	
	if (slibus_err_checksum(sl->xbuff) >= 0) {
		sl->xtry = ibus_tx_try;
		slibus_start_tx(sl);
		sl->dev->stats.tx_packets++;
		sl->dev->stats.tx_bytes += sl->xbuff[1] - 2;
	}
	spin_unlock(&sl->lock);

out:
	kfree_skb(skb);
	return NETDEV_TX_OK;
}

/* Netdevice UP -> DOWN routine */
static int sli_close(struct net_device *dev)
{
	struct slibus *sl = netdev_priv(dev);

	spin_lock_bh(&sl->lock);
	if (sl->tty) {
		/* TTY discipline is running. */
		clear_bit(TTY_DO_WRITE_WAKEUP, &sl->tty->flags);
	}
	netif_stop_queue(dev);
	sl->rcount   = 0;
	sl->rhead    = sl->rbuff;
	sl->xleft    = 0;
	spin_unlock_bh(&sl->lock);

	return 0;
}

/* Netdevice DOWN -> UP routine */
static int sli_open(struct net_device *dev)
{
	struct slibus *sl = netdev_priv(dev);

	if (sl->tty == NULL)
		return -ENODEV;

	sl->flags &= (1 << SLF_INUSE);
	netif_start_queue(dev);
	return 0;
}

/* Hook the destructor so we can free slibus devs at the right point in time */
static void sli_free_netdev(struct net_device *dev)
{
	int i = dev->base_addr;
	free_netdev(dev);
	slibus_devs[i] = NULL;
}

static const struct net_device_ops sli_netdev_ops = {
	.ndo_open	= sli_open,
	.ndo_stop	= sli_close,
	.ndo_start_xmit	= sli_xmit,
};

static void sli_setup(struct net_device *dev)
{
	dev->netdev_ops		= &sli_netdev_ops;
	dev->destructor		= sli_free_netdev;

	dev->hard_header_len	= 0;
	dev->addr_len		= 0;
	dev->tx_queue_len	= 10;

	dev->mtu			= IBUS_MAX_DLEN + 2;
	dev->type			= ARPHRD_IBUS;

	/* New-style flags. */
	dev->flags			= IFF_NOARP;
	dev->features		= NETIF_F_HW_CSUM; /* NETIF_F_NO_CSUM;*/
}
/*
 * Handle the 'receiver data ready' interrupt.
 * This function is called by the 'tty_io' module in the kernel when
 * a block of I/K bus data has been received.
 */
static void slibus_receive_buf(struct tty_struct *tty,
							const uint8_t *cp, char *fp, int count)
{
	int len = 0;
	struct slibus *sl = (struct slibus *) tty->disc_data;

	if (!sl || sl->magic != SLIBUS_MAGIC || !netif_running(sl->dev))
		return;

	/* Check place in rx buffer */
	if (IBUS_RX_BUFFER_SIZE < count + sl->rcount + sl->rhead - sl->rbuff) {
		sl->dev->stats.rx_dropped++;
		sl->rcount = 0;
		sl->rhead = sl->rbuff;
		sl->tty->ops->flush_chars(sl->tty);
		return;
	}
	/* I/K bus is busy */
	set_bit(SLF_BUS_BUSY, &sl->flags);

	/* Read the characters out of the buffer */
	while (count--) {
		if (fp && *fp++) { /* Ignore symbol */
			cp++;
			continue;
		}
		sl->rhead[sl->rcount++] = *cp++;
	}
	/* There is length of the frame */
	if (sl->rcount > 1) len = sl->rhead[1] + 2;

	/* Incorrect frame length */
	if (len > IBUS_MAX_FRAME_SIZE) {
		sl->dev->stats.rx_dropped++;
		sl->rcount = 0;
		sl->rhead = sl->rbuff;
		sl->tty->ops->flush_chars(sl->tty);
		clear_bit(SLF_BUS_BUSY, &sl->flags);
		clear_bit(SLF_RECEIVING, &sl->flags);
		wake_up(&sl->tx_wq);
		return;
	}

	if (len && (sl->rcount >= len)) { /* There is a completely frame */

		if (test_and_clear_bit(SLF_RECEIVING, &sl->flags))
			hrtimer_cancel(&sl->rx_timer);

		if (slibus_err_checksum(sl->rhead)) {
			if (!test_and_set_bit(SLF_ERROR, &sl->flags))
				sl->dev->stats.rx_errors++;

			sl->rhead = sl->rbuff;
			sl->rcount = 0;
			clear_bit(SLF_BUS_BUSY, &sl->flags);
			clear_bit(SLF_RECEIVING, &sl->flags);
			wake_up(&sl->tx_wq);
		}
		else {
			int next_len;
			if (sl->rcount > len) { /* more frames */
				next_len = sl->rcount - len;
				sl->rcount = len;
				sli_bump(sl);
				sl->rhead += len;
				sl->rcount = next_len;
				goto receiving;
			}
			else { /* only one completely frame */
				sli_bump(sl);
				sl->rhead = sl->rbuff;
				sl->rcount = 0;
			}

		}
		return;
	}

receiving:
	/* Incomplete frame, wait the rest */
	set_bit(SLF_RECEIVING, &sl->flags);
	sli_timer_start(&sl->rx_timer, IBUS_RX_CHAR_TIMEOUT); /*wait between parts of rx frame*/

}

static enum hrtimer_restart slibus_rx_timeout_handler(struct hrtimer *hrtimer)
{
	struct slibus *sl = container_of(hrtimer, struct slibus, rx_timer);

	clear_bit(SLF_RECEIVING, &sl->flags);
	clear_bit(SLF_BUS_BUSY, &sl->flags);
	sl->dev->stats.rx_dropped++;
	sl->rcount = 0;
	sl->rhead = sl->rbuff;
	wake_up(&sl->tx_wq);
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart slibus_tx_timeout_handler(struct hrtimer *hrtimer)
{
	struct slibus *sl = container_of(hrtimer, struct slibus, tx_timer);

	clear_bit(SLF_CHK_TX, &sl->flags);

	if (test_and_clear_bit(SLF_ERROR, &sl->flags))
			sl->dev->stats.collisions++;

	if (sl->xtry--) {
		/* retransmit */
		slibus_start_tx(sl);
	}
	else {
		/* negative acknowledgement */
		netif_wake_queue(sl->dev);
		sl->dev->stats.tx_errors++;
	}
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart slibus_gp_timeout_handler(struct hrtimer *hrtimer)
{
	struct slibus *sl = container_of(hrtimer, struct slibus, gp_timer);

	clear_bit(SLF_BUS_BUSY, &sl->flags);
	wake_up(&sl->tx_wq);

	return HRTIMER_NORESTART;
}

/* Collect hanged up channels */
static void sli_sync(void)
{
	int i;
	struct net_device *dev;
	struct slibus *sl;

	for (i = 0; i < maxdev; i++) {
		dev = slibus_devs[i];
		if (dev == NULL)
			break;

		sl = netdev_priv(dev);
		if (sl->tty)
			continue;
		if (dev->flags & IFF_UP)
			dev_close(dev);
	}
}

/* Find a free SLLIN channel, and link in this `tty' line. */
static struct slibus *sli_alloc(dev_t line)
{
	int i;
	char name[IFNAMSIZ];
	struct net_device *dev = NULL;
	struct slibus *sl;

	for (i = 0; i < maxdev; i++) {
		dev = slibus_devs[i];
		if (dev == NULL)
			break;
	}

	/* Sorry, too many, all slots in use */
	if (i >= maxdev)
		return NULL;

	sprintf(name, "ibus%d", i);
	dev = alloc_netdev(sizeof(*sl), name, NET_NAME_UNKNOWN, sli_setup);
	if (!dev)
		return NULL;

	dev->base_addr  = i;
	sl = netdev_priv(dev);

	/* Initialize channel control data */
	sl->magic = SLIBUS_MAGIC;
	sl->dev	= dev;
	spin_lock_init(&sl->lock);
	INIT_WORK(&sl->tx_work, slibus_transmit);
	slibus_devs[i] = dev;

	return sl;
}

/*
 * Open the high-level part of the SLIBUS channel.
 * This function is called by the TTY module when the
 * SLLIN line discipline is called for.  Because we are
 * sure the tty line exists, we only have to link it to
 * a free SLIBUS channel...
 *
 * Called in process context serialized from other ldisc calls.
 */

static int slibus_open(struct tty_struct *tty)
{
	struct slibus *sl;
	int err;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	if (tty->ops->write == NULL)
		return -EOPNOTSUPP;

	/* RTnetlink lock is misused here to serialize concurrent
	   opens of slip channels. There are better ways, but it is
	   the simplest one.
	 */
	rtnl_lock();

	/* Collect hanged up channels. */
	sli_sync();

	sl = tty->disc_data;

	err = -EEXIST;
	/* First make sure we're not already connected. */
	if (sl && sl->magic == SLIBUS_MAGIC)
		goto err_exit;

	/* OK.  Find a free SLIBUS channel to use. */
	err = -ENFILE;
	sl = sli_alloc(tty_devnum(tty));
	if (sl == NULL)
		goto err_exit;

	sl->tty = tty;
	tty->disc_data = sl;

	if (!test_bit(SLF_INUSE, &sl->flags)) {
		/* Perform the low-level SLIBUS initialization. */
		sl->rcount = 0;
		sl->rhead = sl->rbuff;
		sl->xleft = 0;
		sl->xtry = ibus_tx_try;

		set_bit(SLF_INUSE, &sl->flags);
		clear_bit(SLF_RECEIVING, &sl->flags);
		sltty_configure_port(tty, IBUS_BUADRATE);

		/* nanoseconds per character = bits_per_char / buadrate */
		ibus_char_period = (u64)1000000000 * 11 / IBUS_BUADRATE;

		/* Set up receiver timeout */
		hrtimer_init(&sl->rx_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		sl->rx_timer.function = slibus_rx_timeout_handler;

		/* Set up transmit timeout */
		hrtimer_init(&sl->tx_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		sl->tx_timer.function = slibus_tx_timeout_handler;
		
		/* Guard period timer*/
		hrtimer_init(&sl->gp_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		sl->gp_timer.function = slibus_gp_timeout_handler;

		init_waitqueue_head(&sl->tx_wq);
		clear_bit(SLF_BUS_BUSY, &sl->flags);

		err = register_netdevice(sl->dev);
		if (err)
			goto err_free_chan;
	}

	/* Done.  We have linked the TTY line to a channel. */
	rtnl_unlock();
	tty->receive_room = 65536;	/* We don't flow control */

	/* TTY layer expects 0 on success */
	return 0;

err_free_chan:
	sl->tty = NULL;
	tty->disc_data = NULL;
	clear_bit(SLF_INUSE, &sl->flags);

err_exit:
	rtnl_unlock();

	/* Count references from TTY module */
	return err;
}

/*
 * Close down a SLLIN channel.
 * This means flushing out any pending queues, and then returning. This
 * call is serialized against other ldisc functions.
 *
 * We also use this method for a hangup event.
 */

static void slibus_close(struct tty_struct *tty)
{
	struct slibus *sl = (struct slibus *) tty->disc_data;

	/* First make sure we're connected. */
	if (!sl || sl->magic != SLIBUS_MAGIC || sl->tty != tty)
		return;

	spin_lock_bh(&sl->lock);
	tty->disc_data = NULL;
	sl->tty = NULL;
	spin_unlock_bh(&sl->lock);
	
	flush_work(&sl->tx_work);

	/* Flush network side */
	unregister_netdev(sl->dev);
	/* This will complete via sl_free_netdev */
}

static int slibus_hangup(struct tty_struct *tty)
{
	slibus_close(tty);
	return 0;
}

/* Perform I/O control on an active SLIBUS channel. */
static int slibus_ioctl(struct tty_struct *tty, struct file *file,
		       unsigned int cmd, unsigned long arg)
{
	struct slibus *sl = (struct slibus *) tty->disc_data;
	unsigned int tmp;

	/* First make sure we're connected. */
	if (!sl || sl->magic != SLIBUS_MAGIC)
		return -EINVAL;

	switch (cmd) {
	case SIOCGIFNAME:
		tmp = strlen(sl->dev->name) + 1;
		if (copy_to_user((void __user *)arg, sl->dev->name, tmp))
			return -EFAULT;
		return 0;

	case SIOCSIFHWADDR:
		return -EINVAL;

	default:
		return tty_mode_ioctl(tty, file, cmd, arg);
	}
}

static struct tty_ldisc_ops sli_ldisc = {
	.owner          = THIS_MODULE,
	.magic          = TTY_LDISC_MAGIC,
	.name           = "slibus",
	.open           = slibus_open,
	.close          = slibus_close,
	.hangup         = slibus_hangup,
	.ioctl          = slibus_ioctl,
	.receive_buf    = slibus_receive_buf,
	.write_wakeup   = slibus_write_wakeup,
};

static int __init slibus_init(void)
{
	int status;

	if (maxdev < 4)
		maxdev = 4; /* Sanity */

	printk(banner);
	printk(KERN_INFO "slibus: %d dynamic interface channels.\n", maxdev);

	slibus_devs = kzalloc(sizeof(struct net_device *)*maxdev, GFP_KERNEL);
	if (!slibus_devs) {
		printk(KERN_ERR "slibus: can't allocate slibus device array!\n");
		return -ENOMEM;
	}

	/* Fill in our line protocol discipline, and register it */
	status = tty_register_ldisc(N_SLIBUS, &sli_ldisc);
	if (status)  {
		printk(KERN_ERR "slibus: can't register line discipline\n");
		kfree(slibus_devs);
	}
	return status;
}

static void __exit slibus_exit(void)
{
	int i;
	struct net_device *dev;
	struct slibus *sl;
	unsigned long timeout = jiffies + HZ;
	int busy = 0;

	if (slibus_devs == NULL)
		return;

	/* First of all: check for active disciplines and hangup them.
	 */
	do {
		if (busy)
			msleep_interruptible(100);

		busy = 0;
		for (i = 0; i < maxdev; i++) {
			dev = slibus_devs[i];
			if (!dev)
				continue;
			sl = netdev_priv(dev);
			spin_lock_bh(&sl->lock);
			if (sl->tty) {
				busy++;
				tty_hangup(sl->tty);
			}
			spin_unlock_bh(&sl->lock);
		}
	} while (busy && time_before(jiffies, timeout));

	/* FIXME: hangup is async so we should wait when doing this second
	   phase */

	for (i = 0; i < maxdev; i++) {
		dev = slibus_devs[i];
		if (!dev)
			continue;
		slibus_devs[i] = NULL;

		sl = netdev_priv(dev);
		if (sl->tty) {
			netdev_dbg(sl->dev, "tty discipline still running\n");
			/* Intentionally leak the control block. */
			dev->destructor = NULL;
		}

		unregister_netdev(dev);
	}

	kfree(slibus_devs);
	slibus_devs = NULL;

	i = tty_unregister_ldisc(N_SLIBUS);
	if (i)
		pr_err("slibus: can't unregister ldisc (err %d)\n", i);
}

module_init(slibus_init);
module_exit(slibus_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Vladimir Korol <vovabox@mail.ru>");
