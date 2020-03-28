/*
* The MIT License (MIT)
*
* Copyright (c) 2020 Marco Russi
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/


#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/stats.h"
#include "lwip/snmp.h"
#include "lwip/etharp.h"
#include "netif/ethernet.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "nrf_drv_gpiote.h"
#include "app_util_platform.h"
#include "nrf_delay.h"
#include "app_error.h"

#include "ksz8851_reg.h"
#include "ksz8851_spi.h"
#include "eth_if.h"


#define IFNAME0								        'e'
#define IFNAME1								        'n'

#define NET_MTU								        1500

#define NETIF_TX_BUFFERS              4
#define NETIF_RX_BUFFERS              4

#define KSZ8851_INTN_PIN_NUM          5

#define KSZ8851_THREAD_STACK_SIZE     256
#define KSZ8851_THREAD_PRIO           5
#define KSZ8851_THREAD_PERIOD_MS      100


typedef enum
{
  MICREL_OWNER,
  SOFTWARE_OWNER
} buf_owners_e;


typedef struct
{
  uint8_t owner;
  struct pbuf *buf;
} buffer_st;


typedef struct
{
  buffer_st rx_buf[NETIF_RX_BUFFERS];
  buffer_st tx_buf[NETIF_TX_BUFFERS];
	uint8_t tx_buf_cur;
	uint8_t rx_head;
	uint8_t rx_tail;
	uint8_t tx_head;
	uint8_t tx_tail;
	struct netif *netif;
	uint8_t int_flag;
} device_st;


/* KSZ8851 driver instance */
static device_st device;

static uint16_t pending_frame = 0;

/* MAC address to use */
static uint8_t gs_uc_mac_address[] =
{
	0x00,
	0x11,
	0x22,
	0x33,
	0x44,
	0x77
};


static void configureIntPin( void );
static void inPinHandler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
static void rxPopulateQueue(device_st *d);
static void updateTxAndRx(struct netif *netif);
static void rxInit( device_st *d );
static void txInit( device_st *d );
static void lowLevelInit(struct netif *netif);
static err_t lowLevelOutput(struct netif *netif, struct pbuf *p);
static struct pbuf *lowLevelInput(struct netif *netif);
static void thread( void *arg );


err_t ETH_init(struct netif *netif)
{
  sys_thread_t id;

	LWIP_ASSERT("netif != NULL", (netif != NULL));

	device.netif = netif;

#if LWIP_NETIF_HOSTNAME
	netif->hostname = "nrf52_eth_6lowpan";
#endif /* LWIP_NETIF_HOSTNAME */

	netif->state = &device;
	netif->name[0] = IFNAME0;
	netif->name[1] = IFNAME1;

	/* We directly use etharp_output() here to save a function call.
	 * You can instead declare your own function an call etharp_output()
	 * from it if you have to do some checks before sending (e.g. if link
	 * is available...) */
	netif->output = etharp_output;
	netif->linkoutput = lowLevelOutput;

	lowLevelInit(netif);

	id = sys_thread_new(
	    "ksz8851", thread, &device, KSZ8851_THREAD_STACK_SIZE, KSZ8851_THREAD_PRIO);
	LWIP_ASSERT("ethernetif_init: ksz8851snl Task allocation ERROR!\n",
			(id.thread_handle != 0));
	if (id.thread_handle == 0)
		return ERR_MEM;

	return ERR_OK;
}


static void configureIntPin( void )
{
  nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);

  if (true != nrf_drv_gpiote_is_init())
  {
    APP_ERROR_CHECK(nrf_drv_gpiote_init());
  }

  /* Pull-up resistor enabled */
  in_config.pull = NRF_GPIO_PIN_PULLUP;
  /* Falling edge */
  in_config.sense = NRF_GPIOTE_POLARITY_HITOLO;
  /* Configure pin interrupt */
  APP_ERROR_CHECK(nrf_drv_gpiote_in_init(KSZ8851_INTN_PIN_NUM, &in_config, inPinHandler));

  /* Enable pin interrupt */
  nrf_drv_gpiote_in_event_enable(KSZ8851_INTN_PIN_NUM, true);
}


static void inPinHandler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  (void)pin;
  (void)action;

  /* Set the interrupt flag */
  device.int_flag = 1;
}


static void rxPopulateQueue(device_st *d)
{
  uint8_t i = 0;
  struct pbuf *p = 0;

  /* Set up the RX descriptors */
  for (i = 0; i < NETIF_RX_BUFFERS; i++)
  {
    if (NULL == d->rx_buf[i].buf)
    {
      /* Allocate a new pbuf with the maximum size */
      p = pbuf_alloc(PBUF_RAW, PBUF_POOL_BUFSIZE, PBUF_POOL);
      if (0 == p)
      {
        LWIP_DEBUGF(NETIF_DEBUG,
                    ("rx_populate_queue: pbuf allocation failure\n"));
      }
      /* Make sure lwIP is well configured so one pbuf can contain the maximum packet size */
      LWIP_ASSERT("rx_populate_queue: pbuf size too small!", pbuf_clen(p) <= 1);
      /* Set owner as Micrel */
      d->rx_buf[i].owner = MICREL_OWNER;
      /* Save pbuf pointer to be sent to lwIP upper layer */
      d->rx_buf[i].buf = p;
      LWIP_DEBUGF(NETIF_DEBUG,
                  ("rx_populate_queue: new pbuf allocated with size %d: 0x%p [pos=%d]\n",
                      PBUF_POOL_BUFSIZE, p, i));
    }
  }
}


static void updateTxAndRx(struct netif *netif)
{
  device_st *d = netif->state;
  uint16_t status = 0;
  uint16_t len = 0;
  uint16_t txmir = 0;

  /* Handle TX */
  /* Fetch next packet marked as owned by Micrel */
  if ((MICREL_OWNER == d->tx_buf[d->tx_tail].owner)
  &&  (pending_frame == 0))
  {
    len = d->tx_buf[d->tx_tail].buf->tot_len;

    /* Check if TXQ memory size is available for transmit */
    txmir = ksz8851_regRead(REG_TX_MEM_INFO) & TX_MEM_AVAILABLE_MASK;
    if (txmir < len + 8)
    {
      LWIP_DEBUGF(NETIF_DEBUG,
          ("update: TX not enough memory in queue: %d required %d\n",
          txmir, len + 8));
      return;
    }

    /* Disable all interrupts */
    ksz8851_regWrite(REG_INT_MASK, 0);

    LWIP_DEBUGF(NETIF_DEBUG,
        ("update: TX start packet transmit len=%d [tail=%u head=%u]\n",
        len,
        d->tx_tail, d->tx_head));

    /* Enable TXQ write access */
    ksz8851_regSetbits(REG_RXQ_CMD, RXQ_START);

    /* Perform FIFO write operation */
    ksz8851_fifoWrite(d->tx_buf[d->tx_tail].buf->payload,
                       d->tx_buf[d->tx_tail].buf->tot_len,
                       d->tx_buf[d->tx_tail].buf->len);

    /* Disable TXQ write access */
    ksz8851_regClrbits(REG_RXQ_CMD, RXQ_START);

    /* Enqueue frame in TXQ */
    ksz8851_regSetbits(REG_TXQ_CMD, TXQ_ENQUEUE);

    /* Enable INT_RX flag */
    ksz8851_regWrite(REG_INT_MASK, INT_RX);

    /* Buffer sent, free the corresponding buffer and mark descriptor as owned by software */
    pbuf_free(d->tx_buf[d->tx_tail].buf);
    d->tx_buf[d->tx_tail].buf = NULL;
    d->tx_buf[d->tx_tail].owner = SOFTWARE_OWNER;
    d->tx_tail = (d->tx_tail + 1) % NETIF_TX_BUFFERS;
  }
  /* TODO: consider to remove "else" and so always execute RX after TX */
  /* Handle RX */
  else if (d->int_flag || pending_frame > 0)
  {
    d->int_flag = 0;

    /* Check LINK interrupt status flag */
    status = ksz8851_regRead(REG_INT_STATUS);
    if (status & INT_PHY)
    {
      /* Get link status and notify the interface */
      status = ksz8851_regRead(REG_PHY_STATUS);
      if (status & PHY_LINK_UP)
      {
        netif_set_link_up(netif);
      }
      else
      {
        netif_set_link_down(netif);
      }

      /* Clear interrupt flag */
      ksz8851_regSetbits(REG_INT_STATUS, INT_PHY);
    }

    if (0 == pending_frame)
    {
      /* Read interrupt status for INT_RX flag */
      status = ksz8851_regRead(REG_INT_STATUS);
      if (!(status & INT_RX))
      {
        /* Clear interrupts flags */
        ksz8851_regSetbits(REG_INT_STATUS, status);

        return;
      }

      /* Disable all interrupts */
      ksz8851_regWrite(REG_INT_MASK, 0);

      /* Clear INT_RX flag */
      ksz8851_regSetbits(REG_INT_STATUS, INT_RX);

      /* Check for received frames */
      pending_frame = ksz8851_regRead(REG_RX_FRAME_CNT_THRES) >> 8;
      if (0 == pending_frame)
      {
        /* Enable INT_RX flag */
        ksz8851_regWrite(REG_INT_MASK, INT_RX);
        return;
      }
    }

    /* Don't break Micrel state machine, wait for a free descriptor first! */
    if (SOFTWARE_OWNER == d->rx_buf[d->rx_head].owner)
    {
      LWIP_DEBUGF(NETIF_DEBUG,
                  ("update: out of free descriptor! [tail=%u head=%u]\n",
                      d->rx_tail, d->rx_head));
      return;
    }

    if (NULL == d->rx_buf[d->rx_head].buf)
    {
      rxPopulateQueue(d);
      LWIP_DEBUGF(NETIF_DEBUG,
                  ("update: descriptor with NULL pbuf! [head=%u]\n",
                      d->rx_head));
      return;
    }

    /* Get RX packet status */
    status = ksz8851_regRead(REG_RX_FHR_STATUS);
    if (((status & RX_VALID) == 0) || (status & RX_ERRORS))
    {
      ksz8851_regSetbits(REG_RXQ_CMD, RXQ_CMD_FREE_PACKET);
      LWIP_DEBUGF(NETIF_DEBUG, ("update: RX packet error!\n"));
    }
    else
    {
      /* Read frame length */
      len = ksz8851_regRead(REG_RX_FHR_BYTE_CNT) & RX_BYTE_CNT_MASK;

      /* Drop packet if len is invalid or no descriptor available */
      if (0 == len)
      {
        ksz8851_regSetbits(REG_RXQ_CMD, RXQ_CMD_FREE_PACKET);
        LWIP_DEBUGF(NETIF_DEBUG, ("update: RX bad len!\n"));
      }
      else
      {
        LWIP_DEBUGF(NETIF_DEBUG,
                    ("update: RX start packet receive len=%d [tail=%u head=%u]\n",
                        len,
                        d->rx_tail, d->rx_head));

        /* Reset RX frame pointer */
        ksz8851_regClrbits(REG_RX_ADDR_PTR, ADDR_PTR_MASK);

        /* Start RXQ read access */
        ksz8851_regSetbits(REG_RXQ_CMD, RXQ_START);

        /* Start asynchronous FIFO read operation */
        ksz8851_fifoRead(d->rx_buf[d->rx_head].buf->payload, len);

        /* Remove CRC and update pbuf length */
        len -= 4;
        d->rx_buf[d->rx_head].buf->len = len;
        d->rx_buf[d->rx_head].buf->tot_len = len;

        /* End RXQ read access */
        ksz8851_regClrbits(REG_RXQ_CMD, RXQ_START);

        /* Update frame count to be read */
        pending_frame -= 1;

        /* Enable INT_RX flag if transfer complete */
        if (0 == pending_frame)
        {
          ksz8851_regWrite(REG_INT_MASK, INT_RX);
        }

        /* Mark descriptor ready to be read */
        d->rx_buf[d->rx_head].owner = SOFTWARE_OWNER;
        d->rx_head = (d->rx_head + 1) % NETIF_RX_BUFFERS;
      }
    }
  }
}


static void rxInit( device_st *d )
{
  uint32_t i = 0;

  /* Init pointer index */
  d->rx_head = 0;
  d->rx_tail = 0;

  /* Set up the RX descriptors */
  for (i = 0; i < NETIF_RX_BUFFERS; i++)
  {
    d->rx_buf[i].buf = NULL;
    d->rx_buf[i].owner = MICREL_OWNER;
  }

  /* Build RX buffer and descriptors */
  rxPopulateQueue(d);
}


static void txInit( device_st *d )
{
  uint32_t i = 0;

  /* Init TX index pointer */
  d->tx_head = 0;
  d->tx_tail = 0;

  /* Set up the TX descriptors */
  for (i = 0; i < NETIF_TX_BUFFERS; i++)
  {
    d->tx_buf[i].owner = SOFTWARE_OWNER;
  }
}


static void lowLevelInit(struct netif *netif)
{
  /* Set MAC hardware address length */
  netif->hwaddr_len = sizeof(gs_uc_mac_address);
  /* Set MAC hardware address */
  netif->hwaddr[0] = gs_uc_mac_address[0];
  netif->hwaddr[1] = gs_uc_mac_address[1];
  netif->hwaddr[2] = gs_uc_mac_address[2];
  netif->hwaddr[3] = gs_uc_mac_address[3];
  netif->hwaddr[4] = gs_uc_mac_address[4];
  netif->hwaddr[5] = gs_uc_mac_address[5];

  /* Set maximum transfer unit */
  netif->mtu = NET_MTU;

  /* Device capabilities */
  netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_IGMP;

  rxInit(&device);
  txInit(&device);

  /* Initialize SPI link */
  if (0 != ksz8851_init())
  {
    LWIP_DEBUGF(NETIF_DEBUG,
                ("ksz8851snl_low_level_init: failed to initialize the Micrel driver!\n"));
    LWIP_ASSERT("SPI communication issue", 1);
  }

  configureIntPin();
}


static err_t lowLevelOutput(struct netif *netif, struct pbuf *p)
{
  device_st *d = netif->state;

  /* Make sure the next descriptor is free */
  if (MICREL_OWNER == d->tx_buf[d->tx_head].owner)
  {
    return ERR_IF;
  }

  /* Ensure lwIP won't free this pbuf before the Micrel actually sends it */
  pbuf_ref(p);

  /* Mark descriptor has owned by Micrel. Enqueue pbuf packet */
  d->tx_buf[d->tx_head].owner = MICREL_OWNER;
  d->tx_buf[d->tx_head].buf = p;

  /* Increment head */
  d->tx_head = (d->tx_head + 1) % NETIF_TX_BUFFERS;

  LINK_STATS_INC(link.xmit);

  return ERR_OK;
}


static struct pbuf *lowLevelInput(struct netif *netif)
{
  device_st *d = netif->state;
  struct pbuf *p = 0;

  /* Check that descriptor is owned by software (ie packet received) */
  if (SOFTWARE_OWNER == d->rx_buf[d->rx_tail].owner)
  {
    /* Fetch pre-allocated pbuf */
    p = d->rx_buf[d->rx_tail].buf;

    /* Remove this pbuf from its descriptor */
    d->rx_buf[d->rx_tail].buf = NULL;

    LWIP_DEBUGF(NETIF_DEBUG,
                ("low_level_input: DMA buffer 0x%p received, size=%u [tail=%u head=%u]\n",
                    p->payload, p->tot_len, d->rx_tail, d->rx_head));

    /* Set pbuf total packet size */
    LINK_STATS_INC(link.recv);

    /* Fill empty descriptors with new pbufs */
    rxPopulateQueue(d);

    /* Increment tail */
    d->rx_tail = (d->rx_tail + 1) % NETIF_RX_BUFFERS;
  }

  return p;
}


static void thread( void *arg )
{
  device_st *d = (device_st *)arg;
  struct netif *netif = d->netif;
  struct pbuf *p;

  for ( ; ; )
  {
    /* Update state machine */
    updateTxAndRx(netif);

    /* Move received packet into a new pbuf */
    p = lowLevelInput(netif);
    if (p != NULL)
    {
      /* Send packet to lwIP for processing */
      if (netif->input(p, netif) != ERR_OK)
      {
        LWIP_DEBUGF(NETIF_DEBUG, ("IP input error\n"));
        /* Free buffer */
        pbuf_free(p);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(KSZ8851_THREAD_PERIOD_MS));
  }
}

