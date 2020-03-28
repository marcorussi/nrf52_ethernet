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


#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>

#include "lwip/api.h"
#include "lwip/autoip.h"
#include "lwip/debug.h"
#include "lwip/def.h"
#include "lwip/dhcp.h"
#include "lwip/dns.h"
#include "lwip/etharp.h"
#include "lwip/init.h"
#include "lwip/mem.h"
#include "lwip/netif.h"
#include "lwip/opt.h"
#include "lwip/pbuf.h"
#include "lwip/prot/dhcp.h"
#include "lwip/snmp.h"
#include "lwip/stats.h"
#include "lwip/sys.h"
#include "lwip/tcp.h"
#include "lwip/tcpip.h"
#include "lwip/timeouts.h"
#include "lwip/udp.h"
#include "lwip/apps/mdns.h"
#include "lwip/apps/sntp.h"

#include "FreeRTOS.h"
#include "task.h"

#include "eth_if.h"
#include "net.h"


#define MDNS_LOCAL_NAME                 "nrf52_eth"
#define MDNS_TTL_SEC                    79800

#define DNS_SERV_IP_ADDR_1              8
#define DNS_SERV_IP_ADDR_2              8
#define DNS_SERV_IP_ADDR_3              8
#define DNS_SERV_IP_ADDR_4              8


typedef enum
{
  WAIT_LINK_UP,
  LINK_UP,
  LINK_DOWN,
  NET_ERROR
} net_states_e;


static struct netif eth_if;

static struct dhcp eth_netif_dhcp;

static uint8_t state = 0;

static bool ready = false;


static void ethStatusCallback( struct netif *state_netif );
static void initDnsServ( void *arg );


void NET_task( void *arg )
{
  bool done = false;
  uint32_t period_ms = (uint32_t)arg;
  ip4_addr_t ipaddr, netmask, gw;
  struct dhcp *dhcp = &eth_netif_dhcp;
  static struct netif *netif = &eth_if;

  state = 0;
  ready = false;

  tcpip_init(initDnsServ, NULL);

  ip4_addr_set_zero(&gw);
  ip4_addr_set_zero(&ipaddr);
  ip4_addr_set_zero(&netmask);

  netif_add(netif, &ipaddr, &netmask, &gw, NULL, ETH_init, tcpip_input);

  netif_set_default(netif);

  netif_set_status_callback(netif, ethStatusCallback);

  dhcp_set_struct(netif, dhcp);

  mdns_resp_init();

  while(true != done)
  {
    switch(state)
    {
      case WAIT_LINK_UP:
        if (1 == netif_is_link_up(netif))
        {
          netif_set_up(netif);

          /* Set MDNS local name */
          (void)mdns_resp_add_netif(netif, (const char *)MDNS_LOCAL_NAME, MDNS_TTL_SEC);

          /* Start DHCP discovery */
          if (ERR_OK != dhcp_start(netif))
          {
            /* error: finish this task */
            done = true;
          }

          state = LINK_UP;
        }
        break;

      case LINK_UP:
        if( 1 != netif_is_link_up(netif) )
        {
          netif_set_down(netif);

          state = LINK_DOWN;
        }
        break;

      case LINK_DOWN:
        if( 1 == netif_is_link_up(netif) )
        {
          netif_set_up(netif);

          state = LINK_UP;
        }
        break;
    }

    if (true == ready)
    {
      ready = false;

      sntp_init();
    }

    vTaskDelay(pdMS_TO_TICKS(period_ms));
  }

  vTaskDelete(NULL);
}


static void ethStatusCallback( struct netif *netif )
{
  struct dhcp *dhcp;

  /* Get DHCP data and check if bound */
  dhcp = netif_dhcp_data(netif);
  if (DHCP_STATE_BOUND == dhcp->state)
  {
    mdns_resp_netif_settings_changed(netif);

    ready = true;
  }
}


static void initDnsServ( void *arg )
{
  (void)arg;
  uint8_t i;
  const ip_addr_t dns_server_addr = IPADDR4_INIT_BYTES(
      DNS_SERV_IP_ADDR_1,
      DNS_SERV_IP_ADDR_2,
      DNS_SERV_IP_ADDR_3,
      DNS_SERV_IP_ADDR_4);

  for( i = 0; i < DNS_MAX_SERVERS; i++ )
  {
    dns_setserver(i, &dns_server_addr);
  }
}

