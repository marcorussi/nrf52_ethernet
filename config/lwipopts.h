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


#define NO_SYS                          0

#define MEM_LIBC_MALLOC                 1

#define MEMP_MEM_MALLOC                 1

#define MEM_ALIGNMENT                   4

#define IP_FORWARD                      1

#define LWIP_RAW                        1

#define LWIP_DHCP                       1

#define LWIP_DHCP_GET_NTP_SRV           1

#define SNTP_SERVER_DNS                 1

#define SNTP_SERVER_ADDRESS             "pool.ntp.org"

#define LWIP_DNS                        1

#define LWIP_IGMP                       1

#define LWIP_NUM_NETIF_CLIENT_DATA      1

#define LWIP_SOCKET_SELECT              0

#define LWIP_NETIF_HOSTNAME             1

#define LWIP_NETIF_STATUS_CALLBACK      1

#define TCPIP_THREAD_NAME               "lwip"

#define TCPIP_THREAD_STACKSIZE          1024

#define TCPIP_THREAD_PRIO               4

#define TCPIP_MBOX_SIZE                 8

#define LWIP_MDNS_RESPONDER             1

#define LWIP_DEBUG                      LWIP_DBG_OFF

