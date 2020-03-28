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


#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "nrf_drv_spi.h"
#include "nrfx_spim.h"
#include "nrf_drv_gpiote.h"
#include "app_util_platform.h"
#include "nrf_delay.h"
#include "app_error.h"
#include "ksz8851_reg.h"
#include "ksz8851_spi.h"


#define KSZ8851_RESET_PIN_NUM         1

#define SPI_INSTANCE                  0

#define SET_SPI_CS()                  (nrfx_gpiote_out_set(SPI_SS_PIN))
#define CLEAR_SPI_CS()                (nrfx_gpiote_out_clear(SPI_SS_PIN))


static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);


void ksz8851_fifoRead(uint8_t *buf, uint16_t len)
{
  uint8_t cmdb[11];
  uint8_t pad_bytes;
  uint8_t xfer_len;

  /* calculate number of dummy pad bytes to read a 32-bits aligned buffer */
  pad_bytes = ((len & 0x03) != 0) ? (4 - (len & 0x03)) : 0;

  cmdb[0] = FIFO_READ;

  /* update length to a 32-bits aligned value */
  len += pad_bytes;

  CLEAR_SPI_CS();
  (void)nrf_drv_spi_transfer(&spi, cmdb, 11, NULL, 0);
  /* TODO: Perform non-blocking DMA SPI transfer */
  /* Nordic transfer API function has a limit of 256 bytes of length */
  while (len > 0)
  {
    if (len > 255)
    {
      xfer_len = 255;
    }
    else
    {
      xfer_len = len;
    }
    (void)nrf_drv_spi_transfer(&spi, NULL, 0, buf, xfer_len);
    len -= xfer_len;
    buf += xfer_len;
  }
  SET_SPI_CS();
}


void ksz8851_fifoWrite(uint8_t *buf, uint16_t tot_len, uint16_t len)
{
  (void)tot_len;
  uint8_t pad_bytes;
  static uint8_t frameID = 0;
  uint8_t cmdb[5];
  uint8_t xfer_len;

  /* length is 11 bits long */
  len &= 0x07FF;
  /* calculate number of dummy pad bytes to send a 32-bits aligned buffer */
  pad_bytes = ((len & 0x03) != 0) ? (4 - (len & 0x03)) : 0;

  /* Prepare control word and byte count */
  cmdb[0] = FIFO_WRITE;
  cmdb[1] = frameID++ & 0x3f;
  cmdb[2] = 0;
  cmdb[3] = len & 0xff;
  cmdb[4] = len >> 8;

  /* update length to a 32-bits aligned value */
  len += pad_bytes;

  CLEAR_SPI_CS();
  (void)nrf_drv_spi_transfer(&spi, cmdb, 5, NULL, 0);
  /* TODO: Perform non-blocking DMA SPI transfer ??? */
  /* Nordic transfer API function has a limit of 256 bytes of length */
  while (len > 0)
  {
    if (len > 255)
    {
      xfer_len = 255;
    }
    else
    {
      xfer_len = len;
    }
    (void)nrf_drv_spi_transfer(&spi, buf, xfer_len, NULL, 0);
    len -= xfer_len;
    buf += xfer_len;
  }
  SET_SPI_CS();
}


uint16_t ksz8851_regRead( uint16_t reg )
{
  uint8_t inbuf[4];
  uint8_t outbuf[4];
  uint16_t cmd = 0;
  uint16_t res = 0;

  /* Move register address to cmd bits 9-2, make 32-bit address */
  cmd = (reg << 2) & REG_ADDR_MASK;

  /* Last 2 bits still under "don't care bits" handled with byte enable */
  /* Select byte enable for command */
  if (reg & 2)
  {
    /* Odd word address writes bytes 2 and 3 */
    cmd |= (0xc << 10);
  }
  else
  {
    /* Even word address write bytes 0 and 1 */
    cmd |= (0x3 << 10);
  }

  /* Add command read code. */
  cmd |= CMD_READ;
  outbuf[0] = cmd >> 8;
  outbuf[1] = cmd & 0xff;
  outbuf[2] = 0xff;
  outbuf[3] = 0xff;

  CLEAR_SPI_CS();
  /* Perform blocking SPI transfer. Discard function returned value! TODO: handle it? */
  (void)nrf_drv_spi_transfer(&spi, (uint8_t*)outbuf, 4, (uint8_t *)inbuf, 4);
  SET_SPI_CS();

  res = (inbuf[3] << 8) | inbuf[2];
  return res;
}


void ksz8851_regWrite(uint16_t reg, uint16_t wrdata)
{
  uint8_t outbuf[4];
  uint16_t cmd = 0;

  /* Move register address to cmd bits 9-2, make 32-bit address */
  cmd = (reg << 2) & REG_ADDR_MASK;

  /* Last 2 bits still under "don't care bits" handled with byte enable */
  /* Select byte enable for command */
  if (reg & 2)
  {
    /* Odd word address writes bytes 2 and 3 */
    cmd |= (0xc << 10);
  }
  else
  {
    /* Even word address write bytes 0 and 1 */
    cmd |= (0x3 << 10);
  }

  /* Add command write code */
  cmd |= CMD_WRITE;
  outbuf[0] = cmd >> 8;
  outbuf[1] = cmd & 0xff;
  outbuf[2] = wrdata & 0xff;
  outbuf[3] = wrdata >> 8;

  CLEAR_SPI_CS();
  /* Perform blocking SPI transfer. Discard function returned value! TODO: handle it? */
  (void)nrf_drv_spi_transfer(&spi, (uint8_t*)outbuf, 4, NULL, 0);
  SET_SPI_CS();
}


void ksz8851_regSetbits(uint16_t reg, uint16_t bits_to_set)
{
   uint16_t temp;

   temp = ksz8851_regRead(reg);
   temp |= bits_to_set;
   ksz8851_regWrite(reg, temp);
}


void ksz8851_regClrbits(uint16_t reg, uint16_t bits_to_clr)
{
   uint16_t temp;

   temp = ksz8851_regRead(reg);
   temp &= ~(uint32_t) bits_to_clr;
   ksz8851_regWrite(reg, temp);
}


bool ksz8851_init(void)
{
  uint32_t count = 0;
  uint16_t dev_id = 0;
  bool success = true;
  nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
  nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

  if (true != nrf_drv_gpiote_is_init())
  {
    APP_ERROR_CHECK(nrf_drv_gpiote_init());
  }
  /* Init RESET output pin */
  APP_ERROR_CHECK(nrf_drv_gpiote_out_init(KSZ8851_RESET_PIN_NUM, &out_config));
  nrfx_gpiote_out_set(KSZ8851_RESET_PIN_NUM);
  /* Init SPI SS pin */
  APP_ERROR_CHECK(nrf_drv_gpiote_out_init(SPI_SS_PIN, &out_config));
  SET_SPI_CS();

  spi_config.ss_pin   = NRFX_SPI_PIN_NOT_USED;
  spi_config.miso_pin = SPI_MISO_PIN;
  spi_config.mosi_pin = SPI_MOSI_PIN;
  spi_config.sck_pin  = SPI_SCK_PIN;
  spi_config.frequency = NRF_DRV_SPI_FREQ_1M;
  spi_config.mode = NRF_DRV_SPI_MODE_0;
  spi_config.bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
  /* Initialize the SPI interface */
  if (NRF_SUCCESS == nrf_drv_spi_init(&spi, &spi_config, NULL, NULL))
  {
    /* Reset the Micrel in a proper state */
    do
    {
      /* Reset pulse */
      nrfx_gpiote_out_clear(KSZ8851_RESET_PIN_NUM);
      //vTaskDelay(50);
      nrf_delay_ms(50);
      nrfx_gpiote_out_set(KSZ8851_RESET_PIN_NUM);
      //vTaskDelay(50);
      nrf_delay_ms(50);

      /* Read chip ID. */
      dev_id = ksz8851_regRead(REG_CHIP_ID);
      if (++count > 10)
      {
        return 1;
      }
    } while ((dev_id & 0xFFF0) != CHIP_ID_8851_16);

    /* Write QMU MAC address (low, middle then high) */
    ksz8851_regWrite(REG_MAC_ADDR_0, (0x55 << 8) | 0x66);
    ksz8851_regWrite(REG_MAC_ADDR_2, (0x33 << 8) | 0x44);
    ksz8851_regWrite(REG_MAC_ADDR_4, (0x11 << 8) | 0x22);

    /* Enable QMU Transmit Frame Data Pointer Auto Increment */
    ksz8851_regWrite(REG_TX_ADDR_PTR, ADDR_PTR_AUTO_INC);

    /* Configure QMU transmit control register */
    ksz8851_regWrite(REG_TX_CTRL,
        TX_CTRL_ICMP_CHECKSUM |
        TX_CTRL_UDP_CHECKSUM |
        TX_CTRL_TCP_CHECKSUM |
        TX_CTRL_IP_CHECKSUM |
        TX_CTRL_FLOW_ENABLE |
        TX_CTRL_PAD_ENABLE |
        TX_CTRL_CRC_ENABLE
      );

    /* Enable QMU Receive Frame Data Pointer Auto Increment */
    ksz8851_regWrite(REG_RX_ADDR_PTR, ADDR_PTR_AUTO_INC);

    /* Configure QMU Receive Frame Threshold for one frame */
    ksz8851_regWrite(REG_RX_FRAME_CNT_THRES, 1);

    /* Configure QMU receive control register1 */
    ksz8851_regWrite(REG_RX_CTRL1,
                RX_CTRL_UDP_CHECKSUM |
                RX_CTRL_TCP_CHECKSUM |
                RX_CTRL_IP_CHECKSUM |
                RX_CTRL_MAC_FILTER |
                RX_CTRL_FLOW_ENABLE |
                RX_CTRL_BROADCAST |
                RX_CTRL_ALL_MULTICAST|
                RX_CTRL_UNICAST |
                RX_CTRL_PROMISCUOUS);

    /* Configure QMU receive control register2 */
    ksz8851_regWrite(REG_RX_CTRL2,
                RX_CTRL_IPV6_UDP_NOCHECKSUM |
                RX_CTRL_UDP_LITE_CHECKSUM |
                RX_CTRL_ICMP_CHECKSUM |
                RX_CTRL_BURST_LEN_FRAME);

    /* Configure QMU receive queue: trigger INT and auto-dequeue frame */
    ksz8851_regWrite(REG_RXQ_CMD, RXQ_CMD_CNTL | RXQ_TWOBYTE_OFFSET);

    /* Adjust SPI data output delay */
    ksz8851_regWrite(REG_BUS_CLOCK_CTRL, BUS_CLOCK_166 | BUS_CLOCK_DIVIDEDBY_1);

    /* Restart auto-negotiation */
    ksz8851_regSetbits(REG_PORT_CTRL, PORT_AUTO_NEG_RESTART);

    /* Force link in half duplex if auto-negotiation failed */
    if ((ksz8851_regRead(REG_PORT_CTRL) & PORT_AUTO_NEG_RESTART) != PORT_AUTO_NEG_RESTART)
    {
      ksz8851_regClrbits(REG_PORT_CTRL, PORT_FORCE_FULL_DUPLEX);
    }

    /* Clear interrupt status */
    ksz8851_regWrite(REG_INT_STATUS, 0xFFFF);

    /* Set interrupt mask */
    ksz8851_regWrite(REG_INT_MASK, INT_RX | INT_PHY);

    /* Enable QMU Transmit */
    ksz8851_regSetbits(REG_TX_CTRL, TX_CTRL_ENABLE);

    /* Enable QMU Receive */
    ksz8851_regSetbits(REG_RX_CTRL1, RX_CTRL_ENABLE);
  }
  else
  {
    success = false;
  }

  return success;
}

