/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Raspberry Pi (Trading) Ltd.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <pico/stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "tusb.h"
#include "autobaud.h"

#include "probe_config.h"

#include "RingBuffer.h"
#include "UartInterfaces.h"


TaskHandle_t uart_taskhandle;
TickType_t last_wake;
volatile TickType_t break_expiry;
volatile bool timed_break;

/* Max 1 FIFO worth of data */
// Actually s^-1 so 25ms
#define DEBOUNCE_MS 40
static uint debounce_ticks = 5;

#ifdef PROBE_UART_TX_LED
static volatile uint tx_led_debounce;
#endif

#ifdef PROBE_UART_RX_LED
static uint rx_led_debounce;
#endif

//static BaudInfo_t baud_info;

void cdc_uart_init(void) {
    UART_INTERFACES[0].setup();
    UART_INTERFACES[0].init(PROBE_UART_BAUDRATE);
    UART_INTERFACES[1].setup();
    UART_INTERFACES[1].init(PROBE_UART_BAUDRATE);

#ifdef PROBE_UART_TX_LED
    tx_led_debounce = 0;
    gpio_init(PROBE_UART_TX_LED);
    gpio_set_dir(PROBE_UART_TX_LED, GPIO_OUT);
#endif
#ifdef PROBE_UART_RX_LED
    rx_led_debounce = 0;
    gpio_init(PROBE_UART_RX_LED);
    gpio_set_dir(PROBE_UART_RX_LED, GPIO_OUT);
#endif

#ifdef PROBE_UART_HWFC
    /* HWFC implies that hardware flow control is implemented and the
     * UART operates in "full-duplex" mode (See USB CDC PSTN120 6.3.12).
     * Default to pulling in the active direction, so an unconnected CTS
     * behaves the same as if CTS were not enabled. */
    gpio_set_pulls(PROBE_UART_CTS, 0, 1);
    gpio_set_function(PROBE_UART_RTS, GPIO_FUNC_UART);
    gpio_set_function(PROBE_UART_CTS, GPIO_FUNC_UART);
    uart_set_hw_flow(PROBE_UART_INTERFACE, true, true);
#else
#ifdef PROBE_UART_RTS
    gpio_init(PROBE_UART_RTS);
    gpio_set_dir(PROBE_UART_RTS, GPIO_OUT);
    gpio_put(PROBE_UART_RTS, 1);
#endif
#endif

#ifdef PROBE_UART_DTR
    gpio_init(PROBE_UART_DTR);
    gpio_set_dir(PROBE_UART_DTR, GPIO_OUT);
    gpio_put(PROBE_UART_DTR, 1);
#endif
}




RingBuffer ring_buffers[2][2] = {};


bool cdc_task(uint8_t itf)
{
  /* Batch up to half a FIFO of data - don't clog up on RX */
  const size_t MAX_FETCH = 16;

  UartInterface uart = UART_INTERFACES[itf];

//  uint rx_len = 0;
  bool keep_alive = false;

  // Consume uart fifo regardless even if not connected
  RingBuffer* uart_to_cdc_buffer = &ring_buffers[itf][0];
  {
    size_t size = RingBuffer_size(uart_to_cdc_buffer);
    size_t bytes = RingBuffer_BUFFER_SIZE - size;
    if (bytes > MAX_FETCH){
        bytes = MAX_FETCH;
    }
    while (bytes > 0 && uart.is_readable()){
        uint8_t c = uart.getc();
        RingBuffer_putc(uart_to_cdc_buffer, c);
        bytes--;
    }
  }

  if (tud_cdc_n_connected(itf)) {
      uart.was_connected = 1;
      /* Implicit overflow if we don't write all the bytes to the host.
        * Also throw away bytes if we can't write... */
      if (RingBuffer_size(uart_to_cdc_buffer) > 0) {
//        printf("Try to push UART -> CDC\n");
#ifdef PROBE_UART_RX_LED
        gpio_put(PROBE_UART_RX_LED, 1);
        rx_led_debounce = debounce_ticks;
#endif
        size_t read_bytes;
        const uint8_t* data = RingBuffer_read_buffer(uart_to_cdc_buffer, &read_bytes);
        if (read_bytes > MAX_FETCH){
            read_bytes = MAX_FETCH;
        }
        if (read_bytes > 0){
            size_t write_bytes = tud_cdc_n_write(itf, data, read_bytes);
            tud_cdc_n_write_flush(itf);
            RingBuffer_pop_front(uart_to_cdc_buffer, write_bytes);
        }
      } else {
#ifdef PROBE_UART_RX_LED
        if (rx_led_debounce)
          rx_led_debounce--;
        else
          gpio_put(PROBE_UART_RX_LED, 0);
#endif
      }

#if 1
    /* Reading from a firehose and writing to a FIFO. */
    RingBuffer* cdc_to_uart_buffer = &ring_buffers[itf][1];
    size_t write_bytes;
    uint8_t* data = RingBuffer_write_buffer(cdc_to_uart_buffer, &write_bytes);
    if (write_bytes > MAX_FETCH){
        write_bytes = MAX_FETCH;
    }
    if (write_bytes > 0) {
#ifdef PROBE_UART_TX_LED
      gpio_put(PROBE_UART_TX_LED, 1);
      tx_led_debounce = debounce_ticks;
#endif
      write_bytes = tud_cdc_n_read(itf, data, write_bytes);
      RingBuffer_push_back(cdc_to_uart_buffer, write_bytes);

      size_t read_bytes;
      const uint8_t* read_buffer = RingBuffer_read_buffer(cdc_to_uart_buffer, &read_bytes);

      /* Batch up to half a FIFO of data - don't clog up on RX */
      if (read_bytes > MAX_FETCH){
          read_bytes = MAX_FETCH;
      }

      read_bytes = uart.write(read_buffer, read_bytes);
      RingBuffer_pop_front(cdc_to_uart_buffer, read_bytes);
    } else {
#ifdef PROBE_UART_TX_LED
        if (tx_led_debounce)
          tx_led_debounce--;
        else
          gpio_put(PROBE_UART_TX_LED, 0);
#endif
    }
#else
    /* Reading from a firehose and writing to a FIFO. */
    size_t watermark = MIN(tud_cdc_n_available(itf), sizeof(tx_buf));
    if (watermark > 0) {
      size_t tx_len;
#ifdef PROBE_UART_TX_LED
      gpio_put(PROBE_UART_TX_LED, 1);
      tx_led_debounce = debounce_ticks;
#endif
      /* Batch up to half a FIFO of data - don't clog up on RX */
      watermark = MIN(watermark, 16);
      tx_len = tud_cdc_n_read(itf, tx_buf, watermark);
      uart.write_blocking(tx_buf, tx_len);
    } else {
#ifdef PROBE_UART_TX_LED
        if (tx_led_debounce)
          tx_led_debounce--;
        else
          gpio_put(PROBE_UART_TX_LED, 0);
#endif
    }
#endif


    /* Pending break handling */
    if (timed_break) {
      if (((int)break_expiry - (int)xTaskGetTickCount()) < 0) {
        timed_break = false;
        uart.set_break(false);
#ifdef PROBE_UART_TX_LED
        tx_led_debounce = 0;
#endif
      } else {
        keep_alive = true;
      }
    }
  } else if (uart.was_connected) {
    tud_cdc_n_write_clear(itf);
    uart.set_break(false);
    timed_break = false;
    uart.was_connected = 0;
#ifdef PROBE_UART_TX_LED
    tx_led_debounce = 0;
#endif
  }
  return keep_alive;
}

void cdc_uart_set_baudrate(uint8_t itf, uint32_t baudrate) {
  /* Set the tick thread interval to the amount of time it takes to
   * fill up half a FIFO. Millis is too coarse for integer divide.
   */
  uint32_t micros = (1000 * 1000 * 16 * 10) / MAX(baudrate, 1);
  TickType_t interval = MAX(1, micros / ((1000 * 1000) / configTICK_RATE_HZ));
  debounce_ticks = MAX(1, configTICK_RATE_HZ / (interval * DEBOUNCE_MS));
  printf(
    "itf %d - New baud rate %ld micros %ld interval %lu\n",
    itf, baudrate, micros, interval
  );

  UART_INTERFACES[itf].interval = interval;
  UART_INTERFACES[itf].deinit();
  tud_cdc_n_write_clear(itf);
  tud_cdc_n_read_flush(itf);
  UART_INTERFACES[itf].init(baudrate);
}

void cdc_thread(void *ptr)
{
  BaseType_t delayed;
  last_wake = xTaskGetTickCount();
  bool keep_alive;
  /* Threaded with a polling interval that scales according to linerate */
  while (1) {
    keep_alive = cdc_task(0) | cdc_task(1);
    if (!keep_alive) {
      TickType_t interval0 = UART_INTERFACES[0].interval;
      TickType_t interval1 = UART_INTERFACES[1].interval;
      delayed = xTaskDelayUntil(
        &last_wake,
        interval0 < interval1 ? interval0 : interval1
      );
      if (delayed == pdFALSE)
        last_wake = xTaskGetTickCount();
#if 0
      if (autobaud_running) {
        // Receive baud information from autobaud thread
        if (xQueueReceive(baudQueue, &baud_info, 0) == pdTRUE) {
          cdc_uart_set_baudrate(baud_info.baud);
          // Assume 8N1
          uart_set_format(PROBE_UART_INTERFACE, 8, 1, UART_PARITY_NONE);
        }
      }
#endif
    }
  }
}

void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const* line_coding)
{
#if 0
  if (line_coding->bit_rate == MAGIC_BAUD) {
    if (!autobaud_running)
      autobaud_start();
    return;
  }
  else if (autobaud_running) {
    autobaud_wait_stop();
  }
#endif
  uart_parity_t parity;
  uint data_bits, stop_bits;

  /* Modifying state, so park the thread before changing it. */
  if(tud_cdc_n_connected(0) || tud_cdc_n_connected(1)){
    vTaskSuspend(uart_taskhandle);
  }

  cdc_uart_set_baudrate(itf, line_coding->bit_rate);

  switch (line_coding->parity) {
  case CDC_LINE_CODING_PARITY_ODD:
    parity = UART_PARITY_ODD;
    break;
  case CDC_LINE_CODING_PARITY_EVEN:
    parity = UART_PARITY_EVEN;
    break;
  default:
    probe_info("invalid parity setting %u\n", line_coding->parity);
    /* fallthrough */
  case CDC_LINE_CODING_PARITY_NONE:
    parity = UART_PARITY_NONE;
    break;
  }

  switch (line_coding->data_bits) {
  case 5:
  case 6:
  case 7:
  case 8:
    data_bits = line_coding->data_bits;
    break;
  default:
    probe_info("invalid data bits setting: %u\n", line_coding->data_bits);
    data_bits = 8;
    break;
  }

  /* The PL011 only supports 1 or 2 stop bits. 1.5 stop bits is translated to 2,
   * which is safer than the alternative. */
  switch (line_coding->stop_bits) {
  case CDC_LINE_CONDING_STOP_BITS_1_5:
  case CDC_LINE_CONDING_STOP_BITS_2:
    stop_bits = 2;
  break;
  default:
    probe_info("invalid stop bits setting: %u\n", line_coding->stop_bits);
    /* fallthrough */
  case CDC_LINE_CONDING_STOP_BITS_1:
    stop_bits = 1;
  break;
  }

  uart_set_format(PROBE_UART_INTERFACE, data_bits, stop_bits, parity);
  /* Windows likes to arbitrarily set/get line coding after dtr/rts changes, so
   * don't resume if we shouldn't */
  if(tud_cdc_n_connected(0) || tud_cdc_n_connected(1)){
    vTaskResume(uart_taskhandle);
  }
}

void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
#ifdef PROBE_UART_RTS
  gpio_put(PROBE_UART_RTS, !rts);
#endif
#ifdef PROBE_UART_DTR
  gpio_put(PROBE_UART_DTR, !dtr);
#endif

  UART_INTERFACES[itf].dtr = dtr;
  UART_INTERFACES[itf].rts = rts;

  printf(
    "itf %d - dtr = %d, rts = %d\n",
    itf, dtr, rts
  );

  /* CDC drivers use linestate as a bodge to activate/deactivate the interface.
   * Resume our UART polling on activate, stop on deactivate */
  if (UART_INTERFACES[0].dtr || UART_INTERFACES[1].dtr) {
    vTaskResume(uart_taskhandle);
  } else {
    vTaskSuspend(uart_taskhandle);
#ifdef PROBE_UART_RX_LED
    gpio_put(PROBE_UART_RX_LED, 0);
    rx_led_debounce = 0;
#endif
#ifdef PROBE_UART_TX_LED
    gpio_put(PROBE_UART_TX_LED, 0);
    tx_led_debounce = 0;
#endif
  }
}

void tud_cdc_send_break_cb(uint8_t itf, uint16_t wValue) {
  switch(wValue) {
    case 0:
    UART_INTERFACES[itf].set_break(false);
    timed_break = false;
#ifdef PROBE_UART_TX_LED
    tx_led_debounce = 0;
#endif
    break;
    case 0xffff:
    UART_INTERFACES[itf].set_break(true);
    timed_break = false;
#ifdef PROBE_UART_TX_LED
    gpio_put(PROBE_UART_TX_LED, 1);
    tx_led_debounce = 1 << 30;
#endif
    break;
    default:
    UART_INTERFACES[itf].set_break(true);
    timed_break = true;
#ifdef PROBE_UART_TX_LED
    gpio_put(PROBE_UART_TX_LED, 1);
    tx_led_debounce = 1 << 30;
#endif
    break_expiry = xTaskGetTickCount() + (wValue * (configTICK_RATE_HZ / 1000));
    break;
  }
}
