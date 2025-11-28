/**
 * Serial protocol handling for ESP32.
 */
#include "serial.h"

#include <string.h>
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "swim.h"
#include "version.h"

#define UART_PORT UART_NUM_0
#define UART_BUF_SIZE 1024

#define CMD_SRST 0
#define CMD_ROTF 1
#define CMD_WOTF 2
#define CMD_RESET 0xFD
#define CMD_INIT 0xFE
#define CMD_VERSION 0xFF

static const char *TAG = "serial";

static int cmd_buf_idx = 0;
/**
 * Multi-use buffer for recv and send.
 * 1 byte command
 * 1 byte length
 * 3 byte address
 * 255 byte user data
 */
static uint8_t cmd_buf[1 + 4 + 255];

static inline void send_ack(void) {
  uart_write_bytes(UART_PORT, (const char *)cmd_buf, 1);
}

static inline void send_error(int code) {
  uint8_t resp[3];
  resp[0] = 0xFF;
  code = -code;
  resp[1] = code >> 8;
  resp[2] = code;
  uart_write_bytes(UART_PORT, (const char *)resp, sizeof(resp));
}

static bool command_complete(void) {
  if (cmd_buf_idx == 0) return false;
  switch (cmd_buf[0]) {
    case CMD_ROTF:
      return cmd_buf_idx >= 5;
    case CMD_WOTF:
      return cmd_buf_idx >= 5 && cmd_buf[1] == cmd_buf_idx - 5;
    case CMD_RESET:
      return cmd_buf_idx >= 2;
    default:
      return true;
  }
}

static void serial_task(void *arg) {
  while (1) {
    int read = uart_read_bytes(UART_PORT, cmd_buf + cmd_buf_idx, 1,
                               portMAX_DELAY);
    if (read <= 0) {
      continue;
    }
    cmd_buf_idx += read;

    if (!command_complete()) continue;

    int result = 0;
    send_ack();
    switch (cmd_buf[0]) {
      case CMD_SRST:
        result = srst();
        break;
      case CMD_ROTF:
        result = rotf(cmd_buf + 1, cmd_buf + 5);
        cmd_buf_idx += cmd_buf[1];
        break;
      case CMD_WOTF:
        result = wotf(cmd_buf + 1);
        cmd_buf_idx = 5;
        break;
      case CMD_INIT:
        result = swim_entry();
        cmd_buf[cmd_buf_idx++] = result >> 8;
        cmd_buf[cmd_buf_idx++] = result;
        break;
      case CMD_RESET:
        reset(cmd_buf[1]);
        cmd_buf_idx = 1;
        break;
      case CMD_VERSION:
        cmd_buf[cmd_buf_idx++] = FIRMWARE_VERSION_MAJOR;
        cmd_buf[cmd_buf_idx++] = FIRMWARE_VERSION_MINOR;
        break;
      default:
        result = -1;
    }

    if (result < 0) {
      send_error(result);
    } else {
      cmd_buf[0] = 0;
      uart_write_bytes(UART_PORT, (const char *)cmd_buf, cmd_buf_idx);
    }
    cmd_buf_idx = 0;
  }
  vTaskDelete(NULL);
}

void serial_init(void) {
  const uart_config_t cfg = {
      .baud_rate = 921600,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };
  ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_BUF_SIZE, UART_BUF_SIZE,
                                      0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(UART_PORT, &cfg));
  ESP_ERROR_CHECK(
      uart_set_pin(UART_PORT, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                   UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  xTaskCreatePinnedToCore(serial_task, "serial_task", 4096, NULL, 10, NULL, 0);
  ESP_LOGI(TAG, "UART ready at 921600 baud");
}
