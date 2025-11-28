/**
 * ESP32 entrypoint for ESP-STLINK.
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "serial.h"
#include "swim.h"

static const char *TAG = "esp-stlink";

void app_main(void) {
  swim_init();
  serial_init();
  ESP_LOGI(TAG, "ESP-STLINK (ESP32) ready");
}
