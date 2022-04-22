/* HTPA32x32 ESP32 idf example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "server.h"
#include "connect.h"
#include "esp_spiffs.h"

#include "htpa.h"
#include "driver/gpio.h"
#define TAG "SOS APP main "
xSemaphoreHandle connectionSemaphore;
xSemaphoreHandle HtpaSemaphore;
xSemaphoreHandle HTPAReadySemaphore;
xSemaphoreHandle initSemaphore;
cJSON *rootJSON;

/* */
#define GPIO_INPUT_IO_1 25
#define GPIO_INPUT_PIN_SEL (1ULL << GPIO_INPUT_IO_1)
#define ESP_INTR_FLAG_DEFAULT 0
static xQueueHandle gpio_evt_queue = NULL;
  int32_t long_press_interval;
  uint32_t long_press_interval_1;
  uint32_t long_press_interval_0;
static uint32_t millis()
{
  return esp_timer_get_time() / 1000;
}
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
  uint32_t gpio_num = (uint32_t)arg;
  xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_button_press(void *arg)
{
  ESP_LOGI(TAG, "gpio_task_button_press");

  uint32_t io_num;
  for (;;)
  {

    if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
    {
      // printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
      // ESP_LOGI(TAG, "GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
      if (gpio_get_level(io_num) == 1)
      {
        long_press_interval_1 = millis();
        // long_press_interval_0 = 0;
      }
      else
      {
        long_press_interval_0 = millis();
       }
      long_press_interval = long_press_interval_0 - long_press_interval_1;
     
      if ((long_press_interval_0 != 0) && (long_press_interval_1 != 0) && (long_press_interval > 1000))
      {
        long_press_interval = long_press_interval_0 - long_press_interval_1;
        long_press_interval_0 = 0;
        long_press_interval_1 = 0;
        ESP_LOGI(TAG, "GPIO[%d] intr, pressed for: %d\n", io_num, long_press_interval);
        resetwifi();
      }
    }
  }
}

void print_info()
{
  printf("SOS APP\n");
  esp_log_level_set(TAG, ESP_LOG_DEBUG);
  /* Print chip information */
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);
  printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
         chip_info.cores,
         (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
         (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

  printf("silicon revision %d, ", chip_info.revision);

  printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
         (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
 

      
}

void captureHTPADataTask(void *para){
   {
     initHTPAData();
    while (true)
  {if (xSemaphoreTake(HtpaSemaphore, portMAX_DELAY))
    {
      captureHTPAData();
      
    }
  }
    
  }
}
void OnConnected(void *para)
{

  while (true)
  {
    if (xSemaphoreTake(connectionSemaphore, portMAX_DELAY))
    {
      RegisterEndPoints();
     }
   
  }
}

void app_main(void)
{
  print_info();
   gpio_config_t io_conf = {};
  // disable interrupt
  // set as output mode
  // bit mask of the pins that you want to set,e.g.GPIO18/19
  // disable pull-down mode
  io_conf.pull_down_en = 0;

  // interrupt of rising edge
  io_conf.intr_type = GPIO_INTR_ANYEDGE;
  // bit mask of the pins, use GPIO25 here
  io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
  // set as input mode
  io_conf.mode = GPIO_MODE_INPUT;
  // enable pull-up mode
  io_conf.pull_up_en = 1;
  gpio_config(&io_conf);
  // create a queue to handle gpio event from isr
  gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
  // start gpio task
  xTaskCreate(gpio_task_button_press, "gpio_task_button_press", 2048, NULL, 10, NULL);
  // install gpio isr service
  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
  // hook isr handler for specific gpio pin
  gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void *)GPIO_INPUT_IO_1);

  HtpaSemaphore = xSemaphoreCreateBinary();
  connectionSemaphore = xSemaphoreCreateBinary();
  initSemaphore = xSemaphoreCreateBinary();
  HTPAReadySemaphore = xSemaphoreCreateBinary();
  // wifiInit();
  xTaskCreate(&wifiInit, "init comms", 1024 * 3, NULL, 10, NULL);
  xSemaphoreGive(initSemaphore);

  xTaskCreate(&OnConnected, "handel comms", 1024 * 15, NULL, 5, NULL);
  xTaskCreate(&captureHTPADataTask, "capture HTPA", 1024 * 100, NULL, configMAX_PRIORITIES - 1 , NULL);
  // xSemaphoreGive(HtpaSemaphore);
}
