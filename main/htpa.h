#ifndef _HTPA_H_
#define _HTPA_H_
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "cJSON.h"

#define CHIP_ADDR 0x1A
#define CONFIG_REG 0x01
#define WAKEUP_BIT 0x01
void wakeup();
void captureHTPAData();
void initHTPAData();
esp_err_t eeprom_write_byte(uint8_t deviceaddress, uint16_t eeaddress, uint8_t byte);
esp_err_t eeprom_write(uint8_t deviceaddress, uint16_t eeaddress, uint8_t *data, size_t size);

uint8_t eeprom_read_byte(uint8_t deviceaddress, uint16_t eeaddress);
esp_err_t eeprom_read(uint8_t deviceaddress, uint16_t eeaddress, uint8_t *data, size_t size);

#endif