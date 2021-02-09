/*
 * main.h
 *
 *  Created on: 2020. 11. 02.
 *      Author: CW_KIM
 */

#ifndef MAIN_MAIN_H_
#define MAIN_MAIN_H_

#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#include "soc/io_mux_reg.h"
#include "string.h"

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#include "ESP32_BLE.h"
#include "ESP32_UART.h"
#include "ESP32_DSCNN.h"

void GPIO_init();

void Task_init_init();

void task_create();
void task_delete();

void task_DSCNN_info_periodic_transmit(void *pvParameters);

int16_t norm_calc(float UDS_data, int sensor_num, int sign);

#endif /* MAIN_MAIN_H_ */
