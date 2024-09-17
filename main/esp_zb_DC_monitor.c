/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Zigbee HA_temperature_sensor Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */
#include "esp_zb_DC_monitor.h"
#include "temp_sensor_driver.h"
#include "switch_driver.h"

#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "pzem003.h"
#include "esp_timer.h"
#include "cJSON.h"



/* @brief Set ESP32  Serial Configuration */
pzem_setup_t pzConf =
{
    .pzem_uart   = UART_NUM_1,              /*  <== Specify the UART you want to use, UART_NUM_0, UART_NUM_1, UART_NUM_2 (ESP32 specific) */
    .pzem_rx_pin = GPIO_NUM_4,             /*  <== GPIO for RX */
    .pzem_tx_pin = GPIO_NUM_5,             /*  <== GPIO for TX */
    .pzem_addr   = 0x01,      /*  If your module has a different address, specify here or update the variable in pzem004tv3.h */
};

///static const char * TAG = "APP_MAIN";
TaskHandle_t PMonTHandle = NULL;
_current_values_t pzValues;            /* Measured values */

void PMonTask( void * pz );

void PMonTask( void * pz )
{
    for( ;; )
    {
        PzemGetValues( &pzConf, &pzValues );
        //printf( "Vrms: %.2fV - Irms: %.2fA - P: %.1fW - E: %.1fWh\n", pzValues.voltage, pzValues.current, pzValues.power, pzValues.energy );
        //printf( "Vrms: %.2fV - Irms: %.2fA - P: %.2fW - E: %.0fWh\n", pzValues.voltage, pzValues.current, pzValues.power, random()*1.1);
        printf( "{V:%.2f,I:%.2f,P:%.2f,E:%.1f}\n", pzValues.voltage, pzValues.current, pzValues.power, pzValues.power);
        //ESP_LOGI( TAG, "Stack High Water Mark: %ld Bytes free", ( unsigned long int ) uxTaskGetStackHighWaterMark( NULL ) );     /* Show's what's left of the specified stacksize */

        vTaskDelay( pdMS_TO_TICKS( 2500 ) );
    }


    vTaskDelete( NULL );
}
void app_main(void)
{
    PzemInit( &pzConf ); 

    xTaskCreate(PMonTask, "PowerMon", 4096, NULL, 5, NULL);
        
        


}
