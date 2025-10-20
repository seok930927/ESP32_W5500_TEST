/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "SPI_EXAMPLE";

// SPI 핀 설정
#define SPI_MOSI_PIN    38
#define SPI_MISO_PIN    47
#define SPI_CLK_PIN     21
#define SPI_CS_PIN      18

// SPI 초기화 함수
void spi_init(void)
{
    esp_err_t ret;
    
    // SPI 버스 설정
    spi_bus_config_t buscfg = {
        .miso_io_num = SPI_MISO_PIN,
        .mosi_io_num = SPI_MOSI_PIN,
        .sclk_io_num = SPI_CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    
    // SPI 디바이스 설정
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,  // 1MHz
        .mode = 0,                  // SPI mode 0
        .spics_io_num = SPI_CS_PIN,
        .queue_size = 7,
    };
    
    // SPI 버스 초기화
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "SPI bus initialized successfully");
}

// SPI 데이터 전송 함수
void spi_send_data(spi_device_handle_t spi, uint8_t *data, size_t len)
{
    esp_err_t ret;
    spi_transaction_t t;
    
    memset(&t, 0, sizeof(t));
    t.length = len * 8;  // 비트 단위로 설정
    t.tx_buffer = data;
    
    ret = spi_device_transmit(spi, &t);
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "SPI data sent successfully");
}

// SPI 명령어/주소 전송 후 데이터 수신 함수 (정적 버퍼 버전)
void spi_receive_data(spi_device_handle_t spi, uint16_t addr , uint8_t cmd ,  uint8_t *data, size_t len)
{
    esp_err_t ret;
    spi_transaction_t t;
    
    memset(&t, 0, sizeof(t));
    
    // 최대 8바이트까지 지원 (명령어 1 + 주소 1 + 데이터 6)
    uint8_t tx_rx_buffer[8] = {0};
    uint8_t total_len = 3 + len;  // 명령어+주소(2바이트) + 수신데이터(len바이트)
    
    if (total_len > sizeof(tx_rx_buffer)) {
        ESP_LOGE(TAG, "Data length too large. Max supported: %d bytes", 
                 sizeof(tx_rx_buffer) - 2);
        return;
    }
    
    // 명령어와 주소 설정
    tx_rx_buffer[0] = (addr & 0xff00) >> 8 ;  // 명령어
    tx_rx_buffer[1] = (addr & 0x00ff) >> 0;  // 주소
    tx_rx_buffer[2] = cmd    & 0xff;  // 주소
    
    t.length = total_len * 8;  // total_len << 3 // 전체 길이 (비트 단위)
    t.tx_buffer = tx_rx_buffer;
    t.rx_buffer = tx_rx_buffer;  // 같은 버퍼 사용
    
    ret = spi_device_transmit(spi, &t);
    ESP_ERROR_CHECK(ret);
    
    // 수신된 데이터만 복사 (명령어/주소 부분 제외)
    memcpy(data, &tx_rx_buffer[3], len);
    
    ESP_LOGI(TAG, "SPI command sent: 0x%02X, address: 0x%02X", 
             tx_rx_buffer[0], tx_rx_buffer[1]);
    ESP_LOGI(TAG, "SPI data received successfully");
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting SPI example");
    
    // SPI 초기화
    spi_init();
    
    // SPI 디바이스 핸들
    spi_device_handle_t spi;
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 50000000,  // 1MHz
        .mode = 0,                  // SPI mode 0
        .spics_io_num = SPI_CS_PIN,
        .queue_size = 7,
    };
    
    // SPI 디바이스 추가
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi));
    
    // 테스트 데이터
    uint8_t tx_data[4] = {0xAA, 0xBB, 0xCC, 0xDD};
    uint8_t tx_data2[4] = {0xAA, 0xBB};
    uint8_t rx_data[4] = {0};
    
    unsigned char i = 0;
    while(1) {
        printf("Hello Lihan! SPI Example - Cycle %d\n", i);
        
        // // SPI 데이터 전송
        // spi_send_data(spi, tx_data, sizeof(tx_data));
        
        // // 잠시 대기
        // vTaskDelay(500 / portTICK_PERIOD_MS);
        
        // SPI 데이터 수신 (루프백 테스트)
        spi_receive_data(spi, 0x0039, 0x00, rx_data, 1);
        
        // 수신된 데이터 출력
        ESP_LOGI(TAG, "Received data: 0x%02X 0x%02X 0x%02X 0x%02X", 
                 rx_data[0], rx_data[1], rx_data[2], rx_data[3]);
        
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        i++;
    }
    


        // /* Print chip information */
    // esp_chip_info_t chip_info;
    // uint32_t flash_size;
    // esp_chip_info(&chip_info);
    // printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
    //        CONFIG_IDF_TARGET,
    //        chip_info.cores,
    //        (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
    //        (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
    //        (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
    //        (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    // unsigned major_rev = chip_info.revision / 100;
    // unsigned minor_rev = chip_info.revision % 100;
    // printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    // if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
    //     printf("Get flash size failed");
    //     return;
    // }

    // printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
    //        (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    // printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    // for (int i = 10; i >= 0; i--) {
    
    // }
    // printf("Restarting now.\n");
    // fflush(stdout);
    // esp_restart();
}
