/*
 * CMeteo - By @bismarckssr
 */

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include <inttypes.h>
#include "esp_wifi.h"
#include "nvs_flash.h"
#include <esp_err.h>
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "driver/gpio.h"

static const char *TAG = "CMeteo";


// Constants definition - now configured via menuconfig (idf.py menuconfig)
#define I2C_MASTER_SCL_IO           CONFIG_BME280_I2C_SCL_GPIO
#define I2C_MASTER_SDA_IO           CONFIG_BME280_I2C_SDA_GPIO
#define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ    100000     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000
#define LED_GPIO CONFIG_LED_GPIO

#define OSRS_H CONFIG_BME280_OVERSAMPLING_HUM
#define OSRS_T CONFIG_BME280_OVERSAMPLING_TEMP
#define OSRS_P CONFIG_BME280_OVERSAMPLING_PRESS

#define WIFI_SSID CONFIG_WIFI_SSID
#define WIFI_PASS CONFIG_WIFI_PASSWORD
#define MAX_RETRY CONFIG_WIFI_MAXIMUM_RETRY
#define EXAMPLE_H2E_IDENTIFIER CONFIG_WIFI_H2E_IDENTIFIER
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define INFLUXDB_URL CONFIG_INFLUXDB_URL
#define INFLUXDB_TOKEN CONFIG_INFLUXDB_TOKEN
typedef struct {
    // Temperature calibration
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;

    // Pressure calibration
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;

    // Humidity calibration
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;

    // Temporary storage
    int32_t t_fine;
} bme280_calib_t;

typedef struct {
    float temp;
    float pres;
    float hum;
} result_struct;

static result_struct results;
static bme280_calib_t calib;


void led_flasher(void *pvParameters)
  {
    int delay_ms = (int)pvParameters;
    while(1) {
        gpio_set_level(LED_GPIO, 1); //ON
        vTaskDelay(delay_ms / portTICK_PERIOD_MS);
        gpio_set_level(LED_GPIO, 0); //OFF
        vTaskDelay(delay_ms / portTICK_PERIOD_MS);
    }
}

/**
 * @brief i2c master initialization
 */
static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x76,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}


/**
 * @brief Calibration formula for temperature - from bosch's datasheet
 */
static int32_t calibrate_temperature(uint32_t *raw_temp)
{
    int32_t var1, var2, T;
    var1 = ((((*raw_temp >> 3)-((int32_t) calib.dig_T1 << 1))) * ((int32_t) calib.dig_T2)) >> 11;
    var2 = (((((*raw_temp >> 4)-((int32_t) calib.dig_T1)) * ((*raw_temp >> 4)-((int32_t) calib.dig_T1))) >> 12) * ((int32_t) calib.dig_T3)) >> 14;
    calib.t_fine = var1 + var2;
    T = (calib.t_fine * 5 + 128) >> 8;
    return T;
}

/**
 * @brief Calibration formula for pressure - from bosch's datasheet
 */
static uint32_t calibrate_pressure(uint32_t *raw_pressure)
{
    int64_t var1, var2, p;
    var1 = ((int64_t) calib.t_fine)-128000;
    var2 = var1 * var1 * (int64_t) calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t) calib.dig_P5) << 17);
    var2 = var2 + (((int64_t) calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t) calib.dig_P3) >> 8) + ((var1 * (int64_t) calib.dig_P2) << 12);
    var1 = (((((int64_t) 1) << 47) + var1)) * ((int64_t) calib.dig_P1) >> 33;
    if (var1 == 0) {
        return 0;
    }
    p = 1048576 - *raw_pressure;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t) calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t) calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t) calib.dig_P7) << 4);
    return (uint32_t) p;
}

/**
 * @brief Calibration formula for humidity - from bosch's datasheet
 */
static uint32_t calibrate_humidity(uint32_t *raw_humidity)
{
    int32_t v_x1_u32r;
    v_x1_u32r = (calib.t_fine - ((int32_t)76800));

    v_x1_u32r = (((((*raw_humidity << 14) - (((int32_t)calib.dig_H4) << 20) - (((int32_t)calib.dig_H5) *
    v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r *
    ((int32_t)calib.dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)calib.dig_H3)) >> 11) +
    ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)calib.dig_H2) +
    8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
    ((int32_t)calib.dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (uint32_t)(v_x1_u32r>>12);
}


/**
 * @brief Reads calibration coefficients for the formulas
 */
static void read_coefficients(i2c_master_dev_handle_t *dev_handle)
{
    uint8_t starting_reg = 0x88;
    uint8_t data[26];

    ESP_ERROR_CHECK(i2c_master_transmit_receive(*dev_handle, &starting_reg, 1, data, 26, I2C_MASTER_TIMEOUT_MS));

    // 10011101 -> 00000000 10011101
    calib.dig_T1 = (uint16_t)(data[1] << 8 | data[0]);
    calib.dig_T2 = (int16_t)(data[3] << 8 | data[2]);
    calib.dig_T3 = (int16_t)(data[5] << 8 | data[4]);

    calib.dig_P1 = (uint16_t)(data[7] << 8 | data[6]);
    calib.dig_P2 = (int16_t)(data[9] << 8 | data[8]);
    calib.dig_P3 = (int16_t)(data[11] << 8 | data[10]);
    calib.dig_P4 = (int16_t)(data[13] << 8 | data[12]);
    calib.dig_P5 = (int16_t)(data[15] << 8 | data[14]);
    calib.dig_P6 = (int16_t)(data[17] << 8 | data[16]);
    calib.dig_P7 = (int16_t)(data[19] << 8 | data[18]);
    calib.dig_P8 = (int16_t)(data[21] << 8 | data[20]);
    calib.dig_P9 = (int16_t)(data[23] << 8 | data[22]);

    calib.dig_H1 = (unsigned char)(data[25]);




    starting_reg = 0xE1;

    ESP_ERROR_CHECK(i2c_master_transmit_receive(*dev_handle, &starting_reg, 1, data, 8, I2C_MASTER_TIMEOUT_MS));
    calib.dig_H2 = (int16_t)(data[1] << 8 | data[0]);
    calib.dig_H3 = (unsigned char)(data[2]);
    calib.dig_H4 = (int16_t)((data[3] << 4) | (data[4] & 0x0F));
    calib.dig_H5 = (int16_t)((data[6] << 4) | (data[5] >> 4));
    calib.dig_H6 = (signed char)(data[7]);

    printf("=== BME280 Calibration Coefficients ===\n");
    printf("dig_T1 = %u\n", calib.dig_T1);
    printf("dig_T2 = %d\n", calib.dig_T2);
    printf("dig_T3 = %d\n", calib.dig_T3);

    printf("dig_P1 = %u\n", calib.dig_P1);
    printf("dig_P2 = %d\n", calib.dig_P2);
    printf("dig_P3 = %d\n", calib.dig_P3);
    printf("dig_P4 = %d\n", calib.dig_P4);
    printf("dig_P5 = %d\n", calib.dig_P5);
    printf("dig_P6 = %d\n", calib.dig_P6);
    printf("dig_P7 = %d\n", calib.dig_P7);
    printf("dig_P8 = %d\n", calib.dig_P8);
    printf("dig_P9 = %d\n", calib.dig_P9);

    printf("dig_H1 = %u\n", calib.dig_H1);
    printf("dig_H2 = %d\n", calib.dig_H2);
    printf("dig_H3 = %u\n", calib.dig_H3);
    printf("dig_H4 = %d\n", calib.dig_H4);
    printf("dig_H5 = %d\n", calib.dig_H5);
    printf("dig_H6 = %d\n", calib.dig_H6);
    printf("=======================================\n");


}


/**
 * @brief reads temperature
 */
static void read_temp(i2c_master_dev_handle_t *dev_handle)
{
    uint8_t reg = 0xFA;
    uint8_t reg_status = 0xF3;
    uint8_t data[3]; // 8*3 = 24 bits avail (8+8+3)
    uint32_t temp_uncal;

    //controllo che ci sia da leggere qualcosa
    ESP_ERROR_CHECK(i2c_master_transmit_receive(*dev_handle, &reg_status, 1, data, 1, I2C_MASTER_TIMEOUT_MS));

    while ( (data[0] & 0x08 ) != 0x00 ) {
        ESP_ERROR_CHECK(i2c_master_transmit_receive(*dev_handle, &reg_status, 1, data, 1, I2C_MASTER_TIMEOUT_MS));
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    ESP_ERROR_CHECK(i2c_master_transmit_receive(*dev_handle, &reg, 1, data, 3, I2C_MASTER_TIMEOUT_MS));

    // 11010011 -> 11010011!000000000000
    // 10000011 -> 11010011!10000011!0000
    // 1001 -> 11010011!10000011!1001
    temp_uncal = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | ((uint32_t)data[2] >> 4);
    printf("Temp uncal: %" PRIu32 "\n", temp_uncal);
    int32_t final_temp = calibrate_temperature(&temp_uncal);
    results.temp = final_temp / 100.0f;
    printf("Temp cal: %f\n", results.temp);
}

static void read_pressure(i2c_master_dev_handle_t *dev_handle)
{
    uint8_t reg = 0xF7;
    uint8_t reg_status = 0xF3;
    uint8_t data[3]; // 8*3 = 24 bits avail (8+8+3)
    uint32_t press_uncal;

    //controllo che ci sia da leggere qualcosa
    ESP_ERROR_CHECK(i2c_master_transmit_receive(*dev_handle, &reg_status, 1, data, 1, I2C_MASTER_TIMEOUT_MS));

    while ( (data[0] & 0x08 ) != 0x00 ) {
        ESP_ERROR_CHECK(i2c_master_transmit_receive(*dev_handle, &reg_status, 1, data, 1, I2C_MASTER_TIMEOUT_MS));
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    ESP_ERROR_CHECK(i2c_master_transmit_receive(*dev_handle, &reg, 1, data, 3, I2C_MASTER_TIMEOUT_MS));

    // 11010011 -> 11010011!000000000000
    // 10000011 -> 11010011!10000011!0000
    // 1001 -> 11010011!10000011!1001
    printf("Raw bytes: MSB=0x%02X, LSB=0x%02X, XLSB=0x%02X\n", data[0], data[1], data[2]);
    press_uncal = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | ((uint32_t)data[2] >> 4);
    printf("Press uncal: %" PRIu32 "\n", press_uncal);
    uint32_t final_press = calibrate_pressure(&press_uncal);
    results.pres = final_press / 256.0f / 100.0f;
    printf("Press cal: %f\n", results.pres);
}

static void read_humidity(i2c_master_dev_handle_t *dev_handle)
{
    uint8_t reg = 0xFD;
    uint8_t reg_status = 0xF3;
    uint8_t data[2]; // 8*2 = 16 bits avail (8+8)
    uint32_t hum_uncal;

    //controllo che ci sia da leggere qualcosa
    ESP_ERROR_CHECK(i2c_master_transmit_receive(*dev_handle, &reg_status, 1, data, 1, I2C_MASTER_TIMEOUT_MS));

    while ( (data[0] & 0x08 ) != 0x00 ) {
        ESP_ERROR_CHECK(i2c_master_transmit_receive(*dev_handle, &reg_status, 1, data, 1, I2C_MASTER_TIMEOUT_MS));
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    ESP_ERROR_CHECK(i2c_master_transmit_receive(*dev_handle, &reg, 1, data, 2, I2C_MASTER_TIMEOUT_MS));

    hum_uncal = ((uint32_t)data[0] << 8) | ((uint32_t)data[1]);
    printf("Hum uncal: %" PRIu32 "\n", hum_uncal);
    uint32_t final_hum = calibrate_humidity(&hum_uncal);
    results.hum = final_hum / 1024.0f;
    printf("Hum cal: %f\n", results.hum);
}



static const char *TAG_BME = "bme_init";

static esp_err_t bme280_write_reg_retry(i2c_master_dev_handle_t *dev_handle, uint8_t reg, uint8_t value, int retries)
{
    uint8_t tx[2] = { reg, value };
    for (int attempt = 0; attempt < retries; ++attempt) {
        esp_err_t err = i2c_master_transmit(*dev_handle, tx, sizeof(tx), I2C_MASTER_TIMEOUT_MS);
        if (err == ESP_OK) return ESP_OK;
        ESP_LOGW(TAG_BME, "write reg 0x%02X attempt %d failed: 0x%X", reg, attempt + 1, err);
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
    ESP_LOGE(TAG_BME, "write reg 0x%02X failed after %d attempts", reg, retries);
    return ESP_FAIL;
}


static esp_err_t bme280_read_reg_retry(i2c_master_dev_handle_t *dev_handle, uint8_t reg_to_read, uint8_t *recv_arr, size_t read_size, uint8_t retries)
{

    for (int attempt = 0; attempt < retries; ++attempt) {
        esp_err_t err = i2c_master_transmit_receive(*dev_handle, &reg_to_read, sizeof(reg_to_read), recv_arr, read_size, I2C_MASTER_TIMEOUT_MS);
        if (err == ESP_OK) return ESP_OK;
        ESP_LOGW(TAG_BME, "read reg 0x%02X attempt %d failed: 0x%X", reg_to_read, attempt + 1, err);
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
    ESP_LOGE(TAG_BME, "read reg 0x%02X failed after %d attempts", reg_to_read, retries);
    return ESP_FAIL;
}


void set_mode(i2c_master_dev_handle_t *dev_handle, uint8_t mode)
{
    uint8_t data[3];
    bme280_read_reg_retry(dev_handle, 0xF4, data, 1, 3);
    data[0] &= ~(0x03);
    data[0] |= (mode & 0x03);

    bme280_write_reg_retry(dev_handle, 0xF4, data[0], 3);
}


void bme280_init_defaults(i2c_master_dev_handle_t *dev_handle, uint8_t mode, bool enable_iff)
{

    bme280_write_reg_retry(dev_handle, 0xE0, 0xB6, 3);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    //0) Ottenimento valori in 0xF4
    uint8_t data[3];
    bme280_read_reg_retry(dev_handle, 0xF4, data, 1, 3);
    data[0] &= ~(0x03);
    data[0] |= (mode & 0x03);

    //1) Imposto in modalità di sleep
    bme280_write_reg_retry(dev_handle, 0xF4, data[0], 3);

    //2) Scrivo ctrl hum (ordine obbligato)
    bme280_write_reg_retry(dev_handle, 0xF2, OSRS_H, 3);

    //3) Scrivo ctrl meas
    data[0] &= ~(0xFC);
    data[0] |= (OSRS_T << 5);
    data[0] |= (OSRS_P << 2);
    bme280_write_reg_retry(dev_handle, 0xF4, data[0], 3);

    //4) Abilitazione dell'IIR
    if (enable_iff) {
        printf("IIR abilitato\n");
        uint8_t cfg;
        bme280_read_reg_retry(dev_handle, 0xF5, &cfg, 1, 3);
        cfg &= ~0x1C;  // 0b00011100, maschera
        cfg |= ( (0x02 & 0x07) << 2 );
        // scrivi di nuovo il registro
        bme280_write_reg_retry(dev_handle, 0xF5, cfg, 3);

        for (int i = 0; i < 3; i++) { // N = coeff+1 per sicurezza
            uint8_t tmp[3];
            i2c_master_transmit_receive(*dev_handle, &(uint8_t){0xF7}, 1, tmp, 3, I2C_MASTER_TIMEOUT_MS);
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
}


// WIFI PART
static int s_retry_num = 0;
static EventGroupHandle_t s_wifi_event_group;

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}


static void wifi_init()
{
    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL));


    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));


    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
            .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    s_wifi_event_group = xEventGroupCreate();

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,
        pdFALSE,
        portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap");
        // task flash impostato a 5 secondi
        xTaskCreate(
            led_flasher,           // Funzione del task
            "led_flasher",         // Nome del task (per debug)
            2048,                  // Stack size in bytes
            (void *)CONFIG_LED_BLINK_PERIOD_MS,  // Parametri da passare (NULL se non servono)
            5,                     // Priorità (0-25, più alto = più priorità)
            NULL                   // Handle del task (NULL se non serve)
        );
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to ap. Restarting");
        esp_restart();
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
        esp_restart();
    }

}

// HTTP PART
static esp_http_client_handle_t client;

static void http_init()
{
    esp_http_client_config_t config = {
        .url = INFLUXDB_URL,
        .method = HTTP_METHOD_POST,
        .crt_bundle_attach = &esp_crt_bundle_attach,
        .skip_cert_common_name_check = false
    };
    client = esp_http_client_init(&config);
}
static void send_http(esp_http_client_handle_t *client)
{
    char line[128];
    sprintf(line,
        "environmental_data,sensor=bme280,device_id=%s "
        "temperature=%.2f,humidity=%.1f,pressure=%.2f",
        EXAMPLE_H2E_IDENTIFIER,
        results.temp,
        results.hum,
        results.pres
    );

    ESP_ERROR_CHECK(esp_http_client_set_post_field(*client, line, strlen(line)));
    esp_http_client_set_header(*client, "Content-Type", "text/plain");

    // Build authorization header with token from config
    char auth_header[256];
    snprintf(auth_header, sizeof(auth_header), "Token %s", INFLUXDB_TOKEN);
    esp_http_client_set_header(*client, "Authorization", auth_header);
    esp_err_t err = esp_http_client_perform(*client);
    if (err == ESP_OK) {
        int status = esp_http_client_get_status_code(*client);
        int len = esp_http_client_get_content_length(*client);
        printf("Status = %d, len = %d\n", status, len);
        char buffer[256];
        int read_len = esp_http_client_read_response(*client, buffer, sizeof(buffer)-1);
        if (read_len > 0) {
            buffer[read_len] = 0;
            printf("Response: %s\n", buffer);
        }
    } else {
        printf("HTTP POST failed: %s\n", esp_err_to_name(err));
    }

}


void app_main(void)
{
    printf("CMeteo ESP32 - Copyright bismarckssr x PlayScopa.online\n");
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);

    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;

    gpio_set_level(LED_GPIO, 1);
    i2c_master_init(&bus_handle, &dev_handle);
    ESP_LOGI(TAG, "I2C initialized successfully");
    wifi_init();
    http_init();

    bme280_init_defaults(&dev_handle, 0x02, CONFIG_BME280_ENABLE_IIR);
    read_coefficients(&dev_handle);

    while (true) {
        set_mode(&dev_handle, 0x02);
        read_temp(&dev_handle);
        read_pressure(&dev_handle);
        read_humidity(&dev_handle);
        send_http(&client);
        vTaskDelay((CONFIG_MEASUREMENT_INTERVAL_SEC * 1000) / portTICK_PERIOD_MS);
    }
}
