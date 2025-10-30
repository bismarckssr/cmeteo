idf_component_register(SRCS "cmeteo.c"
                       PRIV_REQUIRES spi_flash esp_wifi nvs_flash driver esp_driver_i2c esp_http_client mbedtls
                       INCLUDE_DIRS "")
