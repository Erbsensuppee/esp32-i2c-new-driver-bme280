idf_component_register(SRCS "esp32-i2c-new-driver-bme280.c" "bme280.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_i2c)
