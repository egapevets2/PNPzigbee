idf_component_register(
    SRCS "I2Cmux.c" "Rotary.c" "unipolarStep.c" "Compass.c" "ssd1306.c" "i2c_manager.c" "serial_gateway.c" "discovery.c" "nvs_config.c" "Expand8.c" "I2Craw.c" "VL53L1X.c" "ledc_manager.c" "pwm_driver.c" "Stepper.c" "adc.c" "UpstreamQ.c" "GPIO_handler.c" "motor_driver.c" "esp_zb_light.c" 
         "light_driver.c"
         "mesh_serial.c"
         "mesh_zigbee.c"
         "serial_bridge.c"
         "apds9930.c" 
         "NeoPixel.c" 
    INCLUDE_DIRS "." "fonts"
    PRIV_REQUIRES esp_driver_gpio esp_driver_uart esp_driver_usb_serial_jtag esp_driver_i2c esp_driver_ledc esp_driver_spi nvs_flash esp_adc esp_wifi esp_event esp_netif esp_timer
)