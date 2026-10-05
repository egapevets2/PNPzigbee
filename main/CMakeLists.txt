idf_component_register(
    SRCS "I2Craw.c" "VL53L1X.c" "BLDC_driver.c" "ledc_manager.c" "pwm_driver.c" "Stepper.c" "adc.c" "UpstreamQ.c" "GPIO_handler.c" "motor_driver.c" "esp_zb_light.c" 
         "light_driver.c"
         "mesh_serial.c"
         "mesh_zigbee.c"
         "serial_bridge.c"
         "apds9930.c" 
         "NeoPixel.c" 
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_i2c esp_driver_ledc nvs_flash esp_adc examples_utils esp_wifi esp_event esp_netif
)