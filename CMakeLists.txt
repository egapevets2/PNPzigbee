cmake_minimum_required(VERSION 3.22)

# Add esp_driver_i2c to the minimal components list
set(COMPONENTS 
    main 
    driver 
    nvs_flash 
    esp_zigbee_lib 
    zcl_responses
    esp_driver_ledc
    esp_driver_i2c
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(MINIMAL_BUILD ON)
project(PNPzigbee)