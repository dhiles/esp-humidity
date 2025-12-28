cmake_minimum_required(VERSION 3.10)

set(SRCS "src/mysystem.cpp")
set(REQUIRES_LIST 
    common 
    work 
    driver 
    nvs_flash 
    json mqtt 
    esp_netif
    esp_wifi 
    mdns 
    esp_http_client 
    esp_http_server 
    esp_psram 
    cxx 
    newlib
    app_update
    esp_hw_support 
    esp_https_ota
    esp32-data
)
list(APPEND SRCS src/myled.cpp)

# Conditionally add source files (and rare extras like camera)
if(CONFIG_ENABLE_CAM)
    list(APPEND SRCS src/send_picture.cpp)
    list(APPEND REQUIRES_LIST esp32-camera)  # Only camera-specific; psram already in base
endif()

if(CONFIG_ENABLE_MQTT)
    list(APPEND SRCS src/mymqtt.cpp)
endif()

if(CONFIG_ENABLE_WIFI)
    list(APPEND SRCS src/mywifi.cpp src/myntp.cpp src/nvs_json.cpp src/nvs_credentials.cpp src/webserver.cpp)
endif()

if(CONFIG_ENABLE_SLEEPLOOP)
    list(APPEND SRCS src/loop.cpp)
endif()

message(STATUS "CONFIG_ENABLE_BLE_PROVISIONING: ${CONFIG_ENABLE_BLE_PROVISIONING}")
message(STATUS "REQUIRES_LIST: ${REQUIRES_LIST}")

#if(CONFIG_ENABLE_BLE_PROVISIONING)
    message(STATUS "BLE Provisioning enabled - adding bt library")
    list(APPEND SRCS src/ble_provisioning.cpp)
    list(APPEND REQUIRES_LIST bt) 
#endif()
message(STATUS "REQUIRES_LIST: ${REQUIRES_LIST}")
# Dedup requires (defensive; harmless even if empty)
list(REMOVE_DUPLICATES REQUIRES_LIST)

# Register component
idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "include" "../../main"
    REQUIRES ${REQUIRES_LIST}
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    $<$<COMPILE_LANGUAGE:C>:-Wno-implicit-function-declaration>
    -Wno-error 
    -Wno-format
    -Wno-unused-but-set-parameter
    -Wno-cpp
    -Wno-missing-field-initializers
    -Wno-deprecated-declarations
    -Wno-conversion
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-unused-parameter>
)
