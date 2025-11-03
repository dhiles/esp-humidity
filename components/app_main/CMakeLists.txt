idf_component_register(
    SRCS 
        "main.cpp"
    
    INCLUDE_DIRS 
        "."
        "../../main"
    
    REQUIRES
        common
        work
        esp32-shared 
        esp_http_server
        esp_timer
        esp_lcd
        fatfs
        driver 
        bme280         
        bt

    PRIV_REQUIRES
        # Private dependencies
    
    EMBED_FILES
        # Files to embed in flash
)

# Optional: Retain your suppression options (C++-aware)
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

# Apply build flags equivalent to PlatformIO configuration
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    CONFIG_MQTT_TRANSPORT_OVER_WS=1
    CONFIG_ESP_IDF_MQTT=1
    BOARD_HAS_PSRAM=1
)
# In your main CMakeLists.txt
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DBOARD_HAS_PSRAM")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOARD_HAS_PSRAM")

# PSRAM configuration (equivalent to board_build.psram_type = opi)
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    CONFIG_SPIRAM=y
    CONFIG_SPIRAM_MODE_OPI=y
    CONFIG_SPIRAM_SPEED_80M=y
)