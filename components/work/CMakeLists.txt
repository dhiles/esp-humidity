idf_component_register(
    INCLUDE_DIRS "."
    
    SRCS "work.cpp" "bme280mgr.cpp"

    REQUIRES
        esp32-shared 
        json
        bme280
        common
        esp_lcd
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

