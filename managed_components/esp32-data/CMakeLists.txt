# Component CMakeLists.txt for esp32-data (C++ version)

idf_component_register(
                    SRCS "sensor_data.cpp"
                    # "sensor_store.cpp" "sensor_api.cpp" "sensor_type_utils.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES 
                        freertos
                        esp_timer
                        nvs_flash
                        json
                    PRIV_REQUIRES 
                        heap
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add C++ standard library
target_link_libraries(${COMPONENT_LIB} INTERFACE stdc++)

