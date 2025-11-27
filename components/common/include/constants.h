#ifndef CONSTANTS_H
#define CONSTANTS_H

#define GREEN_LED GPIO_NUM_10
#define BLUE_LED GPIO_NUM_11
#define RED_LED GPIO_NUM_6

// Hardcoded constants (compile-time)
#define BUFFER_SIZE 1024
#define DEFAULT_TIMEOUT_MS 5000

// Or use constexpr for C++ (compile-time evaluation)
constexpr int MAX_RETRIES = 3;
constexpr float SENSOR_THRESHOLD = 25.0f;

// For runtime-initialized "constants" (if needed), use extern
extern const char* APP_VERSION;  // Defined in a .cpp if value is set at runtime

namespace WiFiConfig {
    constexpr char SSID[] = "tplink2dot4"; //"dishy2";
    constexpr char PASSWORD[] =  "hhhh4100"; // "elon$1971";
    constexpr char MDNS_HOSTNAME[] = "humidoer";
    constexpr char MDNS_INSTANCENAME[] = "humidoer";
}

namespace MQTTConfig {
    constexpr char URI[] = "mqtt://192.168.1.101:1883";
    //constexpr char URI[] = "ws://rr260mqtt.canactiveindustries.com:8080";
    constexpr char CAM1[] = "acreage/cam1";
}

namespace FileSrv {
    constexpr char URL[] = "http://192.168.1.101:8081/upload";
    //constexpr char URL[] = "https://rr260filesrv.canactiveindustries.com/upload";
}

namespace deviceIPSrv {
    constexpr char SETIP_URL[] = "https://deviceip.canactiveindustries.com/setIP";
}


#endif // CONSTANTS_H