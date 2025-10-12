#ifndef CONSTANTS_H
#define CONSTANTS_H

namespace WiFiConfig {
    constexpr char SSID[] = "tplink2dot4"; //"dishy2";
    constexpr char PASSWORD[] =  "hhhh4100"; // "elon$1971";
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


#endif // CONSTANTS_H
