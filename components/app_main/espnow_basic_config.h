#ifndef ESPNOW_BASIC_CONFIG_H
#define ESPNOW_BASIC_CONFIG_H

#include <inttypes.h>
#include <stdbool.h>

// Destination MAC address
// The default address is the broadcast address, which will work out of the box, but the slave will assume every tx succeeds.
// Setting to the master's address will allow the slave to determine if sending succeeded or failed.
//   note: with default config, the master's WiFi driver will log this for you. eg. I (721) wifi:mode : sta (12:34:56:78:9a:bc)
#define MY_RECEIVER_MAC {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}

#define MY_ESPNOW_PMK "pmk1234567890123"
#define MY_ESPNOW_CHANNEL 6

// #define MY_ESPNOW_ENABLE_LONG_RANGE 1

#define MY_SLAVE_DEEP_SLEEP_TIME_MS 10000


typedef struct
{
    uint8_t sender_mac_addr[ESP_NOW_ETH_ALEN];
    const uint8_t *data;
    int len;
} recv_packet_t;


#endif // ESPNOW_BASIC_CONFIG_H