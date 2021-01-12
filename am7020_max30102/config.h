#ifndef _CONFIG_H
#define _CONFIG_H
#include <Arduino.h>

#define DEBUG_MODE true
#if DEBUG_MODE
#define debugSerial Serial
#define DEBUG_PRINTLN(x) debugSerial.println(x)
#define DEBUG_PRINT(x) debugSerial.print(x)
#else
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINT(x)
#endif

/* baudrate */
#define BAUDRATE_9600 9600
#define BAUDRATE_19200 19200
#define BAUDRATE_38400 38400
#define BAUDRATE_115200 115200

#define TINY_GSM_MODEM_SIM7020
/* Define the serial console for debug prints, if needed */
// #define TINY_GSM_DEBUG SerialMon
/* uncomment to dump all AT commands */
// #define DEBUG_DUMP_AT_COMMAND

#define UPLOAD_INTERVAL (1 * 60000)

#define SerialMon Serial
#define MONITOR_BAUDRATE 115200

/* Boards */
#define SerialAT Serial1
#define AM7020_BAUDRATE 115200
#define AM7020_RESET 4

// for taiwan mobile
#define APN "twm.nbiot"
#define BAND 28

/* ---mqtt config--- */
#define MQTT_BROKER "io.adafruit.com"
#define MQTT_PORT 1883

#define MQTT_USERNAME "<YOUR USERNAME>"
#define MQTT_PASSWORD "<YOUR AIO KEY>"

#define MAX30102_MAX_HR_TOPIC MQTT_USERNAME "/feeds/max30102.max-hr"
#define MAX30102_MID_HR_TOPIC MQTT_USERNAME "/feeds/max30102.mid-hr"
#define MAX30102_MIN_HR_TOPIC MQTT_USERNAME "/feeds/max30102.min-hr"
#define MAX30102_TEMP_TOPIC MQTT_USERNAME   "/feeds/max30102.temperature"

#endif /* _CONFIG_H */