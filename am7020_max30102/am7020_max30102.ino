
#include "config.h"
#include <Arduino.h>

#include <PubSubClient.h>
#include <TinyGsmClient.h>

#include "MAX30105.h"
#include <Wire.h>

#include "heartRate.h"

MAX30105 particleSensor;

const byte RATE_SIZE = 4;        // Increase this for more averaging. 4 is good.
byte       rates[RATE_SIZE];     // Array of heart rates
byte       rateSpot = 0;
long       lastBeat = 0;     // Time at which the last beat occurred

float beatsPerMinute;
int   beatAvg;

#ifdef DEBUG_DUMP_AT_COMMAND
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm        modem(debugger, AM7020_RESET);
#else
// 建立 AM7020 modem（設定 Serial 及 EN Pin）
TinyGsm modem(SerialAT, AM7020_RESET);
#endif
// 在 modem 架構上建立 Tcp Client
TinyGsmClient tcpClient(modem);
// 在 Tcp Client 架構上建立 MQTT Client
PubSubClient mqttClient(MQTT_BROKER, MQTT_PORT, tcpClient);

void mqttConnect(void);
void nbConnect(void);
void bubbleSort(int arr[], int n);

void setup()
{
    Serial.begin(115200);     // initialize serial communication at 115200 bits per second:
    SerialAT.begin(BAUDRATE_115200);
    while (!Serial)
        ;

    // Initialize sensor
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST))     // Use default I2C port, 400kHz speed
    {
        Serial.println("MAX30105 was not found. Please check wiring/power. ");
        while (1)
            ;
    }
    Serial.println("Place your index finger on the sensor with steady pressure.");

    particleSensor.setup();                        // Configure sensor with default settings
    particleSensor.setPulseAmplitudeRed(0x0A);     // Turn Red LED to low to indicate sensor is running
    particleSensor.setPulseAmplitudeGreen(0);      // Turn off Green LED

    randomSeed(analogRead(A0));
    // AM7020 NBIOT 連線基地台
    nbConnect();
    // 設定 MQTT KeepAlive time 為 270 秒
    mqttClient.setKeepAlive(270);
}

void loop()
{
    static unsigned long timer_0 = 0, timer_1 = 0, timer_2 = 0;
    static int           data[10];
    static unsigned int  idx = 0;

    long irValue = particleSensor.getIR();

    if (checkForBeat(irValue) == true) {
        // We sensed a beat!
        long delta = millis() - lastBeat;
        lastBeat   = millis();

        beatsPerMinute = 60 / (delta / 1000.0);

        if (beatsPerMinute < 255 && beatsPerMinute > 20) {
            rates[rateSpot++] = (byte)beatsPerMinute;     // Store this reading in the array
            rateSpot %= RATE_SIZE;                        // Wrap variable

            // Take average of readings
            beatAvg = 0;
            for (byte x = 0; x < RATE_SIZE; x++)
                beatAvg += rates[x];
            beatAvg /= RATE_SIZE;
        }
    }
    if (millis() >= timer_2) {
        timer_2 = millis() + 50;
        Serial.print("IR=");
        Serial.print(irValue);
        Serial.print(", BPM=");
        Serial.print(beatsPerMinute);
        Serial.print(", Avg BPM=");
        Serial.print(beatAvg);

        if (irValue < 50000)
            Serial.print(" No finger?");

        Serial.println();
    }
    if (millis() >= timer_0 && irValue >= 50000 && beatAvg > 10) {
        timer_0     = millis() + UPLOAD_INTERVAL;
        data[idx++] = beatAvg;
    }
    // 檢查 MQTT Client 連線狀態
    if (millis() >= timer_1) {
        timer_1 = millis() + 5000;
        if (!mqttClient.connected()) {
            // 檢查 NBIOT 連線狀態
            if (!modem.isNetworkConnected()) {
                nbConnect();
            }
            SerialMon.println(F("=== MQTT NOT CONNECTED ==="));
            mqttConnect();
        }
    }
    if (idx > 9) {
        idx = 0;
        bubbleSort(data, 10);
        mqttClient.publish(MAX30102_MAX_HR_TOPIC, String(data[9]).c_str());
        mqttClient.publish(MAX30102_MID_HR_TOPIC, String((data[4] + data[5]) / 2).c_str());
        mqttClient.publish(MAX30102_MIN_HR_TOPIC, String(data[0]).c_str());
    }
    // MQTT Client polling
    mqttClient.loop();
}

/**
 * AM7020 NBIOT 連線基地台
 */
void nbConnect(void)
{
    debugSerial.println(F("Initializing modem..."));
    // 初始化 & 連線基地台
    while (!modem.init() || !modem.nbiotConnect(APN, BAND)) {
        debugSerial.print(F("."));
    };

    debugSerial.print(F("Waiting for network..."));
    // 等待網路連線
    while (!modem.waitForNetwork()) {
        debugSerial.print(F("."));
    }
    debugSerial.println(F(" success"));
}

/**
 * MQTT Client 連線
 */
void mqttConnect(void)
{
    SerialMon.print(F("Connecting to "));
    SerialMon.print(MQTT_BROKER);
    SerialMon.print(F("..."));

    /* Connect to MQTT Broker */
    // 亂數產生 MQTTID
    String mqttid = ("MQTTID_" + String(random(65536)));
    // MQTT Client 連線
    while (!mqttClient.connect(mqttid.c_str(), MQTT_USERNAME, MQTT_PASSWORD)) {
        SerialMon.println(F(" fail"));
    }
    SerialMon.println(F(" success"));
}

/**
 * 排序（小到大）
 */
void bubbleSort(int arr[], int n)
{
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < i; ++j) {
            if (arr[j] > arr[i]) {
                int temp = arr[j];
                arr[j]   = arr[i];
                arr[i]   = temp;
            }
        }
    }
}
