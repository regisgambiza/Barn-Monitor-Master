#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_SHT31.h>

// SHT30 on I2C:
// ESP32 GPIO9  -> SCL/T
// ESP32 GPIO8  -> SDA/RH
// ESP32 3.3V   -> VIN
// ESP32 GND    -> GND

typedef struct {
  float temp;
  float hum;
} SensorPacket;

typedef struct {
  float temp;
  float hum;
  float batt;
  float adc;
} SensorPacketV2;

static const uint8_t kBroadcastMac[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
static const uint32_t kSendIntervalMs = 1000;
static const uint8_t kSdaPin = 8;
static const uint8_t kSclPin = 9;
static const uint8_t kShtAddr = 0x44;
static const int kAdcPin = 2;
static const float kBatteryMultiplier = 1.48f;

SensorPacketV2 pkt;
unsigned long lastSend = 0;
float lastTemp = 22.0f;
float lastHum = 55.0f;

Adafruit_SHT31 sht31 = Adafruit_SHT31();

void onSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("ESP-NOW send: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void setup() {
  Serial.begin(115200);
  delay(300);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  Serial.print("Sensor MAC: ");
  Serial.println(WiFi.macAddress());

  Wire.begin(kSdaPin, kSclPin);
  if (!sht31.begin(kShtAddr)) {
    Serial.println("SHT30 not found on I2C (0x44). Check wiring.");
  } else {
    Serial.println("SHT30 OK");
  }

  analogReadResolution(12);
  analogSetPinAttenuation(kAdcPin, ADC_11db);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, kBroadcastMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

void loop() {
  unsigned long now = millis();
  if (now - lastSend < kSendIntervalMs) return;
  lastSend = now;

  uint32_t sum = 0;
  for (int i = 0; i < 20; i++) {
    sum += analogRead(kAdcPin);
    delay(3);
  }
  float adc = sum / 20.0f;
  float adcVoltage = (adc / 4095.0f) * 3.3f;
  float vbat = adcVoltage * kBatteryMultiplier;

  float t = sht31.readTemperature();
  float h = sht31.readHumidity();
  if (isfinite(t) && isfinite(h)) {
    lastTemp = t;
    lastHum = h;
  }

  pkt.temp = lastTemp;
  pkt.hum  = lastHum;
  pkt.batt = vbat;
  pkt.adc  = adc;

  esp_now_send(kBroadcastMac, (uint8_t*)&pkt, sizeof(pkt));
  Serial.printf("Sent temp=%.2fC hum=%.2f%% batt=%.2fV\n", pkt.temp, pkt.hum, pkt.batt);
}
