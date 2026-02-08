#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_sleep.h>
#include <Wire.h>
#include <Adafruit_SHT31.h>

// ============================================================================
// CONFIGURATION
// ============================================================================

#define DEBUG_OUTPUT

#ifdef DEBUG_OUTPUT
#define LOG(msg) Serial.print(msg)
#define LOGLN(msg) Serial.println(msg)
#define LOGNL() Serial.println()
#define LOGF(...) Serial.printf(__VA_ARGS__)
#else
#define LOG(msg) do {} while (0)
#define LOGLN(msg) do {} while (0)
#define LOGNL() do {} while (0)
#define LOGF(...) do {} while (0)
#endif

// Power modes
#define USE_DEEP_SLEEP true
#define SLEEP_DURATION_SEC 60

// Sensor thresholds
#define SEND_ON_CHANGE_ONLY false
#define TEMP_CHANGE_THRESHOLD 0.5f
#define HUM_CHANGE_THRESHOLD 2.0f

// SHT30 on I2C:
// ESP32 GPIO9  -> SCL/T
// ESP32 GPIO8  -> SDA/RH
// ESP32 3.3V   -> VIN
// ESP32 GND    -> GND

typedef struct {
  float temp;
  float hum;
  float batt;
  float adc;
} SensorPacketV2;

static const uint8_t kBroadcastMac[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
static const uint32_t kSendIntervalMs = USE_DEEP_SLEEP ? 100 : 60000;
static const uint8_t kMaxSensorFails = 5;
static const uint8_t kMaxSendFails = 3;
static const uint8_t kSdaPin = 8;
static const uint8_t kSclPin = 9;
static const uint8_t kShtAddr = 0x44;
static const uint8_t kAdcPin = 2;
static const float kAdcRefV = 3.3f;
static const float kAdcMax = 4095.0f;
static const float kBattRTop = 10000.0f;
static const float kBattRBottom = 10000.0f;
static const float kBattDivider = (kBattRTop + kBattRBottom) / kBattRBottom;
static const float kAdcCal = 0.712f; // Adjust to match multimeter (e.g. 1.03f)

SensorPacketV2 pkt;
unsigned long lastSend = 0;
float lastTemp = 22.0f;
float lastHum = 55.0f;

Adafruit_SHT31 sht31 = Adafruit_SHT31();

// ============================================================================
// RTC MEMORY (persists across deep sleep)
// ============================================================================

RTC_DATA_ATTR uint8_t rtcBootCount = 0;
RTC_DATA_ATTR float rtcLastTemp = 22.0f;
RTC_DATA_ATTR float rtcLastHum = 55.0f;
RTC_DATA_ATTR uint8_t rtcSensorFailCount = 0;
RTC_DATA_ATTR uint8_t rtcSendFailCount = 0;

static void logHeap(const char* label) {
  LOGF("%s: freeHeap=%u totalHeap=%u minFreeHeap=%u\n",
                label,
                (unsigned)ESP.getFreeHeap(),
                (unsigned)ESP.getHeapSize(),
                (unsigned)ESP.getMinFreeHeap());
}

static void logChannel() {
  uint8_t primary = 0;
  wifi_second_chan_t secondary = WIFI_SECOND_CHAN_NONE;
  esp_err_t res = esp_wifi_get_channel(&primary, &secondary);
  LOG("WiFi: channel -> ");
  LOG(res == ESP_OK ? "OK " : "FAIL ");
  if (res == ESP_OK) {
    LOGF("(primary=%u secondary=%d)\n", (unsigned)primary, (int)secondary);
  } else {
    LOGNL();
  }
}

void onSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  LOG("ESP-NOW send status: ");
  LOGLN(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
  if (status == ESP_NOW_SEND_SUCCESS) {
    rtcSendFailCount = 0;
  } else {
    rtcSendFailCount++;
  }
  if (mac_addr) {
    LOGF("  Last peer: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  mac_addr[0], mac_addr[1], mac_addr[2],
                  mac_addr[3], mac_addr[4], mac_addr[5]);
  }
}

static bool hasSignificantChange(float newTemp, float newHum) {
  if (!SEND_ON_CHANGE_ONLY) return true;
  float tempDiff = fabsf(newTemp - lastTemp);
  float humDiff = fabsf(newHum - lastHum);
  LOGF("Change check: dT=%.2fC dH=%.2f%% thresholds=%.2fC/%.2f%%\n",
       tempDiff, humDiff, TEMP_CHANGE_THRESHOLD, HUM_CHANGE_THRESHOLD);
  return (tempDiff >= TEMP_CHANGE_THRESHOLD) || (humDiff >= HUM_CHANGE_THRESHOLD);
}

static void enterDeepSleep() {
  rtcLastTemp = lastTemp;
  rtcLastHum = lastHum;
  rtcSensorFailCount = (rtcSensorFailCount > kMaxSensorFails) ? kMaxSensorFails : rtcSensorFailCount;

  LOGF("Sleep: enter deep sleep for %us (boot=%u sensor_fail=%u send_fail=%u)\n",
       (unsigned)SLEEP_DURATION_SEC,
       (unsigned)rtcBootCount,
       (unsigned)rtcSensorFailCount,
       (unsigned)rtcSendFailCount);
  esp_now_deinit();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_DURATION_SEC * 1000000ULL);
  delay(100);
  esp_deep_sleep_start();
}

static void readAndSendSensor() {
  unsigned long tStart = micros();
  LOGLN("Read: SHT30");
  float t = sht31.readTemperature();
  float h = sht31.readHumidity();
  unsigned long tRead = micros();

  bool readOk = isfinite(t) && isfinite(h);
  if (readOk) {
    lastTemp = t;
    lastHum = h;
    rtcSensorFailCount = 0;
    LOGF("Read: OK temp=%.2fC hum=%.2f%%\n", t, h);
  } else {
    rtcSensorFailCount++;
    LOGF("Read: FAIL (count=%u, using last)\n", (unsigned)rtcSensorFailCount);
  }

  pkt.temp = lastTemp;
  pkt.hum  = lastHum;
  pkt.adc  = (float)analogRead(kAdcPin);
  pkt.batt = (pkt.adc / kAdcMax) * kAdcRefV * kBattDivider * kAdcCal;
  LOGF("ADC: raw=%.0f pin=%u batt=%.2fV cal=%.3f\n", pkt.adc, (unsigned)kAdcPin, pkt.batt, kAdcCal);

  if (!readOk || hasSignificantChange(pkt.temp, pkt.hum) || rtcBootCount <= 1) {
    LOGF("Send: temp=%.2fC hum=%.2f%% adc=%.0f size=%u\n",
                  pkt.temp, pkt.hum, pkt.adc, (unsigned)sizeof(pkt));
    esp_err_t res = esp_now_send(kBroadcastMac, (uint8_t*)&pkt, sizeof(pkt));
    LOG("Send: esp_now_send -> ");
    LOGLN(res == ESP_OK ? "OK" : "FAIL");
    if (res != ESP_OK) {
      rtcSendFailCount++;
    }
    unsigned long tSent = micros();
    LOGF("Timing: read=%luus send_call=%luus loop_total=%luus\n",
                  (unsigned long)(tRead - tStart),
                  (unsigned long)(tSent - tRead),
                  (unsigned long)(tSent - tStart));
  } else {
    LOGLN("Send: skipped (no significant change)");
  }

  logHeap("Heap loop");
  logChannel();
  LOGF("State: boot=%u lastTemp=%.2fC lastHum=%.2f%% sensor_fail=%u send_fail=%u\n",
       (unsigned)rtcBootCount,
       lastTemp,
       lastHum,
       (unsigned)rtcSensorFailCount,
       (unsigned)rtcSendFailCount);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  rtcBootCount++;
  lastTemp = rtcLastTemp;
  lastHum = rtcLastHum;

  LOGLN("Boot: sensor ESP-NOW sender");
  LOGF("Boot: reason=%u\n", (unsigned)esp_sleep_get_wakeup_cause());
  LOGF("RTC: lastTemp=%.2fC lastHum=%.2f%% bootCount=%u sensorFail=%u sendFail=%u\n",
       rtcLastTemp,
       rtcLastHum,
       (unsigned)rtcBootCount,
       (unsigned)rtcSensorFailCount,
       (unsigned)rtcSendFailCount);
  LOGLN("Init: set WiFi STA mode");
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  LOGLN("Init: WiFi sleep disabled");
  logHeap("Heap after WiFi");
  logChannel();

  LOG("Sensor MAC: ");
  LOGLN(WiFi.macAddress());

  LOGF("I2C: SDA=%u SCL=%u addr=0x%02X\n",
                (unsigned)kSdaPin, (unsigned)kSclPin, (unsigned)kShtAddr);
  LOGF("ADC: pin=%u ref=%.2fV divider=%.2fx\n", (unsigned)kAdcPin, kAdcRefV, kBattDivider);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  Wire.begin(kSdaPin, kSclPin);
  if (!sht31.begin(kShtAddr)) {
    LOGLN("ERROR: SHT30 not found on I2C. Check wiring.");
  } else {
    LOGLN("SHT30 OK");
  }

  LOGLN("Init: ESP-NOW begin");
  if (esp_now_init() != ESP_OK) {
    LOGLN("ERROR: ESP-NOW init failed");
    return;
  }

  esp_now_register_send_cb(onSent);
  LOGLN("Init: ESP-NOW send callback registered");

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, kBroadcastMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_err_t peerRes = esp_now_add_peer(&peerInfo);
  LOG("Init: add broadcast peer -> ");
  LOGLN(peerRes == ESP_OK ? "OK" : "FAIL");
  LOGF("Init: send interval %lu ms\n", (unsigned long)kSendIntervalMs);
  LOGF("Init: deep_sleep=%s sleep_sec=%u send_on_change=%s\n",
       USE_DEEP_SLEEP ? "true" : "false",
       (unsigned)SLEEP_DURATION_SEC,
       SEND_ON_CHANGE_ONLY ? "true" : "false");
  logHeap("Heap after ESP-NOW");
}

void loop() {
  unsigned long now = millis();
  if (!USE_DEEP_SLEEP) {
    if (now - lastSend < kSendIntervalMs) return;
    lastSend = now;
    readAndSendSensor();
    return;
  }

  // Deep sleep mode: read, send, then sleep
  if (now - lastSend < kSendIntervalMs) return;
  lastSend = now;
  readAndSendSensor();
  delay(100);
  enterDeepSleep();
}
