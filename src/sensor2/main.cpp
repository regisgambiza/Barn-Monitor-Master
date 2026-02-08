#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// Simple ESP-NOW dummy sender for sensor2
// Broadcasts temp/hum every 2 seconds.

typedef struct {
  float temp;
  float hum;
} SensorPacket;

static const uint8_t kBroadcastMac[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
static const uint32_t kSendIntervalMs = 2000;

SensorPacket pkt;
unsigned long lastSend = 0;
uint32_t counter = 0;

static void logHeap(const char* label) {
  Serial.printf("%s: freeHeap=%u totalHeap=%u minFreeHeap=%u\n",
                label,
                (unsigned)ESP.getFreeHeap(),
                (unsigned)ESP.getHeapSize(),
                (unsigned)ESP.getMinFreeHeap());
}

static void logChannel() {
  uint8_t primary = 0;
  wifi_second_chan_t secondary = WIFI_SECOND_CHAN_NONE;
  esp_err_t res = esp_wifi_get_channel(&primary, &secondary);
  Serial.print("WiFi: channel -> ");
  Serial.print(res == ESP_OK ? "OK " : "FAIL ");
  if (res == ESP_OK) {
    Serial.printf("(primary=%u secondary=%d)\n", (unsigned)primary, (int)secondary);
  } else {
    Serial.println();
  }
}

void onSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("ESP-NOW send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
  if (mac_addr) {
    Serial.printf("  Last peer: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  mac_addr[0], mac_addr[1], mac_addr[2],
                  mac_addr[3], mac_addr[4], mac_addr[5]);
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("Boot: sensor2 ESP-NOW dummy sender");
  Serial.println("Init: set WiFi STA mode");
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  Serial.println("Init: WiFi sleep disabled");
  logHeap("Heap after WiFi");
  logChannel();

  Serial.print("Sensor2 MAC: ");
  Serial.println(WiFi.macAddress());

  Serial.println("Init: ESP-NOW begin");
  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: ESP-NOW init failed");
    return;
  }

  esp_now_register_send_cb(onSent);
  Serial.println("Init: ESP-NOW send callback registered");

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, kBroadcastMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_err_t peerRes = esp_now_add_peer(&peerInfo);
  Serial.print("Init: add broadcast peer -> ");
  Serial.println(peerRes == ESP_OK ? "OK" : "FAIL");
  Serial.printf("Init: send interval %lu ms\n", (unsigned long)kSendIntervalMs);
  logHeap("Heap after ESP-NOW");
}

void loop() {
  unsigned long now = millis();
  if (now - lastSend < kSendIntervalMs) return;
  lastSend = now;

  unsigned long tStart = micros();

  // Dummy data that changes slowly
  pkt.temp = 20.0f + (counter % 10);
  pkt.hum  = 50.0f + (counter % 20);

  unsigned long tPrep = micros();
  Serial.printf("Send: seq=%lu temp=%.1fC hum=%.1f%% size=%u\n",
                (unsigned long)counter, pkt.temp, pkt.hum, (unsigned)sizeof(pkt));
  esp_err_t res = esp_now_send(kBroadcastMac, (uint8_t*)&pkt, sizeof(pkt));
  Serial.print("Send: esp_now_send -> ");
  Serial.println(res == ESP_OK ? "OK" : "FAIL");
  unsigned long tSent = micros();
  Serial.printf("Timing: prep=%luus send_call=%luus loop_total=%luus\n",
                (unsigned long)(tPrep - tStart),
                (unsigned long)(tSent - tPrep),
                (unsigned long)(tSent - tStart));
  logHeap("Heap loop");
  logChannel();

  counter++;
}
