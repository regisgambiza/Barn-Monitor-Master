#include <WiFi.h>
#include <esp_now.h>
#include <WebServer.h>
#include <WiFiManager.h>
#include <Preferences.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <time.h>

WebServer server(80);
Preferences prefs;

#define MAX_SENSORS 5
#define HISTORY 200
#define MAX_DISCOVERED 10
#define DEFAULT_OFFLINE_AFTER_MS 300000
#define DEFAULT_DISCOVERED_STALE_MS 300000
#define DEFAULT_LOG_INTERVAL_MS 60000

struct Sensor {
  String mac;
  String zone;
  float temp = NAN;
  float hum = NAN;
  unsigned long lastUpdate = 0;
  uint32_t packetCount = 0;
  int rssi = -127;
  uint8_t lastAckCmd = 0;
  uint8_t lastAckStatus = 0;
  unsigned long lastAckMs = 0;
  uint32_t cfgInterval = 0;
  uint8_t cfgRetry = 0;
  uint8_t cfgTx = 0;
  bool cfgOta = false;
  uint32_t cfgIp = 0;
  unsigned long lastStatusMs = 0;

  float tempHistory[HISTORY] = {0};
  float humHistory[HISTORY] = {0};
  int historyIndex = 0;

  bool alarmActive = false;
};

Sensor sensors[MAX_SENSORS];
int sensorCount = 0;

struct DiscoveredSensor {
  String mac;
  float temp = NAN;
  float hum = NAN;
  unsigned long lastSeen = 0;
  int rssi = -127;
  uint8_t lastAckCmd = 0;
  uint8_t lastAckStatus = 0;
  unsigned long lastAckMs = 0;
  uint32_t cfgInterval = 0;
  uint8_t cfgRetry = 0;
  uint8_t cfgTx = 0;
  bool cfgOta = false;
  uint32_t cfgIp = 0;
  unsigned long lastStatusMs = 0;
};

DiscoveredSensor discovered[MAX_DISCOVERED];
int discoveredCount = 0;

unsigned long lastLog = 0;

struct Stage {
  const char* name;
  float tMin;
  float tMax;
  float hMin;
  float hMax;
};

const Stage stageDefaults[] = {
  {"Yellowing",   32.0f, 38.0f, 75.0f, 85.0f},
  {"Leaf Drying", 38.0f, 45.0f, 60.0f, 70.0f},
  {"Stem Drying", 45.0f, 55.0f, 35.0f, 45.0f}
};
const int STAGE_COUNT = sizeof(stageDefaults) / sizeof(stageDefaults[0]);
Stage stages[STAGE_COUNT];
int activeStage = 0;
unsigned long offlineAfterMs = DEFAULT_OFFLINE_AFTER_MS;
unsigned long discoveredStaleMs = DEFAULT_DISCOVERED_STALE_MS;
unsigned long logIntervalMs = DEFAULT_LOG_INTERVAL_MS;

// ---------------- MAC STRING ----------------
String macToString(const uint8_t *mac) {
  char buf[18];
  sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X",
          mac[0], mac[1], mac[2],
          mac[3], mac[4], mac[5]);
  return String(buf);
}

// ---------------- FIND SENSOR ----------------
int findSensor(const String& mac) {
  for (int i = 0; i < sensorCount; i++)
    if (sensors[i].mac == mac) return i;
  return -1;
}

int findDiscovered(const String& mac) {
  for (int i = 0; i < discoveredCount; i++)
    if (discovered[i].mac == mac) return i;
  return -1;
}

void upsertDiscovered(const String& mac, float temp, float hum, int rssi) {
  int idx = findDiscovered(mac);
  if (idx == -1) {
    if (discoveredCount >= MAX_DISCOVERED) return;
    idx = discoveredCount++;
    discovered[idx].mac = mac;
  }
  discovered[idx].temp = temp;
  discovered[idx].hum = hum;
  discovered[idx].rssi = rssi;
  discovered[idx].lastSeen = millis();
}

void removeDiscoveredAt(int idx) {
  if (idx < 0 || idx >= discoveredCount) return;
  for (int i = idx; i < discoveredCount - 1; i++) {
    discovered[i] = discovered[i + 1];
  }
  discoveredCount--;
}

void removeSensorAt(int idx) {
  if (idx < 0 || idx >= sensorCount) return;
  for (int i = idx; i < sensorCount - 1; i++) {
    sensors[i] = sensors[i + 1];
  }
  sensorCount--;
}

bool roleTaken(const String& role, const String& exceptMac = "") {
  for (int i = 0; i < sensorCount; i++) {
    if (sensors[i].zone == role && sensors[i].mac != exceptMac) return true;
  }
  return false;
}
void saveSensors() {
  prefs.begin("barn", false);
  prefs.putInt("count", sensorCount);
  for (int i = 0; i < sensorCount; i++) {
    String keyMac = "mac" + String(i);
    String keyZone = "role" + String(i);
    prefs.putString(keyMac.c_str(), sensors[i].mac);
    prefs.putString(keyZone.c_str(), sensors[i].zone);
  }
  prefs.end();
}

void loadSensors() {
  prefs.begin("barn", false);
  sensorCount = prefs.getInt("count", 0);
  if (sensorCount > MAX_SENSORS) sensorCount = MAX_SENSORS;
  for (int i = 0; i < sensorCount; i++) {
    String keyMac = "mac" + String(i);
    String keyZone = "role" + String(i);
    sensors[i].mac = prefs.getString(keyMac.c_str(), "");
    if (prefs.isKey(keyZone.c_str())) {
      sensors[i].zone = prefs.getString(keyZone.c_str(), sensors[i].mac);
    } else {
      sensors[i].zone = sensors[i].mac;
    }
  }
  prefs.end();
}

void saveStage() {
  prefs.begin("barn", false);
  prefs.putInt("stage", activeStage);
  prefs.end();
}

void loadStage() {
  prefs.begin("barn", false);
  activeStage = prefs.getInt("stage", 0);
  prefs.end();
  if (activeStage < 0 || activeStage >= STAGE_COUNT) activeStage = 0;
}

void saveSettings() {
  prefs.begin("barn", false);
  prefs.putULong("offlineMs", offlineAfterMs);
  prefs.putULong("pruneMs", discoveredStaleMs);
  prefs.putULong("logMs", logIntervalMs);
  for (int i = 0; i < STAGE_COUNT; i++) {
    String k = "st" + String(i);
    prefs.putFloat((k + "tmin").c_str(), stages[i].tMin);
    prefs.putFloat((k + "tmax").c_str(), stages[i].tMax);
    prefs.putFloat((k + "hmin").c_str(), stages[i].hMin);
    prefs.putFloat((k + "hmax").c_str(), stages[i].hMax);
  }
  prefs.end();
}

void loadSettings() {
  prefs.begin("barn", false);
  offlineAfterMs = prefs.getULong("offlineMs", DEFAULT_OFFLINE_AFTER_MS);
  discoveredStaleMs = prefs.getULong("pruneMs", DEFAULT_DISCOVERED_STALE_MS);
  logIntervalMs = prefs.getULong("logMs", DEFAULT_LOG_INTERVAL_MS);
  for (int i = 0; i < STAGE_COUNT; i++) {
    String k = "st" + String(i);
    stages[i].name = stageDefaults[i].name;
    String ktmin = k + "tmin";
    String ktmax = k + "tmax";
    String khmin = k + "hmin";
    String khmax = k + "hmax";
    stages[i].tMin = prefs.isKey(ktmin.c_str()) ? prefs.getFloat(ktmin.c_str(), stageDefaults[i].tMin) : stageDefaults[i].tMin;
    stages[i].tMax = prefs.isKey(ktmax.c_str()) ? prefs.getFloat(ktmax.c_str(), stageDefaults[i].tMax) : stageDefaults[i].tMax;
    stages[i].hMin = prefs.isKey(khmin.c_str()) ? prefs.getFloat(khmin.c_str(), stageDefaults[i].hMin) : stageDefaults[i].hMin;
    stages[i].hMax = prefs.isKey(khmax.c_str()) ? prefs.getFloat(khmax.c_str(), stageDefaults[i].hMax) : stageDefaults[i].hMax;
  }
  prefs.end();
}

bool isReadingValid(float t, float h) {
  if (isnan(t) || isnan(h)) return false;
  if (t < -20.0f || t > 80.0f) return false;
  if (h < 0.0f || h > 100.0f) return false;
  return true;
}

String normalizeRole(const String& role) {
  String r = role;
  r.toLowerCase();
  if (r == "top") return "Top sensor";
  if (r == "middle") return "Middle sensor";
  if (r == "bottom") return "Bottom sensor";
  return "";
}

void updateAlarm(Sensor& s) {
  const Stage& st = stages[activeStage];
  bool outOfRange = (s.temp < st.tMin || s.temp > st.tMax || s.hum < st.hMin || s.hum > st.hMax);
  if (outOfRange && !s.alarmActive) {
    s.alarmActive = true;
    Serial.println("Alarm triggered");
  } else if (!outOfRange && s.alarmActive) {
    s.alarmActive = false;
  }
}

void buildAlarmInfo(const Sensor& s, String& reason, String& solution) {
  const Stage& st = stages[activeStage];
  bool tempLow = s.temp < st.tMin;
  bool tempHigh = s.temp > st.tMax;
  bool humLow = s.hum < st.hMin;
  bool humHigh = s.hum > st.hMax;

  reason = "";
  solution = "";

  if (tempLow) {
    reason += "Temp low. ";
    solution += "Increase heat / reduce ventilation. ";
  }
  if (tempHigh) {
    reason += "Temp high. ";
    solution += "Reduce heat / increase ventilation. ";
  }
  if (humLow) {
    reason += "Humidity low. ";
    solution += "Add moisture / reduce ventilation. ";
  }
  if (humHigh) {
    reason += "Humidity high. ";
    solution += "Increase ventilation / dehumidify. ";
  }

  reason.trim();
  solution.trim();
}

String todayLogFile() {
  time_t now = time(nullptr);
  struct tm timeinfo;
  if (now > 1600000000 && localtime_r(&now, &timeinfo)) {
    char buf[20];
    strftime(buf, sizeof(buf), "/log_%Y%m%d.csv", &timeinfo);
    return String(buf);
  }
  return String("/log.csv");
}

// ---------------- ESP NOW RECEIVE ----------------
typedef struct {
  float temp;
  float hum;
} SensorPacket;

typedef struct {
  uint8_t mac[6];
  float temp;
  float hum;
} SensorPacketWithMac;

typedef struct {
  uint8_t type;
  uint8_t cmd;
  uint16_t reserved;
  uint32_t value;
} SensorCmdPacket;

typedef struct {
  uint8_t type;
  uint8_t cmd;
  char value[64];
} SensorCmdStrPacket;

typedef struct {
  uint8_t type;
  uint8_t cmd;
  uint8_t status;
  uint8_t reserved;
  uint32_t value;
} SensorAckPacket;

typedef struct {
  uint8_t type;
  uint8_t ota;
  uint8_t retry;
  uint8_t tx;
  uint32_t interval;
  uint32_t ip;
} SensorStatusPacket;

enum {
  CMD_SET_INTERVAL = 1,
  CMD_SET_RETRY = 2,
  CMD_SET_TXPOWER = 3,
  CMD_SET_OTA = 4,
  CMD_SET_WIFI_SSID = 5,
  CMD_SET_WIFI_PASS = 6,
  MSG_TYPE_CMD = 2,
  MSG_TYPE_ACK = 3,
  MSG_TYPE_STATUS = 4
};

bool parseMacString(const String& mac, uint8_t out[6]) {
  int values[6];
  if (sscanf(mac.c_str(), "%x:%x:%x:%x:%x:%x",
             &values[0], &values[1], &values[2],
             &values[3], &values[4], &values[5]) != 6) {
    return false;
  }
  for (int i = 0; i < 6; i++) out[i] = (uint8_t) values[i];
  return true;
}

bool ensurePeer(const uint8_t* mac) {
  if (esp_now_is_peer_exist(mac)) return true;
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, mac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  return esp_now_add_peer(&peerInfo) == ESP_OK;
}

bool sendSensorCmdStr(const String& macStr, uint8_t cmd, const String& value) {
  uint8_t mac[6];
  if (!parseMacString(macStr, mac)) return false;
  if (!ensurePeer(mac)) return false;
  SensorCmdStrPacket pkt = {};
  pkt.type = MSG_TYPE_CMD;
  pkt.cmd = cmd;
  value.toCharArray(pkt.value, sizeof(pkt.value));
  return esp_now_send(mac, (uint8_t*)&pkt, sizeof(pkt)) == ESP_OK;
}

bool sendSensorCmd(const String& macStr, uint8_t cmd, uint32_t value) {
  uint8_t mac[6];
  if (!parseMacString(macStr, mac)) return false;
  if (!ensurePeer(mac)) return false;
  SensorCmdPacket pkt = { MSG_TYPE_CMD, cmd, 0, value };
  return esp_now_send(mac, (uint8_t*)&pkt, sizeof(pkt)) == ESP_OK;
}

void handlePacket(const uint8_t *mac, const uint8_t *data, int len, int rssi) {
  float temp = NAN;
  float hum = NAN;
  String macStr = macToString(mac);

  if (len == sizeof(SensorStatusPacket)) {
    SensorStatusPacket st;
    memcpy(&st, data, sizeof(st));
    if (st.type == MSG_TYPE_STATUS) {
      int idx = findSensor(macStr);
      if (idx != -1) {
        sensors[idx].cfgInterval = st.interval;
        sensors[idx].cfgRetry = st.retry;
        sensors[idx].cfgTx = st.tx;
        sensors[idx].cfgOta = st.ota ? true : false;
        sensors[idx].cfgIp = st.ip;
        sensors[idx].lastStatusMs = millis();
      }
      return;
    }
  }

  if (len == sizeof(SensorAckPacket)) {
    SensorAckPacket ack;
    memcpy(&ack, data, sizeof(ack));
    if (ack.type == MSG_TYPE_ACK) {
      int idx = findSensor(macStr);
      if (idx != -1) {
        sensors[idx].lastAckCmd = ack.cmd;
        sensors[idx].lastAckStatus = ack.status;
        sensors[idx].lastAckMs = millis();
      }
      return;
    }
  }

  if (len == sizeof(SensorPacket)) {
    SensorPacket pkt;
    memcpy(&pkt, data, sizeof(pkt));
    temp = pkt.temp;
    hum = pkt.hum;
  } else if (len == sizeof(SensorPacketWithMac)) {
    SensorPacketWithMac pkt;
    memcpy(&pkt, data, sizeof(pkt));
    temp = pkt.temp;
    hum = pkt.hum;
    macStr = macToString(pkt.mac);
  } else {
    return;
  }

  bool valid = isReadingValid(temp, hum);
  int idx = findSensor(macStr);
  if (idx == -1) {
    if (valid) upsertDiscovered(macStr, temp, hum, rssi);
    else upsertDiscovered(macStr, NAN, NAN, rssi);
    return;
  }

  sensors[idx].lastUpdate = millis();
  sensors[idx].packetCount++;
  sensors[idx].rssi = rssi;

  if (!valid) return;

  sensors[idx].temp = temp;
  sensors[idx].hum = hum;

  int h = sensors[idx].historyIndex;
  sensors[idx].tempHistory[h] = temp;
  sensors[idx].humHistory[h] = hum;
  sensors[idx].historyIndex = (h + 1) % HISTORY;

  updateAlarm(sensors[idx]);

  Serial.println("Sensor updated");
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  int rssi = info ? info->rx_ctrl.rssi : -127;
  const uint8_t *src = info ? info->src_addr : nullptr;
  if (!src) return;
  handlePacket(src, data, len, rssi);
}
#else
void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
  handlePacket(mac, data, len, -127);
}
#endif

// ---------------- LOGGING ----------------
void logData() {
  String filename = todayLogFile();
  File f = SPIFFS.open(filename, FILE_APPEND);
  if (!f) {
    Serial.println("Log save failed (SPIFFS open)");
    return;
  }

  time_t now = time(nullptr);
  unsigned long t = (now > 1600000000) ? (unsigned long)now : (millis() / 1000);

  for (int i = 0; i < sensorCount; i++) {
    f.printf("%lu,%s,%s,%.1f,%.1f\n",
      t,
      stages[activeStage].name,
      sensors[i].mac.c_str(),
      sensors[i].temp,
      sensors[i].hum);
  }

  f.close();
  Serial.println("Log saved");
}

// ---------------- JSON DATA ----------------
void handleData() {
  JsonDocument doc;
  doc["stage"] = stages[activeStage].name;
  doc["stageIndex"] = activeStage;
  doc["tMin"] = stages[activeStage].tMin;
  doc["tMax"] = stages[activeStage].tMax;
  doc["hMin"] = stages[activeStage].hMin;
  doc["hMax"] = stages[activeStage].hMax;

  JsonArray arr = doc["sensors"].to<JsonArray>();

  for (int i = 0; i < sensorCount; i++) {
    JsonObject s = arr.add<JsonObject>();
    s["mac"] = sensors[i].mac;
    s["zone"] = sensors[i].zone;
    s["temp"] = sensors[i].temp;
    s["hum"] = sensors[i].hum;
    s["alarm"] = sensors[i].alarmActive;
    s["lastUpdate"] = sensors[i].lastUpdate ? (millis() - sensors[i].lastUpdate) / 1000 : 0;
    s["offline"] = sensors[i].lastUpdate ? ((millis() - sensors[i].lastUpdate) > offlineAfterMs) : true;
    s["packets"] = sensors[i].packetCount;
    s["rssi"] = sensors[i].rssi;

    String reason, solution;
    if (sensors[i].alarmActive) {
      buildAlarmInfo(sensors[i], reason, solution);
    }
    s["alarmReason"] = reason;
    s["alarmSolution"] = solution;

    JsonArray tHist = s["tHist"].to<JsonArray>();
    JsonArray hHist = s["hHist"].to<JsonArray>();

    for (int j = 0; j < HISTORY; j++) {
      int idx = (sensors[i].historyIndex + j) % HISTORY;
      tHist.add(sensors[i].tempHistory[idx]);
      hHist.add(sensors[i].humHistory[idx]);
    }
  }

  JsonArray avail = doc["available"].to<JsonArray>();
  for (int i = 0; i < discoveredCount; ) {
    if (millis() - discovered[i].lastSeen > discoveredStaleMs) {
      removeDiscoveredAt(i);
      continue;
    }
    JsonObject d = avail.add<JsonObject>();
    d["mac"] = discovered[i].mac;
    d["temp"] = discovered[i].temp;
    d["hum"] = discovered[i].hum;
    d["rssi"] = discovered[i].rssi;
    d["lastSeen"] = (millis() - discovered[i].lastSeen) / 1000;
    i++;
  }

  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void handleSetZone() {
  if (!server.hasArg("mac") || !server.hasArg("name")) {
    server.send(400, "text/plain", "Missing args");
    return;
  }
  String mac = server.arg("mac");
  String name = normalizeRole(server.arg("name"));
  if (name.length() == 0) {
    server.send(400, "text/plain", "Invalid role");
    return;
  }
  if (roleTaken(name, mac)) {
    server.send(409, "text/plain", "Role already assigned");
    return;
  }
  int idx = findSensor(mac);
  if (idx == -1) {
    server.send(404, "text/plain", "Sensor not found");
    return;
  }
  sensors[idx].zone = name;
  saveSensors();
  server.send(200, "text/plain", "OK");
}

void handlePair() {
  if (!server.hasArg("mac") || !server.hasArg("role")) {
    server.send(400, "text/plain", "Missing args");
    return;
  }
  String mac = server.arg("mac");
  String role = normalizeRole(server.arg("role"));
  if (role.length() == 0) {
    server.send(400, "text/plain", "Invalid role");
    return;
  }
  if (roleTaken(role, mac)) {
    server.send(409, "text/plain", "Role already assigned");
    return;
  }

  int idx = findSensor(mac);
  if (idx == -1) {
    if (sensorCount >= MAX_SENSORS) {
      server.send(400, "text/plain", "Max sensors reached");
      return;
    }
    idx = sensorCount++;
    sensors[idx].mac = mac;
  }

  sensors[idx].zone = role;
  sensors[idx].historyIndex = 0;
  sensors[idx].alarmActive = false;

  int dIdx = findDiscovered(mac);
  if (dIdx != -1) {
    sensors[idx].temp = discovered[dIdx].temp;
    sensors[idx].hum = discovered[dIdx].hum;
    sensors[idx].lastUpdate = discovered[dIdx].lastSeen;
  }
  saveSensors();

  if (dIdx != -1) removeDiscoveredAt(dIdx);

  server.send(200, "text/plain", "OK");
}

void handleRemove() {
  if (!server.hasArg("mac")) {
    server.send(400, "text/plain", "Missing mac");
    return;
  }
  String mac = server.arg("mac");
  int idx = findSensor(mac);
  if (idx != -1) {
    removeSensorAt(idx);
    saveSensors();
  }
  int dIdx = findDiscovered(mac);
  if (dIdx != -1) removeDiscoveredAt(dIdx);
  server.send(200, "text/plain", "OK");
}

void handleSetStage() {
  if (!server.hasArg("stage")) {
    server.send(400, "text/plain", "Missing stage");
    return;
  }
  int stage = server.arg("stage").toInt();
  if (stage < 0 || stage >= STAGE_COUNT) {
    server.send(400, "text/plain", "Invalid stage");
    return;
  }
  activeStage = stage;
  saveStage();
  Serial.println("Stage changed");
  server.send(200, "text/plain", "OK");
}

void handleSensorCfg() {
  if (!server.hasArg("mac")) {
    server.send(400, "text/plain", "Missing mac");
    return;
  }
  String mac = server.arg("mac");
  bool any = false;

  if (server.hasArg("interval")) {
    uint32_t interval = (uint32_t) server.arg("interval").toInt();
    any |= sendSensorCmd(mac, CMD_SET_INTERVAL, interval);
  }
  if (server.hasArg("retry")) {
    uint32_t retry = (uint32_t) server.arg("retry").toInt();
    any |= sendSensorCmd(mac, CMD_SET_RETRY, retry);
  }
  if (server.hasArg("tx")) {
    int tx = server.arg("tx").toInt();
    if (tx < 8) tx = 8;
    if (tx > 78) tx = 78;
    any |= sendSensorCmd(mac, CMD_SET_TXPOWER, (uint32_t) tx);
  }
  if (server.hasArg("ota")) {
    uint32_t ota = (uint32_t) server.arg("ota").toInt();
    any |= sendSensorCmd(mac, CMD_SET_OTA, ota ? 1 : 0);
  }
  if (server.hasArg("ssid")) {
    any |= sendSensorCmdStr(mac, CMD_SET_WIFI_SSID, server.arg("ssid"));
  }
  if (server.hasArg("pass")) {
    any |= sendSensorCmdStr(mac, CMD_SET_WIFI_PASS, server.arg("pass"));
  }

  if (!any) {
    server.send(400, "text/plain", "No settings provided");
    return;
  }
  server.send(200, "text/plain", "OK");
}

void handleGetSettings() {
  JsonDocument doc;
  doc["offlineMs"] = offlineAfterMs;
  doc["pruneMs"] = discoveredStaleMs;
  doc["logMs"] = logIntervalMs;

  JsonArray sarr = doc["stages"].to<JsonArray>();
  for (int i = 0; i < STAGE_COUNT; i++) {
    JsonObject s = sarr.add<JsonObject>();
    s["name"] = stages[i].name;
    s["tMin"] = stages[i].tMin;
    s["tMax"] = stages[i].tMax;
    s["hMin"] = stages[i].hMin;
    s["hMax"] = stages[i].hMax;
  }

  JsonArray parr = doc["paired"].to<JsonArray>();
  for (int i = 0; i < sensorCount; i++) {
    JsonObject p = parr.add<JsonObject>();
    p["mac"] = sensors[i].mac;
    p["role"] = sensors[i].zone;
  }

  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void handleSetSettings() {
  if (!server.hasArg("plain")) {
    server.send(400, "text/plain", "Missing body");
    return;
  }
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, server.arg("plain"));
  if (err) {
    server.send(400, "text/plain", "Invalid JSON");
    return;
  }

  if (doc["offlineMs"].is<unsigned long>()) offlineAfterMs = doc["offlineMs"].as<unsigned long>();
  if (doc["pruneMs"].is<unsigned long>()) discoveredStaleMs = doc["pruneMs"].as<unsigned long>();
  if (doc["logMs"].is<unsigned long>()) logIntervalMs = doc["logMs"].as<unsigned long>();

  if (doc["stages"].is<JsonArray>()) {
    JsonArray sarr = doc["stages"].as<JsonArray>();
    for (int i = 0; i < STAGE_COUNT && i < (int)sarr.size(); i++) {
      JsonObject s = sarr[i];
      if (s["tMin"].is<float>()) stages[i].tMin = s["tMin"];
      if (s["tMax"].is<float>()) stages[i].tMax = s["tMax"];
      if (s["hMin"].is<float>()) stages[i].hMin = s["hMin"];
      if (s["hMax"].is<float>()) stages[i].hMax = s["hMax"];
    }
  }

  saveSettings();
  server.send(200, "text/plain", "OK");
}

void handleClearPairing() {
  sensorCount = 0;
  saveSensors();
  server.send(200, "text/plain", "OK");
}

void handleClearLogs() {
  File root = SPIFFS.open("/");
  if (!root) {
    server.send(500, "text/plain", "SPIFFS error");
    return;
  }
  File file = root.openNextFile();
  while (file) {
    String name = String(file.name());
    file.close();
    if (name.endsWith(".csv")) {
      SPIFFS.remove(name);
    }
    file = root.openNextFile();
  }
  root.close();
  server.send(200, "text/plain", "OK");
}

void handleResetWiFi() {
  WiFiManager wm;
  wm.resetSettings();
  server.send(200, "text/plain", "OK");
  delay(200);
  ESP.restart();
}

void handleDownloadLog() {
  String filename = todayLogFile();
  if (!SPIFFS.exists(filename)) {
    server.send(404, "text/plain", "Log not found");
    return;
  }
  File f = SPIFFS.open(filename, FILE_READ);
  server.streamFile(f, "text/csv");
  f.close();
}

// ---------------- WEB PAGE ----------------
void handleRoot() {

String page = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
@import url('https://fonts.googleapis.com/css2?family=Archivo:wght@400;600;700&display=swap');
:root{
  --bg:#0b0d11;
  --panel:#151b28;
  --panel-2:#101521;
  --accent:#3da5ff;
  --accent-2:#79f2c0;
  --warn:#ff5252;
  --text:#e8ecf3;
  --muted:#9aa6bd;
}
*{box-sizing:border-box}
body{background:radial-gradient(1200px 800px at 10% -10%, #182039 0%, #0b0d11 40%, #080a0f 100%);color:var(--text);font-family:'Archivo',sans-serif;margin:0;padding:16px}
header{display:grid;grid-template-columns:1fr auto;gap:6px 12px;margin-bottom:14px}
.header-left{display:flex;flex-direction:column}
.header-right{display:flex;flex-direction:column;align-items:flex-end;gap:4px}
.stage-row{display:flex;align-items:center;gap:8px}
h2{font-weight:700;letter-spacing:.3px}
h3{margin:0 0 10px 0;font-size:16px;color:var(--muted);text-transform:uppercase;letter-spacing:.12em}
.badge{padding:6px 10px;border-radius:999px;background:var(--panel-2);color:var(--muted);font-size:12px;display:inline-block;margin-top:6px}
.grid{display:grid;grid-template-columns:1fr;gap:14px}
.panel{background:linear-gradient(180deg, rgba(255,255,255,0.04), rgba(255,255,255,0.01));border:1px solid #1c2335;border-radius:14px;padding:14px}
.card{background:var(--panel);margin:10px 0;padding:14px;border-radius:12px;border:1px solid #20283a;box-shadow:0 12px 30px #0007}
.row{display:flex;justify-content:space-between;align-items:center;gap:10px;flex-wrap:wrap}
.alarm{color:var(--warn);font-weight:700}
.gauge{height:10px;background:#0d111b;border-radius:8px;overflow:hidden;margin:6px 0}
.gauge span{display:block;height:100%;background:linear-gradient(90deg,var(--accent),var(--accent-2))}
.pill{padding:6px 10px;border-radius:999px;background:#0d1220;border:1px solid #1f2638;color:var(--muted);font-size:12px}
.alert{background:#1a1418;border:1px solid #5a1b1b;color:#ffb3b3;border-radius:10px;padding:8px 10px;margin-top:8px}
.alert .title{font-weight:700;margin-bottom:4px}
.alert .body{color:var(--muted)}
.btn{background:var(--panel-2);color:var(--text);border:1px solid #263049;border-radius:8px;padding:6px 10px;cursor:pointer}
.btn:hover{border-color:#3b4a6b}
.btn.warn{border-color:#5a1b1b;color:#ffb3b3}
.role-btns{display:flex;gap:6px;flex-wrap:wrap}
.muted{color:var(--muted)}
select{background:#0f1420;color:var(--text);border:1px solid #333;border-radius:8px;padding:6px 8px}
@media (min-width:900px){
  .grid{grid-template-columns:2fr 1fr}
}
</style>
</head>
<body>
<header>
  <div class="header-left">
    <h2 style="margin:0">Barn Monitor</h2>
    <div id="stageName" class="badge"></div>
  </div>
  <div class="header-right">
    <button class="btn" onclick="toggleSettings()">Settings</button>
    <div class="stage-row">
      <div class="muted">Stage</div>
      <select id="stageSel">
        <option value="0">Yellowing</option>
        <option value="1">Leaf Drying</option>
        <option value="2">Stem Drying</option>
      </select>
    </div>
  </div>
</header>

<div id="settings" class="panel" style="display:none;margin-bottom:14px;">
  <h3>Settings</h3>
  <div id="settingsBody"></div>
  <div class="row" style="margin-top:8px;">
    <button class="btn" onclick="saveSettings()">Save</button>
    <button class="btn" onclick="toggleSettings()">Close</button>
  </div>
</div>

<div class="grid">
  <section class="panel">
    <h3>Paired Sensors</h3>
    <div id="content"></div>
  </section>
  <section class="panel">
    <div class="row" style="margin-bottom:8px;">
      <h3>Pair Sensors</h3>
      <button class="btn" onclick="loadData()">Refresh</button>
    </div>
    <div id="available"></div>
  </section>
</div>

<script>
async function setStage(){
  const v = document.getElementById('stageSel').value;
  await fetch(`/setstage?stage=${v}`);
}

document.getElementById('stageSel').addEventListener('change', setStage);

async function pairSensor(mac, role){
  await fetch(`/pair?mac=${encodeURIComponent(mac)}&role=${encodeURIComponent(role)}`);
  loadData();
}

async function removeSensor(mac){
  await fetch(`/remove?mac=${encodeURIComponent(mac)}`);
  loadData();
}

async function resetWifi(){
  await fetch('/resetwifi');
}

async function clearPairing(){
  await fetch('/clearpairing');
  loadData();
}

async function clearLogs(){
  await fetch('/clearlogs');
}

async function sendSensorCfg(mac, idx){
  const interval = document.getElementById(`cfgInterval${idx}`).value;
  const retry = document.getElementById(`cfgRetry${idx}`).value;
  const tx = document.getElementById(`cfgTx${idx}`).value;
  const ota = document.getElementById(`cfgOta${idx}`).checked ? 1 : 0;
  const ssid = document.getElementById(`cfgSsid${idx}`).value;
  const pass = document.getElementById(`cfgPass${idx}`).value;
  const qs = `mac=${encodeURIComponent(mac)}&interval=${encodeURIComponent(interval)}&retry=${encodeURIComponent(retry)}&tx=${encodeURIComponent(tx)}&ota=${ota}&ssid=${encodeURIComponent(ssid)}&pass=${encodeURIComponent(pass)}`;
  await fetch(`/sensorcfg?${qs}`);
}

function dewPointC(t, h){
  if(!isFinite(t) || !isFinite(h) || h <= 0) return NaN;
  const a = 17.62;
  const b = 243.12;
  const gamma = Math.log(h/100) + (a * t) / (b + t);
  return (b * gamma) / (a - gamma);
}

function fmtRssi(rssi){
  return (rssi === null || rssi === undefined || rssi <= -120) ? "--" : `${rssi} dBm`;
}

function fmtIp(ip){
  if(!ip) return "--";
  return [ip & 255, (ip>>8)&255, (ip>>16)&255, (ip>>24)&255].join(".");
}

function toggleSettings(){
  const el = document.getElementById('settings');
  const show = el.style.display === 'none';
  el.style.display = show ? 'block' : 'none';
  if(show) loadSettings();
}

async function loadSettings(){
  const r = await fetch('/settings');
  const d = await r.json();
  const r2 = await fetch('/data');
  const d2 = await r2.json();
  const statusByMac = {};
  if (d2 && d2.sensors) {
    d2.sensors.forEach(s => { statusByMac[s.mac] = s; });
  }
  let html = '';
  html += `<div class="row"><div class="pill">Offline timeout (min)</div><input id="offlineMin" type="number" min="1" value="${Math.round(d.offlineMs/60000)}"></div>`;
  html += `<div class="row"><div class="pill">Prune discovered (min)</div><input id="pruneMin" type="number" min="1" value="${Math.round(d.pruneMs/60000)}"></div>`;
  html += `<div class="row"><div class="pill">Log interval (sec)</div><input id="logSec" type="number" min="10" value="${Math.round(d.logMs/1000)}"></div>`;

  html += `<div class="card"><div class="row"><div style="font-weight:600;">Stages</div></div>`;
  d.stages.forEach((s, i)=> {
    html += `<div class="row" style="margin-top:6px;"><div class="pill">${s.name}</div></div>`;
    html += `<div class="row"><span class="muted">Temp min/max</span><input id="tmin${i}" type="number" step="0.1" value="${s.tMin}"><input id="tmax${i}" type="number" step="0.1" value="${s.tMax}"></div>`;
    html += `<div class="row"><span class="muted">Hum min/max</span><input id="hmin${i}" type="number" step="0.1" value="${s.hMin}"><input id="hmax${i}" type="number" step="0.1" value="${s.hMax}"></div>`;
  });
  html += `</div>`;

  html += `<div class="card"><div style="font-weight:600;margin-bottom:6px;">Sensor Roles</div>`;
  d.paired.forEach((p,i)=>{
    html += `<div class="row"><span class="muted">${p.mac}</span>
      <select id="role${i}">
        <option value="Top sensor"${p.role==='Top sensor'?' selected':''}>Top sensor</option>
        <option value="Middle sensor"${p.role==='Middle sensor'?' selected':''}>Middle sensor</option>
        <option value="Bottom sensor"${p.role==='Bottom sensor'?' selected':''}>Bottom sensor</option>
      </select></div>`;
  });
  html += `<div class="card"><div style="font-weight:600;margin-bottom:6px;">Sensor Config</div>`;
  d.paired.forEach((p,i)=>{
    html += `<div class="row"><span class="muted">${p.mac}</span></div>`;
    html += `<div class="row"><span class="muted">Interval (ms)</span><input id="cfgInterval${i}" type="number" min="1000" value="5000"></div>`;
    html += `<div class="row"><span class="muted">DHT retries</span><input id="cfgRetry${i}" type="number" min="0" max="5" value="2"></div>`;
    html += `<div class="row"><span class="muted">TX power (8-78)</span><input id="cfgTx${i}" type="number" min="8" max="78" value="78"></div>`;
    html += `<div class="row"><span class="muted">OTA mode</span><input id="cfgOta${i}" type="checkbox"></div>`;
    html += `<div class="row"><span class="muted">OTA SSID</span><input id="cfgSsid${i}" type="text" value=""></div>`;
    const st = statusByMac[p.mac] || {};
    const apiLine = `OTA ${st.cfgOta ? "ON" : "OFF"}, IP ${fmtIp(st.cfgIp)}, Int ${st.cfgInterval || 0}ms, Retry ${st.cfgRetry || 0}, TX ${st.cfgTx || 0}`;
    html += `<div class="muted" style="margin-bottom:6px;">Sensor API: ${apiLine}</div>`;
    html += `<div class="row"><span class="muted">OTA Pass</span><input id="cfgPass${i}" type="password" value=""></div>`;
    html += `<div class="row"><button class="btn" onclick="sendSensorCfg('${p.mac}', ${i})">Apply To Sensor</button></div>`;
  });
  html += `</div>`;

  html += `</div>`;

  html += `<div class="row" style="margin-top:6px;">
    <button class="btn warn" onclick="clearPairing()">Clear Pairing</button>
    <button class="btn warn" onclick="clearLogs()">Clear Logs</button>
    <button class="btn warn" onclick="resetWifi()">Reset WiFi</button>
  </div>`;

  document.getElementById('settingsBody').innerHTML = html;
}

async function saveSettings(){
  const r = await fetch('/settings');
  const d = await r.json();
  const offlineMs = Math.max(1, parseInt(document.getElementById('offlineMin').value || '5',10)) * 60000;
  const pruneMs = Math.max(1, parseInt(document.getElementById('pruneMin').value || '5',10)) * 60000;
  const logMs = Math.max(10, parseInt(document.getElementById('logSec').value || '60',10)) * 1000;

  const stages = d.stages.map((s,i)=>({
    tMin: parseFloat(document.getElementById(`tmin${i}`).value),
    tMax: parseFloat(document.getElementById(`tmax${i}`).value),
    hMin: parseFloat(document.getElementById(`hmin${i}`).value),
    hMax: parseFloat(document.getElementById(`hmax${i}`).value)
  }));

  await fetch('/settings', {
    method:'POST',
    headers:{'Content-Type':'application/json'},
    body: JSON.stringify({offlineMs, pruneMs, logMs, stages})
  });

  d.paired.forEach((p,i)=>{
    const role = document.getElementById(`role${i}`).value;
    fetch(`/setzone?mac=${encodeURIComponent(p.mac)}&name=${encodeURIComponent(role)}`);
  });

  loadData();
}

function clampPct(val, min, max){
  if(!isFinite(val) || !isFinite(min) || !isFinite(max) || max === min) return 0;
  const pct = (val - min) / (max - min) * 100;
  if(!isFinite(pct)) return 0;
  return Math.max(0, Math.min(100, pct));
}

async function loadData(){
  let r = await fetch('/data');
  let d = await r.json();

  document.getElementById('stageName').innerHTML = `Active: ${d.stage} (${d.tMin}-${d.tMax} &deg;C, ${d.hMin}-${d.hMax}%)`;
  document.getElementById('stageSel').value = d.stageIndex;

  let html="";
  if(!d.sensors || d.sensors.length===0){
    html = `
    <div class="card">
      <div style="font-size:16px;margin-bottom:6px;">No paired sensors yet</div>
      <div class="muted">Use the pairing panel to assign Top, Middle, or Bottom.</div>
    </div>`;
  }

  const targetT = (d.tMin + d.tMax) / 2;
  const targetH = (d.hMin + d.hMax) / 2;
  const targetDew = dewPointC(targetT, targetH);

  const order = {'Top sensor':0,'Middle sensor':1,'Bottom sensor':2};
  d.sensors.sort((a,b)=> (order[a.zone]??99) - (order[b.zone]??99));

  d.sensors.forEach((s,i)=>{
    const currentDew = dewPointC(s.temp, s.hum);

    html+=`
    <div class="card">
      <div class="row">
        <div style="font-size:18px;font-weight:700;">${s.zone || `Sensor ${i+1}`}</div>
        <div class="row">
          <div class="pill">${s.offline ? 'OFFLINE' : 'ONLINE'}</div>
          <div class="${s.alarm?'alarm':'pill'}">${s.alarm ? 'ALARM' : 'OK'}</div>
        </div>
      </div>
      <div class="row" style="margin:6px 0;">
        <div class="pill">Target T: ${targetT.toFixed(1)} &deg;C</div>
        <div class="pill">Target H: ${targetH.toFixed(1)} %</div>
        <div class="pill">Target DP: ${targetDew.toFixed(1)} &deg;C</div>
      </div>
      <div class="row" style="margin-bottom:6px;">
        <div class="muted">Packets: ${s.packets}</div>
        <div class="muted">RSSI: ${fmtRssi(s.rssi)}</div>
      </div>
      ${s.alarm ? `<div class="alert">
        <div class="title">${s.alarmReason || 'Out of range'}</div>
        <div class="body">${s.alarmSolution || ''}</div>
      </div>` : ``}
      <div>Temp: ${s.temp.toFixed(1)} &deg;C</div>
      <div class="gauge"><span style="width:${clampPct(s.temp,d.tMin,d.tMax)}%"></span></div>
      <div>Hum: ${s.hum.toFixed(1)} %</div>
      <div class="gauge"><span style="width:${clampPct(s.hum,d.hMin,d.hMax)}%"></span></div>
      <div>Dew point: ${currentDew.toFixed(1)} &deg;C</div>
      <div class="row" style="margin-top:8px;">
        <div class="muted">Last update: ${s.lastUpdate}s ago</div>
        <button class="btn warn" onclick="removeSensor('${s.mac}')">Remove</button>
      </div>
    </div>`;
  });

  document.getElementById("content").innerHTML = html;

  let availHtml = "";
  if(!d.available || d.available.length===0){
    availHtml = `
    <div class="card">
      <div class="muted">No sensors detected yet. Power on a sensor node to see it here.</div>
    </div>`;
  } else {
    d.available.forEach((a,i)=>{
      availHtml += `
      <div class="card">
        <div class="row">
          <div style="font-weight:600;">Sensor ${i+1}</div>
          <div class="muted">Seen ${a.lastSeen}s ago</div>
        </div>
        <div class="muted">Temp ${a.temp.toFixed(1)} &deg;C, Hum ${a.hum.toFixed(1)} %, RSSI ${fmtRssi(a.rssi)}</div>
        <div class="role-btns" style="margin-top:8px;">
          <button class="btn" onclick="pairSensor('${a.mac}','top')">Top</button>
          <button class="btn" onclick="pairSensor('${a.mac}','middle')">Middle</button>
          <button class="btn" onclick="pairSensor('${a.mac}','bottom')">Bottom</button>
        </div>
      </div>`;
    });
  }
  document.getElementById("available").innerHTML = availHtml;
}

setInterval(loadData,3000);
loadData();
</script>
</body>
</html>
)rawliteral";

  server.send(200, "text/html", page);
}

// ---------------- SETUP ----------------
void setup() {

  Serial.begin(115200);

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS mount failed");
  }

  loadSensors();
  loadStage();
  loadSettings();

  WiFiManager wm;
  wm.autoConnect("BarnMonitor-Setup");

  configTime(0, 0, "pool.ntp.org", "time.nist.gov");

  Serial.print("Connected IP: ");
  Serial.println(WiFi.localIP());

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP NOW FAIL");
    return;
  }

  esp_now_register_recv_cb(onReceive);

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/setzone", handleSetZone);
  server.on("/pair", handlePair);
  server.on("/remove", handleRemove);
  server.on("/setstage", handleSetStage);
  server.on("/sensorcfg", handleSensorCfg);
  server.on("/settings", HTTP_GET, handleGetSettings);
  server.on("/settings", HTTP_POST, handleSetSettings);
  server.on("/clearpairing", handleClearPairing);
  server.on("/clearlogs", handleClearLogs);
  server.on("/resetwifi", handleResetWiFi);
  server.on("/downloadlog", handleDownloadLog);
  server.onNotFound([](){ server.send(404, "text/plain", "Not found"); });

  server.begin();

  Serial.println("Master ready");
}

// ---------------- LOOP ----------------
void loop() {

  server.handleClient();

  if (millis() - lastLog > logIntervalMs) {
    lastLog = millis();
    logData();
  }
}
