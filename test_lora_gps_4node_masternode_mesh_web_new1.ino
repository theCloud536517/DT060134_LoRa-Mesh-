#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

// Định nghĩa chân UART1 nằm cùng dãy hàng chân với chân VIN của ESP32-WROOM-32
#define LORA_RX_PIN 18 // Nối với chân TXD (Chân 4) của module DX-LR32
#define LORA_TX_PIN 19 // Nối với chân RXD (Chân 3) của module DX-LR32

const char* AP_SSID = "LoRa_Mesh_Gateway";
const char* AP_PASS = "12345678";

// Đặt = true để nhận gói tin trực tiếp từ Node 2 khi test khoảng cách 1-1.
// Khi lắp đủ các trạm Relay và muốn ép N2 đi qua trung gian thì đổi thành false.
bool TEST_DIRECT_N2 = true;

WebServer server(80);
HardwareSerial LoRaSerial(1); // Sử dụng bộ UART1 phần cứng

String cache[30];
int cache_idx = 0;

struct NodeInfo {
  String id;
  String path;
  int hops;
  double lat;
  double lon;
  double spd;
  int bat;
  bool valid;
  int rssi;
  unsigned long lastSeen;
  bool active;
};

NodeInfo nodes[4] = {
  {"N1", "", 0, 0, 0, 0, 100, false, 0, 0, false},
  {"N2", "", 0, 0, 0, 0, 100, false, 0, 0, false},
  {"N3", "", 0, 0, 0, 0, 100, false, 0, 0, false},
  {"N4", "", 0, 0, 0, 0, 100, false, 0, 0, false}
};

#define MAX_TRAIL_POINTS 200
struct TrailPoint {
  double lat;
  double lon;
};
TrailPoint n2Trail[MAX_TRAIL_POINTS];
int trailCount = 0;

void recordTrailPoint(double lat, double lon) {
  if (lat == 0 || lon == 0) return;
  if (trailCount > 0) {
    double dLat = fabs(lat - n2Trail[trailCount - 1].lat);
    double dLon = fabs(lon - n2Trail[trailCount - 1].lon);
    if (dLat < 0.000022 && dLon < 0.000022) return;
  }
  if (trailCount < MAX_TRAIL_POINTS) {
    n2Trail[trailCount].lat = lat;
    n2Trail[trailCount].lon = lon;
    trailCount++;
  } else {
    for (int i = 1; i < MAX_TRAIL_POINTS; i++) {
      n2Trail[i - 1] = n2Trail[i];
    }
    n2Trail[MAX_TRAIL_POINTS - 1].lat = lat;
    n2Trail[MAX_TRAIL_POINTS - 1].lon = lon;
  }
}

String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;
  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

bool inCache(String msgID) {
  for (int i = 0; i < 30; i++) {
    if (cache[i] == msgID) return true;
  }
  return false;
}

// Giải mã tọa độ, tốc độ và dung lượng pin (% BAT) từ chuỗi payload
void parseGPS(String data, double &lat, double &lon, double &spd, int &bat, bool &valid) {
  int batIdx = data.indexOf(",BAT:");
  if (batIdx != -1) {
    bat = data.substring(batIdx + 5).toInt();
  }

  if (data.startsWith("LAT:")) {
    int lonIdx = data.indexOf(",LON:");
    int spdIdx = data.indexOf(",SPD:");
    if (lonIdx != -1) {
      lat = data.substring(4, lonIdx).toDouble();
      if (spdIdx != -1) {
        lon = data.substring(lonIdx + 5, spdIdx).toDouble();
        if (batIdx != -1 && batIdx > spdIdx) {
          spd = data.substring(spdIdx + 5, batIdx).toDouble();
        } else {
          spd = data.substring(spdIdx + 5).toDouble();
        }
      } else {
        lon = data.substring(lonIdx + 5).toDouble();
        spd = 0.0;
      }
      valid = true;
      return;
    }
  }
  valid = false;
  spd = 0.0;
}

void handleGetData() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  String json = "{\"nodes\":[";
  bool first = true;
  for (int i = 0; i < 4; i++) {
    if (nodes[i].active) {
      if (!first) json += ",";
      first = false;
      unsigned long age = (millis() - nodes[i].lastSeen) / 1000;
      json += "{";
      json += "\"id\":\"" + nodes[i].id + "\",";
      json += "\"path\":\"" + nodes[i].path + "\",";
      json += "\"hops\":" + String(nodes[i].hops) + ",";
      json += "\"lat\":" + String(nodes[i].lat, 6) + ",";
      json += "\"lon\":" + String(nodes[i].lon, 6) + ",";
      json += "\"spd\":" + String(nodes[i].spd, 1) + ",";
      json += "\"bat\":" + String(nodes[i].bat) + ",";
      json += "\"valid\":" + String(nodes[i].valid ? "true" : "false") + ",";
      json += "\"rssi\":" + String(nodes[i].rssi) + ",";
      json += "\"age\":" + String(age);
      json += "}";
    }
  }
  json += "],\"trail\":[";
  for (int i = 0; i < trailCount; i++) {
    if (i > 0) json += ",";
    json += "[" + String(n2Trail[i].lon, 6) + "," + String(n2Trail[i].lat, 6) + "]";
  }
  json += "]}";
  server.send(200, "application/json", json);
}

void handleClearTrail() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  trailCount = 0;
  server.send(200, "text/plain", "OK");
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("WiFi AP da phat: ");
  Serial.println(AP_SSID);
  Serial.print("Dia chi IP Gateway: ");
  Serial.println(WiFi.softAPIP());

  server.on("/data", HTTP_GET, handleGetData);
  server.on("/clear_trail", HTTP_GET, handleClearTrail);
  server.begin();

  // Khởi tạo UART1 trên chân GPIO 13 (RX1) và GPIO 14 (TX1)
  LoRaSerial.begin(9600, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);

  Serial.println("===================================================");
  Serial.println(" MASTER GATEWAY ESP32 (PIN D13/D14 + VIN) READY!");
  Serial.println("===================================================");
}

void loop() {
  server.handleClient();

  if (LoRaSerial.available() > 0) {
    String rxData = LoRaSerial.readStringUntil('\n');
    rxData.trim();

    if (rxData.length() < 5) return;

    String msgID  = getValue(rxData, '|', 0);
    String source = getValue(rxData, '|', 1);
    String sender = getValue(rxData, '|', 2);
    String path   = getValue(rxData, '|', 3);
    String hops   = getValue(rxData, '|', 4);
    String data   = getValue(rxData, '|', 5);

    if (data == "") return;

    if (!TEST_DIRECT_N2 && sender == "N2") return;

    if (inCache(msgID)) return;
    cache[cache_idx] = msgID;
    cache_idx = (cache_idx + 1) % 30;

    int hopsCount = hops.toInt();
    int estRssi = -75 - (hopsCount * 8);

    for (int i = 0; i < 4; i++) {
      if (nodes[i].id == source) {
        nodes[i].path = path;
        nodes[i].hops = hopsCount;
        nodes[i].rssi = estRssi;
        nodes[i].lastSeen = millis();
        nodes[i].active = true;
        parseGPS(data, nodes[i].lat, nodes[i].lon, nodes[i].spd, nodes[i].bat, nodes[i].valid);

        if (source == "N2" && nodes[i].valid) {
          recordTrailPoint(nodes[i].lat, nodes[i].lon);
        }
        break;
      }
    }

    String fullPath = path + " -> Gateway";
    Serial.print("[DATA UART] " + fullPath);
    Serial.print(" | Hops: " + hops);
    Serial.println(" | Data: " + data);
  }
}