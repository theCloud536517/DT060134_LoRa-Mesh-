#include <Wire.h>
#include <TinyGPSPlus.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_INA219.h>
#include <WiFi.h>
// ================= QUY HOẠCH CHÂN KHỚP 100% VỚI PCB =================
// 1. Giao tiếp UART1 với Module LoRa DX-LR32
#define LORA_RX_PIN   18  // Nối với chân TXD (Chân 4) của DX-LR32
#define LORA_TX_PIN   19  // Nối với chân RXD (Chân 3) của DX-LR32

// 2. Giao tiếp UART2 với Module GPS ATGM336H
#define GPS_RX_PIN    16  // RX2 nối với chân TX của GPS
#define GPS_TX_PIN    17  // TX2 nối với chân RX của GPS

// 3. Giao tiếp Bus I2C (Dùng chung cho OLED 1.3" và INA219)
#define I2C_SDA_PIN   21
#define I2C_SCL_PIN   22

// 4. Cụm Nút nhấn SOS, Còi Buzzer và Đèn LED trạng thái
#define SOS_BTN_PIN   4   // Nút nhấn J2 (Active-LOW, có trở kéo lên R2 10k & tụ C5)
#define BUZZER_PIN    26  // Còi chíp chủ động J1
#define LED_ST_PIN    14  // Đèn LED 5mm (L1 qua trở R1 330R)

// Định danh Node ("N1", "N2", "N3", "N4")
String NODE_ID = "N3";

HardwareSerial LoRaSerial(1); // UART1 cho LoRa DX-LR32
TinyGPSPlus gps;
Adafruit_SH1106G display = Adafruit_SH1106G(128, 64, &Wire, -1);
Adafruit_INA219 ina219;

bool oledReady = false;
bool inaReady = false;
bool isSOS = false;
int lastBtnState = HIGH;
unsigned long lastDebounceTime = 0;

// Bộ nhớ đệm chống lặp gói (Broadcast Storm Prevention)
String cache[15];
int cache_idx = 0;
int msg_counter = 0;
String lastRelayInfo = "San sang";

// Biến lưu thông số năng lượng từ INA219
float batVoltage = 0.0;
float current_mA = 0.0;
int batPercent = 0;

// Hàm tách trường dữ liệu theo dấu '|'
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
  for (int i = 0; i < 15; i++) {
    if (cache[i] == msgID) return true;
  }
  return false;
}

void addToCache(String msgID) {
  cache[cache_idx] = msgID;
  cache_idx = (cache_idx + 1) % 15;
}

// Đọc cảm biến dòng/áp INA219
void readPowerINA219() {
  if (!inaReady) return;
  float shuntvoltage = ina219.getShuntVoltage_mV();
  float busvoltage = ina219.getBusVoltage_V();
  batVoltage = busvoltage + (shuntvoltage / 1000.0);
  current_mA = ina219.getCurrent_mA();

  // Quy đổi phần trăm pin Li-ion 18650 (3.2V = 0%, 4.2V = 100%)
  batPercent = (int)((batVoltage - 3.2) * 100.0 / (4.2 - 3.2));
  if (batPercent > 100) batPercent = 100;
  if (batPercent < 0) batPercent = 0;
}

// Xử lý nút nhấn SOS, Còi Buzzer và Đèn LED đa chế độ
void handleSOSAndIndicators() {
  unsigned long now = millis();
  int reading = digitalRead(SOS_BTN_PIN);

  if (reading == LOW && lastBtnState == HIGH && (now - lastDebounceTime > 250)) {
    isSOS = !isSOS; // Đảo trạng thái SOS khi nhấn nút
    lastDebounceTime = now;
    if (!isSOS) {
      digitalWrite(BUZZER_PIN, LOW);
      digitalWrite(LED_ST_PIN, LOW);
    }
  }
  lastBtnState = reading;

  if (isSOS) {
    // Chế độ SOS: LED chớp nhanh 5Hz, Còi kêu nhịp 3 tiếng bíp cứu hộ
    digitalWrite(LED_ST_PIN, (now % 200) < 100 ? HIGH : LOW);
    unsigned long cycle = now % 1500;
    bool beep = (cycle < 150) || (cycle >= 300 && cycle < 450) || (cycle >= 600 && cycle < 750);
    digitalWrite(BUZZER_PIN, beep ? HIGH : LOW);
  } else {
    digitalWrite(BUZZER_PIN, LOW);
    if (gps.location.isValid()) {
      // Đã khóa GPS: Nháy kép (Double-blink) mỗi 2 giây
      unsigned long cycle = now % 2000;
      digitalWrite(LED_ST_PIN, ((cycle < 80) || (cycle >= 200 && cycle < 280)) ? HIGH : LOW);
    } else {
      // Đang chờ GPS: Nháy đơn mỗi 2 giây
      digitalWrite(LED_ST_PIN, (now % 2000) < 100 ? HIGH : LOW);
    }
  }
}

// Cập nhật màn hình OLED 1.3 inch
void updateOLED() {
  if (!oledReady) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);

  // Dòng 1: ID Node, Trạng thái SOS và Phần trăm Pin
  display.setCursor(0, 0);
  display.print(NODE_ID + (isSOS ? " [SOS!]" : " [OK]"));
  display.setCursor(72, 0);
  display.print(String(batVoltage, 1) + "V " + String(batPercent) + "%");
  display.drawLine(0, 10, 127, 10, SH110X_WHITE);

  // Dòng 2 & 3: Tọa độ GPS và Dòng tiêu thụ (mA)
  if (gps.location.isValid()) {
    display.setCursor(0, 14);
    display.print("LA:" + String(gps.location.lat(), 5));
    display.setCursor(0, 25);
    display.print("LO:" + String(gps.location.lng(), 5));
  } else {
    display.setCursor(0, 18);
    display.print("GPS: Wait_GPS...");
  }
  display.setCursor(0, 36);
  display.print("I:" + String((int)current_mA) + "mA | TX:" + String(msg_counter));

  // Dòng 4: Luồng chuyển tiếp Mesh (Relay)
  display.drawLine(0, 48, 127, 48, SH110X_WHITE);
  display.setCursor(0, 52);
  display.print("R: " + lastRelayInfo);
  display.display();
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_OFF);
  btStop();
  setCpuFrequencyMhz(80);
  // Khởi tạo UART2 cho GPS và UART1 cho LoRa DX-LR32
  Serial2.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  LoRaSerial.begin(9600, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);

  pinMode(SOS_BTN_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_ST_PIN, OUTPUT);

  // Khởi tạo Bus I2C trên chân 21 (SDA), 22 (SCL) với chống treo 50ms
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setTimeOut(50);

  if (display.begin(0x3C, true)) {
    oledReady = true;
    display.clearDisplay();
    display.setTextColor(SH110X_WHITE);
    display.setCursor(10, 25);
    display.println("Khoi dong " + NODE_ID + "...");
    display.display();
  }

  if (ina219.begin()) {
    inaReady = true;
  }

  readPowerINA219();
  updateOLED();
}

void loop() {
  // 1. Đọc liên tục luồng dữ liệu NMEA từ GPS qua UART2
  while (Serial2.available() > 0) {
    gps.encode(Serial2.read());
  }

  // 2. Quét nút nhấn khẩn cấp SOS & điều khiển Còi/LED
  handleSOSAndIndicators();

  // 3. CHU TRÌNH PHÁT GÓI TIN ĐỊNH KỲ (5s + Random Jitter)
  static unsigned long lastTx = 0;
  static int randomInterval = 5000;

  if (millis() - lastTx > randomInterval) {
    readPowerINA219();
    msg_counter++;
    String msgID = NODE_ID + "_" + String(msg_counter);
    addToCache(msgID);

    String gpsData = "Wait_GPS";
    if (gps.location.isValid()) {
      double spd = gps.speed.isValid() ? gps.speed.kmph() : 0.0;
      if (spd < 0.5) spd = 0.0;
      gpsData = "LAT:" + String(gps.location.lat(), 6) +
                ",LON:" + String(gps.location.lng(), 6) +
                ",SPD:" + String(spd, 1);
    }

    // Ghép thêm thông số năng lượng INA219 và cờ cảnh báo SOS
    String powerAndSos = ",BAT:" + String(batVoltage, 2) +
                         ",CUR:" + String((int)current_mA) +
                         ",SOS:" + String(isSOS ? 1 : 0);

    String payload = msgID + "|" + NODE_ID + "|" + NODE_ID + "|" + NODE_ID + "|0|" + gpsData + powerAndSos;

    LoRaSerial.println(payload); // Phát qua UART1 tới DX-LR32
    Serial.println("-> [TX] " + payload);
    updateOLED();

    lastTx = millis();
    randomInterval = 5000 + random(0, 2000);
  }

  // 4. CHU TRÌNH LẮNG NGHE & CHUYỂN TIẾP ĐA CHẶNG (MESH RELAY)
  if (LoRaSerial.available() > 0) {
    String rxData = LoRaSerial.readStringUntil('\n');
    rxData.trim();

    String msgID   = getValue(rxData, '|', 0);
    String source  = getValue(rxData, '|', 1);
    String sender  = getValue(rxData, '|', 2);
    String path    = getValue(rxData, '|', 3);
    String hopsStr = getValue(rxData, '|', 4);
    String data    = getValue(rxData, '|', 5);

    if (data == "") return;
    int hops = hopsStr.toInt();

    // Kiểm tra chống lặp gói (Broadcast Storm) và giới hạn bước nhảy
    if (inCache(msgID) || source == NODE_ID || hops >= 5) {
      return; // Hủy gói tin trùng lặp
    }

    addToCache(msgID);
    String newPath = path + "->" + NODE_ID;
    String newPayload = msgID + "|" + source + "|" + NODE_ID + "|" + newPath + "|" + String(hops + 1) + "|" + data;

    lastRelayInfo = newPath;
    delay(random(50, 200)); // Trễ ngẫu nhiên chống xung đột kênh truyền (CSMA-like)
    LoRaSerial.println(newPayload);

    Serial.println("=> [RELAY] " + newPayload);
    updateOLED();
  }
}