#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include <math.h>

/*
  ESP32: GPS + SIM + MAX30100 + SHT30 + SENSOR WATER + ThingSpeak qua 4G (không dùng WiFi)
  + Điều khiển 2 relay (GPIO5, GPIO18)
  - Khi cảm biến nước vượt ngưỡng -> Bật relay (cả hai)
  - Có moving-average, hysteresis, và thời gian tối thiểu giữa hai lần chuyển relay để tránh chattering
  - Relay module: bạn chọn COM-HIGH => active HIGH (OUTPUT = HIGH -> relay ON)
*/

// ================== PIN ==================
#define SENSOR_WATER 33
#define BUTTON_ON    25   // BẬT SMS
#define BUTTON_OFF   26   // TẮT SMS
const int WATER_THRESHOLD = 1500;

// ===== Relay =====
#define RELAY_PUMP_PIN  5    // relay 1: bơm/van
#define RELAY_ALARM_PIN 18   // relay 2: còi/đèn
// COM-HIGH -> relay active HIGH (OUTPUT HIGH = ON)
const bool RELAY_ACTIVE_LOW = false;

// Hysteresis để tránh chattering
const int WATER_HYSTERESIS = 50; // ngưỡng thấp hơn = WATER_THRESHOLD - WATER_HYSTERESIS

// Lọc tín hiệu: moving average
const int WATER_SAMPLES = 8; // số mẫu trung bình (tăng để mịn hơn)
int waterBuf[WATER_SAMPLES];
int waterIdx = 0;
bool waterBufFilled = false;

// Thời gian tối thiểu giữa hai lần chuyển trạng thái relay (ms)
const unsigned long RELAY_MIN_TOGGLE_MS = 3000;

// Trạng thái gửi SMS
bool smsEnabled = false;

// Trạng thái relay hiện tại (giữ trạng thái để chỉ viết khi thay đổi)
bool relayPumpState  = false; // false = OFF, true = ON
bool relayAlarmState = false;

// Chống dội nút
bool lastBtnOn  = HIGH;
bool lastBtnOff = HIGH;
unsigned long lastDebOn  = 0;
unsigned long lastDebOff = 0;
const unsigned long debounceDelay = 40;

// ================== MAX30100 + SHT30 ==================
#define MAX30100_ADDR 0x57
#define SHT30_ADDR    0x44

// MAX30100 registers
#define MAX30100_FIFO_WR_PTR 0x02
#define MAX30100_OVF_COUNTER 0x03
#define MAX30100_FIFO_RD_PTR 0x04
#define MAX30100_FIFO_DATA   0x05
#define MAX30100_MODE_CONFIG 0x06
#define MAX30100_SPO2_CONFIG 0x07
#define MAX30100_LED_CONFIG  0x09

void i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val){
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t i2cRead8(uint8_t addr, uint8_t reg){
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)1);
  if (Wire.available()) return Wire.read();
  return 0;
}

void i2cReadBytes(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len){
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, len);
  uint8_t i=0;
  while (Wire.available() && i < len) buf[i++] = Wire.read();
}

bool initMAX30100(){
  i2cWrite8(MAX30100_ADDR, MAX30100_FIFO_WR_PTR, 0x00);
  i2cWrite8(MAX30100_ADDR, MAX30100_OVF_COUNTER, 0x00);
  i2cWrite8(MAX30100_ADDR, MAX30100_FIFO_RD_PTR, 0x00);
  i2cWrite8(MAX30100_ADDR, MAX30100_MODE_CONFIG, 0x03); // HR only
  i2cWrite8(MAX30100_ADDR, MAX30100_SPO2_CONFIG, 0x4F);
  i2cWrite8(MAX30100_ADDR, MAX30100_LED_CONFIG, 0xFF);
  delay(10);
  return ((i2cRead8(MAX30100_ADDR, MAX30100_MODE_CONFIG) & 0x07) == 0x03);
}

// Đọc MAX30100 raw
bool readMAX30100Raw(uint16_t &ir, uint16_t &red){
  uint8_t buf[4];
  i2cReadBytes(MAX30100_ADDR, MAX30100_FIFO_DATA, buf, 4);
  ir  = (buf[0] << 8) | buf[1];
  red = (buf[2] << 8) | buf[3];
  return true;
}

// SHT30
bool readSHT30(float &t, float &h){
  Wire.beginTransmission(SHT30_ADDR);
  Wire.write(0x24);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) return false;

  delay(20);
  Wire.requestFrom(SHT30_ADDR, (uint8_t)6);
  if (Wire.available() < 6) return false;

  uint8_t d[6];
  for (int i=0;i<6;i++) d[i] = Wire.read();

  uint16_t rawT = (d[0]<<8)|d[1];
  uint16_t rawH = (d[3]<<8)|d[4];

  t = -45.0 + 175.0 * (rawT / 65535.0);
  h = 100.0 * (rawH / 65535.0);
  return true;
}

bool haveMAX=false, haveSHT=false;
float g_temperature=NAN, g_humidity=NAN;

// HR & SpO2
const int SAMPLE_INTERVAL_MS=10;
uint32_t lastSampleTime=0;
const int BUFFER_SIZE=200;
uint16_t irBuffer[BUFFER_SIZE], redBuffer[BUFFER_SIZE];
int bufferIndex=0;
bool bufferFilled=false;

float heartRate=0;
bool validHR=false;
float spo2=0;
bool validSpO2=false;

bool fingerOnSensor=false;
float dcIR_ema=0, acAbsAvg=0;
const float DC_ALPHA=0.95f;
const float NOISE_ALPHA=0.95f;
const uint32_t MIN_RR_MS=400, MAX_RR_MS=1500, REFRACTORY_MS=300;
const uint16_t IR_FINGER_ON=5000, IR_FINGER_OFF=2000;

const int HR_AVG_N=5;
uint16_t rrIntervals[HR_AVG_N];
int rrIndex=0, rrCount=0;
const float MAX_HR_JUMP=20;

bool lastAbove=false;
uint32_t lastPeakTime=0;

void sampleMaxAndUpdateHR(){
  if (millis() - lastSampleTime < SAMPLE_INTERVAL_MS) return;
  lastSampleTime = millis();

  uint16_t ir=0, red=0;
  if (!readMAX30100Raw(ir, red)) return;
  int idx = bufferIndex++;

  irBuffer[idx]  = ir;
  redBuffer[idx] = red;
  if (bufferIndex >= BUFFER_SIZE){ bufferIndex=0; bufferFilled=true; }

  int count = bufferFilled ? BUFFER_SIZE : bufferIndex;
  if (count < 20) return;

  if (!fingerOnSensor){
    if (ir > IR_FINGER_ON){
      fingerOnSensor=true;
      validHR=false;
      rrIndex=rrCount=0;
      lastPeakTime=0;
      lastAbove=false;
      dcIR_ema=ir;
      acAbsAvg=0;
    } else { validHR=false; return; }
  } else {
    if (ir < IR_FINGER_OFF){
      fingerOnSensor=false;
      validHR=false;
      rrIndex=rrCount=0;
      lastPeakTime=0;
      lastAbove=false;
      return;
    }
  }

  dcIR_ema = DC_ALPHA*dcIR_ema + (1.0-DC_ALPHA)*ir;
  float ac= ir - dcIR_ema;
  float acAbs=fabs(ac);
  acAbsAvg = NOISE_ALPHA*acAbsAvg + (1.0-NOISE_ALPHA)*acAbs;

  if (acAbsAvg < 25){
    validHR=false;
    return;
  }

  float threshold = acAbsAvg*0.6;
  if (threshold < 20) threshold=20;

  bool above = (ac > threshold);
  uint32_t now = millis();

  if (lastPeakTime != 0 && (now-lastPeakTime) < REFRACTORY_MS){
    lastAbove=above;
    return;
  }

  if (!lastAbove && above){
    if (lastPeakTime != 0){
      uint32_t interval= now-lastPeakTime;
      if (interval>MIN_RR_MS && interval<MAX_RR_MS){
        rrIntervals[rrIndex] = interval;
        rrIndex = (rrIndex+1) % HR_AVG_N;
        if (rrCount < HR_AVG_N) rrCount++;

        float sum=0;
        for (int i=0;i<rrCount;i++) sum += rrIntervals[i];
        float avgRR = sum / rrCount;
        float newHR = 60000.0 / avgRR;

        if (validHR && fabs(newHR-heartRate) > MAX_HR_JUMP){
          // spike -> bỏ
        } else {
          if (!validHR) heartRate=newHR;
          else heartRate = 0.9*heartRate + 0.1*newHR;
          validHR=true;
        }
      }
    }
    lastPeakTime=now;
  }
  lastAbove=above;
}

void computeSpO2(){
  int count = bufferFilled ? BUFFER_SIZE : bufferIndex;
  if (count < 50){ validSpO2=false; return; }

  uint32_t sumIR=0,sumRED=0;
  for (int i=0;i<count;i++){ sumIR+=irBuffer[i]; sumRED+=redBuffer[i]; }

  float dcIR = (float)sumIR/count;
  float dcRED= (float)sumRED/count;

  double sqIR=0, sqRED=0;
  for (int i=0;i<count;i++){
    float xIR = irBuffer[i]-dcIR;
    float xRED= redBuffer[i]-dcRED;
    sqIR  += xIR*xIR;
    sqRED += xRED*xRED;
  }

  float acIR  = sqrt(sqIR/count);
  float acRED = sqrt(sqRED/count);

  if (dcIR<1000 || dcRED<1000 || acIR<1 || acRED<1){
    validSpO2=false; return;
  }

  float R = (acRED/dcRED) / (acIR/dcIR);
  float s = 110.0 - 25.0*R;
  if (s>100) s=100;
  if (s<50)  s=50;
  spo2=s;
  validSpO2=true;
}

// ================== SIM + GPS ==================
TinyGPSPlus gps;
HardwareSerial GPS_Serial(2);

#define MCU_SIM_BAUDRATE 115200
#define MCU_SIM_TX_PIN   2
#define MCU_SIM_RX_PIN   4
#define MCU_SIM_EN_PIN   15

#define PHONE_NUMBER "+84349294684"

HardwareSerial SIM_Serial(1);

// ThingSpeak
const char* TS_API_KEY = "YNNOSS5T98TAYGEO";

// timers
unsigned long lastGpsPrint=0;
unsigned long lastTsSend=0;
unsigned long lastSensorPrint=0;
unsigned long lastSmsTime=0;
const unsigned long SMS_INTERVAL_MS=60000;
const unsigned long TS_INTERVAL_MS = 20000;  // 20s

// ====== SHARE DATA GIỮA LOOP VÀ TASK SIM ======
volatile bool  tsReqPending  = false;  // yêu cầu gửi ThingSpeak
volatile bool  smsReqPending = false;  // yêu cầu gửi SMS

volatile float latestLat = 0;
volatile float latestLon = 0;
volatile bool  latestFix = false;

volatile float tsLat = 0, tsLon = 0;   // snapshot để gửi TS
volatile float smsLat = 0, smsLon = 0; // snapshot để gửi SMS

// ========== SIM helpers ==========
void flushSIM(){
  while (SIM_Serial.available()) SIM_Serial.read();
}

bool sim_wait_for(const char *target, unsigned long timeout){
  String resp = "";
  unsigned long start = millis();

  while (millis() - start < timeout) {
    while (SIM_Serial.available()) {
      char c = SIM_Serial.read();
      Serial.print(c);
      resp += c;
      if (resp.indexOf(target) >= 0) return true;
    }
    vTaskDelay(1);
  }
  return false;
}

bool sim_cmd_ok(const char *cmd, unsigned long timeout=3000){
  flushSIM();
  Serial.print(">> ");
  Serial.println(cmd);
  SIM_Serial.println(cmd);
  return sim_wait_for("OK", timeout);
}

// Khởi tạo 4G / PDP
bool simSetup4G() {
  if (!sim_cmd_ok("AT", 2000)) return false;
  sim_cmd_ok("ATE0", 2000);

  if (!sim_cmd_ok("AT+CPIN?", 5000)) return false;
  sim_cmd_ok("AT+CSQ",   2000);
  sim_cmd_ok("AT+CREG?", 5000);
  sim_cmd_ok("AT+CGREG?",5000);

  sim_cmd_ok("AT+CGDCONT=1,\"IP\",\"v-internet\"", 5000);

  Serial.println("OK[SIM] NETOPEN...");
  flushSIM();
  Serial.println(">> AT+NETOPEN");
  SIM_Serial.println("AT+NETOPEN");

  String resp = "";
  unsigned long start = millis();
  while (millis() - start < 15000) {
    while (SIM_Serial.available()) {
      char c = SIM_Serial.read();
      Serial.print(c);
      resp += c;
    }
    vTaskDelay(1);
  }

  if (resp.indexOf("OK") == -1 &&
      resp.indexOf("Network is already opened") == -1) {
    Serial.println("[SIM] NETOPEN FAIL");
    return false;
  }

  Serial.println("[SIM] NETOPEN OK");
  sim_cmd_ok("AT+IPADDR", 5000);
  Serial.println("[SIM] 4G socket service READY");
  return true;
}

// HTTP GET tới host:port bằng SIM
bool simHttpGet(const char* host, uint16_t port, const String& req) {
  flushSIM();

  {
    String cmd = String("AT+CIPOPEN=1,\"TCP\",\"") + host + "\"," + String(port);
    Serial.print(">> ");
    Serial.println(cmd);
    SIM_Serial.println(cmd);

    if (!sim_wait_for("+CIPOPEN: 1,0", 20000)) {
      Serial.println("[SIM] CIPOPEN FAIL");
      return false;
    }
    Serial.println("[SIM] CIPOPEN OK");
  }

  {
    String cmd = String("AT+CIPSEND=1,") + req.length();
    flushSIM();
    Serial.print(">> ");
    Serial.println(cmd);
    SIM_Serial.println(cmd);

    if (!sim_wait_for(">", 10000)) {
      Serial.println("[SIM] no '>' prompt after CIPSEND");
      return false;
    }
  }

  Serial.println("[SIM] Sending HTTP GET...");
  SIM_Serial.print(req);

  if (!sim_wait_for("+CIPSEND: 1,", 15000) && !sim_wait_for("OK", 5000)) {
    Serial.println("[SIM] CIPSEND maybe FAIL");
  }

  sim_wait_for("+IPD", 5000);
  sim_wait_for("CIPCLOSE", 10000);

  SIM_Serial.println("AT+CIPCLOSE=1");
  sim_wait_for("OK", 5000);

  Serial.println("[SIM] HTTP done, socket closed");
  return true;
}

// Gửi SMS vị trí
void sendLocationSMS(float lat, float lon){
  flushSIM();
  SIM_Serial.println("AT+CMGF=1");
  delay(500);

  SIM_Serial.print("AT+CMGS=\"");
  SIM_Serial.print(PHONE_NUMBER);
  SIM_Serial.println("\"");
  delay(500);

  String msg="CANH BAO VI TRI!\nLat: "+String(lat,6)+
             "\nLon: "+String(lon,6)+
             "\nMap: https://maps.google.com/?q="+String(lat,6)+","+String(lon,6);

  SIM_Serial.print(msg);
  delay(200);
  SIM_Serial.write(0x1A);
  delay(3000);

  Serial.println("SMS sent.");
}

// GPS helpers
void updateGPSStream(){
  while (GPS_Serial.available()) {
    gps.encode(GPS_Serial.read());
  }
}

// Gửi dữ liệu lên ThingSpeak qua 4G (dùng toạ độ đã snapshot)
void sendToThingSpeak(float lat, float lon) {
  float temp = isnan(g_temperature) ? 0 : g_temperature;
  float hum  = isnan(g_humidity)    ? 0 : g_humidity;

  int hrInt = (haveMAX && validHR)   ? (int)round(heartRate) : 0;
  int oxInt = (haveMAX && validSpO2) ? (int)round(spo2)      : 0;

  String url = "/update?api_key=";
  url += TS_API_KEY;

  url += "&field1=";
  url += String(lat, 6);

  url += "&field2=";
  url += String(lon, 6);

  url += "&field3=";
  url += String(temp, 1);

  url += "&field4=";
  url += String(hum, 1);

  url += "&field5=";
  url += hrInt;

  url += "&field6=";
  url += oxInt;

  String req = "GET ";
  req += url;
  req += " HTTP/1.1\r\n"
         "Host: api.thingspeak.com\r\n"
         "Connection: close\r\n\r\n";

  Serial.println("[TS] HTTP GET via SIM:");
  Serial.println(req);

  if (simHttpGet("api.thingspeak.com", 80, req)) {
    Serial.println("[TS] OK via SIM");
  } else {
    Serial.println("[TS] FAIL via SIM");
  }
}

// ========== TASK SIM (FreeRTOS) ==========
void taskSIM(void *pvParameters) {
  (void) pvParameters;

  for (;;) {
    // Ưu tiên SMS
    if (smsReqPending) {
      float lat = smsLat;
      float lon = smsLon;
      smsReqPending = false;

      Serial.println("[SIM-TASK] Sending SMS...");
      sendLocationSMS(lat, lon);
      Serial.println("[SIM-TASK] SMS done.");
    }

    // Sau đó tới ThingSpeak
    if (tsReqPending) {
      float lat = tsLat;
      float lon = tsLon;
      tsReqPending = false;

      Serial.println("[SIM-TASK] Sending TS via SIM...");
      sendToThingSpeak(lat, lon);
      Serial.println("[SIM-TASK] TS done.");
    }

    vTaskDelay(pdMS_TO_TICKS(100));  // nghỉ 100 ms
  }
}

// ================== HELPER RELAY ==================
void relayWrite(int pin, bool on) {
  if (RELAY_ACTIVE_LOW) digitalWrite(pin, on ? LOW : HIGH);
  else digitalWrite(pin, on ? HIGH : LOW);
}

// ================== HELPER NƯỚC (moving average) ==================
int addWaterSampleAndGetAvg(int sample) {
  waterBuf[waterIdx++] = sample;
  if (waterIdx >= WATER_SAMPLES) {
    waterIdx = 0;
    waterBufFilled = true;
  }
  int count = waterBufFilled ? WATER_SAMPLES : waterIdx;
  long sum = 0;
  for (int i = 0; i < count; ++i) sum += waterBuf[i];
  return (int)(sum / count);
}

// ================== SETUP ==================
void setup(){
  Serial.begin(115200);
  delay(2000);

  pinMode(BUTTON_ON,  INPUT_PULLUP);
  pinMode(BUTTON_OFF, INPUT_PULLUP);

  // Relay pins
  pinMode(RELAY_PUMP_PIN, OUTPUT);
  pinMode(RELAY_ALARM_PIN, OUTPUT);
  // đảm bảo relay OFF ban đầu
  relayWrite(RELAY_PUMP_PIN, false);
  relayWrite(RELAY_ALARM_PIN, false);
  relayPumpState = false;
  relayAlarmState = false;

  // Khởi tạo buffer nước bằng giá trị đọc đầu tiên
  int initSample = analogRead(SENSOR_WATER);
  for (int i=0;i<WATER_SAMPLES;i++) waterBuf[i] = initSample;
  waterIdx = 0;
  waterBufFilled = true;

  Wire.begin(21,22);
  Wire.setClock(100000);

  float T0,H0;
  haveSHT = readSHT30(T0,H0);
  haveMAX = initMAX30100();
  g_temperature=T0;
  g_humidity=H0;

  pinMode(MCU_SIM_EN_PIN, OUTPUT);
  digitalWrite(MCU_SIM_EN_PIN, LOW); // hoặc HIGH tùy mạch SIM của bạn

  GPS_Serial.begin(9600, SERIAL_8N1, 16,17); // GPS RX/TX
  SIM_Serial.begin(MCU_SIM_BAUDRATE, SERIAL_8N1, MCU_SIM_RX_PIN, MCU_SIM_TX_PIN);

  delay(8000); // chờ module SIM lên

  Serial.println("== INIT SIM ==");
  sim_cmd_ok("AT");
  sim_cmd_ok("ATI");
  sim_cmd_ok("AT+CIMI");

  if (simSetup4G()) {
    Serial.println("== 4G READY ==");
  } else {
    Serial.println("== 4G FAIL ==");
  }

  // Tạo task SIM trên core 0
  xTaskCreatePinnedToCore(
    taskSIM,
    "SIM_TASK",
    8192,
    NULL,
    1,
    NULL,
    0   // core 0
  );
}

// ================== LOOP (TASK CẢM BIẾN) ==================
void loop() {
  static unsigned long lastRelayToggleTime = 0;

  // luôn feed GPS để nhanh fix
  updateGPSStream();

  // ====== CẬP NHẬT GPS + LƯU TOẠ ĐỘ ======
  if (millis() - lastGpsPrint >= 2000) {
    lastGpsPrint = millis();

    if (gps.location.isValid()) {
      double lat = gps.location.lat();
      double lon = gps.location.lng();

      latestLat = lat;
      latestLon = lon;
      latestFix = true;

      Serial.print("Lat: "); Serial.print(lat, 6);
      Serial.print("  Lng: "); Serial.println(lon, 6);
    } else {
      latestFix = false;
      Serial.println("Chua co fix GPS.");
    }
  }

  // ====== ĐỌC NÚT ======
  int readingOn  = digitalRead(BUTTON_ON);
  int readingOff = digitalRead(BUTTON_OFF);

  // DEBUG: in ra khi có thay đổi raw
  static int prevOn  = HIGH;
  static int prevOff = HIGH;
  if (readingOn != prevOn) {
    Serial.print("[RAW]  BUTTON_ON  = ");
    Serial.println(readingOn);
    prevOn = readingOn;
  }
  if (readingOff != prevOff) {
    Serial.print("[RAW]  BUTTON_OFF = ");
    Serial.println(readingOff);
    prevOff = readingOff;
  }

  // ====== BUTTON ON: NHẤN 1 CÁI -> BẬT smsEnabled ======
  if (readingOn == LOW && lastBtnOn == HIGH) {
    smsEnabled = true;
    lastSmsTime = 0;
    Serial.println("[BTN] ON pressed  -> smsEnabled = TRUE");
  }
  lastBtnOn = readingOn;

  // ====== BUTTON OFF: NHẤN 1 CÁI -> TẮT smsEnabled ======
  if (readingOff == LOW && lastBtnOff == HIGH) {
    smsEnabled = false;
    Serial.println("[BTN] OFF pressed -> smsEnabled = FALSE");
  }
  lastBtnOff = readingOff;

  // ====== CẢM BIẾN NƯỚC (đọc + moving average) ======
  int rawWater = analogRead(SENSOR_WATER);
  int waterAvg = addWaterSampleAndGetAvg(rawWater);

  // Dùng hysteresis để tránh relay nhảy liên tục
  static bool alarmOn = false;
  int highThreshold = WATER_THRESHOLD;
  int lowThreshold  = WATER_THRESHOLD - WATER_HYSTERESIS;
  if (!alarmOn && waterAvg > highThreshold) alarmOn = true;
  else if (alarmOn && waterAvg < lowThreshold) alarmOn = false;

  // ====== ĐIỀU KHIỂN RELAY THEO LOGIC DỮ LIỆU =========
  // Cả 2 relay bật khi alarmOn = true, tắt khi alarmOn = false
  bool desiredPumpState = alarmOn;
  bool desiredAlarmState = alarmOn;

  // Chỉ cho phép thay đổi trạng thái relay nếu đã qua RELAY_MIN_TOGGLE_MS kể từ lần thay đổi trước
  if ((desiredPumpState != relayPumpState || desiredAlarmState != relayAlarmState) &&
      (millis() - lastRelayToggleTime >= RELAY_MIN_TOGGLE_MS)) {

    if (desiredPumpState != relayPumpState) {
      relayPumpState = desiredPumpState;
      relayWrite(RELAY_PUMP_PIN, relayPumpState);
      Serial.print("[RELAY] PUMP (pin ");
      Serial.print(RELAY_PUMP_PIN);
      Serial.print(") -> ");
      Serial.println(relayPumpState ? "ON" : "OFF");
    }

    if (desiredAlarmState != relayAlarmState) {
      relayAlarmState = desiredAlarmState;
      relayWrite(RELAY_ALARM_PIN, relayAlarmState);
      Serial.print("[RELAY] ALARM (pin ");
      Serial.print(RELAY_ALARM_PIN);
      Serial.print(") -> ");
      Serial.println(relayAlarmState ? "ON" : "OFF");
    }

    lastRelayToggleTime = millis();
  }

  // ====== LOG và các chức năng khác ======
  if (millis() - lastSensorPrint >= 1000) {
    lastSensorPrint = millis();

    float t, h;
    if (readSHT30(t, h)) {
      g_temperature = t;
      g_humidity    = h;
    }
    if (haveMAX) computeSpO2();

    Serial.print("[SENSOR] raw=");
    Serial.print(rawWater);
    Serial.print(" avg=");
    Serial.print(waterAvg);
    Serial.print(" | T=");
    if (!isnan(g_temperature)) Serial.print(g_temperature, 1); else Serial.print("--");
    Serial.print(" °C | H=");
    if (!isnan(g_humidity)) Serial.print(g_humidity, 1); else Serial.print("--");
    Serial.print(" %");

    Serial.print(" | HR=");
    if (haveMAX && validHR) Serial.print(heartRate, 0); else Serial.print("--");
    Serial.print(" bpm | SpO2=");
    if (haveMAX && validSpO2) Serial.print(spo2, 0); else Serial.print("--");
    Serial.print(" %");

    Serial.print(" | SMS=");
    Serial.print(smsEnabled ? "ON" : "OFF");
    Serial.print(" | RELAY_PUMP=");
    Serial.print(relayPumpState ? "ON" : "OFF");
    Serial.print(" | RELAY_ALARM=");
    Serial.println(relayAlarmState ? "ON" : "OFF");
  }

  // ====== NƯỚC VƯỢT NGƯỠNG: gửi TS và SMS nếu bật ======
  if (alarmOn) {
    if (haveMAX) sampleMaxAndUpdateHR();

    // Lên lịch gửi ThingSpeak mỗi TS_INTERVAL_MS
    if (millis() - lastTsSend >= TS_INTERVAL_MS) {
      lastTsSend = millis();
      if (haveMAX) computeSpO2();

      if (latestFix) {
        tsLat = latestLat;
        tsLon = latestLon;
        tsReqPending = true;     // để taskSIM gửi
      } else {
        Serial.println("[TS] Skip (no GPS fix)");
      }
    }

    // ====== LÊN LỊCH GỬI SMS ======
    if (smsEnabled && (millis() - lastSmsTime >= SMS_INTERVAL_MS)) {
      Serial.println("[SMS] Check GPS...");
      if (latestFix) {
        smsLat = latestLat;
        smsLon = latestLon;
        smsReqPending = true;    // để taskSIM gửi
        Serial.println("[SMS] GPS FIX OK -> Queue SMS...");
      } else {
        Serial.println("[SMS] GPS CHUA FIX -> KHONG queue SMS.");
      }
      lastSmsTime = millis();
    }
  } else {
    validHR = false;
    validSpO2 = false;
  }

  // cho CPU nghỉ 5ms (giúp các task khác chạy)
  delay(5);
}