/*
  ESP32: GPS + SIM + MAX30100 + SHT30 + ThingSpeak + SENSOR WATER

  - BUTTON_ON  (GPIO25): BẬT gửi SMS
  - BUTTON_OFF (GPIO26): TẮT gửi SMS
  - Gửi SMS 1 phút/lần khi:
        + Nước > WATER_THRESHOLD
        + smsEnabled = true
        + GPS đã fix (có tọa độ)
*/

#include <Wire.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <TinyGPSPlus.h>
#include <ThingSpeak.h>
#include <math.h>

// ================== PIN ==================
#define SENSOR_WATER 33
#define BUTTON_ON    25   // BẬT SMS
#define BUTTON_OFF   26   // TẮT SMS
const int WATER_THRESHOLD = 2000;

// Trạng thái gửi SMS
bool smsEnabled = false;

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

// HR & SpO2 (giữ nguyên thuật toán cũ của bạn, rút gọn comment)
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
          // spike -> bỏ qua
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

// WiFi + ThingSpeak
const char* ssid="MADATEK";
const char* password="Madatek68686868";
unsigned long myChannelNumber=3165784;
const char* myWriteAPIKey="YNNOSS5T98TAYGEO";
WiFiClient client;

// timers
unsigned long lastGpsPrint=0;
unsigned long lastTsSend=0;
unsigned long lastSensorPrint=0;
unsigned long lastSmsTime=0;
const unsigned long SMS_INTERVAL_MS=60000;

// SIM helpers
void flushSIM(){
  while (SIM_Serial.available()) SIM_Serial.read();
}

bool sim_wait_for(const char *target, unsigned long timeout){
  String resp="";
  unsigned long start=millis();
  while (millis()-start < timeout){
    while (SIM_Serial.available()){
      char c=SIM_Serial.read();
      Serial.print(c);
      resp += c;
      if (resp.indexOf(target)>=0) return true;
    }
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

void printGPS(){
  if (gps.location.isValid()){
    Serial.print("Lat: "); Serial.print(gps.location.lat(),6);
    Serial.print("  Lng: "); Serial.println(gps.location.lng(),6);
  } else Serial.println("Chua co fix GPS.");
}

void sendToThingSpeak(){
  if (!gps.location.isValid()){
    Serial.println("TS skip (no GPS fix).");
    return;
  }

  float lat = gps.location.lat();
  float lon = gps.location.lng();

  ThingSpeak.setField(1, lat);
  ThingSpeak.setField(2, lon);
  ThingSpeak.setField(3, isnan(g_temperature)?0:g_temperature);
  ThingSpeak.setField(4, isnan(g_humidity)?0:g_humidity);
  ThingSpeak.setField(5, (haveMAX && validHR)?heartRate:0);
  ThingSpeak.setField(6, (haveMAX && validSpO2)?spo2:0);

  String mapStr="https://maps.google.com/?q="+String(lat,6)+","+String(lon,6);
  ThingSpeak.setField(7, mapStr);

  int http = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  Serial.print("TS code: "); Serial.println(http);
}

// ================== SETUP ==================
void setup(){
  Serial.begin(115200);
  delay(2000);

  pinMode(BUTTON_ON,  INPUT_PULLUP);
  pinMode(BUTTON_OFF, INPUT_PULLUP);

  Wire.begin(21,22);
  Wire.setClock(100000);

  float T0,H0;
  haveSHT = readSHT30(T0,H0);
  haveMAX = initMAX30100();
  g_temperature=T0;
  g_humidity=H0;

  pinMode(MCU_SIM_EN_PIN, OUTPUT);
  digitalWrite(MCU_SIM_EN_PIN, LOW);

  GPS_Serial.begin(9600, SERIAL_8N1, 16,17);
  SIM_Serial.begin(MCU_SIM_BAUDRATE, SERIAL_8N1, MCU_SIM_RX_PIN, MCU_SIM_TX_PIN);

  delay(8000);
  sim_cmd_ok("AT");
  sim_cmd_ok("ATI");
  sim_cmd_ok("AT+CPIN?");
  sim_cmd_ok("AT+CSQ");
  sim_cmd_ok("AT+CIMI");

  WiFi.begin(ssid,password);
  while (WiFi.status()!=WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi OK");
  ThingSpeak.begin(client);
}

// ================== LOOP ==================
void loop() {
  // luôn feed GPS để nhanh fix
  updateGPSStream();

  // ====== ĐỌC NÚT ======
  int readingOn  = digitalRead(BUTTON_ON);
  int readingOff = digitalRead(BUTTON_OFF);

  // DEBUG: in ra khi có thay đổi raw
  static int prevOn  = HIGH;
  static int prevOff = HIGH;
  if (readingOn != prevOn) {
    Serial.print("[RAW]  BUTTON_ON  = ");
    Serial.println(readingOn);    // 1 = nhả, 0 = nhấn (INPUT_PULLUP)
    prevOn = readingOn;
  }
  if (readingOff != prevOff) {
    Serial.print("[RAW]  BUTTON_OFF = ");
    Serial.println(readingOff);
    prevOff = readingOff;
  }

// ====== BUTTON ON: NHẤN 1 CÁI -> BẬT smsEnabled, GIỮ STATE ======
  if (readingOn == LOW && lastBtnOn == HIGH) {   // cạnh HIGH -> LOW = nhấn
    smsEnabled = true;        // lưu state = ON
    lastSmsTime = 0;          // reset bộ đếm gửi SMS
    Serial.println("[BTN] ON pressed  -> smsEnabled = TRUE");
  }
  lastBtnOn = readingOn;

// ====== BUTTON OFF: NHẤN 1 CÁI -> TẮT smsEnabled, GIỮ STATE ======
  if (readingOff == LOW && lastBtnOff == HIGH) { // cạnh HIGH -> LOW = nhấn
    smsEnabled = false;       // lưu state = OFF
    Serial.println("[BTN] OFF pressed -> smsEnabled = FALSE");
  }
  lastBtnOff = readingOff;

// ====== BUTTON OFF ======
if (readingOff != lastBtnOff) {
  lastDebOff = millis();
}
if ((millis() - lastDebOff) > debounceDelay) {
  if (readingOff == LOW && lastBtnOff == HIGH) {
    smsEnabled = false;
    Serial.println("[BTN] OFF pressed ➜ smsEnabled = FALSE");
  }
}
lastBtnOff = readingOff;

  // ====== CẢM BIẾN NƯỚC ======
  int water = analogRead(SENSOR_WATER);
  bool alarmOn = (water > WATER_THRESHOLD);

  // ====== NƯỚC VƯỢT NGƯỠNG ======
  if (alarmOn) {
    if (haveMAX) sampleMaxAndUpdateHR();

    if (millis() - lastGpsPrint >= 2000) {
      printGPS();
      lastGpsPrint = millis();
    }

    if (millis() - lastTsSend >= 20000) {
      lastTsSend = millis();
      if (haveMAX) computeSpO2();
      sendToThingSpeak();
    }

    if (millis() - lastSensorPrint >= 1000) {
      lastSensorPrint = millis();

      float t, h;
      if (readSHT30(t, h)) {
        g_temperature = t;
        g_humidity    = h;
      }
      if (haveMAX) computeSpO2();

      Serial.print("[ALARM] WATER=");
      Serial.print(water);

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
      Serial.println(smsEnabled ? "ON" : "OFF");
    }

    // ====== GỬI SMS ======
    if (smsEnabled && (millis() - lastSmsTime >= SMS_INTERVAL_MS)) {
      Serial.println("[SMS] Check GPS...");
      if (gps.location.isValid()) {
        float lat = gps.location.lat();
        float lon = gps.location.lng();
        Serial.println("[SMS] GPS FIX OK -> Gui SMS...");
        sendLocationSMS(lat, lon);
      } else {
        Serial.println("[SMS] GPS CHUA FIX -> KHONG gui SMS.");
      }
      lastSmsTime = millis();
    }
  }

  // ====== NƯỚC BÌNH THƯỜNG ======
  else {
    if (millis() - lastSensorPrint >= 1000) {
      lastSensorPrint = millis();
      Serial.print("Water normal: ");
      Serial.print(water);
      Serial.print(" | SMS=");
      Serial.println(smsEnabled ? "ON" : "OFF");
    }
    validHR = false;
    validSpO2 = false;
  }
}
