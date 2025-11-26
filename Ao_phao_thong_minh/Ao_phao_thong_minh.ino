/*
  ESP32: GPS + SIM + MAX30100 + SHT30 + ThingSpeak + SENSOR WATER
  Full optimized, non-blocking, with HR & SpO2 algorithm integrated.

  - Sửa lỗi: ThingSpeak.setField(...) ambiguous -> cast to float
  - MAX30100: lấy mẫu ~100Hz, xử lý HR non-blocking
  - SHT30: đọc single-shot high repeatability
  - SIM: basic SMS sending via AT commands
  - GPS: TinyGPS++ trên UART2
*/

#include <Wire.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <TinyGPSPlus.h>
#include <ThingSpeak.h>
#include <math.h>

// ------------------ PIN & ADDR ------------------
#define SENSOR_WATER      33

// I2C addresses
#define MAX30100_ADDR 0x57
#define SHT30_ADDR    0x44

// MAX30100 registers
#define MAX30100_FIFO_WR_PTR    0x02
#define MAX30100_OVF_COUNTER    0x03
#define MAX30100_FIFO_RD_PTR    0x04
#define MAX30100_FIFO_DATA      0x05
#define MAX30100_MODE_CONFIG    0x06
#define MAX30100_SPO2_CONFIG    0x07
#define MAX30100_LED_CONFIG     0x09

// ------------------ I2C HELPERS ------------------
void i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t i2cRead8(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)1);
  if (Wire.available()) return Wire.read();
  return 0;
}

void i2cReadBytes(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, len);
  uint8_t i = 0;
  while (Wire.available() && i < len) buf[i++] = Wire.read();
}

// ------------------ MAX30100 INIT & READ ------------------
bool initMAX30100() {
  // Reset FIFO pointers
  i2cWrite8(MAX30100_ADDR, MAX30100_FIFO_WR_PTR, 0x00);
  i2cWrite8(MAX30100_ADDR, MAX30100_OVF_COUNTER, 0x00);
  i2cWrite8(MAX30100_ADDR, MAX30100_FIFO_RD_PTR, 0x00);

  // SpO2 mode
  i2cWrite8(MAX30100_ADDR, MAX30100_MODE_CONFIG, 0x03);

  // SPO2_CONFIG: hi-res, 100 Hz, 1600us
  i2cWrite8(MAX30100_ADDR, MAX30100_SPO2_CONFIG, 0x4F);

  // LED current: RED=IR=0x0F (cả 2 = 0xFF)
  i2cWrite8(MAX30100_ADDR, MAX30100_LED_CONFIG, 0xFF);

  delay(10);
  uint8_t mode = i2cRead8(MAX30100_ADDR, MAX30100_MODE_CONFIG);
  return (mode & 0x07) == 0x03;
}

// Đọc 1 sample IR + RED từ FIFO (4 byte)
bool readMAX30100Raw(uint16_t &ir, uint16_t &red) {
  uint8_t buf[4];
  i2cReadBytes(MAX30100_ADDR, MAX30100_FIFO_DATA, buf, 4);

  // Nếu bus bị lỗi hoặc không đủ dữ liệu thì sẽ trả về 0 (vẫn return true để caller xử lý)
  ir  = ((uint16_t)buf[0] << 8) | buf[1];
  red = ((uint16_t)buf[2] << 8) | buf[3];
  return true;
}

// ------------------ SHT30 READ ------------------
bool readSHT30(float &temperature, float &humidity) {
  Wire.beginTransmission(SHT30_ADDR);
  Wire.write(0x24);
  Wire.write(0x00);  // single shot, high repeatability
  if (Wire.endTransmission() != 0) return false;
  delay(20); // chờ đo

  Wire.requestFrom(SHT30_ADDR, (uint8_t)6);
  if (Wire.available() < 6) return false;

  uint8_t data[6];
  for (int i = 0; i < 6; i++) data[i] = Wire.read();

  uint16_t rawT  = ((uint16_t)data[0] << 8) | data[1];
  uint16_t rawRH = ((uint16_t)data[3] << 8) | data[4];

  temperature = -45.0f + 175.0f * ((float)rawT / 65535.0f);
  humidity    = 100.0f * ((float)rawRH / 65535.0f);
  return true;
}

// ------------------ HR + SpO2 ALGORITHM (from your original) ------------------
// Sampling config
const int SAMPLE_INTERVAL_MS = 10;   // ~100 Hz
uint32_t lastSampleTime = 0;

// Buffers
const int BUFFER_SIZE = 200;        // ~2s dữ liệu
uint16_t irBuffer[BUFFER_SIZE];
uint16_t redBuffer[BUFFER_SIZE];
int bufferIndex = 0;
bool bufferFilled = false;

// HR/SpO2 results
float heartRate = 0;
bool validHR = false;
float spo2 = 0;
bool validSpO2 = false;

// Finger detection + signal processing vars
bool fingerOnSensor = false;
float dcIR_ema = 0;
float acAbsAvg = 0;
const float DC_ALPHA   = 0.95f;
const float NOISE_ALPHA= 0.95f;
const uint32_t MIN_RR_MS     = 400;
const uint32_t MAX_RR_MS     = 1500;
const uint32_t REFRACTORY_MS = 300;
const uint16_t IR_FINGER_ON  = 5000;
const uint16_t IR_FINGER_OFF = 2000;

// RR smoothing
const int HR_AVG_N = 5;
uint16_t rrIntervals[HR_AVG_N];
int rrIndex = 0;
int rrCount = 0;
const float MAX_HR_JUMP = 20.0f;
bool lastAbove = false;
uint32_t lastPeakTime = 0;
uint16_t lastIR = 0;

// Globals for SHT30 values
float g_temperature = NAN;
float g_humidity    = NAN;

// sample + HR update (non-blocking)
void sampleMaxAndUpdateHR() {
  // Lấy mẫu đúng tần số SAMPLE_INTERVAL_MS (~100 Hz)
  if (millis() - lastSampleTime < SAMPLE_INTERVAL_MS) return;
  lastSampleTime = millis();

  uint16_t ir = 0, red = 0;
  if (!readMAX30100Raw(ir, red)) return;
  lastIR = ir;

  // Ghi vào buffer cho SpO2
  irBuffer[bufferIndex]  = ir;
  redBuffer[bufferIndex] = red;
  bufferIndex++;
  if (bufferIndex >= BUFFER_SIZE) {
    bufferIndex = 0;
    bufferFilled = true;
  }

  int count = bufferFilled ? BUFFER_SIZE : bufferIndex;
  if (count < 20) return;   // chưa đủ mẫu để xử lý HR

  // PHÁT HIỆN CÓ / KHÔNG CÓ TAY
  if (!fingerOnSensor) {
    if (ir > IR_FINGER_ON) {
      fingerOnSensor = true;
      // reset HR state
      validHR      = false;
      heartRate    = 0;
      rrIndex      = 0;
      rrCount      = 0;
      lastPeakTime = 0;
      lastAbove    = false;
      dcIR_ema     = ir;
      acAbsAvg     = 0;
    } else {
      validHR = false;
      return;
    }
  } else {
    if (ir < IR_FINGER_OFF) {
      fingerOnSensor = false;
      validHR      = false;
      heartRate    = 0;
      rrIndex      = 0;
      rrCount      = 0;
      lastPeakTime = 0;
      lastAbove    = false;
      return;
    }
  }

  // LỌC DC & TÍNH AC
  if (dcIR_ema == 0) dcIR_ema = ir;
  else dcIR_ema = DC_ALPHA * dcIR_ema + (1.0f - DC_ALPHA) * ir;

  float ac = ir - dcIR_ema;
  float acAbs = fabs(ac);
  acAbsAvg = NOISE_ALPHA * acAbsAvg + (1.0f - NOISE_ALPHA) * acAbs;

  const float MIN_AC = 25.0f;
  if (acAbsAvg < MIN_AC) {
    validHR = false;
    return;
  }

  // Ngưỡng động
  float threshold = acAbsAvg * 0.6f;
  const float MIN_THR = 20.0f;
  if (threshold < MIN_THR) threshold = MIN_THR;

  bool above = (ac > threshold);
  uint32_t now = millis();

  // Refractory
  if (lastPeakTime != 0 && (now - lastPeakTime) < REFRACTORY_MS) {
    lastAbove = above;
    return;
  }

  // PHÁT HIỆN ĐỈNH
  if (!lastAbove && above) {
    if (lastPeakTime != 0) {
      uint32_t interval = now - lastPeakTime;
      if (interval > MIN_RR_MS && interval < MAX_RR_MS) {
        rrIntervals[rrIndex] = interval;
        rrIndex = (rrIndex + 1) % HR_AVG_N;
        if (rrCount < HR_AVG_N) rrCount++;

        float sum = 0;
        for (int i = 0; i < rrCount; i++) sum += rrIntervals[i];
        float avgRR = sum / rrCount;
        float newHR = 60000.0f / avgRR;

        if (validHR && fabs(newHR - heartRate) > MAX_HR_JUMP) {
          // ignore spike
        } else {
          if (!validHR) heartRate = newHR;
          else heartRate = 0.90f * heartRate + 0.10f * newHR;
          validHR = true;
        }
      }
    }
    lastPeakTime = now;
  }
  lastAbove = above;
}

// Tính SpO2 trên buffer (gọi không thường xuyên, vd mỗi 1s)
void computeSpO2() {
  int count = bufferFilled ? BUFFER_SIZE : bufferIndex;
  if (count < 50) {
    validSpO2 = false;
    return;
  }

  uint32_t sumIR = 0, sumRED = 0;
  for (int i = 0; i < count; i++) {
    sumIR  += irBuffer[i];
    sumRED += redBuffer[i];
  }
  float dcIR  = (float)sumIR / count;
  float dcRED = (float)sumRED / count;

  double sqSumIR  = 0;
  double sqSumRED = 0;
  for (int i = 0; i < count; i++) {
    float xIR  = irBuffer[i]  - dcIR;
    float xRED = redBuffer[i] - dcRED;
    sqSumIR  += xIR * xIR;
    sqSumRED += xRED * xRED;
  }

  float acIR  = sqrt(sqSumIR  / count);
  float acRED = sqrt(sqSumRED / count);

  if (dcIR < 1000 || dcRED < 1000 || acIR < 1 || acRED < 1) {
    validSpO2 = false;
    return;
  }

  float R = (acRED / dcRED) / (acIR / dcIR);
  float s = 110.0f - 25.0f * R;  // xấp xỉ
  if (s > 100) s = 100;
  if (s < 50)  s = 50;

  spo2 = s;
  validSpO2 = true;
}

// ------------------ FLAGS ------------------
bool haveMAX = false;
bool haveSHT = false;

// ------------------ GPS + SIM + ThingSpeak ------------------
TinyGPSPlus gps;
HardwareSerial GPS_Serial(2);   // UART2 cho GPS

// SIM definitions (UART1)
#define simSerial       Serial1
#define MCU_SIM_BAUDRATE 115200
#define MCU_SIM_TX_PIN   2
#define MCU_SIM_RX_PIN   4
#define MCU_SIM_EN_PIN   15

#define PHONE_NUMBER "+84967952009"

HardwareSerial SIM_Serial(1);

// WiFi / ThingSpeak
const char* ssid = "Nha 05 - P401";
const char* password = "66668888";

unsigned long myChannelNumber = 3165784;
const char* myWriteAPIKey = "YNNOSS5T98TAYGEO";

WiFiClient client;

// Timers
unsigned long lastSimCheck   = 0;
unsigned long lastGpsPrint   = 0;
unsigned long lastTsSend     = 0;
unsigned long lastSensorPrint = 0;

int senSor_Water = 0;

// ------------------ SIM helper ------------------
void flushSIM() {
  while (SIM_Serial.available()) SIM_Serial.read();
}

bool sim_wait_for(const char *target, unsigned long timeout) {
  String resp = "";
  unsigned long start = millis();
  while (millis() - start < timeout) {
    while (SIM_Serial.available()) {
      char c = SIM_Serial.read();
      // in ra Serial để debug
      if ((c >= 32 && c <= 126) || c == '\r' || c == '\n') {
        Serial.print(c);
      }
      resp += c;
      if (resp.indexOf(target) >= 0) {
        return true;  // đã thấy chuỗi mong muốn
      }
    }
  }
  return false; // timeout
}

bool sim_cmd_ok(const char *cmd, unsigned long timeout = 3000) {
  flushSIM();
  Serial.print(">> ");
  Serial.println(cmd);
  SIM_Serial.println(cmd);
  return sim_wait_for("OK", timeout);
}

void sendLocationSMS(float lat, float lon) {
  flushSIM();
  Serial.println("Gui lenh AT+CMGF=1");
  SIM_Serial.println("AT+CMGF=1");
  delay(1000);

  Serial.println("Gui lenh AT+CMGS...");
  SIM_Serial.print("AT+CMGS=\"");
  SIM_Serial.print(PHONE_NUMBER);
  SIM_Serial.println("\"");
  delay(1000);

  String msg = "CANH BAO VI TRI!\n";
  msg += "Lat: " + String(lat, 6) + "\n";
  msg += "Lon: " + String(lon, 6) + "\n";
  msg += "Map: https://maps.google.com/?q=";
  msg += String(lat, 6) + ",";
  msg += String(lon, 6);

  Serial.println("Gui noi dung SMS:");
  Serial.println(msg);

  SIM_Serial.print(msg);
  delay(200);

  SIM_Serial.write(0x1A);
  Serial.println("Da gui CTRL+Z, doi module gui SMS...");
  delay(5000);
}

// ------------------ GPS helper ------------------
void updateGPS() {
  while (GPS_Serial.available() > 0) {
    gps.encode(GPS_Serial.read());
  }
}

void printGPS() {
  if (gps.location.isValid()) {
    Serial.print("Lat: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print("  Lng: ");
    Serial.println(gps.location.lng(), 6);
  } else {
    Serial.println("Chua co fix GPS.");
  }
}

// ------------------ ThingSpeak send ------------------
void sendToThingSpeak() {
  if (!gps.location.isValid()) {
    Serial.println("Bo qua gui TS vi chua co GPS fix.");
    return;
  }

  float latitude  = (float)gps.location.lat();   // cast về float để tránh ambiguous
  float longitude = (float)gps.location.lng();

  String mapStr = "Map: https://maps.google.com/?q=";
  mapStr += String(latitude, 6);
  mapStr += ",";
  mapStr += String(longitude, 6);

  ThingSpeak.setField(1, latitude);
  ThingSpeak.setField(2, longitude);

  if (!isnan(g_temperature)) ThingSpeak.setField(3, (float)g_temperature);
  else ThingSpeak.setField(3, 0);

  if (!isnan(g_humidity)) ThingSpeak.setField(4, (float)g_humidity);
  else ThingSpeak.setField(4, 0);

  if (haveMAX && validHR) ThingSpeak.setField(5, (float)heartRate);
  else ThingSpeak.setField(5, 0);

  if (haveMAX && validSpO2) ThingSpeak.setField(6, (float)spo2);
  else ThingSpeak.setField(6, 0);

  ThingSpeak.setField(7, mapStr);

  int httpCode = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  Serial.print("ThingSpeak HTTP code: ");
  Serial.println(httpCode);

  if (httpCode == 200) {
    Serial.println("Gui SMS vi tri...");
    sendLocationSMS(latitude, longitude);
  }
}

// ------------------ SETUP ------------------
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println();
  Serial.println("ESP32: GPS + SIM + MAX30100 + SHT30 + ThingSpeak + SENSOR WATER");

  analogSetAttenuation(ADC_11db);

  // I2C
  Wire.begin(21, 22);
  Wire.setClock(100000);

  // Khởi tạo SHT30 & MAX30100
  float T0, H0;
  haveSHT = readSHT30(T0, H0);       // thử đọc 1 lần để check kết nối
  haveMAX = initMAX30100();

  Serial.print("SHT30   : "); Serial.println(haveSHT ? "OK" : "FAIL");
  Serial.print("MAX30100: "); Serial.println(haveMAX ? "OK" : "FAIL");

  // Power SIM
  pinMode(MCU_SIM_EN_PIN, OUTPUT);
  digitalWrite(MCU_SIM_EN_PIN, LOW);

  // GPS UART2 (TX = 17, RX = 16)
  GPS_Serial.begin(9600, SERIAL_8N1, 16, 17);
  Serial.println("Dang lay du lieu GPS...");

  // SIM UART1
  SIM_Serial.begin(MCU_SIM_BAUDRATE, SERIAL_8N1, MCU_SIM_RX_PIN, MCU_SIM_TX_PIN);
  delay(8000);
  sim_cmd_ok("AT");
  sim_cmd_ok("ATI");
  sim_cmd_ok("AT+CPIN?");
  sim_cmd_ok("AT+CSQ");
  sim_cmd_ok("AT+CIMI");

  // WiFi
  Serial.print("Dang ket noi WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.print("\nWiFi connected, IP: ");
  Serial.println(WiFi.localIP());

  // ThingSpeak
  ThingSpeak.begin(client);
  Serial.println("ThingSpeak ready.");

  // Init globals for display
  g_temperature = T0;
  g_humidity = H0;
}

// ------------------ LOOP ------------------
void loop() {
  // Cập nhật GPS liên tục
  senSor_Water = analogRead(SENSOR_WATER);
  if (senSor_Water > 2000) {
    updateGPS();
  }

  // Lấy mẫu MAX30100 liên tục (100Hz)
  if (haveMAX) sampleMaxAndUpdateHR();

  // In GPS mỗi 2s
  if (millis() - lastGpsPrint >= 2000) {
    printGPS();
    lastGpsPrint = millis();
  }

  // Gửi ThingSpeak + SMS mỗi 20s (chi khi sensor nước cao)
  if (millis() - lastTsSend >= 20000) {
    lastTsSend = millis();
    if (senSor_Water > 2000) {
      // Trước khi gửi, cập nhật SpO2 (khoảng 1s toán)
      if (haveMAX) computeSpO2();
      sendToThingSpeak();
    } else {
      Serial.println("Nuoc binh thuong -> khong gui TS.");
    }
  }

  // Mỗi 1s in dữ liệu cảm biến
  if (millis() - lastSensorPrint >= 1000) {
    lastSensorPrint = millis();

    // Cập nhật SHT30 - nếu fail thì giữ giá trị cũ
    if (haveSHT) {
      float ttmp, htmp;
      if (readSHT30(ttmp, htmp)) {
        g_temperature = ttmp;
        g_humidity = htmp;
      } else {
        Serial.println("SHT30 read FAIL (giu gia tri cu).");
      }
    }

    // computeSpO2 nếu chưa tính trong thời gian gửi
    if (haveMAX) {
      computeSpO2();
    }

    Serial.print("HR=");
    if (haveMAX && validHR) Serial.print(heartRate, 1); else Serial.print("NaN");
    Serial.print(" bpm, SpO2=");
    if (haveMAX && validSpO2) Serial.print(spo2, 1); else Serial.print("NaN");
    Serial.print("% | T=");
    if (!isnan(g_temperature)) Serial.print(g_temperature, 1); else Serial.print("Err");
    Serial.print("C, RH=");
    if (!isnan(g_humidity)) Serial.print(g_humidity, 1); else Serial.print("Err");
    Serial.print("% | Water=");
    Serial.println(senSor_Water);
  }
}
