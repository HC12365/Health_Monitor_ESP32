/*********************************************
 * 心率 + 血氧 + 环境温湿度 + 烟雾报警
 * 双色LED + 低电平蜂鸣器 + OLED + OneNET MQTT
 * 稳定版：JSON 分段拼接 + 报警记忆上传
 *********************************************/
#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "DHT.h"
#include "MAX30105.h"
#include "heartRate.h"

// ==================== 你的配置 ====================
const char* ssid = "n97";
const char* password = "qwertyui";
const char* mqttPassword = "version=2018-10-31&res=products%2FDrq3Q0fZ05%2Fdevices%2Fesp32&et=1999999999&method=md5&sign=RD5%2Fkj8iJ9K0btcVXIQGew%3D%3D";

// ==================== OneNET 固定参数 ====================
const char* mqttServer = "studio-mqtt.heclouds.com";
const int   mqttPort   = 1883;
const char* clientId   = "esp32";
const char* username   = "Drq3Q0fZ05";

// 物模型标识符（必须与平台一致）
const char* ATTR_TEMP   = "temperature";
const char* ATTR_HUMI   = "humidity";
const char* ATTR_SMOKE  = "smoke_alarm";
const char* ATTR_HR     = "heart_rate";
const char* ATTR_SPO2   = "spo2";

WiFiClient espClient;
PubSubClient client(espClient);

// ==================== 引脚定义 ====================
#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

const int analogPin = 36;
const int digitalPin = 5;
const int ledPin = 13;
const int buzzerPin = 14;      // 低电平触发蜂鸣器
const int ledRedPin = 15;      // 双色LED红
const int ledGreenPin = 16;    // 双色LED绿

// ==================== MAX30102 ====================
MAX30105 particleSensor;

const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute = 0;
int beatAvg = 0;

const int SPO2_SAMPLES = 30;
float irBuffer[SPO2_SAMPLES];
float redBuffer[SPO2_SAMPLES];
int sampleIndex = 0;
bool bufferFull = false;
float spo2 = 0;
bool spo2Valid = false;

// ==================== OLED ====================
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

bool lastFingerPresent = false;
float temperature = 0, humidity = 0;
char tempStr[6], humStr[6];

// ========== 血氧计算 ==========
float calculateSpO2(float* red, float* ir, int len) {
  if (len < 10) return 0;
  float redDC = 0, irDC = 0;
  for (int i = 0; i < len; i++) {
    redDC += red[i];
    irDC += ir[i];
  }
  redDC /= len;
  irDC /= len;
  float redAC = 0, irAC = 0;
  for (int i = 0; i < len; i++) {
    redAC += (red[i] - redDC) * (red[i] - redDC);
    irAC += (ir[i] - irDC) * (ir[i] - irDC);
  }
  redAC = sqrt(redAC / len);
  irAC = sqrt(irAC / len);
  if (redDC < 1e-6 || irDC < 1e-6 || irAC < 1e-6) return 0;
  float R = (redAC / redDC) / (irAC / irDC);
  float spo2_calc = 110 - 25 * R;
  if (spo2_calc > 100) spo2_calc = 100;
  if (spo2_calc < 70) spo2_calc = 0;
  return spo2_calc;
}

void resetHeartRate() {
  for (byte i = 0; i < RATE_SIZE; i++) rates[i] = 0;
  rateSpot = 0;
  lastBeat = 0;
  beatsPerMinute = 0;
  beatAvg = 0;
}

void setupWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected, IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("\nWiFi failed, running hardware only.");
  }
}

void reconnectMQTT() {
  if (WiFi.status() == WL_CONNECTED && !client.connected()) {
    Serial.print("MQTT connecting...");
    if (client.connect(clientId, username, mqttPassword)) {
      Serial.println("OK");
    } else {
      Serial.print("FAIL, rc=");
      Serial.println(client.state());
      delay(5000);
    }
  }
}

// ==================== 上报数据 ====================
void uploadData(float temp, float hum, bool smoke, int hr, float spo2_val, bool spo2_valid) {
  if (!client.connected()) {
    reconnectMQTT();
    if (!client.connected()) {
      Serial.println("Upload skipped (no MQTT)");
      return;
    }
  }

  String topic = "$sys/" + String(username) + "/" + String(clientId) + "/thing/property/post";
  
  String payload = "{";
  payload += "\"id\":\"" + String(millis()) + "\",";
  payload += "\"version\":\"1.0\",";
  payload += "\"params\":{";
  
  payload += "\"" + String(ATTR_TEMP) + "\":{\"value\":" + String(temp, 1) + "},";
  payload += "\"" + String(ATTR_HUMI) + "\":{\"value\":" + String(hum, 1) + "},";
  payload += "\"" + String(ATTR_SMOKE) + "\":{\"value\":" + String(smoke ? "true" : "false") + "},";
  payload += "\"" + String(ATTR_HR) + "\":{\"value\":" + String(hr) + "},";
  if (spo2_valid) {
    payload += "\"" + String(ATTR_SPO2) + "\":{\"value\":" + String(spo2_val, 0) + "}";
  } else {
    payload += "\"" + String(ATTR_SPO2) + "\":{\"value\":0}";
  }
  
  payload += "}}";

  Serial.print("Uploading: ");
  Serial.println(payload);
  
  if (client.publish(topic.c_str(), payload.c_str())) {
    Serial.println("Upload success");
  } else {
    Serial.println("Upload failed");
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);

  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);

  dht.begin();

  particleSensor.begin(Wire, I2C_SPEED_STANDARD);
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x1F);
  particleSensor.setPulseAmplitudeIR(0x1F);
  particleSensor.setPulseAmplitudeGreen(0x00);

  pinMode(digitalPin, INPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  pinMode(buzzerPin, OUTPUT);
  pinMode(ledRedPin, OUTPUT);
  pinMode(ledGreenPin, OUTPUT);
  digitalWrite(buzzerPin, HIGH);
  digitalWrite(ledRedPin, LOW);
  digitalWrite(ledGreenPin, HIGH);

  setupWiFi();
  client.setServer(mqttServer, mqttPort);
  reconnectMQTT();

  delay(1000);
  Serial.println("System Ready");
}

void loop() {
  // 维持连接
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  // 传感器采集
  long irValue = particleSensor.getIR();
  long redValue = particleSensor.getRed();
  bool fingerPresent = (irValue > 30000);

  if (lastFingerPresent && !fingerPresent) {
    resetHeartRate();
  }
  lastFingerPresent = fingerPresent;

  if (fingerPresent) {
    if (checkForBeat(irValue)) {
      long delta = millis() - lastBeat;
      lastBeat = millis();
      beatsPerMinute = 60000.0 / delta;
      if (beatsPerMinute > 30 && beatsPerMinute < 220) {
        rates[rateSpot++] = (byte)beatsPerMinute;
        rateSpot %= RATE_SIZE;
        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++) beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
    }
  } else {
    beatsPerMinute = 0;
  }

  if (fingerPresent && irValue > 50000 && redValue > 50000) {
    irBuffer[sampleIndex] = irValue;
    redBuffer[sampleIndex] = redValue;
    sampleIndex++;
    if (sampleIndex >= SPO2_SAMPLES) {
      sampleIndex = 0;
      bufferFull = true;
    }
    if (bufferFull) {
      float spo2_val = calculateSpO2(redBuffer, irBuffer, SPO2_SAMPLES);
      if (spo2_val > 0) {
        spo2 = spo2_val;
        spo2Valid = true;
      } else {
        spo2Valid = false;
      }
    }
  } else {
    sampleIndex = 0;
    bufferFull = false;
    spo2Valid = false;
    spo2 = 0;
  }

  // 烟雾报警（实时检测）
  int digitalVal = digitalRead(digitalPin);
  bool alarm = (digitalVal == LOW);
  static bool smokeDetected = false;
  if (alarm) {
    smokeDetected = true;  // 只要有过报警，就记住
  }

  // 硬件控制（仍用实时 alarm）
  if (alarm) {
    digitalWrite(buzzerPin, LOW);
    digitalWrite(ledRedPin, HIGH);
    digitalWrite(ledGreenPin, LOW);
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(buzzerPin, HIGH);
    digitalWrite(ledRedPin, LOW);
    digitalWrite(ledGreenPin, HIGH);
    digitalWrite(ledPin, LOW);
  }

  // 每秒更新显示（用实时 alarm 显示状态）
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate >= 1000) {
    lastUpdate = millis();

    float h = dht.readHumidity();
    float t = dht.readTemperature();
    if (isnan(h)) h = 0;
    if (isnan(t)) t = 0;
    temperature = t;
    humidity = h;
    dtostrf(temperature, 0, 1, tempStr);
    dtostrf(humidity, 0, 1, humStr);

    int analogVal = analogRead(analogPin);
    const char* smokeStatus = alarm ? "ALARM" : "SAFE";

    u8g2.firstPage();
    do {
      u8g2.setDrawColor(0);
      u8g2.drawBox(0, 0, 128, 12);
      u8g2.drawBox(0, 14, 128, 12);
      u8g2.drawBox(0, 28, 128, 12);
      u8g2.drawBox(0, 42, 128, 12);
      u8g2.setDrawColor(1);

      int x = 0;
      u8g2.drawStr(x, 12, "EnT:");
      x += u8g2.getStrWidth("EnT:");
      u8g2.drawStr(x, 12, tempStr);
      x += u8g2.getStrWidth(tempStr);
      u8g2.drawStr(x, 12, "\xb0");
      x += u8g2.getStrWidth("\xb0");
      u8g2.drawStr(x, 12, "C");
      x += u8g2.getStrWidth("C");
      u8g2.drawStr(x, 12, " ");
      x += u8g2.getStrWidth(" ");
      u8g2.drawStr(x, 12, "EnH:");
      x += u8g2.getStrWidth("EnH:");
      u8g2.drawStr(x, 12, humStr);
      x += u8g2.getStrWidth(humStr);
      u8g2.drawStr(x, 12, "%");

      char line2[30];
      sprintf(line2, "Smoke:%d  %s", analogVal, smokeStatus);
      u8g2.drawStr(0, 26, line2);

      char line3[30];
      if (spo2Valid && spo2 > 70) {
        sprintf(line3, "SpO2: %.0f%%", spo2);
      } else {
        sprintf(line3, "SpO2: ---%%");
      }
      u8g2.drawStr(0, 40, line3);

      char line4[30];
      if (!fingerPresent) {
        sprintf(line4, "Heart Rate: -- bpm");
      } else if (beatAvg == 0) {
        sprintf(line4, "Heart Rate: 00 bpm");
      } else {
        sprintf(line4, "Heart Rate: %d bpm", beatAvg);
      }
      u8g2.drawStr(0, 54, line4);
    } while (u8g2.nextPage());

    Serial.printf("T:%.1f H:%.1f Smoke:%d %s SpO2:%.0f HR_Avg:%d\n",
                  temperature, humidity, analogVal, smokeStatus, spo2, beatAvg);
  }

  // ★ 每10秒上报，使用记忆标志 ★
  static unsigned long lastUploadTime = 0;
  if (millis() - lastUploadTime >= 10000) {
    lastUploadTime = millis();
    uploadData(temperature, humidity, smokeDetected, beatAvg, spo2, spo2Valid);
    smokeDetected = false;  // 上报后清除记忆
  }

  delay(20);
}
