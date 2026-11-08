// ESP32 thường: điều khiển quạt 5V qua L298N bằng fuzzy logic (DHT11) + OLED + gửi dữ liệu lên web Python qua WiFi
// Board package: chạy được với ESP32 core 1.0.x, 2.x và 3.x (tự chọn cách gọi PWM).
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>

// ============================= WiFi (cùng mạng với ESP32-CAM và máy tính) =============================
const char* ssid = "NLBVN";
const char* password = "66666666";

// ============================= DHT11 =============================
#define DHT_PIN 4
#define DHT_TYPE DHT11
DHT dht(DHT_PIN, DHT_TYPE);

// ============================= OLED 0.96 inch =============================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SDA_PIN 21
#define SCL_PIN 22
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ============================= L298N =============================
#define ENA 25
#define IN1 26
#define IN2 27
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8
#define PWM_CH 0                 // chỉ dùng cho core 1.x / 2.x

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_WRITE(v) ledcWrite(ENA, (v))
#else
  #define PWM_WRITE(v) ledcWrite(PWM_CH, (v))
#endif

#define READ_INTERVAL_MS 2000    // DHT11 nên đọc cách nhau khoảng 2 giây

WebServer server(80);

float g_temp = NAN, g_hum = NAN;
bool  g_dhtOk = false;
float g_fanPercent = 0;
int   g_pwm = 0;
bool  g_manual = false;          // false = tự động theo fuzzy, true = web đặt tay
float g_manualPercent = 0;
unsigned long g_lastRead = 0;
unsigned long g_lastWifiTry = 0;

// ============================= Hàm membership nhiệt độ =============================
float tempLow(float T) {
  if (T <= 20) return 1.0;
  if (T >= 28) return 0.0;
  return (28.0 - T) / 8.0;
}
float tempMedium(float T) {
  if (T <= 24 || T >= 34) return 0.0;
  if (T < 29) return (T - 24.0) / 5.0;
  return (34.0 - T) / 5.0;
}
float tempHigh(float T) {
  if (T <= 30) return 0.0;
  if (T >= 38) return 1.0;
  return (T - 30.0) / 8.0;
}

// ============================= Hàm membership độ ẩm =============================
float humLow(float H) {
  if (H <= 40) return 1.0;
  if (H >= 60) return 0.0;
  return (60.0 - H) / 20.0;
}
float humMedium(float H) {
  if (H <= 45 || H >= 75) return 0.0;
  if (H < 60) return (H - 45.0) / 15.0;
  return (75.0 - H) / 15.0;
}
float humHigh(float H) {
  if (H <= 65) return 0.0;
  if (H >= 90) return 1.0;
  return (H - 65.0) / 25.0;
}

// ============================= FUZZY CONTROL (giữ nguyên 9 luật) =============================
float fuzzyControl(float T, float H) {
  float TL = tempLow(T), TM = tempMedium(T), TH = tempHigh(T);
  float HL = humLow(H),  HM = humMedium(H),  HH = humHigh(H);

  float r1 = min(TL, HL), r2 = min(TL, HM), r3 = min(TL, HH);
  float r4 = min(TM, HL), r5 = min(TM, HM), r6 = min(TM, HH);
  float r7 = min(TH, HL), r8 = min(TH, HM), r9 = min(TH, HH);

  float output = r1 * 25 + r2 * 30 + r3 * 50 +
                 r4 * 50 + r5 * 60 + r6 * 75 +
                 r7 * 80 + r8 * 90 + r9 * 100;
  float total = r1 + r2 + r3 + r4 + r5 + r6 + r7 + r8 + r9;

  if (total == 0) return 0;
  return output / total;
}

// ============================= Tính % quạt và xuất PWM =============================
void updateFan() {
  float percent;
  if (g_manual)        percent = g_manualPercent;                  // web đặt tay
  else if (!g_dhtOk)   percent = 0;                                // lỗi cảm biến: tắt quạt (như code gốc)
  else                 percent = fuzzyControl(g_temp, g_hum);
  g_fanPercent = constrain(percent, 0, 100);
  g_pwm = (int)(g_fanPercent * 255.0 / 100.0);
  PWM_WRITE(g_pwm);
}

// ============================= OLED =============================
void drawDisplay() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  if (!g_dhtOk) {
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("DHT11");
    display.println("ERROR");
    display.setTextSize(1);
    display.setCursor(0, 45);
    display.println(g_manual ? "Che do TAY" : "Check sensor!");
  } else {
    display.setCursor(0, 0);
    display.print("TEMP: "); display.print(g_temp, 1); display.println(" C");
    display.setCursor(0, 16);
    display.print("HUM : "); display.print(g_hum, 1); display.println(" %");
    display.setCursor(0, 32);
    display.print("FAN : "); display.print(g_fanPercent, 0); display.print(" %");
    display.print(g_manual ? " TAY" : " AUTO");
    display.setCursor(0, 48);
    display.print("PWM : "); display.print(g_pwm);
  }
  display.display();
}

// ============================= Web API (cho Python đọc / điều khiển) =============================
void numOrNull(char *out, size_t n, float v, bool ok) {
  if (ok && !isnan(v)) snprintf(out, n, "%.1f", v);
  else snprintf(out, n, "null");
}

void handleStatus() {
  char t[12], h[12], buf[320];
  numOrNull(t, sizeof(t), g_temp, g_dhtOk);
  numOrNull(h, sizeof(h), g_hum, g_dhtOk);
  snprintf(buf, sizeof(buf),
           "{\"temperature\":%s,\"humidity\":%s,\"dht_ok\":%s,\"fan_percent\":%.1f,\"pwm\":%d,"
           "\"mode\":\"%s\",\"uptime_s\":%lu,\"rssi\":%d}",
           t, h, g_dhtOk ? "true" : "false", g_fanPercent, g_pwm,
           g_manual ? "manual" : "auto", millis() / 1000UL, WiFi.RSSI());
  server.send(200, "application/json", buf);
}

// GET /api/fan?mode=auto            -> quay về fuzzy tự động
// GET /api/fan?mode=manual&percent=60 -> đặt tay 60 %
void handleFan() {
  String mode = server.arg("mode");
  if (mode == "manual") {
    g_manual = true;
    g_manualPercent = constrain(server.arg("percent").toFloat(), 0, 100);
  } else if (mode == "auto") {
    g_manual = false;
  } else {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"mode phai la auto hoac manual\"}");
    return;
  }
  updateFan();
  drawDisplay();
  server.send(200, "application/json", "{\"ok\":true}");
}

// ============================= SETUP =============================
void setup() {
  Serial.begin(115200);
  dht.begin();
  Wire.begin(SDA_PIN, SCL_PIN);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("Khong tim thay OLED!");
    while (1);
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // L298N: quay 1 chiều
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(ENA, PWM_FREQ, PWM_RESOLUTION);
#else
  ledcSetup(PWM_CH, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ENA, PWM_CH);
#endif
  PWM_WRITE(0);

  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println("FUZZY");
  display.setTextSize(1);
  display.setCursor(0, 25);
  display.println("ESP32 CONTROL");
  display.setCursor(0, 40);
  display.println("Connecting WiFi...");
  display.display();

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {   // chờ tối đa 15 giây, không có WiFi quạt vẫn chạy
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("IP quat (dien vao FAN_URL trong app.py): http://");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Chua co WiFi - quat van chay tu dong");
  }

  server.on("/api/status", handleStatus);
  server.on("/api/fan", handleFan);
  server.begin();

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("IP:");
  display.println(WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : String("khong co WiFi"));
  display.display();
  delay(2000);
}

// ============================= LOOP (không dùng delay để web luôn phản hồi) =============================
void loop() {
  server.handleClient();

  // thử nối lại WiFi mỗi 10 giây nếu bị mất
  if (WiFi.status() != WL_CONNECTED && millis() - g_lastWifiTry > 10000) {
    g_lastWifiTry = millis();
    WiFi.disconnect();
    WiFi.begin(ssid, password);
  }

  if (g_lastRead != 0 && millis() - g_lastRead < READ_INTERVAL_MS) return;
  g_lastRead = millis();
  if (g_lastRead == 0) g_lastRead = 1;

  float t = dht.readTemperature();
  float h = dht.readHumidity();
  g_dhtOk = !(isnan(t) || isnan(h));
  if (g_dhtOk) { g_temp = t; g_hum = h; }
  else Serial.println("DHT11 ERROR");

  updateFan();

  Serial.println("======================");
  Serial.printf("Temperature: %.1f C\nHumidity: %.1f %%\nFan: %.1f %% (%s)\nPWM: %d\n",
                g_dhtOk ? g_temp : NAN, g_dhtOk ? g_hum : NAN, g_fanPercent,
                g_manual ? "TAY" : "AUTO", g_pwm);

  drawDisplay();
}
