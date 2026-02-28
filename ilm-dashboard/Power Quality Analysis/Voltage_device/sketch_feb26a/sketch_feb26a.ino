#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

/* ------------------ Pins ------------------ */
#define ADC_PIN 34

/* ------------------ WiFi + Backend ------------------ */
#define WIFI_SSID     "Akalanka"
#define WIFI_PASSWORD "12345678"

// Local backend (your laptop IP) OR Render URL later
// Example local: "http://192.168.75.253:5000"
#define SERVER_BASE "http://192.168.135.253:5000"

/* ------------------ Sampling ------------------ */
#define FS       1000               // Hz
#define F_EXPECT 50                 // nominal
#define CYCLES_FOR_MEAN  10
#define SAMPLES_PER_CYCLE (FS / F_EXPECT)            // ~20
#define TOTAL_SAMPLES     (SAMPLES_PER_CYCLE * CYCLES_FOR_MEAN) // ~200

/* ------------------ ADC/Scaling ------------------ */
#define ADC_MAX          4095.0
#define VREF             3.3

#define VOLTAGE_DIV      0.66
#define BIAS_VOLTAGE     1.65

#define ZMPT_SENSITIVITY 0.00388
#define SLOPE_COM        1.0

/* ------------------ LCD ------------------ */
LiquidCrystal_I2C lcd(0x27, 16, 2);

/* ------------------ Buffers ------------------ */
float x[TOTAL_SAMPLES];

/* Convert ADC reading to mains volts (instantaneous) */
static inline float adcToMainsVolts(int adcValue) {
  float v_adc    = (adcValue / ADC_MAX) * VREF;
  float v_sensor = v_adc / VOLTAGE_DIV;
  float v_center = v_sensor - BIAS_VOLTAGE;
  float v_mains  = (v_center / ZMPT_SENSITIVITY) * SLOPE_COM;
  return v_mains;
}

/* Acquire block of samples at FS */
void sampleBlock() {
  const uint32_t Ts_us = 1000000UL / FS;
  uint32_t t = micros();

  for (int i = 0; i < TOTAL_SAMPLES; i++) {
    int a = analogRead(ADC_PIN);
    x[i] = adcToMainsVolts(a);

    while ((uint32_t)(micros() - t) < Ts_us) {}
    t += Ts_us;
  }
}

float computeMean(const float *arr, int n) {
  double s = 0.0;
  for (int i = 0; i < n; i++) s += arr[i];
  return (float)(s / n);
}

float computeRMS_centered(float mean) {
  double ss = 0.0;
  for (int i = 0; i < TOTAL_SAMPLES; i++) {
    float v = x[i] - mean;
    ss += (double)v * (double)v;
  }
  return (float)sqrt(ss / (double)TOTAL_SAMPLES);
}

float estimateFreqZeroCross(float mean) {
  double tPrev = -1.0;
  double periodSum = 0.0;
  int periodCount = 0;

  for (int i = 1; i < TOTAL_SAMPLES; i++) {
    float y0 = (x[i - 1] - mean);
    float y1 = (x[i] - mean);

    if (y0 < 0.0f && y1 >= 0.0f) {
      float denom = (y1 - y0);
      if (fabs(denom) < 1e-12f) continue;

      float frac = (-y0) / denom;
      double tCross = ((double)(i - 1) + (double)frac) / (double)FS;

      if (tPrev >= 0.0) {
        double T = tCross - tPrev;
        if (T > (1.0 / 70.0) && T < (1.0 / 40.0)) {
          periodSum += T;
          periodCount++;
        }
      }
      tPrev = tCross;
    }
  }

  if (periodCount < 2) return 0.0f;
  double Tavg = periodSum / (double)periodCount;
  return (float)(1.0 / Tavg);
}

const char* classifyDisturbance(float Vrms) {
  const float Vnom = 230.0;
  if (0.90f * Vnom <= Vrms && Vrms <= 1.10f * Vnom) return "VOLT NORMAL";
  else if (Vrms <= 0.10f * Vnom) return "INTERRUPT";
  else if (Vrms < 0.90f * Vnom) return "VOLT SAG";
  else return "VOLT SWELL"; // ✅ fixed
}

void updateLCD(float Vrms, float freq, const char* status) {
  lcd.setCursor(0, 0);
  char line1[17];
  snprintf(line1, sizeof(line1), "%6.1fV %5.2fHz", Vrms, freq);
  lcd.print(line1);
  for (int i = strlen(line1); i < 16; i++) lcd.print(' ');

  lcd.setCursor(0, 1);
  lcd.print(status);
  for (int i = strlen(status); i < 16; i++) lcd.print(' ');
}

void connectWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\n✅ WiFi Connected");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

void sendMasterToBackend(float Vrms, float freq) {
  if (WiFi.status() != WL_CONNECTED) return;

  String url = String(SERVER_BASE) + "/api/ingest/master";

  StaticJsonDocument<128> doc;
  doc["voltage_rms"] = Vrms;
  doc["frequency"]   = freq;

  String body;
  serializeJson(doc, body);

  HTTPClient http;
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(body);
  http.end();

  Serial.print("POST /api/ingest/master -> ");
  Serial.println(code);
}

void setup() {
  Serial.begin(115200);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("PQ Monitor");
  delay(700);
  lcd.clear();

  connectWiFi();
}

void loop() {
  sampleBlock();

  float mean = computeMean(x, TOTAL_SAMPLES);
  float Vrms = computeRMS_centered(mean);
  float freq = estimateFreqZeroCross(mean);

  const char* status = classifyDisturbance(Vrms);

  updateLCD(Vrms, freq, status);

  Serial.print("Vrms=");
  Serial.print(Vrms, 2);
  Serial.print(" V, f=");
  Serial.print(freq, 2);
  Serial.print(" Hz, ");
  Serial.println(status);

  // ✅ Send to backend (master values)
  sendMasterToBackend(Vrms, freq);

  delay(500); // sending twice per second is enough
}
