#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <math.h>

// ================== SETTINGS ==================
#define WIFI_SSID "Akalanka"
#define WIFI_PASSWORD "12345678"

// Your Node backend (local)
#define SERVER_BASE "http://192.168.135.253:5000"

// ✅ Plug B ID
#define DEVICE_ID "plug_b"

// Relay control pin
#define RELAY_PIN 8

// If relay action is reversed, swap these:
#define RELAY_ON  HIGH
#define RELAY_OFF LOW
// =============================================

// Sensor pins (change if your wiring differs)
#define SENSOR_PIN 2
#define IMPULSE_PIN 3

#define VREF 3.3
#define SENSOR_SENSITIVITY 0.066
#define BIAS_VOLTAGE 2.5
#define SAMPLING_INTERVAL_US 400
#define SAMPLES_PER_WINDOW 2500

#define INTERCEPT -0.0354
#define SLOPE1 0.9572
#define SLOPE2 -0.016

#define IMPULSE_THRESHOLD 3.0
const float ADC_RESOLUTION = 4095.0;

float currentSamples[SAMPLES_PER_WINDOW];
unsigned long timestamps[SAMPLES_PER_WINDOW];

int sampleIndex = 0;
unsigned long lastSampleTime = 0;

bool impulseDetected = false;
bool impulseHigh = false;
unsigned long impulseMidTimestamp = 0;

// Poll control state
unsigned long lastControlPollMs = 0;
const unsigned long CONTROL_POLL_INTERVAL_MS = 1500;

String urlIngestPlug() {
  return String(SERVER_BASE) + "/api/ingest/plug";
}

String urlGetState() {
  return String(SERVER_BASE) + "/api/state/" + String(DEVICE_ID);
}

void setRelay(bool on) {
  digitalWrite(RELAY_PIN, on ? RELAY_ON : RELAY_OFF);
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

// GET state from backend
void pollControlState() {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  http.begin(urlGetState());

  int code = http.GET();
  if (code == 200) {
    String payload = http.getString();
    StaticJsonDocument<256> doc;

    if (deserializeJson(doc, payload) == DeserializationError::Ok) {
      int is_on = doc["is_on"] | 1;
      setRelay(is_on == 1);
      Serial.print("🔁 PlugB state is_on=");
      Serial.println(is_on);
    } else {
      Serial.println("❌ JSON parse error (state)");
    }
  } else {
    Serial.print("❌ State GET failed code=");
    Serial.println(code);
  }
  http.end();
}

// POST plug metrics to backend
void sendPlugMetrics(float currentRMS, float powerW, float pf) {
  if (WiFi.status() != WL_CONNECTED) return;

  StaticJsonDocument<256> doc;
  doc["device_id"] = DEVICE_ID;
  doc["current_rms"] = currentRMS;
  doc["active_power"] = powerW;
  doc["power_factor"] = pf;

  String body;
  serializeJson(doc, body);

  HTTPClient http;
  http.begin(urlIngestPlug());
  http.addHeader("Content-Type", "application/json");

  int code = http.POST(body);
  http.end();

  Serial.print("📤 PlugB POST ingest code=");
  Serial.println(code);
}

void setup() {
  Serial.begin(19200);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pinMode(RELAY_PIN, OUTPUT);
  setRelay(true); // default ON

  connectWiFi();
}

void loop() {
  // Control polling
  if (millis() - lastControlPollMs >= CONTROL_POLL_INTERVAL_MS) {
    lastControlPollMs = millis();
    pollControlState();
  }

  // Detect impulse
  if (!impulseDetected) {
    int impulseADC = analogRead(IMPULSE_PIN);
    float impulseVoltage = (impulseADC / ADC_RESOLUTION) * VREF;
    unsigned long now = micros();

    static unsigned long riseTime = 0;
    if (!impulseHigh && impulseVoltage > IMPULSE_THRESHOLD) {
      riseTime = now;
      impulseHigh = true;
    }

    if (impulseHigh && impulseVoltage < IMPULSE_THRESHOLD) {
      unsigned long fallTime = now;
      unsigned long pulseWidth = fallTime - riseTime;
      impulseMidTimestamp = riseTime + (pulseWidth / 2);
      impulseDetected = true;
      impulseHigh = false;
    }
    return;
  }

  // Sample current
  if (micros() - lastSampleTime >= SAMPLING_INTERVAL_US) {
    lastSampleTime = micros();

    int adcValue = analogRead(SENSOR_PIN);
    float voltage = (adcValue / ADC_RESOLUTION) * VREF / 0.954;
    float current = (voltage - BIAS_VOLTAGE) / SENSOR_SENSITIVITY;

    if (sampleIndex < SAMPLES_PER_WINDOW) {
      currentSamples[sampleIndex] = current;
      timestamps[sampleIndex] = lastSampleTime;
      sampleIndex++;
    }

    if (sampleIndex >= SAMPLES_PER_WINDOW) {
      // Mean remove + RMS
      float sum = 0.0;
      for (int i = 0; i < SAMPLES_PER_WINDOW; i++) sum += currentSamples[i];
      float mean = sum / SAMPLES_PER_WINDOW;

      float sumSquares = 0.0;
      for (int i = 0; i < SAMPLES_PER_WINDOW; i++) {
        float centered = currentSamples[i] - mean;
        sumSquares += centered * centered;
        currentSamples[i] = centered;
      }

      float currentRMS = sqrt(sumSquares / SAMPLES_PER_WINDOW);
      currentRMS = (SLOPE2 * currentRMS * currentRMS) + (SLOPE1 * currentRMS) + INTERCEPT;

      // Zero-crossing timing for PF
      unsigned long zeroCrossTime = 0;
      for (int i = 1; i < SAMPLES_PER_WINDOW; i++) {
        if (currentSamples[i - 1] < 0 && currentSamples[i] >= 0) {
          float slope = currentSamples[i] - currentSamples[i - 1];
          float fraction = -currentSamples[i - 1] / slope;
          zeroCrossTime = timestamps[i - 1] + (unsigned long)(fraction * (timestamps[i] - timestamps[i - 1]));
          break;
        }
      }

      float time_diff_us = (float)(zeroCrossTime - impulseMidTimestamp);
      float time_diff_s = fmod(time_diff_us, 5000.0) / 1e6;
      float phase_diff_deg = time_diff_s * 18000.0;
      float phase_diff_rad = phase_diff_deg * (M_PI / 180.0);

      float pf = cos(phase_diff_rad);
      float powerW = 230.0 * currentRMS * pf; // voltage is from master; here for local calculation

      Serial.print("PlugB Irms=");
      Serial.print(currentRMS, 3);
      Serial.print(" PF=");
      Serial.print(pf, 3);
      Serial.print(" P=");
      Serial.print(powerW, 1);
      Serial.println("W");

      sendPlugMetrics(currentRMS, powerW, pf);

      sampleIndex = 0;
      impulseDetected = false;
    }
  }
}
