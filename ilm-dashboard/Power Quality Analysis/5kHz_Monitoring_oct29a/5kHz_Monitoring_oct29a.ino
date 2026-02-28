#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <math.h>

// ================== USER SETTINGS ==================
#define WIFI_SSID "Akalanka"
#define WIFI_PASSWORD "12345678"

// Your PC IP running Node server
#define SERVER_BASE "http://192.168.135.253:5000"

// Set this plug id (CHANGE for each plug)
#define DEVICE_ID "plug_a"

// Relay control pin (your control pin)
#define RELAY_PIN 8
// Relay logic: many relay modules are ACTIVE LOW (0=ON). If yours is opposite, change these.
#define RELAY_ON  LOW
#define RELAY_OFF HIGH
// ====================================================

// Sensor pins
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

// Buffers
float currentSamples[SAMPLES_PER_WINDOW];
unsigned long timestamps[SAMPLES_PER_WINDOW];

int sampleIndex = 0;
unsigned long lastSampleTime = 0;

bool impulseDetected = false;
bool impulseHigh = false;
unsigned long impulseMidTimestamp = 0;

// Control polling
unsigned long lastControlPollMs = 0;
const unsigned long CONTROL_POLL_INTERVAL_MS = 1500; // 1.5s

// -------------------- Helpers --------------------
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

// GET /api/state/plug_a  -> {"device_id":"plug_a","is_on":1,...}
void pollControlState() {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  http.begin(urlGetState());

  int code = http.GET();
  if (code == 200) {
    String payload = http.getString();

    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, payload);
    if (!err) {
      int is_on = doc["is_on"] | 1;
      setRelay(is_on == 1);
      Serial.print("🔁 Control state is_on = ");
      Serial.println(is_on);
    } else {
      Serial.println("❌ JSON parse error (state)");
    }
  } else {
    Serial.print("❌ State GET failed, code=");
    Serial.println(code);
  }
  http.end();
}

// POST /api/ingest/plug
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
  String resp = http.getString();

  Serial.print("📤 POST ingest/plug code=");
  Serial.println(code);
  // Serial.println(resp); // optional

  http.end();
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(19200);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pinMode(RELAY_PIN, OUTPUT);
  setRelay(true); // default ON (or change if you want default OFF)

  connectWiFi();
}

// -------------------- Loop --------------------
void loop() {
  // 1) Poll control state periodically
  if (millis() - lastControlPollMs >= CONTROL_POLL_INTERVAL_MS) {
    lastControlPollMs = millis();
    pollControlState();
  }

  // 2) Detect impulse (your original logic)
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

  // 3) Sample current
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

    // 4) Process window
    if (sampleIndex >= SAMPLES_PER_WINDOW) {
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

      // Your voltage is master device, but for local calculation you used 230
      float powerW = 230.0 * currentRMS * pf;

      Serial.print("Irms=");
      Serial.print(currentRMS, 3);
      Serial.print("A  PF=");
      Serial.print(pf, 3);
      Serial.print("  P=");
      Serial.print(powerW, 1);
      Serial.println("W");

      // 5) Send to Node backend
      sendPlugMetrics(currentRMS, powerW, pf);

      // reset for next window
      sampleIndex = 0;
      impulseDetected = false;
    }
  }
}
