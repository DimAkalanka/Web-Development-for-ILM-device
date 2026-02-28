#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

#define SENSOR_PIN 34
#define IMPULSE_PIN 35

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

// Wi-Fi and backend URL
#define WIFI_SSID "Akalanka"
#define WIFI_PASSWORD "12345678"
#define SERVER_URL "http://192.168.75.253:5000/esp_data" // 

void setup() {
  Serial.begin(19200);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\n✅ WiFi Connected.");
}

void loop() {
  // --- Detect Impulse ---
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

  // --- Sample Current ---
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

    // --- Process Window ---
    if (sampleIndex >= SAMPLES_PER_WINDOW) {
      // Compute mean and center data
      float sum = 0.0;
      for (int i = 0; i < SAMPLES_PER_WINDOW; i++) sum += currentSamples[i];
      float mean = sum / SAMPLES_PER_WINDOW;

      float sumSquares = 0.0;
      for (int i = 0; i < SAMPLES_PER_WINDOW; i++) {
        float centered = currentSamples[i] - mean;
        sumSquares += centered * centered;
        currentSamples[i] = centered;
      }

      // RMS current with correction
      float currentRMS = sqrt(sumSquares / SAMPLES_PER_WINDOW);
      currentRMS = (SLOPE2 * currentRMS * currentRMS) + (SLOPE1 * currentRMS) + INTERCEPT;

      // Zero-crossing detection for phase
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
      float phase_diff_rad = phase_diff_deg * (3.14159265358979323846 / 180.0);

      float pf = cos(phase_diff_rad);
      float power = 230.0 * currentRMS * pf;

      Serial.print("RMS Current: ");
      Serial.print(currentRMS, 3);
      Serial.print(" A | Power: ");
      Serial.print(power, 2);
      Serial.print(" W | PF: ");
      Serial.println(pf, 3);

      // --- Send to Backend ---
      if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        http.begin(SERVER_URL);
        http.addHeader("Content-Type", "application/json");

        DynamicJsonDocument doc(200);
        doc["current"] = currentRMS;
        doc["power"] = power;
        doc["power_factor"] = pf;

        String requestBody;
        serializeJson(doc, requestBody);

        int responseCode = http.POST(requestBody);
        String response = http.getString();
        Serial.print("HTTP Response Code: ");
        Serial.println(responseCode);
        Serial.println("Response: " + response);

        http.end();
      }

      sampleIndex = 0;
      impulseDetected = false;
    }
  }
}
