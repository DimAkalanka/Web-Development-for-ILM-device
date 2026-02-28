 #include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h>

#define SENSOR_PIN 2           // ADC input pin
#define IMPULSE_PIN 3          // Impulse signal pin
#define CONTROL_PIN 8          // Relay control pin

#define VREF 3.3
#define SENSOR_SENSITIVITY 0.066  // ACS712: 66mV per A
#define BIAS_VOLTAGE 2.5

#define VOLTAGE_RMS 235     // Assume 235 V
#define VOLTAGE_PEAK 332.3402 // 235 x root(2)

#define SAMPLING_INTERVAL_US 400  // 2.5kHz
#define SAMPLES_PER_WINDOW 250   // 250 x 400 US = 100 millisec; (5 ensembles)

#define IMPULSE_THRESHOLD 3.0

const float ADC_RESOLUTION = 4095.0;

// Calibrated constants
const float filter_com = 0.954;
const float VoltageDiv_com = 0.66; 
const float slope_com = 0.653;

// Current threshold
const float Noice_TH = 0.1; // Neglect below 24 W

// Wi-Fi credentials
#define WIFI_SSID "Akalanka"
#define WIFI_PASSWORD "12345678"

// Backend URLs
#define SERVER_URL "http://192.168.153.253:5000/esp_data"
#define GET_STATE_URL "http://192.168.153.253:5000/get_state"

// NTP Server
#define NTP_SERVER "pool.ntp.org"
#define GMT_OFFSET_SEC 19800  // +5:30 for Sri Lanka (5.5 * 3600)
#define DAYLIGHT_OFFSET_SEC 0

// Define structure for sensor data
typedef struct {
    float currentRMS;
    float power;
    float VIrms;
    float DPF;
    unsigned long timestamp_ms;  // Local millis timestamp
} SensorData_t;

// Queue handle
QueueHandle_t sensorDataQueue;

// Task priorities
#define DATA_TASK_PRIORITY    2
#define WIFI_TASK_PRIORITY    1

// Stack sizes
#define DATA_TASK_STACK       4096
#define WIFI_TASK_STACK       8192

// Task handles
TaskHandle_t dataTaskHandle = NULL;
TaskHandle_t wifiTaskHandle = NULL;

// Shared variables for data acquisition task
float currentSamples[SAMPLES_PER_WINDOW];
unsigned long timestamps[SAMPLES_PER_WINDOW];
int sampleIndex = 0;
unsigned long lastSampleTime = 0;

bool impulseDetected = false;
bool impulseHigh = false;
unsigned long impulseMidTimestamp = 0;

// ==================== DATA ACQUISITION TASK ====================
void dataAcquisitionTask(void *parameter) {
    Serial.println("Data Acquisition Task Started");
    
    while (1) {
        // Wait for impulse detection (voltage zero crossing)
        if (!impulseDetected) {
            int impulseADC = analogRead(IMPULSE_PIN);
            float impulseVoltage = (impulseADC / ADC_RESOLUTION) * VREF;
            unsigned long now = micros();

            static unsigned long riseTime = 0;
            if (!impulseHigh && impulseVoltage > IMPULSE_THRESHOLD) {
                riseTime = now;
                impulseHigh = true;
            }
            
            // Save the timestamp
            if (impulseHigh && impulseVoltage < IMPULSE_THRESHOLD) {
                unsigned long fallTime = now;
                unsigned long pulseWidth = fallTime - riseTime;
                impulseMidTimestamp = riseTime + (pulseWidth / 2);
                impulseDetected = true;
                impulseHigh = false;
            }
            
            vTaskDelay(1); // Small delay to prevent watchdog issues
            continue;
        }

        // Start current sampling
        if (micros() - lastSampleTime >= SAMPLING_INTERVAL_US) {
            lastSampleTime = micros();

            int adcValue = analogRead(SENSOR_PIN);
            float voltage = (adcValue / ADC_RESOLUTION) * VREF / (filter_com * VoltageDiv_com * slope_com);
            float current = (voltage - BIAS_VOLTAGE) / SENSOR_SENSITIVITY;

            // Save until the sample size reached
            if (sampleIndex < SAMPLES_PER_WINDOW) {
                currentSamples[sampleIndex] = current;
                timestamps[sampleIndex] = lastSampleTime;
                sampleIndex++;
            }

            // Process Window when full
            if (sampleIndex >= SAMPLES_PER_WINDOW) {
                float sumSquares = 0.0, total = 0.0, sumVI = 0.0;
                
                // Calculate mean
                for (int i = 0; i < SAMPLES_PER_WINDOW; i++) {
                    total += currentSamples[i];
                }
                float mean = total / SAMPLES_PER_WINDOW;
                
                // RMS calculation and instantaneous power calculation
                for (int i = 0; i < SAMPLES_PER_WINDOW; i++) {
                    float centered = currentSamples[i] - mean;
                    sumSquares += centered * centered;
                    currentSamples[i] = centered;

                    // For instantaneous method: phase of the voltage point from crossing point
                    float time_diff_us = (float)(timestamps[i] - impulseMidTimestamp);
                    float time_diff_s = fmod(time_diff_us, 20000.0) / 1e6;
                    float phase_diff_deg = (time_diff_s * 18000.0) - 10;
                    float phase_diff_rad = phase_diff_deg * (M_PI / 180.0);

                    sumVI += centered * VOLTAGE_PEAK * sin(phase_diff_rad); 
                }

                float currentRMS = sqrt(sumSquares / SAMPLES_PER_WINDOW);
                float P_avg = fabs(sumVI) / SAMPLES_PER_WINDOW;

                float DPF = 0;
                // Error handling
                if (currentRMS == 0) {
                    DPF = 0;
                } else {
                    DPF = P_avg / (VOLTAGE_RMS * currentRMS);
                }

                float power = P_avg;
                float RMS_VI = (VOLTAGE_RMS * currentRMS);

                // Take the current threshold as noise
                if (Noice_TH > currentRMS) {
                    currentRMS = 0;
                    DPF = 0;
                    power = 0;
                    RMS_VI = 0;
                }
                
                // Print CSV format: CurrentRMS,Power,VIrms,DPF
                Serial.print(currentRMS, 3);
                Serial.print(",");
                Serial.print(power, 2);
                Serial.print(",");
                Serial.print(RMS_VI, 2);
                Serial.print(",");
                Serial.println(DPF, 3);

                // Prepare data for queue
                SensorData_t sensorData;
                sensorData.currentRMS = currentRMS;
                sensorData.power = power;
                sensorData.VIrms = RMS_VI;
                sensorData.DPF = DPF;
                sensorData.timestamp_ms = millis();

                // Send to queue (non-blocking with timeout)
                // Check queue level before sending
                UBaseType_t queueLevel = uxQueueMessagesWaiting(sensorDataQueue);
                
                if (queueLevel >= 80) { // If queue >80% full, skip this reading
                    Serial.printf("Queue critical (%d/100) - skipping reading to prevent overflow\n", queueLevel);
                } else if (xQueueSend(sensorDataQueue, &sensorData, pdMS_TO_TICKS(10)) != pdPASS) {
                    Serial.println("Queue full - data dropped!");
                } else {
                    // Monitor queue usage
                    if (queueLevel > 60) { // Warning if >60% full
                        Serial.printf("WARNING: Queue filling up! (%d/100)\n", queueLevel);
                    }
                }

                // Reset for next window
                sampleIndex = 0;
                impulseDetected = false;
            }
        }
        
        // Yield to allow other tasks to run
        taskYIELD();
    }
}

// ==================== WIFI COMMUNICATION TASK ====================
void wifiCommunicationTask(void *parameter) {
    Serial.println("WiFi Communication Task Started");
    
    SensorData_t receivedData;
    unsigned long lastStateCheck = 0;
    const unsigned long STATE_CHECK_INTERVAL = 2000; // 2 seconds
    
    // Bundling variables - accumulate 10 readings for 1 Hz transmission
    const int BUNDLE_SIZE = 10;
    float sum_currentRMS = 0;
    float sum_power = 0;
    float sum_VIrms = 0;
    float sum_DPF = 0;
    int bundle_count = 0;
    unsigned long bundle_start_time = 0;
    
    // Wait for WiFi to be connected (done in setup)
    while (WiFi.status() != WL_CONNECTED) {
        Serial.println("Waiting for WiFi...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    Serial.println("WiFi Connected in task");
    
    while (1) {
        // Check WiFi status and reconnect if needed
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi disconnected! Reconnecting...");
            WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
            
            int attempts = 0;
            while (WiFi.status() != WL_CONNECTED && attempts < 20) {
                vTaskDelay(pdMS_TO_TICKS(500));
                Serial.print(".");
                attempts++;
            }
            
            if (WiFi.status() == WL_CONNECTED) {
                Serial.println("\nWiFi reconnected!");
            } else {
                Serial.println("\nWiFi reconnection failed!");
                vTaskDelay(pdMS_TO_TICKS(5000));
                continue;
            }
        }
        
        // Check if data is available in queue
        // AGGRESSIVE DRAINING: Process multiple items if queue is backing up
        int items_to_process = 1;
        UBaseType_t currentQueueLevel = uxQueueMessagesWaiting(sensorDataQueue);
        
        if (currentQueueLevel > 60) {
            items_to_process = 5; // Process 5 at once if queue getting full
            Serial.printf("Queue high (%d/100) - draining faster\n", currentQueueLevel);
        }
        
        for (int i = 0; i < items_to_process; i++) {
            if (xQueueReceive(sensorDataQueue, &receivedData, pdMS_TO_TICKS(10)) != pdPASS) {
                break; // No more data available
            }
            // Accumulate values for bundling
            sum_currentRMS += receivedData.currentRMS;
            sum_power += receivedData.power;
            sum_VIrms += receivedData.VIrms;
            sum_DPF += receivedData.DPF;
            bundle_count++;
            
            if (bundle_count == 1) {
                bundle_start_time = millis(); // Start timing
            }
            
            Serial.printf("Bundling: %d/10 readings collected\n", bundle_count);
            
            // When we have 10 readings, calculate averages and send
            if (bundle_count >= BUNDLE_SIZE) {
                // Calculate averages
                float avg_currentRMS = sum_currentRMS / BUNDLE_SIZE;
                float avg_power = sum_power / BUNDLE_SIZE;
                float avg_VIrms = sum_VIrms / BUNDLE_SIZE;
                float avg_DPF = sum_DPF / BUNDLE_SIZE;
                
                unsigned long bundle_duration = millis() - bundle_start_time;
                
                Serial.println("========== AVERAGED DATA (1 Hz) ==========");
                Serial.printf("Current RMS: %.3f A (avg of 10)\n", avg_currentRMS);
                Serial.printf("Power: %.2f W (avg of 10)\n", avg_power);
                Serial.printf("VI RMS: %.2f VA (avg of 10)\n", avg_VIrms);
                Serial.printf("DPF: %.3f (avg of 10)\n", avg_DPF);
                Serial.printf("Bundle time: %lu ms\n", bundle_duration);
                Serial.println("==========================================");
                
                // Monitor transmission timing
                unsigned long txStartTime = millis();
                
                // Get NTP timestamp
                time_t now;
                struct tm timeinfo;
                char timestamp[64];
                
                if (getLocalTime(&timeinfo)) {
                    strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", &timeinfo);
                } else {
                    // Fallback to millis if NTP not available
                    snprintf(timestamp, sizeof(timestamp), "millis:%lu", millis());
                }
                
                // Prepare JSON data with AVERAGED values
                DynamicJsonDocument doc(256);
                doc["timestamp"] = timestamp;
                doc["current"] = avg_currentRMS;
                doc["power"] = avg_power;
                doc["VIrms"] = avg_VIrms;
                doc["DPF"] = avg_DPF;
                doc["sample_count"] = BUNDLE_SIZE;  // Indicate this is averaged data

                String requestBody;
                serializeJson(doc, requestBody);

                // Send to local server
                HTTPClient http;
                http.begin(SERVER_URL);
                http.addHeader("Content-Type", "application/json");
                http.setTimeout(5000); // 5 second timeout (larger payload, server processing time)
                
                int responseCode = http.POST(requestBody);
                unsigned long txDuration = millis() - txStartTime;
                
                if (responseCode > 0) {
                    String response = http.getString();
                    Serial.printf("TX OK [%d] in %lums | Queue: %d/100\n", 
                                  responseCode, txDuration, uxQueueMessagesWaiting(sensorDataQueue));
                } else {
                    Serial.print("POST failed: ");
                    Serial.println(http.errorToString(responseCode));
                    // Don't retry - just drop the bundle to prevent queue overflow
                    Serial.println("Bundle dropped (queue protection)");
                }
                
                http.end();
                
                // Reset bundling accumulators
                sum_currentRMS = 0;
                sum_power = 0;
                sum_VIrms = 0;
                sum_DPF = 0;
                bundle_count = 0;
            }
        } // End of for loop (draining)
        
        // Check control state periodically
        if (millis() - lastStateCheck >= STATE_CHECK_INTERVAL) {
            lastStateCheck = millis();
            
            HTTPClient http;
            http.begin(GET_STATE_URL);
            http.setTimeout(3000); // 3 second timeout (small response, quick operation)
            
            int responseCode = http.GET();
            
            if (responseCode == 200) {
                String payload = http.getString();
                DynamicJsonDocument doc(128);
                DeserializationError error = deserializeJson(doc, payload);
                
                if (!error) {
                    String state = doc["state"];
                    
                    if (state == "OFF") {
                        digitalWrite(CONTROL_PIN, HIGH);
                        Serial.println("Control Pin: HIGH (OFF state)");
                    } else { 
                        digitalWrite(CONTROL_PIN, LOW);
                        Serial.println("Control Pin: LOW (ON state)");
                    }
                } else {
                    Serial.print("JSON parse error: ");
                    Serial.println(error.c_str());
                }
            } else if (responseCode < 0) {
                Serial.println("GET state timeout - will retry in 2s");
            } else {
                Serial.print("State check failed [");
                Serial.print(responseCode);
                Serial.println("]");
            }
            
            http.end();
        }
        
        // Small delay to prevent task starvation
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ==================== SETUP ====================
void setup() {
    Serial.begin(19200);
    Serial.println("\n\n=== ESP32-C3 FreeRTOS Power Monitor ===");
    
    // Configure ADC
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    // Configure pins
    pinMode(SENSOR_PIN, INPUT); 
    pinMode(IMPULSE_PIN, INPUT); 
    pinMode(CONTROL_PIN, OUTPUT);
    digitalWrite(CONTROL_PIN, LOW); // Default LOW

    // Connect to WiFi
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi");
    
    int wifi_attempts = 0;
    while (WiFi.status() != WL_CONNECTED && wifi_attempts < 40) {
        Serial.print(".");
        delay(500);
        wifi_attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi Connected!");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
        
        // Configure NTP
        configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
        Serial.println("NTP Time configured");
        
        // Wait a bit for NTP to sync
        delay(2000);
        
        struct tm timeinfo;
        if (getLocalTime(&timeinfo)) {
            Serial.println("NTP Time synced successfully");
            Serial.println(&timeinfo, "Current time: %Y-%m-%d %H:%M:%S");
        } else {
            Serial.println("NTP sync pending...");
        }
    } else {
        Serial.println("\nWiFi Connection Failed!");
        Serial.println("Continuing with limited functionality...");
    }

    // Create queue (holds 100 sensor readings = 10 seconds buffer)
    // Large buffer handles slow HTTP POST + control state checks
    sensorDataQueue = xQueueCreate(100, sizeof(SensorData_t));
    
    if (sensorDataQueue == NULL) {
        Serial.println("ERROR: Failed to create queue!");
        while(1) { delay(1000); } // Halt
    }
    Serial.println("Queue created successfully");

    // Create Data Acquisition Task
    BaseType_t task1Created = xTaskCreatePinnedToCore(
        dataAcquisitionTask,      // Task function
        "DataAcq",                // Task name
        DATA_TASK_STACK,          // Stack size
        NULL,                     // Parameters
        DATA_TASK_PRIORITY,       // Priority (higher)
        &dataTaskHandle,          // Task handle
        0                         // Core 0 (ESP32-C3 is single core, but kept for compatibility)
    );
    
    if (task1Created == pdPASS) {
        Serial.println("Data Acquisition Task created");
    } else {
        Serial.println("ERROR: Failed to create Data Acquisition Task!");
    }

    // Create WiFi Communication Task
    BaseType_t task2Created = xTaskCreatePinnedToCore(
        wifiCommunicationTask,    // Task function
        "WiFiComm",               // Task name
        WIFI_TASK_STACK,          // Stack size
        NULL,                     // Parameters
        WIFI_TASK_PRIORITY,       // Priority (lower)
        &wifiTaskHandle,          // Task handle
        0                         // Core 0
    );
    
    if (task2Created == pdPASS) {
        Serial.println("WiFi Communication Task created");
    } else {
        Serial.println("ERROR: Failed to create WiFi Communication Task!");
    }

    Serial.println("=== Setup Complete - Tasks Running ===\n");
}

// ==================== LOOP ====================
void loop() {
    // Empty - all work done by FreeRTOS tasks
    // This loop still runs but does nothing
    vTaskDelay(pdMS_TO_TICKS(10000)); // 10 second delay
}