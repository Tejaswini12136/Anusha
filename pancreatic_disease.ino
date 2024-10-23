#define BLYNK_TEMPLATE_ID "TMPL3d1lK_1n2"
#define BLYNK_TEMPLATE_NAME "pancreatic"
#define BLYNK_AUTH_TOKEN "FdJdW9NpgqdGf_XB5x2kn264MsYEujbo"

#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h" // For BPM calculations
#include "spo2_algorithm.h" // Include SpO2 algorithm header
#include <WiFi.h>*/\
#include <BlynkSimpleEsp32.h>
#include <DHT.h>

MAX30105 particleSensor;

// DHT11 configuration
#define DHTPIN 4        // Pin where the DHT11 is connected
#define DHTTYPE DHT11   // DHT 11
DHT dht(DHTPIN, DHTTYPE);

const byte RATE_SIZE = 4;
byte rates[RATE_SIZE]; // Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; // Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;

uint32_t irBuffer[100]; // Buffer to store infrared LED sensor data
uint32_t redBuffer[100]; // Buffer to store red LED sensor data
int bufferLength = 100; // Length of buffer for SpO2 calculation
int32_t spo2; // SPO2 value
int8_t validSPO2; // Indicator to check if SPO2 calculation is valid
int32_t heartRate; // Calculated heart rate
int8_t validHeartRate; // Indicator to check if heart rate calculation is valid

// Wi-Fi credentials
const char* ssid = "vivoT12";
const char* password = "china143";
char auth[] = BLYNK_AUTH_TOKEN;

#define REPORTING_PERIOD_MS 1000
uint32_t tsLastReport = 0; 

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing...");

  // Initialize MAX30105 sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) { // Use default I2C port, 400kHz speed
    Serial.println("MAX30105 was not found. Please check wiring/power.");
    while (1);
  }

  Serial.println("Place your index finger on the sensor with steady pressure.");
  particleSensor.setup(); // Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x1F); // Set Red LED for SpO2
  particleSensor.setPulseAmplitudeIR(0x1F); // Set IR LED for BPM

  // Initialize DHT sensor
  dht.begin();

  // Set up WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi Connected");

  // Set up Blynk
  Blynk.begin(auth, ssid, password);
}

void loop() {
  // Collect 100 samples of both IR and Red LED
  for (int i = 0; i < bufferLength; i++) {
    while (particleSensor.available() == false) // Wait for new data
      particleSensor.check();
      
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); // Move to next sample
  }

  // Calculate heart rate and SpO2
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  // Display results in Serial Monitor
  Serial.print("Heart Rate: ");
  if (validHeartRate) {
    Serial.print(heartRate);
  } else {
    Serial.print("Invalid");
  }
  
  Serial.print(" bpm, SpO2: ");
  if (validSPO2) {
    Serial.print(spo2);
  } else {
    Serial.print("Invalid");
  }
  Serial.println();


  // Read temperature and humidity from DHT11
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Display temperature and humidity
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.print("°C, Humidity: ");
  Serial.print(h);
  Serial.println("%");

  // Check for temperature alert
  if (t > 34) {
    Blynk.logEvent("high_temperature", "Alert: Temperature exceeds 33°C! check once and need to consult doctor");
  }

  // Report to Blynk periodically
  if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
    // Send Heart Rate and SpO2 to Blynk
    if (validHeartRate) {
      Blynk.virtualWrite(V0, heartRate);   // Send Heart Rate to Virtual Pin 0
    }
    if (validSPO2) {
      Blynk.virtualWrite(V1, spo2);  // Send SpO2 to Virtual Pin 1
    }
    // Send Temperature and Humidity to Blynk
    Blynk.virtualWrite(V2, t); // Send Temperature to Virtual Pin 2
    Blynk.virtualWrite(V3, h); // Send Humidity to Virtual Pin 3
    
    // Log event for low heart rate
    if (heartRate >100) {
      Blynk.logEvent("heart_rate", "pancreatic symptoms detected, need to consult doctor");
    }
    
    tsLastReport = millis();
  }

  Blynk.run();
  delay(1000); // Delay for next measurement
}
