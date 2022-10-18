#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "wifi_credentials.h"

#define DEVICE_NAME "basilisk"
#define WIFI_CONNECT_TIMEOUT_MS 15000

#define PIR_SENSOR_PIN A2
#define BATTERY_VOLTAGE_PIN A13

#define SPRINKLER_RELAY_PIN 12

#define MAX_ANALOG_VAL 4095
#define MAX_BATTERY_VOLTAGE 4.2 // Max LiPoly voltage of a 3.7 battery is 4.2

int pirVal = 0;
int batteryVoltage = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFI Connection Failed");
    delay(1000);

    if (millis() > WIFI_CONNECT_TIMEOUT_MS) {
      Serial.println("Giving up. Continuing without WiFi");
      break;
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    ArduinoOTA.setHostname(DEVICE_NAME);
    ArduinoOTA.begin();
  
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }

  pinMode(PIR_SENSOR_PIN, INPUT);
  pinMode(BATTERY_VOLTAGE_PIN, INPUT);
  
  pinMode(SPRINKLER_RELAY_PIN, OUTPUT);
  digitalWrite(SPRINKLER_RELAY_PIN, LOW);
}

void loop() {
  ArduinoOTA.handle();
  
  /*
  digitalWrite(SPRINKLER_RELAY_PIN, HIGH);
  delay(500);
  digitalWrite(SPRINKLER_RELAY_PIN, LOW);
  delay(10000);
  */

  pirVal = digitalRead(PIR_SENSOR_PIN);

  if (pirVal == HIGH) {
    Serial.println("Motion detected");
  } else {
    Serial.println("Motion NOT detected");
  }

  delay(1000);
}

void checkBattery() {
  // Reference voltage on ESP32 is 1.1V
  // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc.html#adc-calibration
  // See also: https://bit.ly/2zFzfMT
  int rawValue = analogRead(BATTERY_VOLTAGE_PIN);
  float voltageLevel = (rawValue / 4095.0) * 2 * 1.1 * 3.3;
  float batteryFraction = voltageLevel / MAX_BATTERY_VOLTAGE;

  Serial.println((String)"Raw:" + rawValue + " Voltage:" + voltageLevel + "V Percent: " + (batteryFraction * 100) + "%");
}
