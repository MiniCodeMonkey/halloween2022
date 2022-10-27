#include <Adafruit_MotorShield.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <TelnetStream.h>
#include <ArduinoOTA.h>
#include "wifi_credentials.h"

WiFiClient client;

#define DEVICE_NAME "ghosts"
#define WIFI_CONNECT_TIMEOUT_MS 15000

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_StepperMotor *stepper = AFMS.getStepper(200, 2);

uint8_t currentSpeed = 0;

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

    TelnetStream.begin();
  }

  if (!AFMS.begin()) {
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
}

void loop() {
  ArduinoOTA.handle();
  stepper->step(100, FORWARD, DOUBLE);

  uint8_t speedValue = analogRead(A3) / 16;
  if (speedValue < 20) {
    speedValue = 20;
  }

  if (speedValue != currentSpeed) {
    Serial.print("Changed speed to ");
    Serial.println(currentSpeed);
    currentSpeed = speedValue;
    stepper->setSpeed(currentSpeed);
  }
}
