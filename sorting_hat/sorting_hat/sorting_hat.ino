#include <Wire.h>
#include <Adafruit_VL6180X.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <TelnetStream.h>
#include <ArduinoOTA.h>
#include <SonosUPnP.h>
#include <MicroXPath_P.h>
#include "wifi_credentials.h"

WiFiClient client;
SonosUPnP g_sonos = SonosUPnP(client, 0);

IPAddress g_SpeakerIP(192, 168, 3, 13);

#define DEVICE_NAME "sortinghat"
#define WIFI_CONNECT_TIMEOUT_MS 60000
#define WAIT_FOR_PERSON_TO_LEAVE_TIMEOUT_MS 20000

//String FILESERVER_PREFIX = "http://bravo.local:8000/";
String FILESERVER_PREFIX = "http://192.168.3.35:8000/";

#define HOUSES_COUNT 4
String houses[HOUSES_COUNT] = { "hufflepuff", "ravenclaw", "slytherin", "gryffindor" };

Adafruit_VL6180X vl = Adafruit_VL6180X();

void setup() {
  Serial.begin(115200);
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Connection Failed");
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

  if (!vl.begin()) {
    TelnetStream.println("Failed to find sensor");
    while (1);
  }

  randomSeed(analogRead(0));
}

void loop() {
  ArduinoOTA.handle();
  
  uint8_t range = vl.readRange();
  uint8_t status = vl.readRangeStatus();

  if (status == VL6180X_ERROR_NONE) {
    TelnetStream.print("Range: ");
    TelnetStream.println(range);

    sortIntoHouse();
  }
  delay(50);
}

void sortIntoHouse() {
    g_sonos.setVolume(g_SpeakerIP, 40);

    String selectedHouse = houses[random(0, HOUSES_COUNT)];

    Serial.println(selectedHouse);

    String filename_prefix = "sorting_hat_";
    String filename_suffix = ".mp3";
    String url = FILESERVER_PREFIX + filename_prefix + selectedHouse + filename_suffix;

    Serial.println(url);
    
    g_sonos.playHttp(g_SpeakerIP, url.c_str());

    bool isPlaying = true;

    while (isPlaying) {
      delay(1000);
      isPlaying = g_sonos.getState(g_SpeakerIP) == SONOS_STATE_PLAYING;
    }

    Serial.println("Done!");

    waitForPersonToLeave();
    
    Serial.println("Ready for next student");
    delay(5000);
}

void waitForPersonToLeave() {
  int startTime = millis();
  uint8_t range = vl.readRange();
  uint8_t status = vl.readRangeStatus();
  
  while (status == VL6180X_ERROR_NONE) {
    Serial.println("Waiting for person to leave");
    delay(500);
    range = vl.readRange();
    status = vl.readRangeStatus();

    if (millis()-startTime > WAIT_FOR_PERSON_TO_LEAVE_TIMEOUT_MS) {
      Serial.println("Timed out, waiting for person to leave");
      break;
    }
  }
}
