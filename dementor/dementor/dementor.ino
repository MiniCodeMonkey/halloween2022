#include <SPI.h>
#include <SdFat.h>
#include <FreeStack.h>
#include <SFEMP3Shield.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

#define SPI_SPEED SD_SCK_MHZ(4)

#define RELAY1_PIN     4
#define RELAY2_PIN     0
#define PIR_SENSOR_PIN    5

SdFat sd;
SFEMP3Shield MP3player;
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *lidMotor = AFMS.getMotor(2);
unsigned long lastGrowlTime;

int pirValue;

void setup() {  
  pinMode(RELAY1_PIN, OUTPUT);
  digitalWrite(RELAY1_PIN, LOW);

  pinMode(PIR_SENSOR_PIN, INPUT);
  
  Serial.begin(115200);
  while (!Serial);

  randomSeed(analogRead(0));

  initSound();
  initMotor();
}

void initMotor() {  
  if (!AFMS.begin()) {
    Serial.println("Could not find Motor Shield. Check wiring.");
  }
  Serial.println("Motor Shield found.");

  lidMotor->setSpeed(100);
  lidMotor->run(RELEASE);
}

void initSound() {
  if (!sd.begin(SD_SEL, SPI_SPEED)) {
    sd.initErrorHalt();
  }
  if (!sd.chdir("/")) {
    sd.errorHalt("sd.chdir");
  }

  if (MP3player.begin() != 0) {
    Serial.print(F("Error when trying to start MP3 player"));
  }

  MP3player.setVolume(20, 20);
}

void loop() {
  pirValue = digitalRead(PIR_SENSOR_PIN);
  if (pirValue == HIGH && millis()-lastGrowlTime > 15000) {
    Serial.println("Motion detected.");
    lastGrowlTime = millis();

    growl();

    Serial.println("Running motor.");
    digitalWrite(RELAY1_PIN, HIGH);  
    lidMotor->run(FORWARD);
    delay(1500);
    digitalWrite(RELAY1_PIN, LOW);
    lidMotor->run(RELEASE);
    delay(100);

    while (MP3player.isPlaying()) {
      Serial.println("Waiting for sound.");
      delay(100);
    }

    delay(2500);

    MP3player.playMP3("yawns001.mp3");
  }

  if (!MP3player.isPlaying()) {
    Serial.println("Snooring");
    MP3player.playMP3("snore001.mp3");
  }
    
  delay(100);
}

void growl() {  
  Serial.println("Preparing to growl");
  char filename[9];
  int trackNo = random(1, 3);
  Serial.println(trackNo);
  sprintf(filename, "growl00%d.mp3", trackNo);

  Serial.println(filename);

  MP3player.stopTrack();
  MP3player.playMP3(filename);
}
