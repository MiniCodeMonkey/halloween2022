#include <SPI.h>
#include <SdFat.h>
#include <FreeStack.h>
#include <SFEMP3Shield.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

#define SPI_SPEED SD_SCK_MHZ(4)

#define BUTTON_PIN     12
#define RELAY1_PIN     4
#define RELAY2_PIN     1
#define PIR_SENSOR_PIN 5

#define LID_OPEN_TIME_MS 5000

SdFat sd;
SFEMP3Shield MP3player;
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_StepperMotor *winchMotor = AFMS.getStepper(200, 2);
unsigned long lastTriggerTime;

int pirValue;

void setup() {  
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  pinMode(RELAY1_PIN, OUTPUT);
  digitalWrite(RELAY1_PIN, LOW);

  pinMode(RELAY2_PIN, OUTPUT);
  digitalWrite(RELAY2_PIN, LOW);

  pinMode(PIR_SENSOR_PIN, INPUT);
  
  Serial.begin(115200);
  while (!Serial);

  randomSeed(analogRead(0));

  /*while (true) {
    digitalWrite(RELAY1_PIN, LOW);
    digitalWrite(RELAY2_PIN, HIGH);
    delay(500);

    digitalWrite(RELAY1_PIN, HIGH);
    digitalWrite(RELAY2_PIN, LOW);
    delay(500);

    digitalWrite(RELAY1_PIN, HIGH);
    digitalWrite(RELAY2_PIN, HIGH);
    delay(500);

    digitalWrite(RELAY1_PIN, LOW);
    digitalWrite(RELAY2_PIN, LOW);
    delay(5000);
  }*/

  initSound();
  initMotor();
}

void initMotor() {  
  if (!AFMS.begin()) {
    Serial.println("Could not find Motor Shield. Check wiring.");
  }
  Serial.println("Motor Shield found.");

  winchMotor->setSpeed(500);

  /*while (true) {
    winchMotor->step(2000, FORWARD, DOUBLE);
    delay(1000);
    winchMotor->step(2000, BACKWARD, DOUBLE);
    delay(2500);
  }*/
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
  int buttonVal = digitalRead(BUTTON_PIN);

  if (buttonVal == LOW) {
    Serial.println("Button pressed! Opening lid");
    openLid();
    delay(LID_OPEN_TIME_MS);
    stopLid();

    do {
      buttonVal = digitalRead(BUTTON_PIN);
      Serial.println("Waiting for button to be pressed again");
      delay(1000);
    }
    while (buttonVal == HIGH);

    Serial.println("Closing lid");
    closeLid();
    delay(LID_OPEN_TIME_MS);
    stopLid();
  }
  
  pirValue = digitalRead(PIR_SENSOR_PIN);
  if (pirValue == HIGH) {
    Serial.println("Motion detected.");
    lastTriggerTime = millis();

    playSound();
    openLid();
    delay(2000);

    raiseAndLowerDementorAndLid();
    
    delay(10000);
  }
   
  delay(100);
}

void playSound() {  
  MP3player.stopTrack();
  MP3player.playMP3("track007.mp3");
}

void openLid() {
  digitalWrite(RELAY1_PIN, HIGH);  
  digitalWrite(RELAY2_PIN, LOW);
}

void closeLid() {
  digitalWrite(RELAY1_PIN, LOW);  
  digitalWrite(RELAY2_PIN, HIGH);
}

void stopLid() {
  digitalWrite(RELAY1_PIN, LOW);  
  digitalWrite(RELAY2_PIN, LOW);
}

void raiseAndLowerDementorAndLid() {
  winchMotor->step(2000, FORWARD, DOUBLE);
  delay(2000);
  winchMotor->step(2000, BACKWARD, DOUBLE);
  delay(2000);
  closeLid();
  delay(2500);

  stopLid();
}
