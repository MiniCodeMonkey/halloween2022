#include <SPI.h>
#include <SdFat.h>
#include <FreeStack.h>
#include <SFEMP3Shield.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

#define SPI_SPEED SD_SCK_MHZ(4)

#define RELAY1_PIN       A1
#define RELAY2_PIN       3
#define FOG_MACHINE_PIN  A2
#define PIR_SENSOR_PIN   A3
#define BUTTON_PIN       4

#define LID_MOVE_TIME_MS 18000

SdFat sd;
SFEMP3Shield MP3player;
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_StepperMotor *winchMotor = AFMS.getStepper(200, 1);
unsigned long lastTriggerTime;

int pirValue;

void setup() {
  initSound();
  initMotor();
  
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  pinMode(RELAY1_PIN, OUTPUT);
  digitalWrite(RELAY1_PIN, LOW);

  pinMode(RELAY2_PIN, OUTPUT);
  digitalWrite(RELAY2_PIN, LOW);

  pinMode(FOG_MACHINE_PIN, OUTPUT);
  digitalWrite(FOG_MACHINE_PIN, LOW);

  pinMode(PIR_SENSOR_PIN, INPUT);
  
  Serial.begin(115200);
  while (!Serial);

  // testConnections();
  
  resetLidPosition();
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

void initMotor() {  
  if (AFMS.begin()) {
    Serial.println("Motor Shield found.");
  } else {
    Serial.println("Could not find Motor Shield. Check wiring.");
  }

  winchMotor->setSpeed(500);
}

void testConnections() {
  while (true) {
    playSound();
    
    raiseDementor();
    delay(20000);

    lowerDementor();
    delay(2000);

    openLid();
    delay(1000);
    closeLid();
    delay(1000);
    stopLid();

    startFog();
    delay(1000);
    stopFog();

    delay(5000);
   }
}

void resetLidPosition() {
  Serial.println("Resetting lid position");  
  closeLid();
  delay(LID_MOVE_TIME_MS);
  stopLid();
  Serial.println("Lid is closed");
}

void loop() {
  checkIfButtonPressed();
  
  pirValue = digitalRead(PIR_SENSOR_PIN);
  if (pirValue == HIGH) {
    Serial.println("Motion detected.");
    lastTriggerTime = millis();

    Serial.println("Playing sounds");
    playSound();
    delay(15000);

    startFog();
    openLid();
    delay(1000);
    raiseDementor();
    stopLid();
    stopFog();
    
    delay(35000);

    lowerDementor();
    closeLid();
    delay(LID_MOVE_TIME_MS);
    stopLid();
  }
   
  delay(100);
}

void playSound() {  
  MP3player.stopTrack();
  MP3player.playMP3("track007.mp3");
}

void openLid() {
  Serial.println("Opening lid");
  digitalWrite(RELAY1_PIN, HIGH);  
  digitalWrite(RELAY2_PIN, LOW);
}

void closeLid() {
  Serial.println("Closing lid");
  digitalWrite(RELAY1_PIN, LOW);  
  digitalWrite(RELAY2_PIN, HIGH);
}

void stopLid() {
  Serial.println("Stopping lid");
  digitalWrite(RELAY1_PIN, LOW);  
  digitalWrite(RELAY2_PIN, LOW);
}

void startFog() {
  Serial.println("Starting fog");
  digitalWrite(FOG_MACHINE_PIN, HIGH);
}

void stopFog() {
  Serial.println("Stopping fog");
  digitalWrite(FOG_MACHINE_PIN, LOW);
}

void raiseDementor() {
  Serial.println("Raising dementor");
  winchMotor->step(4000, FORWARD, DOUBLE);
  delay(2000);
}

void lowerDementor() {
  Serial.println("Lowering dementor");
  winchMotor->step(4000, BACKWARD, DOUBLE);
}

void checkIfButtonPressed() {
  int buttonVal = digitalRead(BUTTON_PIN);

  if (buttonVal == LOW) {
    Serial.println("Button pressed! Opening lid");
    openLid();
    delay(LID_MOVE_TIME_MS);
    stopLid();

    do {
      buttonVal = digitalRead(BUTTON_PIN);
      Serial.println("Waiting for button to be pressed again");
      delay(1000);
    }
    while (buttonVal == HIGH);

    Serial.println("Closing lid");
    closeLid();
    delay(LID_MOVE_TIME_MS);
    stopLid();
  }
}
