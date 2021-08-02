#include <Arduino.h>

#include <Wire.h>
#include <SerialTransfer.h>
#include <SoftwareSerial.h>

/* THE ARDUINO UNO SLAVE */

SerialTransfer slaveMCU;
SoftwareSerial Extra(2, 3); // Rx: 2, Tx: 3
unsigned long tic = millis();
unsigned long toc = tic;
#define DELTA 1000

struct PAYMASTER {
  /*
  water: instruction to switch pump on or off. Note the float sensor in pump's circuit will prevent overflow.
  fan: instruction to control fan speed - LO, MED, HIGH. Note PC fan requires an int between 0 and 255.
  led: instruction to control LED brightness. Note that the FastLED library requires an int between 0 and 255.
  */
  bool water;
  uint8_t fan; 
  uint8_t led;
} instructions;

struct PAYSLAVE {
  /*
  fan: the fan speed read off the pin no. 3 (yellow wire) of a PC fan.
  */
  uint8_t fan;
} status = {
  234
};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(201);
  Extra.begin(9600);
  delay(201);
  slaveMCU.begin(Extra);
  delay(201);
}

void debug() {
  Serial.print("SLAVE: ");
  Serial.print(millis());
  Serial.print("   Water: ");
  Serial.print((bool)instructions.water);
  Serial.print(", Fan: ");
  Serial.print(instructions.fan);
  Serial.print(", LED: ");
  Serial.println(instructions.led);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (slaveMCU.available()) {
    slaveMCU.rxObj(instructions);
    debug();
  } else if (slaveMCU.status < 0) {
    Serial.print("ERROR: ");

    if(slaveMCU.status == -1)
      Serial.println(F("CRC_ERROR"));
    else if(slaveMCU.status == -2)
      Serial.println(F("PAYLOAD_ERROR"));
    else if(slaveMCU.status == -3)
      Serial.println(F("STOP_BYTE_ERROR"));
  }

  toc = millis();
  if ((toc - tic) > DELTA) {
    tic = toc;
    slaveMCU.txObj(status, sizeof(status));
    slaveMCU.sendDatum(status);
  }
}