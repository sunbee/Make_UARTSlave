#include <Arduino.h>

#include <Wire.h>
#include <SerialTransfer.h>
#include <SoftwareSerial.h>

#define FAN_CONTROLLER  3
#define SPEED_LO        0
#define SPEED_MED       201
#define SPEED_HI        255

int count = 0;

/* THE ARDUINO UNO SLAVE */

SerialTransfer slaveMCU;
SoftwareSerial Extra(8, 9); // Rx: 8, Tx: 9; Need 2, 3 for interrupts
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
  int fan;
} status;

void counter() {
  count++;

}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(201);
  Extra.begin(9600);
  delay(201);
  slaveMCU.begin(Extra);
  delay(201);
  /*
  Set up the pin no. 3 for controlling fan speed.
  The fan has a built in PWM controller, 
  so it should be enough to send a PWN signal.
  */
  pinMode(FAN_CONTROLLER, OUTPUT);
  analogWrite(FAN_CONTROLLER, 0);
  /*
  Set up pin no. 2 to receive interrupts
  in order to calculate the fan speed.
  */
  attachInterrupt(digitalPinToInterrupt(2), counter, RISING);
}

void execute() {
  analogWrite(FAN_CONTROLLER, instructions.fan);
}

void debugRx() {
  Serial.print("SLAVE RX: ");
  Serial.print(millis());
  Serial.print("   Water: ");
  Serial.print((bool)instructions.water);
  Serial.print(", Fan: ");
  Serial.print(instructions.fan);
  Serial.print(", LED: ");
  Serial.println(instructions.led);
}

void debugTx() {
  Serial.print("SLAVE TX: ");
  Serial.print(millis());
  Serial.print("   Fan: ");
  Serial.println(status.fan);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (slaveMCU.available()) {
    slaveMCU.rxObj(instructions);
    debugRx();
    execute();
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
    /*
    Compute the fan speed from accumulated pulses 
    over DELTA ms and send result over UART. Reset 
    the timer and counter.
    */
    tic = toc;
    status.fan = count * DELTA * 60 / (2 * 1000);
    slaveMCU.txObj(status, sizeof(status));
    slaveMCU.sendDatum(status);  
    debugTx();
    count = 0; 
  }
}