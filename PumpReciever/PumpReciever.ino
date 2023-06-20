#define WATERPIN 7  // The pin connected to the water pump

#include <Wire.h>
#include <avr/wdt.h>  //Watchdog

enum PumpState {
  PUMP_OFF,
  PUMP_ON
};

PumpState pumpState = PUMP_OFF;  // Current state of the water pump
unsigned long startTime = 0;     // Variable to store the start time
const unsigned long pumpDuration = 2000;

void setup() {
  Serial.begin(9600);
  wdt_enable(WDTO_8S);  // Set the watchdog timeout to 8 seconds
  pinMode(WATERPIN, OUTPUT);
  Wire.begin(8);  // Set the Arduino's address as 8
  Wire.onReceive(receiveEvent);
  pinMode(WATERPIN, HIGH);
}

void loop() {
  wdt_reset();

  pumpTimer();
}

void pumpTimer() {
  if (pumpState == PUMP_ON && millis() - startTime >= pumpDuration) {
    digitalWrite(WATERPIN, HIGH);  // Turn off the water pump
    pumpState = PUMP_OFF;
    Serial.println("Pump turned off");
  }
}

void receiveEvent(int numBytes) {
  while (Wire.available()) {
    char signal = Wire.read();
    Serial.println(signal);

    if (signal == '1' && pumpState == PUMP_OFF) {
      // Turn on the water pump
      digitalWrite(WATERPIN, LOW);
      pumpState = PUMP_ON;
      startTime = millis();
      Serial.println("Received signal: ON");
    } else if (signal == '0' && pumpState == PUMP_ON) {
      // Turn off the water pump
      digitalWrite(WATERPIN, HIGH);
      pumpState = PUMP_OFF;
      startTime = 0;  // Reset the start time
      Serial.println("Received signal: OFF");
    }
  }
}
