#define soilWet 500   // define max value we consider soil 'wet'
#define soilDry 750   // define min value we consider soil 'dry'
#define CONTRAST 150  //for the LCD without ising a potentiometer
#define LCDPIN 6      //digitalPin for LCD Screen instead of poteniometer
#define SENSMIN 0     // the lowest value that comes out of the
#define SENSMAX 1023  // the highest value that comes out of the potentiometer.
#define POTENT A1     // potent analogPin
#define MSENSOR A0    // moisture analogPin
#define MSENSPOW 7    //moisture digitalPin
#define RECIEVER 8    //  I2C communication
#define MAPMIN 0
#define MAPMAX 100
#define POTENTMIN 50
#define POTENTMAX 100
#define TIMEOUT 60000;

#define ACTIVE '1'
#define INACTIVE '0'

#include <avr/wdt.h>  //Watchdog
#include <Wire.h>
#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

enum State {
  IDLE,
  MEASURING_MOISTURE,
  MEASURING_POTENT,
  SETLCD,
  SENDREQUEST,
};

State currentState = IDLE;  // Initial state

unsigned long previousMillis = 0;    // Variable to store the previous time
const unsigned long interval = 700;  // Interval between updates (in milliseconds)

int moistureLevel;
int potentLevel;
bool pumpActive = false;

void setup() {
  Serial.begin(9600);
  wdt_enable(WDTO_8S);  // Set the watchdog timeout to 8 seconds
  pinMode(MSENSPOW, OUTPUT);
  digitalWrite(MSENSPOW, LOW);
  analogWrite(LCDPIN, CONTRAST);
  lcd.begin(16, 2);
  Wire.begin();
}

void loop() {
  wdt_reset();
  stateMachine();
}

void stateMachine() {
  unsigned long currentMillis = millis();  // Current time

  // Check if the interval has elapsed
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;  // Update the previous time
    switch (currentState) {

      case IDLE:
        if (!pumpActive) {
          Serial.println("IDLE");

          currentState = MEASURING_MOISTURE;

        } else {
          // Additional condition to handle the case when pumpActive is true for too long
          unsigned long pumpActiveTimeout = TIMEOUT;  // Timeout duration in milliseconds (60 seconds)
          Serial.println("TIME");
          if (currentMillis - previousMillis >= pumpActiveTimeout) {
            Serial.println("OUT");
            pumpActive = false;  // Set pumpActive to false if timeout occurs
          }
        }
        break;

      case MEASURING_MOISTURE:
        moistureLevel = readSensor(MSENSPOW, MSENSOR);
        moistureLevel = mapPercent(moistureLevel, SENSMIN, SENSMAX, MAPMAX, MAPMIN);
        Serial.println("MOISTURE");

        currentState = MEASURING_POTENT;
        break;

      case MEASURING_POTENT:
        potentLevel = analogRead(POTENT);
        potentLevel = mapPercent(potentLevel, SENSMIN, SENSMAX, POTENTMIN, POTENTMAX);
        Serial.println("POTENT");

        currentState = SETLCD;
        break;

      case SETLCD:
        setLCD(moistureLevel, potentLevel);
        Serial.println("LCD");

        currentState = SENDREQUEST;
        break;

      case SENDREQUEST:
        checkRequest(moistureLevel, potentLevel);
        Serial.println("REQUEST");

        currentState = IDLE;
    }
  }
}


// This function returns the analog soil moisture measurement
// This is used due to using a digitalPin instead of the 3.3v for a power source (which is under 20mA)
int readSensor(int digitalPin, int analogPin) {
  digitalWrite(digitalPin, HIGH);  // Turn the sensor ON
  delay(10);                       // Allow power not blow up >:|
  int moisture = analogRead(analogPin);
  digitalWrite(digitalPin, LOW);  //Turn the sensor off
  return moisture;
}


//Takes arguments and returns the value as a percentage
int mapPercent(int value, int min, int max, int valueMin, int valueMax) {
  return map(value, min, max, valueMin, valueMax);
}


void setLCD(int moisture, int potent) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Moisture: ");
  lcd.print(moisture);
  lcd.print("%");
  lcd.setCursor(0, 1);
  lcd.print("Target: ");
  lcd.print(potent);
  lcd.print("%");
}


void checkRequest(int moisture, int potent) {
  if (moisture < potent) {
    // Turn on the water pump
    Serial.println("ON");
    sendRequest(RECIEVER, ACTIVE);

  } else if (moisture >= potent) {
    // Turn off the water pump
    Serial.println("OFF");
    sendRequest(RECIEVER, INACTIVE);
  }
}


//  I2C communication
void sendRequest(int address, byte value) {
  Wire.beginTransmission(address);
  Wire.write(value);
  Wire.endTransmission();
  Serial.println("SENT");
}