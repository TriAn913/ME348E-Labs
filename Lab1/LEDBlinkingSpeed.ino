#include <Bump_Switch.h>
#include <Encoder.h>
#include <GP2Y0A21_Sensor.h>
#include <QTRSensors.h>
#include <Romi_Motor_Power.h>
#include <RSLK_Pins.h>
#include <SimpleRSLK.h>


int potPin = A5;      // potentiometer
int buttonPin = A4;   // button
int ledPin = 12;      // LED

int potValue = 0;     // potentiometer value
int buttonValue = 0;  // button value (0 when pressed, 1 when not)
int buttonPressCount = 0; // number of times button is pressed
bool prevButState = true; // used to prevent buttonPressCount from continuously increasing when the button is held down

int ledState = LOW;             // ledState used to set the LED
long previousMillis = 0;        // will store last time LED was updated
long interval = 1000;           // interval at which to blink (milliseconds)

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication:
  Serial.begin(9600);
  digitalWrite(ledPin, LOW); // ensure the led is off at the start
}

// the loop routine runs over and over again forever:
void loop() {
  // read the value from the sensor:
  potValue = analogRead(potPin);
  buttonValue = digitalRead(buttonPin);

  // determine number of button presses. Only increment if it changes from off to on
  if (buttonValue == LOW && !prevButState) {
    buttonPressCount++;
    prevButState = true;
  }
  if (buttonValue == HIGH && prevButState){
    prevButState = false;
  }

  // begin led blinking once button has been pressed twice
  if (buttonPressCount >= 2) {
    unsigned long currentMillis = millis();
    interval = potValue;
    if (currentMillis - previousMillis > interval) {
      // save the last time you blinked the LED
      previousMillis = currentMillis;

      // if the LED is off turn it on and vice-versa:
      if (ledState == LOW)
        ledState = HIGH;
      else
        ledState = LOW;

      // set the LED with the ledState of the variable:
      digitalWrite(ledPin, ledState);
    }
  }
  /*
  // debugging input values and selected variables
  Serial.print("potVal: ");
  Serial.println(potValue);
  Serial.print("buttonVal: ");
  Serial.println(buttonValue);
  Serial.print("prevVal: ");
  Serial.println(prevButState);
  Serial.print("presses: ");
  Serial.println(buttonPressCount);
  */
  delay(2);
}
