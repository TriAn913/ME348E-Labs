int IRbeacon1 = 38;
int interval = 2; // ms
int previousMillis = 0;

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600); // msp430g2231 must use 4800
  // make the on-board pushbutton's pin an input pullup:
  pinMode(IRbeacon1, INPUT_PULLUP); // NOTE: because this is a pullup, a 1 indicates no beacon detected, 0 is yes beacon detected
}

// non-blocking by using interval and millis()
void loop() {
 
  unsigned long currentMillis = millis();
    if (currentMillis - previousMillis > interval) {
      // save the last time you blinked the LED
      previousMillis = currentMillis;

      int IRState = digitalRead(IRbeacon1);

      Serial.println(IRState); // Note - it's 0 if beacon is ON, 1 if beacon is OFF.
    }
}
