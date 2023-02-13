#include <Bump_Switch.h>
#include <Encoder.h>
#include <GP2Y0A21_Sensor.h>
#include <QTRSensors.h>
#include <Romi_Motor_Power.h>
#include <RSLK_Pins.h>
#include <SimpleRSLK.h>
/* notes:
  The code below is non-blocking with a hard-coded delay of 2ms per loop. */

bool onToggle = false; // toggle all functions on/off (off disables all motors)
int lpButtonValue;     // launchpad left button
int lpButtonLastValue;

int lprButtonValue; // launchpad right button
int lprButtonLastValue;

// line follower
uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS];
uint16_t sensorMinVal[LS_NUM_SENSORS];
uint8_t lineColor = DARK_LINE;
uint32_t linePos = 0;

void setup()
{
  Serial.begin(9600);

  setupRSLK();

  setupLed(RED_LED);

  pinMode(PUSH2, INPUT_PULLUP); // left launchpad button
  pinMode(PUSH1, INPUT_PULLUP); // right launchpad button

  setDefaults();

  clearMinMax(sensorMinVal, sensorMaxVal);
}
void loop()
{
  // read global sensors
  lpButtonValue = digitalRead(PUSH2);
  lprButtonValue = digitalRead(PUSH1);

  // all toggle (Left LP Button)
  if (lpButtonValue == 1 && lpButtonLastValue == 0)
  { // button from released to pressed
    lpButtonLastValue = 1;
    onToggle = !onToggle;
  }
  if (lpButtonValue == 0 && lpButtonLastValue == 1)
  { // button from pressed to released
    lpButtonLastValue = 0;
  }

  // clearMinMax toggle (Right LP Button)
  if (lprButtonValue == 1 && lprButtonLastValue == 0)
  { // button from released to pressed
    lprButtonLastValue = 1;
    clearMinMax(sensorMinVal, sensorMaxVal);
  }
  if (lprButtonValue == 0 && lprButtonLastValue == 1)
  { // button from pressed to released
    lprButtonLastValue = 0;
  }

  if (onToggle)
  {
    // read sensors when toggled on
    //    line follower
    readLineSensor(sensorVal);
    readCalLineSensor(sensorVal,
                      sensorCalVal,
                      sensorMinVal,
                      sensorMaxVal,
                      lineColor);

    linePos = getLinePosition(sensorCalVal, lineColor);

    setSensorMinMax(sensorVal, sensorMinVal, sensorMaxVal);

    /*PRINT PIN STATE FOR DEBUGGING PURPOSES*/
    Serial.print("LF:[");
    Serial.print(sensorVal[0]); // the left-most sensor if facing same direction as robot
    Serial.print(",");
    Serial.print(sensorVal[1]);
    Serial.print(",");
    Serial.print(sensorVal[2]);
    Serial.print(",");
    Serial.print(sensorVal[3]);
    Serial.print(",");
    Serial.print(sensorVal[4]);
    Serial.print(",");
    Serial.print(sensorVal[5]);
    Serial.print(",");
    Serial.print(sensorVal[6]);
    Serial.print(",");
    Serial.print(sensorVal[7]);
    Serial.print("]");
    Serial.print(" maxLF:[");
    Serial.print(sensorMaxVal[0]); // the left-most sensor if facing same direction as robot
    Serial.print(",");
    Serial.print(sensorMaxVal[1]);
    Serial.print(",");
    Serial.print(sensorMaxVal[2]);
    Serial.print(",");
    Serial.print(sensorMaxVal[3]);
    Serial.print(",");
    Serial.print(sensorMaxVal[4]);
    Serial.print(",");
    Serial.print(sensorMaxVal[5]);
    Serial.print(",");
    Serial.print(sensorMaxVal[6]);
    Serial.print(",");
    Serial.print(sensorMaxVal[7]);
    Serial.print("]");
    Serial.print(" minLF:[");
    Serial.print(sensorMinVal[0]); // the left-most sensor if facing same direction as robot
    Serial.print(",");
    Serial.print(sensorMinVal[1]);
    Serial.print(",");
    Serial.print(sensorMinVal[2]);
    Serial.print(",");
    Serial.print(sensorMinVal[3]);
    Serial.print(",");
    Serial.print(sensorMinVal[4]);
    Serial.print(",");
    Serial.print(sensorMinVal[5]);
    Serial.print(",");
    Serial.print(sensorMinVal[6]);
    Serial.print(",");
    Serial.print(sensorMinVal[7]);
    Serial.print("]");

    // Serial.print(" calLF:[");
    // Serial.print(sensorCalVal[0]); // the left-most sensor if facing same direction as robot
    // Serial.print(",");
    // Serial.print(sensorCalVal[1]);
    // Serial.print(",");
    // Serial.print(sensorCalVal[2]);
    // Serial.print(",");
    // Serial.print(sensorCalVal[3]);
    // Serial.print(",");
    // Serial.print(sensorCalVal[4]);
    // Serial.print(",");
    // Serial.print(sensorCalVal[5]);
    // Serial.print(",");
    // Serial.print(sensorCalVal[6]);
    // Serial.print(",");
    // Serial.print(sensorCalVal[7]);
    // Serial.print("]");
    // Serial.print(" linePos:");
    // Serial.print(linePos);
  }
  else
  { // toggle off
    disableMotor(BOTH_MOTORS);
    setDefaults();
  }
  Serial.println("");
  delay(2);
} // END LOOP

void setDefaults()
{
  // lp button values
  lpButtonValue = 0; // launchpad left button
  lpButtonLastValue = 0;
}
