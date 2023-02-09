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


// line follower
uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS];
uint16_t sensorMinVal[LS_NUM_SENSORS];
uint8_t lineColor = DARK_LINE;
uint32_t linePos = 0;

#define maxVal 900
#define minVal 1100
static const int maxWhite[8] = {maxVal, maxVal, maxVal, maxVal, maxVal, maxVal, maxVal, maxVal};
static const int minBlack[8] = {minVal, minVal, minVal, minVal, minVal, minVal, minVal, minVal};






void setup()
{
  Serial.begin(9600);

  setupRSLK();

  setupLed(RED_LED);

  pinMode(PUSH2, INPUT_PULLUP); // left launchpad button

  setDefaults();
  
  clearMinMax(sensorMinVal, sensorMaxVal);
}
void loop()
{
  // read global sensors
  lpButtonValue = digitalRead(PUSH2);
  Serial.print("LP BTN: ");
  Serial.print(lpButtonValue);

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
