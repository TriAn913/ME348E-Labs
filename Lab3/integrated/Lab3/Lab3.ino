#include <Bump_Switch.h>
#include <Encoder.h>
#include <GP2Y0A21_Sensor.h>
#include <QTRSensors.h>
#include <Romi_Motor_Power.h>
#include <RSLK_Pins.h>
#include <SimpleRSLK.h>
/* notes:
  The code below is non-blocking with a hard-coded delay of 2ms per loop. */

bool onToggle = false;  // toggle all functions on/off (off disables all motors)
bool lpButtonValue;     // launchpad left button
bool lpButtonLastValue; // the value of the launchpad left button value the last time checked

uint16_t motorSpeed = 10;
// static const float wheelDiameter = 2.7559055;

// encoder counts
uint32_t leftCount;
uint32_t rightCount;

// line follower
uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS] = {};
uint16_t sensorMinVal[LS_NUM_SENSORS] = {};
uint8_t lineColor = DARK_LINE;
uint32_t linePos = 9999;
uint8_t interCounts; // number of intersections crossed
bool prevInter;      // whether was on intersection last time checked (true: was on intersection; false: was not on intersection)
bool onInter;        // is on intersection (true: on intersection; false: not on intersection)
#define MAX_VAL 900
#define MIN_VAL 1100

// linefollower PID
uint32_t error;
uint32_t prevError;
static const uint32_t setpoint = 3500;
static const float dt = 0.1f;
float integral;
float deriv;
uint16_t output;
static const float kp = 1.0f / 3.5f / 100.0f; // error of 3500 (max) returns 10
static const float ki = 0;
static const float kd = 0.1f / 3.5f / 100.0f;

// states
enum states // operational state
{
  NONE,
  LINE_A,
  LINE_B,
  TURN,
  STOP
};

states curr_state[1];

// substates
uint8_t substate; // which substate is active; 0:none, 1:init, 2:main, 3:exit, 4:finished

int turnRotations;
#define LEFT 0
#define RIGHT 1
#define NINETY_DEG_TURN 180
#define ONE_EIGHTY_DEG_TURN 360
uint8_t turnDirection; // 0:left, 1:right

void setDefaults()
{
  // lp button values
  lpButtonValue = 0; // launchpad left button value
  lpButtonLastValue = 0;

  setDefaultsIntersection();
  setDefaultsLFPID();
  setDefaultsEncoderCnts();
  
  // turning
  turnRotations = 0;
  turnDirection = LEFT; // 0:left, 1:right

  // states and substates
  curr_state[0] = LINE_A;
  curr_state[1] = NONE;
  substate = 0;

}

void setDefaultsIntersection() {
  // line follower intersection values
  interCounts = 0;
  prevInter = false;
  onInter = false;
}

void setDefaultsLFPID() {
  // line follower PID
  error = 0;
  prevError = 0;
  integral = 0;
  deriv = 0;
  output = 0;
}

void setDefaultsEncoderCnts() {
  // encoder counts
  leftCount = 0;
  rightCount = 0;
  resetLeftEncoderCnt();
  resetRightEncoderCnt();
}

void setup()
{
  Serial.begin(9600);

  setupRSLK();

  setupLed(RED_LED);

  pinMode(PUSH2, INPUT_PULLDOWN); // left launchpad button

  setDefaults();
}
void loop()
{
  // read global sensors
  lpButtonValue = digitalRead(PUSH2);
  Serial.print("LLPB:");
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

    linePos = getLinePosition2(sensorCalVal, lineColor);
    //    encoder
    leftCount = getEncoderLeftCnt();
    rightCount = getEncoderRightCnt();

    /*PRINT PIN STATE FOR DEBUGGING PURPOSES*/
    Serial.print(" st:");
    Serial.print(curr_state[0]);
    Serial.print(" subst:");
    Serial.print(substate);
    Serial.print(" output:");
    Serial.print(output);
    Serial.print(" LF_cal:[");
    Serial.print(sensorCalVal[0]); // the left-most sensor if facing same direction as robot
    Serial.print(",");
    Serial.print(sensorCalVal[1]);
    Serial.print(",");
    Serial.print(sensorCalVal[2]);
    Serial.print(",");
    Serial.print(sensorCalVal[3]);
    Serial.print(",");
    Serial.print(sensorCalVal[4]);
    Serial.print(",");
    Serial.print(sensorCalVal[5]);
    Serial.print(",");
    Serial.print(sensorCalVal[6]);
    Serial.print(",");
    Serial.print(sensorCalVal[7]);
    Serial.print("]");
    Serial.print(" LF_raw:[");
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

    // command states
    switch (curr_state[0])
    {
    case LINE_A:
    {
      // init function
      if (substate == 0)
      {
        enableMotor(BOTH_MOTORS);
        setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
        setMotorSpeed(BOTH_MOTORS, motorSpeed);
        substate = 1;
      }
      // main function
      else if (substate == 1)
      {
        checkIntersection();

        Serial.print(" interCnt:");
        Serial.print(interCounts);

        // exit condition check
        if (interCounts >= 2)
        {
          substate = 2;
        }

        // PID motor if on line and not on intersection
        if (!onInter && linePos != 9999)
        {
          computePID();

          // set motors based on PID and base value
          if (motorSpeed + output > 0)
          {
            setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
          }
          else
          {
            setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
          }
          if (motorSpeed - output > 0)
          {
            setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
          }
          else
          {
            setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
          }
          setMotorSpeed(LEFT_MOTOR, abs(motorSpeed + output));
          setMotorSpeed(RIGHT_MOTOR, abs(motorSpeed - output));
        }
      }
      // exit function
      else if (substate == 2)
      {
        curr_state[1] = curr_state[0];
        curr_state[0] = TURN;

        setDefaultsLFPID();
        setDefaultsIntersection();

        turnDirection = RIGHT;
        turnRotations = NINETY_DEG_TURN;

        substate = 0;
      }
    }
    case LINE_B:
    {
      // init function
      if (substate == 0)
      {
        enableMotor(BOTH_MOTORS);
        setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
        setMotorSpeed(BOTH_MOTORS, motorSpeed);
        substate = 1;
      }
      // main function
      else if (substate == 1)
      {
        checkIntersection();

        // exit condition check
        if (interCounts >= 1)
        {
          substate = 2;
        }

        // PID motor if on line and not on intersection
        if (!onInter && linePos != 9999)
        {
          computePID();

          // set motors based on PID and base value
          if (motorSpeed + output > 0)
          {
            setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
          }
          else
          {
            setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
          }
          if (motorSpeed - output > 0)
          {
            setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
          }
          else
          {
            setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
          }
          setMotorSpeed(LEFT_MOTOR, abs(motorSpeed + output));
          setMotorSpeed(RIGHT_MOTOR, abs(motorSpeed - output));
        }
      }

      // exit function
      if (substate == 2)
      {
        curr_state[1] = curr_state[0];
        curr_state[0] = TURN;

        setDefaultsLFPID();
        setDefaultsIntersection();

        turnDirection = LEFT;
        turnRotations = NINETY_DEG_TURN;

        substate = 0;
      }
    }
    case TURN:
    {
      // main function
      turning();

      // exit function
      if (substate == 2)
      {
        if (curr_state[1] == LINE_A)
        {
          curr_state[1] = curr_state[0];
          curr_state[0] = LINE_B;
        }
        else if (curr_state[1] == LINE_B)
        {
          curr_state[1] = curr_state[0];
          curr_state[0] = STOP;
        }
        substate = 0;
      }
    }
    case STOP:
    {
      disableMotor(BOTH_MOTORS);
      break;
    }
    } // end switch case
  }   // end toggle on
  else
  { // toggle off
    disableMotor(BOTH_MOTORS);
    setDefaults();
  }
  Serial.println("");
  delay(2);
} // END LOOP

void turning()
{
  // init
  if (substate == 0)
  {
    setDefaultsEncoderCnts();
    if (turnDirection == RIGHT)
    {
      setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
      setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
    }
    else
    {
      setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
      setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    }
    setMotorSpeed(BOTH_MOTORS, motorSpeed);
    substate = 1;
  }
  // main
  else if (substate == 1)
  {
    if (leftCount > turnRotations)
    {
      substate = 2;
      setMotorSpeed(BOTH_MOTORS, 0);
    }
  }
  // exit
  else if (substate == 2)
  {
  }
}

void computePID()
{
  error = setpoint - linePos;
  integral += error * dt;
  deriv = (prevError - error) / dt;
  output = kp * error + ki * integral + kd * deriv;
  prevError = error;
}

void checkIntersection()
{
  // check to see if on intersection
  for (uint8_t i = 0; i < 8; i++)
  {
    if (sensorCalVal[i] > MAX_VAL)
    {
      onInter = true;
    }
    else
    {
      onInter = false;
      break;
    }
  }

  // intersection counter
  if (onInter == true && prevInter == false)
  { // from not on line to on line
    interCounts++;
    prevInter = true;
  }
  if (onInter == false && lpButtonLastValue == 1)
  { // from on line to not on line
    prevInter = false;
  }
}

/// \brief Get line position (modified version of getLinePosition())
/// \param[in] calVal is an array that is filled with the line sensor calibrated values.
///
/// \param[in] mode determines if the line is dark or light.
/// - 0 is used when the line is darker than the floor
/// - 1 is used when the line is lighter than the floor.
///
/// \return value between 0 - 7000.
///  - 9999 no line detected
///  - ...
///  - 0000 line is directly on the left most sensor
///  - ...
///  - 3500 line directly over two middle sensors.
///  - ...
///  - 7000 is under right most line sensor
///
///  Using calibrated line sensor value this function provides a numerical value indicating
///  where the robot is detecting the line. This function can be overridden.
uint32_t getLinePosition2(uint16_t *calVal, uint8_t mode)
{
	uint32_t avg = 0; // this is for the weighted total
	uint32_t sum = 0; // this is for the denominator, which is <= 64000

	uint32_t _lastPosition;
	for (uint8_t i = 0; i < LS_NUM_SENSORS; i++)
	{
		uint16_t value = calVal[i];

		// only average in values that are above a noise threshold
		if (value > 50)
		{
			avg += (uint32_t)value * (i * 1000);
			sum += value;
		}
	}

	_lastPosition = sum!=0 ? avg / sum : 9999;
	return _lastPosition;
}