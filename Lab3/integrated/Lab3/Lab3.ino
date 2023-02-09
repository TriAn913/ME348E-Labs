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
bool lpButtonValue;    // launchpad left button
bool lpButtonLastValue;

uint16_t motorSpeed = 10;
// static const float wheelDiameter = 2.7559055;

// encoder counts
int leftCount;
int rightCount;

// line follower
uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS];
uint16_t sensorMinVal[LS_NUM_SENSORS];
uint8_t lineColor = DARK_LINE;
uint32_t linePos = 0;
uint8_t interCounts; // number of intersections counted
bool prevInter;      // false: was not on intersection last loop, true: was on intersection last loop
bool onInter;
#define MAX_VAL 900
#define MIN_VAL 1100

// linefollower PID
double error;
double prevError;
static const double setpoint = 3500;
static const double dt = 0.1;
double integral;
double deriv;
double output;
double kp, ki, kd;

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
int substate; // which substate is active; 0:none, 1:init, 2:main, 3:exit, 4:finished

int turnRotations;
#define LEFT 0
#define RIGHT 1
#define NINETY_DEG_TURN 180
#define ONE_EIGHTY_DEG_TURN 360
uint8_t turnDirection; // 0:left, 1:right

void setDefaults()
{
  // lp button values
  lpButtonValue = 0; // launchpad left button
  lpButtonLastValue = 0;

  // line follower values
  interCounts = 0;
  prevInter = false;
  onInter = false;

  // line follower PID
  error = 0;
  prevError = 0;
  integral = 0;
  deriv = 0;
  output = 0;
  kp, ki, kd = 1 / 7 / 20, 0, 0.1 / 7 / 20;

  // encoder counts
  leftCount = 0;
  rightCount = 0;

  // turning
  turnRotations = 0;
  turnDirection = LEFT; // 0:left, 1:right

  // states and substates
  curr_state[0] = LINE_A;
  curr_state[1] = NONE;
  substate = 0;
}

void setup()
{
  Serial.begin(9600);

  setupRSLK();

  setupLed(RED_LED);

  pinMode(PUSH2, INPUT_PULLDOWN); // left launchpad button

  for (int x = 0; x < LS_NUM_SENSORS; x++)
  {
    sensorMinVal[x] = MIN_VAL;
    sensorMaxVal[x] = MAX_VAL;
  }

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

    linePos = getLinePosition(sensorCalVal, lineColor);
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

        computePID();

        // set motors based on PID and base value
        setMotorSpeed(LEFT_MOTOR, motorSpeed + output);
        setMotorSpeed(RIGHT_MOTOR, motorSpeed - output);
      }

      // exit function
      else if (substate == 2)
      {
        curr_state[1] = curr_state[0];
        curr_state[0] = TURN;

        interCounts = 0;
        error = 0;
        prevError = 0;
        integral = 0;
        deriv = 0;
        output = 0;

        prevInter = false;
        onInter = false;

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

        computePID();

        // set motors based on PID and base value
        setMotorSpeed(LEFT_MOTOR, motorSpeed + output);
        setMotorSpeed(RIGHT_MOTOR, motorSpeed - output);
      }

      // exit function
      if (substate == 2)
      {
        curr_state[1] = curr_state[0];
        curr_state[0] = TURN;

        interCounts = 0;
        error = 0;
        prevError = 0;
        integral = 0;
        deriv = 0;
        output = 0;

        prevInter = false;
        onInter = false;

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
    resetLeftEncoderCnt();
    resetRightEncoderCnt();
    leftCount = 0;
    rightCount = 0;
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
  for (int i = 0; i < 8; i++)
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