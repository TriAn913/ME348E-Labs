#include <Bump_Switch.h>
#include <Encoder.h>
#include <GP2Y0A21_Sensor.h>
#include <QTRSensors.h>
#include <Romi_Motor_Power.h>
#include <RSLK_Pins.h>
#include <SimpleRSLK.h>
/* notes:
  ideally, would change all cases into functions with pass by reference parameters or make each sensor/motor an object with its own class variables,
  but that would take more work than what is done below. The code below is non-blocking with a hard-coded delay of 2ms per loop. */

bool onToggle = false; // toggle all functions on/off (off disables all motors)
int lpButtonValue = 0; // launchpad left button
int lpButtonLastValue = 0;

int motorSpeed = 10;
static const float wheelDiameter = 2.7559055;

// Sensor groupings
int bpLeft[3] = {0, 0, 0};
int bpLeftSz = 0;
int bpRight[3] = {0, 0, 0};
int bpRightSz = 0;

// encoder counts
int leftCount = 0;
int rightCount = 0;

enum states // commanded state
{
  KEEP_DRIVING,
  AVOID_HEADON,
  AVOID_LEFTOBS,
  AVOID_RIGHTOBS,
  STOP
};

enum subStates // system status
{
  NONE,
  BACKING_UP,
  TURNING,
  MOVING_FORWARD
};

int substateComplete; // which state has been completed 0:none, 1:init, 2:reached exit condition, 3:finished exit command
int turnRotations = 0;
#define LEFT 0
#define RIGHT 0
int turnDirection = LEFT; // 0:left, 1:right
states curr_state;
subStates curr_substate;

void setup()
{
  Serial.begin(9600);

  setupRSLK();

  setupLed(RED_LED);

  pinMode(PUSH2, INPUT_PULLDOWN);       // left launchpad button
  pinMode(BP_SW_PIN_0, INPUT_PULLDOWN); // FR
  pinMode(BP_SW_PIN_1, INPUT_PULLDOWN); // R
  pinMode(BP_SW_PIN_2, INPUT_PULLDOWN); // MR
  pinMode(BP_SW_PIN_3, INPUT_PULLDOWN); // ML
  pinMode(BP_SW_PIN_4, INPUT_PULLDOWN); // L
  pinMode(BP_SW_PIN_5, INPUT_PULLDOWN); // FL

  curr_state = KEEP_DRIVING; // Initalize the current state to the default, keep driving.
  curr_substate = NONE;      // Initalize the current substate to the default, none.
  substateComplete = 0;
}
void loop()
{
  // read global sensors
  lpButtonValue = digitalRead(PUSH2);
  Serial.print("LP BTN: ");
  Serial.print(lpButtonValue);
  // check on/off button toggle
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
    bpRight[0] = digitalRead(BP_SW_PIN_0);
    bpRight[1] = digitalRead(BP_SW_PIN_1);
    bpRight[2] = digitalRead(BP_SW_PIN_2);
    bpLeft[0] = digitalRead(BP_SW_PIN_3);
    bpLeft[1] = digitalRead(BP_SW_PIN_4);
    bpLeft[2] = digitalRead(BP_SW_PIN_5);

    leftCount = getEncoderLeftCnt();
    rightCount = getEncoderRightCnt();

    /*PRINT PIN STATE FOR DEBUGGING PURPOSES*/
    Serial.print(" St: ");
    Serial.print(curr_state);
    Serial.print(" SbSt: ");
    Serial.print(curr_substate);
    Serial.print(" BP: [");
    Serial.print(bpRight[0]);
    Serial.print(",");
    Serial.print(bpRight[1]);
    Serial.print(",");
    Serial.print(bpRight[2]);
    Serial.print(",");
    Serial.print(bpLeft[0]);
    Serial.print(",");
    Serial.print(bpLeft[1]);
    Serial.print(",");
    Serial.println(bpLeft[2]);
    Serial.print("]");

    // command states
    switch (curr_state)
    {
    case KEEP_DRIVING:
    {
      if (curr_substate == NONE)
      {
        curr_substate = MOVING_FORWARD;
        substateComplete = 0;
      }

      // count bump sensor values
      bpLeftSz = bpLeft[0] + bpLeft[1] + bpLeft[2];
      bpRightSz = bpRight[0] + bpRight[1] + bpRight[2];
      // set state
      if (bpLeftSz + bpRightSz > 3)
      { // more than 3
        curr_state = STOP;
        curr_substate = NONE;
      }
      else if (bpLeftSz > 0 && bpRightSz > 0)
      { // at least 1 on both sides
        curr_state = AVOID_HEADON;
        curr_substate = NONE;
        substateComplete = 0;
      }
      else if (bpRightSz > 0)
      { // limit switch on right activated (none on left)
        curr_state = AVOID_RIGHTOBS;
        curr_substate = NONE;
        substateComplete = 0;
      }
      else if (bpLeftSz > 0)
      { // limit switch on left activated (none on right)
        curr_state = AVOID_LEFTOBS;
        curr_substate = NONE;
        substateComplete = 0;
      }
      break;
    }
    case AVOID_HEADON:
    {
      if (curr_substate == NONE)
      {
        curr_substate = BACKING_UP;
        substateComplete = 0;
      }
      else if (curr_substate == BACKING_UP)
      {
        if (substateComplete == 2)
        {
          curr_substate = TURNING;
          turnRotations = 360;
          turnDirection = LEFT;
          substateComplete = 0;
        }
      }
      else if (curr_substate == TURNING)
      {
        if (substateComplete == 2)
        {
          curr_state = KEEP_DRIVING;
          curr_substate = NONE;
          substateComplete = 0;
        }
      }
      break;
    }
    case AVOID_LEFTOBS:
    {
      if (curr_substate == NONE)
      {
        curr_substate = BACKING_UP;
        substateComplete = 0;
      }
      else if (curr_substate == BACKING_UP)
      {
        if (substateComplete == 2)
        {
          curr_substate = TURNING;
          turnRotations = 180;
          turnDirection = RIGHT;
          substateComplete = 0;
        }
      }
      else if (curr_substate == TURNING)
      {
        if (substateComplete == 2)
        {
          curr_state = KEEP_DRIVING;
          curr_substate = NONE;
          substateComplete = 0;
        }
      }
      break;
    }
    case AVOID_RIGHTOBS:
    {
      if (curr_substate == NONE)
      {
        curr_substate = BACKING_UP;
      }
      else if (curr_substate == BACKING_UP)
      {
        if (substateComplete == 2)
        {
          curr_substate = TURNING;
          turnRotations = 180;
          turnDirection = LEFT;
          substateComplete = 0;
        }
      }
      else if (curr_substate == TURNING)
      {
        if (substateComplete == 2)
        {
          curr_state = KEEP_DRIVING;
          curr_substate = NONE;
          substateComplete = 0;
        }
      }
      break;
    }
    case STOP:
    {
      disableMotor(BOTH_MOTORS);
      break;
    }
    }

    // substates (current system status)
    switch (curr_substate)
    {
    case (NONE):
    {
      break;
    }
    case (MOVING_FORWARD):
    {
      if (substateComplete == 0)
      {
        enableMotor(BOTH_MOTORS);
        setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
        setMotorSpeed(BOTH_MOTORS, motorSpeed);
        substateComplete = 1;
      }
      break;
    }
    case (BACKING_UP):
    {
      if (substateComplete == 0)
      {
        resetLeftEncoderCnt();
        resetRightEncoderCnt();
        leftCount = 0;
        rightCount = 0;
        enableMotor(BOTH_MOTORS);
        setMotorDirection(BOTH_MOTORS, MOTOR_DIR_BACKWARD);
        setMotorSpeed(BOTH_MOTORS, motorSpeed);
        substateComplete = 1;
      }
      if (leftCount > 180)
      {
        substateComplete = 2;
        setMotorSpeed(BOTH_MOTORS, 0);
      }
      break;
    }
    case (TURNING):
    {
      if (substateComplete == 0)
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
        substateComplete = 1;
      }
      if (leftCount > turnRotations)
      {
        substateComplete = 2;
        setMotorSpeed(BOTH_MOTORS, 0);
      }
      break;
    }
    }
  } // end toggle on
  else
  { // toggle off
    disableMotor(BOTH_MOTORS);
    curr_state = KEEP_DRIVING; // Initalize the current state to the default, keep driving.
    curr_substate = NONE;      // Initalize the current substate to the default, none.
  }
  Serial.println("");
  delay(2);
} // END LOOP
