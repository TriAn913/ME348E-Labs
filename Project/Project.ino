#include <Bump_Switch.h>
#include <Encoder.h>
#include <GP2Y0A21_Sensor.h>
#include <QTRSensors.h>
#include <Romi_Motor_Power.h>
#include <RSLK_Pins.h>
#include "definitions.h"

logLevels logLevel = INFO;

uint16_t motorSpeed = 10;

// linefollower
uint16_t sensorVal[LS_NUM_SENSORS];                                                                    // raw sensor values
uint16_t sensorCalVal[LS_NUM_SENSORS];                                                                 // calibrated sensor values
static const uint16_t sensorMaxVal[LS_NUM_SENSORS] = {875, 858, 663, 759, 588, 797, 679, 950};         // max white sensor values
static const uint16_t sensorMinVal[LS_NUM_SENSORS] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000}; // min black sensor values
#define lineColor DARK_LINE
uint32_t linePos = 9999; // line position
uint8_t interCounts;     // number of intersections crossed
bool prevInter;          // whether was on intersection last time checked (true: was on intersection; false: was not on intersection)
bool onInter;            // is on intersection (true: on intersection; false: not on intersection)

// linefollower PID
int32_t lf_error;
int32_t lf_prevError;
#define LF_MIDDLE 3500
#define LF_DT 0.1f
float lf_integral;
int16_t lf_output;
static const float lf_kp = 1.0f / 3.5f / 100.0f;  // error of 3500 (max) returns 10
static const float lf_ki = 0;                     // not needed for this application (would be useful if needed constant power at the setpoint)
static const float lf_kd = -0.2f / 3.5f / 100.0f; // derivative constant. used to prevent oscillations

// launchpad left button (toggle on/off)
bool onToggle = 0;          // toggle all functions on/off (off disables all motors)
bool lpButtonValue = 0;     // launchpad left button
bool lpButtonLastValue = 1; // the value of the launchpad left button value the last time checked

// encoder
uint32_t leftCount;  // left motor encoder count
uint32_t rightCount; // right motor encoder count

// states
enum lvl0states // operational state
{
    NONE,
    NAVIGATE,
    FIRST_AIM,
    AIM_AND_SHOOT,
    STOP,

};
enum lvl1states // operational state
{
    NONE,
    N_,
    FA_,
    ASH_,
    ST_
};
lvl0states op_state;     // the current operational lvl0 state
lvl1states op_1state[1]; // [0]: the current operational lvl1 substate; [1]: the previous operational lvl1 substate;

// lvl3 substates. The function controls the transition out of entry and exit. The function-calling state controls the transition out of main and finished.
enum lvl3states // operational state
{
    NONE,
    ENTRY,
    MAIN,
    EXIT,
    FINISHED
};
uint8_t substate;            // active operational lvl3 substate
uint8_t lf_substate;         // active linefollower lvl3 substate
uint8_t rMotionDeg_substate; // active robot commanded motion lvl3 substate

// turn function parameters
/*
#define NINETY_DEG_TURN 170
#define ONE_EIGHTY_DEG_TURN 340
*/

void setDefaults()
{
    setDefaultsIntersection();
    setDefaultsLFPID();
    setDefaultsEncoderCnts();

    // states and substates
    op_state = lvl0states::NAVIGATE;
    op_1state[0] = lvl1states::NONE;
    op_1state[1] = lvl1states::NONE;
    substate = lvl3states::NONE;
    lf_substate = lvl3states::NONE;
}

void setDefaultsIntersection()
{
    // line follower intersection values
    interCounts = 0;
    prevInter = true;
    onInter = false;
}

void setDefaultsLFPID()
{
    // line follower PID
    lf_error = 0;
    lf_prevError = 0;
    lf_integral = 0;
    lf_output = 0;
}

void setDefaultsEncoderCnts()
{
    // encoder counts
    leftCount = 0;
    rightCount = 0;
    resetRightEncoderCnt();
    resetLeftEncoderCnt();
}

void setup()
{
    customSetupRSLK(); // distance sensors and line sensors on the default rslk kit
}

void loop()
{
    // read global sensors
    allToggle();

    if (onToggle)
    {
    }
    else
    {
        disableMotor(BOTH_MOTORS);
    }
}

/// \brief lvl4: Checks the state of the Left Launchpad button and toggles the onToggle variable.
/// Uses the global variables \c lpButtonValue , \c logLevel , \c lpButtonLastValue , \c onToggle
void allToggle()
{
    lpButtonValue = digitalRead(PUSH2);
    customLog("LLPB", logLevel, INFO, false);
    customLog(lpButtonValue, logLevel, INFO, false);

    // all toggle (Left LP Button)
    if (lpButtonValue == 1 && lpButtonLastValue == 0)
    { // button: from released to pressed
        lpButtonLastValue = 1;
        onToggle = !onToggle;
    }
    else if (lpButtonValue == 0 && lpButtonLastValue == 1)
    { // button: from pressed to released
        lpButtonLastValue = 0;
    }
}

/// \brief lvl3: Given a line position, uses computePID() to determine the drive motors' speeds and sets them.
/// Uses the global variables \c lf_substate , \c lf_prevError , \c linePos , \c lf_output , \c lf_integral , \c lf_kp
/// , \c lf_ki , \c lf_kd , and \c motorSpeed .
///
void followLine()
{
    if (lf_substate == 1)
    {
        enableMotor(BOTH_MOTORS);
        lf_substate = 2;
    }
    else if (lf_substate == 2)
    {
        computePID(LF_MIDDLE, lf_prevError, linePos, lf_output, LF_DT, lf_integral, lf_kp, lf_ki, lf_kd);

        setMotorSpeed2(LEFT_MOTOR, motorSpeed - lf_output);
        setMotorSpeed2(RIGHT_MOTOR, motorSpeed + lf_output);
    }
    else if (lf_substate == 3)
    {
        setDefaultsLFPID();
        lf_substate = 4;
    }
}

/// \brief lvl4: Performs the necessary functions to get the line position. Uses \c sensorVal , \c sensorCalVal
/// , \c sensorMinVal , \c sensorMaxVal , and \c linePos .
void readLineSensors()
{
    readLineSensor(sensorVal);
    readCalLineSensor(sensorVal, sensorCalVal, sensorMinVal, sensorMaxVal, lineColor);
    linePos = getLinePosition(sensorCalVal, lineColor);
}

/// \brief lvl3: Use to control robot motion (point turns and straight translation).
///
/// Turns the drive motors a given rotation (degrees of wheel turn) and sets the drive motor direction based on the desired robot movement
/// (forward, backward, turn right, turn left). Uses \c
///
/// \param wheelDeg the commanded angle the wheel needs to turn (degrees). Use robotDeg2WheelDeg() to determine the wheel rotation based on the desired robot turn angle.
/// Use inches2WheelDeg() to determine the wheel rotation based on the desired distance in inches.
///
/// \param robotDirection the direction in which to turn (TURN_LEFT (0) or TURN_RIGHT (1)) or move (ROBOT_FORWARD (2) or ROBOT_BACKWARD (3))
void robotMotionDeg(uint32_t wheelDeg, bool robotDirection = ROBOT_FORWARD)
{
    // init
    if (substate == 0)
    {
        setDefaultsEncoderCnts();

        switch (robotDirection)
        {
        case (ROBOT_FORWARD):
        {
            setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
            setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
            break;
        }
        case (ROBOT_TURN_RIGHT):
        {
            setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
            setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
            break;
        }
        case (ROBOT_TURN_LEFT):
        {
            setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
            setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
            break;
        }
        case (ROBOT_BACKWARD):
        {
            setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
            setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
            break;
        }
        }
        setMotorSpeed(BOTH_MOTORS, motorSpeed);
        substate = 1;
    }
    // main
    else if (substate == 1)
    {
        if (leftCount >= wheelDeg)
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