#include <Bump_Switch.h>
#include <Encoder.h>
#include <GP2Y0A21_Sensor.h>
#include <QTRSensors.h>
#include <Romi_Motor_Power.h>
#include <RSLK_Pins.h>
#include "definitions.h"


logLevels logLevel = NONE;


void setup()
{
    

    customSetupRSLK(); // distance sensors and line sensors on the default rslk kit
}

void loop()
{
    customLog(encoder[0].getEncoderCnt(), logLevel, DEBUG, false);
}