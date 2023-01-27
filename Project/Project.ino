#include <Bump_Switch.h>
#include <Encoder.h>
#include <GP2Y0A21_Sensor.h>
#include <QTRSensors.h>
#include <Romi_Motor_Power.h>
#include <RSLK_Pins.h>
#include "definitions.h"
#include "Custom_Encoder.h"

void setup()
{
    Custom_Encoder encoder[2];
    encoder[0].begin(ENCODER_ELA_PIN, ENCODER_ELB_PIN); // left wheel encoder on the rslk kit
    encoder[1].begin(ENCODER_ERA_PIN, ENCODER_ERB_PIN); // right wheel encoder on the rslk kit

    customSetupRSLK(); // distance sensors and line sensors on the default rslk kit
}

void loop()
{

}