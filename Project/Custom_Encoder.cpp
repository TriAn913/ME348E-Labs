#include "Custom_Encoder.h"
#include <stdlib.h>
#include <ti/drivers/GPIO.h>
Custom_Encoder::Custom_Encoder()
{
    configured = false;
}

void Custom_Encoder::begin(uint8_t ea_pin, uint8_t eb_pin)
{
    _ea_pin = ea_pin;
    _eb_pin = eb_pin;

    /* Encoder drives pins high during pulse so by default they should be pulled low */
    pinMode(_ea_pin, INPUT_PULLDOWN);
    pinMode(_eb_pin, INPUT_PULLDOWN);

    /* Only count the rising edge of the encoder */
    attachInterrupt2(digitalPinToInterrupt(eb_pin), this, &Custom_Encoder::Trigger_Encoder, RISING);
    configured = true;
}

int32_t Custom_Encoder::getEncoderCnt()
{
    return enc_cnt;
}

void Custom_Encoder::resetEncoderCnt()
{
    enc_cnt = 0;
}

void Custom_Encoder::Trigger_Encoder()
{
    enc_dir = digitalRead(_ea_pin);

    if (enc_dir == 1)
    {
        enc_cnt++;
    }
    else
    {
        enc_cnt--;
    }
}

void attachInterrupt2(uint8_t pin, Custom_Encoder *a, void (Custom_Encoder::*userFunc)(void), int mode)
{
    GPIO_PinConfig intType;

    switch (mode)
    {
    case LOW:
        intType = GPIO_CFG_IN_INT_LOW;
        break;
    case CHANGE:
        intType = GPIO_CFG_IN_INT_BOTH_EDGES;
        break;
    case RISING:
        intType = GPIO_CFG_IN_INT_RISING;
        break;
    case FALLING:
        intType = GPIO_CFG_IN_INT_FALLING;
        break;
    case HIGH:
        intType = GPIO_CFG_IN_INT_HIGH;
        break;
    }

    GPIO_setConfig(pin, GPIO_CFG_IN_INT_ONLY | intType);

    GPIO_setCallback(pin, (GPIO_CallbackFxn)(a->*userFunc));

    GPIO_enableInt(pin);
}