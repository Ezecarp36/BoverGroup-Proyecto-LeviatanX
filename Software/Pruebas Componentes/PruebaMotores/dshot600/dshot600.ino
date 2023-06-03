#include <Arduino.h>
#include "DShotRMT.h"

const auto MOTOR01_PIN = GPIO_NUM_32;
const auto DSHOT_MODE = DSHOT600;

DShotRMT motor01(MOTOR01_PIN, RMT_CHANNEL_0);

void setup()
{
  motor01.begin(DSHOT_MODE);
  pinMode(MOTOR01_PIN, OUTPUT);
}

void loop()
{
    motor01.sendThrottleValue(1300);
}

