#ifndef _FAN_H_
#define _FAN_H_
#include <stdio.h>
#include "driver/ledc.h"
#include "esp_err.h"

enum{
    FAN_LEVEL_0,
    FAN_LEVEL_1,
    FAN_LEVEL_2,
    FAN_LEVEL_3,
};

void motor_set_speed(int duty);
void motor_init();
#endif