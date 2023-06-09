#ifndef USER_MAIN_H
#define USER_MAIN_H

// glibc include
#include <stdbool.h>

// stm32 include
#include "main.h"

// stm32_module include
#include "stm32_module/stm32_module.h"

// project include
#include "simple_motor_controller.h"

/* exported variable ---------------------------------------------------------*/
extern ButtonMonitor button_monitor;

extern LedController led_controller;

extern ServoController servo_controller;

extern MotorController motor_controller;

/* entry point ---------------------------------------------------------------*/
void user_init(void);

#endif  // USER_MAIN_H
