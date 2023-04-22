#ifndef USER_MAIN_H
#define USER_MAIN_H

// glibc include
#include <stdbool.h>

// stm32 include
#include "main.h"

/* init function -------------------------------------------------------------*/
void user_init(void);

void button_init(void);

void filter_init(void);

void led_init(void);

void servo_init(void);

void motor_init(void);

/* task code -----------------------------------------------------------------*/
void user_task_code(void *argument);

void max_power_test(void *argument);

void max_power_button_callback(void *argument, const GPIO_PinState state);

void lift_test(void *argument);

void lift_button_callback(void *_argument, const GPIO_PinState state);

void path_find_test(void *argument);

void path_find_button_callback(void *argument, const GPIO_PinState state);

void print_test(void *argument);

#endif  // USER_MAIN_H
