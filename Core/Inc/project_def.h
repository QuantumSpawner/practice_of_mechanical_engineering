#ifndef PROJECT_DEF_H
#define PROJECT_DEF_H

// stm32 include
#include "main.h"
#include "stm32f4xx_hal.h"

/* test macro ----------------------------------------------------------------*/
#define PRINT_GRAYSCALE_DATA

/* gpio config ---------------------------------------------------------------*/
// microswitch
#define MICROSWITCHS_FB_PORT GPIOC
#define MICROSWITCHS_FB_PIN GPIO_PIN_0
#define MICROSWITCHS_FT_PORT GPIOC
#define MICROSWITCHS_FT_PIN GPIO_PIN_1
#define MICROSWITCHS_RB_PORT GPIOC
#define MICROSWITCHS_RB_PIN GPIO_PIN_2
#define MICROSWITCHS_RT_PORT GPIOC
#define MICROSWITCHS_RT_PIN GPIO_PIN_3

// motor
#define MOTOR_FRONT_1 GPIOB
#define MOTOR_FRONT_1_PIN GPIO_PIN_12
#define MOTOR_FRONT_2 GPIOB
#define MOTOR_FRONT_2_PIN GPIO_PIN_13
#define MOTOR_REAR_1 GPIOB
#define MOTOR_REAR_1_PIN GPIO_PIN_14
#define MOTOR_REAR_2 GPIOB
#define MOTOR_REAR_2_PIN GPIO_PIN_15

/* pwm config ----------------------------------------------------------------*/
// servo
#define SERVO_FL_TIM &htim3
#define SERVO_FL_TIM_CHANNEL TIM_CHANNEL_1
#define SERVO_FR_TIM &htim3
#define SERVO_FR_TIM_CHANNEL TIM_CHANNEL_2
#define SERVO_RL_TIM &htim3
#define SERVO_RL_TIM_CHANNEL TIM_CHANNEL_3
#define SERVO_RR_TIM &htim3
#define SERVO_RR_TIM_CHANNEL TIM_CHANNEL_4

/* adc config ----------------------------------------------------------------*/
#define ADC1_BUFFER_SIZE 2

/* module config -------------------------------------------------------------*/
// button
#define NUM_INFRARED 5
#define NUM_MICROSWITCH 4
#define NUM_BUTTON 1 + NUM_INFRARED + NUM_MICROSWITCH
#define BUTTON_USER 0

#define MICROSWITCH_FB 1
#define MICROSWITCH_FT 2
#define MICROSWITCH_RB 3
#define MICROSWITCH_RT 4

// grayscale sensor (filters)
#define NUM_GRAYSCALE 2
#define GRAYSCALE_LEFT 0
#define GRAYSCALE_RIGHT 1

#define GRAYSCALE_MOVING_AVERAGE_SIZE 10
#define GRAYSCALE_LEFT_LOWER_BOUND 0.581F
#define GRAYSCALE_LEFT_UPPER_BOUND 0.683F
#define GRAYSCALE_RIGHT_LOWER_BOUND 0.755F
#define GRAYSCALE_RIGHT_UPPER_BOUND 0.831F
#define GRAYSCALE_LOWER_THRESHOLD 0.50F

// led
#define NUM_LED 1
#define LED_USER 0

// motor
#define NUM_MOTOR 2
#define MOTOR_FRONT 0
#define MOTOR_REAR 1

// servo
#define NUM_SERVO 4
#define SERVO_FL 0
#define SERVO_FR 1
#define SERVO_RL 2
#define SERVO_RR 3

/* other config --------------------------------------------------------------*/
#define USER_TASK_STACK_SIZE 1024

// speed control
#define SPEED_STOP 0.0F
#define SPEED_MINIMUM 0.0F
#define SPEED_MAXIMUM 0.4F
#define SPEED_RANGE (SPEED_MAXIMUM - SPEED_MINIMUM)

#endif  // PROJECT_DEF_H
