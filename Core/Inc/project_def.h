#ifndef PROJECT_DEF_H
#define PROJECT_DEF_H

// stm32 include
#include "main.h"

/* test macro ----------------------------------------------------------------*/
// #define ENABLE_WATCH_DOG

// #define MAX_POWER_TEST
// #define LIFT_TEST
// #define GO_STRAIGHT_TEST
// #define PATH_TRACK_TEST
// #define BUTTON_TEST
// #define COMPONENT_TEST
// #define WRITE_EEPROM_TEST

//#define PRINT_RAW_ANALOG_DATA
// #define PRINT_FILTERED_ANALOG_DATA

#define SIMPLE_PATH_TRACK
// #define COMPLEX_PATH_TRACK_TEST

 #define ENABLE_ALARM

/* project config ------------------------------------------------------------*/
#define USER_TASK_STACK_SIZE 128
#define MPU6050_TASK_STACK_SIZE 512

// analog sensor
#define NUM_ANALOG 6
#define ANALOG_FL2 0
#define ANALOG_FL1 4
#define ANALOG_FR1 5
#define ANALOG_FR2 3
#define ANALOG_RL 1
#define ANALOG_RR 2

#define ANALOG_LOWER_THRESHOLD 0.50F

// speed control
#define SERVO_SPEED_STOP 0.0F
#define SERVO_SPEED_MINIMUM -0.1F
#define SERVO_SPEED_MAXIMUM 0.15F
#define SERVO_SPEED_REAGE (SERVO_SPEED_MAXIMUM - SERVO_SPEED_MINIMUM)

// eeprom
#define EEPROM_NORMALIZE_FILTER_SIZE NUM_ANALOG * 2
#define EEPROM_SIZE (EEPROM_NORMALIZE_FILTER_SIZE)
#define EEPROM_NORMALIZE_FILTER_START_ADDRESS 0
#define EEPROM_NORMALIZE_FILTER_LOWER_BOUND_ADDRESS(x) \
  (EEPROM_NORMALIZE_FILTER_START_ADDRESS + 2 * (x))
#define EEPROM_NORMALIZE_FILTER_UPPER_BOUND_ADDRESS(x) \
  (EEPROM_NORMALIZE_FILTER_LOWER_BOUND_ADDRESS(x) + 1)

/* module config -------------------------------------------------------------*/
// button
#define NUM_BUTTON_BUILTIN 1
#define NUM_BUTTON_USER 3
#define NUM_BUTTON_MICRO 4
#define NUM_BUTTON (NUM_BUTTON_BUILTIN + NUM_BUTTON_USER + NUM_BUTTON_MICRO)

#define BUTTON_BUILTIN 0

#define BUTTON(X) (BUTTON_BASE + X)
#define BUTTON_BASE (BUTTON_BUILTIN + NUM_BUTTON_BUILTIN)

#define BUTTON1 BUTTON(0)
#define BUTTON2 BUTTON(1)
#define BUTTON3 BUTTON(2)

#define BUTTON_MICRO_BASE (BUTTON_BASE + NUM_BUTTON_USER)
#define BUTTON_MICRO(X) (BUTTON_MICRO_BASE + X)

#define BUTTON_MICRO_FB BUTTON_MICRO(0)
#define BUTTON_MICRO_FT BUTTON_MICRO(1)
#define BUTTON_MICRO_RB BUTTON_MICRO(2)
#define BUTTON_MICRO_RT BUTTON_MICRO(3)

// moving average filter
#define MOVING_AVERAGE_FILTER_MOVING_AVERAGE_SIZE 10
#define NUM_MOVING_AVERAGE_FILTER NUM_ANALOG

#define MOVING_AVERAGE_FILTER_ANALOG_FL1 ANALOG_FL1
#define MOVING_AVERAGE_FILTER_ANALOG_FL2 ANALOG_FL2
#define MOVING_AVERAGE_FILTER_ANALOG_FR1 ANALOG_FR1
#define MOVING_AVERAGE_FILTER_ANALOG_FR2 ANALOG_FR2
#define MOVING_AVERAGE_FILTER_ANALOG_RL ANALOG_RL
#define MOVING_AVERAGE_FILTER_ANALOG_RR ANALOG_RR

// normalize filter
#define NUM_NORMALIZE_FILTER NUM_ANALOG

#define NORMALIZE_FILTER_ANALOG_FL1 ANALOG_FL1
#define NORMALIZE_FILTER_ANALOG_FL2 ANALOG_FL2
#define NORMALIZE_FILTER_ANALOG_FR1 ANALOG_FR1
#define NORMALIZE_FILTER_ANALOG_FR2 ANALOG_FR2
#define NORMALIZE_FILTER_ANALOG_RL ANALOG_RL
#define NORMALIZE_FILTER_ANALOG_RR ANALOG_RR

// led
#define NUM_LED_BUILTIN 1
#define NUM_LED_USER 3
#define NUM_LED (NUM_LED_BUILTIN + NUM_LED_USER)

#define LED_BUILTIN 0

#define LED_BASE (LED_BUILTIN + NUM_LED_BUILTIN)
#define LED(LED) (LED_BASE + LED)

#define LED_GREEN LED(0)
#define LED_YELLOW LED(1)
#define LED_RED LED(2)

// servo
#define NUM_SERVO_WHEEL 4
#define NUM_SERVO (NUM_SERVO_WHEEL)

#define SERVO_WHEEL_FL 0
#define SERVO_WHEEL_FR 1
#define SERVO_WHEEL_RL 2
#define SERVO_WHEEL_RR 3

// motor
#define NUM_MOTOR 4
#define MOTOR_FL 0
#define MOTOR_FR 1
#define MOTOR_RL 2
#define MOTOR_RR 3

/* gpio config ---------------------------------------------------------------*/
// button
#define BUTTON_BUILTIN_PORT B1_GPIO_Port
#define BUTTON_BUILTIN_PIN B1_Pin

#define BUTTON1_PORT GPIOB
#define BUTTON1_PIN GPIO_PIN_1
#define BUTTON2_PORT GPIOB
#define BUTTON2_PIN GPIO_PIN_2
#define BUTTON3_PORT GPIOB
#define BUTTON3_PIN GPIO_PIN_6

// microswitch
#define MICRO_FB_PORT GPIOC
#define MICRO_FB_PIN GPIO_PIN_11
#define MICRO_FT_PORT GPIOC
#define MICRO_FT_PIN GPIO_PIN_10
#define MICRO_RB_PORT GPIOD
#define MICRO_RB_PIN GPIO_PIN_2
#define MICRO_RT_PORT GPIOC
#define MICRO_RT_PIN GPIO_PIN_12

// led
#define LED_BUILTIN_PORT LD2_GPIO_Port
#define LED_BUILTIN_PIN LD2_Pin

#define LED_GREEN_PORT GPIOB
#define LED_GREEN_PIN GPIO_PIN_10
#define LED_YELLOW_PORT GPIOB
#define LED_YELLOW_PIN GPIO_PIN_5
#define LED_RED_PORT GPIOB
#define LED_RED_PIN GPIO_PIN_4

// motor
#define MOTOR_FL_1_PORT GPIOA
#define MOTOR_FL_1_PIN GPIO_PIN_4
#define MOTOR_FL_2_PORT GPIOB
#define MOTOR_FL_2_PIN GPIO_PIN_0
#define MOTOR_FR_1_PORT GPIOB
#define MOTOR_FR_1_PIN GPIO_PIN_13
#define MOTOR_FR_2_PORT GPIOC
#define MOTOR_FR_2_PIN GPIO_PIN_4
#define MOTOR_RL_1_PORT GPIOA
#define MOTOR_RL_1_PIN GPIO_PIN_1
#define MOTOR_RL_2_PORT GPIOA
#define MOTOR_RL_2_PIN GPIO_PIN_0
#define MOTOR_RR_1_PORT GPIOB
#define MOTOR_RR_1_PIN GPIO_PIN_14
#define MOTOR_RR_2_PORT GPIOB
#define MOTOR_RR_2_PIN GPIO_PIN_15

/* pwm config ----------------------------------------------------------------*/
#define SERVO_WHEEL_FL_TIM &htim1
#define SERVO_WHEEL_FL_TIM_CHANNEL TIM_CHANNEL_1
#define SERVO_WHEEL_FR_TIM &htim1
#define SERVO_WHEEL_FR_TIM_CHANNEL TIM_CHANNEL_3
#define SERVO_WHEEL_RL_TIM &htim3
#define SERVO_WHEEL_RL_TIM_CHANNEL TIM_CHANNEL_4
#define SERVO_WHEEL_RR_TIM &htim3
#define SERVO_WHEEL_RR_TIM_CHANNEL TIM_CHANNEL_3
#define SERVO_REV1_TIM &htim1
#define SERVO_REV1_TIM_CHANNEL TIM_CHANNEL_2
#define SERVO_REV2_TIM &htim3
#define SERVO_REV2_TIM_CHANNEL TIM_CHANNEL_2
#define SERVO_REV3_TIM &htim3
#define SERVO_REV3_TIM_CHANNEL TIM_CHANNEL_1
#define SERVO_REV4_TIM &htim1
#define SERVO_REV4_TIM_CHANNEL TIM_CHANNEL_4

#endif  // PROJECT_DEF_H
