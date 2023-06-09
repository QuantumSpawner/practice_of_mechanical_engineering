#include "user_main.h"

// glibc include
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

// stm32 include
#include "main.h"

// freertos include
#include "FreeRTOS.h"
#include "task.h"

// mpu6050 include
#include "driver_mpu6050_dmp.h"

// stm32_module include
#include "stm32_module/stm32_module.h"

// project include
#include "eeprom.h"
#include "filter.h"
#include "path_track.h"
#include "project_def.h"
#include "simple_motor_controller.h"

/* module --------------------------------------------------------------------*/
// button
ButtonMonitor button_monitor;
static struct button_cb button_cb[NUM_BUTTON];

// led
LedController led_controller;
static struct led_cb led_cb[NUM_LED];

// servo
ServoController servo_controller;
static struct servo_cb servo_cb[NUM_SERVO];

// motor
MotorController motor_controller;
static struct motor_cb motor_cb[NUM_MOTOR];

// freertos task
static TaskHandle_t user_task_handle;
static TaskHandle_t running_indicator_task_handle;
static TaskHandle_t mpu6050_task_handle;

static StaticTask_t user_task_cb;
static StaticTask_t running_indicator_task_cb;
static StaticTask_t mpu6050_task_cb;

static StackType_t user_task_stack[USER_TASK_STACK_SIZE];
static StackType_t running_indicator_task_stack[configMINIMAL_STACK_SIZE];
static StackType_t mpu6050_task_stack[MPU6050_TASK_STACK_SIZE];

// control flags
static volatile bool is_45_deg = false, is_90_deg = false;

/* static function prototype -------------------------------------------------*/
#if defined(MAX_POWER_TEST)
static void max_power_test(void *argument);
static void max_power_button_callback(void *argument,
                                      const GPIO_PinState state);
#elif defined(LIFT_TEST)
static void lift_test(void *argument);
static void lift_button_callback(void *_argument, const GPIO_PinState state);
#elif defined(GO_STRAIGHT_TEST)
static void go_straight_test(void *argument);
static void go_straight_button_callback(void *argument,
                                        const GPIO_PinState state);
#elif defined(PATH_TRACK_TEST)
static void path_track_test(void *argument);
static void path_track_button_callback(void *argument,
                                       const GPIO_PinState state);
#elif defined(BUTTON_TEST)
static void button_test(void *argument);
static void button_test_button_callback(void *_argument,
                                        const GPIO_PinState state);
#elif defined(COMPONENT_TEST)
static void component_test(void *argument);
static void component_test_button_callback(void *_argument,
                                           const GPIO_PinState state);
#elif defined(PRINT_TEST)
static void print_test(void *argument);
#else
static void user_task_code(void *argument);
static void user_button_callback(void *_argument, const GPIO_PinState state);
static void stop_wings_button_callback(void *_argument,
                                       const GPIO_PinState state);
static void operate_wings_button_callback(void *_argument,
                                          const GPIO_PinState state);
#endif

static void running_indicator_task_code(void *argument);
static void mpu6050_task_code(void *argument);

static void button_init();
static void led_init();
static void servo_init();
static void motor_init();

static void a_receive_callback(uint8_t type);

/* function ------------------------------------------------------------------*/
void user_init() {
  // module init
  if (HAL_FLASH_Unlock() != HAL_OK) {
    Error_Handler();
  }
  if (EE_Init() != HAL_OK) {
    Error_Handler();
  }
  if (mpu6050_dmp_init(0x68, a_receive_callback, NULL, NULL) != 0) {
    Error_Handler();
  }
  button_init();
  filter_init();
  led_init();
  servo_init();
  motor_init();

  // task start
#if defined(MAX_POWER_TEST)
  user_task_handle = xTaskCreateStatic(
      max_power_test, "max_power_test", USER_TASK_STACK_SIZE, NULL,
      TaskPriorityNormal, user_task_stack, &user_task_cb);
#elif defined(LIFT_TEST)
  user_task_handle =
      xTaskCreateStatic(lift_test, "lift_test", USER_TASK_STACK_SIZE, NULL,
                        TaskPriorityNormal, user_task_stack, &user_task_cb);
#elif defined(GO_STRAIGHT_TEST)
  user_task_handle = xTaskCreateStatic(
      go_straight_test, "go_straight_test", USER_TASK_STACK_SIZE, NULL,
      TaskPriorityNormal, user_task_stack, &user_task_cb);
#elif defined(PATH_TRACK_TEST)
  user_task_handle = xTaskCreateStatic(
      path_track_test, "path_track_test", USER_TASK_STACK_SIZE, NULL,
      TaskPriorityNormal, user_task_stack, &user_task_cb);
#elif defined(BUTTON_TEST)
  user_task_handle =
      xTaskCreateStatic(button_test, "button_test", USER_TASK_STACK_SIZE, NULL,
                        TaskPriorityNormal, user_task_stack, &user_task_cb);
#elif defined(COMPONENT_TEST)
  user_task_handle = xTaskCreateStatic(
      component_test, "component_test", USER_TASK_STACK_SIZE, NULL,
      TaskPriorityNormal, user_task_stack, &user_task_cb);
#elif defined(PRINT_TEST)
  user_task_handle =
      xTaskCreateStatic(print_test, "print_test", USER_TASK_STACK_SIZE, NULL,
                        TaskPriorityNormal, user_task_stack, &user_task_cb);
#elif !defined(WRITE_EEPROM_TEST)
  user_task_handle = xTaskCreateStatic(
      user_task_code, "user_task_code", USER_TASK_STACK_SIZE, NULL,
      TaskPriorityNormal, user_task_stack, &user_task_cb);
#endif

  running_indicator_task_handle = xTaskCreateStatic(
      running_indicator_task_code, "running_indicator_task_code",
      configMINIMAL_STACK_SIZE, NULL, TaskPriorityNormal,
      running_indicator_task_stack, &running_indicator_task_cb);
  mpu6050_task_handle = xTaskCreateStatic(
      mpu6050_task_code, "mpu6050_task_code", MPU6050_TASK_STACK_SIZE, NULL,
      TaskPriorityNormal, mpu6050_task_stack, &mpu6050_task_cb);
}

/* max_power_test ------------------------------------------------------------*/
#if defined(MAX_POWER_TEST)
static void max_power_test(void *argument) {
  (void)argument;

  vTaskDelete(NULL);
}

static void max_power_button_callback(void *argument,
                                      const GPIO_PinState state) {
  (void)argument;

  if (state == GPIO_PIN_RESET) {
    LedController_turn_on(&led_controller, LED_BUILTIN);
    ServoController_set_duty(&servo_controller, SERVO_WHEEL_FL, 1);
    ServoController_set_duty(&servo_controller, SERVO_WHEEL_FR, 1);
    ServoController_set_duty(&servo_controller, SERVO_WHEEL_RL, 1);
    ServoController_set_duty(&servo_controller, SERVO_WHEEL_RR, 1);

    MotorController_set(&motor_controller, MOTOR_FL, MotorForward);
    MotorController_set(&motor_controller, MOTOR_FR, MotorForward);
    MotorController_set(&motor_controller, MOTOR_RL, MotorForward);
    MotorController_set(&motor_controller, MOTOR_RR, MotorForward);
  } else {
    LedController_turn_off(&led_controller, LED_BUILTIN);
    ServoController_set_duty(&servo_controller, SERVO_WHEEL_FL, 0);
    ServoController_set_duty(&servo_controller, SERVO_WHEEL_FR, 0);
    ServoController_set_duty(&servo_controller, SERVO_WHEEL_RL, 0);
    ServoController_set_duty(&servo_controller, SERVO_WHEEL_RR, 0);

    MotorController_set(&motor_controller, MOTOR_FL, MotorStop);
    MotorController_set(&motor_controller, MOTOR_FR, MotorStop);
    MotorController_set(&motor_controller, MOTOR_RL, MotorStop);
    MotorController_set(&motor_controller, MOTOR_RR, MotorStop);
  }
}

#elif defined(LIFT_TEST)
/* lift_test -----------------------------------------------------------------*/
static void lift_test(void *argument) {
  (void)argument;

  vTaskDelete(NULL);
}

static void operate_wings_button_callback(void *_argument,
                                          const GPIO_PinState state) {
  uint32_t argument = (uint32_t)_argument;

  GPIO_PinState button_state;
  switch (argument) {
    case BUTTON1:
      if (state == GPIO_PIN_SET) {
        ButtonMonitor_read_state(&button_monitor, BUTTON_MICRO_FB,
                                 &button_state);
        if (button_state != GPIO_PIN_SET) {
          MotorController_set(&motor_controller, MOTOR_RR, MotorForward);
        }
      } else {
        MotorController_set(&motor_controller, MOTOR_RR, MotorStop);
      }
      break;

    case BUTTON2:
      if (state == GPIO_PIN_SET) {
        ButtonMonitor_read_state(&button_monitor, BUTTON_MICRO_FT,
                                 &button_state);
        if (button_state != GPIO_PIN_SET) {
          MotorController_set(&motor_controller, MOTOR_RR, MotorBackward);
        }
      } else {
        MotorController_set(&motor_controller, MOTOR_RR, MotorStop);
      }
      break;

    default:
      break;
  }
}

static void stop_wings_button_callback(void *_argument,
                                       const GPIO_PinState state) {
  uint32_t argument = (uint32_t)_argument;

  switch (argument) {
    case BUTTON_MICRO_FB:
      if (state == GPIO_PIN_SET) {
        MotorController_set(&motor_controller, MOTOR_RR, MotorStop);
      }
      break;

    case BUTTON_MICRO_FT:
      if (state == GPIO_PIN_SET) {
        MotorController_set(&motor_controller, MOTOR_RR, MotorStop);
      }
      break;

    default:
      break;
  }
}

#elif defined(GO_STRAIGHT_TEST)
/* go_straight_test ----------------------------------------------------------*/
static volatile bool start_go_straight_test = false;
static void go_straight_test(void *argument) {
  // wait for button to be pressed
  xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);

  LedController_turn_on(&led_controller, LED_BUILTIN);
  ServoController_set_duty(&servo_controller, SERVO_WHEEL_FL,
                           SERVO_SPEED_MAXIMUM);
  ServoController_set_duty(&servo_controller, SERVO_WHEEL_FR,
                           SERVO_SPEED_MAXIMUM);
  ServoController_set_duty(&servo_controller, SERVO_WHEEL_RL,
                           SERVO_SPEED_MAXIMUM);
  ServoController_set_duty(&servo_controller, SERVO_WHEEL_RR,
                           SERVO_SPEED_MAXIMUM);

  vTaskDelete(NULL);
}
static void go_straight_button_callback(void *argument,
                                        const GPIO_PinState state) {
  (void)argument;

  if (state == GPIO_PIN_RESET) {
    xTaskNotify(user_task_handle, 0, eNoAction);
  }
}

#elif defined(PATH_TRACK_TEST)
/* path_track_test -----------------------------------------------------------*/
static volatile bool start_path_find_test = false;
void path_track_test(void *argument) {
  (void)argument;

  // wait for button to be pressed
  xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);

  LedController_turn_on(&led_controller, LED_BUILTIN);
  while (!is_90_deg) {
    path_track_once();

    vTaskDelay(100);
  }

  vTaskDelete(NULL);
}

void path_track_button_callback(void *argument, const GPIO_PinState state) {
  (void)argument;

  if (state == GPIO_PIN_RESET) {
    xTaskNotify(user_task_handle, 0, eNoAction);
  }
}

#elif defined(BUTTON_TEST)
/* button_test ---------------------------------------------------------------*/
static void button_test(void *argument) {
  (void)argument;

  vTaskDelete(NULL);
}

static void button_test_button_callback(void *_argument,
                                        const GPIO_PinState state) {
  uint32_t argument = (uint32_t)_argument;

  switch (argument) {
    case BUTTON_BUILTIN:
      if (state == GPIO_PIN_RESET) {
        LedController_turn_on(&led_controller, LED_BUILTIN);
      } else {
        LedController_turn_off(&led_controller, LED_BUILTIN);
      }
      break;

    case BUTTON1:
      if (state == GPIO_PIN_SET) {
        LedController_turn_on(&led_controller, LED_GREEN);
      } else {
        LedController_turn_off(&led_controller, LED_GREEN);
      }
      break;

    case BUTTON2:
      if (state == GPIO_PIN_SET) {
        LedController_turn_on(&led_controller, LED_YELLOW);
      } else {
        LedController_turn_off(&led_controller, LED_YELLOW);
      }
      break;

    case BUTTON3:
      if (state == GPIO_PIN_SET) {
        LedController_turn_on(&led_controller, LED_RED);
      } else {
        LedController_turn_off(&led_controller, LED_RED);
      }
      break;

    default:
      break;
  }
}

#elif defined(COMPONENT_TEST)
/* component_test ------------------------------------------------------------*/
static void component_test(void *argument) {
  (void)argument;

  vTaskDelete(NULL);
}

static void component_test_button_callback(void *_argument,
                                           const GPIO_PinState state) {
  uint32_t argument = (uint32_t)_argument;

  switch (argument) {
    case BUTTON_BUILTIN:
      if (state == GPIO_PIN_RESET) {
        LedController_turn_on(&led_controller, LED_BUILTIN);
        ServoController_set_duty(&servo_controller, SERVO_WHEEL_FL, 1);
        MotorController_set(&motor_controller, MOTOR_FL, MotorForward);
      } else {
        LedController_turn_off(&led_controller, LED_BUILTIN);
        ServoController_set_duty(&servo_controller, SERVO_WHEEL_FL, 0);
        MotorController_set(&motor_controller, MOTOR_FL, MotorStop);
      }
      break;

    case BUTTON1:
      if (state == GPIO_PIN_SET) {
        LedController_turn_on(&led_controller, LED_GREEN);
        ServoController_set_duty(&servo_controller, SERVO_WHEEL_FR, 1);
        MotorController_set(&motor_controller, MOTOR_FR, MotorForward);
      } else {
        LedController_turn_off(&led_controller, LED_GREEN);
        ServoController_set_duty(&servo_controller, SERVO_WHEEL_FR, 0);
        MotorController_set(&motor_controller, MOTOR_FR, MotorStop);
      }
      break;

    case BUTTON2:
      if (state == GPIO_PIN_SET) {
        LedController_turn_on(&led_controller, LED_YELLOW);
        ServoController_set_duty(&servo_controller, SERVO_WHEEL_RL, 1);
        MotorController_set(&motor_controller, MOTOR_RL, MotorForward);
      } else {
        LedController_turn_off(&led_controller, LED_YELLOW);
        ServoController_set_duty(&servo_controller, SERVO_WHEEL_RL, 0);
        MotorController_set(&motor_controller, MOTOR_RL, MotorStop);
      }
      break;

    case BUTTON3:
      if (state == GPIO_PIN_SET) {
        LedController_turn_on(&led_controller, LED_RED);
        ServoController_set_duty(&servo_controller, SERVO_WHEEL_RR, 1);
        MotorController_set(&motor_controller, MOTOR_RR, MotorForward);
      } else {
        LedController_turn_off(&led_controller, LED_RED);
        ServoController_set_duty(&servo_controller, SERVO_WHEEL_RR, 0);
        MotorController_set(&motor_controller, MOTOR_RR, MotorStop);
      }
      break;

    default:
      break;
  }
}

#elif defined(PRINT_TEST)
/* print_test ----------------------------------------------------------------*/
static void print_test(void *argument) {
  (void)argument;

  while (1) {
    printf("Hello World\n");
    vTaskDelay(100);
  }
}

#else
static void user_task_code(void *argument) {
  (void)argument;

  // wait for button to be pressed
  xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);

  LedController_turn_on(&led_controller, LED_BUILTIN);
  MotorController_set(&motor_controller, MOTOR_RR, MotorBackward);

  GPIO_PinState button_state;
  do {
    ButtonMonitor_read_state(&button_monitor, BUTTON_MICRO_FT, &button_state);
    if (button_state == GPIO_PIN_SET) {
      break;
    }
    vTaskDelay(100);
  } while (1);

  // while (!is_45_deg) {
  //   path_track_once();
  //   vTaskDelay(100);
  // }

  // MotorController_set(&motor_controller, MOTOR_RR, MotorForward);
  // do {
  //   ButtonMonitor_read_state(&button_monitor, BUTTON_MICRO_FB,
  //   &button_state); if (button_state == GPIO_PIN_SET) {
  //     break;
  //   }
  //   vTaskDelay(100);
  // } while (1);

  while (!is_90_deg) {
    path_track_once();
    vTaskDelay(100);
  }

  ServoController_set_duty(&servo_controller, SERVO_WHEEL_FL, 0);
  ServoController_set_duty(&servo_controller, SERVO_WHEEL_FR, 0);
  ServoController_set_duty(&servo_controller, SERVO_WHEEL_RL, 0);
  ServoController_set_duty(&servo_controller, SERVO_WHEEL_RR, 0);

  vTaskDelete(NULL);
}

static void user_button_callback(void *_argument, const GPIO_PinState state) {
  if (state == GPIO_PIN_RESET) {
    xTaskNotify(user_task_handle, 0, eNoAction);
  }
}

static void operate_wings_button_callback(void *_argument,
                                          const GPIO_PinState state) {
  uint32_t argument = (uint32_t)_argument;

  GPIO_PinState button_state;
  switch (argument) {
    case BUTTON1:
      if (state == GPIO_PIN_SET) {
        ButtonMonitor_read_state(&button_monitor, BUTTON_MICRO_FB,
                                 &button_state);
        if (button_state != GPIO_PIN_SET) {
          MotorController_set(&motor_controller, MOTOR_RR, MotorForward);
        }
      } else {
        MotorController_set(&motor_controller, MOTOR_RR, MotorStop);
      }
      break;

    case BUTTON2:
      if (state == GPIO_PIN_SET) {
        ButtonMonitor_read_state(&button_monitor, BUTTON_MICRO_FT,
                                 &button_state);
        if (button_state != GPIO_PIN_SET) {
          MotorController_set(&motor_controller, MOTOR_RR, MotorBackward);
        }
      } else {
        MotorController_set(&motor_controller, MOTOR_RR, MotorStop);
      }
      break;

    default:
      break;
  }
}

static void stop_wings_button_callback(void *_argument,
                                       const GPIO_PinState state) {
  uint32_t argument = (uint32_t)_argument;

  switch (argument) {
    case BUTTON_MICRO_FB:
      if (state == GPIO_PIN_SET) {
        MotorController_set(&motor_controller, MOTOR_RR, MotorStop);
      }
      break;

    case BUTTON_MICRO_FT:
      if (state == GPIO_PIN_SET) {
        MotorController_set(&motor_controller, MOTOR_RR, MotorStop);
      }
      break;

    default:
      break;
  }
}

#endif

static void running_indicator_task_code(void *argument) {
  (void)argument;

  MotorController_set(&motor_controller, MOTOR_FR, MotorStop);
  while (1) {
    LedController_turn_on(&led_controller, LED_GREEN);
#ifdef ENABLE_ALARM
    MotorController_set(&motor_controller, MOTOR_FR, MotorForward);
#endif  // ENABLE_ALARM
    vTaskDelay(250);
    LedController_turn_off(&led_controller, LED_GREEN);
#ifdef ENABLE_ALARM
    MotorController_set(&motor_controller, MOTOR_FR, MotorStop);
#endif  // ENABLE_ALARM
    vTaskDelay(250);
  }
}

static void mpu6050_task_code(void *argument) {
  (void)argument;

  while (1) {
    uint16_t len = 4;
    static int16_t gs_accel_raw[4][3];
    static float gs_accel_g[4][3];
    static int16_t gs_gyro_raw[4][3];
    static float gs_gyro_dps[4][3];
    static int32_t gs_quat[4][4];
    static float gs_pitch[4];
    static float gs_roll[4];
    static float gs_yaw[4];

    vTaskDelay(200);

    /* read */
    if (mpu6050_dmp_read_all(gs_accel_raw, gs_accel_g, gs_gyro_raw, gs_gyro_dps,
                             gs_quat, gs_pitch, gs_roll, gs_yaw, &len) != 0) {
      Error_Handler();
    }

    if (gs_yaw[len - 1] > 16) {
      is_45_deg = true;
      is_90_deg = true;
      LedController_turn_on(&led_controller, LED_RED);
    } else if (gs_yaw[len - 1] > 6.7) {
      is_45_deg = true;
      is_90_deg = false;
      LedController_turn_off(&led_controller, LED_RED);
    } else {
      is_45_deg = false;
      is_90_deg = false;
      LedController_turn_off(&led_controller, LED_RED);
    }
  }
}

/* init functions ------------------------------------------------------------*/
static void button_init(void) {
  ButtonMonitor_ctor(&button_monitor);
  // should be in the same order as taht in project_def.h
  ButtonMonitor_add_button(&button_monitor, &button_cb[BUTTON_BUILTIN],
                           BUTTON_BUILTIN_PORT, BUTTON_BUILTIN_PIN);
  ButtonMonitor_add_button(&button_monitor, &button_cb[BUTTON1], BUTTON1_PORT,
                           BUTTON1_PIN);
  ButtonMonitor_add_button(&button_monitor, &button_cb[BUTTON2], BUTTON2_PORT,
                           BUTTON2_PIN);
  ButtonMonitor_add_button(&button_monitor, &button_cb[BUTTON3], BUTTON3_PORT,
                           BUTTON3_PIN);
  ButtonMonitor_add_button(&button_monitor, &button_cb[BUTTON_MICRO_FB],
                           MICRO_FB_PORT, MICRO_FB_PIN);
  ButtonMonitor_add_button(&button_monitor, &button_cb[BUTTON_MICRO_FT],
                           MICRO_FT_PORT, MICRO_FT_PIN);
  ButtonMonitor_add_button(&button_monitor, &button_cb[BUTTON_MICRO_RB],
                           MICRO_RB_PORT, MICRO_RB_PIN);
  ButtonMonitor_add_button(&button_monitor, &button_cb[BUTTON_MICRO_RT],
                           MICRO_RT_PORT, MICRO_RT_PIN);

#if defined(MAX_POWER_TEST)
  ButtonMonitor_register_callback(&button_monitor, BUTTON_BUILTIN,
                                  &max_power_button_callback, NULL);
#elif defined(LIFT_TEST)
  ButtonMonitor_register_callback(&button_monitor, BUTTON1,
                                  &operate_wings_button_callback,
                                  (void *)BUTTON1);
  ButtonMonitor_register_callback(&button_monitor, BUTTON2,
                                  &operate_wings_button_callback,
                                  (void *)BUTTON2);
  ButtonMonitor_register_callback(&button_monitor, BUTTON_MICRO_FB,
                                  &stop_wings_button_callback,
                                  (void *)BUTTON_MICRO_FB);
  ButtonMonitor_register_callback(&button_monitor, BUTTON_MICRO_FT,
                                  &stop_wings_button_callback,
                                  (void *)BUTTON_MICRO_FT);
#elif defined(GO_STRAIGHT_TEST)
  ButtonMonitor_register_callback(&button_monitor, BUTTON_BUILTIN,
                                  &go_straight_button_callback, NULL);
#elif defined(PATH_TRACK_TEST)
  ButtonMonitor_register_callback(&button_monitor, BUTTON_BUILTIN,
                                  &path_track_button_callback, NULL);
#elif defined(BUTTON_TEST)
  ButtonMonitor_register_callback(&button_monitor, BUTTON_BUILTIN,
                                  &button_test_button_callback,
                                  (void *)BUTTON_BUILTIN);
  ButtonMonitor_register_callback(
      &button_monitor, BUTTON1, &button_test_button_callback, (void *)BUTTON1);
  ButtonMonitor_register_callback(
      &button_monitor, BUTTON2, &button_test_button_callback, (void *)BUTTON2);
  ButtonMonitor_register_callback(
      &button_monitor, BUTTON3, &button_test_button_callback, (void *)BUTTON3);
#elif defined(COMPONENT_TEST)
  ButtonMonitor_register_callback(&button_monitor, BUTTON_BUILTIN,
                                  &component_test_button_callback,
                                  (void *)BUTTON_BUILTIN);
  ButtonMonitor_register_callback(&button_monitor, BUTTON1,
                                  &component_test_button_callback,
                                  (void *)BUTTON1);
  ButtonMonitor_register_callback(&button_monitor, BUTTON2,
                                  &component_test_button_callback,
                                  (void *)BUTTON2);
  ButtonMonitor_register_callback(&button_monitor, BUTTON3,
                                  &component_test_button_callback,
                                  (void *)BUTTON3);
#elif defined(WRITE_EEPROM_TEST)
  ButtonMonitor_register_callback(&button_monitor, BUTTON_BUILTIN,
                                  &filter_calibration_button_callback, NULL);
#else
  ButtonMonitor_register_callback(&button_monitor, BUTTON_BUILTIN,
                                  &user_button_callback, NULL);
  ButtonMonitor_register_callback(&button_monitor, BUTTON1,
                                  &operate_wings_button_callback,
                                  (void *)BUTTON1);
  ButtonMonitor_register_callback(&button_monitor, BUTTON2,
                                  &operate_wings_button_callback,
                                  (void *)BUTTON2);
  ButtonMonitor_register_callback(&button_monitor, BUTTON3,
                                  &filter_calibration_button_callback, NULL);
  ButtonMonitor_register_callback(&button_monitor, BUTTON_MICRO_FB,
                                  &stop_wings_button_callback,
                                  (void *)BUTTON_MICRO_FB);
  ButtonMonitor_register_callback(&button_monitor, BUTTON_MICRO_FT,
                                  &stop_wings_button_callback,
                                  (void *)BUTTON_MICRO_FT);
#endif

  ButtonMonitor_start(&button_monitor);
}

static void led_init(void) {
  LedController_ctor(&led_controller);
  // should be in the same order as taht in project_def.h
  LedController_add_led(&led_controller, &led_cb[LED_BUILTIN], LED_BUILTIN_PORT,
                        LED_BUILTIN_PIN);
  LedController_add_led(&led_controller, &led_cb[LED_GREEN], LED_GREEN_PORT,
                        LED_GREEN_PIN);
  LedController_add_led(&led_controller, &led_cb[LED_YELLOW], LED_YELLOW_PORT,
                        LED_YELLOW_PIN);
  LedController_add_led(&led_controller, &led_cb[LED_RED], LED_RED_PORT,
                        LED_RED_PIN);
  LedController_start(&led_controller);
}

static void servo_init(void) {
  ServoController_ctor(&servo_controller);
  // should be in the same order as taht in project_def.h
  ServoController_add_servo(&servo_controller, &servo_cb[SERVO_WHEEL_FL],
                            SERVO_WHEEL_FL_TIM, SERVO_WHEEL_FL_TIM_CHANNEL);
  ServoController_add_servo(&servo_controller, &servo_cb[SERVO_WHEEL_FR],
                            SERVO_WHEEL_FR_TIM, SERVO_WHEEL_FR_TIM_CHANNEL);
  ServoController_add_servo(&servo_controller, &servo_cb[SERVO_WHEEL_RL],
                            SERVO_WHEEL_RL_TIM, SERVO_WHEEL_RL_TIM_CHANNEL);
  ServoController_add_servo(&servo_controller, &servo_cb[SERVO_WHEEL_RR],
                            SERVO_WHEEL_RR_TIM, SERVO_WHEEL_RR_TIM_CHANNEL);
  ServoController_start(&servo_controller);

  ServoController_set_direction(&servo_controller, SERVO_WHEEL_FL,
                                SERVO_COUNTER_CLOCKWISE);
  ServoController_set_direction(&servo_controller, SERVO_WHEEL_FR,
                                SERVO_CLOCKWISE);
  ServoController_set_direction(&servo_controller, SERVO_WHEEL_RL,
                                SERVO_COUNTER_CLOCKWISE);
  ServoController_set_direction(&servo_controller, SERVO_WHEEL_RR,
                                SERVO_CLOCKWISE);
}

static void motor_init(void) {  // motor controller
  MotorController_ctor(&motor_controller);
  // should be in the same order as taht in project_def.h
  MotorController_add_motor(&motor_controller, &motor_cb[MOTOR_FL],
                            MOTOR_FL_1_PORT, MOTOR_FL_1_PIN, MOTOR_FL_2_PORT,
                            MOTOR_FL_2_PIN);
  MotorController_add_motor(&motor_controller, &motor_cb[MOTOR_FR],
                            MOTOR_FR_1_PORT, MOTOR_FR_1_PIN, MOTOR_FR_2_PORT,
                            MOTOR_FR_2_PIN);
  MotorController_add_motor(&motor_controller, &motor_cb[MOTOR_RL],
                            MOTOR_RL_1_PORT, MOTOR_RL_1_PIN, MOTOR_RL_2_PORT,
                            MOTOR_RL_2_PIN);
  MotorController_add_motor(&motor_controller, &motor_cb[MOTOR_RR],
                            MOTOR_RR_1_PORT, MOTOR_RR_1_PIN, MOTOR_RR_2_PORT,
                            MOTOR_RR_2_PIN);
  MotorController_set(&motor_controller, MOTOR_FL, MotorStop);
  MotorController_set(&motor_controller, MOTOR_FR, MotorStop);
  MotorController_set(&motor_controller, MOTOR_RL, MotorStop);
  MotorController_set(&motor_controller, MOTOR_RR, MotorStop);
}

static void a_receive_callback(uint8_t type) {
  switch (type) {
    case MPU6050_INTERRUPT_MOTION: {
      mpu6050_interface_debug_print("mpu6050: irq motion.\n");

      break;
    }
    case MPU6050_INTERRUPT_FIFO_OVERFLOW: {
      mpu6050_interface_debug_print("mpu6050: irq fifo overflow.\n");

      break;
    }
    case MPU6050_INTERRUPT_I2C_MAST: {
      mpu6050_interface_debug_print("mpu6050: irq i2c master.\n");

      break;
    }
    case MPU6050_INTERRUPT_DMP: {
      mpu6050_interface_debug_print("mpu6050: irq dmp\n");

      break;
    }
    case MPU6050_INTERRUPT_DATA_READY: {
      mpu6050_interface_debug_print("mpu6050: irq data ready\n");

      break;
    }
    default: {
      mpu6050_interface_debug_print("mpu6050: irq unknown code.\n");

      break;
    }
  }
}

/* callback function ---------------------------------------------------------*/
int __io_putchar(int ch) {
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

void __module_assert_fail(const char *assertion, const char *file,
                          unsigned int line, const char *function) {
  (void)assertion;
  (void)file;
  (void)line;
  (void)function;

  Error_Handler();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) { mpu6050_dmp_irq_handler(); }
