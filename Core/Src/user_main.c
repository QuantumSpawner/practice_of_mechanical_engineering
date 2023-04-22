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

// stm32_module include
#include "stm32_module/stm32_module.h"

// project include
#include "project_def.h"
#include "simple_motor_controller.h"

/* static variable -----------------------------------------------------------*/
// adc
uint16_t adc1_buffer[NUM_ANALOG];

/* module --------------------------------------------------------------------*/
// button
static ButtonMonitor button_monitor;
static struct button_cb button_cb[NUM_BUTTON];

// filter
static MovingAverageFilter
    grayscale_moving_average_filter[NUM_MOVING_AVERAGE_FILTER];
static float
    grayscale_moving_average_buffer[NUM_MOVING_AVERAGE_FILTER]
                                   [MOVING_AVERAGE_FILTER_MOVING_AVERAGE_SIZE];
static NormalizeFilter grayscale_normalize_filer[NUM_NORMALIZE_FILTER];

// led
static LedController led_controller;
static struct led_cb led_cb[NUM_LED];

// servo
static ServoController servo_controller;
static struct servo_cb servo_cb[NUM_SERVO];

// motor
static MotorController motor_controller;
static struct motor_cb motor_cb[NUM_MOTOR];

// freertos task
static TaskHandle_t user_task_handle;
static StaticTask_t user_task_cb;
static StackType_t user_task_stack[USER_TASK_STACK_SIZE];

/* function ------------------------------------------------------------------*/
void user_init() {
  // module init
  button_init();
  filter_init();
  led_init();
  servo_init();
  motor_init();

  // stm32 init
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_buffer, NUM_ANALOG);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);

  // task start
#if defined(MAX_POWER_TEST)
  user_task_handle = xTaskCreateStatic(
      max_power_test, "max_power_test", USER_TASK_STACK_SIZE, NULL,
      TaskPriorityNormal, user_task_stack, &user_task_cb);
#elif defined(LIFT_TEST)
  user_task_handle =
      xTaskCreateStatic(lift_test, "lift_test", USER_TASK_STACK_SIZE, NULL,
                        TaskPriorityNormal, user_task_stack, &user_task_cb);
#elif defined(PATH_FIND_TEST)
  user_task_handle = xTaskCreateStatic(
      path_find_test, "path_find_test", USER_TASK_STACK_SIZE, NULL,
      TaskPriorityNormal, user_task_stack, &user_task_cb);
#endif
}

void user_task_code(void *argument) {
  (void)argument;

  while (1) {
    vTaskDelay(1);
  }
}
/* max_power_test ------------------------------------------------------------*/
void max_power_test(void *argument) {
  (void)argument;

  ServoController_set_direction(&servo_controller, SERVO_WHEEL_FL,
                                SERVO_COUNTER_CLOCKWISE);
  ServoController_set_direction(&servo_controller, SERVO_WHEEL_FR,
                                SERVO_CLOCKWISE);

  vTaskDelete(NULL);
}

void max_power_button_callback(void *argument, const GPIO_PinState state) {
  (void)argument;

  if (state == GPIO_PIN_RESET) {
    LedController_turn_on(&led_controller, LED_BUILTIN);
    ServoController_set_duty(&servo_controller, SERVO_WHEEL_FL, 1);
    ServoController_set_duty(&servo_controller, SERVO_WHEEL_FR, 1);

    MotorController_set(&motor_controller, MOTOR_FL, MotorForward);
    MotorController_set(&motor_controller, MOTOR_FR, MotorForward);
    MotorController_set(&motor_controller, MOTOR_RL, MotorForward);
    MotorController_set(&motor_controller, MOTOR_RR, MotorForward);
  } else {
    LedController_turn_off(&led_controller, LED_BUILTIN);
    ServoController_set_duty(&servo_controller, SERVO_WHEEL_FL, 0);
    ServoController_set_duty(&servo_controller, SERVO_WHEEL_FR, 0);

    MotorController_set(&motor_controller, MOTOR_FL, MotorStop);
    MotorController_set(&motor_controller, MOTOR_FR, MotorStop);
    MotorController_set(&motor_controller, MOTOR_RL, MotorStop);
    MotorController_set(&motor_controller, MOTOR_RR, MotorStop);
  }
}

/* lift_test -----------------------------------------------------------------*/
volatile bool start_lift_test = false, lift_at_button, lift_at_top = false;
GPIO_PinState button[NUM_BUTTON];
void lift_test(void *argument) {
  (void)argument;

  // wait for button to be pressed
  while (!start_lift_test) {
    vTaskDelay(10);
  }

  LedController_turn_on(&led_controller, LED_BUILTIN);
  for (int i = 0; i < NUM_BUTTON_MICRO; i++) {
    ButtonMonitor_read_state(&button_monitor, BUTTON_MICRO(i),
                             &button[BUTTON_MICRO(i)]);
  }

  if (button[BUTTON_MICRO_FB] != GPIO_PIN_SET ||
      button[BUTTON_MICRO_RB] != GPIO_PIN_SET) {
    lift_at_button = false;
    MotorController_set(&motor_controller, MOTOR_FL, MotorBackward);
    MotorController_set(&motor_controller, MOTOR_FR, MotorBackward);
    MotorController_set(&motor_controller, MOTOR_RL, MotorBackward);
    MotorController_set(&motor_controller, MOTOR_RR, MotorBackward);
  } else {
    lift_at_button = true;
  }

  // wait for lit to reset
  while (!lift_at_button) {
    vTaskDelay(10);
  }

  LedController_turn_off(&led_controller, LED_BUILTIN);
  MotorController_set(&motor_controller, MOTOR_FL, MotorForward);
  MotorController_set(&motor_controller, MOTOR_FR, MotorForward);
  MotorController_set(&motor_controller, MOTOR_RL, MotorForward);
  MotorController_set(&motor_controller, MOTOR_RR, MotorForward);

  // wait for lit to reach top
  while (!lift_at_top) {
    vTaskDelay(10);
  }

  LedController_turn_on(&led_controller, LED_BUILTIN);
  MotorController_set(&motor_controller, MOTOR_FL, MotorStop);
  MotorController_set(&motor_controller, MOTOR_FR, MotorStop);
  MotorController_set(&motor_controller, MOTOR_RL, MotorStop);
  MotorController_set(&motor_controller, MOTOR_RR, MotorStop);

  vTaskDelete(NULL);
}

void lift_button_callback(void *_argument, const GPIO_PinState state) {
  uint32_t argument = (uint32_t)_argument;

  button[argument] = state;

  if (argument == BUTTON_BUILTIN) {
    if (state == GPIO_PIN_RESET) {
      start_lift_test = true;
    }
  } else if (argument == BUTTON_MICRO_FB) {
    if (state == GPIO_PIN_SET && button[BUTTON_MICRO_RB] == GPIO_PIN_SET) {
      lift_at_button = true;
    }
  } else if (argument == BUTTON_MICRO_RB) {
    if (state == GPIO_PIN_SET && button[BUTTON_MICRO_FB] == GPIO_PIN_SET) {
      lift_at_button = true;
    }
  } else if (argument == BUTTON_MICRO_FT) {
    if (state == GPIO_PIN_SET && button[BUTTON_MICRO_RT] == GPIO_PIN_SET) {
      lift_at_top = true;
    }
  } else if (argument == BUTTON_MICRO_RT) {
    if (state == GPIO_PIN_SET && button[BUTTON_MICRO_FT] == GPIO_PIN_SET) {
      lift_at_top = true;
    }
  }
}

/* path_find_test ------------------------------------------------------------*/
volatile bool start_path_find_test = false;
void path_find_test(void *argument) {
  (void)argument;

  // wait for button to be pressed
  while (!start_path_find_test) {
    vTaskDelay(10);
  }

  LedController_turn_on(&led_controller, LED_BUILTIN);
  ServoController_set_direction(&servo_controller, SERVO_WHEEL_FL,
                                SERVO_COUNTER_CLOCKWISE);
  ServoController_set_direction(&servo_controller, SERVO_WHEEL_FR,
                                SERVO_CLOCKWISE);

  ServoController_set_duty(&servo_controller, SERVO_WHEEL_FL, 1);
  ServoController_set_duty(&servo_controller, SERVO_WHEEL_FR, 1);

  // initialize to 0 for the filters to initialize
  float grayscale_left = 0, grayscale_right = 0;
  while (1) {
    NormalizeFilter_get_filtered_data(&grayscale_normalize_filer[ANALOG_FL1],
                                      &grayscale_left);
    NormalizeFilter_get_filtered_data(&grayscale_normalize_filer[ANALOG_FR1],
                                      &grayscale_right);

    float servo_left_duty, servo_right_duty;
    if (grayscale_left < ANALOG_LOWER_THRESHOLD &&
        grayscale_right < ANALOG_LOWER_THRESHOLD) {
      servo_left_duty = SERVO_SPEED_STOP;
      servo_right_duty = SERVO_SPEED_STOP;

    } else if (grayscale_left < ANALOG_LOWER_THRESHOLD) {
      servo_left_duty = SERVO_SPEED_MINIMUM;
      servo_right_duty = SERVO_SPEED_MAXIMUM;

    } else if (grayscale_right < ANALOG_LOWER_THRESHOLD) {
      servo_left_duty = SERVO_SPEED_MAXIMUM;
      servo_right_duty = SERVO_SPEED_MINIMUM;

    } else {
      float grayscale_difference_ratio = (grayscale_left - grayscale_right) /
                                         (grayscale_left + grayscale_right);

      if (grayscale_difference_ratio > 0) {
        servo_left_duty = SERVO_SPEED_MAXIMUM;
        servo_right_duty = SERVO_SPEED_MAXIMUM -
                           SERVO_SPEED_REAGE * grayscale_difference_ratio;

      } else {
        servo_left_duty = SERVO_SPEED_MAXIMUM +
                          SERVO_SPEED_REAGE * grayscale_difference_ratio;
        servo_right_duty = SERVO_SPEED_MAXIMUM;
      }
    }

    ServoController_set_duty(&servo_controller, SERVO_WHEEL_FL,
                             servo_left_duty);
    ServoController_set_duty(&servo_controller, SERVO_WHEEL_FR,
                             servo_right_duty);

    vTaskDelay(100);
  }
}

void path_find_button_callback(void *argument, const GPIO_PinState state) {
  (void)argument;

  if (state == GPIO_PIN_RESET) {
    start_path_find_test = true;
  }
}

/* print_test ----------------------------------------------------------------*/
void print_test(void *argument) {
  (void)argument;

  while (1) {
    printf("Hello World\n");
    vTaskDelay(100);
  }
}

/* init_functions ------------------------------------------------------------*/
void button_init(void) {
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
  ButtonMonitor_register_callback(&button_monitor, BUTTON_BUILTIN,
                                  &lift_button_callback,
                                  (void *)BUTTON_BUILTIN);
  ButtonMonitor_register_callback(&button_monitor, BUTTON_MICRO_FB,
                                  &lift_button_callback,
                                  (void *)BUTTON_MICRO_FB);
  ButtonMonitor_register_callback(&button_monitor, BUTTON_MICRO_FT,
                                  &lift_button_callback,
                                  (void *)BUTTON_MICRO_FT);
  ButtonMonitor_register_callback(&button_monitor, BUTTON_MICRO_RB,
                                  &lift_button_callback,
                                  (void *)BUTTON_MICRO_RB);
  ButtonMonitor_register_callback(&button_monitor, BUTTON_MICRO_RT,
                                  &lift_button_callback,
                                  (void *)BUTTON_MICRO_RT);
#elif defined(PATH_FIND_TEST)
  ButtonMonitor_register_callback(&button_monitor, BUTTON_BUILTIN,
                                  &path_find_button_callback, NULL);
#endif

  ButtonMonitor_start(&button_monitor);
}

void filter_init(void) {
  MovingAverageFilter_ctor(
      &grayscale_moving_average_filter[MOVING_AVERAGE_FILTER_ANALOG_FL1],
      grayscale_moving_average_buffer[MOVING_AVERAGE_FILTER_ANALOG_FL1],
      MOVING_AVERAGE_FILTER_MOVING_AVERAGE_SIZE, NULL);
  MovingAverageFilter_ctor(
      &grayscale_moving_average_filter[MOVING_AVERAGE_FILTER_ANALOG_FL2],
      grayscale_moving_average_buffer[MOVING_AVERAGE_FILTER_ANALOG_FL2],
      MOVING_AVERAGE_FILTER_MOVING_AVERAGE_SIZE, NULL);
  MovingAverageFilter_ctor(
      &grayscale_moving_average_filter[MOVING_AVERAGE_FILTER_ANALOG_FR1],
      grayscale_moving_average_buffer[MOVING_AVERAGE_FILTER_ANALOG_FR1],
      MOVING_AVERAGE_FILTER_MOVING_AVERAGE_SIZE, NULL);
  MovingAverageFilter_ctor(
      &grayscale_moving_average_filter[MOVING_AVERAGE_FILTER_ANALOG_FR2],
      grayscale_moving_average_buffer[MOVING_AVERAGE_FILTER_ANALOG_FR2],
      MOVING_AVERAGE_FILTER_MOVING_AVERAGE_SIZE, NULL);
  MovingAverageFilter_ctor(
      &grayscale_moving_average_filter[MOVING_AVERAGE_FILTER_ANALOG_RL],
      grayscale_moving_average_buffer[MOVING_AVERAGE_FILTER_ANALOG_RL],
      MOVING_AVERAGE_FILTER_MOVING_AVERAGE_SIZE, NULL);
  MovingAverageFilter_ctor(
      &grayscale_moving_average_filter[MOVING_AVERAGE_FILTER_ANALOG_RR],
      grayscale_moving_average_buffer[MOVING_AVERAGE_FILTER_ANALOG_RR],
      MOVING_AVERAGE_FILTER_MOVING_AVERAGE_SIZE, NULL);

  NormalizeFilter_ctor(&grayscale_normalize_filer[NORMALIZE_FILTER_ANALOG_FL1],
                       NORMALIZE_FILTER_LOWER_BOUND_ANALOG_FL1,
                       NORMALIZE_FILTER_LOWER_BOUND_ANALOG_FL1,
                       (Filter *)&grayscale_moving_average_filter
                           [MOVING_AVERAGE_FILTER_ANALOG_FL1]);
  NormalizeFilter_ctor(&grayscale_normalize_filer[NORMALIZE_FILTER_ANALOG_FL2],
                       NORMALIZE_FILTER_LOWER_BOUND_ANALOG_FL2,
                       NORMALIZE_FILTER_LOWER_BOUND_ANALOG_FL2,
                       (Filter *)&grayscale_moving_average_filter
                           [MOVING_AVERAGE_FILTER_ANALOG_FL2]);
  NormalizeFilter_ctor(&grayscale_normalize_filer[NORMALIZE_FILTER_ANALOG_FR1],
                       NORMALIZE_FILTER_LOWER_BOUND_ANALOG_FR1,
                       NORMALIZE_FILTER_LOWER_BOUND_ANALOG_FR1,
                       (Filter *)&grayscale_moving_average_filter
                           [MOVING_AVERAGE_FILTER_ANALOG_FR1]);
  NormalizeFilter_ctor(&grayscale_normalize_filer[NORMALIZE_FILTER_ANALOG_FR2],
                       NORMALIZE_FILTER_LOWER_BOUND_ANALOG_FR2,
                       NORMALIZE_FILTER_LOWER_BOUND_ANALOG_FR2,
                       (Filter *)&grayscale_moving_average_filter
                           [MOVING_AVERAGE_FILTER_ANALOG_FR2]);
  NormalizeFilter_ctor(
      &grayscale_normalize_filer[NORMALIZE_FILTER_ANALOG_RL],
      NORMALIZE_FILTER_LOWER_BOUND_ANALOG_RL,
      NORMALIZE_FILTER_LOWER_BOUND_ANALOG_RL,
      (Filter
           *)&grayscale_moving_average_filter[MOVING_AVERAGE_FILTER_ANALOG_RL]);
  NormalizeFilter_ctor(
      &grayscale_normalize_filer[NORMALIZE_FILTER_ANALOG_RR],
      NORMALIZE_FILTER_LOWER_BOUND_ANALOG_RR,
      NORMALIZE_FILTER_LOWER_BOUND_ANALOG_RR,
      (Filter
           *)&grayscale_moving_average_filter[MOVING_AVERAGE_FILTER_ANALOG_RR]);
}

void led_init(void) {
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

void servo_init(void) {
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
}

void motor_init(void) {  // motor controller
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

/* callback function ---------------------------------------------------------*/
int __io_putchar(int ch) {
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc == &hadc1) {
    NormalizeFilter_update(&grayscale_normalize_filer[ANALOG_FL1],
                           adc1_buffer[ANALOG_FL1] / 4096.0, NULL);
    NormalizeFilter_update(&grayscale_normalize_filer[ANALOG_FL2],
                           adc1_buffer[ANALOG_FL2] / 4096.0, NULL);
    NormalizeFilter_update(&grayscale_normalize_filer[ANALOG_FR1],
                           adc1_buffer[ANALOG_FR1] / 4096.0, NULL);
    NormalizeFilter_update(&grayscale_normalize_filer[ANALOG_FR2],
                           adc1_buffer[ANALOG_FR2] / 4096.0, NULL);
    NormalizeFilter_update(&grayscale_normalize_filer[ANALOG_RL],
                           adc1_buffer[ANALOG_RL] / 4096.0, NULL);
    NormalizeFilter_update(&grayscale_normalize_filer[ANALOG_RR],
                           adc1_buffer[ANALOG_RR] / 4096.0, NULL);
#ifdef PRINT_GRAYSCALE_DATA
    printf("%f %f\n", adc1_buffer[ANALOG_FL1] / 4096.0,
           adc1_buffer[ANALOG_FR1] / 4096.0);
#endif  // PRINT_GRAYSCALE_DATA
  }
}

extern void Error_Handler();
void __module_assert_fail(const char *assertion, const char *file,
                          unsigned int line, const char *function) {
  (void)assertion;
  (void)file;
  (void)line;
  (void)function;

  Error_Handler();
}
