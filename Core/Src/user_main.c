#include "user_main.h"

// glibc include
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
union {
  struct {
    uint16_t grayscale_left;

    uint16_t grayscale_right;
  };
  uint16_t data[ADC1_BUFFER_SIZE];
} adc1_buffer;

/* module --------------------------------------------------------------------*/
static ButtonMonitor button_monitor;
static struct button_cb button_cb[NUM_BUTTON];

static MovingAverageFilter grayscale_moving_average_filter[NUM_GRAYSCALE];
static float grayscale_moving_average_buffer[NUM_GRAYSCALE]
                                            [GRAYSCALE_MOVING_AVERAGE_SIZE];
static NormalizeFilter grayscale_normalize_filer[NUM_GRAYSCALE];

static LedController led_controller;
static struct led_cb led_cb[NUM_LED];

static MotorController motor_controller;
static struct motor_cb motor_cb[NUM_MOTOR];

static ServoController servo_controller;
static struct servo_cb servo_cb[NUM_SERVO];

static TaskHandle_t user_task_handle;
static StaticTask_t user_task_cb;
static StackType_t user_task_stack[USER_TASK_STACK_SIZE];

/* function ------------------------------------------------------------------*/
void user_init() {
  /* module init -------------------------------------------------------------*/
  // led
  LedController_ctor(&led_controller);
  // should be in the same order as taht in project_def.h
  LedController_add_led(&led_controller, &led_cb[LED_USER], LD2_GPIO_Port,
                        LD2_Pin);
  LedController_start(&led_controller);

  // filters
  MovingAverageFilter_ctor(&grayscale_moving_average_filter[GRAYSCALE_LEFT],
                           grayscale_moving_average_buffer[GRAYSCALE_LEFT],
                           GRAYSCALE_MOVING_AVERAGE_SIZE, NULL);
  MovingAverageFilter_ctor(&grayscale_moving_average_filter[GRAYSCALE_RIGHT],
                           grayscale_moving_average_buffer[GRAYSCALE_RIGHT],
                           GRAYSCALE_MOVING_AVERAGE_SIZE, NULL);
  NormalizeFilter_ctor(
      &grayscale_normalize_filer[GRAYSCALE_LEFT], GRAYSCALE_LEFT_LOWER_BOUND,
      GRAYSCALE_LEFT_UPPER_BOUND,
      (Filter *)&grayscale_moving_average_filter[GRAYSCALE_LEFT]);
  NormalizeFilter_ctor(
      &grayscale_normalize_filer[GRAYSCALE_RIGHT], GRAYSCALE_RIGHT_LOWER_BOUND,
      GRAYSCALE_RIGHT_UPPER_BOUND,
      (Filter *)&grayscale_moving_average_filter[GRAYSCALE_RIGHT]);

  // button
  ButtonMonitor_ctor(&button_monitor);
  // should be in the same order as taht in project_def.h
  ButtonMonitor_add_button(&button_monitor, &button_cb[BUTTON_USER],
                           B1_GPIO_Port, B1_Pin);
  ButtonMonitor_add_button(&button_monitor, &button_cb[MICROSWITCH_FB],
                           MICROSWITCHS_FB_PORT, MICROSWITCHS_FB_PIN);
  ButtonMonitor_add_button(&button_monitor, &button_cb[MICROSWITCH_FT],
                           MICROSWITCHS_FT_PORT, MICROSWITCHS_FT_PIN);
  ButtonMonitor_add_button(&button_monitor, &button_cb[MICROSWITCH_RB],
                           MICROSWITCHS_RB_PORT, MICROSWITCHS_RB_PIN);
  ButtonMonitor_add_button(&button_monitor, &button_cb[MICROSWITCH_RT],
                           MICROSWITCHS_RT_PORT, MICROSWITCHS_RT_PIN);
  ButtonMonitor_start(&button_monitor);

  // motor
  MotorController_ctor(&motor_controller);
  // should be in the same order as taht in project_def.h
  MotorController_add_motor(&motor_controller, &motor_cb[MOTOR_FRONT],
                            MOTOR_FRONT_1, MOTOR_FRONT_1_PIN, MOTOR_FRONT_2,
                            MOTOR_FRONT_2_PIN);
  MotorController_add_motor(&motor_controller, &motor_cb[MOTOR_REAR],
                            MOTOR_REAR_1, MOTOR_REAR_1_PIN, MOTOR_REAR_2,
                            MOTOR_REAR_2_PIN);
  MotorController_set(&motor_controller, MOTOR_FRONT, MotorStop);
  MotorController_set(&motor_controller, MOTOR_REAR, MotorStop);

  // servo
  ServoController_ctor(&servo_controller);
  // should be in the same order as taht in project_def.h
  ServoController_add_servo(&servo_controller, &servo_cb[SERVO_FL],
                            SERVO_FL_TIM, SERVO_FL_TIM_CHANNEL);
  ServoController_add_servo(&servo_controller, &servo_cb[SERVO_FR],
                            SERVO_FR_TIM, SERVO_FR_TIM_CHANNEL);
  ServoController_add_servo(&servo_controller, &servo_cb[SERVO_RL],
                            SERVO_RL_TIM, SERVO_RL_TIM_CHANNEL);
  ServoController_add_servo(&servo_controller, &servo_cb[SERVO_RR],
                            SERVO_RR_TIM, SERVO_RR_TIM_CHANNEL);
  ServoController_start(&servo_controller);

  /* adc init ----------------------------------------------------------------*/
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_buffer.data, ADC1_BUFFER_SIZE);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);

  // main init
  user_task_handle =
      xTaskCreateStatic(max_power_test, "user_task", USER_TASK_STACK_SIZE, NULL,
                        TaskPriorityNormal, user_task_stack, &user_task_cb);
}

void user_task_code(void *argument) {
  (void)argument;

  while (1) {
    vTaskDelay(1);
  }
}

/* test task -----------------------------------------------------------------*/
void path_find_test(void *argument) {
  (void)argument;

  // wait for button to be pressed
  while (1) {
    GPIO_PinState button_state;
    ButtonMonitor_read_state(&button_monitor, BUTTON_USER, &button_state);
    if (button_state == GPIO_PIN_RESET) {
      break;
    }
    vTaskDelay(10);
  }

  LedController_turn_on(&led_controller, LED_USER);

  ServoController_set_direction(&servo_controller, SERVO_FL,
                                SERVO_COUNTER_CLOCKWISE);
  ServoController_set_direction(&servo_controller, SERVO_FR, SERVO_CLOCKWISE);

  // initialize to 0 for the filters to initialize
  float grayscale_left = 0, grayscale_right = 0;
  while (1) {
    NormalizeFilter_get_filtered_data(
        &grayscale_normalize_filer[GRAYSCALE_LEFT], &grayscale_left);
    NormalizeFilter_get_filtered_data(
        &grayscale_normalize_filer[GRAYSCALE_RIGHT], &grayscale_right);

    float servo_left_duty, servo_right_duty;
    if (grayscale_left < GRAYSCALE_LOWER_THRESHOLD &&
        grayscale_right < GRAYSCALE_LOWER_THRESHOLD) {
      servo_left_duty = SPEED_STOP;
      servo_right_duty = SPEED_STOP;

    } else if (grayscale_left < GRAYSCALE_LOWER_THRESHOLD) {
      servo_left_duty = SPEED_MINIMUM;
      servo_right_duty = SPEED_MAXIMUM;

    } else if (grayscale_right < GRAYSCALE_LOWER_THRESHOLD) {
      servo_left_duty = SPEED_MAXIMUM;
      servo_right_duty = SPEED_MINIMUM;

    } else {
      float grayscale_difference_ratio = (grayscale_left - grayscale_right) /
                                         (grayscale_left + grayscale_right);

      if (grayscale_difference_ratio > 0) {
        servo_left_duty = SPEED_MAXIMUM;
        servo_right_duty =
            SPEED_MAXIMUM - SPEED_RANGE * grayscale_difference_ratio;

      } else {
        servo_left_duty =
            SPEED_MAXIMUM + SPEED_RANGE * grayscale_difference_ratio;
        servo_right_duty = SPEED_MAXIMUM;
      }
    }

    ServoController_set_duty(&servo_controller, SERVO_FL, servo_left_duty);
    ServoController_set_duty(&servo_controller, SERVO_FR, servo_right_duty);

    vTaskDelay(100);
  }
}

void max_power_test(void *argument) {
  (void)argument;

  ServoController_set_direction(&servo_controller, SERVO_FL,
                                SERVO_COUNTER_CLOCKWISE);
  ServoController_set_direction(&servo_controller, SERVO_FR, SERVO_CLOCKWISE);

  while (1) {
    GPIO_PinState button_state;
    ButtonMonitor_read_state(&button_monitor, BUTTON_USER, &button_state);
    if (button_state == GPIO_PIN_RESET) {
      LedController_turn_on(&led_controller, 0);
      ServoController_set_duty(&servo_controller, SERVO_FL, 1);
      ServoController_set_duty(&servo_controller, SERVO_FR, 1);

      MotorController_set(&motor_controller, MOTOR_FRONT, MotorForward);
      MotorController_set(&motor_controller, MOTOR_REAR, MotorForward);
    } else {
      LedController_turn_off(&led_controller, 0);
      ServoController_set_duty(&servo_controller, SERVO_FL, 0);
      ServoController_set_duty(&servo_controller, SERVO_FR, 0);

      MotorController_set(&motor_controller, MOTOR_FRONT, MotorStop);
      MotorController_set(&motor_controller, MOTOR_REAR, MotorStop);
    }
    vTaskDelay(10);
  }
}

void print_test(void *argument) {
  (void)argument;

  while (1) {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

    printf("ADC value: %ld\n", HAL_ADC_GetValue(&hadc1));
    vTaskDelay(100);
  }
}

/* callback function ---------------------------------------------------------*/
int __io_putchar(int ch) {
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc == &hadc1) {
    NormalizeFilter_update(&grayscale_normalize_filer[GRAYSCALE_LEFT],
                           adc1_buffer.grayscale_left / 4096.0, NULL);
    NormalizeFilter_update(&grayscale_normalize_filer[GRAYSCALE_RIGHT],
                           adc1_buffer.grayscale_right / 4096.0, NULL);
#ifdef PRINT_GRAYSCALE_DATA
    printf("%f\n", adc1_buffer.grayscale_left / 4096.0);
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
