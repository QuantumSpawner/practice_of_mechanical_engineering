#include "filter.h"

// glibc include
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// stm32 include
#include "main.h"

// freertos include
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

// stm32_module include
#include "stm32_module/stm32_module.h"

// project include
#include "eeprom.h"
#include "project_def.h"
#include "user_main.h"

/* private type --------------------------------------------------------------*/
typedef struct double_linked_list {
  uint16_t data;

  struct double_linked_list *prev;

  struct double_linked_list *next;
} DoubleLinkedList;

/* exported variable ---------------------------------------------------------*/
MovingAverageFilter grayscale_moving_average_filter[NUM_MOVING_AVERAGE_FILTER];
NormalizeFilter grayscale_normalize_filer[NUM_NORMALIZE_FILTER];

/* static variable -----------------------------------------------------------*/
// adc
static uint16_t adc1_buffer[NUM_ANALOG];

// filter
static float
    grayscale_moving_average_buffer[NUM_MOVING_AVERAGE_FILTER]
                                   [MOVING_AVERAGE_FILTER_MOVING_AVERAGE_SIZE];

// task control
static uint32_t
    filter_calibration_task_buffer[FILTER_CALIBRATION_TASK_STACK_SIZE];
static StaticTask_t filter_calibration_task_cb;
static TaskHandle_t filter_calibration_task_handle = NULL;

// timer control
static StaticTimer_t filter_calibration_timer_cb;
static TimerHandle_t filter_calibration_timer_handle = NULL;

// calibration data
static DoubleLinkedList lower_range[NUM_ANALOG][FILTER_CALIBRATION_HOLD_SIZE];
static DoubleLinkedList *lower_range_head[NUM_ANALOG];

static DoubleLinkedList upper_range[NUM_ANALOG][FILTER_CALIBRATION_HOLD_SIZE];
static DoubleLinkedList *upper_range_head[NUM_ANALOG];

// control flag
static volatile bool calibration_flag = false;
static volatile bool calibrating = false;

/* static function prototype -------------------------------------------------*/
static void filter_calibration_task_code(void *argument);
static void start_filter_calibration_callback(TimerHandle_t timer);

/* task implementation -------------------------------------------------------*/
static void filter_calibration_task_code(void *argument) {
  (void)argument;

  // wait for calibration to start by pressing the button again
  while (!calibration_flag) {
    LedController_blink(&led_controller, LED_YELLOW, 250);
    vTaskDelay(500);
  }
  calibration_flag = false;

  // calibration process
  LedController_turn_on(&led_controller, LED_YELLOW);
  for (int i = 0; i < NUM_ANALOG; i++) {
    lower_range[i][0].prev = NULL;
    upper_range[i][0].prev = NULL;
    for (int j = 0; j < FILTER_CALIBRATION_HOLD_SIZE - 1; j++) {
      lower_range[i][j].data = 16383;
      lower_range[i][j].next = &lower_range[i][j + 1];
      lower_range[i][j + 1].prev = &lower_range[i][j];
      upper_range[i][j].data = 0;
      upper_range[i][j].next = &upper_range[i][j + 1];
      upper_range[i][j + 1].prev = &upper_range[i][j];
    }
    lower_range[i][FILTER_CALIBRATION_HOLD_SIZE - 1].data = 16383;
    lower_range[i][FILTER_CALIBRATION_HOLD_SIZE - 1].prev = NULL;
    upper_range[i][FILTER_CALIBRATION_HOLD_SIZE - 1].data = 0;
    upper_range[i][FILTER_CALIBRATION_HOLD_SIZE - 1].prev = NULL;

    lower_range_head[i] = &lower_range[i][0];
    upper_range_head[i] = &upper_range[i][0];
  }
  calibrating = true;
  while (!calibration_flag) {
    vTaskDelay(100);
  }
  calibration_flag = false;

  // calculate the average of the upper and lower range
  uint16_t upper_range_average[NUM_ANALOG];
  uint16_t lower_range_average[NUM_ANALOG];
  for (int i = 0; i < NUM_ANALOG; i++) {
    DoubleLinkedList *temp = upper_range_head[i];
    uint32_t sum = 0;
    while (temp != NULL) {
      sum += temp->data;
      temp = temp->next;
    }
    upper_range_average[i] = sum / FILTER_CALIBRATION_HOLD_SIZE;

    temp = lower_range_head[i];
    sum = 0;
    while (temp != NULL) {
      sum += temp->data;
      temp = temp->next;
    }
    lower_range_average[i] = sum / FILTER_CALIBRATION_HOLD_SIZE;
  }

  // write to eeprom
  for (int i = 0; i < NUM_ANALOG; i++) {
    EE_WriteVariable(EEPROM_NORMALIZE_FILTER_LOWER_BOUND_ADDRESS(i),
                     lower_range_average[i]);
    EE_WriteVariable(EEPROM_NORMALIZE_FILTER_UPPER_BOUND_ADDRESS(i),
                     upper_range_average[i]);
  }

  // finish calibration
  LedController_turn_off(&led_controller, LED_YELLOW);
  filter_calibration_task_handle = NULL;
  vTaskDelete(NULL);
}

/* exported function ---------------------------------------------------------*/
void filter_init(void) {
  for (int i = 0; i < NUM_ANALOG; i++) {
#ifdef WRITE_EEPROM_TEST
    MovingAverageFilter_ctor(&grayscale_moving_average_filter[i],
                             grayscale_moving_average_buffer[i],
                             MOVING_AVERAGE_FILTER_MOVING_AVERAGE_SIZE, NULL);
    NormalizeFilter_ctor(&grayscale_normalize_filer[i], 0.0F, 1.0F,
                         (Filter *)&grayscale_moving_average_filter[i]);
#else
    uint16_t lower_bound, upper_bound;
    EE_ReadVariable(EEPROM_NORMALIZE_FILTER_LOWER_BOUND_ADDRESS(i),
                    &lower_bound);
    EE_ReadVariable(EEPROM_NORMALIZE_FILTER_UPPER_BOUND_ADDRESS(i),
                    &upper_bound);

    MovingAverageFilter_ctor(&grayscale_moving_average_filter[i],
                             grayscale_moving_average_buffer[i],
                             MOVING_AVERAGE_FILTER_MOVING_AVERAGE_SIZE, NULL);
    NormalizeFilter_ctor(&grayscale_normalize_filer[i], lower_bound / 4096.0,
                         upper_bound / 4096.0,
                         (Filter *)&grayscale_moving_average_filter[i]);
#endif  // WRITE_EEPROM_TEST
  }

  // start adc
  // dma for transfering data from adc to adc1_buffer
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_buffer, NUM_ANALOG);
  // timer8 for auto triggering adc
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
}

void filter_calibration_button_callback(void *argument,
                                        const GPIO_PinState state) {
  (void)argument;

#ifdef WRITE_EEPROM_TEST
  if (state == GPIO_PIN_RESET) {
    for (int i = 0; i < NUM_ANALOG; i++) {
      EE_WriteVariable(EEPROM_NORMALIZE_FILTER_LOWER_BOUND_ADDRESS(i), 4096);
      EE_WriteVariable(EEPROM_NORMALIZE_FILTER_UPPER_BOUND_ADDRESS(i), 0);
    }
  }
#else
  if (state == GPIO_PIN_SET) {
    if (filter_calibration_task_handle == NULL) {
      filter_calibration_timer_handle = xTimerCreateStatic(
          "filter_calibration_timer", 2000, pdFALSE, NULL,
          start_filter_calibration_callback, &filter_calibration_timer_cb);
      xTimerStart(filter_calibration_timer_handle, 0);
    } else {
      calibration_flag = true;
    }
  } else {
    if (filter_calibration_timer_handle != NULL) {
      xTimerStop(filter_calibration_timer_handle, 0);
      filter_calibration_timer_handle = NULL;
    }
  }
#endif  // WRITE_EEPROM_TEST
}

/* callback function ---------------------------------------------------------*/
static void start_filter_calibration_callback(TimerHandle_t timer) {
  (void)timer;

  filter_calibration_task_handle = xTaskCreateStatic(
      filter_calibration_task_code, "filter_calibration",
      FILTER_CALIBRATION_TASK_STACK_SIZE, NULL, TaskPriorityNormal,
      filter_calibration_task_buffer, &filter_calibration_task_cb);

  xTimerStop(filter_calibration_timer_handle, 0);
  filter_calibration_timer_handle = NULL;
}

static void adc_complete_deffered_callback(void *argment1, uint32_t argument2) {
  (void)argment1;
  (void)argument2;

  // copy adc1_buffer to adc_data to prevent adc1_buffer from being modified by
  // dma while processing
  uint16_t adc_data[NUM_ANALOG];
  memcpy(adc_data, adc1_buffer, sizeof(adc_data));

  // update filter
  for (int i = 0; i < NUM_ANALOG; i++) {
    NormalizeFilter_update(&grayscale_normalize_filer[i], adc_data[i] / 4096.0F,
                           NULL);
  }

#ifdef PRINT_RAW_ANALOG_DATA
  for (int i = 0; i < NUM_ANALOG - 1; i++) {
    printf("%g ", adc_data[i] / 4096.0F);
  }
  printf("%g\n", adc_data[NUM_ANALOG - 1] / 4096.0F);
#elif defined(PRINT_FILTERED_ANALOG_DATA)
  float data;
  for (int i = 0; i < NUM_ANALOG - 1; i++) {
    NormalizeFilter_get_filtered_data(&grayscale_normalize_filer[i], &data);
    printf("%g ", data);
  }
  NormalizeFilter_get_filtered_data(&grayscale_normalize_filer[NUM_ANALOG - 1],
                                    &data);
  printf("%g\n", data);
#endif  // PRINT_RAW_ANALOG_DATA

  if (calibrating) {
    for (int i = 0; i < NUM_ANALOG; i++) {
      if (adc_data[i] < lower_range_head[i]->data) {
        lower_range_head[i]->data = adc_data[i];

        // if need to move the head
        if (adc_data[i] < lower_range_head[i]->next->data) {
          DoubleLinkedList *temp = lower_range_head[i]->next->next;
          bool move_to_tail = false;
          while (adc_data[i] < temp->data) {
            if (temp->next == NULL) {
              move_to_tail = true;
              break;
            }
            temp = temp->next;
          }

          DoubleLinkedList *new_head = lower_range_head[i]->next;
          if (move_to_tail) {
            lower_range_head[i]->next = NULL;
            lower_range_head[i]->prev = temp;
            temp->next = lower_range_head[i];
          } else {
            lower_range_head[i]->next = temp;
            lower_range_head[i]->prev = temp->prev;
            temp->prev->next = lower_range_head[i];
            temp->prev = lower_range_head[i];
          }
          lower_range_head[i] = new_head;
        }
      }

      if (adc_data[i] > upper_range_head[i]->data) {
        upper_range_head[i]->data = adc_data[i];

        // if need to move the head
        if (adc_data[i] > upper_range_head[i]->next->data) {
          DoubleLinkedList *temp = upper_range_head[i]->next->next;
          bool move_to_tail = false;
          while (adc_data[i] > temp->data) {
            if (temp->next == NULL) {
              move_to_tail = true;
              break;
            }
            temp = temp->next;
          }

          DoubleLinkedList *new_head = upper_range_head[i]->next;
          if (move_to_tail) {
            upper_range_head[i]->next = NULL;
            upper_range_head[i]->prev = temp;
            temp->next = upper_range_head[i];
          } else {
            upper_range_head[i]->next = temp;
            upper_range_head[i]->prev = temp->prev;
            temp->prev->next = upper_range_head[i];
            temp->prev = upper_range_head[i];
          }
          upper_range_head[i] = new_head;
        }
      }
    }
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc == &hadc1) {
    BaseType_t require_contex_switch = pdFALSE;
    xTimerPendFunctionCallFromISR(adc_complete_deffered_callback, NULL, 0,
                                  &require_contex_switch);
    portYIELD_FROM_ISR(require_contex_switch);
  }
}
