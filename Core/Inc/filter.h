#ifndef FILTER_H
#define FILTER_H

// stm32 include
#include "main.h"

// stm32_module include
#include "stm32_module/stm32_module.h"

// project include
#include "project_def.h"

// patameter
#define FILTER_CALIBRATION_TASK_STACK_SIZE 128
#define FILTER_CALIBRATION_HOLD_SIZE 20

/* exported variable ---------------------------------------------------------*/
extern MovingAverageFilter
    grayscale_moving_average_filter[NUM_MOVING_AVERAGE_FILTER];
extern NormalizeFilter grayscale_normalize_filer[NUM_NORMALIZE_FILTER];

/* exported function ---------------------------------------------------------*/
void filter_init(void);

void filter_calibration_button_callback(void *argument,
                                        const GPIO_PinState state);

#endif  // FILTER_H
