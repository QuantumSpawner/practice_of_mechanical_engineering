#include "simple_motor_controller.h"

// glibc include
#include <stdint.h>

// stm32 include
#include "stm32_module/stm32_hal.h"

// freertos include
#include "FreeRTOS.h"

// stm32_module include
#include "stm32_module/module_common.h"

/* constructor ---------------------------------------------------------------*/
void MotorController_ctor(MotorController* const self) {
  module_assert(IS_NOT_NULL(self));

  // initialize member variable
  List_ctor(&self->motor_list_);
}
/* member function -----------------------------------------------------------*/
ModuleRet MotorController_add_motor(MotorController* const self,
                                    struct motor_cb* const motor_cb,
                                    GPIO_TypeDef* const motor_port1,
                                    const uint16_t motor_pin1,
                                    GPIO_TypeDef* const motor_port2,
                                    const uint16_t motor_pin2) {
  module_assert(IS_NOT_NULL(self));
  module_assert(IS_NOT_NULL(motor_cb));
  module_assert(IS_GPIO_ALL_INSTANCE(motor_port1));
  module_assert(IS_GPIO_PIN(motor_pin1));
  module_assert(IS_GPIO_ALL_INSTANCE(motor_port2));
  module_assert(IS_GPIO_PIN(motor_pin2));

  motor_cb->motor_port1 = motor_port1;
  motor_cb->motor_pin1 = motor_pin1;
  motor_cb->motor_port2 = motor_port2;
  motor_cb->motor_pin2 = motor_pin2;

  // add to list
  taskENTER_CRITICAL();
  List_push_back(&self->motor_list_, &motor_cb->motor_list_cb, (void*)motor_cb);
  taskEXIT_CRITICAL();
  return ModuleOK;
}

ModuleRet MotorController_set(MotorController* const self, const int motor_num,
                              MotorMode mode) {
  module_assert(IS_NOT_NULL(self));

  if (List_size(&self->motor_list_) <= motor_num) {
    return ModuleError;
  }

  taskENTER_CRITICAL();
  struct motor_cb* motor_cb =
      (struct motor_cb*)List_at(&self->motor_list_, motor_num);
  HAL_GPIO_WritePin(motor_cb->motor_port1, motor_cb->motor_pin1,
                    (mode == MotorForward) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(motor_cb->motor_port2, motor_cb->motor_pin2,
                    (mode == MotorBackward) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  taskEXIT_CRITICAL();
  return ModuleOK;
}
