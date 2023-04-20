#ifndef SIMPLE_MOTOR_CONTROLLER_H
#define SIMPLE_MOTOR_CONTROLLER_H

// glibc include
#include <stdint.h>

// stm32 include
#include "stm32_module/stm32_hal.h"

// stm32_module include
#include "stm32_module/module_common.h"

/* type ----------------------------------------------------------------------*/
typedef enum motor_mode {
  MotorStop = 0,
  MotorForward,
  MotorBackward,
} MotorMode;

struct motor_cb {
  GPIO_TypeDef* motor_port1;

  GPIO_TypeDef* motor_port2;

  uint16_t motor_pin1;

  uint16_t motor_pin2;

  /// @brief List control block for tracking the list of motors.
  struct list_cb motor_list_cb;
};

/* class ---------------------------------------------------------------------*/
/**
 * @brief Class for controlling motor.
 *
 */
typedef struct motor_controller {
  // member variable
  List motor_list_;

} MotorController;

/* constructor ---------------------------------------------------------------*/
/**
 * @brief Constructor for MotorController.
 *
 * @param[in,out] self The instance of the class.
 * @return None.
 */
void MotorController_ctor(MotorController* const self);

/* member function -----------------------------------------------------------*/
/**
 * @brief Function for adding motor to motor controller.
 *
 * @param[in,out] self The instance of the class.
 * @param[in] motor_cb The motor control block.
 * @param[in] motor_port1 The port of the motor.
 * @param[in] motor_pin1 The pin of the motor.
 * @param[in] motor_port2 The port of the motor.
 * @param[in] motor_pin2 The pin of the motor.
 * @return ModuleRet Error code.
 * @note User is resposible for allocating memory for motor_cb.
 */
ModuleRet MotorController_add_motor(MotorController* const self,
                                    struct motor_cb* const motor_cb,
                                    GPIO_TypeDef* const motor_port1,
                                    const uint16_t motor_pin1,
                                    GPIO_TypeDef* const motor_port2,
                                    const uint16_t motor_pin2);

/**
 * @brief Function for setting motor mode.
 *
 * @param[in,out] self The instance of the class.
 * @param[in] motor_num The number of the motor.
 * @param[in] mode The mode of the motor.
 * @return ModuleRet Error code.
 */
ModuleRet MotorController_set(MotorController* const self, const int motor_num,
                              MotorMode mode);

#endif  // SIMPLE_MOTOR_CONTROLLER_H
