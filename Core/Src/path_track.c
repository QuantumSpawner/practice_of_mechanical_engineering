#include "path_track.h"

// glibc include
#include <stdint.h>

// stm32 include
#include "main.h"

// stm32_module include
#include "stm32_module/stm32_module.h"

// project include
#include "filter.h"
#include "project_def.h"
#include "user_main.h"

void path_track_once(void) {
  // initialize to 0 for the filters to initialize
  static float sensor_data[NUM_ANALOG] = {0};

  for (int i = 0; i < NUM_ANALOG; i++) {
    NormalizeFilter_get_filtered_data(&grayscale_normalize_filer[i],
                                      &sensor_data[i]);
  }

#ifdef SIMPLE_PATH_TRACK
  float servo_left_duty, servo_right_duty;
  if (sensor_data[ANALOG_FL1] > ANALOG_LOWER_THRESHOLD &&
      sensor_data[ANALOG_FR1] > ANALOG_LOWER_THRESHOLD) {
    servo_left_duty = SERVO_SPEED_MAXIMUM;
    servo_right_duty = SERVO_SPEED_MAXIMUM;

  } else if (sensor_data[ANALOG_FL1] > ANALOG_LOWER_THRESHOLD) {
    servo_left_duty = SERVO_SPEED_MINIMUM;
    servo_right_duty = SERVO_SPEED_MAXIMUM;

  } else if (sensor_data[ANALOG_FR1] > ANALOG_LOWER_THRESHOLD) {
    servo_left_duty = SERVO_SPEED_MAXIMUM;
    servo_right_duty = SERVO_SPEED_MINIMUM;

  } else {
    float grayscale_difference_ratio =
        (sensor_data[ANALOG_FL1] == sensor_data[ANALOG_FR1])
            ? 0
            : (sensor_data[ANALOG_FR1] - sensor_data[ANALOG_FL1]) /
                  (sensor_data[ANALOG_FL1] + sensor_data[ANALOG_FR1]);

    if (grayscale_difference_ratio > 0) {
      servo_left_duty = SERVO_SPEED_MAXIMUM;
      servo_right_duty =
          SERVO_SPEED_MAXIMUM - SERVO_SPEED_REAGE * grayscale_difference_ratio;

    } else {
      servo_left_duty =
          SERVO_SPEED_MAXIMUM + SERVO_SPEED_REAGE * grayscale_difference_ratio;
      servo_right_duty = SERVO_SPEED_MAXIMUM;
    }
  }

  if (servo_left_duty > 0) {
    ServoController_set_direction(&servo_controller, SERVO_WHEEL_FL,
                                  SERVO_COUNTER_CLOCKWISE);
    ServoController_set_direction(&servo_controller, SERVO_WHEEL_RL,
                                  SERVO_COUNTER_CLOCKWISE);
    ServoController_set_duty(&servo_controller, SERVO_WHEEL_FL,
                             servo_left_duty);
    ServoController_set_duty(&servo_controller, SERVO_WHEEL_RL,
                             servo_left_duty);
  } else {
    ServoController_set_direction(&servo_controller, SERVO_WHEEL_FL,
                                  SERVO_CLOCKWISE);
    ServoController_set_direction(&servo_controller, SERVO_WHEEL_RL,
                                  SERVO_CLOCKWISE);
    ServoController_set_duty(&servo_controller, SERVO_WHEEL_FL,
                             -servo_left_duty);
    ServoController_set_duty(&servo_controller, SERVO_WHEEL_RL,
                             -servo_left_duty);
  }

  if (servo_right_duty > 0) {
    ServoController_set_direction(&servo_controller, SERVO_WHEEL_FR,
                                  SERVO_CLOCKWISE);
    ServoController_set_direction(&servo_controller, SERVO_WHEEL_RR,
                                  SERVO_CLOCKWISE);
    ServoController_set_duty(&servo_controller, SERVO_WHEEL_FR,
                             servo_right_duty);
    ServoController_set_duty(&servo_controller, SERVO_WHEEL_RR,
                             servo_right_duty);
  } else {
    ServoController_set_direction(&servo_controller, SERVO_WHEEL_FR,
                                  SERVO_COUNTER_CLOCKWISE);
    ServoController_set_direction(&servo_controller, SERVO_WHEEL_RR,
                                  SERVO_COUNTER_CLOCKWISE);
    ServoController_set_duty(&servo_controller, SERVO_WHEEL_FR,
                             -servo_right_duty);
    ServoController_set_duty(&servo_controller, SERVO_WHEEL_RR,
                             -servo_right_duty);
  }

#else

#endif  // SIMPLE_PATH_TRACK
}
