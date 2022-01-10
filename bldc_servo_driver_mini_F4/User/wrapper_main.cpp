#include "wrapper_main.hpp"
#include "main.h"
#include "servo_driver_model.hpp"
#include "wrapper_interrupt.hpp"

void cpp_wrapper_main_setup(void) {
  initialize_servo_driver_model();
}

void cpp_wrapper_main_loop(void) {
  loop_servo_driver_model();
}
