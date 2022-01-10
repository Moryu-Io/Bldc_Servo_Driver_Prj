#ifndef SERVO_DRIVER_MDOEL_HPP_
#define SERVO_DRIVER_MDOEL_HPP_

#include "main.h"
#include "com_base.hpp"

COM_BASE* get_debug_com();

void initialize_servo_driver_model();
void loop_servo_driver_model();

#endif