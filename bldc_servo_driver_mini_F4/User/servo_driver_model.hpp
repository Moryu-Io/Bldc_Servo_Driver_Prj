#ifndef SERVO_DRIVER_MDOEL_HPP_
#define SERVO_DRIVER_MDOEL_HPP_

#include "main.h"
#include "com_base.hpp"
#include "bldc.hpp"
#include "bldc_drive_method.hpp"
#include "bldc_servo_manager.hpp"

BLDC* get_bldc_if();
BldcDriveMethod* get_bldcdrv_method();
BldcServoManager* get_bldcservo_manager();

COM_BASE* get_debug_com();

void initialize_servo_driver_model();
void loop_servo_driver_model();

#endif