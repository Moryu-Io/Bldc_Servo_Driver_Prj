#ifndef SERVO_DRIVER_MDOEL_HPP_
#define SERVO_DRIVER_MDOEL_HPP_

#include "main.h"
#include "com_base.hpp"
#include "bldc.hpp"
#include "bldc_drive_method.hpp"
#include "i2cc.hpp"

BLDC* get_bldc_if();
BldcDriveMethod* get_bldcdrv_method();

I2CC* get_i2cc();
COM_BASE* get_debug_com();

void initialize_servo_driver_model();
void loop_servo_driver_model();

#endif