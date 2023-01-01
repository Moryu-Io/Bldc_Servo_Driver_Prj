#ifndef SERVO_DRIVER_MDOEL_HPP_
#define SERVO_DRIVER_MDOEL_HPP_

#include "main.h"
#include "com_base.hpp"
#include "bldc.hpp"
#include "bldc_drive_method.hpp"
#include "bldc_servo_manager.hpp"
#include "bldc_mode_base.hpp"
#include "flash_interface.hpp"

BLDC* get_bldc_if();
BldcDriveMethod* get_bldcdrv_method();
BldcServoManager* get_bldcservo_manager();

BldcModeBase* get_bldcmode_off();
BldcModeBase* get_bldcmode_posctrl();

FlashIF* get_flash_if();

void set_ext_com(EXT_COM_BASE* ecom);
EXT_COM_BASE* get_ext_com();
EXT_COM_BASE* get_ext_com_default();
COM_BASE* get_debug_com();
EXT_COM_BASE* get_debug_com_pretend_ext();

void initialize_servo_driver_model();
void loop_servo_driver_model();

void set_flash_parameter_to_models();

#endif