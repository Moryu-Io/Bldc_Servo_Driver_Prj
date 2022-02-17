#include "bldc_mode_base.hpp"
#include "servo_driver_model.hpp"

BLDC* BldcModeBase::P_BLDC_;


BldcModeBase::BldcModeBase(){
    BldcModeBase::P_BLDC_ = get_bldc_if();
}


void BldcModePowerOff::update(){
    BLDC::DriveDuty _duty = {
        .u8_U_out_enable = DRIVE_OUT_BOTH_ENABLE,
        .u8_V_out_enable = DRIVE_OUT_BOTH_ENABLE,
        .u8_W_out_enable = DRIVE_OUT_BOTH_ENABLE,
        .Duty            = {},
    };

    BldcModeBase::P_BLDC_->update();
    BldcModeBase::P_BLDC_->set_drive_duty(_duty);
}

