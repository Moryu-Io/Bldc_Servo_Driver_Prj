#include "bldc_servo_manager.hpp"

void BldcServoManager::update(){
    if(!is_servo_enable){
        BLDC::DriveDuty _duty = {
            .u8_U_out_enable = DRIVE_OUT_BOTH_ENABLE,
            .u8_V_out_enable = DRIVE_OUT_BOTH_ENABLE,
            .u8_W_out_enable = DRIVE_OUT_BOTH_ENABLE,
            .Duty            = {},
        };
        config_.p_bldc->set_drive_duty(_duty);
    } else {

        config_.p_bldc->update();

        config_.p_posout_lpf->update(config_.p_pos_ctrl->update(config_.p_bldc->get_angle()));

        BldcDriveMethod::Ref inputVol = {
            .Vq = 0.0f,
            .Vd = 0.0f,
            .Iq = config_.p_posout_lpf->get_output(),
            .Id = 0.0f,
        };
        config_.p_bldc_drv->set(inputVol);
        config_.p_bldc_drv->update();
    }

}