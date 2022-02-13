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

    set_status_memory();

}

#define CONV_U32TYPE(x) (*((uint32_t*)&(x)))

void BldcServoManager::set_status_memory(){
    switch(u8_status_type){
        case 0x00:
            u32_status_memory[0] = 0;
            u32_status_memory[1] = 0;
            u32_status_memory[2] = 0;
            u32_status_memory[3] = 0;
            break;
        case 0x01:
            u32_status_memory[0] = CONV_U32TYPE(config_.p_bldc->fl_calc_Iq_tgt_);
            u32_status_memory[1] = CONV_U32TYPE(config_.p_bldc->fl_calc_Id_tgt_);
            u32_status_memory[2] = CONV_U32TYPE(config_.p_bldc->fl_calc_Iq_meas_);
            u32_status_memory[3] = CONV_U32TYPE(config_.p_bldc->fl_calc_Id_meas_);
            break;
        default:
            u32_status_memory[0] = 0;
            u32_status_memory[1] = 0;
            u32_status_memory[2] = 0;
            u32_status_memory[3] = 0;
            break;

    }
}
