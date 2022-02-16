#include "bldc_mode_pos_control.hpp"


void BldcModePosControl::init(){

}


void BldcModePosControl::update(){
    P_BLDC_->update();

    parts_.p_posout_lpf->update(parts_.p_pos_ctrl->update(P_BLDC_->get_angle()));

    BldcDriveMethod::Ref inputVol = {
        .Vq = 0.0f,
        .Vd = 0.0f,
        .Iq = parts_.p_posout_lpf->get_output(),
        .Id = 0.0f,
    };
    parts_.p_bldc_drv->set(inputVol);
    parts_.p_bldc_drv->update();

}

