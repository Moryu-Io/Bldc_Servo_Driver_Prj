#include "bldc_mode_pos_control.hpp"

void BldcModePosControl::init()
{
}

void BldcModePosControl::update()
{

    P_BLDC_->update();

    parts_.p_posout_lpf->update(parts_.p_pos_ctrl->update(P_BLDC_->get_out_angle()));

    BldcDriveMethod::Ref inputVol = {
        .Vq = 0.0f,
        .Vd = 0.0f,
        .Iq = parts_.p_posout_lpf->get_output(),
        .Id = 0.0f,
    };
    parts_.p_bldc_drv->set(inputVol);
    parts_.p_bldc_drv->update();
}



void BldcModePosControl::get_status_1(Status *p_mem){
    p_mem->stsPosCtrl_1.s16_tgt_ang_deg_Q4 = static_cast<int16_t>(parts_.p_tgt_interp->get_target() >> 12);
}