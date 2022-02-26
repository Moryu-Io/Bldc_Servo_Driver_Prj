#include "bldc_mode_pos_control.hpp"

void BldcModePosControl::init() {
}

void BldcModePosControl::update() {

  P_BLDC_->update();

  parts_.p_pos_ctrl->set_target((float)parts_.p_tgt_interp->update_target() / (float)0x10000);

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

void BldcModePosControl::set_Instruction(Instr *p_instr) {
  parts_.p_tgt_interp->set_target(p_instr->InstrPosCtrl.s32_tgt_pos,
                                  p_instr->InstrPosCtrl.s32_move_time_ms * 10);
}

void BldcModePosControl::get_status_1(Status *p_mem) {
  p_mem->stsPosCtrl_1.s16_tgt_ang_deg_Q4 = static_cast<int16_t>(parts_.p_tgt_interp->get_target() >> 12);
}