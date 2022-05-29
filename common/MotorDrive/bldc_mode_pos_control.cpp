#include "bldc_mode_pos_control.hpp"

void BldcModePosControl::init() {
  // 現在位置でターゲットを設定
  parts_.p_tgt_interp->set_nowtarget(P_BLDC_->get_out_angle() * (float)0x10000);
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
  switch(p_instr->u16_instr_id) {
  case INSTR_ID_POSCTR_MOVE_ANGLE:
    /*  */
    parts_.p_tgt_interp->set_target(p_instr->InstrPosCtrl_MvAng.s32_tgt_pos,
                                    p_instr->InstrPosCtrl_MvAng.s32_move_time_ms * 10);
    break;
  case INSTR_ID_POSCTR_ANGLE_INIT:
    if(p_instr->InstrPosCtrl_AngIni.u8_set_angle_flag == 0){
      /* 現在位置を0とする */
      P_BLDC_->set_ref_ont_angle(P_BLDC_->get_out_angle()); // 位置オフセット
      parts_.p_tgt_interp->set_nowtarget(0);
      parts_.p_tgt_interp->set_target(0, 0);

      /* 制御初期化 */
      parts_.p_posout_lpf->reset();
      parts_.p_pos_ctrl->reset();
    } else if(p_instr->InstrPosCtrl_AngIni.u8_set_angle_flag == 1){
      /* 現在位置を指定位置とする */
      float _fl_new_ref_ang = P_BLDC_->get_raw_out_angle() - p_instr->InstrPosCtrl_AngIni.s32_init_pos / (float)0x10000;
      P_BLDC_->set_ref_ont_angle(_fl_new_ref_ang);
      parts_.p_tgt_interp->set_nowtarget(p_instr->InstrPosCtrl_AngIni.s32_init_pos);
      parts_.p_tgt_interp->set_target(p_instr->InstrPosCtrl_AngIni.s32_init_pos, 0);

      /* 制御初期化 */
      parts_.p_posout_lpf->reset();
      parts_.p_pos_ctrl->reset();
    }

    break;
  default:
    break;
  }
}

void BldcModePosControl::get_status_1(Status *p_mem) {
  p_mem->stsPosCtrl_1.s16_tgt_ang_deg_Q4 = static_cast<int16_t>(parts_.p_tgt_interp->get_target() >> 12);
}