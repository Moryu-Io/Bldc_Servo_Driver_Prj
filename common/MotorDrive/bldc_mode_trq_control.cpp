#include "bldc_mode_trq_control.hpp"
#include "mymath.hpp"

void BldcModeTrqControl::init() {
  // 現在位置でターゲットを設定
  fl_tgt_Iq_A_ = 0;
  fl_tgt_Id_A_ = 0;
}

void BldcModeTrqControl::update() {
  P_BLDC_->update();

  BldcDriveMethod::Ref inputCurr = {
      .Vq = 0.0f,
      .Vd = 0.0f,
      .Iq = fl_tgt_Iq_A_,
      .Id = fl_tgt_Id_A_,
  };
  parts_.p_bldc_drv->set(inputCurr);
  parts_.p_bldc_drv->update();
}

void BldcModeTrqControl::set_Instruction(Instr *p_instr) {
  switch(p_instr->u16_instr_id) {
  case INSTR_ID_TRQCTR_SET_TARGET:
    {
    fl_tgt_Iq_A_ = p_instr->InstrTrqCtrl_SetTgt.fl_tgt_Iq_A;
    fl_tgt_Id_A_ = p_instr->InstrTrqCtrl_SetTgt.fl_tgt_Id_A;
    float _I_lim = P_BLDC_->get_currlim_A();
    fl_tgt_Iq_A_ = mymath::satf(fl_tgt_Iq_A_, _I_lim, -_I_lim);
    fl_tgt_Id_A_ = mymath::satf(fl_tgt_Id_A_, _I_lim, -_I_lim);
    }
    break;
  default:
    break;
  }
}

void BldcModeTrqControl::get_status_1(Status *p_mem) {
}