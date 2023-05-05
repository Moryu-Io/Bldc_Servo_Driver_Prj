#include "bldc_drive_method.hpp"
#include "mymath.hpp"
#include "debug_printf.hpp"

void BldcDriveMethod6Step::update() {
  drive_6step();
}

void BldcDriveMethod6Step::itr_callback() {
  drive_6step();
}

void BldcDriveMethod6Step::drive_6step() {
  uint8_t _h_state = p_bldc_->get_hall_state();

  BLDC::DriveDuty _duty = {};

  if(InRef_.Vq >= 0) {
    switch(_h_state) {
    case 1:
      /* hall_UがHigh、U→Wに電流 */
      _duty.u8_U_out_enable = DRIVE_OUT_BOTH_ENABLE;
      _duty.u8_V_out_enable = DRIVE_OUT_DISABLE;
      _duty.u8_W_out_enable = DRIVE_OUT_LOW_ENABLE;
      _duty.Duty.U          = InRef_.Vq;
      _duty.Duty.V          = 0.0f;
      _duty.Duty.W          = 0.0f;
      break;
    case 3:
      /* hall_U,VがHigh、V→Wに電流 */
      _duty.u8_U_out_enable = DRIVE_OUT_DISABLE;
      _duty.u8_V_out_enable = DRIVE_OUT_BOTH_ENABLE;
      _duty.u8_W_out_enable = DRIVE_OUT_LOW_ENABLE;
      _duty.Duty.U          = 0.0f;
      _duty.Duty.V          = InRef_.Vq;
      _duty.Duty.W          = 0.0f;
      break;
    case 2:
      /* hall_VがHigh、V→Uに電流 */
      _duty.u8_U_out_enable = DRIVE_OUT_LOW_ENABLE;
      _duty.u8_V_out_enable = DRIVE_OUT_BOTH_ENABLE;
      _duty.u8_W_out_enable = DRIVE_OUT_DISABLE;
      _duty.Duty.U          = 0.0f;
      _duty.Duty.V          = InRef_.Vq;
      _duty.Duty.W          = 0.0f;
      break;
    case 6:
      /* hall_V,WがHigh、W→Uに電流 */
      _duty.u8_U_out_enable = DRIVE_OUT_LOW_ENABLE;
      _duty.u8_V_out_enable = DRIVE_OUT_DISABLE;
      _duty.u8_W_out_enable = DRIVE_OUT_BOTH_ENABLE;
      _duty.Duty.U          = 0.0f;
      _duty.Duty.V          = 0.0f;
      _duty.Duty.W          = InRef_.Vq;
      break;
    case 4:
      /* hall_WがHigh、W→Vに電流 */
      _duty.u8_U_out_enable = DRIVE_OUT_DISABLE;
      _duty.u8_V_out_enable = DRIVE_OUT_LOW_ENABLE;
      _duty.u8_W_out_enable = DRIVE_OUT_BOTH_ENABLE;
      _duty.Duty.U          = 0.0f;
      _duty.Duty.V          = 0.0f;
      _duty.Duty.W          = InRef_.Vq;
      break;
    case 5:
      /* hall_U,WがHigh、U→Vに電流 */
      _duty.u8_U_out_enable = DRIVE_OUT_BOTH_ENABLE;
      _duty.u8_V_out_enable = DRIVE_OUT_LOW_ENABLE;
      _duty.u8_W_out_enable = DRIVE_OUT_DISABLE;
      _duty.Duty.U          = InRef_.Vq;
      _duty.Duty.V          = 0.0f;
      _duty.Duty.W          = 0.0f;
      break;
    default:
      break;
    }
  } else {
    switch(_h_state) {
    case 1:
      /* hall_UがHigh、W→Uに電流 */
      _duty.u8_U_out_enable = DRIVE_OUT_LOW_ENABLE;
      _duty.u8_V_out_enable = DRIVE_OUT_DISABLE;
      _duty.u8_W_out_enable = DRIVE_OUT_BOTH_ENABLE;
      _duty.Duty.U          = 0.0f;
      _duty.Duty.V          = 0.0f;
      _duty.Duty.W          = -InRef_.Vq;
      break;
    case 3:
      /* hall_U,VがHigh、W→Vに電流 */
      _duty.u8_U_out_enable = DRIVE_OUT_DISABLE;
      _duty.u8_V_out_enable = DRIVE_OUT_LOW_ENABLE;
      _duty.u8_W_out_enable = DRIVE_OUT_BOTH_ENABLE;
      _duty.Duty.U          = 0.0f;
      _duty.Duty.V          = 0.0f;
      _duty.Duty.W          = -InRef_.Vq;
      break;
    case 2:
      /* hall_VがHigh、U→Vに電流 */
      _duty.u8_U_out_enable = DRIVE_OUT_BOTH_ENABLE;
      _duty.u8_V_out_enable = DRIVE_OUT_LOW_ENABLE;
      _duty.u8_W_out_enable = DRIVE_OUT_DISABLE;
      _duty.Duty.U          = -InRef_.Vq;
      _duty.Duty.V          = 0.0f;
      _duty.Duty.W          = 0.0f;
      break;
    case 6:
      /* hall_V,WがHigh、U→Wに電流 */
      _duty.u8_U_out_enable = DRIVE_OUT_BOTH_ENABLE;
      _duty.u8_V_out_enable = DRIVE_OUT_DISABLE;
      _duty.u8_W_out_enable = DRIVE_OUT_LOW_ENABLE;
      _duty.Duty.U          = -InRef_.Vq;
      _duty.Duty.V          = 0.0f;
      _duty.Duty.W          = 0.0f;
      break;
    case 4:
      /* hall_WがHigh、V→Wに電流 */
      _duty.u8_U_out_enable = DRIVE_OUT_DISABLE;
      _duty.u8_V_out_enable = DRIVE_OUT_BOTH_ENABLE;
      _duty.u8_W_out_enable = DRIVE_OUT_LOW_ENABLE;
      _duty.Duty.U          = 0.0f;
      _duty.Duty.V          = -InRef_.Vq;
      _duty.Duty.W          = 0.0f;
      break;
    case 5:
      /* hall_U,WがHigh、V→Uに電流 */
      _duty.u8_U_out_enable = DRIVE_OUT_LOW_ENABLE;
      _duty.u8_V_out_enable = DRIVE_OUT_BOTH_ENABLE;
      _duty.u8_W_out_enable = DRIVE_OUT_DISABLE;
      _duty.Duty.U          = 0.0f;
      _duty.Duty.V          = -InRef_.Vq;
      _duty.Duty.W          = 0.0f;
      break;
    default:
      break;
    }
  }

  p_bldc_->set_drive_duty(_duty);
}

void BldcDriveMethodSine::update() {
  float _e_ang = mymath::normalize_rad_0to2pi(
                  mymath::deg2rad( p_bldc_->get_elec_angle() ) );
  float _cos   = mymath::cosf(_e_ang);
  float _sin   = mymath::sinf(_e_ang);

  float _Va          = InRef_.Vd * _cos - InRef_.Vq * _sin;
  float _Vb          = InRef_.Vd * _sin + InRef_.Vq * _cos;

  BLDC::DriveDuty _duty = {
      .u8_U_out_enable = DRIVE_OUT_BOTH_ENABLE,
      .u8_V_out_enable = DRIVE_OUT_BOTH_ENABLE,
      .u8_W_out_enable = DRIVE_OUT_BOTH_ENABLE,
      .Duty            = {},
  };

  conv_sv(_Va, _Vb, _duty.Duty);

  p_bldc_->set_drive_duty(_duty);
}


void BldcDriveMethodSine::conv_sv(float _Va, float _Vb, BLDC::DrivePhase& _duty_uvw) {
  float _sct_ang_deg = mymath::rad2deg(mymath::atan2f(_Vb, _Va));

  if(_sct_ang_deg < -120.0f) {
    /* Sector 3 */
    float _V1 = -_Va + _Vb * 0.57735027f;  // -Va + (Vb/cos(30deg))*sin(30deg)
    float _V2 = -_Vb * 1.154700539f;       // -Vb / cos(30deg)
    float _V3 = (p_bldc_->get_Vm() - (_V1 + _V2)) * 0.5f;
    _duty_uvw.U = _V3;
    _duty_uvw.V = _V1 + _V3;
    _duty_uvw.W = _V1 + _V2 + _V3;
  } else if(_sct_ang_deg < -60.0f) {
    /* Sector 4 */
    float _V1 = -_Va - _Vb * 0.57735027f; // -Va - (Vb/cos(30deg))*sin(30deg)
    float _V2 =  _Va - _Vb * 0.57735027f; //  Va - (Vb/cos(30deg))*sin(30deg)
    float _V3 = (p_bldc_->get_Vm() - (_V1 + _V2)) * 0.5f;
    _duty_uvw.U = _V2 + _V3;
    _duty_uvw.V = _V3;
    _duty_uvw.W = _V1 + _V2 +_V3;
  } else if(_sct_ang_deg < 0.0f) {
    /* Sector 5 */
    float _V1 = _Va + _Vb * 0.57735027f;   // Va + (Vb/cos(30deg))*sin(30deg)
    float _V2 = -_Vb * 1.154700539f;       // -Vb / cos(30deg)
    float _V3 = (p_bldc_->get_Vm() - (_V1 + _V2)) * 0.5f;
    _duty_uvw.U = _V1 + _V2 + _V3;
    _duty_uvw.V = _V3;
    _duty_uvw.W = _V2 + _V3;
  } else  if(_sct_ang_deg < 60.0f) {
    /* Sector 0 */
    float _V1 = _Va - _Vb * 0.57735027f; // Va - (Vb/cos(30deg))*sin(30deg)
    float _V2 = _Vb * 1.154700539f;      // Vb / cos(30deg)
    float _V3 = (p_bldc_->get_Vm() - (_V1 + _V2)) * 0.5f;
    _duty_uvw.U = _V1 + _V2 + _V3;
    _duty_uvw.V = _V2 + _V3;
    _duty_uvw.W = _V3;
  } else if(_sct_ang_deg < 120.0f) {
    /* Sector 1 */
    float _V1 =  _Va + _Vb * 0.57735027f; //  Va + (Vb/cos(30deg))*sin(30deg)
    float _V2 = -_Va + _Vb * 0.57735027f; // -Va + (Vb/cos(30deg))*sin(30deg)
    float _V3 = (p_bldc_->get_Vm() - (_V1 + _V2)) * 0.5f;
    _duty_uvw.U = _V1 + _V3;
    _duty_uvw.V = _V1 + _V2 + _V3;
    _duty_uvw.W = _V3;
  } else {
    /* Sector 2 */
    float _V1 = _Vb * 1.154700539f;       //  Vb / cos(30deg)
    float _V2 = -_Va - _Vb * 0.57735027f; // -Va - (Vb/cos(30deg))*sin(30deg)
    float _V3 = (p_bldc_->get_Vm() - (_V1 + _V2)) * 0.5f;
    _duty_uvw.U = _V3;
    _duty_uvw.V = _V1 + _V2 + _V3;
    _duty_uvw.W = _V2 + _V3;
  }

  fl_outpwm_ang_deg_ = _sct_ang_deg;
}

void BldcDriveMethodVector::update() {
  float _e_ang = mymath::normalize_rad_0to2pi(
                  mymath::deg2rad( p_bldc_->get_elec_angle() ) );
  float _cos   = mymath::cosf(_e_ang);
  float _sin   = mymath::sinf(_e_ang);

  BLDC::DrivePhase nowCurr = p_bldc_->get_current();

  /* Clark変換 */
  float _Ia = 0.0f;
  float _Ib = 0.0f;
  if(fl_outpwm_ang_deg_ < -60.0f){
    /* Sector 3,4 はW線を使わない */
    _Ia = nowCurr.U;
    _Ib = (nowCurr.U + 2.0f*nowCurr.V) * 0.57735027f; // (U + 2V) / sqrt(3)
  }else if(fl_outpwm_ang_deg_ < 60.0f){
    /* Sector 5,0 はU線を使わない */
    _Ia = -(nowCurr.V + nowCurr.W);
    _Ib = (nowCurr.V - nowCurr.W) * 0.57735027f; // (V - W) / sqrt(3)
  } else {
    /* Sector 1,2 はV線を使わない */
    _Ia = nowCurr.U;
    _Ib = -(nowCurr.U + 2.0f*nowCurr.W) * 0.57735027f; // -(U + 2W) / sqrt(3)
  }

  /* Park変換 */
  float _Id =  _Ia * _cos + _Ib * _sin;
  float _Iq = -_Ia * _sin + _Ib * _cos;

  /* 電流PI制御量計算 */
  pid_id.set_target(InRef_.Id);
  pid_iq.set_target(InRef_.Iq);
  float _vm = p_bldc_->get_Vm() * 0.8660254f;
  float _Vd = mymath::satf(pid_id.update(_Id), _vm, -_vm);
  float _Vq = mymath::satf(pid_iq.update(_Iq), _vm, -_vm);

  /* 情報の保存 */
  p_bldc_->fl_calc_Iq_meas_ = _Iq;
  p_bldc_->fl_calc_Id_meas_ = _Id;
  p_bldc_->fl_calc_Iq_tgt_  = InRef_.Iq;
  p_bldc_->fl_calc_Id_tgt_  = InRef_.Id;
  p_bldc_->fl_calc_Vq_      = _Vq;
  p_bldc_->fl_calc_Vd_      = _Vd;

  /* 逆Park変換 */
  float _Va          = _Vd * _cos - _Vq * _sin;
  float _Vb          = _Vd * _sin + _Vq * _cos;

  BLDC::DriveDuty _duty = {
      .u8_U_out_enable = DRIVE_OUT_BOTH_ENABLE,
      .u8_V_out_enable = DRIVE_OUT_BOTH_ENABLE,
      .u8_W_out_enable = DRIVE_OUT_BOTH_ENABLE,
      .Duty            = {},
  };

  conv_sv(_Va, _Vb, _duty.Duty);

  p_bldc_->set_drive_duty(_duty);
}


void BldcDriveMethodSineWithCurr::update() {
  float _e_ang = mymath::normalize_rad_0to2pi(
                  mymath::deg2rad( p_bldc_->get_elec_angle() ) );
  float _cos   = mymath::cosf(_e_ang);
  float _sin   = mymath::sinf(_e_ang);

  float _Va          = InRef_.Vd * _cos - InRef_.Vq * _sin;
  float _Vb          = InRef_.Vd * _sin + InRef_.Vq * _cos;

  BLDC::DriveDuty _duty = {
      .u8_U_out_enable = DRIVE_OUT_BOTH_ENABLE,
      .u8_V_out_enable = DRIVE_OUT_BOTH_ENABLE,
      .u8_W_out_enable = DRIVE_OUT_BOTH_ENABLE,
      .Duty            = {},
  };

  conv_sv(_Va, _Vb, _duty.Duty);

  p_bldc_->set_drive_duty(_duty);

  BLDC::DrivePhase nowCurr = p_bldc_->get_current();

  /* Clark変換 */
  float _Ia = 0.0f;
  float _Ib = 0.0f;
  if(fl_outpwm_ang_deg_ < -60.0f){
    /* Sector 3,4 はW線を使わない */
    _Ia = nowCurr.U;
    _Ib = (nowCurr.U + 2.0f*nowCurr.V) * 0.57735027f; // (U + 2V) / sqrt(3)
  }else if(fl_outpwm_ang_deg_ < 60.0f){
    /* Sector 5,0 はU線を使わない */
    _Ia = -(nowCurr.V + nowCurr.W);
    _Ib = (nowCurr.V - nowCurr.W) * 0.57735027f; // (V - W) / sqrt(3)
  } else {
    /* Sector 1,2 はV線を使わない */
    _Ia = nowCurr.U;
    _Ib = -(nowCurr.U + 2.0f*nowCurr.W) * 0.57735027f; // -(U + 2W) / sqrt(3)
  }

  /* Park変換 */
  float _Id =  _Ia * _cos + _Ib * _sin;
  float _Iq = -_Ia * _sin + _Ib * _cos;

  /* 情報の保存 */
  p_bldc_->fl_calc_Iq_meas_ = _Iq;
  p_bldc_->fl_calc_Id_meas_ = _Id;
  p_bldc_->fl_calc_Iq_tgt_  = InRef_.Iq;
  p_bldc_->fl_calc_Id_tgt_  = InRef_.Id;
  p_bldc_->fl_calc_Vq_      = InRef_.Vq;
  p_bldc_->fl_calc_Vd_      = InRef_.Vd;
}
