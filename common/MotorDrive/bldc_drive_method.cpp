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
  float _sct_ang_deg = mymath::rad2deg(mymath::atan2f(_Vb, _Va));

  
  debug_printf("%d,%d\n", (int)(_sct_ang_deg), (int)(_e_ang*100));

  BLDC::DriveDuty _duty = {
      .u8_U_out_enable = DRIVE_OUT_BOTH_ENABLE,
      .u8_V_out_enable = DRIVE_OUT_BOTH_ENABLE,
      .u8_W_out_enable = DRIVE_OUT_BOTH_ENABLE,
      .Duty            = {},
  };

  if(_sct_ang_deg < -120.0f) {
    /* Sector 3 */
    float _V1 = -_Va + _Vb * 0.57735027f;  // -Va + (Vb/cos(30deg))*sin(30deg)
    float _V2 = -_Vb * 1.154700539f;       // -Vb / cos(30deg)
    float _V3 = (p_bldc_->get_Vm() - (_V1 + _V2)) * 0.5f;
    _duty.Duty.U = _V3;
    _duty.Duty.V = _V1 + _V3;
    _duty.Duty.W = _V1 + _V2 + _V3;
  } else if(_sct_ang_deg < -60.0f) {
    /* Sector 4 */
    float _V1 = -_Va - _Vb * 0.57735027f; // -Va - (Vb/cos(30deg))*sin(30deg)
    float _V2 =  _Va - _Vb * 0.57735027f; //  Va - (Vb/cos(30deg))*sin(30deg)
    float _V3 = (p_bldc_->get_Vm() - (_V1 + _V2)) * 0.5f;
    _duty.Duty.U = _V2 + _V3;
    _duty.Duty.V = _V3;
    _duty.Duty.W = _V1 + _V2 +_V3;
  } else if(_sct_ang_deg < 0.0f) {
    /* Sector 5 */
    float _V1 = _Va + _Vb * 0.57735027f;   // Va + (Vb/cos(30deg))*sin(30deg)
    float _V2 = -_Vb * 1.154700539f;       // -Vb / cos(30deg)
    float _V3 = (p_bldc_->get_Vm() - (_V1 + _V2)) * 0.5f;
    _duty.Duty.U = _V1 + _V2 + _V3;
    _duty.Duty.V = _V3;
    _duty.Duty.W = _V2 + _V3;
  } else  if(_sct_ang_deg < 60.0f) {
    /* Sector 0 */
    float _V1 = _Va - _Vb * 0.57735027f; // Va - (Vb/cos(30deg))*sin(30deg)
    float _V2 = _Vb * 1.154700539f;      // Vb / cos(30deg)
    float _V3 = (p_bldc_->get_Vm() - (_V1 + _V2)) * 0.5f;
    _duty.Duty.U = _V1 + _V2 + _V3;
    _duty.Duty.V = _V2 + _V3;
    _duty.Duty.W = _V3;
  } else if(_sct_ang_deg < 120.0f) {
    /* Sector 1 */
    float _V1 =  _Va + _Vb * 0.57735027f; //  Va + (Vb/cos(30deg))*sin(30deg)
    float _V2 = -_Va + _Vb * 0.57735027f; // -Va + (Vb/cos(30deg))*sin(30deg)
    float _V3 = (p_bldc_->get_Vm() - (_V1 + _V2)) * 0.5f;
    _duty.Duty.U = _V1 + _V3;
    _duty.Duty.V = _V1 + _V2 + _V3;
    _duty.Duty.W = _V3;
  } else {
    /* Sector 2 */
    float _V1 = _Vb * 1.154700539f;       //  Vb / cos(30deg)
    float _V2 = -_Va - _Vb * 0.57735027f; // -Va - (Vb/cos(30deg))*sin(30deg)
    float _V3 = (p_bldc_->get_Vm() - (_V1 + _V2)) * 0.5f;
    _duty.Duty.U = _V3;
    _duty.Duty.V = _V1 + _V2 + _V3;
    _duty.Duty.W = _V2 + _V3;
  }
  
  p_bldc_->set_drive_duty(_duty);
}

void BldcDriveMethodVector::update() {
}
