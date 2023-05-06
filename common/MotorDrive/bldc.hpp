#ifndef BLDC_HPP_
#define BLDC_HPP_

#include "main.h"

#define DRIVE_OUT_BOTH_ENABLE (2)
#define DRIVE_OUT_LOW_ENABLE (1)
#define DRIVE_OUT_DISABLE (0)

class BLDC {
public:
  BLDC(){};

  struct DrivePhase {
    float U;
    float V;
    float W;
  };

  struct DriveDuty {
    uint8_t    u8_U_out_enable;
    uint8_t    u8_V_out_enable;
    uint8_t    u8_W_out_enable;
    DrivePhase Duty;
  };

  struct CurrentRaw {
    uint16_t U;
    uint16_t V;
    uint16_t W;
  };

  virtual void init() = 0;
  virtual void update(){};
  virtual void update_lowrate(){};
  virtual void hall_itr_callback(){};

  virtual void set_drive_duty(DriveDuty &_Vol) = 0;
  void set_current_lim(float _lim){ fl_current_lim_A_ = _lim; };

  /* 設定値書き込み */
  void set_ref_ont_angle(float _ref_ang) { fl_ref_out_ang_deg_ = _ref_ang; }
  void set_elec_angle_gain(float _eang_g) { fl_elec_angle_gain_CNTtoDeg_ = _eang_g; }
  void set_elec_angle_offset(int32_t _eang_ofs) { s32_elec_angle_offset_CNT_ = _eang_ofs; }
  void set_elec_angle_dir(int8_t _eang_dir) { s8_elec_angle_dir_ = _eang_dir; }
  void set_curr_raw_mid(CurrentRaw _curr_mid) { st_curr_raw_mid_ = _curr_mid; }

  /* 検査やデバッグ用の強制状態上書き */
  void overwrite_elec_angle_forTEST(float _eang) { fl_now_elec_ang_deg_ = _eang; }

  /* 状態取得 */
  uint8_t    get_hall_state() { return u8_now_hall_state_; }
  int32_t    get_angle_raw() { return s32_pre_angle_raw; }
  int32_t    get_angle_count() { return s32_angle_rotor_count_; }
  float      get_elec_angle() { return fl_now_elec_ang_deg_; }
  float      get_out_angle() { return fl_now_out_ang_deg_ - fl_ref_out_ang_deg_; }
  float      get_out_vel() { return fl_now_out_vel_dps_; }
  float      get_raw_out_angle() { return fl_now_out_ang_deg_; }
  float      get_ref_out_angle() { return fl_ref_out_ang_deg_; }
  float      get_Vm() { return fl_Vm_; }
  float      get_tempr_deg() { return fl_temperature_deg; }
  float      get_currlim_A() { return fl_current_lim_A_; }
  DrivePhase get_current() { return now_current_; };
  CurrentRaw get_current_raw() { return now_curr_raw_; };

  virtual bool get_fault_state() { return false; };
  virtual bool get_ready_state() { return true; }

  /* 外部で計算されたBLDC関連情報の保存場所 */
  float fl_calc_Iq_meas_;
  float fl_calc_Id_meas_;
  float fl_calc_Iq_tgt_;
  float fl_calc_Id_tgt_;
  float fl_calc_Vq_;
  float fl_calc_Vd_;

protected:
  uint8_t    u8_now_hall_state_;
  int8_t     s8_now_motor_dir_;
  int32_t    s32_pre_angle_raw;
  int32_t    s32_angle_rotor_count_;
  float      fl_now_elec_ang_deg_;
  float      fl_now_out_ang_deg_;
  float      fl_now_out_vel_dps_;
  float      fl_current_lim_A_;
  DrivePhase now_current_;
  DrivePhase now_bev_;
  CurrentRaw now_curr_raw_;

  float fl_Vm_;
  float fl_temperature_deg;

  /* 調整値 */
  float   fl_ref_out_ang_deg_;          // 関節によって異なる値
  float   fl_elec_angle_gain_CNTtoDeg_; // モータによって異なる値
  int32_t s32_elec_angle_offset_CNT_;   // モータによって異なる値
  int8_t  s8_elec_angle_dir_;           // モータによって異なる値

  CurrentRaw st_curr_raw_mid_;           // 電流AD値の中点調整値
};

#endif