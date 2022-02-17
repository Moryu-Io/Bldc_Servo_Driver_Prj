#ifndef BLDC_HPP_
#define BLDC_HPP_

#include "main.h"

#define DRIVE_OUT_BOTH_ENABLE (2)
#define DRIVE_OUT_LOW_ENABLE (1)
#define DRIVE_OUT_DISABLE (0)

class BLDC
{
public:
  BLDC(){};

  struct DrivePhase
  {
    float U;
    float V;
    float W;
  };

  struct DriveDuty
  {
    uint8_t u8_U_out_enable;
    uint8_t u8_V_out_enable;
    uint8_t u8_W_out_enable;
    DrivePhase Duty;
  };

  virtual void init() = 0;
  virtual void update(){};

  virtual void hall_itr_callback(){};

  virtual void set_drive_duty(DriveDuty &_Vol) = 0;

  uint8_t    get_hall_state() { return u8_now_hall_state_; }
  int32_t    get_angle_count(){ return s32_angle_rotor_count_; }
  float      get_elec_angle() { return fl_now_elec_ang_deg_; }
  float      get_out_angle()  { return fl_now_out_ang_deg_; }
  float      get_Vm()         { return fl_Vm_; }
  DrivePhase get_current()    { return now_current_; };

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
  uint8_t  u8_now_hall_state_;
  int8_t   s8_now_motor_dir_;
  int32_t  s32_angle_rotor_count_;
  float    fl_now_elec_ang_deg_;
  float    fl_now_out_ang_deg_;
  DrivePhase now_current_;
  DrivePhase now_bev_;

  float fl_Vm_;
  float fl_gear_ratio_inv_;

};

#endif