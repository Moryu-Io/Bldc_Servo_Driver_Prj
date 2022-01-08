#ifndef BLDC_HPP_
#define BLDC_HPP_

#include "main.h"
#include "stm32g4xx.h"

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

  virtual void init() = 0;

  virtual void hall_itr_callback(){};

  virtual void set_drive_duty(DriveDuty &_Vol) = 0;

  virtual uint8_t  get_hall_state() { return u8_now_hall_state_; };
  virtual uint32_t get_hall_count() { return u16_hall_counter_; };
  virtual float    get_elec_angle() { return 0; }
  virtual float    get_Vm()         { return 12.0f; }
  virtual float    get_VmInv()      { return 1.0f/12.0f; }

  virtual bool get_fault_state() { return false; };
  virtual bool get_ready_state() { return true; }

protected:
  uint8_t    u8_now_hall_state_;
  int8_t     s8_now_motor_dir_;
  uint16_t   u16_hall_counter_;
  DrivePhase fl_now_current_;
  DrivePhase fl_now_bev_;
};

#endif