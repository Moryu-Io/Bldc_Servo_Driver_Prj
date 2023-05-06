#ifndef BLDC_DRIVE_METHOD_HPP_
#define BLDC_DRIVE_METHOD_HPP_

#include "bldc.hpp"
#include "controller.hpp"

class BldcDriveMethod {
public:
  BldcDriveMethod(BLDC *_bldc) : p_bldc_(_bldc){};

  struct Ref {
    float Vq;
    float Vd;
    float Iq;
    float Id;
  };

  /* 入力値 */
  virtual void set(Ref &_ref) { InRef_ = _ref; };
          Ref  get()          { return InRef_; };

  /* 更新処理関数 */
  virtual void update() = 0;

  /* 割込用の関数(必要に応じて) */
  virtual void itr_callback(){};

protected:
  Ref InRef_;

  BLDC *p_bldc_;

};

class BldcDriveMethod6Step : public BldcDriveMethod {
public:
  BldcDriveMethod6Step(BLDC *_bldc) : BldcDriveMethod(_bldc){};

  void update() override;
  void itr_callback() override;

protected:
  void drive_6step();

};

class BldcDriveMethodSine : public BldcDriveMethod {
public:
  BldcDriveMethodSine(BLDC *_bldc) : BldcDriveMethod(_bldc){};

  void update() override;

protected:
  void conv_sv(float _Va, float _Vb, BLDC::DrivePhase& _duty_uvw);

  float fl_outpwm_ang_deg_;
};

class BldcDriveMethodVector : public BldcDriveMethodSine {
public:
  BldcDriveMethodVector(BLDC *_bldc, float _c_f=10000.0f) : BldcDriveMethodSine(_bldc),
    pid_iq(_c_f, (PID::Gain){8.0f, 10000.0f, 0.0f, 5.0f}),
    pid_id(_c_f, (PID::Gain){8.0f, 10000.0f, 0.0f, 5.0f}),
    fl_ff_kv_(0) {};

  void update() override;

  void set_iq_gain(PID::Gain _gain){
    pid_iq.set_PIDgain(_gain);
  };

  void set_id_gain(PID::Gain _gain){
    pid_id.set_PIDgain(_gain);
  };

  void set_ff_kv(float _kv) { fl_ff_kv_ = _kv; };

protected:
  PID pid_iq;
  PID pid_id;
  float fl_ff_kv_;  // 逆起電力定数[V/(deg/s)]
};

class BldcDriveMethodSineWithCurr : public BldcDriveMethodSine {
public:
  BldcDriveMethodSineWithCurr(BLDC *_bldc, float _c_f=10000.0f) : BldcDriveMethodSine(_bldc) {};

  void update() override;
};

#endif