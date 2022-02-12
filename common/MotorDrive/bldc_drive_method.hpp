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

  /* 入力 */
  virtual void set(Ref &_ref) { InRef_ = _ref; };

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
};

class BldcDriveMethodVector : public BldcDriveMethodSine {
public:
  BldcDriveMethodVector(BLDC *_bldc) : BldcDriveMethodSine(_bldc),
    pid_iq(10000.0f, 30.0f, 15.0f, 0.0f, 0.3f),
    pid_id(10000.0f, 30.0f, 15.0f, 0.0f, 0.3f) {};

  void update() override;

protected:
  PID pid_iq;
  PID pid_id;
};

#endif