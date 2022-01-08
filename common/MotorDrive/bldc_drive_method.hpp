#ifndef BLDC_DRIVE_METHOD_HPP_
#define BLDC_DRIVE_METHOD_HPP_

#include "bldc.hpp"

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
};

class BldcDriveMethodVector : BldcDriveMethod {
public:
  BldcDriveMethodVector(BLDC *_bldc) : BldcDriveMethod(_bldc){};

  void update() override;
};

#endif