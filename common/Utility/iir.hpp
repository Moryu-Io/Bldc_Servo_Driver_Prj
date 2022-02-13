#ifndef IIR_HPP_
#define IIR_HPP_

/**
 * @brief Digital Infinite Impulse Response Filter
 * @note 1次IIRフィルタ
 * @note y_{n} = A1*y_{n-1} + B0*x_{n} + B1*x_{n-1}
 * 
 */
class IIR1 {
public:
  IIR1(float _A1, float _B0, float _B1)
      : A1_(_A1), B0_(_B0), B1_(_B1), now_Y_(0.0f), prev_Y_(0.0f),
        prev_X_(0.0f) {
    reset();
  }

  void reset() {
    now_Y_ = 0.0f;
    prev_Y_ = 0.0f;
    prev_X_ = 0.0f;
  }

  void set_coefs(float _A1, float _B0, float _B1) {
    A1_ = _A1;
    B0_ = _B0;
    B1_ = _B1;
    reset();
  }

  void get_coefs(float &_A1, float &_B0, float &_B1) {
    _A1 = A1_;
    _B0 = B0_;
    _B1 = B1_;
  }

  float update(float _now_X) {
    now_Y_ = A1_ * prev_Y_ + B0_ * _now_X + B1_ * prev_X_;
    prev_Y_ = now_Y_;
    prev_X_ = _now_X;

    return now_Y_;
  }

  float get_output() { return now_Y_; }

private:
  float A1_;
  float B0_;
  float B1_;

  float now_Y_;
  float prev_Y_;
  float prev_X_;
};


/**
 * @brief Digital Infinite Impulse Response Filter
 * @note 2次IIRフィルタ
 * @note y_{n} = A1*y_{n-1} + A2*y_{n-2} + B0*x_{n} + B1*x_{n-1} + B2*x_{n-2}
 * 
 */
class IIR2 {
public:
  IIR2(float _A1, float _A2, float _B0, float _B1, float _B2)
      : A1_(_A1), A2_(_A2), B0_(_B0), B1_(_B1), B2_(_B2), now_Y_(0.0f),
        prev1_Y_(0.0f), prev2_Y_(0.0f), prev1_X_(0.0f), prev2_X_(0.0f) {
    reset();
  }

  void reset() {
    now_Y_ = 0.0f;
    prev1_Y_ = 0.0f;
    prev2_Y_ = 0.0f;
    prev1_X_ = 0.0f;
    prev2_X_ = 0.0f;
  }

  void set_coefs(float _A1, float _A2, float _B0, float _B1, float _B2) {
    A1_ = _A1;
    A2_ = _A2;
    B0_ = _B0;
    B1_ = _B1;
    B2_ = _B2;
    reset();
  }

  void get_coefs(float &_A1, float &_A2, float &_B0, float &_B1, float &_B2) {
    _A1 = A1_;
    _A2 = A2_;
    _B0 = B0_;
    _B1 = B1_;
    _B2 = B2_;
  }

  float update(float _now_X) {
    now_Y_ = A1_ * prev1_Y_ + A2_ * prev2_Y_ + B0_ * _now_X + B1_ * prev1_X_ +
             B2_ * prev2_X_;
    prev2_Y_ = prev1_Y_;
    prev1_Y_ = now_Y_;
    prev2_X_ = prev1_X_;
    prev1_X_ = _now_X;

    return now_Y_;
  }

  float get_output() { return now_Y_; }

private:
  float A1_;
  float A2_;
  float B0_;
  float B1_;
  float B2_;

  float now_Y_;
  float prev1_Y_;
  float prev2_Y_;
  float prev1_X_;
  float prev2_X_;
};

#endif