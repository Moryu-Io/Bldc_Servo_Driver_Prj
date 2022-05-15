#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include "main.h"

class controller
{
public:
    controller(float _c_freq) : freq_(_c_freq), dt_(1.0f / _c_freq){};

    virtual float update(float _nowval) = 0;
    virtual void reset() = 0;

    void set_target(float _tgtval) { now_tgt_ = _tgtval; };
    float get_target() { return now_tgt_; };

protected:
    float freq_ = 1.0f;
    float dt_ = 1.0f;

    float now_val_ = 0.0f;
    float prev_val_ = 0.0f;
    float now_tgt_ = 0.0f;

    float now_error_ = 0.0f;
    float prev_error_ = 0.0f;

    float now_ctrl_ = 0.0f;
};

class PID : public controller
{
public:
    PID(float _c_freq, float _p_gain, float _i_gain, float _d_gain,
        float _i_limit) : controller(_c_freq), Pgain_(_p_gain), Igain_(_i_gain), Dgain_(_d_gain),
                          I_limit_(_i_limit){};

    float update(float _nowval) override
    {
        now_val_ = _nowval;
        now_error_ = now_tgt_ - now_val_;

        // 積分計算
        Integ_ += Igain_ * dt_ * now_error_;
        Integ_ = (Integ_ >= I_limit_) ? I_limit_ : ((Integ_ <= -I_limit_) ? -I_limit_ : Integ_);

        // 制御量計算
        now_ctrl_ = Pgain_ * now_error_ + Integ_;

        prev_val_ = now_val_;
        prev_error_ = now_error_;

        return now_ctrl_;
    };

    void reset() override
    {
        Integ_ = 0.0f;

        now_val_ = 0.0f;
        prev_val_ = 0.0f;
        now_tgt_ = 0.0f;
        now_error_ = 0.0f;
        prev_error_ = 0.0f;

        now_ctrl_ = 0.0f;
    };

    void set_PIDgain(float _pg, float _ig, float _dg)
    {
        Pgain_ = _pg;
        Igain_ = _ig;
        Dgain_ = _dg;
    }
    void set_I_limit(float _i_lim) { I_limit_ = _i_lim; };

private:
    float Pgain_ = 0.0f;
    float Igain_ = 0.0f;
    float Dgain_ = 0.0f;

    float Integ_ = 0.0f;
    float I_limit_ = 0.0f;
};

#endif