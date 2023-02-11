#include "pwminc.hpp"

void PWMINC::init() {
  timx_->PSC = u16_pwm_dura_;
  timx_->ARR = u16_max_cnt_;

  LL_TIM_DisableCounter(timx_);
  LL_TIM_CC_EnableChannel(timx_, LL_TIM_CHANNEL_CH1); // Rising Edge
  LL_TIM_CC_EnableChannel(timx_, LL_TIM_CHANNEL_CH2); // Falling Edge

  LL_TIM_EnableIT_CC1(timx_);
  LL_TIM_EnableCounter(timx_);
}


// CC1割り込みでrise/fall edgeを保存だけしておく
// updateで読み出し(latch)
void PWMINC::update(){
  if(isCapIntr)
  {
    isCapIntr = false;
    u16_cap_rise_edge = u16_cap_rise_edge_buf;
    u16_cap_fall_edge = u16_cap_fall_edge_buf;
    isCapEdge = true;
  } else {
    isCapEdge = false;
  }

}

void PWMINC::intr(){
  if(LL_TIM_IsActiveFlag_CC1(timx_)){
    LL_TIM_ClearFlag_CC1(timx_);
    u16_cap_rise_edge_buf = timx_->CCR1;
    u16_cap_fall_edge_buf = timx_->CCR2;
    isCapIntr = true;
  } else {
  }
}