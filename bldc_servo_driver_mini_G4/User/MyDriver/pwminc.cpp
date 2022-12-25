#include "pwminc.hpp"

void PWMINC::init() {
  timx_->PSC = u16_pwm_dura_;
  timx_->ARR = u16_max_cnt_;

  LL_TIM_DisableCounter(timx_);
  LL_TIM_CC_EnableChannel(timx_, LL_TIM_CHANNEL_CH1); // Rising Edge
  LL_TIM_CC_EnableChannel(timx_, LL_TIM_CHANNEL_CH2); // Falling Edge

  LL_TIM_EnableCounter(timx_);
}



void PWMINC::update(){
  if(LL_TIM_IsActiveFlag_CC1(timx_)){
    u16_cap_rise_edge = timx_->CCR1;
    u16_cap_fall_edge = timx_->CCR2;
    isCapEdge = true;
  } else {
    isCapEdge = false;
  }

}