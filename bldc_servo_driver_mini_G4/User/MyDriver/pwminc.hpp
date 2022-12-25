#ifndef PWM_IN_C_HPP_
#define PWM_IN_C_HPP_

#include "main.h"

class PWMINC{
public:
    PWMINC(TIM_TypeDef * _timx, uint32_t _pwm_dura, uint32_t _maxcnt) 
    : timx_(_timx), u16_pwm_dura_(_pwm_dura), u16_max_cnt_(_maxcnt),
      isCapEdge(), 
      u16_cap_rise_edge(), u16_cap_fall_edge() {};

    void init();
    virtual void update();

    uint32_t get_rise_edge(){ return u16_cap_rise_edge; };
    uint32_t get_fall_edge(){ return u16_cap_fall_edge; };

protected:
    TIM_TypeDef *timx_;

    uint16_t u16_pwm_dura_;
    uint16_t u16_max_cnt_;

    bool isCapEdge;
    uint32_t u16_cap_rise_edge;
    uint32_t u16_cap_fall_edge;

};

#endif