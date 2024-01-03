#ifndef TIMCNT_HPP_
#define TIMCNT_HPP_

#include "main.h"

/**
 * @brief US単位程度をWaitしたりカウントしたりするTimer
 * @note  usの1/10でカウントする前提だよ
 */
class TIMUSCNT{
public:
    TIMUSCNT(TIM_TypeDef *_tim) : tim_(_tim) {}

    void init(){
        LL_TIM_EnableCounter(tim_);
    }

    void wait_us(uint16_t _us){
        volatile uint16_t _start_cnt = tim_->CNT;
        uint32_t _wait_cnt = (_us * 10) & 0xFFFF;
        volatile uint32_t _counter = 0;
        while(_counter < _wait_cnt){
            _counter = (tim_->CNT - _start_cnt) & 0xFFFF;
        };
    }

protected:
    TIM_TypeDef *tim_;


};



#endif