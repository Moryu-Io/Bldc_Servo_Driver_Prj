#ifndef WRAPPER_INTERRUPT_HPP_
#define WRAPPER_INTERRUPT_HPP_


#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void USART6TX_DMATC_ITR();
void TIM6_ITR();
void TIM7_ITR();

#ifdef __cplusplus
}
#endif



#endif