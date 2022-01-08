#ifndef WRAPPER_INTERRUPT_HPP_
#define WRAPPER_INTERRUPT_HPP_


#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx.h"

void USART1TC_ITR();
void I2C3RDMA_ITR();
void I2C3TDMA_ITR();
void TIM4_ITR();
void TIM6_ITR();
void TIM7_ITR();

#ifdef __cplusplus
}
#endif



#endif