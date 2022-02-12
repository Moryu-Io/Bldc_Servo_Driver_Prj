#ifndef WRAPPER_MAIN_HPP_
#define WRAPPER_MAIN_HPP_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void cpp_wrapper_main_setup(void);
void cpp_wrapper_main_loop(void);


/* 割り込みCallback関数 */
void USART6TX_DMATC_ITR();
void TIM6_ITR();
void TIM7_ITR();

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

#ifdef __cplusplus
}
#endif

#endif /* WRAPPER_MAIN_HPP_ */