#include "wrapper_interrupt.hpp"
#include "servo_driver_model.hpp"

void USART6TX_DMATC_ITR(){
    get_debug_com()->tx_callback();
}

void TIM6_ITR() {
    LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_2);
}

void TIM7_ITR() {
    LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_3);
}