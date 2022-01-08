#include "wrapper_interrupt.hpp"
#include "servo_driver_model.hpp"

void USART1TC_ITR(){
    get_debug_com()->tx_callback();
}

void I2C3RDMA_ITR(){
    get_i2cc()->rx_dma_TC();
}

void I2C3TDMA_ITR(){
    get_i2cc()->tx_dma_TC();
}

void TIM4_ITR() {
    get_bldc_if()->hall_itr_callback();
    get_bldcdrv_method()->itr_callback();
}

void TIM6_ITR() {
}
void TIM7_ITR() {
}