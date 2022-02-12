#include "main.h"

#include "wrapper_main.hpp"
#include "servo_driver_model.hpp"
#include "logger.hpp"

void cpp_wrapper_main_setup(void) {
  initialize_servo_driver_model();
}

void cpp_wrapper_main_loop(void) {
  loop_servo_driver_model();
}



void USART6TX_DMATC_ITR(){
    get_debug_com()->tx_callback();
}

void TIM6_ITR() {

}

void TIM7_ITR() {
  get_bldc_if()->update();
  get_bldcdrv_method()->update();

  
  LOG::routine();
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
    
}