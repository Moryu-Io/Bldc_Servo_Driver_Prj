#include "main.h"

#include "ext_com_manager.hpp"
#include "logger.hpp"
#include "servo_driver_model.hpp"
#include "wrapper_main.hpp"

#include "debug_printf.hpp"

volatile uint32_t u32_tim7_end_cnt = 0;

void cpp_wrapper_main_setup(void) {
  initialize_servo_driver_model();
}

void cpp_wrapper_main_loop(void) {
  loop_servo_driver_model();

  // debug_printf("%d\n", u32_tim7_end_cnt);
}

void USART1TX_DMATC_ITR() {
  get_debug_com()->tx_callback();
}

void TIM6_ITR() {
  //ext_com_manage_main();
}

void TIM7_ITR() {
  get_bldcservo_manager()->update();

  u32_tim7_end_cnt = TIM7->CNT;

  LOG::routine();
}

extern "C"
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
  ext_com_manage_main();
}