#include "wrapper_main.hpp"
#include "main.h"
#include "servo_driver_model.hpp"
#include "wrapper_interrupt.hpp"

void cpp_wrapper_main_setup(void) {
  LL_OPAMP_Enable(OPAMP3);
  initialize_servo_driver_model();
  LL_mDelay(1000);
}

void cpp_wrapper_main_loop(void) {
  volatile uint32_t enc_cnt   = get_bldc_if()->get_hall_count();
  volatile uint32_t enc_state = get_bldc_if()->get_hall_state();
  volatile bool falut_sts   = get_bldc_if()->get_fault_state();
  volatile bool ready_sts = get_bldc_if()->get_ready_state();
  LL_mDelay(1);
  BldcDriveMethod::Ref inputVol = {
    .Vq = 1.0f,
    .Vd = 0.0f,
    .Iq = 0.0f,
    .Id = 0.0f,
  };
  get_bldcdrv_method()->set(inputVol);
  loop_servo_driver_model();
}
