#ifndef WRAPPER_MAIN_HPP_
#define WRAPPER_MAIN_HPP_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx.h"

void cpp_wrapper_main_setup(void);
void cpp_wrapper_main_loop(void);

#ifdef __cplusplus
}
#endif

#endif /* WRAPPER_MAIN_HPP_ */