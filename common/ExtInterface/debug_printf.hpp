#ifndef DEBUG_PRINTF_HPP_
#define DEBUG_PRINTF_HPP_

#include <stdio.h>
#include "servo_driver_model.hpp"

template <typename... Args>
void debug_printf(const char *format, Args const &...args){
    COM_BASE* debug_com = get_debug_com();
    uint8_t print_buf[256];
    uint8_t u8_print_size = sprintf((char *)print_buf, format, args...) + 1;
    if(u8_print_size >= 256) u8_print_size = 255;
    debug_com->set_txbytes(print_buf, u8_print_size);
}

#endif