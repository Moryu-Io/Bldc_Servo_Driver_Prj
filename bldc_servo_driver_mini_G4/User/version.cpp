#include "version.hpp"
#include "debug_printf.hpp"

static const uint16_t U16_FW_VERSION = 0x0000;


void print_version(){
    debug_printf("FW Version : 0x%04x\n", U16_FW_VERSION);
}

void print_buildtime(){
    debug_printf("FW build time : %s\n", BUILDTIME);
}
