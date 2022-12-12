#include "version.hpp"
#include "debug_printf.hpp"

static const uint16_t U16_FW_VERSION = 0x0102;


void print_version(unsigned short u16_chiplocalver){
    debug_printf("Common FW Version : 0x%04x\n", U16_FW_VERSION);
    debug_printf("Chip   FW Version : 0x%04x\n", u16_chiplocalver);
}

void print_buildtime(){
    debug_printf("FW build time : %s\n", BUILDTIME);
}
