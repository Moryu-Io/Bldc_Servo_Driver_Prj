#ifndef DEBUG_DAC_HPP_
#define DEBUG_DAC_HPP_

#include "main.h"

class DACC{
public:
    DACC(){};

    void init();

    void set_dac_chA(uint16_t _out);
    void set_dac_chB(uint16_t _out);
    void set_dacs(uint16_t _out_a, uint16_t _out_b);

};

#endif