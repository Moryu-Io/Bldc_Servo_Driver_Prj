#include "debug_dac.hpp"

#define DAC_OUT_MAX (0x1000)

void DACC::init() {
  LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_1);
  LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_2);

  LL_DAC_EnableTrigger(DAC1, LL_DAC_CHANNEL_1);
  LL_DAC_EnableTrigger(DAC1, LL_DAC_CHANNEL_2);
}

void DACC::set_dac_chA(uint16_t _out) {
  _out = (_out > DAC_OUT_MAX) ? DAC_OUT_MAX : _out;
  LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, _out);
  LL_DAC_TrigSWConversion(DAC1, LL_DAC_CHANNEL_1);
}

void DACC::set_dac_chB(uint16_t _out) {
  _out = (_out > DAC_OUT_MAX) ? DAC_OUT_MAX : _out;
  LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_2, _out);
  LL_DAC_TrigSWConversion(DAC1, LL_DAC_CHANNEL_2);
}

void DACC::set_dacs(uint16_t _out_a, uint16_t _out_b) {
  _out_a = (_out_a > DAC_OUT_MAX) ? DAC_OUT_MAX : _out_a;
  _out_b = (_out_b > DAC_OUT_MAX) ? DAC_OUT_MAX : _out_b;
  LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, _out_a);
  LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_2, _out_b);
  LL_DAC_TrigSWConversion(DAC1, LL_DAC_CHANNEL_1);
  LL_DAC_TrigSWConversion(DAC1, LL_DAC_CHANNEL_2);
}
