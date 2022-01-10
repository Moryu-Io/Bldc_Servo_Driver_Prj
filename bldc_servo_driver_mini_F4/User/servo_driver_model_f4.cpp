#include "adcc.hpp"
#include "dacc.hpp"
#include "debug_printf.hpp"
#include "mymath.hpp"
#include "servo_driver_model.hpp"
#include "uart_dmac.hpp"


enum ADC1CH {
  Potentio,  // CH7,  PA7
  VmSens,    // CH9,  PB1
  CsRef,     // CH14, PC4
  MotorTemp, // CH15, PC5
  McuTemp,   
};
static ADCC<5> Adc1Ctrl(ADC1, DMA2, LL_DMA_STREAM_0);

enum ADC2CH {
  CurFb_U,  // CH0, PA0
  CurFb_V,  // CH1, PA1
  CurFb_W,  // CH2, PA2
};
static ADCC<3> Adc2Ctrl(ADC2, DMA2, LL_DMA_STREAM_2);

static DACC Dac1Ctrl;

constexpr uint16_t DEBUG_COM_RXBUF_LENGTH = 512;
constexpr uint16_t DEBUG_COM_TXBUF_LENGTH = 256;
static uint8_t     u8_DEBUG_COM_RXBUF[DEBUG_COM_RXBUF_LENGTH];
static uint8_t     u8_DEBUG_COM_TXBUF[DEBUG_COM_TXBUF_LENGTH];
static UART_DMAC   DebugCom(USART6, DMA2, LL_DMA_STREAM_1, LL_DMA_STREAM_6);

COM_BASE *get_debug_com() { return &DebugCom; };

void initialize_servo_driver_model() {
  DebugCom.init_constparam(u8_DEBUG_COM_RXBUF, DEBUG_COM_RXBUF_LENGTH,
                           u8_DEBUG_COM_TXBUF, DEBUG_COM_TXBUF_LENGTH);
  DebugCom.init_rxtx();

  //Dac1Ctrl.init();
  //Adc1Ctrl.init();
  //Adc2Ctrl.init();
  //Adc1Ctrl.start();
  //Adc2Ctrl.start();

}

void loop_servo_driver_model() {
  LL_mDelay(1000);
  debug_printf("[F4]Test\n");
}