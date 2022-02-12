#include "adcc.hpp"
#include "dacc.hpp"
#include "debug_printf.hpp"
#include "mymath.hpp"
#include "servo_driver_model.hpp"
#include "spic.hpp"
#include "uart_dmac.hpp"
#include "bldc.hpp"
#include "bldc_drive_method.hpp"
#include "logger.hpp"

static float FL_DEBUG_LOG_BUF[4] = {};

enum ADC1CH {
  Potentio,  // CH7,  PA7
  VmSens,    // CH9,  PB1
  CsRef,     // CH14, PC4
  MotorTemp, // CH15, PC5
  McuTemp,
};
static ADCC<5> Adc1Ctrl(ADC1, DMA2, LL_DMA_STREAM_0);

enum ADC2CH {
  CurFb_U, // CH0, PA0
  CurFb_V, // CH1, PA1
  CurFb_W, // CH2, PA2
};
static ADCC<3> Adc2Ctrl(ADC2, DMA2, LL_DMA_STREAM_2);

static DACC Dac1Ctrl;

class MotorAngSenser : public SPIC {
public:
  MotorAngSenser(SPI_TypeDef *_spi) : SPIC(_spi){};

protected:
  void select() override { LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_15); };
  void deselect() override { LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_15); };
};

static MotorAngSenser MotorAngSenserCtrl(SPI3);

class PM3505: public BLDC {
public:
  PM3505(){};

  void init() override {
    /* PWM */
    LL_TIM_EnableUpdateEvent(TIM1);
    LL_TIM_EnableCounter(TIM1);
    LL_TIM_SetRepetitionCounter(TIM1, 1);
    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_13); // PWML HIGH

    /* ADC用トリガタイマ */
    LL_TIM_EnableCounter(TIM3);
    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
    TIM3->CCR1 = 1900;

    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_6); // 3PWM Mode
    LL_mDelay(1);
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_2); // nSleep解除

    Dac1Ctrl.init();
    Dac1Ctrl.set_dac_chB(4095);
    Dac1Ctrl.set_dac_chA(3072); // 1V/A
  };

  void update() override {
    /* 電気角測定 */
    uint16_t txdata = 0xFFFF;
    uint16_t rxdata = 0;

    MotorAngSenserCtrl.send_bytes(&txdata, &rxdata, 1);
    uint16_t ang = rxdata & 0x3FFF;
    fl_now_elec_ang_ = (float)(16383 - ang - 1380) * 0.241713972f + 90.0f;

    /* 電流測定 */
    now_current_.U = Curr_Gain_ADtoA * (Adc2Ctrl.get_adc_data(ADC2CH::CurFb_U) - 2048);
    now_current_.V = Curr_Gain_ADtoA * (Adc2Ctrl.get_adc_data(ADC2CH::CurFb_V) - 2048);
    now_current_.W = Curr_Gain_ADtoA * (Adc2Ctrl.get_adc_data(ADC2CH::CurFb_W) - 2048);

    FL_DEBUG_LOG_BUF[0] = now_current_.U;
    FL_DEBUG_LOG_BUF[1] = now_current_.V;
    FL_DEBUG_LOG_BUF[2] = now_current_.W;
  };

  void set_drive_duty(DriveDuty &_Vol) override {
    uint32_t u_duty = (uint32_t)(mymath::satf(_Vol.Duty.U * Vm_inv, 1.0f, 0.0f) * (float)TIM1->ARR);
    uint32_t v_duty = (uint32_t)(mymath::satf(_Vol.Duty.V * Vm_inv, 1.0f, 0.0f) * (float)TIM1->ARR);
    uint32_t w_duty = (uint32_t)(mymath::satf(_Vol.Duty.W * Vm_inv, 1.0f, 0.0f) * (float)TIM1->ARR);
    TIM1->CCR1      = u_duty;
    TIM1->CCR2      = v_duty;
    TIM1->CCR3      = w_duty;
    set_enable_register(_Vol.u8_U_out_enable, _Vol.u8_V_out_enable, _Vol.u8_W_out_enable);
  };

  bool get_fault_state() override {
    return !LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_10);
  };
  bool get_ready_state() override {
    return true;
  }

private:
  const float Vm_inv = 1.0f / 12.0f;
  const float Curr_Gain_ADtoA = 3.3f/4096.0f;  // 3.3V / 4096AD * 1 A/V

  inline void set_enable_register(uint8_t Uenable, uint8_t Venable, uint8_t Wenable) {
    ///*
    if(Uenable == DRIVE_OUT_BOTH_ENABLE) {
      LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
      LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N);
    } else if(Uenable == DRIVE_OUT_LOW_ENABLE) {
      LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_ACTIVE);
      LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1);
      LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N);
    } else {
      LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_INACTIVE);
      LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1);
      LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1N);
    }

    if(Venable == DRIVE_OUT_BOTH_ENABLE) {
      LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
      LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N);
    } else if(Venable == DRIVE_OUT_LOW_ENABLE) {
      LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_ACTIVE);
      LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH2);
      LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2N);
    } else {
      LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_INACTIVE);
      LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH2);
      LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH2N);
    }

    if(Wenable == DRIVE_OUT_BOTH_ENABLE) {
      LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1);
      LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
    } else if(Wenable == DRIVE_OUT_LOW_ENABLE) {
      LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_ACTIVE);
      LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH3);
      LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3N);
    } else {
      LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_INACTIVE);
      LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH3);
      LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH3N);
    }
  }

};

static PM3505 GmblBldc;

BLDC *get_bldc_if() { return &GmblBldc; };

static BldcDriveMethodSine  bldc_drv_method_sine(&GmblBldc);
static BldcDriveMethodVector  bldc_drv_method_vector(&GmblBldc);
BldcDriveMethod *           get_bldcdrv_method() { return &bldc_drv_method_vector; };

constexpr uint16_t DEBUG_COM_RXBUF_LENGTH = 512;
constexpr uint16_t DEBUG_COM_TXBUF_LENGTH = 256;
static uint8_t     u8_DEBUG_COM_RXBUF[DEBUG_COM_RXBUF_LENGTH];
static uint8_t     u8_DEBUG_COM_TXBUF[DEBUG_COM_TXBUF_LENGTH];
static UART_DMAC   DebugCom(USART6, DMA2, LL_DMA_STREAM_1, LL_DMA_STREAM_6);

COM_BASE *get_debug_com() { return &DebugCom; };


void initialize_servo_driver_model() {
  LL_TIM_EnableCounter(TIM2);
  
  DebugCom.init_constparam(u8_DEBUG_COM_RXBUF, DEBUG_COM_RXBUF_LENGTH,
                           u8_DEBUG_COM_TXBUF, DEBUG_COM_TXBUF_LENGTH);
  DebugCom.init_rxtx();

  MotorAngSenserCtrl.init();

  
  Adc1Ctrl.init();
  Adc2Ctrl.init();
  Adc1Ctrl.start();
  Adc2Ctrl.start();
  GmblBldc.init();

  //Dac1Ctrl.init();
  //Adc1Ctrl.init();
  //Adc2Ctrl.init();
  //Adc1Ctrl.start();
  //Adc2Ctrl.start();

  /* 100Hz */
  LL_TIM_EnableIT_UPDATE(TIM6);
  LL_TIM_EnableCounter(TIM6);

  /* 10kHz */
  LL_TIM_EnableIT_UPDATE(TIM7);
  LL_TIM_EnableCounter(TIM7);
}

void loop_servo_driver_model() {
  LL_mDelay(10);
  BldcDriveMethod::Ref inputVol = {
    .Vq = 0.0f,
    .Vd = 0.0f,
    .Iq = 0.1f,
    .Id = 0.0f,
  };

  if(!DebugCom.is_rxBuf_empty()){
    uint8_t _u8_c = 0;
    DebugCom.get_rxbyte(_u8_c);
    switch(_u8_c){
      case 's':
        LOG::disable_logging();
        LOG::clear_LogData();
        LOG::clear_LogAddressArray();
        LOG::put_LogAddress((uint32_t*)&FL_DEBUG_LOG_BUF[0]);
        LOG::put_LogAddress((uint32_t*)&FL_DEBUG_LOG_BUF[1]);
        LOG::put_LogAddress((uint32_t*)&FL_DEBUG_LOG_BUF[2]);
        LOG::enable_logging();
        get_bldcdrv_method()->set(inputVol);
        break;
      case 'p':
        LOG::disable_logging();
        LOG::print_LogData_byFLOAT();
        break;
      case 'd':
      break;
    };
  }
  
}