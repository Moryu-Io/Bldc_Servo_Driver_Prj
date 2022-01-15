#include "adcc.hpp"
#include "dacc.hpp"
#include "debug_printf.hpp"
#include "mymath.hpp"
#include "servo_driver_model.hpp"
#include "spic.hpp"
#include "uart_dmac.hpp"
#include "bldc.hpp"
#include "bldc_drive_method.hpp"

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
    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_13); // PWML HIGH

    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_6); // 3PWM Mode
    LL_mDelay(1);
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_2); // nSleep解除

    Dac1Ctrl.init();
    Dac1Ctrl.set_dac_chB(4095);
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

  float get_elec_angle() override {
    //int32_t tim_count = TIM2->CNT % 1000000;
    //float fl_elec_ang_deg  = (float)tim_count * 360.0f * 0.000001f;
    float fl_elec_ang_deg  = 0;

    uint8_t txbuf[2] = {0xFF,0xFF};
    uint8_t rxbuf[2] = {};

    MotorAngSenserCtrl.send_bytes(txbuf, rxbuf, 2);
    uint16_t ang = ((rxbuf[0] << 8) | rxbuf[1]) & 0x3FFF;
    fl_elec_ang_deg = (float)(16383 - ang - 1380) * 0.241713972f + 90.0f;

    return fl_elec_ang_deg;
  }

  bool get_fault_state() override {
    return !LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_10);
  };
  bool get_ready_state() override {
    return true;
  }

private:
  const float Vm_inv = 1.0f / 12.0f;

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
BldcDriveMethod *           get_bldcdrv_method() { return &bldc_drv_method_sine; };

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
  LL_mDelay(100);
  BldcDriveMethod::Ref inputVol = {
    .Vq = 1.0f,
    .Vd = 0.0f,
    .Iq = 0.0f,
    .Id = 0.0f,
  };
  get_bldcdrv_method()->set(inputVol);
  
}