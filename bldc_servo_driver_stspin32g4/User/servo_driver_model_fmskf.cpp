#include "adcc.hpp"
#include "bldc.hpp"
#include "bldc_drive_method.hpp"
#include "debug_dac.hpp"
#include "debug_printf.hpp"
#include "gate_drive_controller.hpp"
#include "mymath.hpp"
#include "servo_driver_model.hpp"
#include "uart_dmac.hpp"


enum ADC1CH {
  CurFb_U, // CH3,  PA2
  CurFb_W, // CH12, PB1
  Bemf_U,  // CH6,  PC0
  Bemf_V,  // CH7,  PC1
  Bemf_W,  // CH8,  PC2
  TempSens // TEMP
};
static ADCC<6> Adc1Ctrl(ADC1, DMA2, LL_DMA_CHANNEL_1);

enum ADC2CH {
  CurFb_V,  // CH3,  PA6
  AD_A,     // CH12, PB2
  AD_B,     // CH5,  PC4
  VBAT_MON, // CH1,  PA0
};
static ADCC<4> Adc2Ctrl(ADC2, DMA2, LL_DMA_CHANNEL_2);

class Maxon_BLDC : public BLDC {
public:
  Maxon_BLDC(){};

  void init() override {
    /* HallSensor */
    LL_TIM_EnableIT_CC1(TIM4);
    LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1);
    LL_TIM_EnableCounter(TIM4);

    /* PWM */
    LL_TIM_EnableUpdateEvent(TIM1);
    LL_TIM_EnableCounter(TIM1);
    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    //LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
    //LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
    //LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1);
    //LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N |
    //                              LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N |
    //                              LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N );
  };

  void hall_itr_callback() override {
    uint32_t portb               = LL_GPIO_ReadInputPort(GPIOB);
    uint8_t  _u8_next_hall_state = (uint8_t)((portb >> 6) & 0b111);
    switch(_u8_next_hall_state) {
    case 1:
      if(u8_now_hall_state_ == 5)
        s8_now_motor_dir_ = 1;
      else if(u8_now_hall_state_ == 3)
        s8_now_motor_dir_ = -1;
      break;
    case 3:
      if(u8_now_hall_state_ == 1)
        s8_now_motor_dir_ = 1;
      else if(u8_now_hall_state_ == 2)
        s8_now_motor_dir_ = -1;
      break;
    case 2:
      if(u8_now_hall_state_ == 3)
        s8_now_motor_dir_ = 1;
      else if(u8_now_hall_state_ == 6)
        s8_now_motor_dir_ = -1;
      break;
    case 6:
      if(u8_now_hall_state_ == 2)
        s8_now_motor_dir_ = 1;
      else if(u8_now_hall_state_ == 4)
        s8_now_motor_dir_ = -1;
      break;
    case 4:
      if(u8_now_hall_state_ == 6)
        s8_now_motor_dir_ = 1;
      else if(u8_now_hall_state_ == 5)
        s8_now_motor_dir_ = -1;
      break;
    case 5:
      if(u8_now_hall_state_ == 4)
        s8_now_motor_dir_ = 1;
      else if(u8_now_hall_state_ == 1)
        s8_now_motor_dir_ = -1;
      break;
    }

    u16_hall_counter_ += s8_now_motor_dir_;

    /* 電気角キャリブレーション用(いずれ自動化) */
    //uint16_t HallAd_A = Adc2Ctrl.get_adc_data(ADC2CH::AD_A);
    //uint16_t HallAd_B = Adc2Ctrl.get_adc_data(ADC2CH::AD_B);
    //float hallad_center_shift_A = (float)HallAd_A - 2048;
    //float hallad_center_shift_B = (float)HallAd_B - 2048;
    //float fl_hall_atan_deg = mymath::rad2deg(mymath::atan2f(hallad_center_shift_A, hallad_center_shift_B));
    //debug_printf("%d,%d,%d \n", u8_now_hall_state_, _u8_next_hall_state, (int)(fl_hall_atan_deg*10.0f));

    u8_now_hall_state_ = _u8_next_hall_state;
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
    uint16_t HallAd_A = Adc2Ctrl.get_adc_data(ADC2CH::AD_A);
    uint16_t HallAd_B = Adc2Ctrl.get_adc_data(ADC2CH::AD_B);
    float hallad_center_shift_A = (float)HallAd_A - 2048;
    float hallad_center_shift_B = (float)HallAd_B - 2048;
    float fl_hall_atan_deg = mymath::rad2deg(mymath::atan2f(hallad_center_shift_A, hallad_center_shift_B));
    float fl_elec_ang_deg  = (fl_hall_atan_deg - 71.5f + 22.5f) * 4.0f;  // 機械角→電気角変換
    return fl_elec_ang_deg;
  }

  bool get_fault_state() override {
    return ((LL_GPIO_ReadInputPort(GPIOE) & GPIO_PIN_14) != GPIO_PIN_14);
  };
  bool get_ready_state() override {
    return ((LL_GPIO_ReadInputPort(GPIOE) & GPIO_PIN_15) == GPIO_PIN_15);
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
    //*/
    /*
    uint32_t u32_CCER_U = (Uenable == DRIVE_OUT_BOTH_ENABLE)
                              ? (TIM_CCER_CC1E | TIM_CCER_CC1NE)
                              : ((Uenable == DRIVE_OUT_LOW_ENABLE) ? TIM_CCER_CC1NE : 0);
    uint32_t u32_CCER_V = (Venable == DRIVE_OUT_BOTH_ENABLE)
                              ? (TIM_CCER_CC2E | TIM_CCER_CC2NE)
                              : ((Venable == DRIVE_OUT_LOW_ENABLE) ? TIM_CCER_CC2NE : 0);
    uint32_t u32_CCER_W = (Wenable == DRIVE_OUT_BOTH_ENABLE)
                              ? (TIM_CCER_CC3E | TIM_CCER_CC3NE)
                              : ((Wenable == DRIVE_OUT_LOW_ENABLE) ? TIM_CCER_CC3NE : 0);
    return u32_CCER_U | u32_CCER_V | u32_CCER_W;
    //*/
  }

  void update_elecangle() {
  }
};

static Maxon_BLDC MxnBldc;

BLDC *get_bldc_if() { return &MxnBldc; };

static BldcDriveMethod6Step bldc_drv_method_6step(&MxnBldc);
static BldcDriveMethodSine  bldc_drv_method_sine(&MxnBldc);
BldcDriveMethod *           get_bldcdrv_method() { return &bldc_drv_method_sine; };

static GateDriveController GateDrvController(I2C3, DMA1, LL_DMA_CHANNEL_1, LL_DMA_CHANNEL_2);
I2CC *                     get_i2cc() { return &GateDrvController; };

static DACC Dac1Ctrl;

class FSD_RS485 : public UART_DMAC {
public:
  FSD_RS485(const USART_TypeDef *_u, const DMA_TypeDef *_d,
            const uint32_t _dch_r, const uint32_t _dch_t)
      : UART_DMAC(_u, _d, _dch_r, _dch_t){};

protected:
  void enable_tx() override { LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_14); };
  void disable_tx() override { LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_14); };
};

constexpr uint16_t RS485_RXBUF_LENGTH = 512;
constexpr uint16_t RS485_TXBUF_LENGTH = 256;
static uint8_t     u8_RS485_RXBUF[RS485_RXBUF_LENGTH];
static uint8_t     u8_RS485_TXBUF[RS485_TXBUF_LENGTH];
static FSD_RS485   Rs485Com(USART1, DMA1, LL_DMA_CHANNEL_3, LL_DMA_CHANNEL_4);

COM_BASE *get_debug_com() { return &Rs485Com; };

void initialize_servo_driver_model() {
  Rs485Com.init_constparam(u8_RS485_RXBUF, RS485_RXBUF_LENGTH,
                           u8_RS485_TXBUF, RS485_TXBUF_LENGTH);
  Rs485Com.init_rxtx();

  Dac1Ctrl.init();
  Adc1Ctrl.init();
  Adc2Ctrl.init();
  Adc1Ctrl.start();
  Adc2Ctrl.start();

  MxnBldc.init();
  GateDrvController.init();
  //GateDrvController.set_reset();
  //GateDrvController.set_VCC();
  GateDrvController.set_clear();
  LL_mDelay(1000);
  GateDrvController.get_status_reg();
}

void loop_servo_driver_model() {
  //uint16_t HallAd_A = Adc2Ctrl.get_adc_data(ADC2CH::AD_A);
  //uint16_t HallAd_B = Adc2Ctrl.get_adc_data(ADC2CH::AD_B);
  //Dac1Ctrl.set_dacs(HallAd_A, HallAd_B);
  bldc_drv_method_sine.update();
  //debug_printf("%d \n", (int)(MxnBldc.get_elec_angle()));
}