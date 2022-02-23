#include "can.h"

#include "adcc.hpp"
#include "canc.hpp"
#include "dacc.hpp"
#include "spic.hpp"
#include "uart_dmac.hpp"
#include "debug_printf.hpp"
#include "logger.hpp"
#include "iir.hpp"
#include "mymath.hpp"
#include "controller.hpp"

#include "servo_driver_model.hpp"

#include "bldc.hpp"
#include "bldc_drive_method.hpp"
#include "bldc_mode_base.hpp"
#include "bldc_mode_pos_control.hpp"
#include "bldc_mode_test.hpp"
#include "bldc_servo_manager.hpp"


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
    // カウント開始してからRCレジスタを変えることで、UEVタイミングを変更
    LL_TIM_SetRepetitionCounter(TIM1, 1);
    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_13); // PWML HIGH

    /* ADC用トリガタイマ(TIM3) */
    /* このタイマはTIM1のUEVでカウンタリセットされ */
    /* OC1REFのトリガー出力でADC3を駆動する */
    LL_TIM_EnableCounter(TIM3);
    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
    TIM3->CCR1 = 1900;

    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_6); // 3PWM Mode
    LL_mDelay(1);
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_2); // nSleep解除

    Dac1Ctrl.init();
    Dac1Ctrl.set_dac_chB(4095);
    Dac1Ctrl.set_dac_chA(3072); // Current Sens Gain : 1V/A
  };

  void update_lowrate() override {
    fl_Vm_ = Adc1Ctrl.get_adc_data(ADC1CH::VmSens) * Vm_Gain_ADtoV;
  };

  void update() override {
    /* 電気角測定 */
    uint16_t txdata = 0xFFFF;
    uint16_t rxdata = 0;

    MotorAngSenserCtrl.send_bytes(&txdata, &rxdata, 1);
    int32_t ang       = rxdata & 0x3FFF;    // 14bit Absolute Encoder
    int32_t delta_ang = ang - s32_pre_angle_raw;
    delta_ang =  (delta_ang > 0x1FFF)  ? (delta_ang - 0x3FFF)
               :((delta_ang < -0x1FFF) ? (delta_ang + 0x3FFF)
                                        : delta_ang );
    s32_pre_angle_raw = ang;
    // s32_angle_rotor_count_ += delta_ang;
    s32_angle_rotor_count_ -= delta_ang;  // 逆方向のため

    fl_now_out_ang_deg_  = (float)(s32_angle_rotor_count_) * Angle_Gain_CNTtoDeg;
    fl_now_elec_ang_deg_ = (float)(16383 - ang - 1380) * 0.241713972f + 90.0f;

    /* 電流測定 */
    now_current_.U = Curr_Gain_ADtoA * (Adc2Ctrl.get_adc_data(ADC2CH::CurFb_U) - 2048);
    now_current_.V = Curr_Gain_ADtoA * (Adc2Ctrl.get_adc_data(ADC2CH::CurFb_V) - 2048);
    now_current_.W = Curr_Gain_ADtoA * (Adc2Ctrl.get_adc_data(ADC2CH::CurFb_W) - 2048);
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
  const float Vm_Gain_ADtoV = 3.3f/4096.0f * (325.0f + 33.0f) / 33.0f;
  const float Curr_Gain_ADtoA = 3.3f/4096.0f;  // 3.3V / 4096AD * 1 A/V
  const float Angle_Gain_CNTtoDeg = 360.0f / 16384.0f / 1.0f;

  int32_t s32_pre_angle_raw = 0;

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

static BldcDriveMethodSine   bldc_drv_method_sine(&GmblBldc);
static BldcDriveMethodVector bldc_drv_method_vector(&GmblBldc);
BldcDriveMethod* get_bldcdrv_method() { return &bldc_drv_method_vector; };

static PID AngleController(10000.0f, 0.01f, 0.1f, 0.0f, 0.5f);
static IIR1 AngleCountrollerOut_filter(0.70f,0.15f,0.15f);
static TargetInterp AngleTargetInterp;

BldcModePosControl::Parts bldc_mode_posctrl_parts = {
  .p_bldc_drv   = &bldc_drv_method_vector,
  .p_pos_ctrl   = &AngleController,
  .p_posout_lpf = &AngleCountrollerOut_filter,
  .p_tgt_interp = &AngleTargetInterp,
};
static BldcModePosControl mode_pos_control(bldc_mode_posctrl_parts);
static BldcModePowerOff   mode_off;

/************************ MODE TEST ******************************/
BldcModeTestCurrStep::Parts bldc_mode_test_currstep_parts = {
  .p_bldc_drv   = &bldc_drv_method_vector,
};
static BldcModeTestCurrStep   mode_test_curr_step(bldc_mode_test_currstep_parts);
/*****************************************************************/

static BldcServoManager bldc_manager(&mode_off);
BldcServoManager* get_bldcservo_manager() { return &bldc_manager; };


/************************ CAN INTERFACE ******************************/
CANC CanIf(&hcan1, 0x001);
/*********************************************************************/

/***************************** DEBUG ***********************************/
constexpr uint16_t DEBUG_COM_RXBUF_LENGTH = 512;
constexpr uint16_t DEBUG_COM_TXBUF_LENGTH = 256;
static uint8_t     u8_DEBUG_COM_RXBUF[DEBUG_COM_RXBUF_LENGTH];
static uint8_t     u8_DEBUG_COM_TXBUF[DEBUG_COM_TXBUF_LENGTH];
static UART_DMAC   DebugCom(USART6, DMA2, LL_DMA_STREAM_1, LL_DMA_STREAM_6);

COM_BASE *get_debug_com() { return &DebugCom; };
/***********************************************************************/


void initialize_servo_driver_model() {
  LL_TIM_EnableCounter(TIM2);
  
  DebugCom.init_constparam(u8_DEBUG_COM_RXBUF, DEBUG_COM_RXBUF_LENGTH,
                           u8_DEBUG_COM_TXBUF, DEBUG_COM_TXBUF_LENGTH);
  DebugCom.init_rxtx();
  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13);
  CanIf.init();

  MotorAngSenserCtrl.init();

  Adc1Ctrl.init();
  Adc2Ctrl.init();
  Adc1Ctrl.start();
  Adc2Ctrl.start();
  GmblBldc.init();

  /* 100Hz */
  LL_TIM_EnableIT_UPDATE(TIM6);
  LL_TIM_EnableCounter(TIM6);

  /* 10kHz */
  LL_TIM_EnableIT_UPDATE(TIM7);
  LL_TIM_EnableCounter(TIM7);
}

void loop_servo_driver_model() {
  LL_mDelay(100);

  GmblBldc.update_lowrate();
  //debug_printf("%0.1f\n", GmblBldc.get_Vm());

  uint32_t u32_r_cmd_id = 0;
  uint8_t txd[8] = {};
  uint8_t rxd[8] = {};
  uint8_t dlc;

  if(CanIf.getFillLevelRxMailboxes() > 0){
    CanIf.receive(u32_r_cmd_id,dlc,rxd);
    for(int i=0;i<8;i++) txd[i] = rxd[7-i];
    
    CanIf.transmit(u32_r_cmd_id+1, txd);
  }
  //CanIf.transmit(u32_r_cmd_id+1, txd);


  if(!DebugCom.is_rxBuf_empty()){
    uint8_t _u8_c = 0;
    DebugCom.get_rxbyte(_u8_c);
    switch(_u8_c){
      case 's':
        LOG::disable_logging();
        LOG::clear_LogData();
        LOG::clear_LogAddressArray();
        LOG::put_LogAddress(&bldc_manager.u32_status_memory[0]);
        LOG::put_LogAddress(&bldc_manager.u32_status_memory[1]);
        LOG::put_LogAddress(&bldc_manager.u32_status_memory[2]);
        LOG::put_LogAddress(&bldc_manager.u32_status_memory[3]);
        LOG::enable_logging();
        break;
      case 'p':
        LOG::disable_logging();
        debug_printf("Start\n");
        LOG::print_LogData_byFLOAT();
        debug_printf("End\n");
        break;
      case 'd':
        bldc_manager.set_mode(&mode_pos_control);
        break;
      case 'f':
        bldc_manager.set_mode(&mode_off);
        break;
      case 'z':
        AngleController.set_target(0.0f);
        break;
      case 'x':
        AngleController.set_target(60.0f);
        break;
      case 'c':
        {
        while(DebugCom.get_rxBuf_datasize() < 8){;};
        float _fl_buf[2];
        DebugCom.get_rxbytes((uint8_t*)_fl_buf, 8);
        bldc_mode_test_currstep_parts.fl_tgt_Iq_A = _fl_buf[0];
        bldc_mode_test_currstep_parts.fl_tgt_Id_A = _fl_buf[1];
        bldc_manager.set_mode(&mode_test_curr_step);
        }
        break;
    };
  }
  
}