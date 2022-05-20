#include "fdcan.h"

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

#include "flash_interface.hpp"


enum ADC1CH {
  Potentio,  // CH4,  PA3
  CsRef,     // CH11, PB12
  MotorTemp, // CH12, PB1
  VmSens,    // CH14, PB11
  McuTemp,
};
static ADCC<5> Adc1Ctrl(ADC1, DMA1, LL_DMA_CHANNEL_1);

enum ADC2CH {
  CurFb_U, // CH3, PA6
  CurFb_V, // CH4, PA7
  CurFb_W, // CH5, PC4
};
static ADCC<3> Adc2Ctrl(ADC2, DMA1, LL_DMA_CHANNEL_2);

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
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_6); // PWML HIGH

    /* ADC用トリガタイマ(TIM3) */
    /* このタイマはTIM1のUEVでカウンタリセットされ */
    /* OC1REFのトリガー出力でADC1,2を駆動する */
    LL_TIM_EnableCounter(TIM3);
    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
    //TIM3->CCR1 = 1900;
    TIM3->CCR1 = 3900;

    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_15); // 3PWM Mode
    LL_mDelay(1);
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_14); // nSleep解除

    Dac1Ctrl.init();
    Dac1Ctrl.set_dac_chB(4095);
    Dac1Ctrl.set_dac_chA(3072); // Current Sens Gain : 1V/A
  };

  void update_lowrate() override {
    //fl_Vm_ = Adc1Ctrl.get_adc_data(ADC1CH::VmSens) * Vm_Gain_ADtoV;
    fl_Vm_ = 12.0f;
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
    fl_now_elec_ang_deg_ = ((float)(ang - s32_elec_angle_offset_CNT_) * fl_elec_angle_gain_CNTtoDeg_ - 90.0f)*(float)s8_elec_angle_dir_;

    /* 電流測定 */
    now_current_.U = Curr_Gain_ADtoA * ((int16_t)Adc2Ctrl.get_adc_data(ADC2CH::CurFb_U) - 2047);
    now_current_.V = Curr_Gain_ADtoA * ((int16_t)Adc2Ctrl.get_adc_data(ADC2CH::CurFb_V) - 2047);
    now_current_.W = Curr_Gain_ADtoA * ((int16_t)Adc2Ctrl.get_adc_data(ADC2CH::CurFb_W) - 2047);
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
    return !LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_13);
  };

  bool get_ready_state() override {
    return true;
  }

private:
  const float Vm_inv = 1.0f / 12.0f;
  const float Vm_Gain_ADtoV = 3.3f/4096.0f * (400.0f + 33.0f) / 33.0f;
  const float Curr_Gain_ADtoA = 3.3f/4096.0f;  // 3.3V / 4096AD * 1 A/V
  const float Angle_Gain_CNTtoDeg = 360.0f / 16384.0f / 1.0f;



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

static PID AngleController(10000.0f, 0.03f, 0.001f, 0.0f, 1.0f);
static PI_D AngleController_PI_D(10000.0f, 0.04f, 0.01f, 0.0003f, 1.0f, 800.0f);
static IIR1 AngleCountrollerOut_filter(0.70f,0.15f,0.15f);
static TargetInterp AngleTargetInterp;

BldcModePosControl::Parts bldc_mode_posctrl_parts = {
  .p_bldc_drv   = &bldc_drv_method_vector,
  .p_pos_ctrl   = &AngleController_PI_D,
  .p_posout_lpf = &AngleCountrollerOut_filter,
  .p_tgt_interp = &AngleTargetInterp,
};
static BldcModePosControl mode_pos_control(bldc_mode_posctrl_parts);
static BldcModePowerOff   mode_off;

BldcModeBase* get_bldcmode_off() { return &mode_off; };
BldcModeBase* get_bldcmode_posctrl() { return &mode_pos_control; };

static BldcServoManager bldc_manager(&mode_off);
BldcServoManager* get_bldcservo_manager() { return &bldc_manager; };


/************************ CAN INTERFACE ******************************/
CANC CanIf(&hfdcan1, 0x001);

EXT_COM_BASE* p_ExtCom = &CanIf;

EXT_COM_BASE* get_ext_com() { return p_ExtCom; };
/*********************************************************************/

/************************ FLASH INTERFACE ******************************/
FlashIF FlashIf(255, 0x0807F800);

FlashIF* get_flash_if() { return &FlashIf; };
/*********************************************************************/

/************************ MODE TEST ******************************/
BldcModeTestElecAngle::Parts bldc_mode_test_elecang_parts = {
  .p_bldc_drv   = &bldc_drv_method_sine,
  .p_flashif    = &FlashIf,
};
BldcModeTestCurrStep::Parts bldc_mode_test_currstep_parts = {
  .p_bldc_drv   = &bldc_drv_method_vector,
};
BldcModeTestPosStep::Parts bldc_mode_test_posstep_parts = {
  .p_mode_posctrl   = &mode_pos_control,
};
BldcModeTestSineDriveOpen::Parts bldc_mode_test_sindrvopen_parts = {
  .p_bldc_drv   = &bldc_drv_method_sine,
};
static BldcModeTestElecAngle  mode_test_elec_ang(bldc_mode_test_elecang_parts);
static BldcModeTestCurrStep   mode_test_curr_step(bldc_mode_test_currstep_parts);
static BldcModeTestPosStep    mode_test_pos_step(bldc_mode_test_posstep_parts);
static BldcModeTestSineDriveOpen   mode_test_sindrvopen(bldc_mode_test_sindrvopen_parts);
/*****************************************************************/


/***************************** DEBUG ***********************************/
constexpr uint16_t DEBUG_COM_RXBUF_LENGTH = 512;
constexpr uint16_t DEBUG_COM_TXBUF_LENGTH = 256;
static uint8_t     u8_DEBUG_COM_RXBUF[DEBUG_COM_RXBUF_LENGTH];
static uint8_t     u8_DEBUG_COM_TXBUF[DEBUG_COM_TXBUF_LENGTH];
static UART_DMAC   DebugCom(USART1, DMA2, LL_DMA_CHANNEL_1, LL_DMA_CHANNEL_2);
static EXT_DEBUG_CAN_COM DebugComPretendCan((COM_BASE*)&DebugCom);

COM_BASE *get_debug_com() { return &DebugCom; };
/***********************************************************************/


void set_flash_parameter_to_models();

void initialize_servo_driver_model() {
  //LL_TIM_EnableCounter(TIM2);

  FlashIf.load(); // RAMへ展開
  if(FlashIf.mirrorRam.var.u8_is_reset_flash == 0xFF){
    // Flashがリセットされている場合は初期値で埋める
    FlashIf.mirrorRam.var = C_FlashInitParams.var;
    FlashIf.save();
  }
  
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

  /* パラメータ書き込み(暫定) */
  GmblBldc.set_elec_angle_gain(FlashIf.mirrorRam.var.fl_elec_angle_gain_CNTtoDeg);
  GmblBldc.set_elec_angle_offset(FlashIf.mirrorRam.var.s32_elec_angle_offset_CNT);
  GmblBldc.set_elec_angle_dir(FlashIf.mirrorRam.var.s8_elec_angle_dir);

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
  //debug_printf("%0.1f,%0.1f,%0.1f,%d,%d,%d\n", GmblBldc.get_elec_angle(), GmblBldc.fl_calc_Vq_,GmblBldc.fl_calc_Vd_, TIM1->CCR1, TIM1->CCR2, TIM1->CCR3);
  //debug_printf("%0.1f,%0.1f\n", GmblBldc.get_out_angle(), GmblBldc.get_elec_angle());


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
        {
        AngleTargetInterp.set_nowtarget((int32_t)(GmblBldc.get_out_angle()*(float)0x10000));
        BldcModeBase::Instr instr = {
          .InstrPosCtrl = {
            .s32_tgt_pos = 0,
            .s32_move_time_ms = 1000,
          },
        };
        mode_pos_control.set_Instruction(&instr);
        bldc_manager.set_mode(&mode_pos_control);
        }
        break;
      case 'e':
        bldc_manager.set_mode(&mode_off);
        break;
      case 'z':
        {
        BldcModeBase::Instr instr = {
          .InstrPosCtrl = {
            .s32_tgt_pos = -30 << 16,
            .s32_move_time_ms = 200,
          },
        };

        mode_pos_control.set_Instruction(&instr);
        }
        break;
      case 'x':
        {
        BldcModeBase::Instr instr = {
          .InstrPosCtrl = {
            .s32_tgt_pos = -120 << 16,
            .s32_move_time_ms = 200,
          },
        };
        mode_pos_control.set_Instruction(&instr);
        }
        break;
      case 'f':
        {
        while(DebugCom.get_rxBuf_datasize() < 1){;};
        uint8_t _u8_flash_cmd = 0;
        DebugCom.get_rxbyte(_u8_flash_cmd);
        switch (_u8_flash_cmd)
        {
        case 's':
          FlashIf.save();
          break;
        case 'r':
          FlashIf.erase();
          FlashIf.load();
          break;
        case 'p':
          {
            int _size = sizeof(FlashIf.mirrorRam);
            for(int i=0; i < _size/4; i++){
              debug_printf("%04x:%02x,%02x,%02x,%02x\n", i*4, 
                                                        FlashIf.mirrorRam.u8_d[4*i], 
                                                        FlashIf.mirrorRam.u8_d[4*i+1],
                                                        FlashIf.mirrorRam.u8_d[4*i+2],
                                                        FlashIf.mirrorRam.u8_d[4*i+3]);
            }
          }
          break;
        case 'w':
          {
            while(DebugCom.get_rxBuf_datasize() < 3){;};
            uint8_t _u8_write_data[3] = {};
            DebugCom.get_rxbytes(_u8_write_data, 3);
            uint16_t u16_addr = _u8_write_data[0] | (_u8_write_data[1] << 8);
            FlashIf.mirrorRam.u8_d[u16_addr] = _u8_write_data[2];
          }
          break;
        case 'd': /* flashに書かれているパラメータを各所に展開 */
          set_flash_parameter_to_models();
          break;
        default:
          break;
        };
        }
        break;
      case 'c': // 模擬CAN通信
        {
        while(DebugCom.get_rxBuf_datasize() < 13){;};
        EXT_DEBUG_CAN_COM::RcvData _rcv = {};
        DebugCom.get_rxbytes((uint8_t*)&_rcv, 13);
        DebugComPretendCan.setReceiveData(&_rcv);
        p_ExtCom = &DebugComPretendCan; // CANinterfaceを付け替え
        while(DebugComPretendCan.getFillLevelRxMailboxes()){;}; // CAN処理待ち
        p_ExtCom = &CanIf;  // CANinterfaceを元に戻す
        }
        break;
      case 't':
        {
        while(DebugCom.get_rxBuf_datasize() < 1){;};
        uint8_t _u8_test_cmd = 0;
        DebugCom.get_rxbyte(_u8_test_cmd);
        switch (_u8_test_cmd)
        {
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
        case 'e':
          bldc_manager.set_mode(&mode_test_elec_ang);
          break;
        case 's':
          {   
          while(DebugCom.get_rxBuf_datasize() < 8){;};
          float _fl_buf[2];
          DebugCom.get_rxbytes((uint8_t*)_fl_buf, 8);
          bldc_mode_test_sindrvopen_parts.fl_tgt_Vq_V = _fl_buf[0];
          bldc_mode_test_sindrvopen_parts.fl_tgt_Vd_V = _fl_buf[1];
          bldc_manager.set_mode(&mode_test_sindrvopen);
          }
          break;
        case 'p': /* 位置制御ランプ応答テスト */
          { 
          while(DebugCom.get_rxBuf_datasize() < 5){;};
          uint16_t _u16_buf[3] = {};
          DebugCom.get_rxbytes((uint8_t*)_u16_buf, 5);
          bldc_mode_test_posstep_parts.s16_tgt_pos_deg  = (int16_t)_u16_buf[0];
          bldc_mode_test_posstep_parts.u16_move_time_ms = _u16_buf[1];
          bldc_mode_test_posstep_parts.u8_mabiki = (uint8_t)_u16_buf[2];
          bldc_manager.set_mode(&mode_test_pos_step);
          }
          break;
        default:
          break;
        }

        }
        break;
    };
  }
  
}


void set_flash_parameter_to_models(){

  /* 位置制御器パラメータ展開 */
  AngleController_PI_D.set_PIDgain(FlashIf.mirrorRam.var.fl_PosCtrl_Pgain,
                              FlashIf.mirrorRam.var.fl_PosCtrl_Igain,
                              FlashIf.mirrorRam.var.fl_PosCtrl_Dgain);
  AngleController_PI_D.set_I_limit(FlashIf.mirrorRam.var.fl_PosCtrl_I_Limit);       
  AngleController_PI_D.set_VelLpf_CutOff(FlashIf.mirrorRam.var.fl_PosCtrl_VelLpf_CutOffFrq);    
                
}
