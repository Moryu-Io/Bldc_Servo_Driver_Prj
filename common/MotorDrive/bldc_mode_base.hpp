#ifndef BLDC_MODE_BASE_HPP_
#define BLDC_MODE_BASE_HPP_

#include "bldc.hpp"
#include "main.h"

#define CONV_U32TYPE(x) (*((uint32_t *)&(x)))

/**
 * @brief BLDC制御MODE基底クラス
 *
 */
class BldcModeBase {
public:
  BldcModeBase();

  virtual void init()   = 0;
  virtual void update() = 0;
  virtual void end()    = 0;

  virtual bool isCompleted() { return false; };

  static BLDC *P_BLDC_;

  static const uint16_t INSTR_ID_POSCTR_MOVE_ANGLE = 0x0010;
  static const uint16_t INSTR_ID_POSCTR_ANGLE_INIT = 0x0011;

  union Instr {
    uint16_t u16_instr_id;
    struct M_PosCtrl_MvAng {
      uint16_t u16_instr_id;
      int32_t  s32_tgt_pos;
      int32_t  s32_move_time_ms;
    } InstrPosCtrl_MvAng;
    struct M_PosCtrl_AngIni {
      uint16_t u16_instr_id;
      uint8_t  u8_set_angle_flag;
      int32_t  s32_init_pos;
    } InstrPosCtrl_AngIni;
  };

  virtual void set_Instruction(Instr *p_instr){};

  union Status {
    struct M_PosCtrl_1 {
      int16_t  s16_tgt_ang_deg_Q4;
      uint16_t u16_movetime_ms;
    } stsPosCtrl_1;
  };

  virtual void get_status_1(Status *p_mem){};
  virtual void get_status_2(Status *p_mem){};
};

/**
 * @brief BLDC制御OFFクラス
 *
 */
class BldcModePowerOff : public BldcModeBase {
public:
  BldcModePowerOff()
      : BldcModeBase(){};

  void init() override{};
  void update() override;
  void end() override{};
};

#endif
