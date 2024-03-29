#include "ext_com_manager.hpp"

#include "ext_com_define.hpp"

#include "com_base.hpp"
#include "servo_driver_model.hpp"

static bool     IS_ALWAYS_RES_SUMMARY       = false; // 常にSummayを返すモード
static uint32_t U32_ALWAYS_RES_INTERVAL     = 3;     // n回に1回送信する
static uint32_t U32_ALWAYS_RES_INTERVAL_CNT = 0;     // n回に1回送信する

static uint32_t U32_TEST_COUNT = 0;

void getStatusSummary(RES_MESSAGE &msg);
void getStatusMoveAngle(RES_MESSAGE &msg);

void ext_com_manage_main() {
  EXT_COM_BASE     *p_extcom  = get_ext_com();
  BldcServoManager *p_bldcmng = get_bldcservo_manager();

  REQ_MESSAGE unReqMsg     = {};
  RES_MESSAGE unResMsg     = {};
  bool        bRes         = false;
  uint32_t    u32_r_cmd_id = 0;
  uint32_t    cmdid        = 0;
  uint8_t     dlc;

  if(p_extcom->getFillLevelRxMailboxes() > 0) {
    /* 受信データ有り */
    p_extcom->receive(u32_r_cmd_id, dlc, unReqMsg.u8_data);

    cmdid = EXTRACT_CMD_ID(u32_r_cmd_id);

    switch(cmdid) {
    case CMD_ID_REQ_TORQUE_ON: {
      p_bldcmng->set_mode(get_bldcmode_posctrl());
    } break;
    case CMD_ID_REQ_TORQUE_OFF: {
      p_bldcmng->set_mode(get_bldcmode_off());
    } break;
    case CMD_ID_REQ_TORQUE_CTRL: {
      p_bldcmng->set_mode(get_bldcmode_trqctrl());
    } break;
    case CMD_ID_REQ_MOVE_ANGLE: {
      BLDC               *p_bldc                 = get_bldc_if();
      BldcModeBase::Instr _instr                 = {};
      _instr.u16_instr_id                        = BldcModeBase::INSTR_ID_POSCTR_MOVE_ANGLE;
      _instr.InstrPosCtrl_MvAng.s32_tgt_pos      = unReqMsg.reqMvAng.s32_tgt_ang_deg_Q16;
      _instr.InstrPosCtrl_MvAng.s32_move_time_ms = (int32_t)unReqMsg.reqMvAng.u16_movetime_ms;
      p_bldc->set_current_lim((float)unReqMsg.reqMvAng.u16_currlim_A_Q8 * 0.00390625f);
      p_bldcmng->set_instr_buf(&_instr);

    } break;
    case CMD_ID_REQ_ANGLE_INIT: {
      /* 位置制御の基準角度を初期化する */
      if(unReqMsg.reqAngIni.u8_set_angle_flag & 0b10000000) {
        uint8_t _u8_setang = unReqMsg.reqAngIni.u8_set_angle_flag & 0b00000001;
        if(_u8_setang == 0) {
          /* 現在位置を0とする */
          get_bldc_if()->set_ref_ont_angle(get_bldc_if()->get_out_angle());
        } else if(_u8_setang == 1) {
          /* 現在位置を指定位置とする */
          float _fl_new_ref_ang = get_bldc_if()->get_raw_out_angle() - (float)unReqMsg.reqAngIni.s32_init_ang_deg_Q16 / (float)0x10000;
          get_bldc_if()->set_ref_ont_angle(_fl_new_ref_ang);
        }
      } else {
        BldcModeBase::Instr _instr                   = {};
        _instr.u16_instr_id                          = BldcModeBase::INSTR_ID_POSCTR_ANGLE_INIT;
        _instr.InstrPosCtrl_AngIni.u8_set_angle_flag = unReqMsg.reqAngIni.u8_set_angle_flag & 0b00000001;
        _instr.InstrPosCtrl_AngIni.s32_init_pos      = unReqMsg.reqAngIni.s32_init_ang_deg_Q16;
        p_bldcmng->set_instr_buf(&_instr);
      }
    } break;
    case CMD_ID_REQ_SET_TARGET_CURR: {
      /* 電流制御の目標を設定する */
      BldcModeBase::Instr _instr             = {};
      _instr.u16_instr_id                    = BldcModeBase::INSTR_ID_TRQCTR_SET_TARGET;
      _instr.InstrTrqCtrl_SetTgt.fl_tgt_Iq_A = (float)unReqMsg.reqSetTgtCur.s32_target_iq_A_Q16 / 65536.0f;
      _instr.InstrTrqCtrl_SetTgt.fl_tgt_Id_A = (float)unReqMsg.reqSetTgtCur.s32_target_id_A_Q16 / 65536.0f;
      p_bldcmng->set_instr_buf(&_instr);
    } break;
    case CMD_ID_RES_STATUS_MOVE_ANGLE: {
      getStatusMoveAngle(unResMsg);
      bRes = true;
    } break;
    case CMD_ID_RES_TEST_COM: {
      unResMsg.resTestCom.u32_test_const_num = 0x12345678;
      U32_TEST_COUNT++;
      unResMsg.resTestCom.u32_test_count = U32_TEST_COUNT;
      bRes = true;
    } break;

    default:
      break;
    }
  }

  /* この周期でまだResを作成していない(bRes == False)
   * and (Summary要求ビットが立っている or 常にSummayを返す) 場合 */
  if(!bRes && (IS_REQ_STATUS_SUMMARY(u32_r_cmd_id) || IS_ALWAYS_RES_SUMMARY)) {
    cmdid = CMD_ID_RES_STATUS_SUMMARY;
    getStatusSummary(unResMsg);
    bRes = true;
  }

  U32_ALWAYS_RES_INTERVAL_CNT++;
  if(!bRes && IS_ALWAYS_RES_SUMMARY && (U32_ALWAYS_RES_INTERVAL_CNT >= U32_ALWAYS_RES_INTERVAL)) {
    cmdid = CMD_ID_RES_STATUS_SUMMARY;
    getStatusSummary(unResMsg);
    bRes                        = true;
    U32_ALWAYS_RES_INTERVAL_CNT = 0;
  }

  /* Resが必要なら返す */
  // if(bRes && (p_extcom->getFreeLevelTxMailboxes() > 0)) {
  if(bRes) {
    p_extcom->transmit(cmdid, unResMsg.u8_data);
  }
}

void getStatusSummary(RES_MESSAGE &msg) {
  BLDC *p_bldc = get_bldc_if();

#if 0
  msg.resSummary.s16_out_ang_deg_Q4 = static_cast<int16_t>(p_bldc->get_out_angle() * 16);
  msg.resSummary.s8_motor_curr_A_Q4 = static_cast<int8_t>(p_bldc->fl_calc_Iq_meas_ * 16);
  msg.resSummary.s8_motor_vol_V_Q3  = static_cast<int8_t>(p_bldc->fl_calc_Vq_ * 8);
  msg.resSummary.u8_vm_V_Q3         = static_cast<uint8_t>(p_bldc->get_Vm() * 8);
  msg.resSummary.s8_motor_tempr_deg = static_cast<int8_t>(p_bldc->get_tempr_deg());
#else
  msg.resSummary2.s32_out_ang_deg_Q8 = static_cast<int32_t>(p_bldc->get_out_angle() * 256);
  msg.resSummary2.s8_motor_curr_A_Q4 = static_cast<int8_t>(p_bldc->fl_calc_Iq_meas_ * 16);
  msg.resSummary2.s8_motor_vol_V_Q3  = static_cast<int8_t>(p_bldc->fl_calc_Vq_ * 8);
  msg.resSummary2.u8_vm_V_Q3         = static_cast<uint8_t>(p_bldc->get_Vm() * 8);
  msg.resSummary2.s8_motor_tempr_deg = static_cast<int8_t>(p_bldc->get_tempr_deg());
#endif
}

void getStatusMoveAngle(RES_MESSAGE &msg) {
  BLDC *p_bldc = get_bldc_if();

  msg.resStsMvAng.s16_out_ang_deg_Q4 = static_cast<int16_t>(p_bldc->get_out_angle() * 16);
  msg.resStsMvAng.s16_tgt_ang_deg_Q4 = 0;
  msg.resStsMvAng.u16_movetime_ms    = 0;
}