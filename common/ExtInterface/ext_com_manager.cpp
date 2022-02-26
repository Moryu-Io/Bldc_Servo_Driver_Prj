#include "ext_com_manager.hpp"

#include "ext_com_define.hpp"

#include "com_base.hpp"
#include "servo_driver_model.hpp"

void getStatusSummary(RES_MESSAGE &msg);
void getStatusMoveAngle(RES_MESSAGE &msg);

void ext_com_manage_main() {
  EXT_COM_BASE     *p_extcom  = get_ext_com();
  BldcServoManager *p_bldcmng = get_bldcservo_manager();

  uint32_t u32_r_cmd_id = 0;
  uint8_t  dlc;

  if(p_extcom->getFillLevelRxMailboxes() > 0) {
    /* 受信データ有り */
    REQ_MESSAGE unReqMsg = {};
    RES_MESSAGE unResMsg = {};
    bool        bRes     = false;

    p_extcom->receive(u32_r_cmd_id, dlc, unReqMsg.u8_data);

    uint32_t cmdid = EXTRACT_CMD_ID(u32_r_cmd_id);

    switch(cmdid) {
    case CMD_ID_REQ_TORQUE_ON: {
      p_bldcmng->set_mode(get_bldcmode_posctrl());
    } break;
    case CMD_ID_REQ_TORQUE_OFF: {
      p_bldcmng->set_mode(get_bldcmode_off());
    } break;
    case CMD_ID_REQ_MOVE_ANGLE: {
      BldcModeBase::Instr _instr           = {};
      _instr.InstrPosCtrl.s32_tgt_pos      = unReqMsg.reqMvAng.s32_tgt_ang_deg_Q16;
      _instr.InstrPosCtrl.s32_move_time_ms = (int32_t)unReqMsg.reqMvAng.u16_movetime_ms;
      p_bldcmng->set_instr_buf(&_instr);

    } break;
    case CMD_ID_RES_STATUS_MOVE_ANGLE: {
      getStatusMoveAngle(unResMsg);
      bRes = true;
    } break;

    default:
      break;
    }

    /* Summary要求ビットが立っている場合 */
    if(IS_REQ_STATUS_SUMMARY(u32_r_cmd_id)) {
      cmdid = CMD_ID_RES_STATUS_SUMMARY;
      getStatusSummary(unResMsg);
      bRes = true;
    }

    /* Resが必要なら返す */
    if(bRes && (p_extcom->getFreeLevelTxMailboxes() > 0)) {
      p_extcom->transmit(cmdid, unResMsg.u8_data);
    }
  }
}

void getStatusSummary(RES_MESSAGE &msg) {
  BLDC *p_bldc = get_bldc_if();

  msg.resSummary.s16_out_ang_deg_Q4 = static_cast<int16_t>(p_bldc->get_out_angle() * 16);
  msg.resSummary.s8_motor_curr_A_Q4 = static_cast<int8_t>(p_bldc->fl_calc_Iq_meas_ * 16);
  msg.resSummary.s8_motor_vol_V_Q3  = static_cast<int8_t>(p_bldc->fl_calc_Vq_ * 8);
  msg.resSummary.u8_vm_V_Q3         = static_cast<uint8_t>(p_bldc->get_Vm() * 8);
  msg.resSummary.s8_motor_tempr_deg = static_cast<int8_t>(p_bldc->get_tempr_deg());
}

void getStatusMoveAngle(RES_MESSAGE &msg) {
  BLDC *p_bldc = get_bldc_if();

  msg.resStsMvAng.s16_out_ang_deg_Q4 = static_cast<int16_t>(p_bldc->get_out_angle() * 16);
  msg.resStsMvAng.s16_tgt_ang_deg_Q4 = 0;
  msg.resStsMvAng.u16_movetime_ms    = 0;
}