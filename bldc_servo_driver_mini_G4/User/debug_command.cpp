#include "debug_command.hpp"
#include "logger.hpp"
#include "version.hpp"
#include "canc.hpp"
#include "bldc_mode_test.hpp"

extern uint16_t U16_CHIP_LOCAL_VER;

extern BldcModeTestElecAngle     mode_test_elec_ang;
extern BldcModeTestCurrStep      mode_test_curr_step;
extern BldcModeTestPosStep       mode_test_pos_step;
extern BldcModeTestSineDriveOpen mode_test_sindrvopen;
extern BldcModeTestVdqStep       mode_test_vdqstep;

void debug_command_routine() {
  COM_BASE         *_debug_com     = get_debug_com();
  BldcServoManager *_bldc_manager  = get_bldcservo_manager();
  BldcModeBase     *_mode_pos_ctrl = get_bldcmode_posctrl();
  BldcModeBase     *_mode_off      = get_bldcmode_off();
  FlashIF          *_flash_if      = get_flash_if();

  if(!_debug_com->is_rxBuf_empty()) {
    uint8_t _u8_c = 0;
    _debug_com->get_rxbyte(_u8_c);
    switch(_u8_c) {
    case 's':
      LOG::disable_logging();
      LOG::clear_LogData();
      LOG::clear_LogAddressArray();
      LOG::put_LogAddress(&_bldc_manager->u32_status_memory[0]);
      LOG::put_LogAddress(&_bldc_manager->u32_status_memory[1]);
      LOG::put_LogAddress(&_bldc_manager->u32_status_memory[2]);
      LOG::put_LogAddress(&_bldc_manager->u32_status_memory[3]);
      LOG::enable_logging();
      break;
    case 'p':
      LOG::disable_logging();
      debug_printf("Start\n");
      LOG::print_LogData_byFLOAT();
      debug_printf("End\n");
      break;
    case 'd': {
      BLDC *_p_bldc = get_bldc_if();
      // まず現在位置で位置制御開始
      BldcModeBase::Instr instr = {
          .InstrPosCtrl_MvAng = {
              .u16_instr_id     = BldcModeBase::INSTR_ID_POSCTR_MOVE_ANGLE,
              .s32_tgt_pos      = (int32_t)(_p_bldc->get_out_angle() * (float)0x10000),
              .s32_move_time_ms = 0,
          },
      };
      _mode_pos_ctrl->set_Instruction(&instr);
      _bldc_manager->set_mode(_mode_pos_ctrl);

      // 待ち
      LL_mDelay(10);

      // 0degまで移動
      instr.InstrPosCtrl_MvAng.s32_tgt_pos      = 0;    // 0deg
      instr.InstrPosCtrl_MvAng.s32_move_time_ms = 1000; // 1s
      _mode_pos_ctrl->set_Instruction(&instr);
    } break;
    case 'e':
      _bldc_manager->set_mode(_mode_off);
      break;
    case 'z': {
      BldcModeBase::Instr instr = {
          .InstrPosCtrl_MvAng = {
              .u16_instr_id     = BldcModeBase::INSTR_ID_POSCTR_MOVE_ANGLE,
              .s32_tgt_pos      = -30 << 16,
              .s32_move_time_ms = 200,
          },
      };

      _mode_pos_ctrl->set_Instruction(&instr);
    } break;
    case 'x': {
      BldcModeBase::Instr instr = {
          .InstrPosCtrl_MvAng = {
              .u16_instr_id     = BldcModeBase::INSTR_ID_POSCTR_MOVE_ANGLE,
              .s32_tgt_pos      = -120 << 16,
              .s32_move_time_ms = 200,
          },
      };
      _mode_pos_ctrl->set_Instruction(&instr);
    } break;
    case 'f': {
      while(_debug_com->get_rxBuf_datasize() < 1) { ; };
      uint8_t _u8_flash_cmd = 0;
      _debug_com->get_rxbyte(_u8_flash_cmd);
      switch(_u8_flash_cmd) {
      case 's':
        _flash_if->save();
        break;
      case 'e':
        _flash_if->erase();
        _flash_if->load();
        break;
      case 'p': {
        int _size = sizeof(_flash_if->mirrorRam);
        for(int i = 0; i < _size / 4; i++) {
          debug_printf("%04x:%02x,%02x,%02x,%02x\n", i * 4,
                       _flash_if->mirrorRam.u8_d[4 * i],
                       _flash_if->mirrorRam.u8_d[4 * i + 1],
                       _flash_if->mirrorRam.u8_d[4 * i + 2],
                       _flash_if->mirrorRam.u8_d[4 * i + 3]);
        }
      } break;
      case 'w': {
        while(_debug_com->get_rxBuf_datasize() < 3) { ; };
        uint8_t _u8_write_data[3] = {};
        _debug_com->get_rxbytes(_u8_write_data, 3);
        uint16_t u16_addr                   = _u8_write_data[0] | (_u8_write_data[1] << 8);
        _flash_if->mirrorRam.u8_d[u16_addr] = _u8_write_data[2];
      } break;
      case 'r': {
        while(_debug_com->get_rxBuf_datasize() < 2) { ; };
        uint8_t _u8_read_data[2] = {};
        _debug_com->get_rxbytes(_u8_read_data, 2);
        uint16_t u16_addr                   = _u8_read_data[0] | (_u8_read_data[1] << 8);
        debug_printf("%02x\n", (uint8_t)_flash_if->mirrorRam.u8_d[u16_addr]);
      } break;
      case 'd': /* flashに書かれているパラメータを各所に展開 */
        set_flash_parameter_to_models();
        ((CANC*)get_ext_com_default())->init(); // CAN通信初期化(無理やり...)
        break;
      default:
        break;
      };
    } break;
    case 'c': // 模擬CAN通信
    {
      EXT_DEBUG_CAN_COM *_debug_comPretendCan = (EXT_DEBUG_CAN_COM *)get_debug_com_pretend_ext();

      while(_debug_com->get_rxBuf_datasize() < 13) { ; };
      EXT_DEBUG_CAN_COM::RcvData _rcv = {};
      _debug_com->get_rxbytes((uint8_t *)&_rcv, 13);
      _debug_comPretendCan->setReceiveData(&_rcv);
      set_ext_com(_debug_comPretendCan);                            // CANinterfaceを付け替え
      while(_debug_comPretendCan->getFillLevelRxMailboxes()) { ; }; // CAN処理待ち
      set_ext_com(get_ext_com_default());                           // CANinterfaceを元に戻す
    } break;
    case 't': {
      while(_debug_com->get_rxBuf_datasize() < 1) { ; };
      uint8_t _u8_test_cmd = 0;
      _debug_com->get_rxbyte(_u8_test_cmd);
      switch(_u8_test_cmd) {
      case 'c': {
        /* 電流Step応答テスト */
        while(_debug_com->get_rxBuf_datasize() < 8) { ; };
        float _fl_buf[2];
        _debug_com->get_rxbytes((uint8_t *)_fl_buf, 8);
        
        BldcModeBase::Instr instr = {
            .InstrTestCurrStep = {
                .u16_instr_id = BldcModeBase::INSTR_ID_TEST_CURR_STEP,
                .fl_tgt_Iq_A  = _fl_buf[0],
                .fl_tgt_Id_A  = _fl_buf[1],
            },
        };
        _bldc_manager->set_mode_with_instr(&mode_test_curr_step, &instr);
      } break;
      case 'v': {
        /* 電圧Step応答テスト */
        while(_debug_com->get_rxBuf_datasize() < 8) { ; };
        float _fl_buf[2];
        _debug_com->get_rxbytes((uint8_t *)_fl_buf, 8);
        
        BldcModeBase::Instr instr = {
            .InstrTestVoltOpen = {
                .u16_instr_id = BldcModeBase::INSTR_ID_TEST_VOLT_STEP,
                .fl_tgt_Vq_V  = _fl_buf[0],
                .fl_tgt_Vd_V  = _fl_buf[1],
            },
        };
        _bldc_manager->set_mode_with_instr(&mode_test_vdqstep, &instr);
      } break;
      case 'e':
        _bldc_manager->set_mode(&mode_test_elec_ang);
        break;
      case 's': {
        while(_debug_com->get_rxBuf_datasize() < 8) { ; };
        float _fl_buf[2];
        _debug_com->get_rxbytes((uint8_t *)_fl_buf, 8);

        BldcModeBase::Instr instr = {
            .InstrTestSDrvOpen = {
                .u16_instr_id = BldcModeBase::INSTR_ID_TEST_SDRV_OPEN,
                .fl_tgt_Vq_V  = _fl_buf[0],
                .fl_tgt_Vd_V  = _fl_buf[1],
            },
        };
        _bldc_manager->set_mode_with_instr(&mode_test_sindrvopen, &instr);
      } break;
      case 'p': /* 位置制御ランプ応答テスト */
      {
        while(_debug_com->get_rxBuf_datasize() < 5) { ; };
        uint16_t _u16_buf[3] = {};
        _debug_com->get_rxbytes((uint8_t *)_u16_buf, 5);

        BldcModeBase::Instr instr = {
            .InstrTestPosStep = {
                .u16_instr_id = BldcModeBase::INSTR_ID_TEST_POS_STEP,
                .s16_tgt_pos_deg  = (int16_t)_u16_buf[0],
                .u16_move_time_ms = _u16_buf[1],
                .u8_mabiki        = (uint8_t)_u16_buf[2],
            },
        };
        _bldc_manager->set_mode_with_instr(&mode_test_pos_step, &instr);
      } break;
      default:
        break;
      }

    } break;
    case 'b': {
      print_version(U16_CHIP_LOCAL_VER);
      print_buildtime();
    } break;
    };
  }
}