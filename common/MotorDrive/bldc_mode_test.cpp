#include "bldc_mode_test.hpp"
#include "logger.hpp"
#include "mymath.hpp"


void BldcModeTestElecAngle::init()
{
    /* 内部変数初期化 */
    nowState_ = APPLY_OPEN_EANG;
    is_comp_ = false;
    u32_test_cnt_ = 0;
    s32_ElecOffsetSum= 0;
    st_curr_max.U = 0;
    st_curr_max.V = 0;
    st_curr_max.W = 0;
    st_curr_min.U = 0xFFFF;
    st_curr_min.V = 0xFFFF;
    st_curr_min.W = 0xFFFF;

    debug_printf("BldcModeTestElecAngle\n");
}

void BldcModeTestElecAngle::update()
{
    switch (nowState_)
    {
    case APPLY_OPEN_EANG:   nowState_ = apply_open_eang(); break;
    case WAIT_STABLE:       nowState_ = wait_stable();     break;
    case AVARAGE_EANG:      nowState_ = average_eang();    break;
    case FORCE_SINE:        nowState_ = force_sine();      break;
    case MEAS_CURR:         nowState_ = measure_current(); break;
    case DRV_STOP:          nowState_ = drive_stop();      break;
    default:                                               break;
    }
}
BldcModeTestElecAngle::state BldcModeTestElecAngle::apply_open_eang(){
    BldcDriveMethod::Ref input = {
        .Vq = 4.0f,
    };
    P_BLDC_->update();
    P_BLDC_->overwrite_elec_angle_forTEST(0.0f);
    parts_.p_bldc_drv->set(input);
    parts_.p_bldc_drv->update();

    return state::WAIT_STABLE;
}
BldcModeTestElecAngle::state BldcModeTestElecAngle::wait_stable(){
    P_BLDC_->update();
    P_BLDC_->overwrite_elec_angle_forTEST(0.0f);
    parts_.p_bldc_drv->update();
    if(u32_test_cnt_ < U32_TEST_STABLE_COUNT){
        u32_test_cnt_++;
        return state::WAIT_STABLE;
    } else {
        u32_test_cnt_ = 0;
        return state::AVARAGE_EANG;
    }
}
BldcModeTestElecAngle::state BldcModeTestElecAngle::average_eang(){
    P_BLDC_->update();
    P_BLDC_->overwrite_elec_angle_forTEST(0.0f);
    parts_.p_bldc_drv->update();
    if(u32_test_cnt_ < (uint32_t)(1 << U32_TEST_AVARAGE_COUNT_SHIFT)){
        s32_ElecOffsetSum += P_BLDC_->get_angle_raw();
        u32_test_cnt_++;
        return state::AVARAGE_EANG;
    } else {
        int32_t s32_elecoffset = s32_ElecOffsetSum >> U32_TEST_AVARAGE_COUNT_SHIFT;
        parts_.p_flashif->mirrorRam.var.s32_elec_angle_offset_CNT = s32_elecoffset;
        P_BLDC_->set_elec_angle_offset(s32_elecoffset);
        u32_test_cnt_ = 0;
        return state::FORCE_SINE;
    }
}
BldcModeTestElecAngle::state BldcModeTestElecAngle::force_sine(){
    BldcDriveMethod::Ref input = {
        .Vq = 4.0f,
        .Vd = 0.0f,
    };
    P_BLDC_->update();
    parts_.p_bldc_drv->set(input);
    parts_.p_bldc_drv->update();

    if(u32_test_cnt_ < U32_TEST_STABLE_COUNT){
        u32_test_cnt_++;
        return state::FORCE_SINE;
    } else {
        u32_test_cnt_ = 0;
        return state::MEAS_CURR;
    }
}
BldcModeTestElecAngle::state BldcModeTestElecAngle::measure_current(){
    BldcDriveMethod::Ref input = {
        .Vq = 4.0f,
        .Vd = 0.0f,
    };
    P_BLDC_->update();
    parts_.p_bldc_drv->set(input);
    parts_.p_bldc_drv->update();

    if(u32_test_cnt_ < U32_TEST_SINE_COUNT){
        BLDC::CurrentRaw _cur_ad = P_BLDC_->get_current_raw();
        st_curr_max.U = MAX(_cur_ad.U, st_curr_max.U);
        st_curr_max.V = MAX(_cur_ad.V, st_curr_max.V);
        st_curr_max.W = MAX(_cur_ad.W, st_curr_max.W);
        st_curr_min.U = MIN(_cur_ad.U, st_curr_min.U);
        st_curr_min.V = MIN(_cur_ad.V, st_curr_min.V);
        st_curr_min.W = MIN(_cur_ad.W, st_curr_min.W);
        u32_test_cnt_++;
        return state::MEAS_CURR;
    } else {
        u32_test_cnt_ = 0;
        return state::DRV_STOP;
    }
}
BldcModeTestElecAngle::state BldcModeTestElecAngle::drive_stop(){
    BldcDriveMethod::Ref input = {
        .Vq = 0.0f,
        .Vd = 0.0f,
    };
    P_BLDC_->update();
    parts_.p_bldc_drv->set(input);
    parts_.p_bldc_drv->update();
    is_comp_ = true;
    return state::DRV_STOP;
}
void BldcModeTestElecAngle::end()
{
    int32_t s32_elecoffset = s32_ElecOffsetSum >> U32_TEST_AVARAGE_COUNT_SHIFT;
    BLDC::CurrentRaw _cur_mid = {};
    _cur_mid.U = (st_curr_max.U + st_curr_min.U) >> 1;
    _cur_mid.V = (st_curr_max.V + st_curr_min.V) >> 1;
    _cur_mid.W = (st_curr_max.W + st_curr_min.W) >> 1;
    P_BLDC_->set_curr_raw_mid(_cur_mid);
    parts_.p_flashif->mirrorRam.var.u16_curr_ad_mid_u = _cur_mid.U;
    parts_.p_flashif->mirrorRam.var.u16_curr_ad_mid_v = _cur_mid.V;
    parts_.p_flashif->mirrorRam.var.u16_curr_ad_mid_w = _cur_mid.W;

    debug_printf("Eang:%d\n", (int)(s32_elecoffset));
    debug_printf("U :%d,%d,%d\n", (int)(st_curr_max.U),(int)(st_curr_min.U),(int)(_cur_mid.U));
    debug_printf("V :%d,%d,%d\n", (int)(st_curr_max.V),(int)(st_curr_min.V),(int)(_cur_mid.V));
    debug_printf("W :%d,%d,%d\n", (int)(st_curr_max.W),(int)(st_curr_min.W),(int)(_cur_mid.W));
}


void BldcModeTestCurrStep::init()
{
    /* 内部変数初期化 */
    is_comp_ = false;
    u16_test_cnt_ = 0;

    /* Logging設定 */
    LOG::disable_logging();
    LOG::clear_LogData();
    LOG::clear_LogAddressArray();
    LOG::put_LogAddress((uint32_t*)&P_BLDC_->fl_calc_Iq_tgt_);
    LOG::put_LogAddress((uint32_t*)&P_BLDC_->fl_calc_Id_tgt_);
    LOG::put_LogAddress((uint32_t*)&P_BLDC_->fl_calc_Iq_meas_);
    LOG::put_LogAddress((uint32_t*)&P_BLDC_->fl_calc_Id_meas_);
    LOG::put_LogAddress((uint32_t*)&P_BLDC_->fl_calc_Vq_);
    LOG::put_LogAddress((uint32_t*)&P_BLDC_->fl_calc_Vd_);
    LOG::enable_logging();
}


void BldcModeTestCurrStep::update()
{    
    BldcDriveMethod::Ref input = {};

    if(u16_test_cnt_ < U16_TEST_CURR_STABLE_COUNT){
        u16_test_cnt_++;
    } else if(u16_test_cnt_ < U16_TEST_CURR_END_COUNT) {
        input.Iq = fl_tgt_Iq_A_;
        input.Id = fl_tgt_Id_A_;

        u16_test_cnt_++;
    } else {
        LOG::disable_logging();
        is_comp_ = true;
    }

    P_BLDC_->update();
    parts_.p_bldc_drv->set(input);
    parts_.p_bldc_drv->update();

}

void BldcModeTestCurrStep::set_Instruction(Instr *p_instr){
  switch(p_instr->u16_instr_id) {
  case INSTR_ID_TEST_CURR_STEP:
    fl_tgt_Iq_A_ = p_instr->InstrTestCurrStep.fl_tgt_Iq_A;
    fl_tgt_Id_A_ = p_instr->InstrTestCurrStep.fl_tgt_Id_A;
    break;
  default:
    break;
  }
}


/*******************************************************
 * 電圧ステップ応答
 *******************************************************/
void BldcModeTestVdqStep::init()
{
    /* 内部変数初期化 */
    is_comp_ = false;
    u16_test_cnt_ = 0;

    /* Logging設定 */
    LOG::disable_logging();
    LOG::clear_LogData();
    LOG::clear_LogAddressArray();
    LOG::put_LogAddress((uint32_t*)&fl_out_ang_deg_);
    LOG::put_LogAddress((uint32_t*)&P_BLDC_->fl_calc_Iq_meas_);
    LOG::put_LogAddress((uint32_t*)&P_BLDC_->fl_calc_Id_meas_);
    LOG::put_LogAddress((uint32_t*)&P_BLDC_->fl_calc_Vq_);
    LOG::put_LogAddress((uint32_t*)&P_BLDC_->fl_calc_Vd_);
    LOG::enable_logging();
}

void BldcModeTestVdqStep::update()
{    
    BldcDriveMethod::Ref input = {};

    if(u16_test_cnt_ < U16_TEST_VOLT_STEP_START_CNT){
        u16_test_cnt_++;
    } else if(u16_test_cnt_ < U16_TEST_VOLT_END_COUNT) {
        input.Vq = fl_tgt_Vq_V_;
        input.Vd = fl_tgt_Vd_V_;

        u16_test_cnt_++;
    } else {
        LOG::disable_logging();
        input.Vq = 0.0f;
        input.Vd = 0.0f;
        is_comp_ = true;
    }

    P_BLDC_->update();
    parts_.p_bldc_drv->set(input);
    parts_.p_bldc_drv->update();

    fl_out_ang_deg_ = P_BLDC_->get_out_angle();
}

void BldcModeTestVdqStep::set_Instruction(Instr *p_instr){
  switch(p_instr->u16_instr_id) {
  case INSTR_ID_TEST_VOLT_STEP:
    fl_tgt_Vq_V_ = p_instr->InstrTestVoltOpen.fl_tgt_Vq_V;
    fl_tgt_Vd_V_ = p_instr->InstrTestVoltOpen.fl_tgt_Vd_V;
    break;
  default:
    break;
  }
}


void BldcModeTestPosStep::init()
{
    /* 内部変数初期化 */
    is_comp_ = false;
    u16_test_cnt_ = 0;

    /* Logging設定 */
    LOG::disable_logging();
    LOG::clear_LogData();
    LOG::clear_LogAddressArray();
    LOG::set_Mabiki_Num(u8_mabiki_);
    LOG::put_LogAddress((uint32_t*)&fl_tgt_forlog);
    LOG::put_LogAddress((uint32_t*)&fl_pos_forlog);
    LOG::put_LogAddress((uint32_t*)&P_BLDC_->fl_calc_Iq_tgt_);
    LOG::put_LogAddress((uint32_t*)&P_BLDC_->fl_calc_Iq_meas_);
    LOG::enable_logging();

    /* 位置制御モードの初期化 */
    parts_.p_mode_posctrl->init();
}


void BldcModeTestPosStep::update()
{
    if(u16_test_cnt_ < U16_TEST_CURR_STABLE_COUNT){
        u16_test_cnt_++;
    } else if(u16_test_cnt_ == U16_TEST_CURR_STABLE_COUNT){
        BldcModeBase::Instr instr = {
            .InstrPosCtrl_MvAng = {
                .u16_instr_id     = BldcModeBase::INSTR_ID_POSCTR_MOVE_ANGLE,
                .s32_tgt_pos      = s16_tgt_pos_deg_ << 16,
                .s32_move_time_ms = u16_move_time_ms_,
            },
        };
        parts_.p_mode_posctrl->set_Instruction(&instr);
        u16_test_cnt_++;
    } else if(u16_test_cnt_ < U16_TEST_CURR_END_COUNT) {
        u16_test_cnt_++;
    } else {
        LOG::disable_logging();
        is_comp_ = true;
    }
    fl_tgt_forlog = parts_.p_mode_posctrl->get_now_tgt();
    fl_pos_forlog = P_BLDC_->get_out_angle();

    /* 位置制御モードのルーチン */
    parts_.p_mode_posctrl->update();

}

void BldcModeTestPosStep::set_Instruction(Instr *p_instr){
  switch(p_instr->u16_instr_id) {
  case INSTR_ID_TEST_POS_STEP:
    s16_tgt_pos_deg_  = p_instr->InstrTestPosStep.s16_tgt_pos_deg;
    u16_move_time_ms_ = p_instr->InstrTestPosStep.u16_move_time_ms;
    u8_mabiki_        = p_instr->InstrTestPosStep.u8_mabiki;
    break;
  default:
    break;
  }
}

void BldcModeTestSineDriveOpen::update()
{    
    BldcDriveMethod::Ref input = {
        .Vq = fl_tgt_Vq_V_,
        .Vd = fl_tgt_Vd_V_,
    };

    P_BLDC_->update();
    parts_.p_bldc_drv->set(input);
    parts_.p_bldc_drv->update();

}

void BldcModeTestSineDriveOpen::set_Instruction(Instr *p_instr){
  switch(p_instr->u16_instr_id) {
  case INSTR_ID_TEST_SDRV_OPEN:
    fl_tgt_Vq_V_ = p_instr->InstrTestSDrvOpen.fl_tgt_Vq_V;
    fl_tgt_Vd_V_ = p_instr->InstrTestSDrvOpen.fl_tgt_Vd_V;
    break;
  default:
    break;
  }
}

void BldcModeTestSineDriveElecOpen::update()
{    
    BldcDriveMethod::Ref input = {
        .Vq = fl_tgt_Vq_V_,
        .Vd = fl_tgt_Vd_V_,
    };
    const float elec_vel_degps = 360.0f;
    const float nowtime = (float)u32_counter / 20000.0f;
    u32_counter++;
    if(u32_counter > (uint32_t)(20000*(elec_vel_degps/360.0f))) u32_counter = 0;

    P_BLDC_->update();
    parts_.p_bldc_drv->set(input);
    P_BLDC_->overwrite_elec_angle_forTEST(elec_vel_degps*nowtime);
    parts_.p_bldc_drv->update();

}

void BldcModeTestSineDriveElecOpen::set_Instruction(Instr *p_instr){
  switch(p_instr->u16_instr_id) {
  case INSTR_ID_TEST_SDRV_OPEN:
    fl_tgt_Vq_V_ = p_instr->InstrTestSDrvOpen.fl_tgt_Vq_V;
    fl_tgt_Vd_V_ = p_instr->InstrTestSDrvOpen.fl_tgt_Vd_V;
    break;
  default:
    break;
  }
}