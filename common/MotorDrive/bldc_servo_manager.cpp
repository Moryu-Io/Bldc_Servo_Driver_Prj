#include "bldc_servo_manager.hpp"


BldcServoManager::BldcServoManager(BldcModeBase* _p_initMode)
    : p_nowMode_(_p_initMode), p_preMode_(_p_initMode), is_mode_lock_(false), u8_status_type(1)
{

}


void BldcServoManager::set_mode(BldcModeBase* _p_mode){
    is_mode_lock_ = true;
    p_nowMode_->end();
    p_preMode_ = p_nowMode_;
    p_nowMode_ = _p_mode;
    p_nowMode_->init();
    is_mode_lock_ = false;
}

void BldcServoManager::set_mode_with_instr(BldcModeBase* _p_mode, BldcModeBase::Instr* _p_instr){
    is_mode_lock_ = true;
    p_nowMode_->end();
    p_preMode_ = p_nowMode_;
    p_nowMode_ = _p_mode;
    p_nowMode_->set_Instruction(_p_instr);
    p_nowMode_->init();
    is_mode_lock_ = false;
}

void BldcServoManager::update(){
    if(is_mode_lock_) return;

    if(is_updated_instr){
        p_nowMode_->set_Instruction(&un_mode_instr);
        is_updated_instr = false;
    }

    p_nowMode_->update();

    set_status_memory();

    if(p_nowMode_->isCompleted()){
        set_mode(p_preMode_);
    }

}


void BldcServoManager::set_instr_buf(BldcModeBase::Instr* _p_instr){
    un_mode_instr = *_p_instr;
    is_updated_instr = true;
}


void BldcServoManager::set_status_memory(){
    switch(u8_status_type){
        case 0x00:
            u32_status_memory[0] = CONV_U32TYPE(BldcModeBase::P_BLDC_->fl_calc_Iq_tgt_);
            u32_status_memory[1] = CONV_U32TYPE(BldcModeBase::P_BLDC_->fl_calc_Id_tgt_);
            u32_status_memory[2] = CONV_U32TYPE(BldcModeBase::P_BLDC_->fl_calc_Iq_meas_);
            u32_status_memory[3] = CONV_U32TYPE(BldcModeBase::P_BLDC_->fl_calc_Id_meas_);
            break;
        case 0x01:
            {
            BLDC::DrivePhase _cur = BldcModeBase::P_BLDC_->get_current();
            float _elec_angle = BldcModeBase::P_BLDC_->get_elec_angle();
            u32_status_memory[0] = CONV_U32TYPE(_cur.U);
            u32_status_memory[1] = CONV_U32TYPE(_cur.V);
            u32_status_memory[2] = CONV_U32TYPE(_cur.W);
            u32_status_memory[3] = CONV_U32TYPE(_elec_angle);
            }
            break;
        case 0x02:
            {
            float _elec_angle = BldcModeBase::P_BLDC_->get_elec_angle();
            float du = (float)TIM1->CCR1;
            float dv = (float)TIM1->CCR2;
            float dw = (float)TIM1->CCR3;
            u32_status_memory[0] = CONV_U32TYPE(du);
            u32_status_memory[1] = CONV_U32TYPE(dv);
            u32_status_memory[2] = CONV_U32TYPE(dw);
            u32_status_memory[3] = CONV_U32TYPE(_elec_angle);
            }
            break;
        default:
            u32_status_memory[0] = 0;
            u32_status_memory[1] = 0;
            u32_status_memory[2] = 0;
            u32_status_memory[3] = 0;
            break;

    }
}
