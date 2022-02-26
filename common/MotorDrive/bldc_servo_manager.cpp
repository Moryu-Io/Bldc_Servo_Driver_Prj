#include "bldc_servo_manager.hpp"


BldcServoManager::BldcServoManager(BldcModeBase* _p_initMode)
    : p_nowMode_(_p_initMode), p_preMode_(_p_initMode), is_mode_lock_(false), u8_status_type()
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
            break;
        case 0x02:
            break;
        default:
            u32_status_memory[0] = 0;
            u32_status_memory[1] = 0;
            u32_status_memory[2] = 0;
            u32_status_memory[3] = 0;
            break;

    }
}
