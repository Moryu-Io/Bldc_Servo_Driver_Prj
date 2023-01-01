#ifndef BLDC_SERVO_MANAGER_HPP_
#define BLDC_SERVO_MANAGER_HPP_

#include "bldc_mode_base.hpp"

/**
 * @brief BLDC制御モードのManager
 * 
 */
class BldcServoManager {
public:
    BldcServoManager(BldcModeBase* _p_initMode);

    void update();

    void set_mode(BldcModeBase* _p_mode);
    void set_mode_with_instr(BldcModeBase* _p_mode, BldcModeBase::Instr* _p_instr);
    void set_instr_buf(BldcModeBase::Instr* _p_instr);
    
    void set_status_type(uint8_t _type){ u8_status_type = _type; };

    uint32_t u32_status_memory[4];

private:
    void set_status_memory();

    BldcModeBase* p_nowMode_;
    BldcModeBase* p_preMode_;
    bool    is_mode_lock_;
    uint8_t u8_status_type;

    BldcModeBase::Instr un_mode_instr;
    bool  is_updated_instr;

};


#endif
