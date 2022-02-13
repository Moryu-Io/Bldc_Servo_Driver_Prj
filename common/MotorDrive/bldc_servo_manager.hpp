#ifndef BLDC_SERVO_MANAGER_HPP_
#define BLDC_SERVO_MANAGER_HPP_

#include "bldc.hpp"
#include "bldc_drive_method.hpp"
#include "controller.hpp"
#include "iir.hpp"

class BldcServoManager {
public:
    struct ConfigParams{
        BLDC*            p_bldc;
        BldcDriveMethod* p_bldc_drv;
        controller*      p_pos_ctrl;
        IIR1*            p_posout_lpf;
    };

    BldcServoManager(ConfigParams &_config)
        : config_(_config) {};

    void update();

    void servo_enable(){  is_servo_enable = true;  };
    void servo_disable(){ is_servo_enable = false; };

protected:
    ConfigParams& config_;

private:
    bool is_servo_enable = false;


};


#endif
