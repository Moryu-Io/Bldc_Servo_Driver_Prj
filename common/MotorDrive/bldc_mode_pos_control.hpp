#ifndef BLDC_MODE_POS_CONTROL_HPP_
#define BLDC_MODE_POS_CONTROL_HPP_

#include "bldc_mode_base.hpp"
#include "bldc_drive_method.hpp"
#include "controller.hpp"
#include "iir.hpp"
#include "target_interp.hpp"

/**
 * @brief BLDC位置制御クラス
 * 
 */
class BldcModePosControl : public BldcModeBase {
public:
    struct Parts{
        BldcDriveMethod* p_bldc_drv;
        controller*      p_pos_ctrl;
        IIR1*            p_posout_lpf;
        TargetInterp*    p_tgt_interp;
    };

    BldcModePosControl(Parts& _parts)
        : parts_(_parts) {};

    void init() override;
    void update() override;
    void end() override {};

protected:
    Parts& parts_;

};



#endif
