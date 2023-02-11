#ifndef BLDC_MODE_TRQ_CONTROL_HPP_
#define BLDC_MODE_TRQ_CONTROL_HPP_

#include "bldc_mode_base.hpp"
#include "bldc_drive_method.hpp"
#include "controller.hpp"
#include "iir.hpp"
#include "target_interp.hpp"

/**
 * @brief BLDC位置制御クラス
 * 
 */
class BldcModeTrqControl : public BldcModeBase {
public:
    struct Parts{
        BldcDriveMethod* p_bldc_drv;
    };

    BldcModeTrqControl(Parts& _parts)
        : parts_(_parts) {};

    void init() override;
    void update() override;
    void end() override {};

    virtual void set_Instruction(Instr *p_instr);
    virtual void get_status_1(Status *p_mem);

protected:
    Parts& parts_;

    float fl_tgt_Iq_A_;
    float fl_tgt_Id_A_;

};



#endif
