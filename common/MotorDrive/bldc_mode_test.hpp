#ifndef BLDC_MODE_TEST_HPP_
#define BLDC_MODE_TEST_HPP_

#include "bldc_mode_base.hpp"
#include "bldc_drive_method.hpp"
#include "controller.hpp"
#include "iir.hpp"
#include "target_interp.hpp"

/**
 * @brief BLDC電流制御ステップ応答テスト
 * 
 */
class BldcModeTestCurrStep : public BldcModeBase {
public:
    struct Parts{
        BldcDriveMethod* p_bldc_drv;
        float fl_tgt_Iq_A;
        float fl_tgt_Id_A;
    };

    BldcModeTestCurrStep(Parts& _parts)
        : parts_(_parts) {};

    void init() override;
    void update() override;
    void end() override {};

    bool isCompleted() override { return is_comp_; };

protected:
    Parts& parts_;

    bool is_comp_;
    uint16_t u16_test_cnt_;
    const uint16_t U16_TEST_CURR_STABLE_COUNT = 200;
    const uint16_t U16_TEST_CURR_END_COUNT    = 1000;


};



#endif
