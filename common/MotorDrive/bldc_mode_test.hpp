#ifndef BLDC_MODE_TEST_HPP_
#define BLDC_MODE_TEST_HPP_

#include "bldc_mode_base.hpp"
#include "bldc_drive_method.hpp"
#include "controller.hpp"
#include "iir.hpp"
#include "target_interp.hpp"
#include "flash_interface.hpp"

/**
 * @brief BLDC電気角測定テスト
 * 
 */
class BldcModeTestElecAngle : public BldcModeBase {
public:
    struct Parts{
        BldcDriveMethod* p_bldc_drv;
        FlashIF* p_flashif;
    };

    BldcModeTestElecAngle(Parts& _parts)
        : parts_(_parts) {};

    void init() override;
    void update() override;
    void end() override;

    bool isCompleted() override { return is_comp_; };

protected:
    Parts& parts_;

    bool is_comp_;
    int32_t s32_ElecOffsetSum;
    uint32_t u32_test_cnt_;
    const uint32_t U32_TEST_STABLE_COUNT  = 10000;
    const uint32_t U32_TEST_AVARAGE_COUNT_SHIFT = 8;

};


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
